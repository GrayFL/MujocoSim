"""Real SimulationDevice management rejection must not retire its transport lease."""
import asyncio
import json
import os
import time
import uuid

import pytest

from rsim import Runtime
from rsim.core import CommandEnvelope, CommandRejected, VelocityCommand
from rsim.transport.descriptor import TransportConfig
from mujocosim.rsim import MujocoChassisFactory, SimulationDevice
from test_simulation import make_room


def test_stale_advance_after_reset_does_not_retire_simulation_device(tmp_path):
    make_room(tmp_path)
    async def check():
        key = 'managed-reset:' + uuid.uuid4().hex
        transport = TransportConfig('cyclonedds', 64)
        factory = MujocoChassisFactory(scene_dir=tmp_path, scheduling='controlled', motion_enabled=True)
        shared = factory.shared(key=key, transport=transport, log_path=tmp_path/'provider.log')
        view = SimulationDevice(key=key, transport=transport)
        async with Runtime(shared.state, view.state, view.velocity, view.advance_request,
                           view.reset_request, view.inspect_request):
            await view.state.get(timeout=5)
            assert view.manifest['pid'] == shared.manifest['pid'] != os.getpid()
            await asyncio.sleep(.15)  # DDS discovery happens before starting command TTLs.
            first = CommandEnvelope(VelocityCommand(.1), 'motion', 'before-reset', 1,
                                    time.monotonic_ns()+400_000_000)
            accepted = await view.velocity.set(first)
            old_request = (accepted['context'], accepted['command_id'], 2)
            previous = await view.advance(*old_request)
            assert previous['physical_substeps'] == 2
            reset = await view.reset()
            assert reset['episode'] == 1
            with pytest.raises(CommandRejected) as stale:
                await view.advance(*old_request)
            current = await view.synchronize()
            assert current['phase'] == 'READY' and current['physical_substeps'] == 0
            # Let the forwarding motion guard observe the provider's reset;
            # its pre-reset lease must be retired, not transferred implicitly.
            with pytest.raises(CommandRejected) as retired:
                await view.velocity.set(CommandEnvelope(VelocityCommand(.1), 'motion', 'before-reset', 2,
                                                        time.monotonic_ns()+100_000_000))
            assert retired.value.reason == 'retired'
            # Motion must start a new controller epoch after reset. The same
            # management connection must remain usable for the new source step.
            fresh = CommandEnvelope(VelocityCommand(.1), 'motion', 'after-reset', 1,
                                    time.monotonic_ns()+400_000_000)
            accepted = await view.velocity.set(fresh)
            with pytest.raises(CommandRejected) as unknown:
                await view.advance(dict(accepted['context'], step_index=99), accepted['command_id'], 3)
            assert unknown.value.reason == 'stale_context'
            advanced = await view.advance(accepted['context'], accepted['command_id'], 3)
            assert advanced['kind'] == 'PhysicsAdvanced'
            assert advanced['context']['episode'] == 1
            assert advanced['physical_substeps'] == 3 and advanced['stamp_ns'] == 6_000_000
            assert stale.value.reason == 'stale_context'
            (tmp_path/'management.json').write_text(json.dumps(dict(
                worker_pid=view.manifest['pid'], client_pid=os.getpid(), reset=reset,
                stale_reason=stale.value.reason, unknown_reason=unknown.value.reason,
                motion_retired_reason=retired.value.reason, accepted=accepted, advanced=advanced), indent=2)+'\n')
    asyncio.run(check())


def test_rejected_remote_resume_keeps_provider_available_for_reset(tmp_path):
    make_room(tmp_path)
    async def check():
        key = 'managed-resume:' + uuid.uuid4().hex
        transport = TransportConfig('cyclonedds', 64)
        factory = MujocoChassisFactory(scene_dir=tmp_path, scheduling='controlled', motion_enabled=True)
        shared = factory.shared(key=key, transport=transport, log_path=tmp_path/'provider.log')
        view = SimulationDevice(key=key, transport=transport)
        async with Runtime(shared.state, view.state, view.velocity, view.advance_request,
                           view.reset_request, view.inspect_request, view.control_request):
            await view.state.get(timeout=5)
            pid = view.manifest['pid']
            assert pid == shared.manifest['pid'] != os.getpid()
            await asyncio.sleep(.15)
            aborted = await view.abort('test needs explicit reset')
            assert aborted['phase'] == 'ABORTED'
            with pytest.raises(CommandRejected, match='successful reset') as refusal:
                await view.resume()
            # The watchdog must not reinterpret a rejected state transition as
            # the management service failing and shut down the physical owner.
            await asyncio.sleep(.07)
            status = await asyncio.wait_for(view.describe(), 2)
            assert status['phase'] == 'ABORTED' and status['paused']
            reset = await asyncio.wait_for(view.reset(), 2)
            resumed = await asyncio.wait_for(view.resume(), 2)
            assert resumed['phase'] == 'READY' and not resumed['paused']
            assert resumed['context']['episode'] == reset['episode']+1
            accepted = await view.velocity.set(VelocityCommand(.1), ttl=.4)
            advanced = await view.advance(accepted['context'], accepted['command_id'], 2)
            assert advanced['physical_substeps'] == 2 and advanced['stamp_ns'] == 4_000_000
            assert view.manifest['pid'] == pid
            (tmp_path/'resume.json').write_text(json.dumps(dict(
                client_pid=os.getpid(), worker_pid=pid, refusal_reason=refusal.value.reason,
                status_after_refusal=status, reset=reset, resumed=resumed, advanced=advanced), indent=2)+'\n')
    asyncio.run(check())


def test_resume_boundary_preserves_local_contract_and_unexpected_owner_errors(tmp_path, monkeypatch):
    make_room(tmp_path)
    async def check():
        world = MujocoChassisFactory(scene_dir=tmp_path, scheduling='controlled').build()
        async with Runtime(world.state):
            await world.abort()
            with pytest.raises(RuntimeError, match='successful reset'):
                await world.resume()
            async def fail_owner(*args, **kwargs):
                raise RuntimeError('unexpected owner failure')
            monkeypatch.setattr(world, '_control', fail_owner)
            request = CommandEnvelope({'operation': 'resume'}, 'management', 'local', 1,
                                      time.monotonic_ns()+100_000_000)
            with pytest.raises(RuntimeError, match='unexpected owner failure'):
                await world._control_request(request)
    asyncio.run(check())
