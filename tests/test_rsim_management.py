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
