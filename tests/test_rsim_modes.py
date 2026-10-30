"""Real-engine fixed-step modes and queryable owner transactions (v4.1 Phase 2)."""
import asyncio
from dataclasses import asdict, replace
import threading
import time

import numpy as np
import pytest

from rsim import Runtime
from rsim.core import CommandEnvelope, CommandRejected, SourceCommandBound, VelocityCommand
from mujocosim.config import SensorConfig
from mujocosim.rsim import MujocoChassisFactory
from test_simulation import make_room


def envelope(world, *, sequence=1, duration=.4, source_end_ns=None, epoch='run'):
    bound = (None if source_end_ns is None else
             SourceCommandBound(world.context, world.clock, source_end_ns))
    return CommandEnvelope(VelocityCommand(.2), 'controller', epoch, sequence,
                           time.monotonic_ns()+int(duration*1e9), bound)


def identity(command):
    return {name: getattr(command, name) for name in ('controller_id', 'controller_epoch', 'sequence')}


def test_controlled_real_engine_stages_integrates_once_and_caches_after_expiry(tmp_path):
    make_room(tmp_path)
    async def check():
        world = MujocoChassisFactory(scene_dir=tmp_path, scheduling='controlled', motion_enabled=True,
                                    sensors=SensorConfig(width=80, height=60),
                                    camera_hz=50., lidar_hz=50., ack_capacity=1).build()
        async with Runtime(world):
            assert world._physics_task is None
            initial = await world.state.get()
            before_image = await world.image.get()
            image_copy = before_image.data.pixels.copy()
            await asyncio.sleep(.025)
            assert (await world.state.get()).stamp_ns == initial.stamp_ns == 0
            command = envelope(world, duration=.1, source_end_ns=100_000_000)
            accepted = await world.velocity.set(command)
            assert accepted['kind'] == 'CommandAccepted'
            assert accepted['context'] == asdict(world.context)
            assert accepted['command_id'] == identity(command)
            assert (await world.state.get()).stamp_ns == 0
            request_context = dict(accepted['context'])
            result, concurrent_retry = await asyncio.gather(
                world.advance(request_context, identity(command), 20),
                world.advance(request_context, identity(command), 20))
            assert result == concurrent_retry
            assert result['kind'] == 'PhysicsAdvanced' and result['phase'] == 'READY'
            assert result['actual_substeps'] == result['requested_substeps'] == result['physical_substeps'] == 20
            assert result['request_context'] == request_context
            assert result['context']['step_index'] == 1
            assert result['stamp_ns'] == 40_000_000
            assert (await world.pose.get()).data.position[0] > 0
            frames = [await port.get() for port in (world.image, world.depth, world.scan, world.imu, world.state)]
            assert {frame.stamp_ns for frame in frames} == {40_000_000}
            assert {frame.metadata['step_index'] for frame in frames} == {1}
            np.testing.assert_array_equal(before_image.data.pixels, image_copy)
            assert not before_image.data.pixels.flags.writeable
            await asyncio.sleep(.11)  # Original command's host TTL is now expired.
            assert await world.advance(request_context, identity(command), 20) == result
            with pytest.raises(CommandRejected, match='different contents'):
                await world.advance(request_context, identity(command), 21)
            command2 = envelope(world, sequence=2)
            accepted2 = await world.velocity.set(command2)
            result2 = await world.advance(accepted2['context'], identity(command2), 5)
            assert result2['physical_substeps'] == 25 and result2['stamp_ns'] == 50_000_000
            assert (await world.describe())['ack_cache_size'] == 1
            with pytest.raises(CommandRejected, match='no longer cached'):
                await world.advance(request_context, identity(command), 20)
            await world.reset()
            status = await world.describe()
            assert status['phase'] == 'READY' and status['physical_substeps'] == status['stamp_ns'] == 0
            assert status['context']['episode'] == 1 and status['last_result'] is None
            with pytest.raises(CommandRejected, match='stale'):
                await world.advance(result2['request_context'], identity(command2), 5)
    asyncio.run(check())


def test_controlled_source_bound_partial_progress_aborts_and_reset_recovers(tmp_path):
    make_room(tmp_path)
    async def check():
        world = MujocoChassisFactory(scene_dir=tmp_path, scheduling='controlled', motion_enabled=True).build()
        async with Runtime(world.state, world.velocity):
            command = envelope(world, source_end_ns=6_000_000)
            accepted = await world.velocity.set(command)
            with pytest.raises(CommandRejected, match='interval expired'):
                await world.advance(accepted['context'], identity(command), 5)
            status = await world.synchronize()
            assert status['phase'] == 'ABORTED' and status['state_known']
            assert status['physical_substeps'] == 3 and status['stamp_ns'] == 6_000_000
            assert status['context']['step_index'] == 0
            assert status['last_result']['kind'] == 'PhysicsAborted'
            assert status['last_result']['actual_substeps'] == 3
            with pytest.raises(CommandRejected):
                await world.advance(accepted['context'], identity(command), 5)
            assert (await world.describe())['physical_substeps'] == 3
            with pytest.raises(RuntimeError, match='successful reset'):
                await world.resume()
            await world.reset()
            with pytest.raises(CommandRejected, match='retired'):
                await world.velocity.set(replace(command, sequence=2, deadline_ns=time.monotonic_ns()+100_000_000))
            current = envelope(world, epoch='new')
            accepted = await world.velocity.set(current)
            result = await world.advance(accepted['context'], identity(current), 1)
            assert result['physical_substeps'] == 1 and result['context']['episode'] == 1
    asyncio.run(check())


def test_controlled_host_deadline_checked_before_each_real_substep(tmp_path, monkeypatch):
    import mujoco
    make_room(tmp_path)
    original = mujoco.mj_step
    async def check():
        world = MujocoChassisFactory(scene_dir=tmp_path, scheduling='controlled', motion_enabled=True).build()
        async with Runtime(world.state, world.velocity):
            calls = []
            def slow_step(model, data):
                calls.append(time.monotonic_ns())
                time.sleep(.008)
                return original(model, data)
            monkeypatch.setattr(mujoco, 'mj_step', slow_step)
            command = envelope(world, duration=.025)
            accepted = await world.velocity.set(command)
            with pytest.raises(CommandRejected):
                await world.advance(accepted['context'], identity(command), 10)
            status = await world.describe()
            assert status['phase'] == 'ABORTED'
            assert 0 < status['physical_substeps'] < 10
            assert len(calls) == status['physical_substeps']
            assert all(stamp < command.deadline_ns for stamp in calls), [stamp-command.deadline_ns for stamp in calls]
            assert status['stamp_ns'] == status['physical_substeps']*2_000_000
    asyncio.run(check())


def test_cancelled_controlled_advance_finishes_commit_and_owner_cleanup(tmp_path, monkeypatch):
    import mujoco
    make_room(tmp_path)
    original = mujoco.mj_step
    entered, release = threading.Event(), threading.Event()
    thread_ids = set()
    async def check():
        world = MujocoChassisFactory(scene_dir=tmp_path, scheduling='controlled', motion_enabled=True).build()
        async with Runtime(world.state, world.velocity):
            def blocking_step(model, data):
                thread_ids.add(threading.get_ident())
                entered.set()
                assert release.wait(1)
                return original(model, data)
            monkeypatch.setattr(mujoco, 'mj_step', blocking_step)
            command = envelope(world)
            accepted = await world.velocity.set(command)
            task = asyncio.create_task(world.advance(accepted['context'], identity(command), 5))
            async with asyncio.timeout(.5):
                while not entered.is_set():
                    await asyncio.sleep(.001)
            task.cancel()
            await asyncio.sleep(0)
            task.cancel()
            await asyncio.sleep(0)
            assert not task.done()
            release.set()
            with pytest.raises(asyncio.CancelledError):
                await task
            status = await world.synchronize()
            assert status['phase'] == 'READY' and status['physical_substeps'] == 5
            assert status['last_result']['kind'] == 'PhysicsAdvanced'
            assert await world.advance(accepted['context'], identity(command), 5) == status['last_result']
            assert len(thread_ids) == 1 and threading.get_ident() not in thread_ids
        assert not world._transactions and world._executor is None
    asyncio.run(check())


@pytest.mark.parametrize('fault', ['step_and_stop', 'unreadable_time', 'sample'])
def test_controlled_combined_native_failures_are_queryable_until_reset(tmp_path, monkeypatch, fault):
    make_room(tmp_path)
    async def check():
        world = MujocoChassisFactory(scene_dir=tmp_path, scheduling='controlled', motion_enabled=True).build()
        async with Runtime(world.state, world.velocity):
            backend = world.backend
            with monkeypatch.context() as patch:
                if fault == 'step_and_stop':
                    original = backend.step_guarded
                    def partial(duration, provider):
                        original(.004, provider)
                        raise RuntimeError('step failed after integration')
                    def stop_failed():
                        raise RuntimeError('stop failed')
                    patch.setattr(backend, 'step_guarded', partial)
                    patch.setattr(backend, 'stop', stop_failed)
                elif fault == 'unreadable_time':
                    def unreadable(_):
                        raise RuntimeError('native timestamp unavailable')
                    patch.setattr(type(backend), 'stamp_ns', property(unreadable))
                else:
                    original = backend.sample
                    def sample_failed(demand, force=False):
                        if backend.stamp_ns:
                            raise RuntimeError('sensor snapshot failed')
                        return original(demand, force)
                    patch.setattr(backend, 'sample', sample_failed)
                command = envelope(world)
                accepted = await world.velocity.set(command)
                with pytest.raises(CommandRejected):
                    await world.advance(accepted['context'], identity(command), 5)
                status = await world.describe()
                assert status['phase'] == 'ABORTED'
                assert status['last_result']['kind'] == 'PhysicsAborted'
                if fault == 'unreadable_time':
                    assert not status['state_known']
                    assert status['stamp_ns'] is status['physical_substeps'] is None
                    assert status['last_result']['actual_substeps'] is None
                else:
                    expected = 2 if fault == 'step_and_stop' else 5
                    assert status['state_known']
                    assert status['physical_substeps'] == expected
                    assert status['stamp_ns'] == expected*2_000_000
                    if fault == 'step_and_stop':
                        assert 'stop failed' in status['abort_reason']
                with pytest.raises(CommandRejected):
                    await world.advance(accepted['context'], identity(command), 5)
            await world.reset()
            assert (await world.describe())['phase'] == 'READY'
            assert (await world.describe())['state_known']
    asyncio.run(check())


@pytest.mark.parametrize('rate', [1., 2., 10.])
def test_accelerated_fixed_dt_pause_resume_rate_and_tick_backpressure(tmp_path, rate):
    make_room(tmp_path)
    async def check():
        world = MujocoChassisFactory(scene_dir=tmp_path, scheduling='accelerated', speedup=rate,
                                    motion_enabled=True).build()
        async with Runtime(world.state, world.velocity):
            ticks = world.subscribe(policy='every', capacity=1)
            first = await asyncio.wait_for(ticks.tick(), 1)
            second = await asyncio.wait_for(ticks.tick(), 1)
            assert second.stamp_ns-first.stamp_ns == 20_000_000
            assert second.step_index == first.step_index+1
            state = await world.state.get()
            assert state.metadata['source_rate'] == rate
            command = envelope(world)
            await world.velocity.set(command)
            await asyncio.sleep(.06)  # Fill the finite event queue and stop at its backpressure.
            before = await world.describe()
            await asyncio.sleep(.025)
            assert (await world.describe())['stamp_ns'] == before['stamp_ns']
            await asyncio.wait_for(world.pause(), 1)
            paused = await world.describe()
            assert paused['paused'] and world.command == VelocityCommand()
            assert world._physics_task is None
            await asyncio.sleep(.025)
            assert (await world.describe())['stamp_ns'] == paused['stamp_ns']
            rate_update = await world.set_rate(rate*2)
            assert rate_update['context'] == paused['context']
            resumed = await world.resume()
            assert resumed['context']['episode'] == paused['context']['episode']+1
            assert resumed['context']['step_index'] == paused['context']['step_index']
            assert resumed['stamp_ns'] == paused['stamp_ns']
            boundary = await asyncio.wait_for(ticks.tick(), 1)
            assert boundary.episode == resumed['context']['episode']
            assert boundary.stamp_ns == paused['stamp_ns']
            assert all(frame.metadata['episode'] == boundary.episode for frame in world.state.frames)
            with pytest.raises(CommandRejected, match='retired'):
                await world.velocity.set(replace(command, sequence=2, deadline_ns=time.monotonic_ns()+100_000_000))
            changed = await world.set_rate(rate)
            assert changed['context']['episode'] == boundary.episode+1
            assert changed['source_rate'] == rate
            assert await world.inspect(lambda sim: sim.physics.timestep) == .002
            await ticks.aclose()
            await world.pause()
    asyncio.run(check())


@pytest.mark.parametrize('scheduling', ['realtime', 'accelerated'])
def test_continuous_modes_apply_original_source_bound_without_reanchoring(tmp_path, scheduling):
    make_room(tmp_path)
    async def check():
        world = MujocoChassisFactory(scene_dir=tmp_path, scheduling=scheduling,
                                    motion_enabled=True, speedup=2.).build()
        async with Runtime(world.state, world.velocity):
            await world.pause()
            await world.resume()
            current = await world.describe()
            command = envelope(world, source_end_ns=current['stamp_ns']+6_000_000)
            accepted = await world.velocity.set(command)
            assert accepted['source_deadline_ns'] == command.source_bound.deadline_ns
            await asyncio.sleep(.04)
            state = await world.state.get()
            assert state.stamp_ns > command.source_bound.deadline_ns
            assert world.command == VelocityCommand()
            assert state.data['wheel_targets'] == [0., 0.]
            assert (await world.describe())['command_expirations'] == 1
            await world.pause()
    asyncio.run(check())


def test_controlled_rejects_unstaged_identity_and_expired_first_request(tmp_path):
    make_room(tmp_path)
    async def check():
        world = MujocoChassisFactory(scene_dir=tmp_path, scheduling='controlled', motion_enabled=True).build()
        async with Runtime(world.state, world.velocity):
            command = envelope(world)
            accepted = await world.velocity.set(command)
            wrong = dict(identity(command), sequence=2)
            with pytest.raises(CommandRejected):
                await world.advance(accepted['context'], wrong, 2)
            assert (await world.describe())['phase'] == 'ABORTED'
            assert (await world.describe())['physical_substeps'] == 0
            await world.reset()
            command = envelope(world, epoch='fresh')
            accepted = await world.velocity.set(command)
            with pytest.raises(CommandRejected, match='expired'):
                await world.advance(accepted['context'], identity(command), 2, _deadline_ns=time.monotonic_ns()-1)
            assert (await world.describe())['phase'] == 'ABORTED'
            assert (await world.describe())['physical_substeps'] == 0
    asyncio.run(check())


def test_controlled_pause_cancels_only_event_backpressure_not_committed_ack(tmp_path):
    make_room(tmp_path)
    async def check():
        world = MujocoChassisFactory(scene_dir=tmp_path, scheduling='controlled', motion_enabled=True).build()
        async with Runtime(world.state, world.velocity):
            ticks = world.subscribe(policy='every', capacity=1)
            first = envelope(world)
            accepted = await world.velocity.set(first)
            await world.advance(accepted['context'], identity(first), 1)
            second = envelope(world, sequence=2)
            accepted = await world.velocity.set(second)
            advance = asyncio.create_task(world.advance(accepted['context'], identity(second), 1))
            async with asyncio.timeout(1):
                while (await world.describe())['physical_substeps'] < 2:
                    await asyncio.sleep(.001)
            assert not advance.done()  # Committed physics waits for bounded event delivery.
            await asyncio.wait_for(world.pause(), 1)
            result = await asyncio.wait_for(advance, 1)
            assert result['kind'] == 'PhysicsAdvanced' and result['physical_substeps'] == 2
            assert (await world.describe())['last_result'] == result
            await ticks.aclose()
    asyncio.run(check())
