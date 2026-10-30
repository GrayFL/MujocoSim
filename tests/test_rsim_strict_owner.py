"""Real owner boundaries consumed by the strict single-process ACK protocol."""
import asyncio
from dataclasses import replace
import threading
import time

import pytest

from rsim import Runtime
from rsim.core import CommandEnvelope, CommandRejected, SourceCommandBound, VelocityCommand
from rsim.core.lockstep import CommandAccepted, PhysicsAdvanced, SingleProcessStepper
from mujocosim.rsim import MujocoChassisFactory
from test_simulation import make_room


def strict_stepper(world, *, substeps, advance=None, source_end_ns=None):
    async def sample(context):
        assert context == world.context
        return await world.state.get()
    async def compute(context, state):
        return VelocityCommand(.2)
    async def accept(context, command):
        if source_end_ns is None:
            return await world.velocity.set(command, ttl=.4)
        envelope = CommandEnvelope(command, 'strict', 'strict-epoch', context.step_index+1,
                                   time.monotonic_ns()+400_000_000,
                                   SourceCommandBound(context, world.clock, source_end_ns))
        return await world.velocity.set(envelope)
    async def run_advance(context, accepted):
        return await world.advance(context, accepted.command_id.to_dict(), substeps)
    return SingleProcessStepper(sample=sample, compute=compute, accept=accept,
        advance=advance or run_advance, stop=world.abort, timeout=1., context=world.context,
        require_physics_ack=True, requested_substeps=substeps)


def test_strict_stepper_accepts_real_unbounded_controlled_source_time(tmp_path):
    """A short host lease does not invent a matching simulation-time lease."""
    make_room(tmp_path)
    async def check():
        world = MujocoChassisFactory(scene_dir=tmp_path, scheduling='controlled', motion_enabled=True).build()
        async with Runtime(world.state, world.velocity):
            async def sample(context):
                assert context == world.context
                return await world.state.get()
            async def compute(context, state):
                return VelocityCommand(.2)
            async def accept(context, command):
                return await world.velocity.set(command, ttl=.2)
            async def advance(context, accepted):
                return await world.advance(context, accepted.command_id.to_dict(), 150)
            stepper = SingleProcessStepper(sample=sample, compute=compute, accept=accept,
                advance=advance, stop=world.abort, timeout=1., context=world.context,
                require_physics_ack=True, requested_substeps=150)
            await stepper.step()
            assert stepper.last_command_ack.source_deadline_ns is None
            assert stepper.last_physics_ack.stamp_ns == 300_000_000
            assert stepper.last_physics_ack.physical_substeps == 150
            assert stepper.context == world.context
            assert not stepper.needs_synchronization and not stepper.aborted
    asyncio.run(check())


def test_command_queued_before_reset_cannot_stage_in_the_new_episode(tmp_path):
    make_room(tmp_path)
    entered, release = threading.Event(), threading.Event()
    async def check():
        world = MujocoChassisFactory(scene_dir=tmp_path, scheduling='controlled', motion_enabled=True).build()
        async with Runtime(world.state, world.velocity):
            command = CommandEnvelope(VelocityCommand(.2), 'old', 'old-epoch', 1,
                                      time.monotonic_ns()+400_000_000)
            await world.velocity.set(command)
            def hold_owner(sim):
                entered.set()
                assert release.wait(1)
            inspection = asyncio.create_task(world.inspect(hold_owner))
            async with asyncio.timeout(.5):
                while not entered.is_set():
                    await asyncio.sleep(.001)
            reset_queued = asyncio.Event()
            original_reset = world._reset_locked
            async def resetting(*args, **kwargs):
                reset_queued.set()
                return await original_reset(*args, **kwargs)
            world._reset_locked = resetting
            reset = asyncio.create_task(world.reset())
            await reset_queued.wait()
            pending = asyncio.create_task(world.velocity.set(replace(command, sequence=2)))
            try:
                async with asyncio.timeout(.5):
                    while world.velocity.guard.sequences.get(('old', 'old-epoch')) != 2:
                        await asyncio.sleep(0)
                assert not pending.done()
            finally:
                release.set()
            await inspection
            await reset
            with pytest.raises(CommandRejected, match='retired'):
                await pending
            assert world.command == VelocityCommand()
            assert world.context.episode == 1
            assert (await world.describe())['phase'] == 'READY'
            with pytest.raises(CommandRejected, match='retired'):
                await world.velocity.set(replace(command, sequence=3, deadline_ns=time.monotonic_ns()+100_000_000))
    asyncio.run(check())


def test_strict_lost_ack_queries_committed_physics_and_reset_retires_it(tmp_path):
    make_room(tmp_path)
    async def check():
        world = MujocoChassisFactory(scene_dir=tmp_path, scheduling='controlled', motion_enabled=True).build()
        async with Runtime(world.state, world.velocity):
            async def lose_ack(context, accepted):
                await world.advance(context, accepted.command_id.to_dict(), 5)
                raise TimeoutError('reply was lost after owner commit')
            stepper = strict_stepper(world, substeps=5, advance=lose_ack)
            with pytest.raises(TimeoutError, match='reply was lost'):
                await stepper.step()
            assert stepper.aborted and stepper.needs_synchronization
            assert stepper.last_physics_ack is None
            assert isinstance(stepper.last_command_ack, CommandAccepted)
            status = await world.synchronize()
            assert status['phase'] == 'ABORTED'  # Stepper's explicit stop.
            result = PhysicsAdvanced.from_mapping(status['last_result'])
            assert result.actual_substeps == result.physical_substeps == 5
            assert result.stamp_ns == 10_000_000
            assert result.context == world.context
            retry = await world.advance(result.request_context, result.command_id.to_dict(), 5)
            assert PhysicsAdvanced.from_mapping(retry) == result
            assert (await world.describe())['physical_substeps'] == 5
            async def reset_source():
                await world.reset()
                await world.resume()
                return world.context
            await stepper.reset(reset_source)
            assert stepper.context == world.context and stepper.context.step_index == 0
            assert not stepper.aborted and not stepper.needs_synchronization
            with pytest.raises(CommandRejected, match='stale'):
                await world.advance(result.request_context, result.command_id.to_dict(), 5)
            assert (await world.describe())['physical_substeps'] == 0
    asyncio.run(check())


def test_strict_partial_source_bound_never_constructs_a_success_ack(tmp_path):
    make_room(tmp_path)
    async def check():
        world = MujocoChassisFactory(scene_dir=tmp_path, scheduling='controlled', motion_enabled=True).build()
        async with Runtime(world.state, world.velocity):
            stepper = strict_stepper(world, substeps=5, source_end_ns=6_000_000)
            with pytest.raises(CommandRejected, match='interval expired'):
                await stepper.step()
            assert stepper.last_command_ack.source_deadline_ns == 6_000_000
            assert stepper.last_physics_ack is None and stepper.needs_synchronization
            status = await world.synchronize()
            assert status['phase'] == 'ABORTED'
            assert status['stamp_ns'] == 6_000_000 and status['physical_substeps'] == 3
            assert status['last_result']['actual_substeps'] == 3
            with pytest.raises(ValueError, match='expected PhysicsAdvanced'):
                PhysicsAdvanced.from_mapping(status['last_result'])
    asyncio.run(check())


def test_strict_cancelled_real_owner_finishes_then_requires_synchronization(tmp_path, monkeypatch):
    import mujoco
    make_room(tmp_path)
    entered, release = threading.Event(), threading.Event()
    original = mujoco.mj_step
    async def check():
        world = MujocoChassisFactory(scene_dir=tmp_path, scheduling='controlled', motion_enabled=True).build()
        async with Runtime(world.state, world.velocity):
            def blocking_step(model, data):
                entered.set()
                assert release.wait(1)
                return original(model, data)
            monkeypatch.setattr(mujoco, 'mj_step', blocking_step)
            stepper = strict_stepper(world, substeps=5)
            task = asyncio.create_task(stepper.step())
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
            assert stepper.aborted and stepper.needs_synchronization
            assert stepper.last_physics_ack is None
            status = await world.synchronize()
            actual = PhysicsAdvanced.from_mapping(status['last_result'])
            assert actual.physical_substeps == 5 and actual.stamp_ns == 10_000_000
            assert status['phase'] == 'ABORTED' and status['context'] == actual.context.to_dict()
    asyncio.run(check())
