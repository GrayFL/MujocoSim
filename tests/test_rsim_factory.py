"""Factory, native episode, resource and command safety acceptance (F/W/SAFE)."""
import asyncio
from dataclasses import FrozenInstanceError, replace
import json
import os
import subprocess
import sys
import threading
import time
import uuid

import numpy as np
import pytest

from rsim import Runtime
from rsim.core import CommandEnvelope, CommandRejected, ComponentError, VelocityCommand
from rsim.runtime.sharing import component_ports
from mujocosim.config import PhysicsConfig, SensorConfig
from mujocosim.rsim import MujocoChassisFactory, SimulationDevice
from mujocosim.rsim.components import MuJoCoSource, Observation
from test_simulation import make_room


def test_client_imports_never_load_engine_or_app_dependencies():
    code = '''
import importlib.abc, sys
class Blocked(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('mujoco', 'trimesh', 'pyrobo', 'matplotlib'):
            raise AssertionError('eager import: ' + fullname)
sys.meta_path.insert(0, Blocked())
from mujocosim.rsim import MujocoChassisFactory, SimulationDevice
device = SimulationDevice(key='existing-world')
assert device.connection is None
assert len(MujocoChassisFactory.port_specs()) == 13
'''
    subprocess.run([sys.executable, '-c', code], cwd='/tmp', check=True)


def test_factory_is_immutable_lazy_and_owns_the_only_declaration(tmp_path, monkeypatch):
    make_room(tmp_path)
    from mujocosim.rsim.components import source
    def fail(*args, **kwargs):
        raise AssertionError('construction created an executor')
    monkeypatch.setattr(source, 'ThreadPoolExecutor', fail)
    spawn = [0., 0., 0.]
    factory = MujocoChassisFactory(scene_dir=tmp_path, spawn=spawn, clock='test:scene')
    spawn[0] = 8
    assert factory.spawn == (0., 0., 0.)
    with pytest.raises(FrozenInstanceError):
        factory.hz = 20
    world = factory.build()
    assert world.backend is world._executor is None
    assert not hasattr(world, 'simulation')
    assert component_ports(world) == factory.specs()
    shared = factory.shared(key='lazy-world')
    device = factory.device(key='lazy-world')
    assert shared.port_specs == device.port_specs == factory.specs()
    assert shared.factory.func.__self__ is factory
    assert shared.factory.keywords['expected_fingerprint'] == shared.provider_version
    assert shared.connection is device.connection is None
    specs = factory.specs()
    specs.pop('pose')
    assert 'pose' in factory.specs()


def test_fingerprint_is_stable_sensitive_and_content_based(tmp_path):
    make_room(tmp_path)
    factory = MujocoChassisFactory(scene_dir=tmp_path)
    fingerprint = factory.fingerprint()
    script = ('from mujocosim.rsim import MujocoChassisFactory; '
              f'print(MujocoChassisFactory(scene_dir={str(tmp_path)!r}).fingerprint())')
    remote = subprocess.check_output([sys.executable, '-c', script], cwd='/tmp', text=True).strip()
    assert remote == fingerprint
    assert replace(factory, motion_enabled=True).fingerprint() != fingerprint
    assert replace(factory, max_ttl=.25).fingerprint() != fingerprint
    assert replace(factory, physics=PhysicsConfig(timestep=.001)).fingerprint() != fingerprint
    assert replace(factory, robot=replace(factory.robot, config=replace(factory.robot.config, wheel_track=.6))).fingerprint() != fingerprint
    (tmp_path/'provider.log').write_text('logging must not change device configuration')
    assert factory.fingerprint() == fingerprint
    path = tmp_path/'scene.xml'
    path.write_text(path.read_text() + '\n')
    assert factory.fingerprint() != fingerprint


def test_local_open_rechecks_scene_identity_before_engine_creation(tmp_path):
    make_room(tmp_path)
    factory = MujocoChassisFactory(scene_dir=tmp_path)
    world = factory.build()
    path = tmp_path/'scene.xml'
    path.write_text(path.read_text() + '\n')
    async def check():
        with pytest.raises(ValueError, match='scene resources changed'):
            async with Runtime(world.state):
                pytest.fail('changed scene became ready under the old identity')
        assert world.backend is None and world._executor is None
    asyncio.run(check())


def test_shared_description_rejects_changed_scene_before_worker_ready(tmp_path):
    make_room(tmp_path)
    factory = MujocoChassisFactory(scene_dir=tmp_path)
    key = 'changed-scene:' + uuid.uuid4().hex
    log = tmp_path/'worker.log'
    old = factory.shared(key=key, log_path=log)
    path = tmp_path/'scene.xml'
    path.write_text(path.read_text() + '\n')
    async def check():
        with pytest.raises(ComponentError, match='scene resources changed'):
            async with Runtime(old.state):
                pytest.fail('changed scene became ready under the advertised old fingerprint')
        assert 'scene resources changed' in log.read_text()
        assert not old.directory.exists()
        current = factory.shared(key=key, log_path=log)
        assert current.provider_version != old.provider_version
        async with Runtime(current.state):
            assert current.manifest['provider_version'] == factory.fingerprint()
            assert (await current.state.get(timeout=5)).metadata['episode'] == 0
    asyncio.run(check())


def test_invalid_motion_permission_and_robot_settings_are_rejected(tmp_path):
    factory = MujocoChassisFactory(scene_dir=tmp_path)
    with pytest.raises(TypeError, match='booleans'):
        replace(factory, motion_enabled='false')
    with pytest.raises(ValueError, match='robot dimensions'):
        replace(factory, robot=replace(factory.robot, config=replace(factory.robot.config, wheel_track=float('nan'))))


def test_world_single_owner_snapshots_static_clock_and_native_reset(tmp_path, monkeypatch):
    make_room(tmp_path)
    from mujocosim import simulation
    original = simulation.Simulation
    created = []
    def construct(*args, **kwargs):
        value = original(*args, **kwargs)
        created.append(value)
        return value
    monkeypatch.setattr(simulation, 'Simulation', construct)

    async def check():
        factory = MujocoChassisFactory(scene_dir=tmp_path, motion_enabled=True,
                    sensors=SensorConfig(width=80, height=60), clock='room')
        world = factory.build()
        declared = factory.specs()
        async with Runtime(world):
            assert len(created) == 1
            assert component_ports(world) == declared
            before = await world.image.get()
            snapshot = before.data.pixels.copy()
            await world.velocity.set(VelocityCommand(.25), ttl=.15)
            await asyncio.sleep(.1)
            assert (await world.pose.get()).data.position[0] > .002
            np.testing.assert_array_equal(before.data.pixels, snapshot)
            assert not before.data.pixels.flags.writeable
            stamp = (await world.state.get()).stamp_ns
            result = await world.reset_request.set({})
            assert result == {'episode': 1, 'step_index': 0}
            state = await world.state.get()
            assert state.stamp_ns == 0 < stamp
            assert state.metadata['episode'] == state.data['episode'] == 1
            assert state.metadata['host_monotonic_ns'] > 0
            assert all(frame.metadata['episode'] == 1 for frame in world.pose.frames)
            assert world.command == VelocityCommand()
            assert component_ports(world) == declared
            await asyncio.sleep(.03)
            assert (await world.state.get()).metadata['episode'] == 1  # no TTL-triggered reset
        assert world._executor is None
        await world.close()
        assert created[0]._renderer is None
    asyncio.run(check())


def test_expiry_pause_and_reset_do_not_revive_old_commands(tmp_path):
    make_room(tmp_path)
    async def check():
        world = MujocoChassisFactory(scene_dir=tmp_path, motion_enabled=True).build()
        async with Runtime(world.state, world.velocity):
            old = CommandEnvelope(VelocityCommand(.2), 'test', 'old', 1, time.monotonic_ns()+400_000_000)
            await world.velocity.set(old)
            await world.inspect(lambda sim: time.sleep(.45))
            await asyncio.sleep(.03)
            assert (await world.state.get()).data['wheel_targets'] == [0., 0.]
            await world.reset()
            with pytest.raises(CommandRejected, match='retired'):
                await world.velocity.set(replace(old, sequence=2, deadline_ns=time.monotonic_ns()+400_000_000))
            fresh = replace(old, controller_epoch='new', deadline_ns=time.monotonic_ns()+300_000_000)
            await world.velocity.set(fresh)
            await world.pause()
            paused = (await world.state.get()).stamp_ns
            await asyncio.sleep(.04)
            assert (await world.state.get()).stamp_ns == paused
            assert world.command == VelocityCommand()
            await world.resume()
            await asyncio.sleep(.03)
            assert (await world.state.get()).data['wheel_targets'] == [0., 0.]
    asyncio.run(check())


def test_partial_initialization_and_cancelled_reset_cleanup():
    class Backend:
        def __init__(self):
            self.closed = 0
            self.stamp = 0
        def open(self):
            raise ValueError('failed initialization')
        def stop(self):
            pass
        def close(self):
            self.closed += 1
        def sample(self, demand, force=False):
            return Observation(self.stamp, {'state': {}})
        def step(self, duration, commands):
            self.stamp += duration
        def reset(self):
            time.sleep(.04)
            self.stamp = 0
    async def check():
        backend = Backend()
        world = MuJoCoSource(lambda: backend, outputs={'state': 8})
        with pytest.raises(Exception, match='failed initialization'):
            async with Runtime(world):
                pass
        await world.close()
        assert backend.closed == 1 and world._executor is None
        backend = Backend()
        backend.open = lambda: None
        world = MuJoCoSource(lambda: backend, outputs={'state': 8})
        async with Runtime(world):
            task = asyncio.create_task(world.reset())
            await asyncio.sleep(.01)
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await task
            assert (await world.state.get()).metadata['episode'] == 1
            assert all(frame.metadata['episode'] == 1 for frame in world.state.frames)
        assert backend.closed == 1
    asyncio.run(check())


class BlockingResetBackend:
    """Real owner-thread synchronization without a renderer or wall-time race."""
    def __init__(self):
        self.entered, self.release = threading.Event(), threading.Event()
        self.stamp, self.closed = 0, 0
        self.thread_ids, self.events = set(), []
        self.sample_failed = False

    def record(self, event):
        self.thread_ids.add(threading.get_ident())
        self.events.append(event)

    def open(self):
        self.record('open')

    def sample(self, demand, force=False):
        self.record('sample')
        if self.sample_failed:
            raise ValueError('snapshot failed after reset')
        return Observation(self.stamp, {'state': {}})

    def step(self, duration, commands):
        self.record('step')
        self.stamp += duration

    def reset(self):
        self.record('reset-start')
        self.entered.set()
        assert self.release.wait(1), 'test must release the native reset'
        self.stamp = 0
        self.record('reset-done')

    def stop(self):
        self.record('stop')

    def close(self):
        self.record('close')
        self.closed += 1


async def cancel_reset_twice(backend, task):
    async with asyncio.timeout(.5):
        while not backend.entered.is_set():
            await asyncio.sleep(.001)
    task.cancel()
    await asyncio.sleep(0)
    task.cancel()
    await asyncio.sleep(0)
    assert not task.done(), 'cancellation must wait for the native transaction'


def test_reset_multiple_cancellations_preserve_episode_transaction():
    async def check():
        backend = BlockingResetBackend()
        world = MuJoCoSource(lambda: backend, outputs={'state': 8})
        async with Runtime(world.state):
            await world.pause()
            task = asyncio.create_task(world.reset())
            try:
                await cancel_reset_twice(backend, task)
            finally:
                backend.release.set()
            with pytest.raises(asyncio.CancelledError):
                await task
            frame = await world.state.get()
            assert frame.stamp_ns == 0 and frame.metadata['episode'] == 1
            assert all(frame.metadata['episode'] == 1 for frame in world.state.frames)
            assert not world._reset_tasks and not world._resetting
            assert len(backend.thread_ids) == 1
            assert threading.get_ident() not in backend.thread_ids
        assert backend.closed == 1 and world._executor is None
    asyncio.run(check())


def test_runtime_close_joins_cancelled_reset_before_destroying_owner():
    async def check():
        backend = BlockingResetBackend()
        world = MuJoCoSource(lambda: backend, outputs={'state': 8})
        publish = world._publish
        async def tracked_publish(observation):
            await publish(observation)
            if world._episode:
                backend.events.append('episode-published')
        world._publish = tracked_publish
        runtime = Runtime(world.state)
        async with runtime:
            await world.pause()
            task = asyncio.create_task(world.reset())
            try:
                await cancel_reset_twice(backend, task)
                closing = asyncio.create_task(runtime.aclose())
                await asyncio.sleep(.01)
                assert not closing.done() and backend.closed == 0
            finally:
                backend.release.set()
            with pytest.raises(asyncio.CancelledError):
                await task
            await closing
        assert backend.events.index('reset-done') < backend.events.index('episode-published')
        assert backend.events.index('episode-published') < backend.events.index('close')
        assert world._executor is None and not world._reset_tasks and backend.closed == 1
        assert len(backend.thread_ids) == 1
    asyncio.run(check())


def test_failed_reset_snapshot_aborts_episode_until_successful_reset():
    async def check():
        backend = BlockingResetBackend()
        backend.release.set()
        world = MuJoCoSource(lambda: backend, outputs={'state': 8})
        async with Runtime(world.state):
            await world.pause()
            assert world.state.frames
            backend.sample_failed = True
            with pytest.raises(ValueError, match='snapshot failed'):
                await world.reset()
            assert not world.state.frames and world._paused and world._episode == 1
            with pytest.raises(RuntimeError, match='successful reset'):
                await world.resume()
            backend.sample_failed = False
            assert await world.reset() == {'episode': 2, 'step_index': 0}
            assert (await world.state.get()).metadata['episode'] == 2
            await world.resume()
    asyncio.run(check())


def test_shared_factory_spawn_reuse_generic_subset_and_remote_reset(tmp_path):
    from rsim.devices import ChassisPortsDevice
    from rsim.transport.descriptor import TransportConfig
    make_room(tmp_path)
    async def check():
        key = 'mujocosim-test:' + uuid.uuid4().hex
        transport = TransportConfig('cyclonedds', 61)
        factory = MujocoChassisFactory(scene_dir=tmp_path, sensors=SensorConfig(width=80, height=60))
        shared = factory.shared(key=key, transport=transport, log_path=tmp_path/'provider.log')
        generic = ChassisPortsDevice(key=key, clock=factory.clock, transport=transport)
        special = SimulationDevice(key=key, transport=transport)
        async with Runtime(shared.state, generic.pose, special.state, special.reset_request):
            first = await generic.pose.get(timeout=10)
            assert first.metadata['episode'] == 0
            assert generic.manifest['pid'] == special.manifest['pid'] == shared.manifest['pid']
            assert shared.manifest['pid'] != os.getpid()
            await asyncio.sleep(.15)  # permit endpoint discovery before deadline starts
            ack = await special.reset()
            assert ack['episode'] == 1
            state = await special.state.get(timeout=5)
            while state.metadata['episode'] != 1:
                state = await special.state.get(after=state.sequence, timeout=5)
            assert all(frame.metadata['episode'] == 1 for frame in special.state.frames)
            assert generic.manifest['instance_id'] == special.manifest['instance_id']
    asyncio.run(check())
