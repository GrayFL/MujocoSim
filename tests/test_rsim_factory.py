"""Factory, native episode, resource and command safety acceptance (F/W/SAFE)."""
import asyncio
from dataclasses import FrozenInstanceError, replace
import json
import os
import subprocess
import sys
import time
import uuid

import numpy as np
import pytest

from rsim import Runtime
from rsim.core import CommandEnvelope, CommandRejected, VelocityCommand
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
    assert shared.factory.__self__ is factory
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
    path = tmp_path/'scene.xml'
    path.write_text(path.read_text() + '\n')
    assert factory.fingerprint() != fingerprint


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
