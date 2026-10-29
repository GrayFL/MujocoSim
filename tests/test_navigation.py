"""Project-level truth mapping and arbitration; no RSim framework suites."""
import asyncio
import math
import time

import numpy as np
import pytest

pytest.importorskip('pyrobo')
from rsim.runtime import Runtime
from rsim.transport.descriptor import TransportConfig
from mujocosim.apps.demo import Keyboard
from mujocosim.apps.navigation import NavigationWorld
from mujocosim.components.mapping import MappingConfig
from mujocosim.components.navigation import planning_points
from mujocosim.config import SensorConfig
from mujocosim.integrations.navigation import NavigationClient
from mujocosim.robots import DriveCommand
from test_simulation import make_room


def test_truth_velocity_in_base_frame(tmp_path):
    from mujocosim import Simulation
    from mujocosim.robots import DifferentialDrive
    make_room(tmp_path)
    with Simulation(tmp_path, robot=DifferentialDrive()) as sim:
        sim.reset([0, 0, math.pi/2])
        sim.command(DriveCommand(.2, .15))
        sim.step(.7)
        start = sim.pose2d.copy()
        velocity = sim.robot.telemetry()['body_velocity']
        sim.step(.04)
        end = sim.pose2d
        forward = (end[:2]-start[:2]) @ np.array([math.cos(start[2]), math.sin(start[2])])/.04
        assert velocity[3] == pytest.approx(forward, abs=.01)
        assert velocity[2] == pytest.approx((end[2]-start[2])/.04, abs=.01)


def test_graphmap_transform_and_navigation_preemption(tmp_path):
    make_room(tmp_path)
    async def check():
        transport = TransportConfig('inprocess')
        world = NavigationWorld(tmp_path, name='test_nav', transport=transport, motion_enabled=True,
            sensors=SensorConfig(width=160, height=120), mapping=MappingConfig(stride=4))
        client = NavigationClient('test_nav', transport=transport)
        async with Runtime(world, client):
            frame = await world.map.get(timeout=10)
            while not frame.data['images']:
                frame = await world.map.get(after=frame.sequence, timeout=5)
            cloud = frame.data
            assert cloud['xyz'].shape == cloud['rgb'].shape
            assert not cloud['xyz'].flags.writeable
            assert not world.mapper.builder.environment.enable_2d_map
            xyz = cloud['xyz']
            # Analytic room: all observed points lie on one of the walls/floor.
            distance = np.min(np.abs(np.c_[xyz[:,0]-1.95, xyz[:,0]+1.95,
                                          xyz[:,1]-1.95, xyz[:,1]+1.95, xyz[:,2]]), axis=1)
            assert np.quantile(distance, .99) < .03
            assert np.all((cloud['obstacles'][:,2] >= .08) & (cloud['obstacles'][:,2] <= 1.16))
            action = await client.go_to(.8, 0., 0.)
            await asyncio.sleep(.3)
            # An idle controller must leave navigation untouched.
            await client.manual(active=False)
            assert world.navigator.action_id == action
            await client.manual(.1, 0.)
            assert world.navigator.results[action]['status'] == 'preempted'
            await asyncio.sleep(.4)
            assert world.navigator.manual_owner is None
            np.testing.assert_allclose(world.navigator.sent, 0)
            await client.manual(brake=True)
            assert world.navigator.status == 'manual_stop'
            # A stale/in-flight plan cannot resurrect a superseded goal.
            await asyncio.sleep(.2)
            assert world.navigator.goal is None
            action = await client.go_to(.8, 0., 0.)
            # Stop client renewals to model a dead/stalled notebook kernel.
            client.action_id = None
            await asyncio.sleep(.7)
            assert world.navigator.results[action]['status'] == 'client_lost'
            np.testing.assert_allclose(world.navigator.sent, 0)
            action = await client.go_to(.8, 0., 0.)
            with pytest.raises(TimeoutError):
                await client.wait_for_arrival(action, timeout=.1)
            assert world.navigator.results[action]['status'] == 'cancelled'
            old_session = cloud['session_id']
            await world.source.reset()
            frame = await world.map.get(after=frame.sequence, timeout=5)
            while frame.data['episode'] == 0:
                frame = await world.map.get(after=frame.sequence, timeout=5)
            assert frame.data['session_id'] != old_session
    asyncio.run(check())


def test_viewer_arrows_do_not_use_wasd():
    keys = Keyboard()
    keys(ord('W'))
    assert keys.command == (0., 0.)
    keys(265)
    assert keys.command[0] > 0
    keys(263)
    assert keys.command[1] > 0
    keys(ord(' '))
    assert keys.command == (0., 0.)


def test_planning_selection_preserves_observations_and_coverage():
    from scipy.spatial import cKDTree
    original = np.random.default_rng(2).uniform(-1, 1, (5000, 3))
    selected = planning_points(original, .05)
    assert len(selected) < len(original)
    assert cKDTree(original).query(selected)[0].max() == 0
    assert cKDTree(selected[:, :2]).query(original[:, :2])[0].max() <= math.sqrt(2)*.05


def test_navigation_shared_view_uses_static_pose_clock_and_new_interface(tmp_path):
    import uuid
    from rsim.runtime.sharing import SharedProvider
    from mujocosim.integrations.navigation import NavigationMap

    make_room(tmp_path)
    async def check():
        name = 'world_' + uuid.uuid4().hex[:10]
        transport = TransportConfig('cyclonedds', 63)
        world = NavigationWorld(tmp_path, name=name, transport=transport,
                                sensors=SensorConfig(width=80, height=60))
        shared = SharedProvider(world, key='mujocosim-navigation:' + name,
                                interface_version='mujocosim-navigation-v2',
                                transport=transport, log_path=tmp_path/'navigation-worker.log')
        view = NavigationMap(name, transport=transport)
        async with Runtime(shared.state, view.pose, view.map):
            pose = await view.pose.get(timeout=10)
            mapped = await view.map.get(timeout=10)
            assert pose.clock == 'simulation'
            assert pose.metadata['pose_kind'] == 'ground_truth'
            assert view.manifest['interface_version'] == 'mujocosim-navigation-v2'
            assert mapped.data['episode'] == pose.metadata['episode'] == 0
    asyncio.run(check())
