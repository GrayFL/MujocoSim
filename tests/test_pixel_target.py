"""Pixel geometry, native physical reachability, and actual-trace topology."""
import asyncio
import threading
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

pytest.importorskip('pyrobo')
from mujocosim import Simulation
from mujocosim.apps.navigation import NavigationWorld
from mujocosim.components.navigation import NavigationConfig, PlanningWorker
from mujocosim.components.pixel_geometry import (PhysicalProbe, PixelTargetConfig,
    scene_description, scene_snapshot, unproject_pixel)
from mujocosim.components.reachability import ReachabilityWorker
from mujocosim.components.topology import WaypointTopology
from mujocosim.config import SensorConfig
from mujocosim.integrations.navigation import NavigationClient
from mujocosim.robots import DifferentialDrive
from rsim.runtime import Runtime
from rsim.runtime.service import RemoteError
from rsim.transport.descriptor import TransportConfig
from test_simulation import make_room


def test_pixel_unprojection_matches_independent_optical_equations():
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('xyz', [.3, -.5, 1.2]).as_matrix()
    matrix[:3, 3] = [2., -3., 1.4]
    k = np.array([[7., 0., 3.2], [0., 8., 2.4], [0., 0., 1.]])
    observation = dict(frame_id='pinned', episode=0, stamp_ns=42, K=k,
                       world_from_camera_optical=matrix, depth=np.full((6, 8), 2.3))
    for u, v in [(0, 0), (3, 2), (7, 5), (3.1, 2.2)]:
        result = unproject_pixel(observation, u, v)
        col, row = result['pixel']
        optical = [(col-3.2)*2.3/7., (row-2.4)*2.3/8., 2.3]
        expected = matrix[:3, :3] @ optical + matrix[:3, 3]
        np.testing.assert_allclose(result['world_point'], expected, atol=1e-12)
    for uv in [(-.01, 0), (8, 0), (0, 6), (np.nan, 0)]:
        with pytest.raises(ValueError):
            unproject_pixel(observation, *uv)
    for z in [np.nan, np.inf, 0, -1]:
        observation['depth'][2, 3] = z
        with pytest.raises(ValueError, match='valid metric depth'):
            unproject_pixel(observation, 3, 2)


def test_support_rejects_subsampling_sized_holes():
    probe = PhysicalProbe.__new__(PhysicalProbe)
    probe.radius, probe.config = .45, PixelTargetConfig()
    probe.floors = np.array([[-2, -2, -.001, 2], [.001, -2, 2, 2]])
    assert not probe.supported([0, 0])[0]  # A 2 mm slot must not disappear.
    assert probe.supported([-1, 0])[0]
    assert not probe.supported([1.8, 0])[0]
    probe.floors[0, 2] = .001
    assert probe.supported([0, 0])[0]  # Adjacent floor boxes form one support.


def test_reachable_target_has_connected_native_path_and_does_not_teleport(tmp_path, monkeypatch):
    make_room(tmp_path)
    path = tmp_path/'scene.xml'
    path.write_text(path.read_text().replace('</worldbody>', '''
        <geom name="partition" type="box" pos="0.8 0 1.25" size="0.05 2 1.25"
              contype="1" conaffinity="2"/>
        </worldbody>'''))
    with Simulation(tmp_path, robot=DifferentialDrive()) as sim:
        before = sim.data.qpos.copy()
        worker = ReachabilityWorker(scene_description(sim), PixelTargetConfig(), NavigationConfig())
        snapshot = scene_snapshot(sim)
        def forbid_grid(*args, **kwargs):
            raise AssertionError('reachability must not load a prepared navigation grid')
        monkeypatch.setattr(np, 'load', forbid_grid)
        for wanted in [[1.4, 0., 1.], [8., 0., 1.]]:
            target = dict(frame_id='wall', episode=0, stamp_ns=0, pixel=[10, 10],
                          depth_m=1., world_point=wanted, coordinate_frame='world')
            result = worker.resolve(target, snapshot, np.empty((0, 3)), threading.Event())
            assert result['goal'][0] < .25
            # Analytic nearest point in the robot's connected half-room: the
            # partition's left face minus the conservative physical radius.
            closest_x = .8-.05-worker.radius-worker.config.physical_margin
            optimum = wanted[0]-closest_x
            distance = np.linalg.norm(np.asarray(result['goal'][:2])-wanted[:2])
            assert optimum <= distance <= optimum+.05
            assert result['world_point'] == wanted
            assert result['search']['reached_nodes'] < result['search']['sampled_nodes']
            assert all(worker.probe.edge_free(a, b) for a, b in zip(result['path'][:-1], result['path'][1:]))
        np.testing.assert_array_equal(sim.data.qpos, before)
        cancelled = threading.Event()
        cancelled.set()
        with pytest.raises(ValueError, match='cancelled'):
            worker.resolve(target, snapshot, np.empty((0, 3)), cancelled)


def test_topology_never_connects_missing_frames_or_reset_episodes():
    graph = WaypointTopology(spacing=.5, merge_distance=.1)
    for i in range(21):
        graph.update([i*.05, 0, 0], i*.1, i+1, 0)
    graph.anchor(dict(pixel=[20, 30], world_point=[1., 0., 0.]))
    data = graph.snapshot()
    assert len(data['nodes']) >= 3 and len(data['edges']) >= 2
    for edge in data['edges']:
        trace = np.asarray(edge['trajectory'])
        assert np.max(np.linalg.norm(np.diff(trace[:, :2], axis=0), axis=1)) <= .051
        assert edge['evidence'] == 'executed_truth_poses'
    count = len(graph.edges)
    graph.update([1.4, 0, 0], 2.2, 24, 0)
    graph.anchor()
    assert len(graph.edges) == count
    graph.update([0, 0, 0], 2.3, 25, 1)
    assert graph.snapshot()['episode'] == 1 and not graph.edges


def test_motion_rechecks_new_physical_obstacles_on_a_previously_valid_route(tmp_path):
    make_room(tmp_path)
    path = tmp_path/'scene.xml'
    path.write_text(path.read_text().replace('</worldbody>', '''
      <body name="moving_box" pos="0.68 1.2 0.15"><freejoint name="box_free"/>
        <geom type="box" size="0.15 0.15 0.15" mass="2" contype="2" conaffinity="3"/>
      </body></worldbody>'''))
    with Simulation(tmp_path, robot=DifferentialDrive()) as sim:
        worker = PlanningWorker(NavigationConfig())
        worker.enable_physics(scene_description(sim), PixelTargetConfig())
        route = dict(points=np.array([[0., 0.], [.5, 0.], [1.2, 0.]]), bounds=(-2., 2., -2., 2.))
        snapshot = scene_snapshot(sim)
        first = worker.step(1, 1., [0, 0, 0], [.2, 0], [1.2, 0, 0], np.empty((0, 3)), route['bounds'], route, snapshot)
        assert first.safe and first.command.linear > 0
        index = int(sim.model.joint('box_free').qposadr[0])
        snapshot['qpos'][index+1] = 0.  # Current physical world differs from the earlier route.
        second = worker.step(1, 1.1, [0, 0, 0], [.2, 0], [1.2, 0, 0], np.empty((0, 3)), route['bounds'], route, snapshot)
        assert not second.safe and second.status == 'route_blocked'
        assert second.command.linear == second.command.angular == 0


def test_pixel_jobs_do_not_block_control_and_old_frames_expire_on_reset(tmp_path):
    make_room(tmp_path)
    async def check():
        bus = TransportConfig('inprocess')
        world = NavigationWorld(tmp_path, name='pixel_jobs', transport=bus, motion_enabled=True,
            sensors=SensorConfig(width=160, height=120), pixel_target=PixelTargetConfig(frame_capacity=2))
        client = NavigationClient('pixel_jobs', transport=bus)
        async with Runtime(world, client):
            frame = await world.map.get(timeout=10)
            while not frame.data['images']:
                frame = await world.map.get(after=frame.sequence, timeout=5)
            assert world.pixel_target.pixel_to_world(80, 60)['world_point'][0] == pytest.approx(1.95, abs=.01)
            assert (await client.pixel_to_world(80, 60))['frame_id'] == client.frame_id
            capture = await client.capture()
            point = await client.pixel_to_world(80, 60)
            assert point['world_point'][0] == pytest.approx(1.95, abs=.01)
            local = await world.pixel_target.resolve_pixel(80, 60)
            assert local['goal'][0] < point['world_point'][0]
            original = world.pixel_target.worker.resolve
            entered = threading.Event()
            def slow(*args):
                entered.set()
                time.sleep(.8)
                return original(*args)
            world.pixel_target.worker.resolve = slow
            action = await client.go_to(.7, 0., 0.)
            task = asyncio.create_task(client.resolve_pixel(80, 60))
            while not entered.is_set():
                await asyncio.sleep(.01)
            await asyncio.sleep(.6)
            assert world.navigator.action_id == action  # Heartbeats survived the worker delay.
            await client.manual(.1, 0.)
            assert world.navigator.results[action]['status'] == 'preempted'
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await task
            await client.manual(brake=True)
            await asyncio.sleep(.5)
            assert world.navigator.action_id is None
            assert await client.pixel_to_world(80, 60) == point  # Historical pose survives actual motion.
            await client.capture()
            await client.capture()
            with pytest.raises(RemoteError, match='evicted'):
                await client.pixel_to_world(80, 60, frame_id=capture['frame_id'])
            await world.source.reset()
            with pytest.raises(RemoteError, match='previous simulation episode'):
                await client.pixel_to_world(80, 60)
    asyncio.run(check())
