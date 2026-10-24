"""RSim point-cloud navigation component with one writer and manual preemption."""
import asyncio
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, replace
import math
import time
import uuid

import numpy as np
from pyrobo import (Bounds2D, DWAConfig, Footprint, NavigationInput, NavigationOutput, Navigator,
                    PointCloud, RRTConnectConfig, Twist2D, VoronoiConfig, pose2d)
from pyrobo.pointcloud import PointObstacles
from pyrobo.control import DynamicWindow
from pyrobo.types import path_poses, wrap_angle
from rsim.core.component import Component
from rsim.core.commands import CommandEnvelope, VelocityCommand

from .mapping import laser_observation, readonly
from .pixel_geometry import PhysicalProbe, scene_snapshot


@dataclass(frozen=True)
class NavigationConfig:
    planner: str = 'rrt_connect'
    hz: float = 10.
    max_speed: float = 1.6
    max_yaw_rate: float = 1.2
    linear_acceleration: float = .4
    angular_acceleration: float = 1.2
    command_ttl: float = .25
    sensor_timeout: float = .5
    point_radius: float = .10
    point_spacing: float = .02
    goal_tolerance: float = .08
    bounds: tuple | None = None

    def __post_init__(self):
        if self.planner not in ('rrt_connect', 'voronoi'):
            raise ValueError('planner must be rrt_connect or voronoi')
        numeric = [v for k, v in vars(self).items() if k not in ('planner', 'bounds')]
        if not np.isfinite(numeric).all() or min(numeric) <= 0:
            raise ValueError('navigation settings must be finite and positive')
        if self.command_ttl > .5 or 1/self.hz >= self.command_ttl:
            raise ValueError('navigation period must fit command TTL, at most .5 s')
        if self.bounds is not None:
            Bounds2D(*self.bounds)


class CircularRouteObstacles(PointObstacles):
    """Use the same yaw-independent clearance as the physical route search."""
    def __init__(self, cloud, *, point_radius, radius):
        super().__init__(cloud, point_radius=point_radius)
        self.radius = radius

    def clearances(self, states, footprint, bounds):
        states = np.asarray(states)
        xy = states[..., :2].reshape(-1, 2)
        clearance = self.tree.query(xy)[0]-self.radius-self.point_radius
        boundary = np.minimum.reduce([xy[:, 0]-bounds.xmin, bounds.xmax-xy[:, 0],
                                      xy[:, 1]-bounds.ymin, bounds.ymax-xy[:, 1]])-self.radius
        return np.minimum(clearance, boundary).reshape(states.shape[:-1])


class PlanningWorker:
    def __init__(self, config):
        self.config, self.generation = config, None
        planner = (RRTConnectConfig(time_budget=.08, clearance=.02) if config.planner == 'rrt_connect'
                   else VoronoiConfig(time_budget=.15, clearance=.02))
        self.navigator = Navigator(footprint=Footprint(length=.6, width=.6, margin=.03),
            planner=planner, point_radius=config.point_radius,
            controller=DWAConfig(max_speed=config.max_speed, max_yaw_rate=config.max_yaw_rate,
                linear_acceleration=config.linear_acceleration, angular_acceleration=config.angular_acceleration,
                prediction_time=1.2, integration_dt=.1, linear_samples=5, angular_samples=11,
                lookahead=.65, goal_tolerance=config.goal_tolerance, yaw_tolerance=.08,
                min_turn_yaw_rate=.18), stall_timeout=5.)
        self.probe = None
        # Hard swept-footprint/contact checks provide clearance. A strong soft
        # clearance reward can otherwise trap DWA before a valid narrow turn.
        self.route_controller = DynamicWindow(replace(self.navigator.controller.config,
            lookahead=.35, clearance_weight=.02), self.navigator.footprint)

    def enable_physics(self, description, config):
        self.probe = PhysicalProbe(description, self.navigator.footprint.radius+.02, description['robot_height'], config)

    def step(self, generation, stamp, xy_yaw, velocity, goal, obstacles, bounds, route=None, snapshot=None):
        if generation != self.generation:
            self.navigator.reset()
            self.generation = generation
            self.route_index = 0
            self.waypoint_index = 1
            self.route_controller.reset()
            self.progress_position, self.progress_stamp = np.asarray(xy_yaw[:2]), stamp
        if route is not None:
            return self.route_step(stamp, xy_yaw, velocity, goal, obstacles, route, snapshot)
        return self.navigator.step(NavigationInput(timestamp=stamp, pose=pose2d(*xy_yaw, body='base_footprint'),
            velocity=Twist2D(*velocity), goal=pose2d(*goal, body='goal'),
            obstacles=PointCloud(obstacles, frame='world', timestamp=None),
            bounds=Bounds2D(*bounds), dt=1/self.config.hz))

    def route_step(self, stamp, xy_yaw, velocity, goal, obstacles, route, snapshot):
        self.probe.update(snapshot)
        bounds = Bounds2D(*route['bounds'])
        cloud = CircularRouteObstacles(PointCloud(obstacles, frame='world'),
            point_radius=self.config.point_radius, radius=self.probe.radius)
        pose, target = pose2d(*xy_yaw, body='base_footprint'), pose2d(*goal, body='goal')
        speed = Twist2D(*velocity)
        points = route['points']
        window = points[self.route_index:self.route_index+10]
        self.route_index += int(np.argmin(np.linalg.norm(window-np.asarray(xy_yaw[:2]), axis=1)))
        path = path_poses(points[self.route_index:], 'world', 'base_footprint')
        corners = route.get('waypoints', points)
        self.waypoint_index = min(self.waypoint_index, len(corners)-1)
        while (self.waypoint_index < len(corners)-1 and
               np.linalg.norm(np.asarray(xy_yaw[:2])-corners[self.waypoint_index]) < .04):
            self.waypoint_index += 1
            self.route_controller.reset()
        corner = corners[self.waypoint_index]
        final_leg = self.waypoint_index == len(corners)-1
        controller = self.route_controller
        controller.config = replace(controller.config, goal_tolerance=self.config.goal_tolerance if final_leg else .03)
        if not final_leg:
            direction = math.atan2(corner[1]-xy_yaw[1], corner[0]-xy_yaw[0])
            target = pose2d(*corner, direction, body='waypoint')
        local_path = path_poses(np.vstack((xy_yaw[:2], corner)), 'world', 'base_footprint')
        at_position = np.linalg.norm(np.asarray(xy_yaw[:2])-goal[:2]) <= self.config.goal_tolerance
        at_heading = abs(float(wrap_angle(xy_yaw[2]-goal[2]))) <= controller.config.yaw_tolerance
        if at_position and at_heading:
            local = controller.stop(pose, speed, cloud, bounds, 1/self.config.hz)
            status = 'arrived' if np.max(np.abs(velocity)) < 1e-3 else 'braking'
        else:
            local = controller.step(pose, speed, target, local_path, cloud, bounds, 1/self.config.hz)
            status = 'aligning' if at_position else 'navigating'
        trajectory = local.trajectory.position[:, :2]
        safe = local.safe and self.probe.free(trajectory[0]) and all(
            self.probe.edge_free(a, b) for a, b in zip(trajectory[:-1], trajectory[1:]))
        failure = None if safe else ('physical_support_or_contact' if local.safe else 'point_clearance')
        if not safe:
            status = 'route_blocked'
        if np.linalg.norm(np.asarray(xy_yaw[:2])-self.progress_position) > .1:
            self.progress_position, self.progress_stamp = np.asarray(xy_yaw[:2]), stamp
        if stamp-self.progress_stamp > 15 and status != 'arrived':
            status, safe = 'route_blocked', False
            failure = 'no_progress'
        return NavigationOutput(stamp, local.command if safe else Twist2D(), status, safe,
            path, local.trajectory, dict(planner='physical_point_roadmap',
                controller_mode=local.mode, feasible_candidates=local.feasible_candidates,
                route_index=self.route_index, waypoint_index=self.waypoint_index,
                physical_checked=bool(local.safe), failure=failure))


def finite_dict(values):
    result = {}
    for name, value in values.items():
        if isinstance(value, dict):
            value = finite_dict(value)
        elif isinstance(value, (float, np.floating)):
            value = float(value) if np.isfinite(value) else None
        elif isinstance(value, np.generic):
            value = value.item()
        result[name] = value
    return result


def planning_points(points, spacing):
    """Keep original XYZ observations, selecting once per XY sampling cell.

    Collision tests project all height-filtered returns onto XY anyway. Dense
    vertical surfaces need not create thousands of almost coincident Voronoi
    sites. This is point selection, not an occupied/free-space raster; omitted
    points are within sqrt(2)*spacing in XY of a retained original observation.
    """
    _, indices = np.unique(np.floor(points[:, :2]/spacing), axis=0, return_index=True)
    return points[np.sort(indices)]


class PointCloudNavigation(Component):
    """Inputs are RSim signals; the only actuator output is a deadline envelope.

    Physics, mapping and planning have separate owner threads. A solver result
    is discarded after cancel/preemption/reset or after its acquisition TTL.
    """
    def __init__(self, source, mapper, *, config=None, motion_enabled=False):
        super().__init__(inputs=(source.pose, source.state, source.scan, mapper.map))
        self.source, self.mapper = source, mapper
        self.config = config or NavigationConfig()
        if self.config.point_radius < math.sqrt(2)*(mapper.config.resolution+self.config.point_spacing):
            raise ValueError('point_radius must cover mapping and planning point selection distances')
        self.motion_enabled = motion_enabled
        self.command = self.signal('command', history=4, clock='host:monotonic')
        self.state = self.signal('state', history=32)
        self.path = self.signal('path', history=2)
        self.executor = None
        self.status = 'idle'
        self.goal = self.action_id = None
        self.route = None
        self.results = {}
        self.generation = 0
        self.owner = None
        self.owner_deadline = 0
        self.desired = np.zeros(2)
        self.desired_deadline = 0
        self.sent = np.zeros(2)
        self.bounds = self.config.bounds
        self.diagnostics = {}
        self.manual_owner = None
        self.quit_requested = False

    async def open(self):
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='mujocosim-navigation')
        self.worker = await asyncio.get_running_loop().run_in_executor(self.executor, PlanningWorker, self.config)
        self.command_identity, self.command_epoch = 'navigation:' + uuid.uuid4().hex, uuid.uuid4().hex
        self.sequence, self.last_planned_stamp = 0, None
        self.last_physics = None
        self.episode = None
        self.task('plan', self.plan, hz=self.config.hz)
        self.task('actuate', self.actuate, hz=50.)
        self.task('status', self.publish_state, hz=10.)

    def current_pose(self):
        if not self.source.pose.frames:
            return None
        pose = self.source.pose.frames[-1].data
        return [float(pose.position[0]), float(pose.position[1]), float(pose.euler_rad[2])]

    def _finish(self, status):
        if self.action_id is not None:
            self.results[self.action_id] = dict(status=status, pose=self.current_pose())
            while len(self.results) > 32:
                self.results.pop(next(iter(self.results)))
        self.goal = self.action_id = self.owner = None
        self.route = None
        self.generation += 1
        self.desired[:] = 0
        self.sent[:] = 0
        self.desired_deadline = 0
        self.status = status

    async def enable_physical_routes(self, description, config):
        await asyncio.get_running_loop().run_in_executor(self.executor, self.worker.enable_physics, description, config)

    def go_to(self, x, y, yaw=None, *, owner, deadline, route=None):
        if not self.motion_enabled:
            raise ValueError('nonzero movement requires --enable-motion')
        current = self.current_pose()
        if current is None or not self.mapper.map.frames:
            raise ValueError('waiting for pose and point-cloud map')
        yaw = math.atan2(y-current[1], x-current[0]) if yaw is None else yaw
        if not np.isfinite([x, y, yaw]).all():
            raise ValueError('goal must contain finite world x/y/yaw')
        if self.owner is not None and self.owner != owner:
            raise ValueError('another notebook owns navigation; cancel or use manual control')
        if self.manual_owner is not None:
            raise ValueError('release manual control before sending a navigation goal')
        self._finish('superseded')
        self.goal = [float(x), float(y), float(yaw)]
        if route is not None:
            self.route = dict(points=readonly(route['path'], float),
                              waypoints=readonly(route.get('waypoints', route['path']), float), bounds=tuple(route['bounds']))
        self.owner, self.owner_deadline = owner, deadline
        self.action_id = uuid.uuid4().hex
        self.status, self.manual_owner = 'planning', None
        self.last_planned_stamp = None
        return self.action_id

    def renew(self, owner, deadline):
        if self.owner != owner:
            return False
        self.owner_deadline = deadline
        return True

    def cancel(self, status='cancelled'):
        self._finish(status)
        self.manual_owner = None

    def manual(self, linear, angular, *, owner, deadline, active=True, brake=False):
        if not np.isfinite([linear, angular]).all():
            raise ValueError('manual velocity must be finite')
        if brake:
            self.cancel('manual_stop')
            return
        if not active:
            # Idle panels never acquire or cancel a notebook's goal.
            if self.status == 'manual' and self.manual_owner == owner:
                self.cancel('idle')
            return
        if not self.motion_enabled:
            raise ValueError('manual movement requires --enable-motion')
        if self.status != 'manual' or self.manual_owner != owner:
            self._finish('preempted')
        self.manual_owner, self.status = owner, 'manual'
        self.desired[:] = np.clip([linear, angular],
            [-self.config.max_speed, -self.config.max_yaw_rate],
            [self.config.max_speed, self.config.max_yaw_rate])
        self.desired_deadline = deadline

    async def plan(self):
        if self.goal is None or not all((self.mapper.map.frames, self.source.state.frames, self.source.scan.frames)):
            return
        state = self.source.state.frames[-1]
        scan = self.source.scan.frames[-1]
        mapped = self.mapper.map.frames[-1].data
        if not len(mapped['xyz']):
            self.status = 'waiting_map'
            self.desired_deadline = 0
            return
        stamp = state.stamp_ns/1e9
        if stamp == self.last_planned_stamp:
            return
        if (time.time_ns()-scan.received_ns > self.config.sensor_timeout*1e9 or
                time.time_ns()-state.received_ns > self.config.sensor_timeout*1e9 or
                mapped['episode'] != state.metadata['episode']):
            self.status = 'waiting_sensor'
            self.desired_deadline = 0
            return
        self.last_planned_stamp = stamp
        pose = self.current_pose()
        velocity = np.array([state.data['linear_velocity'], state.data['angular_velocity']])
        if velocity[0] < -.02 or abs(velocity[1]) > self.config.max_yaw_rate+.05:
            self.status = 'braking'
            self.desired_deadline = 0
            return
        velocity = np.clip(velocity, [0., -self.config.max_yaw_rate],
                            [self.config.max_speed, self.config.max_yaw_rate])
        live = laser_observation(scan, scan.stamp_ns)['points_world']
        obstacles = np.concatenate((mapped['obstacles'], live)).astype(np.float64)
        input_count = len(obstacles)
        obstacles = planning_points(obstacles, self.config.point_spacing)
        if self.bounds is None:
            xy = mapped['xyz'][:, :2]
            lo, hi = xy.min(axis=0)-.5, xy.max(axis=0)+.5
            self.bounds = (float(lo[0]), float(hi[0]), float(lo[1]), float(hi[1]))
        generation, goal = self.generation, tuple(self.goal)
        acquired = time.monotonic_ns() - max(0, time.time_ns()-state.received_ns)
        try:
            route = self.route
            snapshot = await self.source.inspect(scene_snapshot) if route is not None else None
            output = await asyncio.get_running_loop().run_in_executor(self.executor, self.worker.step,
                generation, stamp, pose, velocity, goal, obstacles, self.bounds, route, snapshot)
        except Exception as error:
            if generation == self.generation:
                self.diagnostics = dict(error=f'{type(error).__name__}: {error}')
                self._finish('error')
            return
        if generation != self.generation:
            return
        self.diagnostics = finite_dict(output.diagnostics)
        self.diagnostics['input_obstacle_points'] = input_count
        self.status = output.status
        self.desired[:] = [output.command.linear, output.command.angular] if output.safe else [0, 0]
        self.desired_deadline = acquired + int(self.config.command_ttl*1e9)
        await self.path.publish(dict(frame_id='world', points=readonly(output.path.position, np.float32),
            trajectory=readonly(output.trajectory.position, np.float32), action_id=self.action_id,
            revision=mapped['revision'], planner=self.config.planner), stamp_ns=state.stamp_ns, clock=state.clock)
        if output.status == 'arrived':
            self._finish('arrived')
        elif output.diagnostics.get('planner_status') in ('invalid_goal', 'invalid_start'):
            self._finish(output.diagnostics['planner_status'])
        elif not output.safe:
            self._finish('route_blocked' if output.status == 'route_blocked' else 'emergency_stop')

    async def actuate(self):
        now = time.monotonic_ns()
        if self.owner is not None and now >= self.owner_deadline:
            self.cancel('client_lost')
        physics = self.source.state.frames[-1] if self.source.state.frames else None
        fresh = physics is not None and time.time_ns()-physics.received_ns < self.config.sensor_timeout*1e9
        if physics is not None:
            episode = physics.metadata['episode']
            if self.episode is not None and self.episode != episode:
                self.cancel('simulation_reset')
                self.bounds = self.config.bounds
            self.episode = episode
        valid = fresh and now < self.desired_deadline
        if valid:
            stamp = physics.data['physics_time']
            dt = max(0., min(.05, stamp-(self.last_physics if self.last_physics is not None else stamp)))
            rates = np.array([self.config.linear_acceleration, self.config.angular_acceleration])
            self.sent += np.clip(self.desired-self.sent, -rates*dt, rates*dt)
            deadline = self.desired_deadline
        else:
            self.sent[:] = 0
            deadline = now+int(self.config.command_ttl*1e9)
            if self.status == 'manual':
                self.status, self.manual_owner = 'idle', None
        if physics is not None:
            self.last_physics = physics.data['physics_time']
        self.sequence += 1
        envelope = CommandEnvelope(VelocityCommand(*map(float, self.sent)), self.command_identity,
                                   self.command_epoch, self.sequence, deadline)
        await self.command.publish(envelope, stamp_ns=now, clock='host:monotonic')

    async def publish_state(self):
        mapping = self.mapper.state.frames[-1].data if self.mapper.state.frames else {}
        state = dict(status=self.status, action_id=self.action_id, goal=self.goal, pose=self.current_pose(),
            command=self.sent.tolist(), planner=self.config.planner, motion_enabled=self.motion_enabled,
            map=dict(mapping), diagnostics=dict(self.diagnostics), bounds=self.bounds, episode=self.episode,
            owner=self.owner, manual_owner=self.manual_owner)
        state['quit_requested'] = self.quit_requested
        if self.source.state.frames:
            physics = self.source.state.frames[-1].data
            state['physics_time'] = physics['physics_time']
            state['measured_velocity'] = [physics['linear_velocity'], physics['angular_velocity']]
        await self.state.publish(state, stamp_ns=time.monotonic_ns(), clock='host:monotonic')

    async def close(self):
        self.cancel('closed')
        if self.executor is not None:
            self.executor.shutdown(wait=True)
            self.executor = None
