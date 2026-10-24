"""Context-bound image targeting, physical reachability and executed-waypoint graph."""
import asyncio
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import threading
import time
import uuid

import numpy as np
from rsim.core.component import Component

from .mapping import readonly, laser_observation
from .navigation import planning_points
from .pixel_geometry import PixelTargetConfig, scene_description, scene_snapshot, unproject_pixel
from .reachability import ReachabilityWorker
from .topology import WaypointTopology


class PixelTarget(Component):
    """Public pixels use (u, v) = (column, row), with depth in metres.

    ``capture`` pins a paired RGBD observation; ``pixel_to_world`` can then use
    that immutable frame after the robot moves. Resolving is a bounded worker
    job; only a separate, live, authorized navigation request can actuate it.
    """
    def __init__(self, source, mapper, navigation, *, config=None):
        super().__init__(navigation, inputs=(source.image, source.depth, source.pose, source.state, mapper.map))
        self.source, self.mapper, self.navigation = source, mapper, navigation
        self.config = config or PixelTargetConfig()
        self.observation = self.signal('observation', history=2)
        self.targets = self.signal('targets', history=4)
        self.topology = self.signal('topology', history=2)
        self.frames, self.jobs, self.actions = OrderedDict(), OrderedDict(), {}
        self.graph = WaypointTopology(self.config.waypoint_spacing, self.config.merge_distance)
        self.executor = None
        self.last_sequence = 0
        self.last_topology = None

    @property
    def episode(self):
        if not self.source.state.frames:
            raise ValueError('waiting for robot state')
        return self.source.state.frames[-1].metadata['episode']

    async def open(self):
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='mujocosim-pixel-target')
        description = await self.source.inspect(scene_description)
        self.worker = await asyncio.get_running_loop().run_in_executor(self.executor,
            ReachabilityWorker, description, self.config, self.navigation.config)
        await self.navigation.enable_physical_routes(description, self.config)
        self.task('resolve-pixels', self.resolve_jobs, hz=20.)
        self.task('executed-topology', self.record, hz=20.)

    def _capture(self):
        images = {(f.stamp_ns, f.clock, f.metadata['episode']): f for f in self.source.image.frames}
        for depth in reversed(self.source.depth.frames):
            image = images.get((depth.stamp_ns, depth.clock, depth.metadata['episode']))
            if image is None or depth.metadata['episode'] != self.episode:
                continue
            for key in ('K', 'world_from_camera_optical'):
                if not np.array_equal(image.metadata[key], depth.metadata[key]):
                    raise ValueError('RGB and depth calibration do not match')
            data = dict(frame_id=uuid.uuid4().hex, episode=self.episode, stamp_ns=depth.stamp_ns,
                clock=depth.clock, rgb=readonly(image.data.pixels), depth=readonly(depth.data.pixels),
                K=depth.metadata['K'], world_from_camera_optical=depth.metadata['world_from_camera_optical'])
            self.frames[data['frame_id']] = data
            while len(self.frames) > self.config.frame_capacity:
                self.frames.popitem(last=False)
            return data
        raise ValueError('waiting for a paired RGBD observation in the current episode')

    async def capture(self):
        data = self._capture()
        await self.observation.publish(data, stamp_ns=data['stamp_ns'], clock=data['clock'])
        return {k: v for k, v in data.items() if k not in ('rgb', 'depth')} | dict(
            width=data['depth'].shape[1], height=data['depth'].shape[0])

    def pixel_to_world(self, u, v, *, frame_id=None):
        if frame_id is None:
            if not self.frames:
                self._capture()
            frame_id = next(reversed(self.frames))
        observation = self.frames.get(frame_id)
        if observation is None:
            raise ValueError('unknown or evicted observation; capture another frame')
        if observation['episode'] != self.episode:
            raise ValueError('observation belongs to a previous simulation episode')
        return unproject_pixel(observation, u, v)

    def start_resolve(self, u, v, *, frame_id=None, owner):
        target = self.pixel_to_world(u, v, frame_id=frame_id)
        if sum(job['status'] in ('queued', 'running') for job in self.jobs.values()) >= 2:
            raise ValueError('pixel resolver is busy; wait for or cancel a pending job')
        identifier = uuid.uuid4().hex
        self.jobs[identifier] = dict(owner=owner, target=target, status='queued', cancel=threading.Event())
        while len(self.jobs) > 32:
            oldest = next(k for k, v in self.jobs.items() if v['status'] not in ('queued', 'running'))
            del self.jobs[oldest]
        return identifier

    async def resolve_pixel(self, u, v, *, frame_id=None, timeout=30.):
        """In-process convenience API; the RPC service uses nonblocking jobs."""
        owner = 'local-pixel:'+uuid.uuid4().hex
        identifier = self.start_resolve(u, v, frame_id=frame_id, owner=owner)
        try:
            async with asyncio.timeout(timeout):
                while True:
                    job = self.job(identifier, owner)
                    if job['status'] == 'ready':
                        return job['result']
                    if job['status'] in ('error', 'cancelled'):
                        raise ValueError(job.get('error', 'pixel resolution cancelled'))
                    await asyncio.sleep(.05)
        except (TimeoutError, asyncio.CancelledError):
            self.cancel_jobs(owner, identifier)
            raise

    def job(self, identifier, owner):
        job = self.jobs.get(identifier)
        if job is None or job['owner'] != owner:
            raise ValueError('unknown pixel job for this client')
        return job

    def job_result(self, identifier, owner):
        job = self.job(identifier, owner)
        return {k: job[k] for k in ('status', 'result', 'error') if k in job}

    def cancel_jobs(self, owner, identifier=None):
        jobs = [self.job(identifier, owner)] if identifier else [v for v in self.jobs.values() if v['owner'] == owner]
        for job in jobs:
            job['cancel'].set()
            job['status'] = 'cancelled'

    async def resolve_jobs(self):
        item = next(((k, v) for k, v in self.jobs.items() if v['status'] == 'queued'), None)
        if item is None:
            return
        identifier, job = item
        job['status'] = 'running'
        try:
            episode = self.episode
            if job['target']['episode'] != episode:
                raise ValueError('simulation reset before pixel resolution')
            snapshot = await self.source.inspect(scene_snapshot)
            if not self.mapper.map.frames or not self.source.scan.frames:
                raise ValueError('waiting for current point-cloud map and laser')
            mapped, scan = self.mapper.map.frames[-1].data, self.source.scan.frames[-1]
            if mapped['episode'] != episode or scan.metadata['episode'] != episode:
                raise ValueError('map and laser belong to a different episode')
            live = laser_observation(scan, scan.stamp_ns)['points_world']
            obstacles = planning_points(np.vstack((mapped['obstacles'], live)), self.navigation.config.point_spacing)
            result = await asyncio.get_running_loop().run_in_executor(self.executor,
                self.worker.resolve, job['target'], snapshot, obstacles, job['cancel'])
            if episode != self.episode or job['cancel'].is_set():
                raise ValueError('pixel resolution invalidated by reset or cancellation')
            result['map_revision'] = mapped['revision']
            result['job_id'] = identifier
            job.update(status='ready', result=result)
            await self.targets.publish(result, stamp_ns=time.monotonic_ns(), clock='host:monotonic')
        except Exception as error:
            job.update(status='cancelled' if job['cancel'].is_set() else 'error', error=str(error))

    def go_to_resolved(self, identifier, *, owner, deadline):
        job = self.job(identifier, owner)
        if job['status'] != 'ready':
            raise ValueError('pixel resolution is not ready or was cancelled')
        result = job['result']
        if result['episode'] != self.episode:
            raise ValueError('simulation reset; resolve the pixel again')
        current = self.navigation.current_pose()
        if current is None or np.linalg.norm(np.asarray(current[:2])-result['start_pose'][:2]) > .12:
            raise ValueError('robot moved since resolution; resolve the pixel again')
        action = self.navigation.go_to(*result['goal'], owner=owner, deadline=deadline, route=result)
        self.actions[action] = result
        return dict(action_id=action, target=result)

    async def record(self):
        for frame in self.source.pose.frames:
            if frame.sequence > self.last_sequence:
                p = frame.data
                self.graph.update([p.position[0], p.position[1], p.euler_rad[2]],
                    frame.stamp_ns/1e9, frame.sequence, frame.metadata['episode'])
                self.last_sequence = frame.sequence
        for action, target in tuple(self.actions.items()):
            result = self.navigation.results.get(action)
            if result is not None:
                if result['status'] == 'arrived' and target['episode'] == self.graph.episode:
                    self.graph.anchor({k: target[k] for k in ('frame_id', 'pixel', 'world_point', 'goal')} | dict(action_id=action))
                del self.actions[action]
        for job in self.jobs.values():
            if job['target']['episode'] != self.graph.episode:
                job['cancel'].set()
                job['status'] = 'cancelled'
        identity = self.graph.session, self.graph.revision
        if identity != self.last_topology:
            await self.topology.publish(self.graph.snapshot(), stamp_ns=time.monotonic_ns(), clock='host:monotonic')
            self.last_topology = identity

    def save(self, path):
        self.graph.anchor()
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.graph.snapshot(), indent=2, ensure_ascii=False), encoding='utf-8')

    async def close(self):
        for job in self.jobs.values():
            job['cancel'].set()
        if self.executor is not None:
            self.executor.shutdown(wait=True, cancel_futures=True)
            self.executor = None
