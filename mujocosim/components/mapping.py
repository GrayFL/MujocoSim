"""Truth-pose RGBD/laser mapping with graphmap's three-dimensional Environment."""
import asyncio
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
import time
import uuid

import numpy as np
from graphmap.adapters import pointcloud_observation, rgbd_observation
from graphmap.environment import Environment
from graphmap.pose import Pose
from graphmap.pose.conventions import convert_pose
from graphmap.sensor import CameraIntrinsic
from rsim.core.component import Component


def readonly(value, dtype=None):
    result = np.array(value, dtype=dtype, copy=True)
    result.flags.writeable = False
    return result


@dataclass(frozen=True)
class MappingConfig:
    resolution: float = .05
    stride: int = 8
    hz: float = 3.
    obstacle_height: tuple = (.08, 1.16)

    def __post_init__(self):
        if not np.isfinite([self.resolution, self.hz, *self.obstacle_height]).all():
            raise ValueError('mapping settings must be finite')
        if self.resolution <= 0 or self.hz <= 0 or self.stride < 1:
            raise ValueError('resolution, hz and stride must be positive')
        if not 0 < self.obstacle_height[0] < self.obstacle_height[1]:
            raise ValueError('obstacle height must exclude the floor')


def laser_observation(frame, epoch_stamp):
    scan = frame.data
    ranges = np.asarray(scan['ranges'])
    valid = np.isfinite(ranges) & (ranges >= scan['range_min']) & (ranges <= scan['range_max'])
    angles = scan['angle_min'] + np.arange(len(ranges))*scan['angle_increment']
    xyz = np.column_stack((ranges[valid]*np.cos(angles[valid]),
                           ranges[valid]*np.sin(angles[valid]), np.zeros(valid.sum())))
    sensor = Pose.from_matrix(frame.metadata['world_from_lidar'], wrd_frame='world', ego_frame='lidar')
    robot = Pose.from_matrix(frame.metadata['world_from_base'], wrd_frame='world', ego_frame='base_footprint')
    return pointcloud_observation(xyz, pose_sensor=sensor, pose_robot=robot,
        colors=np.tile(np.array([70, 180, 210], np.uint8), (len(xyz), 1)),
        timestamp=(frame.stamp_ns-epoch_stamp)/1e9)


class MapBuilder:
    """Owned by the mapper thread. No scene mesh or navigation grid is read."""
    def __init__(self, config):
        self.config = config
        self.reset(0)

    def reset(self, episode):
        self.environment = Environment(resolution=self.config.resolution, enable_2d_map=False)
        self.episode, self.session = episode, uuid.uuid4().hex
        self.revision = self.scans = self.images = 0
        self.epoch_stamp = None
        self.last_scan = self.last_depth = 0

    def update(self, scan, rgb, depth):
        episode = scan.metadata['episode']
        if episode != self.episode:
            self.reset(episode)
        if self.epoch_stamp is None:
            self.epoch_stamp = scan.stamp_ns
            if depth is not None:
                self.epoch_stamp = min(self.epoch_stamp, depth.stamp_ns)
        if scan.sequence > self.last_scan:
            self.environment.observe(**laser_observation(scan, self.epoch_stamp), update_2d=False)
            self.last_scan, self.scans = scan.sequence, self.scans+1
        if (depth is not None and depth.sequence > self.last_depth and
                depth.metadata['episode'] == episode and depth.stamp_ns == rgb.stamp_ns):
            image = depth.data.pixels
            k = np.asarray(depth.metadata['K'])
            intrinsic = CameraIntrinsic.from_sensor(width=image.shape[1], height=image.shape[0],
                fx=k[0, 0], fy=k[1, 1], ppx=k[0, 2], ppy=k[1, 2])
            optical = Pose.from_matrix(depth.metadata['world_from_camera_optical'],
                                       wrd_frame='world', ego_frame='camera_optical')
            camera = convert_pose(optical, world='graphmap', local='opencv')
            robot = Pose.from_matrix(depth.metadata['world_from_base'],
                                    wrd_frame='world', ego_frame='base_footprint')
            observation = rgbd_observation(image, intrinsic=intrinsic, pose_camera=camera,
                pose_robot=robot, rgb=rgb.data.pixels, depth_kind='axial', depth_scale=1.,
                depth_range=(.05, 10.), stride=self.config.stride,
                timestamp=(depth.stamp_ns-self.epoch_stamp)/1e9)
            self.environment.observe(**observation, update_2d=False)
            self.last_depth, self.images = depth.sequence, self.images+1
        self.revision += 1
        points = self.environment.infopoints
        xyz = readonly(points.xyz, np.float32)
        lo, hi = self.config.obstacle_height
        obstacles = xyz[(xyz[:, 2] >= lo) & (xyz[:, 2] <= hi)]
        obstacles.flags.writeable = False
        return dict(schema='mujocosim.point-map.v1', frame_id='world', session_id=self.session,
            episode=episode, revision=self.revision, resolution=self.config.resolution,
            xyz=xyz, rgb=readonly(points.color[:, :3], np.uint8), obstacles=obstacles,
            scans=self.scans, images=self.images, scan_time=scan.stamp_ns/1e9,
            sensor_received_ns=scan.received_ns, pose=tuple(map(tuple, scan.metadata['world_from_base'])))

    def save(self, directory):
        self.environment.save(directory)


class PointCloudMapper(Component):
    def __init__(self, source, *, config=None):
        super().__init__(inputs=(source.scan, source.image, source.depth))
        self.source = source
        self.config = config or MappingConfig()
        self.map = self.signal('map', history=2)
        self.state = self.signal('state', history=16)
        self.executor = self.builder = None

    async def work(self, function, *args):
        return await asyncio.get_running_loop().run_in_executor(self.executor, function, *args)

    async def open(self):
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='mujocosim-mapping')
        self.builder = await self.work(MapBuilder, self.config)
        self.task('mapping', self.update, hz=self.config.hz)

    async def update(self):
        scan = await self.source.scan.get(timeout=2)
        depth = self.source.depth.frames[-1] if self.source.depth.frames else None
        rgb = next((f for f in reversed(self.source.image.frames)
                    if depth is not None and f.stamp_ns == depth.stamp_ns), None)
        if rgb is None:
            depth = None
        started = time.monotonic()
        result = await self.work(self.builder.update, scan, rgb, depth)
        await self.map.publish(result, stamp_ns=scan.stamp_ns, clock=scan.clock,
                               received_ns=scan.received_ns)
        await self.state.publish(dict(points=len(result['xyz']), obstacles=len(result['obstacles']),
            revision=result['revision'], episode=result['episode'], scans=result['scans'], images=result['images'],
            processing_ms=(time.monotonic()-started)*1000, map_2d=False),
            stamp_ns=scan.stamp_ns, clock=scan.clock, received_ns=scan.received_ns)

    async def save(self, directory):
        await self.work(self.builder.save, Path(directory))

    async def close(self):
        if self.executor is not None:
            self.executor.shutdown(wait=True)
            self.executor = None
