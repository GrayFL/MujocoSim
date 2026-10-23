"""Project-side mapping of the differential-drive demo to RSim port schemas.

The RSim adapter owns scheduling and deadlines. This module owns every robot,
scene, sensor and message-format choice for this application.
"""
from functools import partial
from pathlib import Path
import math
import numpy as np

from rsim.adapters.mujoco import MuJoCoSource, Observation, CommandSpec
from rsim.core.commands import VelocityCommand, CommandRejected
from rsim.core.model import Image, PointCloud
from mujocosim.config import PhysicsConfig, SensorConfig
from mujocosim.robots import DifferentialDrive, DriveCommand

def readonly(value):
    array = np.array(value, copy=True)
    array.setflags(write=False)
    return array


def matrix_metadata(value):
    """Small transforms are inline metadata, not separately mapped array files."""
    return tuple(tuple(float(x) for x in row) for row in value)


def xyz_cloud(points, frame_id, colors=None):
    dtype = [(axis, '<f4') for axis in 'xyz']
    if colors is not None:
        dtype += [(channel, 'u1') for channel in 'rgb']
    array = np.empty(len(points), dtype=dtype)
    for i, axis in enumerate('xyz'):
        array[axis] = points[:, i]
    if colors is not None:
        for i, channel in enumerate('rgb'):
            array[channel] = colors[:, i]
    array.setflags(write=False)
    return PointCloud(array, frame_id)


class DifferentialDriveBackend:
    def __init__(self, scene=None, *, scene_dir=None, cache_root='assets/cache',
                 up_axis='auto', floor_height=None, pitch=.08, rebuild=False,
                 spawn=None, robot=None, sensors=None, physics=None,
                 lidar_hz=10., camera_hz=5., motion_enabled=False):
        self.scene, self.scene_dir, self.cache_root = scene, scene_dir, cache_root
        self.options = dict(up_axis=up_axis, floor_height=floor_height, pitch=pitch, force=rebuild)
        self.spawn, self.robot = spawn, robot or DifferentialDrive()
        self.sensors, self.physics = sensors or SensorConfig(), physics or PhysicsConfig()
        self.lidar_hz, self.camera_hz = lidar_hz, camera_hz
        self.motion_enabled = motion_enabled
        self.simulation = None
        self._next_scan = self._next_camera = 0.

    def open(self):
        from graphmap.pose import Pose
        from mujocosim.scenes import prepare_scene
        from mujocosim.simulation import Simulation
        self.pose_type = Pose
        self.prepared_scene = prepare_scene(self.scene_dir if self.scene_dir is not None else self.scene,
                                            self.cache_root, **self.options)
        self.simulation = Simulation(self.prepared_scene, robot=self.robot, sensors=self.sensors,
                                     physics=self.physics, spawn=self.spawn)

    def step(self, duration, commands):
        command = commands['velocity']
        self.simulation.command(DriveCommand(command.linear_x, command.angular_z))
        self.simulation.step(duration)

    def reset(self, spawn=None):
        self.simulation.reset(self.spawn if spawn is None else spawn)
        self._next_scan = self._next_camera = 0.

    def stop(self):
        if self.simulation is not None:
            self.simulation.robot.stop()

    def close(self):
        if self.simulation is not None:
            self.simulation.close()

    def sample(self, demand, force=False):
        sim = self.simulation
        stamp = float(sim.data.time)
        stamp_ns = round(stamp*1e9)
        header = {'stamp': {'sec': stamp_ns//1_000_000_000, 'nanosec': stamp_ns%1_000_000_000},
                  'frame_id': 'base_footprint'}
        telemetry = sim.robot.telemetry()
        odometry = telemetry["odometry"]
        base = sim.world_from_base
        pose = self.pose_type.from_matrix(base, wrd_frame='world', ego_frame='base_footprint')
        odom_pose = self.pose_type(x=odometry[0], y=odometry[1], yaw=odometry[2], degrees=False,
                                   wrd_frame='odom', ego_frame='base_footprint')
        quat = telemetry["quaternion_xyzw"]
        gyro = telemetry["gyro"]
        accel = telemetry["accel"]
        covariance3 = (np.eye(3)*1e-6).ravel().tolist()
        imu = dict(header=dict(header), orientation=dict(zip('xyzw', map(float, quat))),
                   orientation_covariance=covariance3.copy(),
                   angular_velocity=dict(zip('xyz', map(float, gyro))),
                   angular_velocity_covariance=covariance3.copy(),
                   linear_acceleration=dict(zip('xyz', map(float, accel))),
                   linear_acceleration_covariance=covariance3.copy())
        odom = dict(header=dict(header, frame_id='odom'), child_frame_id='base_footprint',
                    pose={'pose': {'position': dict(zip('xyz', map(float, odom_pose.position))),
                                   'orientation': dict(zip('xyzw', map(float, odom_pose.quat)))},
                          'covariance': np.diag([.0001, .0001, 1e6, 1e6, 1e6, .0001]).ravel().tolist()})
        values = dict(pose=pose, truth=pose.copy(), imu=imu, odom=odom,
                      state=dict(backend='mujoco', motion_enabled=self.motion_enabled,
                                 simulation_time=stamp, physics_time=float(sim.data.time),
                                 contacts=int(sim.data.ncon), wheel_targets=telemetry['wheel_targets'].tolist()))
        metadata = {'pose_kind': 'ground_truth', 'world_from_base': matrix_metadata(base)}
        if demand.has('scan', 'lidar_points') and (force or stamp+1e-9 >= self._next_scan):
            scan = sim.lidar()
            ranges = readonly(scan.ranges.astype(np.float32))
            values['scan'] = dict(header=dict(header, frame_id='lidar'), angle_min=float(scan.angles[0]),
                                  angle_max=float(scan.angles[-1]), angle_increment=2*np.pi/len(ranges),
                                  time_increment=0., scan_time=1/self.lidar_hz,
                                  range_min=self.sensors.lidar_min, range_max=self.sensors.lidar_max,
                                  ranges=ranges, intensities=readonly(np.zeros(len(ranges), np.float32)))
            values['lidar_points'] = xyz_cloud(scan.points(), 'lidar')
            metadata['world_from_lidar'] = matrix_metadata(scan.world_from_lidar)
            self._next_scan = stamp + 1/self.lidar_hz
        if demand.has('image', 'depth', 'camera_info', 'points') and (force or stamp+1e-9 >= self._next_camera):
            frame = sim.rgbd()
            values['image'] = Image(readonly(frame.rgb), 'rgb8', 'camera_optical')
            values['depth'] = Image(readonly(frame.depth), '32FC1', 'camera_optical')
            values['camera_info'] = dict(header=dict(header, frame_id='camera_optical'),
                                        width=self.sensors.width, height=self.sensors.height,
                                        K=matrix_metadata(frame.K), depth_unit='m', depth_type='optical_z',
                                        world_from_camera_optical=matrix_metadata(frame.world_from_camera_optical))
            if demand.has('points'):
                points, colors = frame.point_cloud()
                values['points'] = xyz_cloud(points, 'camera_optical', colors)
            metadata.update(K=matrix_metadata(frame.K), depth_unit='m',
                            depth_type='optical_z',
                            world_from_camera_optical=matrix_metadata(frame.world_from_camera_optical))
            self._next_camera = stamp + 1/self.camera_hz
        return Observation(stamp, values, metadata, {
            'odom': {'pose_kind': 'wheel_odometry', 'velocity_frame': 'base_footprint'},
            'imu': {'gravity_removed': False}})


class SimulationSource(MuJoCoSource):
    """Local tooling facade; application control still goes through RSim ports."""
    @property
    def simulation(self):
        return self.backend.simulation if self.backend else None

    @property
    def prepared_scene(self):
        return self.backend.prepared_scene

    @property
    def command(self):
        return self.current_commands['velocity']

    async def inspect(self, callback, *args):
        return await super().inspect(lambda backend: callback(backend.simulation, *args))


def validate_velocity(value):
    if not isinstance(value, VelocityCommand):
        raise CommandRejected('expected VelocityCommand')


def zero_velocity(value):
    return value == VelocityCommand()


def create_source(scene=None, *, scene_dir=None, hz=50., speedup=1., motion_enabled=False,
                  history=128, image_history=8, **options):
    """Assemble this project's differential-drive demo, not a universal robot."""
    if (scene is None) == (scene_dir is None):
        raise ValueError('supply exactly one of scene or scene_dir')
    for name, default in [('lidar_hz',10.), ('camera_hz',5.)]:
        rate = options.get(name, default)
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('sensor rates must be finite and positive')
    physics = options.get('physics') or PhysicsConfig()
    if not math.isfinite(hz) or hz <= 0 or not np.isclose(round(1/hz/physics.timestep)*physics.timestep, 1/hz):
        raise ValueError('1/hz must be an integer multiple of the physics timestep')
    factory = partial(DifferentialDriveBackend, scene, scene_dir=scene_dir,
                      motion_enabled=motion_enabled, **options)
    outputs = {name: history for name in ('pose','truth','odom','imu','state','scan','lidar_points')}
    outputs.update({name: image_history for name in ('image','depth','camera_info','points')})
    return SimulationSource(factory, outputs=outputs,
        commands={'velocity': CommandSpec(VelocityCommand(), zero_velocity, validate_velocity)},
        hz=hz, speedup=speedup, motion_enabled=motion_enabled)
