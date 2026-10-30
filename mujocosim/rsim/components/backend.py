"""Owner-thread conversion from MuJoCo physics to shared device payloads."""
from pathlib import Path
import numpy as np
from rsim.core.model import Image, PointCloud
from mujocosim.config import PhysicsConfig, SensorConfig
from mujocosim.deployment import DEFAULT_CACHE
from mujocosim.robots import DifferentialDrive, DriveCommand
from .protocol import Observation

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
    def __init__(self, scene=None, *, scene_dir=None, cache_root=DEFAULT_CACHE,
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
        self._last_velocity = None
        self._last_time = None

    def open(self):
        from graphmap.pose import Pose
        try:
            from mujocosim.simulation import Simulation
        except ModuleNotFoundError as error:
            if error.name == 'mujoco':
                raise ModuleNotFoundError("MuJoCo provider requires mujocosim[engine]; clients only need mujocosim[rsim]") from error
            raise
        from mujocosim.scenes import prepare_scene
        self.pose_type = Pose
        self.prepared_scene = prepare_scene(self.scene_dir if self.scene_dir is not None else self.scene,
                                            self.cache_root, **self.options)
        self.simulation = Simulation(self.prepared_scene, robot=self.robot, sensors=self.sensors,
                                     physics=self.physics, spawn=self.spawn)

    def step(self, duration, commands):
        command = commands['velocity']
        self.simulation.command(DriveCommand(command.linear_x, command.angular_z))
        self.simulation.step(duration)

    @property
    def stamp_ns(self):
        """Native engine time, read exclusively on the owner thread."""
        return round(float(self.simulation.data.time)*1e9)

    def step_guarded(self, duration, command_provider):
        def before_substep(end_stamp_ns):
            command = command_provider(end_stamp_ns)['velocity']
            self.simulation.command(DriveCommand(command.linear_x, command.angular_z))
            # Robot command conversion can itself take time. Recheck at the
            # final integration boundary, applying any newly expired fallback.
            confirmed = command_provider(end_stamp_ns)['velocity']
            if confirmed != command:
                self.simulation.command(DriveCommand(confirmed.linear_x, confirmed.angular_z))
        return self.simulation.step(duration, before_substep=before_substep)

    def reset(self, spawn=None):
        self.simulation.reset(self.spawn if spawn is None else spawn)
        self._next_scan = self._next_camera = 0.
        self._last_velocity = None
        self._last_time = None

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
        linear, angular = float(telemetry['body_velocity'][3]), float(telemetry['body_velocity'][2])
        dt = None if self._last_time is None else stamp-self._last_time
        acceleration = yaw_acceleration = 0.
        if dt is not None and dt > 0:
            acceleration = (linear-self._last_velocity[0])/dt
            yaw_acceleration = (angular-self._last_velocity[1])/dt
        self._last_velocity, self._last_time = (linear, angular), stamp
        # This planar fixture distinguishes horizontal robot/scene contacts
        # from the normal wheel/support contacts with the floor.
        obstacle_contacts = sum(
            (sim.model.geom_group[contact.geom1] == 1) != (sim.model.geom_group[contact.geom2] == 1)
            and abs(contact.frame[2]) < .7
            for contact in sim.data.contact)
        values = dict(pose=pose, truth=pose.copy(), imu=imu, odom=odom,
                      state=dict(backend='mujoco', motion_enabled=self.motion_enabled,
                                 linear_x=linear, angular_z=angular, acceleration=acceleration,
                                 yaw_acceleration=yaw_acceleration, frame='world', pose_kind='ground_truth',
                                 collided=bool(obstacle_contacts), obstacle_contacts=int(obstacle_contacts),
                                 simulation_time=stamp, physics_time=float(sim.data.time),
                                 linear_velocity=float(telemetry['body_velocity'][3]),
                                 angular_velocity=float(telemetry['body_velocity'][2]),
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

