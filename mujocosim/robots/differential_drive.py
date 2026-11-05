"""The demo robot: square two-wheel chassis, velocity motors and encoders."""
from dataclasses import dataclass, field
import math
import xml.etree.ElementTree as ET
import numpy as np
from mujocosim.math import numbers as _numbers

@dataclass(frozen=True)
class DifferentialDriveConfig:
    side: float = .6
    wheel_radius: float = .10
    wheel_width: float = .07
    wheel_track: float = .53
    max_wheel_speed: float = 12.

@dataclass(frozen=True)
class DriveCommand:
    linear: float = 0.
    angular: float = 0.

@dataclass(frozen=True)
class DifferentialDrive:
    config: DifferentialDriveConfig = field(default_factory=DifferentialDriveConfig)

    @property
    def clearance_radius(self):
        return self.config.side / np.sqrt(2) + .08

    def build(self, root):
        robot = self.config
        robot_defaults = ET.SubElement(root.find("default"), "default", **{"class": "robot"})
        ET.SubElement(robot_defaults, "geom", contype="2", conaffinity="1", group="1",
                      friction="1 0.005 0.0001", condim="4")
        body = ET.SubElement(root.find("worldbody"), "body", name="base_footprint", pos="0 0 0.005", childclass="robot")
        ET.SubElement(body, "freejoint", name="base_free")
        ET.SubElement(body, "site", name="imu", pos="0 0 0", size="0.005", rgba="0 0 0 0")
        ET.SubElement(body, "geom", name="chassis", type="box", pos="0 0 0.235",
                      size=_numbers([robot.side/2, robot.side/2, 0.065]), mass="12", rgba="0.08 0.35 0.65 1")
        ET.SubElement(body, "geom", name="front_mark", type="box", pos=_numbers([robot.side/2-.001, 0, .25]),
                      size=_numbers([.001, robot.side/6, .025]), mass="0.001", contype="0", conaffinity="0", rgba="1 0.7 0.1 1")
        for sign, side in ((1, "left"), (-1, "right")):
            wheel = ET.SubElement(body, "body", name=f"{side}_wheel", pos=_numbers([0, sign*robot.wheel_track/2, robot.wheel_radius]))
            ET.SubElement(wheel, "joint", name=f"{side}_wheel_joint", type="hinge", axis="0 1 0", damping="0.05", armature="0.01")
            ET.SubElement(wheel, "geom", name=f"{side}_tire", type="cylinder",
                          size=_numbers([robot.wheel_radius, robot.wheel_width/2]),
                          euler="1.5707963267948966 0 0", mass="0.6", rgba="0.055 0.06 0.07 1",
                          friction="1.3 0.0001 0.00001", priority="1", condim="3")
        # Two low-friction ball supports, not driven wheels, stabilise pitch.
        caster_offset = robot.side * (.235 / .6)
        for x in (-caster_offset, caster_offset):
            ET.SubElement(body, "geom", type="sphere", pos=f"{x} 0 0.04", size="0.04", mass="0.1",
                          rgba="0.55 0.55 0.55 1", friction="0.001 0.00001 0.00001", priority="1", condim="3")
        actuator = ET.SubElement(root, "actuator")
        for side in ("left", "right"):
            ET.SubElement(actuator, "velocity", name=f"{side}_motor", joint=f"{side}_wheel_joint", kv="12",
                          ctrlrange=f"-{robot.max_wheel_speed} {robot.max_wheel_speed}", forcerange="-8 8")
        sensor = ET.SubElement(root, "sensor")
        ET.SubElement(sensor, "gyro", name="imu_gyro", site="imu")
        ET.SubElement(sensor, "accelerometer", name="imu_accel", site="imu")
        return body

    def bind(self, model, data):
        return DifferentialDriveState(model, data, self.config)

class DifferentialDriveState:
    def __init__(self, model, data, config):
        self.model, self.data, self.config = model, data, config
        self.body_id = model.body('base_footprint').id
        self.base_qpos = int(model.joint('base_free').qposadr[0])
        self.wheel_indices = [int(model.joint(f'{side}_wheel_joint').qposadr[0]) for side in ('left', 'right')]
        self.motors = [model.actuator(f'{side}_motor').id for side in ('left', 'right')]
        self.odometry = np.zeros(3)
        self.wheels = data.qpos[self.wheel_indices].copy()

    def reset(self, spawn):
        q = self.base_qpos
        self.data.qpos[q:q+3] = [spawn[0], spawn[1], .006]
        self.data.qpos[q+3:q+7] = [np.cos(spawn[2]/2), 0, 0, np.sin(spawn[2]/2)]
        self.stop()

    def after_reset(self):
        self.odometry[:] = 0
        self.wheels = self.data.qpos[self.wheel_indices].copy()

    def apply(self, command):
        if not isinstance(command, DriveCommand):
            raise TypeError('DifferentialDrive expects DriveCommand')
        linear, angular = command.linear, command.angular
        if not np.isfinite([linear, angular]).all():
            raise ValueError('Velocity must be finite')
        c = self.config
        speeds = np.array([linear-angular*c.wheel_track/2, linear+angular*c.wheel_track/2])/c.wheel_radius
        speeds /= max(1., np.max(np.abs(speeds))/c.max_wheel_speed)
        self.data.ctrl[self.motors] = speeds
        return speeds.copy()

    def stop(self):
        self.data.ctrl[self.motors] = 0

    def after_step(self):
        wheels = self.data.qpos[self.wheel_indices].copy()
        left, right = (wheels-self.wheels)*self.config.wheel_radius
        self.wheels = wheels
        distance, angle = (left+right)/2, (right-left)/self.config.wheel_track
        yaw = self.odometry[2] + angle/2
        self.odometry += [distance*math.cos(yaw), distance*math.sin(yaw), angle]

    def telemetry(self):
        import mujoco
        velocity = np.empty(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                self.body_id, velocity, 0)
        # mjOBJ_BODY + flg_local=1 uses the inertial principal-axis frame,
        # which need not match base_footprint. Rotate world vectors explicitly.
        rotation = self.data.xmat[self.body_id].reshape(3, 3)
        velocity[:3] = rotation.T @ velocity[:3]
        velocity[3:] = rotation.T @ velocity[3:]
        return dict(odometry=self.odometry.copy(),
                    body_velocity=velocity,
                    quaternion_xyzw=np.roll(self.data.xquat[self.body_id], -1).copy(),
                    gyro=self.data.sensor('imu_gyro').data.copy(),
                    accel=self.data.sensor('imu_accel').data.copy(),
                    wheel_targets=self.data.ctrl[self.motors].copy())
