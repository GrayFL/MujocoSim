from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np

from .config import PhysicsConfig, SensorConfig
from .math import transform
from .sensors import SensorSuite, LaserScan, RGBDFrame
from .model import build_model_xml
from .robots.base import Robot


class Simulation:
    def __init__(self, scene_dir, *, robot: Robot, spawn=None, sensors=None, physics=None):
        self.scene_dir = Path(scene_dir).resolve()
        self.metadata = json.loads((self.scene_dir / "metadata.json").read_text(encoding="utf-8"))
        self.definition = robot
        self.physics = physics or PhysicsConfig()
        self.sensors = sensors
        xml = build_model_xml(self.scene_dir, robot, sensors, self.physics, return_text=True)
        self.xml = xml
        # MuJoCo's native Windows file APIs cannot reliably open Unicode paths.
        # Python loads the files into the virtual filesystem, preserving Chinese
        # workspace/source paths without chdir, copies, or environment changes.
        assets = {p.name: p.read_bytes() for p in self.scene_dir.iterdir()
                  if p.suffix.lower() in (".obj", ".png")}
        self.model = self.data = self.robot = self.sensor_suite = self.navigation = None
        self._closed = False
        try:
            self.model = mujoco.MjModel.from_xml_string(xml, assets=assets)
            self.data = mujoco.MjData(self.model)
            self.robot = robot.bind(self.model, self.data)
            self.body_id = self.robot.body_id
            self.sensor_suite = SensorSuite(self.model, self.data, sensors) if sensors else None
            self.navigation = np.load(self.scene_dir / "navigation.npz")
            self.reset(spawn)
        except BaseException:
            self.close()
            raise

    def reset(self, spawn=None):
        spawn = np.asarray(self.metadata["spawn"] if spawn is None else spawn, float)
        if spawn.shape != (3,) or not np.isfinite(spawn).all():
            raise ValueError("spawn must be finite [world_x, world_y, yaw_radians]")
        if self.clearance_at(spawn[:2]) < self.definition.clearance_radius:
            raise ValueError("Spawn lacks floor support or clearance for this robot")
        mujoco.mj_resetData(self.model, self.data)
        self.robot.reset(spawn)
        mujoco.mj_forward(self.model, self.data)
        for _ in range(round(self.physics.settle_seconds/self.physics.timestep)):
            mujoco.mj_step(self.model, self.data)
        self.data.time = 0
        self.robot.after_reset()
        mujoco.mj_forward(self.model, self.data)

    def clearance_at(self, xy):
        ij = np.floor((np.asarray(xy)-self.navigation["origin"])/self.navigation["resolution"]).astype(int)
        grid = self.navigation["clearance"]
        return float(grid[tuple(ij)]) if np.all(ij >= 0) and np.all(ij < grid.shape) else 0.0

    def command(self, value):
        """Apply a robot-specific command; the engine does not interpret it."""
        return self.robot.apply(value)

    def export_model(self):
        """Explicitly write world.xml beside scene assets for external tooling."""
        return build_model_xml(self.scene_dir, self.definition, self.sensors, self.physics)

    def step(self, duration=0.02, *, before_substep=None):
        """Integrate fixed dt steps, optionally checking a caller-owned guard.

        The callback receives the proposed native end timestamp in nanoseconds.
        It runs before every whole step. A failure leaves completed integration
        intact and refreshes kinematics so the owner can inspect that state.
        """
        steps = round(duration/self.physics.timestep)
        if steps < 1 or not np.isclose(steps*self.physics.timestep, duration, atol=1e-10):
            raise ValueError("duration must be a positive integer multiple of the physics timestep")
        completed = 0
        try:
            for _ in range(steps):
                if before_substep is not None:
                    before_substep(round((self.data.time + self.physics.timestep)*1e9))
                mujoco.mj_step(self.model, self.data)
                completed += 1
        finally:
            if completed:
                self.robot.after_step()
                # mj_step leaves kinematic quantities at the pre-integration state.
                mujoco.mj_forward(self.model, self.data)
        return completed

    @property
    def world_from_base(self):
        return transform(self.data.xpos[self.body_id], self.data.xmat[self.body_id])

    @property
    def pose2d(self):
        t = self.world_from_base
        return np.array([t[0, 3], t[1, 3], np.arctan2(t[1, 0], t[0, 0])])

    def graphmap_pose(self):
        """Optional graphmap integration; lazy import keeps the core portable."""
        from graphmap.pose import Pose
        return Pose.from_matrix(self.world_from_base, wrd_frame="world", ego_frame="base_footprint")

    def lidar(self):
        if self.sensor_suite is None:
            raise RuntimeError('This simulation has no sensor rig')
        return self.sensor_suite.lidar()

    def rgbd(self):
        if self.sensor_suite is None:
            raise RuntimeError('This simulation has no sensor rig')
        return self.sensor_suite.rgbd()

    @property
    def _renderer(self):
        return self.sensor_suite._renderer if self.sensor_suite else None

    @_renderer.setter
    def _renderer(self, value):
        if self.sensor_suite is None:
            raise RuntimeError('This simulation has no sensor rig')
        self.sensor_suite._renderer = value

    def save_snapshot(self, directory):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        scan, frame = self.lidar(), self.rgbd()
        from PIL import Image
        Image.fromarray(frame.rgb).save(directory / "rgb.png")
        np.save(directory / "depth_m.npy", frame.depth)
        np.savez_compressed(directory / "sensors.npz", time=scan.time, angles=scan.angles, ranges=scan.ranges,
                            K=frame.K, world_from_base=self.world_from_base,
                            world_from_lidar=scan.world_from_lidar,
                            world_from_camera_optical=frame.world_from_camera_optical)
        info = dict(time=scan.time, depth_unit="m", depth_type="optical_z", invalid_depth="NaN",
                    lidar_unit="m", invalid_lidar="+inf", K=frame.K.tolist(),
                    world_from_source=self.metadata["world_from_source"],
                    world_from_base=self.world_from_base.tolist(),
                    world_from_lidar=scan.world_from_lidar.tolist(),
                    world_from_camera_optical=frame.world_from_camera_optical.tolist())
        (directory / "metadata.json").write_text(json.dumps(info, indent=2), encoding="utf-8")
        return scan, frame

    def overview(self):
        """Render a cutaway view of the robot without changing sensor visibility."""
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=self.sensors.height, width=self.sensors.width)
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [*self.pose2d[:2], .4]
        camera.distance = 3.2
        camera.azimuth = 135
        camera.elevation = -50
        options = mujoco.MjvOption()
        options.geomgroup[:] = [1, 1, 0, 0, 0, 0]
        options.sitegroup[:] = 0
        self._renderer.disable_depth_rendering()
        self._renderer.update_scene(self.data, camera=camera, scene_option=options)
        return self._renderer.render().copy()

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            if self.robot is not None:
                self.robot.stop()
        finally:
            try:
                if self.sensor_suite is not None:
                    self.sensor_suite.close()
            finally:
                if self.navigation is not None:
                    self.navigation.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
