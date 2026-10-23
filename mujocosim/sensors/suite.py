"""Ray lidar and optical-Z RGBD, sampling the current MuJoCo state."""
from dataclasses import dataclass
import mujoco
import numpy as np
from mujocosim.math import transform

@dataclass
class LaserScan:
    time: float
    angles: np.ndarray
    ranges: np.ndarray
    world_from_lidar: np.ndarray

    def points(self, world=False):
        valid = np.isfinite(self.ranges)
        points = np.column_stack((np.cos(self.angles[valid])*self.ranges[valid],
                                  np.sin(self.angles[valid])*self.ranges[valid],
                                  np.zeros(valid.sum())))
        if world:
            points = points @ self.world_from_lidar[:3, :3].T + self.world_from_lidar[:3, 3]
        return points


@dataclass
class RGBDFrame:
    time: float
    rgb: np.ndarray
    depth: np.ndarray
    K: np.ndarray
    world_from_camera_optical: np.ndarray

    def point_cloud(self, stride=4, world=False):
        if stride < 1:
            raise ValueError("stride must be >= 1")
        v, u = np.mgrid[0:self.depth.shape[0]:stride, 0:self.depth.shape[1]:stride]
        z = self.depth[::stride, ::stride]
        valid = np.isfinite(z) & (z > 0)
        points = np.column_stack(((u[valid]-self.K[0, 2])*z[valid]/self.K[0, 0],
                                  (v[valid]-self.K[1, 2])*z[valid]/self.K[1, 1], z[valid]))
        if world:
            points = points @ self.world_from_camera_optical[:3, :3].T + self.world_from_camera_optical[:3, 3]
        return points, self.rgb[::stride, ::stride][valid]


class SensorSuite:
    def __init__(self, model, data, config):
        self.model, self.data, self.sensors = model, data, config
        self.lidar_id = model.site('lidar').id
        self.camera_id = model.camera('rgbd').id
        self._renderer = None
        self._render_options = mujoco.MjvOption()
        self._render_options.geomgroup[:] = [1, 1, 1, 0, 0, 0]
        self._render_options.sitegroup[:] = 0
        self._ray_groups = np.array([1, 0, 1, 0, 0, 0], dtype=np.uint8)
        self.angles = np.linspace(-np.pi, np.pi, config.lidar_beams, endpoint=False)
        self._directions = np.column_stack((np.cos(self.angles), np.sin(self.angles), np.zeros(len(self.angles))))
        self._multi_ray_has_normal = 'normal:' in mujoco.mj_multiRay.__doc__

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    def lidar(self):
        t = transform(self.data.site_xpos[self.lidar_id], self.data.site_xmat[self.lidar_id])
        directions = np.ascontiguousarray(self._directions @ t[:3, :3].T).ravel()
        ids = np.full(len(self.angles), -1, dtype=np.int32)
        distances = np.full(len(self.angles), -1.0)
        args = [self.model, self.data, t[:3, 3].copy(), directions, self._ray_groups, True, -1, ids, distances]
        if self._multi_ray_has_normal:
            args.append(None)
        mujoco.mj_multiRay(*args, len(self.angles), self.sensors.lidar_max)
        distances[(distances < self.sensors.lidar_min) | (distances > self.sensors.lidar_max)] = np.inf
        return LaserScan(float(self.data.time), self.angles.copy(), distances, t)

    def rgbd(self):
        s = self.sensors
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=s.height, width=s.width)
        renderer = self._renderer
        renderer.disable_depth_rendering()
        renderer.update_scene(self.data, camera="rgbd", scene_option=self._render_options)
        rgb = renderer.render().copy()
        try:
            renderer.enable_depth_rendering()
            depth = renderer.render().copy()
        finally:
            renderer.disable_depth_rendering()
        depth[(depth < s.depth_min) | (depth > s.depth_max)] = np.nan
        f = s.height/(2*np.tan(np.deg2rad(s.vertical_fov)/2))
        K = np.array([[f, 0, (s.width-1)/2], [0, f, (s.height-1)/2], [0, 0, 1]])
        # OpenGL camera: right/up/back -> optical: right/down/forward.
        rotation = self.data.cam_xmat[self.camera_id].reshape(3, 3) @ np.diag([1, -1, -1])
        return RGBDFrame(float(self.data.time), rgb, depth, K,
                         transform(self.data.cam_xpos[self.camera_id], rotation))

