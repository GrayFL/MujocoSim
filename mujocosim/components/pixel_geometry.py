"""Frame-correct backprojection and private MuJoCo reachability queries."""
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from graphmap.pose import Pose
from graphmap.pose.conventions import convert_pose
from graphmap.sensor import CameraIntrinsic


@dataclass(frozen=True)
class PixelTargetConfig:
    frame_capacity: int = 16
    search_resolution: float = .05
    roadmap_spacing: float = .28
    max_roadmap_points: int = 1800
    neighbors: int = 16
    connection_distance: float = .85
    local_search_radius: float = 1.5
    search_seconds: float = 12.
    physical_margin: float = .035
    route_clearance: float = .06
    edge_step: float = .05
    waypoint_spacing: float = .8
    merge_distance: float = .18

    def __post_init__(self):
        if any(not np.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError('pixel target settings must be finite and positive')
        for name in ('frame_capacity', 'max_roadmap_points', 'neighbors'):
            if type(getattr(self, name)) is not int:
                raise ValueError(f'{name} must be an integer')
        if self.edge_step > 2*self.physical_margin:
            raise ValueError('physical margin must cover half the edge sampling step')


def unproject_pixel(observation, u, v):
    """u=column, v=row; sample the nearest pixel, never fill invalid depth."""
    uv = np.asarray([u, v], dtype=float)
    depth = observation['depth']
    height, width = depth.shape
    if not np.isfinite(uv).all() or not (0 <= uv[0] < width and 0 <= uv[1] < height):
        raise ValueError(f'pixel must lie inside image width={width}, height={height}')
    col, row = np.minimum(np.floor(uv+.5).astype(int), [width-1, height-1])
    z = float(depth[row, col])
    if not np.isfinite(z) or z <= 0:
        raise ValueError('selected pixel has no valid metric depth')
    k = np.asarray(observation['K'], float)
    intrinsic = CameraIntrinsic.from_sensor(width=width, height=height, fx=k[0, 0], fy=k[1, 1],
                                           ppx=k[0, 2], ppy=k[1, 2])
    optical = Pose.from_matrix(observation['world_from_camera_optical'], wrd_frame='world', ego_frame='camera_optical')
    camera = convert_pose(optical, world='graphmap', local='opencv')
    # graphmap hwd order is row/column/NEGATIVE axial depth, X forward.
    xyz = camera.apply(intrinsic.hwd_to_cam(np.array([[row, col, -z]])))[0]
    return dict(frame_id=observation['frame_id'], episode=observation['episode'],
                stamp_ns=observation['stamp_ns'], pixel=[int(col), int(row)],
                depth_m=z, world_point=xyz.tolist(), coordinate_frame='world')


def scene_description(sim):
    """Small immutable recipe captured on the physics owner thread."""
    height = max(1.16, sim.sensors.camera_height+.04, sim.sensors.lidar_height+.04) if sim.sensors else 1.16
    return dict(xml=sim.xml, directory=str(sim.scene_dir), nq=sim.model.nq,
                robot_side=sim.definition.config.side, robot_height=height)


def scene_snapshot(sim):
    return dict(qpos=sim.data.qpos.copy(), physics_time=float(sim.data.time),
                robot_pose=sim.pose2d.copy())


class PhysicalProbe:
    """An independent model/data pair; queries never reposition the real robot.

    A vertical cylinder encloses every yaw of this .6 m robot, including its
    sensors. Native MuJoCo contacts reject obstacles; physical floor boxes
    supply support. No prepared navigation.npz or occupancy raster is read.
    """
    def __init__(self, description, radius, height, config):
        root = ET.fromstring(description['xml'])
        body = root.find(".//body[@name='base_footprint']")
        if body is None:
            raise ValueError('physical pixel targeting requires the differential-drive backend')
        for geom in body.iter('geom'):
            geom.set('contype', '0')
            geom.set('conaffinity', '0')
        chassis = body.find("geom[@name='chassis']")
        chassis.attrib.update(type='cylinder', size=f'{radius+config.physical_margin} {height/2}',
                              pos=f'0 0 {height/2+.006}', contype='2', conaffinity='1', margin='0')
        assets = {p.name: p.read_bytes() for p in Path(description['directory']).iterdir()
                  if p.suffix.lower() in ('.png', '.obj')}
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'), assets=assets)
        self.data = mujoco.MjData(self.model)
        self.robot_body = self.model.body('base_footprint').id
        self.probe = self.model.geom('chassis').id
        self.qpos_index = int(self.model.joint('base_free').qposadr[0])
        self.radius, self.config = radius, config
        self.floors = None
        self.cache = {}

    def update(self, snapshot):
        self.data.qpos[:] = snapshot['qpos']
        self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)
        self.cache.clear()
        if self.floors is not None:
            return
        # Prepared scenes use actual finite box floors at world z=0. Include
        # only immobile collidable boxes whose horizontal upper face is z=0.
        m, d = self.model, self.data
        rotations = d.geom_xmat.reshape(-1, 3, 3)
        floor = ((m.geom_type == mujoco.mjtGeom.mjGEOM_BOX) & (m.geom_contype != 0) &
                 (m.body_weldid[m.geom_bodyid] == 0) & (rotations[:, 2, 2] > .999) &
                 (np.abs(d.geom_xpos[:, 2]+m.geom_size[:, 2]) < .02))
        boxes = []
        for i in np.flatnonzero(floor):
            rotation = rotations[i, :2, :2]
            if not np.allclose(np.abs(rotation).sum(axis=0), 1., atol=1e-7):
                raise ValueError('pixel targeting currently requires axis-aligned physical floor boxes')
            half = np.abs(rotation) @ m.geom_size[i, :2]
            center = d.geom_xpos[i, :2]
            boxes.append([*(center-half), *(center+half)])
        if not boxes:
            raise ValueError('no supported flat physical floor at world z=0')
        self.floors = np.asarray(boxes)
        self.bounds = (float(self.floors[:, 0].min()), float(self.floors[:, 2].max()),
                       float(self.floors[:, 1].min()), float(self.floors[:, 3].max()))

    def supported(self, xy):
        # Exact coverage by the union of floor rectangles, not point sampling:
        # between consecutive x boundaries the available y intervals are fixed.
        # Check the widest vertical slice of the footprint disk in each strip.
        # Include the edge-sampling margin so support is continuous along edges.
        radius = self.radius+self.config.physical_margin
        result = []
        for x, y in np.asarray(xy).reshape(-1, 2):
            f = self.floors
            f = f[(f[:, 0] < x+radius) & (f[:, 2] > x-radius) &
                  (f[:, 1] < y+radius) & (f[:, 3] > y-radius)]
            if not len(f):
                result.append(False)
                continue
            xs = np.unique(np.clip(np.r_[x-radius, x+radius, f[:, 0], f[:, 2]], x-radius, x+radius))
            valid = True
            for a, b in zip(xs[:-1], xs[1:]):
                if b-a < 1e-10:
                    continue
                middle = (a+b)/2
                strip = f[(f[:, 0] <= middle) & (f[:, 2] >= middle)]
                h = np.sqrt(max(0., radius**2-(np.clip(x, a, b)-x)**2))
                end = y-h
                for lo, hi in sorted(strip[:, [1, 3]].tolist()):
                    if lo > end+1e-8:
                        break
                    end = max(end, hi)
                if end < y+h-1e-8:
                    valid = False
                    break
            result.append(valid)
        return np.asarray(result, bool)

    def free(self, xy):
        key = tuple(np.round(xy, 5))
        if key in self.cache:
            return self.cache[key]
        result = bool(self.supported(xy)[0])
        if result:
            q = self.qpos_index
            self.data.qpos[q:q+7] = [*xy, 0., 1., 0., 0., 0.]
            mujoco.mj_forward(self.model, self.data)
            result = not any(self.probe in (int(c.geom1), int(c.geom2)) and c.dist <= 0
                             for c in self.data.contact)
        self.cache[key] = result
        return result

    def edge_free(self, a, b):
        count = max(1, int(np.ceil(np.linalg.norm(np.asarray(b)-a)/self.config.edge_step)))
        return all(self.free(xy) for xy in np.linspace(a, b, count+1))
