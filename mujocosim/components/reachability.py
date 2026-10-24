"""Continuous point roadmap with a native-physics witness for each returned goal."""
import heapq
import math
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.stats import qmc
from pyrobo import Bounds2D, Footprint, PointCloud
from pyrobo.pointcloud import PointObstacles

from .pixel_geometry import PhysicalProbe


def dense_path(points, spacing=.15):
    result = [np.asarray(points[0])]
    for a, b in zip(points[:-1], points[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(np.asarray(b)-a)/spacing)))
        result.extend(np.linspace(a, b, n+1)[1:])
    return np.asarray(result)


class ReachabilityWorker:
    def __init__(self, description, config, navigation):
        self.config, self.navigation = config, navigation
        self.radius = Footprint(length=description['robot_side'], width=description['robot_side'], margin=.03).radius+.02
        self.probe = PhysicalProbe(description, self.radius, description['robot_height'], config)

    def resolve(self, target, snapshot, obstacles, cancelled):
        started = time.monotonic()
        cfg = self.config
        def check_budget():
            if cancelled.is_set():
                raise ValueError('pixel target request cancelled')
            if time.monotonic()-started > cfg.search_seconds:
                raise ValueError('reachability search budget exhausted; no unverified goal was returned')

        self.probe.update(snapshot)
        bounds = Bounds2D(*(self.navigation.bounds or self.probe.bounds))
        cloud = PointObstacles(PointCloud(obstacles, frame='world'), point_radius=self.navigation.point_radius)
        start = np.asarray(snapshot['robot_pose'][:2])
        wanted = np.asarray(target['world_point'][:2])
        def free(p):
            check_budget()
            return cloud.segment_free(p, p, self.radius, bounds) and self.probe.free(p)
        def edge(a, b, goal=None):
            check_budget()
            if not cloud.segment_free(a, b, self.radius, bounds):
                return False
            # Interior segments need a tracking corridor, not a zero-width
            # tangent. Taper only within 15 cm of the actual start/goal so the
            # nearest target is retained and can be approached under tolerance.
            n = max(1, math.ceil(np.linalg.norm(np.asarray(b)-a)/.1))
            samples = np.linspace(a, b, n+1)
            distance = np.linalg.norm(samples-start, axis=1)
            if goal is not None:
                distance = np.minimum(distance, np.linalg.norm(samples-goal, axis=1))
            padding = cfg.route_clearance*np.minimum(distance/.15, 1.)
            for i, (c, d) in enumerate(zip(samples[:-1], samples[1:])):
                if not cloud.segment_free(c, d, self.radius+min(padding[i:i+2]), bounds):
                    return False
            return self.probe.edge_free(a, b)
        if not free(start):
            raise ValueError('robot start has insufficient physical support or navigation clearance')

        # Halton samples are continuous positions, not cells of an occupancy map.
        lo, hi = np.array([bounds.xmin, bounds.ymin]), np.array([bounds.xmax, bounds.ymax])
        count = min(cfg.max_roadmap_points, max(64, math.ceil(np.prod(hi-lo)/cfg.roadmap_spacing**2)))
        points = lo + qmc.Halton(2, scramble=False).random(count)*(hi-lo)
        points = np.vstack((start, points))
        distance = cloud.tree.query(points)[0]
        valid = distance > self.radius+cloud.point_radius
        valid &= np.all((points >= lo+self.radius) & (points <= hi-self.radius), axis=1)
        points = points[valid]
        if not len(points) or not np.array_equal(points[0], start):
            raise ValueError('robot start is outside the search bounds')
        tree = cKDTree(points)
        distances, neighbors = tree.query(points, k=min(len(points), cfg.neighbors+1), distance_upper_bound=cfg.connection_distance)
        costs = np.full(len(points), np.inf)
        parents = np.full(len(points), -1, dtype=int)
        costs[0] = 0.
        queue, done, edges = [(0., 0)], set(), {}
        while queue:
            cost, index = heapq.heappop(queue)
            if index in done:
                continue
            done.add(index)
            check_budget()
            for length, neighbor in zip(np.atleast_1d(distances[index]), np.atleast_1d(neighbors[index])):
                if neighbor >= len(points) or neighbor in done or cost+length >= costs[neighbor]:
                    continue
                key = tuple(sorted((int(index), int(neighbor))))
                if key not in edges:
                    edges[key] = free(points[neighbor]) and edge(points[index], points[neighbor])
                if edges[key]:
                    costs[neighbor], parents[neighbor] = cost+length, index
                    heapq.heappush(queue, (cost+length, int(neighbor)))

        connected = np.flatnonzero(np.isfinite(costs))
        reachable = cKDTree(points[connected])
        # Search in distance order near the clicked point; include every reached
        # roadmap node as a fallback for targets across a wall or a floor gap.
        samples = [wanted]
        for radius in np.arange(cfg.search_resolution, cfg.local_search_radius+cfg.search_resolution/2, cfg.search_resolution):
            angles = np.arange(max(8, math.ceil(2*np.pi*radius/cfg.search_resolution)))
            angles = angles*2*np.pi/len(angles)
            samples.extend(wanted+radius*np.c_[np.cos(angles), np.sin(angles)])
        # A distant point (e.g. beyond a floor gap) must not inherit the coarse
        # global roadmap's goal accuracy. Refine its nearest connected regions
        # at the same local resolution, with exactly the same path proof below.
        ranked = connected[np.argsort(np.linalg.norm(points[connected]-wanted, axis=1))]
        if np.linalg.norm(points[ranked[0]]-wanted) > cfg.local_search_radius:
            for index in ranked[:8]:
                for radius in np.arange(cfg.search_resolution, 1.5*cfg.roadmap_spacing, cfg.search_resolution):
                    angles = np.arange(max(8, math.ceil(2*np.pi*radius/cfg.search_resolution)))
                    angles = angles*2*np.pi/len(angles)
                    samples.extend(points[index]+radius*np.c_[np.cos(angles), np.sin(angles)])
        samples = np.vstack((samples, points[connected]))
        order = np.lexsort((np.linalg.norm(samples-start, axis=1),
                            np.round(np.linalg.norm(samples-wanted, axis=1), 8)))
        selected = anchor = None
        for p in samples[order]:
            if not free(p):
                continue
            # Direct paths are usually best and avoid roadmap approximation.
            if edge(start, p, goal=p):
                selected, anchor = p, 0
                break
            ds, ids = reachable.query(p, k=min(cfg.neighbors, len(connected)))
            candidates = sorted((costs[connected[int(j)]]+float(d), int(connected[int(j)]))
                                for d, j in zip(np.atleast_1d(ds), np.atleast_1d(ids))
                                if d <= cfg.connection_distance)
            for _, j in candidates:
                if edge(points[j], p, goal=p):
                    selected, anchor = p, j
                    break
            if selected is not None:
                break
        if selected is None:
            raise ValueError('no physically reachable target found in the sampled connected region')
        path = [selected]
        while anchor >= 0:
            path.append(points[anchor])
            anchor = int(parents[anchor])
        path = path[::-1]
        # Shortcut only with the same complete support/contact checks.
        simplified, i = [path[0]], 0
        while i < len(path)-1:
            j = len(path)-1
            while j > i+1 and not edge(path[i], path[j], goal=selected):
                j -= 1
            simplified.append(path[j])
            i = j
        path = dense_path(simplified)
        delta = wanted-selected
        yaw = math.atan2(delta[1], delta[0]) if np.linalg.norm(delta) > .01 else float(snapshot['robot_pose'][2])
        return dict(**target, goal=[float(selected[0]), float(selected[1]), yaw],
                    path=path.tolist(), waypoints=np.asarray(simplified).tolist(),
                    bounds=[bounds.xmin, bounds.xmax, bounds.ymin, bounds.ymax],
                    distance_m=float(np.linalg.norm(np.r_[selected, 0.]-target['world_point'])),
                    search=dict(method='connected_point_roadmap', resolution_m=cfg.search_resolution,
                                route_clearance_m=cfg.route_clearance,
                                sampled_nodes=len(points), reached_nodes=len(connected),
                                elapsed_s=time.monotonic()-started, globally_optimal=False),
                    physics_time=snapshot['physics_time'], start_pose=list(map(float, snapshot['robot_pose'])))
