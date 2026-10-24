"""Sparse waypoints backed only by actually observed base trajectories."""
import copy
import math
import uuid

import numpy as np


class WaypointTopology:
    def __init__(self, spacing=.8, merge_distance=.18):
        self.spacing, self.merge_distance = spacing, merge_distance
        self.reset(None)

    def reset(self, episode):
        self.episode, self.session = episode, uuid.uuid4().hex
        self.nodes, self.edges = [], []
        self.previous = self.anchor_id = None
        self.trace, self.length, self.revision = [], 0., 0

    def _node(self, pose):
        if self.nodes:
            distances = [math.dist(pose[:2], node['pose'][:2]) for node in self.nodes]
            closest = int(np.argmin(distances))
            if distances[closest] < self.merge_distance:
                return closest
        self.nodes.append(dict(id=len(self.nodes), pose=list(pose), observations=[]))
        return len(self.nodes)-1

    def anchor(self, observation=None):
        if not self.trace:
            return None
        destination = self._node(self.trace[-1])
        if self.anchor_id is not None and self.length > .02:
            self.edges.append(dict(id=len(self.edges), source=self.anchor_id, target=destination,
                                   length_m=self.length, trajectory=copy.deepcopy(self.trace),
                                   evidence='executed_truth_poses'))
        if observation is not None:
            self.nodes[destination]['observations'].append(copy.deepcopy(observation))
        self.anchor_id = destination
        self.trace, self.length = [self.trace[-1]], 0.
        self.revision += 1
        return destination

    def update(self, pose, stamp, sequence, episode):
        pose = list(map(float, pose))
        if episode != self.episode:
            self.reset(episode)
        if self.previous is not None:
            previous_pose, previous_stamp, previous_sequence = self.previous
            if sequence <= previous_sequence:
                return
            dt = stamp-previous_stamp
            distance = math.dist(pose[:2], previous_pose[:2])
            # Missing history, a reset or a teleport is not a traversed edge.
            if sequence != previous_sequence+1 or dt <= 0 or dt > .5 or distance > .6*dt+.03:
                self.trace, self.length, self.anchor_id = [], 0., None
            else:
                self.length += distance
        self.previous = pose, stamp, sequence
        if (not self.trace or math.dist(pose[:2], self.trace[-1][:2]) >= .015 or
                abs(pose[2]-self.trace[-1][2]) >= .1):
            self.trace.append(pose)
        if self.anchor_id is None:
            self.anchor()
        else:
            yaw = pose[2]-self.trace[0][2]
            turned = abs(math.atan2(math.sin(yaw), math.cos(yaw))) > math.pi/4
            if self.length >= self.spacing or (self.length > .2 and turned):
                self.anchor()

    def snapshot(self):
        return copy.deepcopy(dict(schema='mujocosim.waypoint-topology.v1', frame_id='world',
            session_id=self.session, episode=self.episode, revision=self.revision,
            nodes=self.nodes, edges=self.edges))
