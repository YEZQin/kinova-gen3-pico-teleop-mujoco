"""Relative tracker anchoring independent of keyboard, SDK and robot transport."""

import numpy as np

from .pose_mapping import Pose


class TrackerFollower:
    def __init__(self, backend, rotation, scale=1.0):
        matrix = np.asarray(rotation, dtype=float)
        if (matrix.shape != (3, 3) or not np.isfinite(matrix).all()
                or not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-6)
                or not np.isclose(np.linalg.det(matrix), 1., atol=1e-6)):
            raise ValueError('rotation must be a proper orthogonal 3x3 matrix')
        if not np.isfinite(scale) or not 0 < scale <= 1:
            raise ValueError('scale must be in (0, 1]')
        self.backend = backend
        self.rotation = matrix.copy()
        self.scale = scale
        self.paused = False
        self.tracker_anchor = None
        self.robot_anchor = None
        self.status = 'waiting_for_tracker'
        self.last_target = None

    def pause(self, reason='paused'):
        self.paused = True
        self.tracker_anchor = None
        self.robot_anchor = None
        self.last_target = None
        self.status = reason
        self.backend.hold()

    def toggle_pause(self):
        if self.paused:
            self.paused = False
            self.status = 'waiting_for_tracker'
        else:
            self.pause()

    def tick(self, sample):
        if self.paused:
            return
        position = np.asarray(sample.position, dtype=float)
        valid = sample.valid and position.shape == (3,) and np.isfinite(position).all()
        if not valid:
            if self.tracker_anchor is not None:
                self.pause('tracking_lost_press_space_to_reanchor')
            return
        if self.tracker_anchor is None:
            self.backend.begin_control()
            pose = self.backend.current_pose()
            if not np.isfinite(pose.position).all():
                self.pause('invalid_robot_feedback')
                raise ValueError('nonfinite robot anchor')
            self.tracker_anchor = position.copy()
            self.robot_anchor = Pose(pose.position.copy(), pose.quaternion.copy())
            self.status = 'following'
            return
        target = Pose(
            self.robot_anchor.position + self.scale * self.rotation @ (position - self.tracker_anchor),
            self.robot_anchor.quaternion.copy(),
        )
        result = self.backend.command_pose(target)
        if not result.accepted:
            self.pause('robot_rejected')
            raise RuntimeError(result.reason)
        self.last_target = target
