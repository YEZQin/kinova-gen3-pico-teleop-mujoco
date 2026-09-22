import numpy as np
from types import SimpleNamespace

from kinova_teleop.pose_mapping import Pose
from kinova_teleop.tracker_follow import TrackerFollower


class Robot:
    def __init__(self):
        self.pose = Pose(np.array([.4, .1, .3]), np.array([1., 0., 0., 0.]))
        self.commands = []
        self.stops = 0
        self.starts = 0

    def begin_control(self):
        self.starts += 1

    def current_pose(self):
        return self.pose

    def command_pose(self, target):
        self.commands.append(target)
        return SimpleNamespace(accepted=True, reason='')

    def hold(self):
        self.stops += 1


def sample(x, valid=True):
    return SimpleNamespace(position=np.array([x, 0., 0.]), valid=valid, reason='stale')


def test_auto_anchor_then_follow_without_grip():
    robot = Robot()
    follower = TrackerFollower(robot, np.eye(3), scale=.5)
    follower.tick(sample(1.))
    assert not robot.commands
    follower.tick(sample(1.02))
    np.testing.assert_allclose(robot.commands[-1].position, [.41, .1, .3])
    np.testing.assert_allclose(robot.commands[-1].quaternion, robot.pose.quaternion)


def test_pause_resume_reanchors_without_jump():
    robot = Robot()
    follower = TrackerFollower(robot, np.eye(3))
    follower.tick(sample(1.))
    follower.toggle_pause()
    follower.tick(sample(2.))
    assert robot.stops == 1 and not robot.commands
    follower.toggle_pause()
    follower.tick(sample(3.))
    assert not robot.commands
    follower.tick(sample(3.001))
    np.testing.assert_allclose(robot.commands[-1].position, [.401, .1, .3])


def test_loss_stops_and_requires_keyboard_resume():
    robot = Robot()
    follower = TrackerFollower(robot, np.eye(3))
    follower.tick(sample(1.))
    follower.tick(sample(1., False))
    follower.tick(sample(2.))
    assert follower.paused and robot.stops == 1 and not robot.commands


def test_startup_invalid_samples_do_not_cancel_automatic_start():
    robot = Robot()
    follower = TrackerFollower(robot, np.eye(3))
    follower.tick(sample(1., False))
    follower.tick(sample(1.))
    assert robot.starts == 1 and not follower.paused
