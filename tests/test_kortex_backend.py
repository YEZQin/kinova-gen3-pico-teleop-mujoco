from __future__ import annotations

import math
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from kinova_teleop.kortex_backend import KortexBackend
from kinova_teleop.pose_mapping import Pose


class _Twist:
    def __init__(self):
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        self.angular_x = 0.0
        self.angular_y = 0.0
        self.angular_z = 0.0


class _TwistCommand:
    def __init__(self):
        self.reference_frame = None
        self.duration = None
        self.twist = _Twist()


class _ServoingModeInformation:
    def __init__(self):
        self.servoing_mode = None


BASE_PB2 = SimpleNamespace(
    TwistCommand=_TwistCommand,
    ServoingModeInformation=_ServoingModeInformation,
    CARTESIAN_REFERENCE_FRAME_BASE=17,
    SINGLE_LEVEL_SERVOING=23,
)


def _feedback(position=(0.0, 0.0, 0.0), angles_deg=(0.0, 0.0, 0.0)):
    return SimpleNamespace(
        base=SimpleNamespace(
            tool_pose_x=position[0],
            tool_pose_y=position[1],
            tool_pose_z=position[2],
            tool_pose_theta_x=angles_deg[0],
            tool_pose_theta_y=angles_deg[1],
            tool_pose_theta_z=angles_deg[2],
        )
    )


class _Base:
    def __init__(self):
        self.sent: list[_TwistCommand] = []
        self.stop_count = 0
        self.servo_modes = []

    def SetServoingMode(self, mode):
        self.servo_modes.append(mode.servoing_mode)

    def SendTwistCommand(self, command):
        self.sent.append(command)

    def Stop(self):
        self.stop_count += 1


class _Cyclic:
    def __init__(self, feedback):
        self.feedback = feedback

    def RefreshFeedback(self):
        return self.feedback


class _Connection:
    def __init__(self, feedback):
        self.base = _Base()
        self.base_cyclic = _Cyclic(feedback)
        self.base_pb2 = BASE_PB2
        self.closed = 0

    def close(self):
        self.base.Stop()
        self.closed += 1


class _Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def _target(position=(0.0, 0.0, 0.0), quaternion=(1.0, 0.0, 0.0, 0.0)):
    return Pose(np.array(position, dtype=np.float64), np.array(quaternion, dtype=np.float64))


def _velocity(command):
    twist = command.twist
    return np.array(
        [
            twist.linear_x,
            twist.linear_y,
            twist.linear_z,
            twist.angular_x,
            twist.angular_y,
            twist.angular_z,
        ]
    )


def test_feedback_fixed_axis_xyz_pose_is_converted_to_wxyz():
    connection = _Connection(_feedback((1.0, 2.0, 3.0), (0.0, 0.0, 90.0)))
    backend = KortexBackend(connection, start_watchdog=False)

    pose = backend.current_pose()

    np.testing.assert_allclose(pose.position, [1.0, 2.0, 3.0], atol=1e-12)
    np.testing.assert_allclose(
        pose.quaternion,
        [math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)],
        atol=1e-12,
    )
    assert connection.base.servo_modes == [23]


def test_servoing_mode_error_stops_before_propagating():
    connection = _Connection(_feedback())

    def fail_servoing_mode(mode):
        raise RuntimeError("servo mode rejected")

    connection.base.SetServoingMode = fail_servoing_mode

    with pytest.raises(RuntimeError, match="servo mode rejected"):
        KortexBackend(connection, start_watchdog=False)

    assert connection.base.stop_count == 1


def test_position_error_is_base_frame_and_clipped_by_vector_norm():
    connection = _Connection(_feedback())
    backend = KortexBackend(connection, kp_linear=1.0, start_watchdog=False)
    backend.begin_control()

    backend.command_pose(_target(position=(0.1, 0.0, 0.0)))

    np.testing.assert_allclose(_velocity(connection.base.sent[-1]), [0.03, 0, 0, 0, 0, 0])
    assert connection.base.sent[-1].reference_frame == 17
    assert connection.base.sent[-1].duration == 0


def test_rotation_error_is_converted_from_radians_and_clipped_in_degrees():
    connection = _Connection(_feedback())
    backend = KortexBackend(connection, kp_angular=1.0, start_watchdog=False)
    backend.begin_control()
    z_90 = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))

    result = backend.command_pose(_target(quaternion=z_90))

    np.testing.assert_allclose(_velocity(connection.base.sent[-1]), [0, 0, 0, 0, 0, 5])
    assert math.isclose(result.rotation_error, math.pi / 2)


def test_hold_stops_only_once_and_close_is_idempotent():
    connection = _Connection(_feedback())
    backend = KortexBackend(connection, start_watchdog=False)
    backend.begin_control()
    backend.command_pose(_target(position=(0.01, 0.0, 0.0)))

    backend.hold()
    backend.hold()
    assert connection.base.stop_count == 1

    backend.close()
    backend.close()
    assert connection.closed == 1
    assert connection.base.stop_count == 2


def test_watchdog_crossing_200_ms_stops_once_and_latches():
    clock = _Clock()
    connection = _Connection(_feedback())
    backend = KortexBackend(connection, monotonic=clock, start_watchdog=False)
    backend.begin_control()
    backend.command_pose(_target(position=(0.01, 0.0, 0.0)))

    clock.now = 0.2
    backend.check_watchdog()
    assert connection.base.stop_count == 0
    clock.now = 0.200001
    backend.check_watchdog()
    backend.check_watchdog()

    assert connection.base.stop_count == 1
    assert backend.command_pose(_target(position=(0.02, 0.0, 0.0))).converged is False
    assert len(connection.base.sent) == 1


def test_fresh_successful_command_resets_watchdog_deadline():
    clock = _Clock()
    connection = _Connection(_feedback())
    backend = KortexBackend(connection, monotonic=clock, start_watchdog=False)
    backend.begin_control()
    backend.command_pose(_target(position=(0.01, 0.0, 0.0)))
    clock.now = 0.15
    backend.command_pose(_target(position=(0.02, 0.0, 0.0)))

    clock.now = 0.21
    backend.check_watchdog()
    assert connection.base.stop_count == 0
    clock.now = 0.350001
    backend.check_watchdog()
    assert connection.base.stop_count == 1


def test_nonzero_command_arms_independent_daemon_watchdog():
    clock = _Clock()
    connection = _Connection(_feedback())
    created_threads = []

    class AdvancingEvent:
        def __init__(self):
            self.wait_count = 0

        def wait(self, timeout):
            self.wait_count += 1
            if self.wait_count == 1:
                clock.now = 0.200001
                return False
            return True

        def set(self):
            pass

    class InlineThread:
        def __init__(self, *, target, name, daemon):
            self.target = target
            self.name = name
            self.daemon = daemon
            created_threads.append(self)

        def start(self):
            pass

        def run(self):
            self.target()

    backend = KortexBackend(
        connection,
        monotonic=clock,
        thread_factory=InlineThread,
        shutdown_event=AdvancingEvent(),
    )
    assert created_threads == []
    backend.begin_control()

    backend.command_pose(_target(position=(0.01, 0.0, 0.0)))
    created_threads[0].run()

    assert len(created_threads) == 1
    assert created_threads[0].daemon is True
    assert connection.base.stop_count == 1


def test_hold_while_feedback_is_in_flight_prevents_stopped_generation_send():
    feedback_started = threading.Event()
    release_feedback = threading.Event()
    connection = _Connection(_feedback())

    def blocking_feedback():
        feedback_started.set()
        assert release_feedback.wait(timeout=1.0)
        return _feedback()

    connection.base_cyclic.RefreshFeedback = blocking_feedback
    backend = KortexBackend(connection, start_watchdog=False)
    backend.begin_control()
    command_thread = threading.Thread(
        target=backend.command_pose,
        args=(_target(position=(0.01, 0.0, 0.0)),),
    )
    command_thread.start()
    assert feedback_started.wait(timeout=1.0)

    backend.hold()
    release_feedback.set()
    command_thread.join(timeout=1.0)

    assert not command_thread.is_alive()
    assert connection.base.stop_count == 1
    assert connection.base.sent == []


def test_watchdog_stop_while_feedback_is_in_flight_prevents_late_send():
    clock = _Clock()
    feedback_started = threading.Event()
    release_feedback = threading.Event()
    connection = _Connection(_feedback())
    backend = KortexBackend(
        connection,
        monotonic=clock,
        start_watchdog=False,
    )
    backend.begin_control()
    backend.command_pose(_target(position=(0.01, 0.0, 0.0)))

    def blocking_feedback():
        feedback_started.set()
        assert release_feedback.wait(timeout=1.0)
        return _feedback()

    connection.base_cyclic.RefreshFeedback = blocking_feedback
    command_thread = threading.Thread(
        target=backend.command_pose,
        args=(_target(position=(0.02, 0.0, 0.0)),),
    )
    command_thread.start()
    assert feedback_started.wait(timeout=1.0)

    clock.now = 0.200001
    backend.check_watchdog()
    release_feedback.set()
    command_thread.join(timeout=1.0)

    assert not command_thread.is_alive()
    assert connection.base.stop_count == 1
    assert len(connection.base.sent) == 1
