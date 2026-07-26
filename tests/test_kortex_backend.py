from __future__ import annotations

import math
import threading
from types import SimpleNamespace

import numpy as np
import pytest

import kinova_teleop.kortex_backend as kortex_backend_module
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


class _FingerList:
    def __init__(self):
        self.entries = []

    def add(self):
        finger = SimpleNamespace(finger_identifier=None, value=None)
        self.entries.append(finger)
        return finger


class _GripperCommand:
    def __init__(self):
        self.mode = None
        self.gripper = SimpleNamespace(finger=_FingerList())


ARMSTATE_SERVOING_READY = 31
ARMSTATE_IN_FAULT = 32

BASE_PB2 = SimpleNamespace(
    TwistCommand=_TwistCommand,
    ServoingModeInformation=_ServoingModeInformation,
    GripperCommand=_GripperCommand,
    CARTESIAN_REFERENCE_FRAME_BASE=17,
    SINGLE_LEVEL_SERVOING=23,
    GRIPPER_POSITION=3,
    ARMSTATE_SERVOING_READY=ARMSTATE_SERVOING_READY,
    ARMSTATE_IN_FAULT=ARMSTATE_IN_FAULT,
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
        self.send_options = []
        self.stop_options = []
        self.arm_state = ARMSTATE_SERVOING_READY
        self.arm_state_reads = 0
        self.clear_faults_count = 0
        self.gripper_sent = []
        self.gripper_options = []

    def SetServoingMode(self, mode, *, options=None):
        self.servo_modes.append(mode.servoing_mode)

    def SendTwistCommand(self, command, *, options=None):
        self.send_options.append(options)
        self.sent.append(command)

    def GetArmState(self, *, options=None):
        self.arm_state_reads += 1
        return SimpleNamespace(active_state=self.arm_state)

    def ClearFaults(self, *, options=None):
        self.clear_faults_count += 1
        self.arm_state = ARMSTATE_SERVOING_READY

    def SendGripperCommand(self, command, *, options=None):
        self.gripper_options.append(options)
        self.gripper_sent.append(command)

    def Stop(self, *, options=None):
        self.stop_options.append(options)
        self.stop_count += 1


class _Cyclic:
    def __init__(self, feedback):
        self.feedback = feedback
        self.options = []

    def RefreshFeedback(self, *, options=None):
        self.options.append(options)
        return self.feedback


class _Connection:
    def __init__(self, feedback, *, close_result=True):
        self.base = _Base()
        self.base_cyclic = _Cyclic(feedback)
        self.base_pb2 = BASE_PB2
        self.closed = 0
        self.close_result = close_result

    def rpc_options(self):
        return SimpleNamespace(timeout_ms=100)

    def close(self):
        self.base.Stop(options=self.rpc_options())
        self.closed += 1
        return self.close_result


class _ManualThread:
    instances = []

    def __init__(self, *, target, args=(), name, daemon):
        self.target = target
        self.args = args
        self.name = name
        self.daemon = daemon
        self.started = False
        self.__class__.instances.append(self)

    def start(self):
        self.started = True
        self.args[0].set()

    def join(self, timeout=None):
        pass


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


def _backend(connection, **kwargs):
    return KortexBackend(connection, **kwargs)


@pytest.fixture(autouse=True)
def _private_watchdog_thread_seam(monkeypatch):
    _ManualThread.instances.clear()
    monkeypatch.setattr(
        kortex_backend_module,
        "_watchdog_thread_factory",
        _ManualThread,
    )


def test_feedback_fixed_axis_xyz_pose_is_converted_to_wxyz():
    connection = _Connection(_feedback((1.0, 2.0, 3.0), (0.0, 0.0, 90.0)))
    backend = _backend(connection)

    pose = backend.current_pose()

    np.testing.assert_allclose(pose.position, [1.0, 2.0, 3.0], atol=1e-12)
    np.testing.assert_allclose(
        pose.quaternion,
        [math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)],
        atol=1e-12,
    )
    assert connection.base.servo_modes == [23]
    assert connection.base_cyclic.options[-1].timeout_ms < 200


def test_servoing_mode_error_stops_before_propagating():
    connection = _Connection(_feedback())

    def fail_servoing_mode(mode, *, options=None):
        raise RuntimeError("servo mode rejected")

    connection.base.SetServoingMode = fail_servoing_mode

    with pytest.raises(RuntimeError, match="servo mode rejected"):
        _backend(connection)

    assert connection.base.stop_count == 1


def test_position_error_is_base_frame_and_clipped_by_vector_norm():
    connection = _Connection(_feedback())
    backend = _backend(connection, kp_linear=1.0)
    backend.begin_control()

    backend.command_pose(_target(position=(0.1, 0.0, 0.0)))

    np.testing.assert_allclose(_velocity(connection.base.sent[-1]), [0.03, 0, 0, 0, 0, 0])
    assert connection.base.sent[-1].reference_frame == 17
    assert connection.base.sent[-1].duration == 0
    assert connection.base.send_options[-1].timeout_ms < 200


def test_rotation_error_is_converted_from_radians_and_clipped_in_degrees():
    connection = _Connection(_feedback())
    backend = _backend(connection, kp_angular=1.0)
    backend.begin_control()
    z_90 = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))

    result = backend.command_pose(_target(quaternion=z_90))

    np.testing.assert_allclose(_velocity(connection.base.sent[-1]), [0, 0, 0, 0, 0, 5])
    assert math.isclose(result.rotation_error, math.pi / 2)


def test_hold_stops_only_once_and_close_is_idempotent():
    connection = _Connection(_feedback())
    backend = _backend(connection)
    backend.begin_control()
    backend.command_pose(_target(position=(0.01, 0.0, 0.0)))

    backend.hold()
    backend.hold()
    assert connection.base.stop_count == 1

    backend.close()
    backend.close()
    assert connection.closed == 1
    assert connection.base.stop_count == 2


def test_false_connection_close_remains_unconfirmed_across_backend_close_retries():
    """A repeated backend close must not erase an explicit False Stop result."""

    connection = _Connection(_feedback(), close_result=False)
    backend = _backend(connection)

    with pytest.raises(RuntimeError, match="unconfirmed"):
        backend.close()
    with pytest.raises(RuntimeError, match="unconfirmed"):
        backend.close()

    assert backend.stop_confirmed is False
    assert connection.closed == 1


def test_watchdog_crossing_200_ms_stops_once_and_latches():
    clock = _Clock()
    connection = _Connection(_feedback())
    backend = _backend(connection, monotonic=clock)
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
    backend = _backend(connection, monotonic=clock)
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
    backend = KortexBackend(connection, monotonic=clock)
    assert _ManualThread.instances == []
    backend.begin_control()

    backend.command_pose(_target(position=(0.01, 0.0, 0.0)))
    clock.now = 0.200001
    backend.check_watchdog()

    assert len(_ManualThread.instances) == 1
    assert _ManualThread.instances[0].daemon is True
    assert _ManualThread.instances[0].started is True
    assert connection.base.stop_count == 1


def test_hold_while_feedback_is_in_flight_prevents_stopped_generation_send():
    feedback_started = threading.Event()
    release_feedback = threading.Event()
    connection = _Connection(_feedback())
    backend = _backend(connection)
    backend.begin_control()
    backend.current_pose()

    def blocking_feedback(*, options=None):
        feedback_started.set()
        assert release_feedback.wait(timeout=1.0)
        return _feedback()

    connection.base_cyclic.RefreshFeedback = blocking_feedback
    command_thread = threading.Thread(
        target=backend.command_pose,
        args=(_target(position=(0.01, 0.0, 0.0)),),
    )
    command_thread.start()
    assert feedback_started.wait(timeout=1.0)

    with pytest.raises(RuntimeError, match="unconfirmed"):
        backend.hold()
    release_feedback.set()
    command_thread.join(timeout=1.0)
    backend.check_watchdog()

    assert not command_thread.is_alive()
    assert connection.base.stop_count == 1
    assert connection.base.sent == []


def test_watchdog_stop_while_feedback_is_in_flight_prevents_late_send():
    clock = _Clock()
    feedback_started = threading.Event()
    release_feedback = threading.Event()
    connection = _Connection(_feedback())
    backend = _backend(connection, monotonic=clock)
    backend.begin_control()
    backend.command_pose(_target(position=(0.01, 0.0, 0.0)))

    def blocking_feedback(*, options=None):
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
    backend.check_watchdog()

    assert not command_thread.is_alive()
    assert connection.base.stop_count == 1
    assert len(connection.base.sent) == 1


def test_blocked_send_has_bounded_options_and_watchdog_latches_before_stop_rpc():
    clock = _Clock()
    send_started = threading.Event()
    release_send = threading.Event()
    connection = _Connection(_feedback())
    backend = _backend(connection, monotonic=clock)
    backend.begin_control()
    backend.command_pose(_target(position=(0.01, 0.0, 0.0)))

    original_send = connection.base.SendTwistCommand

    def blocking_send(command, *, options=None):
        assert options.timeout_ms < 200
        send_started.set()
        assert release_send.wait(timeout=1.0)
        original_send(command, options=options)

    connection.base.SendTwistCommand = blocking_send
    clock.now = 0.15
    blocked_command = threading.Thread(
        target=backend.command_pose,
        args=(_target(position=(0.02, 0.0, 0.0)),),
    )
    blocked_command.start()
    assert send_started.wait(timeout=1.0)

    clock.now = 0.200001
    watchdog = threading.Thread(target=backend.check_watchdog)
    watchdog.start()
    rejected_result = []
    later_command = threading.Thread(
        target=lambda: rejected_result.append(
            backend.command_pose(_target(position=(0.03, 0.0, 0.0)))
        )
    )
    later_command.start()
    later_command.join(timeout=0.25)

    try:
        assert not later_command.is_alive()
        assert rejected_result[0].converged is False
        assert backend.stop_requested is True
    finally:
        release_send.set()
        blocked_command.join(timeout=1.0)
        watchdog.join(timeout=1.0)

    assert connection.base.stop_count == 1


def test_stop_failure_stays_unconfirmed_and_watchdog_retry_can_confirm():
    connection = _Connection(_feedback())
    backend = _backend(connection)
    backend.begin_control()
    attempts = []

    def flaky_stop(*, options=None):
        attempts.append(options.timeout_ms)
        if len(attempts) == 1:
            raise RuntimeError("stop timeout")

    connection.base.Stop = flaky_stop

    with pytest.raises(RuntimeError, match="unconfirmed"):
        backend.hold()

    assert backend.stop_requested is True
    assert backend.stop_confirmed is False
    backend.check_watchdog()
    assert backend.stop_confirmed is True
    assert attempts == [100, 100]


def test_watchdog_stop_exception_does_not_escape_and_is_retried():
    clock = _Clock()
    connection = _Connection(_feedback())
    backend = _backend(connection, monotonic=clock)
    backend.begin_control()
    backend.command_pose(_target(position=(0.01, 0.0, 0.0)))
    attempts = []

    def flaky_stop(*, options=None):
        attempts.append(options.timeout_ms)
        if len(attempts) == 1:
            raise RuntimeError("stop timeout")

    connection.base.Stop = flaky_stop
    clock.now = 0.200001

    backend.check_watchdog()
    assert backend.stop_confirmed is False
    backend.check_watchdog()

    assert backend.stop_confirmed is True
    assert attempts == [100, 100]


def test_watchdog_thread_start_failure_attempts_stop_and_reports_unconfirmed(
    monkeypatch,
):
    connection = _Connection(_feedback())

    class StartFailure:
        def __init__(self, *, target, args=(), name, daemon):
            pass

        def start(self):
            raise RuntimeError("thread unavailable")

    def failed_stop(*, options=None):
        raise RuntimeError("stop unavailable")

    monkeypatch.setattr(
        kortex_backend_module,
        "_watchdog_thread_factory",
        StartFailure,
    )
    connection.base.Stop = failed_stop
    backend = KortexBackend(connection)
    backend.begin_control()

    with pytest.raises(RuntimeError, match="Stop attempted but unconfirmed"):
        backend.command_pose(_target(position=(0.01, 0.0, 0.0)))

    assert backend.stop_requested is True
    assert backend.stop_confirmed is False
    assert connection.base.sent == []


def test_watchdog_thread_construction_failure_attempts_stop(monkeypatch):
    connection = _Connection(_feedback())

    class ConstructionFailure:
        def __init__(self, *, target, args=(), name, daemon):
            raise RuntimeError("thread construction unavailable")

    monkeypatch.setattr(
        kortex_backend_module,
        "_watchdog_thread_factory",
        ConstructionFailure,
    )
    backend = KortexBackend(connection)
    backend.begin_control()

    with pytest.raises(RuntimeError, match="Failed to start Kortex watchdog"):
        backend.command_pose(_target(position=(0.01, 0.0, 0.0)))

    assert connection.base.stop_count == 1
    assert backend.stop_confirmed is True


def test_watchdog_thread_that_never_reports_ready_stops_before_first_nonzero_send(
    monkeypatch,
):
    connection = _Connection(_feedback())

    class SilentThread:
        def __init__(self, *, target, args=(), name, daemon):
            pass

        def start(self):
            pass

        def join(self, timeout=None):
            pass

    monkeypatch.setattr(
        kortex_backend_module,
        "_watchdog_thread_factory",
        SilentThread,
    )
    backend = KortexBackend(connection)
    backend.begin_control()

    with pytest.raises(RuntimeError, match="ready"):
        backend.command_pose(_target(position=(0.01, 0.0, 0.0)))

    assert connection.base.sent == []
    assert connection.base.stop_count == 1
    assert backend.stop_confirmed is True


def test_twist_adaptation_error_attempts_stop():
    connection = _Connection(_feedback())
    connection.base_pb2 = SimpleNamespace(
        TwistCommand=lambda: (_ for _ in ()).throw(RuntimeError("bad message")),
        ServoingModeInformation=_ServoingModeInformation,
        CARTESIAN_REFERENCE_FRAME_BASE=17,
        SINGLE_LEVEL_SERVOING=23,
        ARMSTATE_SERVOING_READY=ARMSTATE_SERVOING_READY,
        ARMSTATE_IN_FAULT=ARMSTATE_IN_FAULT,
    )
    backend = _backend(connection)
    backend.begin_control()

    with pytest.raises(RuntimeError, match="bad message"):
        backend.command_pose(_target(position=(0.01, 0.0, 0.0)))

    assert connection.base.stop_count == 1
    assert backend.stop_confirmed is True


def test_watchdog_cannot_be_disabled_through_production_constructor():
    connection = _Connection(_feedback())

    with pytest.raises(TypeError):
        KortexBackend(connection, start_watchdog=False)
    with pytest.raises(TypeError):
        KortexBackend(connection, shutdown_event=threading.Event())
    with pytest.raises(TypeError):
        KortexBackend(connection, thread_factory=_ManualThread)


def test_stop_attempt_token_cannot_cross_into_new_control_epoch():
    connection = _Connection(_feedback())
    backend = _backend(connection)
    backend.begin_control()
    old_token = backend._request_stop(force=True)
    backend._attempt_stop(token=old_token, raise_on_failure=False)
    assert connection.base.stop_count == 1
    backend.begin_control()
    new_token = backend._request_stop(force=True)

    backend._attempt_stop(token=old_token, raise_on_failure=False)
    assert connection.base.stop_count == 1
    backend._attempt_stop(token=new_token, raise_on_failure=False)

    assert connection.base.stop_count == 2
    assert backend.stop_confirmed is True


def test_queued_old_stop_attempt_is_discarded_after_new_stop_epoch():
    connection = _Connection(_feedback())
    backend = _backend(connection)
    backend.begin_control()

    class FirstAcquireGate:
        def __init__(self):
            self.first = True
            self.queued = threading.Event()
            self.release_first = threading.Event()

        def acquire(self, timeout=None):
            if self.first:
                self.first = False
                self.queued.set()
                return self.release_first.wait(timeout=timeout)
            return True

        def release(self):
            pass

    gate = FirstAcquireGate()
    backend._rpc_lock = gate
    old_token = backend._request_stop(force=True)
    old_attempt = threading.Thread(
        target=lambda: backend._attempt_stop(
            token=old_token,
            raise_on_failure=False,
        )
    )
    old_attempt.start()
    assert gate.queued.wait(timeout=1.0)
    new_token = backend._request_stop(force=True)
    gate.release_first.set()
    old_attempt.join(timeout=1.0)

    assert not old_attempt.is_alive()
    assert connection.base.stop_count == 0
    backend._attempt_stop(token=new_token, raise_on_failure=False)
    assert connection.base.stop_count == 1


def test_stop_request_blocks_new_feedback_before_rpc_admission():
    connection = _Connection(_feedback())
    backend = _backend(connection)
    backend.begin_control()
    backend._request_stop(force=True)
    refresh_count = len(connection.base_cyclic.options)

    with pytest.raises(RuntimeError, match="Stop"):
        backend.current_pose()

    assert len(connection.base_cyclic.options) == refresh_count


def test_confirmed_stop_blocks_ordinary_feedback_until_explicit_rearm_transaction():
    """Clearing Stop admission without begin_control must break this test."""

    connection = _Connection(_feedback((1.0, 2.0, 3.0)))
    backend = _backend(connection)
    backend.begin_control()
    backend.current_pose()
    backend.hold()
    refresh_count = len(connection.base_cyclic.options)
    assert backend.stop_confirmed is True

    with pytest.raises(RuntimeError, match="Stop"):
        backend.current_pose()
    assert len(connection.base_cyclic.options) == refresh_count

    backend.begin_control()
    anchor = backend.current_pose()

    np.testing.assert_allclose(anchor.position, [1.0, 2.0, 3.0])
    assert len(connection.base_cyclic.options) == refresh_count + 1


def test_feedback_queued_before_stop_rechecks_admission_after_rpc_lock():
    connection = _Connection(_feedback())
    backend = _backend(connection)
    backend.begin_control()

    class GateLock:
        def __init__(self):
            self.attempted = threading.Event()
            self.permit = threading.Event()

        def acquire(self, timeout=None):
            self.attempted.set()
            return self.permit.wait(timeout=timeout)

        def release(self):
            pass

    gate = GateLock()
    backend._rpc_lock = gate
    errors = []
    feedback_thread = threading.Thread(
        target=lambda: _capture_error(backend.current_pose, errors)
    )
    feedback_thread.start()
    assert gate.attempted.wait(timeout=1.0)
    backend._request_stop(force=True)
    gate.permit.set()
    feedback_thread.join(timeout=1.0)

    assert not feedback_thread.is_alive()
    assert errors and "Stop" in str(errors[0])
    assert connection.base_cyclic.options == []


def test_old_stop_attempt_after_close_does_not_access_cleared_client():
    connection = _Connection(_feedback())
    backend = _backend(connection)
    backend.begin_control()
    old_token = backend._request_stop(force=True)
    backend.close()
    stop_count = connection.base.stop_count
    connection.base = None

    backend._attempt_stop(token=old_token, raise_on_failure=False)

    assert stop_count == 1


def test_close_never_joins_standard_thread_before_start_completes(monkeypatch):
    start_entered = threading.Event()
    allow_start = threading.Event()
    connection = _Connection(_feedback())

    class GatedStandardThread:
        def __init__(self, *, target, args=(), name, daemon):
            self.inner = threading.Thread(
                target=target,
                args=args,
                name=name,
                daemon=daemon,
            )

        def start(self):
            start_entered.set()
            assert allow_start.wait(timeout=1.0)
            self.inner.start()

        def join(self, timeout=None):
            # Standard Thread raises RuntimeError if join precedes start.
            self.inner.join(timeout=timeout)

    monkeypatch.setattr(
        kortex_backend_module,
        "_watchdog_thread_factory",
        GatedStandardThread,
    )
    backend = KortexBackend(connection)
    backend.begin_control()
    command_errors = []
    command = threading.Thread(
        target=lambda: _capture_error(
            lambda: backend.command_pose(_target(position=(0.01, 0.0, 0.0))),
            command_errors,
        )
    )
    command.start()
    assert start_entered.wait(timeout=1.0)

    close_errors = []
    closer = threading.Thread(
        target=lambda: _capture_error(backend.close, close_errors)
    )
    closer.start()
    allow_start.set()
    command.join(timeout=1.0)
    closer.join(timeout=1.0)

    assert not command.is_alive()
    assert not closer.is_alive()
    assert close_errors == []
    assert connection.closed == 1


def test_command_failure_after_close_token_cannot_override_close_confirmation():
    send_entered = threading.Event()
    release_send = threading.Event()
    failure_request_entered = threading.Event()
    allow_failure_request = threading.Event()
    connection_closed = threading.Event()
    connection = _Connection(_feedback())
    backend = _backend(connection)
    backend.begin_control()

    def failing_send(command, *, options=None):
        send_entered.set()
        assert release_send.wait(timeout=1.0)
        raise RuntimeError("send failed")

    connection.base.SendTwistCommand = failing_send
    original_request_stop = backend._request_stop

    def delayed_failure_request(*, force):
        failure_request_entered.set()
        assert allow_failure_request.wait(timeout=1.0)
        return original_request_stop(force=force)

    backend._request_stop = delayed_failure_request

    def successful_clearing_close():
        connection.closed += 1
        connection.base = None
        connection_closed.set()
        return True

    connection.close = successful_clearing_close
    command_errors = []
    command = threading.Thread(
        target=lambda: _capture_error(
            lambda: backend.command_pose(_target(position=(0.01, 0.0, 0.0))),
            command_errors,
        )
    )
    command.start()
    assert send_entered.wait(timeout=1.0)
    close_errors = []
    closer = threading.Thread(
        target=lambda: _capture_error(backend.close, close_errors)
    )
    closer.start()
    close_token_created = False
    for _ in range(100):
        with backend._state_lock:
            close_token_created = backend._closed
        if close_token_created:
            break
        threading.Event().wait(0.001)
    assert close_token_created

    release_send.set()
    assert failure_request_entered.wait(timeout=1.0)
    assert connection_closed.wait(timeout=1.0)
    allow_failure_request.set()
    command.join(timeout=1.0)
    closer.join(timeout=1.0)

    assert close_errors == []
    assert command_errors and "send failed" in str(command_errors[0])
    assert backend.stop_confirmed is True
    assert connection.closed == 1


def _capture_error(call, errors):
    try:
        call()
    except BaseException as error:
        errors.append(error)


def test_close_does_not_disconnect_during_blocked_feedback_and_can_retry():
    feedback_started = threading.Event()
    release_feedback = threading.Event()
    connection = _Connection(_feedback())
    backend = _backend(connection)
    backend.begin_control()
    backend.current_pose()

    def blocking_feedback(*, options=None):
        feedback_started.set()
        assert release_feedback.wait(timeout=1.0)
        return _feedback()

    connection.base_cyclic.RefreshFeedback = blocking_feedback
    command = threading.Thread(
        target=backend.command_pose,
        args=(_target(position=(0.01, 0.0, 0.0)),),
    )
    command.start()
    assert feedback_started.wait(timeout=1.0)
    close_errors = []
    closer = threading.Thread(
        target=lambda: _capture_error(backend.close, close_errors),
    )
    closer.start()
    closer.join(timeout=0.5)

    close_finished = not closer.is_alive()
    closed_while_blocked = connection.closed
    captured_errors = list(close_errors)
    release_feedback.set()
    command.join(timeout=1.0)
    closer.join(timeout=1.0)

    assert close_finished
    assert closed_while_blocked == 0
    assert captured_errors and "in-flight RPC" in str(captured_errors[0])

    backend.close()
    assert connection.closed == 1
