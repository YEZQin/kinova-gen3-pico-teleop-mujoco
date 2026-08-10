"""Tests for merged Kortex features: fault recovery and the gripper channel."""

from __future__ import annotations

import threading
from types import SimpleNamespace

import numpy as np
import pytest

import kinova_teleop.kortex_backend as kortex_backend_module
from kinova_teleop.kortex_backend import (
    GRIPPER_DEADBAND,
    GRIPPER_MIN_INTERVAL,
    KortexBackend,
    KortexSafetyError,
)
from kinova_teleop.pose_mapping import Pose
from kinova_teleop.workspace import AnchorEnvelope


TEST_ANCHOR_ENVELOPE = AnchorEnvelope((1.0, 1.0, 1.0), float(np.pi))


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


READY = 31
FAULT = 32

BASE_PB2 = SimpleNamespace(
    TwistCommand=_TwistCommand,
    ServoingModeInformation=_ServoingModeInformation,
    GripperCommand=_GripperCommand,
    CARTESIAN_REFERENCE_FRAME_BASE=17,
    SINGLE_LEVEL_SERVOING=23,
    GRIPPER_POSITION=3,
    ARMSTATE_SERVOING_READY=READY,
    ARMSTATE_IN_FAULT=FAULT,
)


class _Base:
    def __init__(self):
        self.sent = []
        self.stop_count = 0
        self.servo_modes = []
        self.arm_states = [READY]
        self.arm_state_reads = 0
        self.clear_faults_count = 0
        self.gripper_sent = []
        self.gripper_options = []

    def _current_state(self):
        if self.arm_state_reads < len(self.arm_states):
            return self.arm_states[self.arm_state_reads]
        return self.arm_states[-1]

    def GetArmState(self, *, options=None):
        state = self._current_state()
        self.arm_state_reads += 1
        return SimpleNamespace(active_state=state)

    def ClearFaults(self, *, options=None):
        self.clear_faults_count += 1

    def SetServoingMode(self, mode, *, options=None):
        self.servo_modes.append(mode.servoing_mode)

    def SendTwistCommand(self, command, *, options=None):
        self.sent.append(command)

    def SendGripperCommand(self, command, *, options=None):
        self.gripper_options.append(options)
        self.gripper_sent.append(command)

    def Stop(self, *, options=None):
        self.stop_count += 1


def _feedback():
    return SimpleNamespace(
        base=SimpleNamespace(
            tool_pose_x=0.0,
            tool_pose_y=0.0,
            tool_pose_z=0.0,
            tool_pose_theta_x=0.0,
            tool_pose_theta_y=0.0,
            tool_pose_theta_z=0.0,
        )
    )


class _Cyclic:
    def __init__(self):
        self.options = []

    def RefreshFeedback(self, *, options=None):
        self.options.append(options)
        return _feedback()


class _Connection:
    def __init__(self):
        self.base = _Base()
        self.base_cyclic = _Cyclic()
        self.base_pb2 = BASE_PB2
        self.closed = 0

    def rpc_options(self):
        return SimpleNamespace(timeout_ms=100)

    def close(self):
        self.base.Stop(options=self.rpc_options())
        self.closed += 1
        return True


class _ManualThread:
    def __init__(self, *, target, args=(), name, daemon):
        self.args = args

    def start(self):
        self.args[0].set()

    def join(self, timeout=None):
        pass


class _Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += float(seconds)


def _target(position=(0.0, 0.0, 0.0)):
    return Pose(
        np.array(position, dtype=np.float64),
        np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
    )


@pytest.fixture(autouse=True)
def _private_watchdog_thread_seam(monkeypatch):
    monkeypatch.setattr(
        kortex_backend_module,
        "_watchdog_thread_factory",
        _ManualThread,
    )


def _backend(connection, clock=None):
    clock = clock or _Clock()
    return KortexBackend(
        connection,
        anchor_envelope=TEST_ANCHOR_ENVELOPE,
        monotonic=clock,
        sleep=clock.sleep,
    )


# --- fault recovery -------------------------------------------------------


def test_init_skips_clear_faults_when_arm_is_ready():
    connection = _Connection()

    _backend(connection)

    # One read before SetServoingMode (fault check), one after (ready check).
    assert connection.base.clear_faults_count == 0
    assert connection.base.arm_state_reads == 2
    assert connection.base.servo_modes == [23]


def test_init_rejects_preexisting_fault_without_clearing():
    connection = _Connection()
    connection.base.arm_states = [FAULT, FAULT, READY]

    with pytest.raises(KortexSafetyError, match="startup fault"):
        _backend(connection)

    assert connection.base.clear_faults_count == 0
    assert connection.base.arm_state_reads == 1
    assert connection.base.servo_modes == []


def test_init_rejects_string_fault_state_without_clearing():
    connection = _Connection()
    connection.base.arm_states = ["ARMSTATE_IN_FAULT"]

    with pytest.raises(KortexSafetyError, match="startup fault"):
        _backend(connection)

    assert connection.base.clear_faults_count == 0
    assert connection.base.arm_state_reads == 1
    assert connection.base.servo_modes == []


def test_init_accepts_string_ready_state():
    connection = _Connection()
    connection.base.arm_states = ["ARMSTATE_SERVOING_READY"]

    _backend(connection)

    assert connection.base.clear_faults_count == 0
    assert connection.base.arm_state_reads == 2
    assert connection.base.servo_modes == [23]


def test_init_fails_safely_when_fault_never_clears():
    connection = _Connection()
    connection.base.arm_states = [FAULT]
    clock = _Clock()

    with pytest.raises(KortexSafetyError, match="startup fault"):
        _backend(connection, clock)

    # The failed initialization must still attempt a Stop.
    assert connection.base.clear_faults_count == 0
    assert connection.base.stop_count == 1
    assert connection.base.servo_modes == []


def test_init_waits_for_transient_non_fault_state_without_clearing():
    connection = _Connection()
    connection.base.arm_states = [99, 99, READY]

    _backend(connection)

    assert connection.base.clear_faults_count == 0
    assert connection.base.arm_state_reads == 3


def test_init_recovers_leftover_low_level_servoing_mode():
    """SetServoingMode itself is the recovery for a stale non-fault mode."""

    LOW_LEVEL = 6
    connection = _Connection()
    base = connection.base
    base.arm_states = [LOW_LEVEL]
    original_set_mode = base.SetServoingMode

    def set_mode_recovers(mode, *, options=None):
        original_set_mode(mode, options=options)
        base.arm_states = [READY]

    base.SetServoingMode = set_mode_recovers

    _backend(connection)

    assert base.clear_faults_count == 0
    assert base.servo_modes == [23]
    assert base.stop_count == 0


def test_running_fault_is_not_cleared_automatically():
    """A fault after init only surfaces as a failed RPC; no ClearFaults."""

    connection = _Connection()
    backend = _backend(connection)
    backend.begin_control()
    backend.current_pose()

    def failing_send(command, *, options=None):
        raise RuntimeError("robot is in fault")

    connection.base.SendTwistCommand = failing_send

    with pytest.raises(RuntimeError, match="in fault"):
        backend.command_pose(_target(position=(0.01, 0.0, 0.0)))

    assert connection.base.clear_faults_count == 0
    assert connection.base.stop_count == 1


def test_confirm_stationary_requires_fresh_low_velocity_feedback():
    connection = _Connection()
    backend = _backend(connection)
    backend.begin_control()
    backend.current_pose()

    feedback = SimpleNamespace(
        tool_twist_linear_x=0.0001,
        tool_twist_linear_y=0.0,
        tool_twist_linear_z=0.0,
        tool_twist_angular_x=0.1,
        tool_twist_angular_y=0.0,
        tool_twist_angular_z=0.0,
    )
    connection.base_cyclic.RefreshFeedback = lambda *, options=None: SimpleNamespace(base=feedback)
    assert backend.confirm_stationary() is True

    feedback.tool_twist_linear_x = 0.01
    assert backend.confirm_stationary() is False


# --- gripper channel ------------------------------------------------------


def test_gripper_sends_position_mode_command():
    connection = _Connection()
    backend = _backend(connection)
    backend.begin_control()

    assert backend.command_gripper(0.5) is True

    command = connection.base.gripper_sent[-1]
    assert command.mode == 3
    fingers = command.gripper.finger.entries
    assert len(fingers) == 1
    assert fingers[0].finger_identifier == 1
    assert fingers[0].value == 0.5
    assert connection.base.gripper_options[-1].timeout_ms < 200


def test_gripper_is_rejected_before_begin_control():
    connection = _Connection()
    backend = _backend(connection)

    assert backend.command_gripper(0.5) is False
    assert connection.base.gripper_sent == []


def test_gripper_is_rejected_after_stop():
    connection = _Connection()
    backend = _backend(connection)
    backend.begin_control()
    backend.hold()

    assert backend.command_gripper(0.5) is False
    assert connection.base.gripper_sent == []


def test_gripper_deadband_and_rate_limit():
    connection = _Connection()
    clock = _Clock()
    backend = _backend(connection, clock)
    backend.begin_control()

    assert backend.command_gripper(0.5) is True
    # Outside the deadband but inside the minimum interval since the last
    # sent command: rate limited.
    clock.now += GRIPPER_MIN_INTERVAL / 2
    assert backend.command_gripper(0.9) is False
    # Within the deadband: deduplicated even after the interval passes.
    clock.now += GRIPPER_MIN_INTERVAL
    assert backend.command_gripper(0.5 + GRIPPER_DEADBAND / 2) is False
    # Outside both: sent.
    assert backend.command_gripper(0.9) is True

    assert len(connection.base.gripper_sent) == 2


def test_gripper_value_is_clamped_to_unit_interval():
    connection = _Connection()
    clock = _Clock()
    backend = _backend(connection, clock)
    backend.begin_control()

    assert backend.command_gripper(1.5) is True
    assert connection.base.gripper_sent[-1].gripper.finger.entries[0].value == 1.0

    clock.now += GRIPPER_MIN_INTERVAL + 0.01
    assert backend.command_gripper(-0.5) is True
    assert connection.base.gripper_sent[-1].gripper.finger.entries[0].value == 0.0


def test_gripper_nonfinite_value_attempts_stop():
    connection = _Connection()
    backend = _backend(connection)
    backend.begin_control()

    with pytest.raises(ValueError, match="finite"):
        backend.command_gripper(float("nan"))

    assert connection.base.stop_count == 1
    assert backend.stop_requested is True


def test_gripper_send_failure_attempts_stop():
    connection = _Connection()
    backend = _backend(connection)
    backend.begin_control()

    def failing_gripper(command, *, options=None):
        raise RuntimeError("gripper offline")

    connection.base.SendGripperCommand = failing_gripper

    with pytest.raises(RuntimeError, match="gripper offline"):
        backend.command_gripper(0.5)

    assert connection.base.stop_count == 1
    assert backend.stop_requested is True


def test_gripper_command_does_not_arm_twist_watchdog():
    connection = _Connection()
    clock = _Clock()
    backend = _backend(connection, clock)
    backend.begin_control()

    backend.command_gripper(1.0)
    clock.now = 10.0
    backend.check_watchdog()

    assert connection.base.stop_count == 0


def test_gripper_after_rearm_is_accepted_again():
    connection = _Connection()
    clock = _Clock()
    backend = _backend(connection, clock)
    backend.begin_control()
    assert backend.command_gripper(0.5) is True
    backend.hold()
    assert backend.command_gripper(0.9) is False

    backend.begin_control()
    backend.current_pose()
    clock.now += GRIPPER_MIN_INTERVAL + 0.01
    assert backend.command_gripper(0.9) is True


# --- concurrency on the new paths ------------------------------------------


def _capture(call, results, errors):
    try:
        results.append(call())
    except BaseException as error:
        errors.append(error)


def test_first_nonzero_twist_is_watchdog_covered_during_send():
    """The deadline must be armed before SendTwistCommand, not after."""

    clock = _Clock()
    connection = _Connection()
    backend = _backend(connection, clock)
    backend.begin_control()
    backend.current_pose()

    send_started = threading.Event()
    release_send = threading.Event()
    original_send = connection.base.SendTwistCommand

    def blocking_send(command, *, options=None):
        send_started.set()
        assert release_send.wait(timeout=2.0)
        original_send(command, options=options)

    connection.base.SendTwistCommand = blocking_send
    results, errors = [], []
    worker = threading.Thread(
        target=_capture,
        args=(
            lambda: backend.command_pose(_target(position=(0.01, 0.0, 0.0))),
            results,
            errors,
        ),
    )
    worker.start()
    assert send_started.wait(timeout=2.0)

    # The control thread is stalled inside the send. The watchdog must
    # already see an armed deadline and latch a stop request.
    clock.now = 0.21
    backend.check_watchdog()
    assert backend.stop_requested is True

    release_send.set()
    worker.join(timeout=2.0)
    assert not worker.is_alive()
    assert errors == []
    assert results and results[0].accepted is False

    backend.check_watchdog()
    assert backend.stop_confirmed is True
    assert connection.base.stop_count == 1


class _GateLock:
    """Lock stand-in that parks the first acquire until released by the test."""

    def __init__(self):
        self.attempted = threading.Event()
        self.permit = threading.Event()

    def acquire(self, timeout=None):
        self.attempted.set()
        return self.permit.wait(timeout=timeout)

    def release(self):
        pass


def test_gripper_queued_behind_stop_is_dropped():
    """A stop landing while the gripper RPC waits for the lock must win."""

    connection = _Connection()
    backend = _backend(connection)
    backend.begin_control()

    gate = _GateLock()
    backend._rpc_lock = gate
    results, errors = [], []
    worker = threading.Thread(
        target=_capture,
        args=(lambda: backend.command_gripper(0.5), results, errors),
    )
    worker.start()
    assert gate.attempted.wait(timeout=2.0)

    backend._request_stop(force=True)
    gate.permit.set()
    worker.join(timeout=2.0)

    assert not worker.is_alive()
    assert errors == []
    assert results == [False]
    assert connection.base.gripper_sent == []


class _NeverLock:
    def acquire(self, timeout=None):
        return False

    def release(self):
        pass


def test_gripper_rpc_lock_timeout_attempts_stop():
    connection = _Connection()
    backend = _backend(connection)
    backend.begin_control()
    backend._rpc_lock = _NeverLock()

    with pytest.raises(KortexSafetyError, match="Timed out"):
        backend.command_gripper(0.5)

    assert backend.stop_requested is True
    assert backend.stop_confirmed is False
    assert connection.base.gripper_sent == []
