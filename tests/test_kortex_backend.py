from __future__ import annotations

import json
import math
import threading
from types import SimpleNamespace

import numpy as np
import pytest

import kinova_teleop.kortex_backend as kortex_backend_module
from kinova_teleop.evidence_log import EvidenceLogger, logger_event_sink
from kinova_teleop.kortex_backend import KortexBackend, KortexSafetyError
from kinova_teleop.pose_mapping import Pose
from kinova_teleop.workspace import AnchorEnvelope, WorkspaceLimits


TEST_ANCHOR_ENVELOPE = AnchorEnvelope((1.0, 1.0, 1.0), math.pi)
FIRST_TRIAL_ANCHOR_ENVELOPE = AnchorEnvelope(
    (0.02, 0.02, 0.02),
    math.radians(5.0),
)


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
    return KortexBackend(
        connection,
        anchor_envelope=kwargs.pop("anchor_envelope", TEST_ANCHOR_ENVELOPE),
        **kwargs,
    )


def _begin_pose_control(backend):
    backend.begin_control()
    return backend.current_pose()


def _active_gripper_backend():
    connection = _Connection(_feedback())
    clock = _Clock()
    backend = _backend(connection, monotonic=clock)
    backend.begin_control()
    backend.current_pose()
    return backend, connection, clock


def test_gripper_sends_position_mode_command():
    connection = _Connection(_feedback())
    backend = _backend(connection)
    backend.begin_control()

    assert backend.command_gripper(0.5) is True

    command = connection.base.gripper_sent[-1]
    assert command.mode == BASE_PB2.GRIPPER_POSITION
    fingers = command.gripper.finger.entries
    assert len(fingers) == 1
    assert fingers[0].finger_identifier == 1
    assert fingers[0].value == 0.5
    assert connection.base.gripper_options[-1].timeout_ms < 200


def test_gripper_is_rejected_before_begin_control():
    connection = _Connection(_feedback())
    backend = _backend(connection)

    assert backend.command_gripper(0.5) is False
    assert connection.base.gripper_sent == []


def test_gripper_deadband_and_rate_limit():
    connection = _Connection(_feedback())
    clock = _Clock()
    backend = _backend(connection, monotonic=clock)
    backend.begin_control()

    assert backend.command_gripper(0.5) is True
    clock.now += kortex_backend_module.GRIPPER_MIN_INTERVAL / 2
    assert backend.command_gripper(0.9) is True
    clock.now += kortex_backend_module.GRIPPER_MIN_INTERVAL
    assert (
        backend.command_gripper(
            0.5 + kortex_backend_module.GRIPPER_DEADBAND / 2
        )
        is True
    )
    assert backend.command_gripper(0.9) is True

    assert len(connection.base.gripper_sent) == 2


def test_gripper_value_is_clamped_to_unit_interval():
    connection = _Connection(_feedback())
    clock = _Clock()
    backend = _backend(connection, monotonic=clock)
    backend.begin_control()

    assert backend.command_gripper(1.5) is True
    assert connection.base.gripper_sent[-1].gripper.finger.entries[0].value == 1.0

    clock.now += kortex_backend_module.GRIPPER_MIN_INTERVAL + 0.01
    assert backend.command_gripper(-0.5) is True
    assert connection.base.gripper_sent[-1].gripper.finger.entries[0].value == 0.0


def test_gripper_nonfinite_value_attempts_stop():
    connection = _Connection(_feedback())
    backend = _backend(connection)
    backend.begin_control()

    with pytest.raises(KortexSafetyError, match="finite"):
        backend.command_gripper(float("nan"))

    assert connection.base.stop_count == 1
    assert backend.stop_requested is True
    assert backend.stop_confirmed is True
    assert backend.fault_reason == "Gripper position must be finite"


def test_gripper_send_failure_attempts_stop():
    connection = _Connection(_feedback())
    backend = _backend(connection)
    backend.begin_control()

    def failing_gripper(command, *, options=None):
        raise RuntimeError("gripper offline")

    connection.base.SendGripperCommand = failing_gripper

    with pytest.raises(KortexSafetyError, match="gripper offline") as captured:
        backend.command_gripper(0.5)

    assert connection.base.stop_count == 1
    assert backend.stop_requested is True
    assert backend.stop_confirmed is True
    assert backend.fault_reason == "gripper offline"
    assert isinstance(captured.value.__cause__, RuntimeError)


def test_gripper_command_does_not_arm_twist_watchdog():
    connection = _Connection(_feedback())
    clock = _Clock()
    backend = _backend(connection, monotonic=clock)
    backend.begin_control()

    assert backend.command_gripper(1.0) is True
    clock.now = 10.0
    backend.check_watchdog()

    assert connection.base.stop_count == 0


class _GateLock:
    def __init__(self):
        self.attempted = threading.Event()
        self.permit = threading.Event()

    def acquire(self, timeout=None):
        self.attempted.set()
        return self.permit.wait(timeout=timeout)

    def release(self):
        pass


class _ObservedRpcLock:
    def __init__(self):
        self._lock = threading.Lock()
        self._count_lock = threading.Lock()
        self.acquire_count = 0
        self.second_attempted = threading.Event()

    def acquire(self, timeout=None):
        with self._count_lock:
            self.acquire_count += 1
            if self.acquire_count == 2:
                self.second_attempted.set()
        if timeout is None:
            return self._lock.acquire()
        return self._lock.acquire(timeout=timeout)

    def release(self):
        self._lock.release()


class _FailureHandoffLock:
    """Make the first releaser wait until the queued writer completes."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._count_lock = threading.Lock()
        self._owner_ordinals: dict[int, int] = {}
        self.acquire_count = 0
        self.second_attempted = threading.Event()
        self.second_acquired = threading.Event()
        self.second_completed = threading.Event()

    def acquire(self, timeout=None):
        with self._count_lock:
            self.acquire_count += 1
            ordinal = self.acquire_count
            if ordinal == 2:
                self.second_attempted.set()
        acquired = (
            self._lock.acquire()
            if timeout is None
            else self._lock.acquire(timeout=timeout)
        )
        if acquired:
            self._owner_ordinals[threading.get_ident()] = ordinal
            if ordinal == 2:
                self.second_acquired.set()
        return acquired

    def release(self):
        ordinal = self._owner_ordinals.pop(threading.get_ident())
        self._lock.release()
        if ordinal == 1:
            assert self.second_acquired.wait(timeout=1.0)
            assert self.second_completed.wait(timeout=1.0)
        elif ordinal == 2:
            self.second_completed.set()


def test_gripper_queued_behind_stop_is_dropped():
    connection = _Connection(_feedback())
    backend = _backend(connection)
    backend.begin_control()

    gate = _GateLock()
    backend._rpc_lock = gate
    results: list[bool] = []
    errors: list[BaseException] = []
    worker = threading.Thread(
        target=_capture_result_or_error,
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


def test_failed_gripper_revokes_queued_twist_before_releasing_rpc_lock() -> None:
    """Moving the fault latch after RPC unlock must allow the queued Twist."""

    backend, connection, _clock = _active_gripper_backend()
    generation = backend._generation
    backend.current_pose = lambda: _target()
    handoff = _FailureHandoffLock()
    backend._rpc_lock = handoff
    send_entered = threading.Event()

    def failing_gripper(_command, *, options=None):
        send_entered.set()
        assert handoff.second_attempted.wait(timeout=1.0)
        raise RuntimeError("gripper offline")

    connection.base.SendGripperCommand = failing_gripper
    gripper_results: list[bool] = []
    gripper_errors: list[BaseException] = []
    twist_results: list[object] = []
    twist_errors: list[BaseException] = []
    gripper = threading.Thread(
        target=_capture_result_or_error,
        args=(lambda: backend.command_gripper(0.5), gripper_results, gripper_errors),
    )
    twist = threading.Thread(
        target=_capture_result_or_error,
        args=(
            lambda: backend.command_pose(_target(position=(0.01, 0.0, 0.0))),
            twist_results,
            twist_errors,
        ),
    )

    gripper.start()
    assert send_entered.wait(timeout=1.0)
    twist.start()
    gripper.join(timeout=2.0)
    twist.join(timeout=2.0)

    assert not gripper.is_alive()
    assert not twist.is_alive()
    assert len(gripper_errors) == 1
    assert "gripper offline" in str(gripper_errors[0])
    assert twist_errors == []
    assert len(twist_results) == 1
    assert twist_results[0].accepted is False
    assert connection.base.sent == []
    assert connection.base.stop_count == 1
    assert backend._generation > generation


def test_failed_twist_revokes_queued_gripper_before_releasing_rpc_lock() -> None:
    """Moving the fault latch after RPC unlock must allow the queued gripper."""

    backend, connection, _clock = _active_gripper_backend()
    generation = backend._generation
    backend.current_pose = lambda: _target()
    handoff = _FailureHandoffLock()
    backend._rpc_lock = handoff
    send_entered = threading.Event()

    def failing_twist(_command, *, options=None):
        send_entered.set()
        assert handoff.second_attempted.wait(timeout=1.0)
        raise RuntimeError("twist offline")

    connection.base.SendTwistCommand = failing_twist
    twist_results: list[object] = []
    twist_errors: list[BaseException] = []
    gripper_results: list[bool] = []
    gripper_errors: list[BaseException] = []
    twist = threading.Thread(
        target=_capture_result_or_error,
        args=(
            lambda: backend.command_pose(_target(position=(0.01, 0.0, 0.0))),
            twist_results,
            twist_errors,
        ),
    )
    gripper = threading.Thread(
        target=_capture_result_or_error,
        args=(lambda: backend.command_gripper(0.5), gripper_results, gripper_errors),
    )

    twist.start()
    assert send_entered.wait(timeout=1.0)
    gripper.start()
    twist.join(timeout=2.0)
    gripper.join(timeout=2.0)

    assert not twist.is_alive()
    assert not gripper.is_alive()
    assert len(twist_errors) == 1
    assert "twist offline" in str(twist_errors[0])
    assert gripper_errors == []
    assert gripper_results == [False]
    assert connection.base.gripper_sent == []
    assert connection.base.stop_count == 1
    assert backend._generation > generation


def test_concurrent_gripper_commands_recheck_rate_limit_after_rpc_lock() -> None:
    backend, connection, _clock = _active_gripper_backend()
    rpc_lock = _ObservedRpcLock()
    backend._rpc_lock = rpc_lock
    send_entered = threading.Event()
    release_send = threading.Event()
    original_send = connection.base.SendGripperCommand

    def blocked_send(command, *, options=None):
        send_entered.set()
        assert release_send.wait(timeout=1.0)
        original_send(command, options=options)

    connection.base.SendGripperCommand = blocked_send
    first_results: list[bool] = []
    first_errors: list[BaseException] = []
    second_results: list[bool] = []
    second_errors: list[BaseException] = []
    first = threading.Thread(
        target=_capture_result_or_error,
        args=(lambda: backend.command_gripper(0.4), first_results, first_errors),
    )
    second = threading.Thread(
        target=_capture_result_or_error,
        args=(lambda: backend.command_gripper(0.9), second_results, second_errors),
    )

    first.start()
    assert send_entered.wait(timeout=1.0)
    second.start()
    assert rpc_lock.second_attempted.wait(timeout=1.0)
    release_send.set()
    first.join(timeout=1.0)
    second.join(timeout=1.0)

    assert not first.is_alive()
    assert not second.is_alive()
    assert first_errors == []
    assert second_errors == []
    assert first_results == [True]
    assert second_results == [True]
    assert [
        command.gripper.finger.entries[0].value
        for command in connection.base.gripper_sent
    ] == [0.4]


def test_gripper_rate_limit_starts_after_slow_rpc_succeeds() -> None:
    backend, connection, clock = _active_gripper_backend()
    send_entered = threading.Event()
    release_send = threading.Event()
    original_send = connection.base.SendGripperCommand

    def blocked_send(command, *, options=None):
        send_entered.set()
        assert release_send.wait(timeout=1.0)
        original_send(command, options=options)

    connection.base.SendGripperCommand = blocked_send
    results: list[bool] = []
    errors: list[BaseException] = []
    worker = threading.Thread(
        target=_capture_result_or_error,
        args=(lambda: backend.command_gripper(0.4), results, errors),
    )
    worker.start()
    assert send_entered.wait(timeout=1.0)
    clock.now = 5.0
    release_send.set()
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert errors == []
    assert results == [True]
    assert backend.command_gripper(0.9) is True
    assert len(connection.base.gripper_sent) == 1
    clock.now += 0.099
    assert backend.command_gripper(0.9) is True
    assert len(connection.base.gripper_sent) == 1
    clock.now += 0.002
    assert backend.command_gripper(0.9) is True
    assert [
        command.gripper.finger.entries[0].value
        for command in connection.base.gripper_sent
    ] == [0.4, 0.9]


def test_gripper_is_rejected_after_stop_and_accepted_after_rearm() -> None:
    backend, connection, clock = _active_gripper_backend()
    assert backend.command_gripper(0.5) is True
    backend.hold()
    assert backend.command_gripper(0.9) is False
    backend.begin_control()
    backend.current_pose()
    clock.now += 0.11
    assert backend.command_gripper(0.9) is True
    assert len(connection.base.gripper_sent) == 2


def test_gripper_rpc_lock_timeout_latches_stop_failure() -> None:
    backend, connection, _clock = _active_gripper_backend()
    backend._rpc_lock.acquire()
    try:
        with pytest.raises(KortexSafetyError, match="Stop attempted but unconfirmed"):
            backend.command_gripper(0.5)
    finally:
        backend._rpc_lock.release()
    assert connection.base.gripper_sent == []
    assert backend.stop_requested is True
    assert backend.stop_confirmed is False
    assert backend.fault_reason == "Stop attempted but unconfirmed"


def test_close_never_sends_gripper_open() -> None:
    backend, connection, _clock = _active_gripper_backend()
    backend.command_gripper(0.7)
    backend.close()
    assert [
        command.gripper.finger.entries[0].value
        for command in connection.base.gripper_sent
    ] == [0.7]


def test_advanced_workspace_projection_clamps_target_without_stop_or_reanchor(
    tmp_path,
):
    connection = _Connection(_feedback(position=(0.0, 0.0, 0.3)))
    events_path = tmp_path / "events.jsonl"
    logger = EvidenceLogger(events_path, run_id="g-001", monotonic_ns=lambda: 10)
    backend = _backend(
        connection,
        anchor_envelope=None,
        advanced_translation=True,
        workspace_limits=WorkspaceLimits((-0.2, -0.2, 0.1), (0.2, 0.2, 0.6)),
        max_linear_speed=0.05,
        event_sink=logger_event_sink(logger),
    )
    backend.begin_control()
    backend.current_pose()

    result = backend.command_pose(_target(position=(1.0, 1.0, 1.0)))

    assert result.accepted is True
    assert result.reanchor_required is False
    assert result.reason == "target clamped to workspace"
    assert getattr(result, "active_rebase_target", None) is not None
    np.testing.assert_allclose(result.active_rebase_target.position, [0.2, 0.2, 0.6])
    assert connection.base.stop_count == 0
    velocity = _velocity(connection.base.sent[-1])
    expected_direction = np.array([0.2, 0.2, 0.3], dtype=float)
    expected_linear = expected_direction * (0.05 / np.linalg.norm(expected_direction))
    np.testing.assert_allclose(velocity[:3], expected_linear)
    np.testing.assert_allclose(velocity[3:], [0.0, 0.0, 0.0])
    records = [json.loads(line) for line in events_path.read_text().splitlines()]
    assert records[-2]["kind"] == "workspace_clamped"
    assert records[-2]["state"] == "MOVING"
    assert records[-2]["payload"] == {"reason": "target clamped to workspace"}


def test_advanced_workspace_projection_resumes_inward_in_same_generation():
    connection = _Connection(_feedback(position=(0.0, 0.0, 0.3)))
    backend = _backend(
        connection,
        anchor_envelope=None,
        advanced_translation=True,
        workspace_limits=WorkspaceLimits((-0.2, -0.2, 0.1), (0.2, 0.2, 0.6)),
        max_linear_speed=0.05,
    )
    _begin_pose_control(backend)
    generation = backend._generation

    clamped = backend.command_pose(_target(position=(1.0, 0.0, 0.3)))
    inward = backend.command_pose(_target(position=(0.1, 0.0, 0.3)))

    assert clamped.accepted is True
    assert clamped.reason == "target clamped to workspace"
    assert inward.accepted is True
    assert inward.reason == ""
    assert backend._generation == generation
    assert connection.base.stop_count == 0
    assert len(connection.base.sent) == 2
    np.testing.assert_allclose(_velocity(connection.base.sent[-1])[:3], [0.05, 0, 0])


def test_translation_policy_constructor_invariants():
    limits = WorkspaceLimits((-0.2, -0.2, 0.1), (0.2, 0.2, 0.6))

    with pytest.raises(ValueError, match="legacy translation requires anchor_envelope"):
        KortexBackend(_Connection(_feedback()), anchor_envelope=None)
    with pytest.raises(ValueError, match="advanced translation requires workspace_limits"):
        KortexBackend(
            _Connection(_feedback()),
            anchor_envelope=None,
            advanced_translation=True,
        )
    with pytest.raises(ValueError, match="advanced translation requires anchor_envelope=None"):
        KortexBackend(
            _Connection(_feedback()),
            anchor_envelope=TEST_ANCHOR_ENVELOPE,
            advanced_translation=True,
            workspace_limits=limits,
        )
    with pytest.raises(TypeError, match="advanced_translation must be a bool"):
        KortexBackend(
            _Connection(_feedback()),
            anchor_envelope=TEST_ANCHOR_ENVELOPE,
            advanced_translation=1,
        )


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


def test_workspace_rejection_stops_before_feedback_or_twist():
    connection = _Connection(_feedback((0.0, 0.0, 0.3)))
    backend = _backend(
        connection,
        workspace_limits=WorkspaceLimits((-0.2, -0.2, 0.1), (0.2, 0.2, 0.6)),
    )
    _begin_pose_control(backend)
    feedback_count = len(connection.base_cyclic.options)
    result = backend.command_pose(_target(position=(1.0, 0.0, 0.3)))

    assert result.accepted is False
    assert result.reanchor_required is True
    assert result.reason == "target outside workspace"
    assert len(connection.base_cyclic.options) == feedback_count
    assert connection.base.sent == []
    assert connection.base.stop_count == 1
    assert backend.stop_confirmed is True
    assert backend.fault_reason is None


def test_workspace_rejection_reports_unconfirmed_stop_failure():
    connection = _Connection(_feedback((0.0, 0.0, 0.3)))
    backend = _backend(
        connection,
        workspace_limits=WorkspaceLimits((-0.2, -0.2, 0.1), (0.2, 0.2, 0.6)),
    )
    _begin_pose_control(backend)

    def failed_stop(*, options=None):
        raise RuntimeError("stop timeout")

    connection.base.Stop = failed_stop
    with pytest.raises(KortexSafetyError, match="Stop attempted but unconfirmed"):
        backend.command_pose(_target(position=(1.0, 0.0, 0.3)))

    assert connection.base.sent == []
    assert backend.fault_reason == "Stop attempted but unconfirmed"
    assert backend.stop_confirmed is False


def test_stale_workspace_rejection_cannot_fault_new_rearmed_generation(
    monkeypatch,
):
    evaluation_started = threading.Event()
    release_evaluation = threading.Event()
    connection = _Connection(_feedback((0.0, 0.0, 0.3)))
    backend = _backend(
        connection,
        workspace_limits=WorkspaceLimits((-0.2, -0.2, 0.1), (0.2, 0.2, 0.6)),
    )
    _begin_pose_control(backend)
    real_validate = kortex_backend_module.validate_target_pose

    def blocking_validate(target, limits):
        evaluation_started.set()
        assert release_evaluation.wait(timeout=1.0)
        return real_validate(target, limits)

    monkeypatch.setattr(
        kortex_backend_module,
        "validate_target_pose",
        blocking_validate,
    )
    results = []
    errors = []
    stale_command = threading.Thread(
        target=lambda: _capture_result_or_error(
            lambda: backend.command_pose(_target(position=(1.0, 0.0, 0.3))),
            results,
            errors,
        )
    )
    stale_command.start()
    assert evaluation_started.wait(timeout=1.0)

    backend.hold()
    _begin_pose_control(backend)
    release_evaluation.set()
    stale_command.join(timeout=1.0)

    assert not stale_command.is_alive()
    assert errors == []
    assert len(results) == 1
    assert results[0].accepted is False
    assert backend.fault_reason is None
    assert connection.base.stop_count == 1


def test_command_without_rearm_anchor_faults_before_feedback_or_twist():
    connection = _Connection(_feedback())
    backend = _backend(connection, anchor_envelope=FIRST_TRIAL_ANCHOR_ENVELOPE)
    backend.begin_control()

    with pytest.raises(KortexSafetyError, match="control anchor is unavailable"):
        backend.command_pose(_target(position=(0.01, 0.0, 0.0)))

    assert connection.base_cyclic.options == []
    assert connection.base.sent == []
    assert connection.base.stop_count == 1


def test_anchor_translation_rejection_stops_recoverably_before_feedback():
    connection = _Connection(_feedback((0.1, -0.2, 0.3)))
    backend = _backend(connection, anchor_envelope=FIRST_TRIAL_ANCHOR_ENVELOPE)
    _begin_pose_control(backend)
    feedback_count = len(connection.base_cyclic.options)

    result = backend.command_pose(_target(position=(0.120001, -0.2, 0.3)))

    assert result.accepted is False
    assert result.reanchor_required is True
    assert result.reason == "target outside anchor translation envelope"
    assert len(connection.base_cyclic.options) == feedback_count
    assert connection.base.sent == []
    assert connection.base.stop_count == 1
    assert backend.stop_confirmed is True
    assert backend.fault_reason is None


def test_anchor_rejection_is_written_to_evidence_logger(tmp_path):
    connection = _Connection(_feedback())
    events_path = tmp_path / "events.jsonl"
    logger = EvidenceLogger(events_path, run_id="g-001", monotonic_ns=lambda: 10)
    backend = _backend(
        connection,
        anchor_envelope=FIRST_TRIAL_ANCHOR_ENVELOPE,
        event_sink=logger_event_sink(logger),
    )
    _begin_pose_control(backend)

    result = backend.command_pose(_target(position=(0.020001, 0.0, 0.0)))

    records = [json.loads(line) for line in events_path.read_text().splitlines()]
    assert result.reanchor_required is True
    assert records[-2]["kind"] == "anchor_rejected"
    assert records[-2]["state"] == "STOPPING"
    assert records[-2]["payload"] == {
        "reason": "target outside anchor translation envelope"
    }


def test_anchor_rejection_raises_unconfirmed_stop_failure():
    connection = _Connection(_feedback())
    backend = _backend(connection, anchor_envelope=FIRST_TRIAL_ANCHOR_ENVELOPE)
    _begin_pose_control(backend)

    def failed_stop(*, options=None):
        raise RuntimeError("stop timeout")

    connection.base.Stop = failed_stop

    with pytest.raises(KortexSafetyError, match="Stop attempted but unconfirmed"):
        backend.command_pose(_target(position=(0.020001, 0.0, 0.0)))

    assert backend.fault_reason == "Stop attempted but unconfirmed"
    assert backend.stop_confirmed is False


def test_repeated_anchor_fault_latch_does_not_create_another_stop_attempt():
    connection = _Connection(_feedback())
    backend = _backend(connection, anchor_envelope=FIRST_TRIAL_ANCHOR_ENVELOPE)
    backend.begin_control()

    first_reason = backend._latch_fault_and_stop(
        "target outside anchor translation envelope"
    )
    repeated_reason = backend._latch_fault_and_stop(
        "target outside anchor rotation envelope"
    )

    assert connection.base.stop_count == 1
    assert backend._fault_reason == "target outside anchor translation envelope"
    assert first_reason == backend._fault_reason
    assert repeated_reason == backend._fault_reason


def test_stale_anchor_rejection_cannot_fault_new_rearmed_generation():
    evaluation_started = threading.Event()
    release_evaluation = threading.Event()
    connection = _Connection(_feedback())
    backend = _backend(connection, anchor_envelope=FIRST_TRIAL_ANCHOR_ENVELOPE)
    _begin_pose_control(backend)

    class BlockingEnvelope:
        def evaluate(self, anchor, target):
            evaluation_started.set()
            assert release_evaluation.wait(timeout=1.0)
            return FIRST_TRIAL_ANCHOR_ENVELOPE.evaluate(anchor, target)

    backend.anchor_envelope = BlockingEnvelope()
    results = []
    errors = []

    def issue_stale_command():
        try:
            results.append(
                backend.command_pose(_target(position=(0.020001, 0.0, 0.0)))
            )
        except BaseException as error:
            errors.append(error)

    stale_command = threading.Thread(target=issue_stale_command)
    stale_command.start()
    assert evaluation_started.wait(timeout=1.0)

    backend.hold()
    connection.base_cyclic.feedback = _feedback((0.1, 0.0, 0.0))
    _begin_pose_control(backend)
    release_evaluation.set()
    stale_command.join(timeout=1.0)

    assert not stale_command.is_alive()
    assert errors == []
    assert len(results) == 1
    assert results[0].accepted is False
    assert backend._fault_reason is None
    assert connection.base.stop_count == 1


def test_anchor_rotation_rejection_stops_before_feedback_or_twist():
    connection = _Connection(_feedback())
    backend = _backend(connection, anchor_envelope=FIRST_TRIAL_ANCHOR_ENVELOPE)
    _begin_pose_control(backend)
    outside = (
        math.cos(math.radians(5.01) / 2.0),
        0.0,
        0.0,
        math.sin(math.radians(5.01) / 2.0),
    )
    feedback_count = len(connection.base_cyclic.options)

    result = backend.command_pose(_target(quaternion=outside))

    assert result.accepted is False
    assert result.reanchor_required is True
    assert result.reason == "target outside anchor rotation envelope"
    assert len(connection.base_cyclic.options) == feedback_count
    assert connection.base.sent == []
    assert connection.base.stop_count == 1


def test_control_anchor_is_an_immutable_copy_of_rearm_feedback():
    connection = _Connection(_feedback())
    backend = _backend(connection, anchor_envelope=FIRST_TRIAL_ANCHOR_ENVELOPE)
    returned = _begin_pose_control(backend)
    returned.position[0] = 0.020001
    returned.quaternion[:] = [0.0, 0.0, 0.0, 1.0]

    result = backend.command_pose(_target(position=(0.020001, 0.0, 0.0)))

    assert result.reanchor_required is True
    assert connection.base.sent == []
    assert connection.base.stop_count == 1


def test_ordinary_feedback_does_not_refresh_control_anchor():
    connection = _Connection(_feedback())
    backend = _backend(connection, anchor_envelope=FIRST_TRIAL_ANCHOR_ENVELOPE)
    _begin_pose_control(backend)
    connection.base_cyclic.feedback = _feedback((0.1, 0.0, 0.0))
    backend.current_pose()
    feedback_count = len(connection.base_cyclic.options)

    result = backend.command_pose(_target(position=(0.1, 0.0, 0.0)))

    assert result.reanchor_required is True
    assert len(connection.base_cyclic.options) == feedback_count
    assert connection.base.sent == []
    assert connection.base.stop_count == 1


def test_healthy_reclutch_refreshes_control_anchor():
    connection = _Connection(_feedback())
    backend = _backend(connection, anchor_envelope=FIRST_TRIAL_ANCHOR_ENVELOPE)
    _begin_pose_control(backend)
    backend.hold()
    connection.base_cyclic.feedback = _feedback((0.1, 0.0, 0.0))

    refreshed = _begin_pose_control(backend)
    result = backend.command_pose(_target(position=(0.12, 0.0, 0.0)))

    np.testing.assert_allclose(refreshed.position, [0.1, 0.0, 0.0])
    assert result.accepted is True
    assert len(connection.base.sent) == 1
    assert connection.base.stop_count == 1


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
    _begin_pose_control(backend)

    backend.command_pose(_target(position=(0.1, 0.0, 0.0)))

    np.testing.assert_allclose(_velocity(connection.base.sent[-1]), [0.05, 0, 0, 0, 0, 0])
    assert connection.base.sent[-1].reference_frame == 17
    assert connection.base.sent[-1].duration == 0
    assert connection.base.send_options[-1].timeout_ms < 200


def test_twist_command_supports_kortex_28_schema_without_duration():
    """Kortex 2.8 TwistCommand has only reference_frame and twist fields."""

    class TwistCommandV28:
        __slots__ = ("reference_frame", "twist")

        def __init__(self):
            self.reference_frame = None
            self.twist = _Twist()

    connection = _Connection(_feedback())
    connection.base_pb2 = SimpleNamespace(
        **{
            **vars(BASE_PB2),
            "TwistCommand": TwistCommandV28,
        }
    )
    backend = _backend(connection, kp_linear=1.0)
    _begin_pose_control(backend)

    result = backend.command_pose(_target(position=(0.01, 0.0, 0.0)))

    assert result.accepted
    assert len(connection.base.sent) == 1
    np.testing.assert_allclose(
        _velocity(connection.base.sent[0]),
        [0.01, 0, 0, 0, 0, 0],
    )


def test_rotation_error_is_converted_from_radians_and_clipped_in_degrees():
    connection = _Connection(_feedback())
    backend = _backend(connection, kp_angular=1.0)
    _begin_pose_control(backend)
    z_90 = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))

    result = backend.command_pose(_target(quaternion=z_90))

    np.testing.assert_allclose(_velocity(connection.base.sent[-1]), [0, 0, 0, 0, 0, 5])
    assert math.isclose(result.rotation_error, math.pi / 2)


def test_hold_stops_only_once_and_close_is_idempotent():
    connection = _Connection(_feedback())
    backend = _backend(connection)
    _begin_pose_control(backend)
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
    _begin_pose_control(backend)
    backend.command_pose(_target(position=(0.01, 0.0, 0.0)))

    clock.now = 0.2
    backend.check_watchdog()
    assert connection.base.stop_count == 0
    clock.now = 0.200001
    backend.check_watchdog()
    backend.check_watchdog()

    assert connection.base.stop_count == 1
    assert backend.fault_reason == "watchdog timeout"
    with pytest.raises(KortexSafetyError, match="watchdog timeout"):
        backend.command_pose(_target(position=(0.02, 0.0, 0.0)))
    with pytest.raises(KortexSafetyError, match="latched"):
        backend.begin_control()
    assert len(connection.base.sent) == 1


def test_watchdog_fault_surfaces_from_current_pose_and_step():
    clock = _Clock()
    connection = _Connection(_feedback())
    backend = _backend(connection, monotonic=clock)
    _begin_pose_control(backend)
    backend.command_pose(_target(position=(0.01, 0.0, 0.0)))
    clock.now = 0.200001
    backend.check_watchdog()

    with pytest.raises(KortexSafetyError, match="watchdog timeout"):
        backend.current_pose()
    with pytest.raises(KortexSafetyError, match="watchdog timeout"):
        backend.step()


def test_fresh_successful_command_resets_watchdog_deadline():
    clock = _Clock()
    connection = _Connection(_feedback())
    backend = _backend(connection, monotonic=clock)
    _begin_pose_control(backend)
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
    backend = KortexBackend(
        connection,
        anchor_envelope=TEST_ANCHOR_ENVELOPE,
        monotonic=clock,
    )
    assert _ManualThread.instances == []
    _begin_pose_control(backend)

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
    _begin_pose_control(backend)
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
    _begin_pose_control(backend)
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
    rejected_errors = []
    later_command = threading.Thread(
        target=lambda: _capture_result_or_error(
            lambda: backend.command_pose(_target(position=(0.03, 0.0, 0.0))),
            rejected_result,
            rejected_errors,
        )
    )
    later_command.start()
    later_command.join(timeout=0.25)

    try:
        assert not later_command.is_alive()
        assert rejected_result == []
        assert len(rejected_errors) == 1
        assert "watchdog timeout" in str(rejected_errors[0])
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
    _begin_pose_control(backend)
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
    assert backend.fault_reason == "Stop attempted but unconfirmed"
    with pytest.raises(KortexSafetyError, match="Stop attempted but unconfirmed"):
        backend.step()
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
    backend = KortexBackend(connection, anchor_envelope=TEST_ANCHOR_ENVELOPE)
    _begin_pose_control(backend)

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
    backend = KortexBackend(connection, anchor_envelope=TEST_ANCHOR_ENVELOPE)
    _begin_pose_control(backend)

    with pytest.raises(RuntimeError, match="Failed to start Kortex watchdog"):
        backend.command_pose(_target(position=(0.01, 0.0, 0.0)))

    assert connection.base.stop_count == 1
    assert backend.stop_confirmed is True
    assert backend.fault_reason == "watchdog unavailable"
    with pytest.raises(KortexSafetyError, match="latched"):
        backend.begin_control()


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
    backend = KortexBackend(connection, anchor_envelope=TEST_ANCHOR_ENVELOPE)
    _begin_pose_control(backend)

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
    _begin_pose_control(backend)

    with pytest.raises(RuntimeError, match="bad message"):
        backend.command_pose(_target(position=(0.01, 0.0, 0.0)))

    assert connection.base.stop_count == 1
    assert backend.stop_confirmed is True


def test_watchdog_cannot_be_disabled_through_production_constructor():
    connection = _Connection(_feedback())

    with pytest.raises(TypeError):
        KortexBackend(
            connection,
            anchor_envelope=TEST_ANCHOR_ENVELOPE,
            start_watchdog=False,
        )
    with pytest.raises(TypeError):
        KortexBackend(
            connection,
            anchor_envelope=TEST_ANCHOR_ENVELOPE,
            shutdown_event=threading.Event(),
        )
    with pytest.raises(TypeError):
        KortexBackend(
            connection,
            anchor_envelope=TEST_ANCHOR_ENVELOPE,
            thread_factory=_ManualThread,
        )


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
    backend = KortexBackend(connection, anchor_envelope=TEST_ANCHOR_ENVELOPE)
    _begin_pose_control(backend)
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
    connection_closed = threading.Event()
    connection = _Connection(_feedback())
    backend = _backend(connection)
    _begin_pose_control(backend)

    def failing_send(command, *, options=None):
        send_entered.set()
        assert release_send.wait(timeout=1.0)
        raise RuntimeError("send failed")

    connection.base.SendTwistCommand = failing_send

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
    assert connection_closed.wait(timeout=1.0)
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


def _capture_result_or_error(call, results, errors):
    try:
        results.append(call())
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
