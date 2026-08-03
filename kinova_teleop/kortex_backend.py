"""Guarded high-level Cartesian Twist backend for Kinova Kortex."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from collections.abc import Mapping
from typing import Any, Callable

import numpy as np

from .backend import BackendResult
from .pose_mapping import (
    Pose,
    matrix_to_quat,
    normalize_quat,
    quat_conjugate,
    quat_multiply,
    quat_to_rotvec,
)
from .workspace import WorkspaceLimits, validate_target_pose

MAX_LINEAR_SPEED = 0.03
MAX_ANGULAR_SPEED_DEG = 5.0
WATCHDOG_TIMEOUT = 0.2
RPC_LOCK_TIMEOUT = 0.15
WATCHDOG_POLL_INTERVAL = 0.01
WATCHDOG_READY_TIMEOUT = 0.1
SERVO_READY_TIMEOUT = 3.0
FAULT_POLL_INTERVAL = 0.05
GRIPPER_DEADBAND = 0.02
GRIPPER_MIN_INTERVAL = 0.1

# Private test seam. Production callers cannot replace or disable the watchdog.
_watchdog_thread_factory = threading.Thread


@dataclass(frozen=True)
class _StopToken:
    epoch: int
    generation: int


class KortexSafetyError(RuntimeError):
    """A bounded safety operation could not be confirmed."""


def _clip_norm(vector: np.ndarray, maximum_norm: float) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm <= maximum_norm or norm == 0.0:
        return vector
    return vector * (maximum_norm / norm)


def _fixed_xyz_quaternion(angles_deg: np.ndarray) -> np.ndarray:
    x, y, z = np.radians(angles_deg)
    sx, cx = math.sin(x), math.cos(x)
    sy, cy = math.sin(y), math.cos(y)
    sz, cz = math.sin(z), math.cos(z)
    rotation = np.array(
        [
            [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
            [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
            [-sy, cy * sx, cy * cx],
        ],
        dtype=np.float64,
    )
    return matrix_to_quat(rotation)


def _inactive_result() -> BackendResult:
    return BackendResult(
        accepted=False,
        converged=False,
        position_error=0.0,
        rotation_error=0.0,
        reason="kortex backend is not accepting commands",
    )


class KortexBackend:
    """Convert base-frame pose errors to guarded Kortex Twist commands."""

    def __init__(
        self,
        connection: Any,
        *,
        kp_linear: float = 1.0,
        kp_angular: float = 1.0,
        max_linear_speed: float = MAX_LINEAR_SPEED,
        max_angular_speed_deg: float = MAX_ANGULAR_SPEED_DEG,
        workspace_limits: WorkspaceLimits | None = None,
        event_sink: Callable[[str, str, Mapping[str, object]], None] | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ):
        values = np.array(
            [kp_linear, kp_angular, max_linear_speed, max_angular_speed_deg],
            dtype=np.float64,
        )
        if not np.isfinite(values).all():
            raise ValueError("Kortex control parameters must be finite")
        if kp_linear < 0.0 or kp_angular < 0.0:
            raise ValueError("Kortex proportional gains must be non-negative")
        if not 0.0 < max_linear_speed <= MAX_LINEAR_SPEED:
            raise ValueError("Linear speed limit must be in (0, 0.03] m/s")
        if not 0.0 < max_angular_speed_deg <= MAX_ANGULAR_SPEED_DEG:
            raise ValueError("Angular speed limit must be in (0, 5] deg/s")
        if connection.base is None or connection.base_cyclic is None:
            raise RuntimeError("Kortex connection is not connected")

        self.connection = connection
        self.kp_linear = float(kp_linear)
        self.kp_angular = float(kp_angular)
        self.max_linear_speed = float(max_linear_speed)
        self.max_angular_speed_deg = float(max_angular_speed_deg)
        self.workspace_limits = workspace_limits
        self.event_sink = event_sink
        self._monotonic = monotonic
        self._sleep = sleep

        self._state_lock = threading.Lock()
        self._rpc_lock = threading.Lock()
        self._watchdog_start_lock = threading.Lock()
        self._shutdown = threading.Event()
        self._watchdog_thread: Any | None = None
        self._watchdog_ready: threading.Event | None = None
        self._watchdog_started = False
        self._deadline: float | None = None
        self._generation = 0
        self._stop_epoch = 0
        self._stop_token: _StopToken | None = None
        self._terminal_stop_token: _StopToken | None = None
        self._active = False
        self._stop_requested = False
        self._stop_confirmed = True
        self._feedback_requires_rearm = False
        self._rearm_feedback_pending = False
        self._closed = False
        self._connection_closed = False
        self._gripper_last_value: float | None = None
        self._gripper_last_time: float | None = None

        try:
            self._require_startup_ready()
            servoing_mode = self.connection.base_pb2.ServoingModeInformation()
            servoing_mode.servoing_mode = (
                self.connection.base_pb2.SINGLE_LEVEL_SERVOING
            )
            if not self._rpc_lock.acquire(timeout=RPC_LOCK_TIMEOUT):
                raise KortexSafetyError("Timed out preparing Kortex servoing mode")
            try:
                self.connection.base.SetServoingMode(
                    servoing_mode,
                    options=self.connection.rpc_options(),
                )
            finally:
                self._rpc_lock.release()
            self._wait_until_servo_ready()
        except BaseException:
            token = self._request_stop(force=True)
            self._attempt_stop(token=token, raise_on_failure=False)
            raise

    def _read_arm_state(self) -> Any:
        if not self._rpc_lock.acquire(timeout=RPC_LOCK_TIMEOUT):
            raise KortexSafetyError("Timed out reading Kortex arm state")
        try:
            return self.connection.base.GetArmState(
                options=self.connection.rpc_options()
            ).active_state
        finally:
            self._rpc_lock.release()

    def _emit(
        self,
        kind: str,
        state: str,
        payload: Mapping[str, object] | None = None,
    ) -> None:
        sink = self.event_sink
        if sink is None:
            return
        try:
            sink(kind, state, dict(payload or {}))
        except Exception:
            # Evidence failures must not disable the watchdog or alter Stop order.
            return

    def _require_startup_ready(self) -> None:
        """Read and latch a startup fault; recovery belongs to the operator."""

        base_pb2 = self.connection.base_pb2
        state = self._read_arm_state()
        fault_state = getattr(base_pb2, "ARMSTATE_IN_FAULT", object())
        if state == fault_state or state == "ARMSTATE_IN_FAULT":
            self._emit("faulted", "FAULTED", {"reason": "startup fault"})
            raise KortexSafetyError(
                "Kortex startup fault is latched; clear it in the Kinova Web App and retry"
            )

    def _wait_until_servo_ready(self) -> None:
        """Wait for SERVOING_READY after single-level servoing was requested."""

        base_pb2 = self.connection.base_pb2
        ready_state = base_pb2.ARMSTATE_SERVOING_READY
        deadline = self._monotonic() + SERVO_READY_TIMEOUT
        while True:
            state = self._read_arm_state()
            if state == ready_state or state == "ARMSTATE_SERVOING_READY":
                return
            if self._monotonic() >= deadline:
                raise KortexSafetyError(
                    f"Kortex arm did not reach SERVOING_READY (state={state}); "
                    "check faults and the servoing mode in the Kinova Web App "
                    "and retry"
                )
            self._sleep(FAULT_POLL_INTERVAL)

    @property
    def stop_requested(self) -> bool:
        with self._state_lock:
            return self._stop_requested

    @property
    def stop_confirmed(self) -> bool:
        """Whether the most recent Stop RPC returned successfully.

        This is not proof that physical motion has ceased.
        """

        with self._state_lock:
            return self._stop_confirmed

    def _state_allows_command_locked(self, generation: int | None = None) -> bool:
        return (
            not self._closed
            and self._active
            and not self._stop_requested
            and (generation is None or generation == self._generation)
        )

    def _feedback_admission_locked(self) -> int | None:
        if self._closed:
            raise KortexSafetyError("Kortex backend is closing")
        if self._stop_requested:
            raise KortexSafetyError("Stop is requested; feedback is blocked")
        if self._feedback_requires_rearm and not self._rearm_feedback_pending:
            raise KortexSafetyError(
                "Stop was requested; begin_control is required before feedback"
            )
        return self._generation if self._rearm_feedback_pending else None

    def current_pose(self) -> Pose:
        with self._state_lock:
            self._feedback_admission_locked()
        if not self._rpc_lock.acquire(timeout=RPC_LOCK_TIMEOUT):
            raise KortexSafetyError("Timed out waiting for in-flight RPC")
        try:
            with self._state_lock:
                rearm_generation = self._feedback_admission_locked()
            feedback = self.connection.base_cyclic.RefreshFeedback(
                options=self.connection.rpc_options()
            ).base

            position = np.array(
                [feedback.tool_pose_x, feedback.tool_pose_y, feedback.tool_pose_z],
                dtype=np.float64,
            )
            angles = np.array(
                [
                    feedback.tool_pose_theta_x,
                    feedback.tool_pose_theta_y,
                    feedback.tool_pose_theta_z,
                ],
                dtype=np.float64,
            )
            if not np.isfinite(position).all() or not np.isfinite(angles).all():
                raise ValueError("Kortex feedback pose must be finite")
            pose = Pose(
                position=position,
                quaternion=_fixed_xyz_quaternion(angles),
            )
            if rearm_generation is not None:
                with self._state_lock:
                    if (
                        self._closed
                        or self._stop_requested
                        or not self._rearm_feedback_pending
                        or self._generation != rearm_generation
                    ):
                        raise KortexSafetyError(
                            "Stop was requested during re-arm feedback"
                        )
                    self._rearm_feedback_pending = False
                    self._feedback_requires_rearm = False
            return pose
        finally:
            self._rpc_lock.release()

    def begin_control(self) -> None:
        with self._state_lock:
            if self._closed:
                raise RuntimeError("Kortex backend is closed")
            if self._stop_requested and not self._stop_confirmed:
                raise KortexSafetyError(
                    "Cannot resume while the previous Stop is unconfirmed"
                )
            self._generation += 1
            self._active = True
            self._stop_requested = False
            self._stop_confirmed = False
            self._stop_token = None
            self._deadline = None
            self._rearm_feedback_pending = True
        self._emit("control_started", "ARMED", {})

    def command_pose(self, target: Pose) -> BackendResult:
        with self._state_lock:
            if not self._state_allows_command_locked():
                return _inactive_result()
            generation = self._generation

        # Workspace checks are deliberately before any feedback or Twist RPC.
        if self.workspace_limits is not None:
            decision = validate_target_pose(target, self.workspace_limits)
            if not decision.accepted:
                self._emit(
                    "workspace_rejected",
                    "STOPPING",
                    {"reason": decision.reason},
                )
                self.hold()
                return BackendResult(False, False, 0.0, 0.0, decision.reason)

        try:
            target_position = np.asarray(target.position, dtype=np.float64)
            if target_position.shape != (3,) or not np.isfinite(target_position).all():
                raise ValueError("Target position must contain three finite values")
            target_quaternion = normalize_quat(target.quaternion)
            measured = self.current_pose()
            position_error = target_position - measured.position
            q_error = quat_multiply(
                target_quaternion,
                quat_conjugate(measured.quaternion),
            )
            rotation_error = quat_to_rotvec(q_error)
            linear = _clip_norm(
                self.kp_linear * position_error,
                self.max_linear_speed,
            )
            angular_rad = _clip_norm(
                self.kp_angular * rotation_error,
                math.radians(self.max_angular_speed_deg),
            )
            angular_deg = np.degrees(angular_rad)
            velocity = np.concatenate((linear, angular_deg))

            command = self.connection.base_pb2.TwistCommand()
            command.reference_frame = (
                self.connection.base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
            )
            command.duration = 0
            (
                command.twist.linear_x,
                command.twist.linear_y,
                command.twist.linear_z,
                command.twist.angular_x,
                command.twist.angular_y,
                command.twist.angular_z,
            ) = map(float, velocity)
        except BaseException:
            token = self._request_stop(force=True)
            self._attempt_stop(token=token, raise_on_failure=False)
            raise

        result = BackendResult(
            accepted=True,
            converged=True,
            position_error=float(np.linalg.norm(position_error)),
            rotation_error=float(np.linalg.norm(rotation_error)),
            reason="",
        )
        nonzero = float(np.linalg.norm(velocity)) > 0.0
        if nonzero:
            self._ensure_watchdog_ready()
        if not self._rpc_lock.acquire(timeout=RPC_LOCK_TIMEOUT):
            token = self._request_stop(force=True)
            self._attempt_stop(token=token, raise_on_failure=False)
            raise KortexSafetyError(
                "Timed out waiting for in-flight RPC; Stop attempted but unconfirmed"
            )

        send_error: BaseException | None = None
        sent = False
        try:
            with self._state_lock:
                if not self._state_allows_command_locked(generation):
                    return _inactive_result()
                if nonzero and self._deadline is None:
                    # Arm pessimistically before the send: the very first
                    # nonzero twist of a control epoch must be watchdog-covered
                    # even if this thread stalls right after SendTwistCommand.
                    # Later commands stay covered by the previous deadline.
                    self._deadline = self._monotonic() + WATCHDOG_TIMEOUT
            try:
                self.connection.base.SendTwistCommand(
                    command,
                    options=self.connection.rpc_options(),
                )
                sent = True
            except BaseException as error:
                send_error = error
        finally:
            self._rpc_lock.release()

        if send_error is not None:
            token = self._request_stop(force=True)
            self._attempt_stop(token=token, raise_on_failure=False)
            raise send_error

        with self._state_lock:
            if not sent or not self._state_allows_command_locked(generation):
                return _inactive_result()
            if nonzero:
                self._deadline = self._monotonic() + WATCHDOG_TIMEOUT
        if nonzero:
            self._emit("moving", "MOVING", {})
        return result

    def command_gripper(self, position: float) -> bool:
        """Send a rate-limited positional gripper command.

        Returns ``True`` when a command was sent, ``False`` when the value was
        deduplicated, rate-limited, or the backend is not accepting commands.
        Gripper commands are positional and self-terminating, so they are not
        covered by the twist watchdog.
        """

        if not math.isfinite(position):
            token = self._request_stop(force=True)
            self._attempt_stop(token=token, raise_on_failure=False)
            raise ValueError("Gripper position must be finite")
        position = float(min(1.0, max(0.0, position)))

        now = self._monotonic()
        with self._state_lock:
            if not self._state_allows_command_locked():
                return False
            generation = self._generation
            if (
                self._gripper_last_value is not None
                and abs(position - self._gripper_last_value) < GRIPPER_DEADBAND
            ):
                return False
            if (
                self._gripper_last_time is not None
                and now - self._gripper_last_time < GRIPPER_MIN_INTERVAL
            ):
                return False

        command = self.connection.base_pb2.GripperCommand()
        command.mode = self.connection.base_pb2.GRIPPER_POSITION
        finger = command.gripper.finger.add()
        finger.finger_identifier = 1
        finger.value = position

        if not self._rpc_lock.acquire(timeout=RPC_LOCK_TIMEOUT):
            token = self._request_stop(force=True)
            self._attempt_stop(token=token, raise_on_failure=False)
            raise KortexSafetyError(
                "Timed out waiting for in-flight RPC; Stop attempted but unconfirmed"
            )

        send_error: BaseException | None = None
        sent = False
        try:
            with self._state_lock:
                if not self._state_allows_command_locked(generation):
                    return False
            try:
                self.connection.base.SendGripperCommand(
                    command,
                    options=self.connection.rpc_options(),
                )
                sent = True
            except BaseException as error:
                send_error = error
        finally:
            self._rpc_lock.release()

        if send_error is not None:
            token = self._request_stop(force=True)
            self._attempt_stop(token=token, raise_on_failure=False)
            raise send_error

        if sent:
            with self._state_lock:
                self._gripper_last_value = position
                self._gripper_last_time = now
        return sent

    def _ensure_watchdog_ready(self) -> None:
        with self._watchdog_start_lock:
            with self._state_lock:
                if (
                    self._watchdog_thread is not None
                    and self._watchdog_ready is not None
                    and self._watchdog_ready.is_set()
                ):
                    return
                if not self._active:
                    return
            ready = threading.Event()
            start_failed = False
            try:
                candidate = _watchdog_thread_factory(
                    target=self._watchdog_loop,
                    args=(ready,),
                    name="kortex-command-watchdog",
                    daemon=True,
                )
                candidate.start()
                with self._state_lock:
                    self._watchdog_thread = candidate
                    self._watchdog_ready = ready
                    self._watchdog_started = True
            except BaseException:
                start_failed = True
            if not start_failed and ready.wait(WATCHDOG_READY_TIMEOUT):
                return

        token = self._request_stop(force=True)
        confirmed = self._attempt_stop(token=token, raise_on_failure=False)
        status = "Stop confirmed" if confirmed else "Stop attempted but unconfirmed"
        raise KortexSafetyError(
            f"Failed to start Kortex watchdog or report ready; {status}"
        ) from None

    def _watchdog_loop(self, ready: threading.Event) -> None:
        ready.set()
        while not self._shutdown.wait(WATCHDOG_POLL_INTERVAL):
            try:
                self.check_watchdog()
            except BaseException:
                # Safety state remains stop-requested/unconfirmed and is retried.
                continue

    def check_watchdog(self) -> None:
        token: _StopToken | None = None
        with self._state_lock:
            if (
                not self._closed
                and self._active
                and self._deadline is not None
                and self._monotonic() > self._deadline
            ):
                token = self._request_stop_locked(force=True)
            elif self._stop_requested and not self._stop_confirmed:
                token = self._stop_token
        if token is not None:
            self._attempt_stop(token=token, raise_on_failure=False)

    def _request_stop_locked(self, *, force: bool) -> _StopToken:
        if self._terminal_stop_token is not None:
            return self._terminal_stop_token
        if self._stop_requested and not force:
            if self._stop_token is None:
                raise RuntimeError("Stop request is missing its token")
            return self._stop_token
        self._stop_epoch += 1
        self._generation += 1
        token = _StopToken(self._stop_epoch, self._generation)
        self._active = False
        self._stop_requested = True
        self._stop_confirmed = False
        self._stop_token = token
        self._deadline = None
        self._feedback_requires_rearm = True
        self._rearm_feedback_pending = False
        return token

    def _request_stop(self, *, force: bool) -> _StopToken:
        emit = False
        with self._state_lock:
            emit = not self._stop_requested
            token = self._request_stop_locked(force=force)
        if emit:
            self._emit("host_stop_requested", "STOPPING", {})
        return token

    def _token_matches_locked(self, token: _StopToken) -> bool:
        return (
            not self._connection_closed
            and self._stop_requested
            and not self._stop_confirmed
            and self._stop_token == token
            and self._generation == token.generation
        )

    def _attempt_stop(
        self,
        *,
        token: _StopToken,
        raise_on_failure: bool,
    ) -> bool:
        with self._state_lock:
            if not self._token_matches_locked(token):
                return False

        acquired = self._rpc_lock.acquire(timeout=RPC_LOCK_TIMEOUT)
        failed = not acquired
        committed = False
        if acquired:
            try:
                with self._state_lock:
                    if not self._token_matches_locked(token):
                        return False
                    base = self.connection.base
                if base is None:
                    failed = True
                else:
                    try:
                        base.Stop(options=self.connection.rpc_options())
                    except BaseException:
                        failed = True
                with self._state_lock:
                    if self._token_matches_locked(token):
                        self._stop_confirmed = not failed
                        committed = True
            finally:
                self._rpc_lock.release()
        elif failed:
            with self._state_lock:
                if self._token_matches_locked(token):
                    self._stop_confirmed = False
        if committed and not failed:
            self._emit("stop_rpc_returned", "STOPPING", {})
        elif failed:
            self._emit("stop_unconfirmed", "FAULTED", {})
        if failed and raise_on_failure:
            raise KortexSafetyError("Stop attempted but unconfirmed") from None
        return committed and not failed

    def hold(self) -> None:
        with self._state_lock:
            if self._stop_requested and self._stop_confirmed:
                return
            force = not self._stop_requested
        token = self._request_stop(force=force)
        self._attempt_stop(token=token, raise_on_failure=True)

    def step(self) -> None:
        """The Kortex controller executes commands asynchronously."""

    def close(self) -> None:
        emit_stop_request = False
        with self._watchdog_start_lock:
            with self._state_lock:
                if self._connection_closed:
                    if not self._stop_confirmed:
                        raise KortexSafetyError(
                            "Kortex connection closed after Stop attempted "
                            "but unconfirmed"
                        ) from None
                    return
                self._closed = True
                if self._terminal_stop_token is None:
                    # The terminal token owns Stop state until cleanup completes.
                    self._stop_requested = False
                    self._stop_confirmed = False
                    close_token = self._request_stop_locked(force=True)
                    self._terminal_stop_token = close_token
                    emit_stop_request = True
                else:
                    close_token = self._terminal_stop_token
                watchdog_thread = (
                    self._watchdog_thread if self._watchdog_started else None
                )
        self._shutdown.set()
        if emit_stop_request:
            self._emit("host_stop_requested", "STOPPING", {})

        if not self._rpc_lock.acquire(timeout=RPC_LOCK_TIMEOUT):
            if (
                watchdog_thread is not None
                and watchdog_thread is not threading.current_thread()
            ):
                watchdog_thread.join(timeout=RPC_LOCK_TIMEOUT)
            raise KortexSafetyError(
                "Cannot close while an in-flight RPC is still active; "
                "Stop attempted but unconfirmed"
            ) from None

        try:
            connection_stop_confirmed = self.connection.close()
            with self._state_lock:
                self._connection_closed = True
                if self._terminal_stop_token == close_token:
                    self._stop_confirmed = bool(connection_stop_confirmed)
        finally:
            self._rpc_lock.release()

        if (
            watchdog_thread is not None
            and watchdog_thread is not threading.current_thread()
        ):
            watchdog_thread.join(timeout=RPC_LOCK_TIMEOUT)
        if not self.stop_confirmed:
            self._emit("stop_unconfirmed", "FAULTED", {})
            raise KortexSafetyError(
                "Kortex connection closed after Stop attempted but unconfirmed"
            ) from None
        self._emit("cleanup_completed", "COMPLETED", {})
