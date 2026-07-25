"""Guarded high-level Cartesian Twist backend for Kinova Kortex."""

from __future__ import annotations

import math
import threading
import time
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

MAX_LINEAR_SPEED = 0.03
MAX_ANGULAR_SPEED_DEG = 5.0
WATCHDOG_TIMEOUT = 0.2
RPC_LOCK_TIMEOUT = 0.15
WATCHDOG_POLL_INTERVAL = 0.01


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
        monotonic: Callable[[], float] = time.monotonic,
        thread_factory: Callable[..., Any] = threading.Thread,
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
        self._monotonic = monotonic
        self._thread_factory = thread_factory

        self._state_lock = threading.Lock()
        self._rpc_lock = threading.Lock()
        self._watchdog_start_lock = threading.Lock()
        self._shutdown = threading.Event()
        self._watchdog_thread: Any | None = None
        self._deadline: float | None = None
        self._generation = 0
        self._active = False
        self._stop_requested = False
        self._stop_confirmed = True
        self._closed = False
        self._connection_closed = False

        try:
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
        except BaseException:
            self._request_stop(force=True)
            self._attempt_stop(raise_on_failure=False)
            raise

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

    def current_pose(self) -> Pose:
        with self._state_lock:
            if self._closed:
                raise KortexSafetyError("Kortex backend is closing")
        if not self._rpc_lock.acquire(timeout=RPC_LOCK_TIMEOUT):
            raise KortexSafetyError("Timed out waiting for in-flight RPC")
        try:
            with self._state_lock:
                if self._closed:
                    raise KortexSafetyError("Kortex backend is closing")
            feedback = self.connection.base_cyclic.RefreshFeedback(
                options=self.connection.rpc_options()
            ).base
        finally:
            self._rpc_lock.release()

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
        return Pose(position=position, quaternion=_fixed_xyz_quaternion(angles))

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
            self._deadline = None

    @staticmethod
    def _inactive_result() -> BackendResult:
        return BackendResult(converged=False)

    def command_pose(self, target: Pose) -> BackendResult:
        with self._state_lock:
            if not self._state_allows_command_locked():
                return self._inactive_result()
            generation = self._generation

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
            self._request_stop(force=True)
            self._attempt_stop(raise_on_failure=False)
            raise

        result = BackendResult(
            converged=True,
            position_error=float(np.linalg.norm(position_error)),
            rotation_error=float(np.linalg.norm(rotation_error)),
        )
        if not self._rpc_lock.acquire(timeout=RPC_LOCK_TIMEOUT):
            self._request_stop(force=True)
            self._attempt_stop(raise_on_failure=False)
            raise KortexSafetyError(
                "Timed out waiting for in-flight RPC; Stop attempted but unconfirmed"
            )

        send_error: BaseException | None = None
        sent = False
        try:
            with self._state_lock:
                if not self._state_allows_command_locked(generation):
                    return self._inactive_result()
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
            self._request_stop(force=True)
            self._attempt_stop(raise_on_failure=False)
            raise send_error

        with self._state_lock:
            if not sent or not self._state_allows_command_locked(generation):
                return self._inactive_result()
            if float(np.linalg.norm(velocity)) > 0.0:
                self._deadline = self._monotonic() + WATCHDOG_TIMEOUT

        if float(np.linalg.norm(velocity)) > 0.0:
            self._ensure_watchdog_started()
        return result

    def _ensure_watchdog_started(self) -> None:
        with self._watchdog_start_lock:
            with self._state_lock:
                if self._watchdog_thread is not None or not self._active:
                    return
            start_failed = False
            try:
                candidate = self._thread_factory(
                    target=self._watchdog_loop,
                    name="kortex-command-watchdog",
                    daemon=True,
                )
                candidate.start()
            except BaseException:
                start_failed = True
            if not start_failed:
                with self._state_lock:
                    self._watchdog_thread = candidate
                return

        self._request_stop(force=True)
        confirmed = self._attempt_stop(raise_on_failure=False)
        status = "Stop confirmed" if confirmed else "Stop attempted but unconfirmed"
        raise KortexSafetyError(
            f"Failed to start Kortex watchdog; {status}"
        ) from None

    def _watchdog_loop(self) -> None:
        while not self._shutdown.wait(WATCHDOG_POLL_INTERVAL):
            try:
                self.check_watchdog()
            except BaseException:
                # Safety state remains stop-requested/unconfirmed and is retried.
                continue

    def check_watchdog(self) -> None:
        should_attempt = False
        with self._state_lock:
            if (
                not self._closed
                and self._active
                and self._deadline is not None
                and self._monotonic() > self._deadline
            ):
                self._request_stop_locked(force=True)
            should_attempt = self._stop_requested and not self._stop_confirmed
        if should_attempt:
            self._attempt_stop(raise_on_failure=False)

    def _request_stop_locked(self, *, force: bool) -> None:
        if self._stop_requested and (self._stop_confirmed or not force):
            return
        self._generation += 1
        self._active = False
        self._stop_requested = True
        self._stop_confirmed = False
        self._deadline = None

    def _request_stop(self, *, force: bool) -> None:
        with self._state_lock:
            self._request_stop_locked(force=force)

    def _attempt_stop(self, *, raise_on_failure: bool) -> bool:
        with self._state_lock:
            if not self._stop_requested or self._stop_confirmed:
                return self._stop_confirmed

        acquired = self._rpc_lock.acquire(timeout=RPC_LOCK_TIMEOUT)
        failed = not acquired
        if acquired:
            try:
                self.connection.base.Stop(options=self.connection.rpc_options())
            except BaseException:
                failed = True
            finally:
                self._rpc_lock.release()

        with self._state_lock:
            if failed:
                self._stop_confirmed = False
            else:
                self._stop_confirmed = True
        if failed and raise_on_failure:
            raise KortexSafetyError("Stop attempted but unconfirmed") from None
        return not failed

    def hold(self) -> None:
        with self._state_lock:
            if self._stop_requested and self._stop_confirmed:
                return
            self._request_stop_locked(force=True)
        self._attempt_stop(raise_on_failure=True)

    def step(self) -> None:
        """The Kortex controller executes commands asynchronously."""

    def close(self) -> None:
        with self._state_lock:
            if self._connection_closed:
                return
            self._closed = True
            # Closing always makes a fresh bounded Stop attempt before disconnect.
            self._stop_requested = False
            self._stop_confirmed = False
            self._request_stop_locked(force=True)
            watchdog_thread = self._watchdog_thread
        self._shutdown.set()

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
        finally:
            self._rpc_lock.release()
        with self._state_lock:
            self._connection_closed = True
            self._stop_confirmed = bool(connection_stop_confirmed)

        if (
            watchdog_thread is not None
            and watchdog_thread is not threading.current_thread()
        ):
            watchdog_thread.join(timeout=RPC_LOCK_TIMEOUT)
        if not self.stop_confirmed:
            raise KortexSafetyError(
                "Kortex connection closed after Stop attempted but unconfirmed"
            ) from None
