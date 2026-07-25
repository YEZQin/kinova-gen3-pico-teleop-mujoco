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
        shutdown_event: Any | None = None,
        start_watchdog: bool = True,
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
        self._start_watchdog = start_watchdog

        self._lock = threading.Lock()
        self._shutdown = shutdown_event if shutdown_event is not None else threading.Event()
        self._watchdog_thread: Any | None = None
        self._deadline: float | None = None
        self._generation = 0
        self._active = False
        self._stopped = True
        self._closed = False

        servoing_mode = self.connection.base_pb2.ServoingModeInformation()
        servoing_mode.servoing_mode = self.connection.base_pb2.SINGLE_LEVEL_SERVOING
        try:
            self.connection.base.SetServoingMode(servoing_mode)
        except BaseException:
            try:
                self.connection.base.Stop()
            except BaseException:
                pass
            raise

    def current_pose(self) -> Pose:
        feedback = self.connection.base_cyclic.RefreshFeedback().base
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
        with self._lock:
            if self._closed:
                raise RuntimeError("Kortex backend is closed")
            self._generation += 1
            self._active = True
            self._stopped = False
            self._deadline = None

    def _result_for_inactive(self) -> BackendResult:
        return BackendResult(converged=False)

    def command_pose(self, target: Pose) -> BackendResult:
        with self._lock:
            if self._closed or not self._active or self._stopped:
                return self._result_for_inactive()
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
        except BaseException:
            self.hold()
            raise

        result = BackendResult(
            converged=True,
            position_error=float(np.linalg.norm(position_error)),
            rotation_error=float(np.linalg.norm(rotation_error)),
        )
        velocity = np.concatenate((linear, angular_deg))

        with self._lock:
            if (
                self._closed
                or not self._active
                or self._stopped
                or generation != self._generation
            ):
                return self._result_for_inactive()

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
            try:
                self.connection.base.SendTwistCommand(command)
            except BaseException:
                self._stop_locked()
                raise

            if float(np.linalg.norm(velocity)) > 0.0:
                self._deadline = self._monotonic() + WATCHDOG_TIMEOUT
                self._arm_watchdog_locked()
            return result

    def _arm_watchdog_locked(self) -> None:
        if not self._start_watchdog or self._watchdog_thread is not None:
            return
        self._watchdog_thread = self._thread_factory(
            target=self._watchdog_loop,
            name="kortex-command-watchdog",
            daemon=True,
        )
        self._watchdog_thread.start()

    def _watchdog_loop(self) -> None:
        while not self._shutdown.wait(0.01):
            self.check_watchdog()

    def check_watchdog(self) -> None:
        """Run one timeout check; also used by deterministic fake-clock tests."""

        with self._lock:
            if (
                not self._closed
                and self._active
                and not self._stopped
                and self._deadline is not None
                and self._monotonic() > self._deadline
            ):
                self._stop_locked()

    def _stop_locked(self) -> None:
        if self._active and not self._stopped:
            try:
                self.connection.base.Stop()
            finally:
                self._active = False
                self._stopped = True
                self._deadline = None

    def hold(self) -> None:
        with self._lock:
            self._stop_locked()

    def step(self) -> None:
        """The Kortex controller executes commands asynchronously."""

    def close(self) -> None:
        watchdog_thread = None
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._active = False
            self._stopped = True
            self._deadline = None
            self._shutdown.set()
            watchdog_thread = self._watchdog_thread
            self.connection.close()
        if (
            watchdog_thread is not None
            and watchdog_thread is not threading.current_thread()
        ):
            watchdog_thread.join(timeout=1.0)
