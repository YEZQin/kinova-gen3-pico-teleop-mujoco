"""Pose mathematics and stateful relative controller mapping."""

from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable

import numpy as np

PICO_TO_WORLD = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)


@dataclass(frozen=True)
class Pose:
    """Position and quaternion pose; quaternion ordering is w, x, y, z."""

    position: np.ndarray
    quaternion: np.ndarray


class ClutchState(str, Enum):
    WAITING_FOR_RELEASE = "waiting_for_release"
    READY = "ready"
    ACTIVE = "active"


class InputFault(str, Enum):
    """Stable classification for input conditions that deactivate mapping."""

    NONE = "none"
    INVALID = "invalid"
    STALE = "stale"
    SOURCE_CHANGED = "source_changed"


@dataclass(frozen=True)
class MappingConfig:
    translation_scale: float = 1.0
    grip_press_threshold: float = 0.9
    grip_release_threshold: float = 0.8
    stale_timeout: float = 0.2
    filter_time_constant: float = 0.05
    max_position_step: float = 0.02
    max_rotation_step: float = 0.15
    orientation_enabled: bool = True
    invert_translation: bool = False
    release_stability_samples: int = 1
    translation_rotation: tuple[tuple[float, float, float], ...] | None = None

    def __post_init__(self) -> None:
        thresholds = np.asarray(
            [self.grip_press_threshold, self.grip_release_threshold],
            dtype=np.float64,
        )
        if not np.isfinite(thresholds).all() or not np.all(
            (0.0 <= thresholds) & (thresholds <= 1.0)
        ):
            raise ValueError("Grip thresholds must be finite values in [0, 1]")
        if self.grip_release_threshold >= self.grip_press_threshold:
            raise ValueError("Grip release threshold must be lower than press threshold")
        if (
            isinstance(self.release_stability_samples, bool)
            or not isinstance(self.release_stability_samples, int)
            or self.release_stability_samples <= 0
        ):
            raise ValueError("release_stability_samples must be a positive integer")
        if self.translation_rotation is not None:
            try:
                rotation = np.asarray(self.translation_rotation, dtype=np.float64)
            except (TypeError, ValueError) as error:
                raise ValueError("translation_rotation must be a finite 3x3 rotation") from error
            if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
                raise ValueError("translation_rotation must be a finite 3x3 rotation")
            if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9, rtol=0.0):
                raise ValueError("translation_rotation must be orthogonal")
            if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-9, rtol=0.0):
                raise ValueError("translation_rotation must have determinant +1")
            if self.invert_translation:
                raise ValueError(
                    "invert_translation cannot be combined with translation_rotation"
                )


@dataclass(frozen=True)
class MappingOutput:
    target: Pose
    active: bool
    clutch_state: ClutchState
    activated: bool
    deactivated: bool
    stale: bool
    input_fault: InputFault = InputFault.NONE


def normalize_quat(quaternion: np.ndarray) -> np.ndarray:
    """Return a normalized, sign-canonical wxyz quaternion."""

    quaternion = np.asarray(quaternion, dtype=np.float64)
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError("Quaternion must contain four finite values")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError("Quaternion norm must be non-zero")
    result = quaternion / norm
    return -result if result[0] < 0.0 else result


def quat_conjugate(quaternion: np.ndarray) -> np.ndarray:
    quaternion = normalize_quat(quaternion)
    return np.array(
        [quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]],
        dtype=np.float64,
    )


def quat_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """Compose two wxyz quaternions, applying right then left."""

    w1, x1, y1, z1 = normalize_quat(left)
    w2, x2, y2, z2 = normalize_quat(right)
    return normalize_quat(
        np.array(
            [
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            ],
            dtype=np.float64,
        )
    )


def quat_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = normalize_quat(quaternion)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quat(matrix: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a normalized wxyz quaternion."""

    matrix = np.asarray(matrix, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("Rotation matrix must be a finite 3x3 array")

    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [
                0.25 * scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            ]
        )
    else:
        diagonal_index = int(np.argmax(np.diag(matrix)))
        if diagonal_index == 0:
            scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = np.array(
                [
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                    0.25 * scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                ]
            )
        elif diagonal_index == 1:
            scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = np.array(
                [
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    0.25 * scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                ]
            )
        else:
            scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = np.array(
                [
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    0.25 * scale,
                ]
            )
    return normalize_quat(quaternion)


def quat_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=np.float64)
    if axis.shape != (3,) or not np.isfinite(axis).all() or not np.isfinite(angle):
        raise ValueError("Axis and angle must be finite")
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-12 or abs(angle) < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    unit_axis = axis / axis_norm
    half_angle = angle * 0.5
    return normalize_quat(
        np.concatenate(([np.cos(half_angle)], unit_axis * np.sin(half_angle)))
    )


def quat_to_rotvec(quaternion: np.ndarray) -> np.ndarray:
    quaternion = normalize_quat(quaternion)
    vector = quaternion[1:]
    vector_norm = float(np.linalg.norm(vector))
    if vector_norm < 1e-12:
        return np.zeros(3, dtype=np.float64)
    angle = 2.0 * np.arctan2(vector_norm, np.clip(quaternion[0], -1.0, 1.0))
    return vector / vector_norm * angle


def slerp(start: np.ndarray, end: np.ndarray, fraction: float) -> np.ndarray:
    start = normalize_quat(start)
    end = normalize_quat(end)
    fraction = float(np.clip(fraction, 0.0, 1.0))
    dot = float(np.dot(start, end))
    if dot < 0.0:
        end = -end
        dot = -dot
    if dot > 0.9995:
        return normalize_quat(start + fraction * (end - start))
    angle = np.arccos(np.clip(dot, -1.0, 1.0))
    return normalize_quat(
        np.sin((1.0 - fraction) * angle) / np.sin(angle) * start
        + np.sin(fraction * angle) / np.sin(angle) * end
    )


def transform_controller_pose(position: np.ndarray, quaternion_xyzw: np.ndarray) -> Pose:
    """Transform a PICO pose into the MuJoCo world basis."""

    position = np.asarray(position, dtype=np.float64)
    quaternion_xyzw = np.asarray(quaternion_xyzw, dtype=np.float64)
    if position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError("Controller position must contain three finite values")
    if quaternion_xyzw.shape != (4,):
        raise ValueError("Controller quaternion must contain four values")
    quaternion_wxyz = normalize_quat(np.roll(quaternion_xyzw, 1))
    world_rotation = PICO_TO_WORLD @ quat_to_matrix(quaternion_wxyz) @ PICO_TO_WORLD.T
    return Pose(
        position=PICO_TO_WORLD @ position,
        quaternion=matrix_to_quat(world_rotation),
    )


def _copy_pose(pose: Pose) -> Pose:
    return Pose(
        np.asarray(pose.position, dtype=np.float64).copy(),
        normalize_quat(pose.quaternion),
    )


def _clip_vector(vector: np.ndarray, maximum_norm: float) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if maximum_norm <= 0.0 or norm <= maximum_norm:
        return vector
    return vector * (maximum_norm / norm)


class RelativePoseMapper:
    """Map clutch-relative controller motion to a filtered robot target pose."""

    def __init__(self, config: MappingConfig):
        self.config = config
        self.clutch_state = ClutchState.WAITING_FOR_RELEASE
        self._controller_reference: Pose | None = None
        self._ee_reference: Pose | None = None
        self._last_target: Pose | None = None
        self._last_timestamp: int | None = None
        self._last_fresh_time: float | None = None
        self._last_update_time: float | None = None
        self._release_streak = 0

    def reset(self, ee_pose: Pose) -> None:
        self.clutch_state = ClutchState.WAITING_FOR_RELEASE
        self._controller_reference = None
        self._ee_reference = None
        self._last_target = _copy_pose(ee_pose)
        self._last_timestamp = None
        self._last_fresh_time = None
        self._last_update_time = None
        self._release_streak = 0

    def _deactivate(self, stale: bool, input_fault: InputFault) -> MappingOutput:
        was_active = self.clutch_state is ClutchState.ACTIVE
        self.clutch_state = ClutchState.WAITING_FOR_RELEASE
        self._controller_reference = None
        self._ee_reference = None
        self._last_timestamp = None
        self._last_fresh_time = None
        self._last_update_time = None
        self._release_streak = 0
        if self._last_target is None:
            raise RuntimeError("Pose mapper has no target")
        return MappingOutput(
            target=_copy_pose(self._last_target),
            active=False,
            clutch_state=self.clutch_state,
            activated=False,
            deactivated=was_active,
            stale=stale,
            input_fault=input_fault,
        )

    def update(
        self,
        sample: Any,
        anchor_pose: Pose | Callable[[], Pose],
        now: float,
    ) -> MappingOutput:
        """Advance the clutch state machine with one controller sample.

        ``anchor_pose`` provides the end-effector pose used as the relative
        anchor. Passing a callable defers the read to the moment the clutch
        engages, which lets hardware backends run their begin-control
        transaction exactly once per activation. If the callable raises, the
        clutch state is left unchanged.
        """

        if self._last_target is None:
            if callable(anchor_pose):
                raise RuntimeError(
                    "reset() must be called before update() when the anchor "
                    "is deferred; a deferred anchor may only run on activation"
                )
            self._last_target = _copy_pose(anchor_pose)

        sample_valid = bool(getattr(sample, "valid", False))
        sample_valid = sample_valid and np.isfinite(getattr(sample, "grip", np.nan))
        new_timestamp = (
            sample_valid
            and (self._last_timestamp is None or int(sample.timestamp_ns) > self._last_timestamp)
        )
        if new_timestamp:
            self._last_timestamp = int(sample.timestamp_ns)
            self._last_fresh_time = float(sample.received_monotonic)

        stale = (
            self._last_fresh_time is None
            or now - self._last_fresh_time > self.config.stale_timeout
        )
        if not sample_valid:
            invalid_reason = str(getattr(sample, "invalid_reason", ""))
            if invalid_reason == "stream is stale":
                input_fault = InputFault.STALE
            elif invalid_reason == "source changed":
                input_fault = InputFault.SOURCE_CHANGED
            else:
                input_fault = InputFault.INVALID
            return self._deactivate(
                stale=stale or input_fault is InputFault.STALE,
                input_fault=input_fault,
            )
        if stale:
            return self._deactivate(stale=True, input_fault=InputFault.STALE)

        try:
            controller_pose = transform_controller_pose(
                sample.position, sample.quaternion_xyzw
            )
        except ValueError:
            return self._deactivate(stale=False, input_fault=InputFault.INVALID)

        grip = float(sample.grip)
        if self.clutch_state is ClutchState.WAITING_FOR_RELEASE:
            if grip < self.config.grip_release_threshold:
                if new_timestamp:
                    self._release_streak += 1
                if self._release_streak >= self.config.release_stability_samples:
                    self.clutch_state = ClutchState.READY
            else:
                self._release_streak = 0
            return MappingOutput(
                target=_copy_pose(self._last_target),
                active=False,
                clutch_state=self.clutch_state,
                activated=False,
                deactivated=False,
                stale=False,
            )

        if self.clutch_state is ClutchState.READY:
            if grip <= self.config.grip_press_threshold:
                return MappingOutput(
                    target=_copy_pose(self._last_target),
                    active=False,
                    clutch_state=self.clutch_state,
                    activated=False,
                    deactivated=False,
                    stale=False,
                )

        elif grip < self.config.grip_release_threshold:
            self.clutch_state = ClutchState.READY
            self._controller_reference = None
            self._ee_reference = None
            return MappingOutput(
                target=_copy_pose(self._last_target),
                active=False,
                clutch_state=self.clutch_state,
                activated=False,
                deactivated=True,
                stale=False,
            )

        if self.clutch_state is ClutchState.ACTIVE and not new_timestamp:
            return MappingOutput(
                target=_copy_pose(self._last_target),
                active=True,
                clutch_state=self.clutch_state,
                activated=False,
                deactivated=False,
                stale=False,
            )

        if self.clutch_state is not ClutchState.ACTIVE:
            ee_pose = anchor_pose() if callable(anchor_pose) else anchor_pose
            self.clutch_state = ClutchState.ACTIVE
            self._controller_reference = controller_pose
            self._ee_reference = _copy_pose(ee_pose)
            self._last_target = _copy_pose(ee_pose)
            self._last_update_time = now
            return MappingOutput(
                target=_copy_pose(self._last_target),
                active=True,
                clutch_state=self.clutch_state,
                activated=True,
                deactivated=False,
                stale=False,
            )

        if self._controller_reference is None or self._ee_reference is None:
            raise RuntimeError("Active mapper is missing reference poses")

        delta = controller_pose.position - self._controller_reference.position
        if self.config.translation_rotation is not None:
            delta = np.asarray(self.config.translation_rotation) @ delta
        translation_sign = -1.0 if self.config.invert_translation else 1.0
        desired_position = self._ee_reference.position + (
            translation_sign
            * self.config.translation_scale
            * delta
        )
        if self.config.orientation_enabled:
            controller_delta = quat_multiply(
                controller_pose.quaternion,
                quat_conjugate(self._controller_reference.quaternion),
            )
            desired_quaternion = quat_multiply(
                controller_delta, self._ee_reference.quaternion
            )
        else:
            desired_quaternion = self._ee_reference.quaternion

        delta_time = max(0.0, now - (self._last_update_time or now))
        if self.config.filter_time_constant <= 0.0:
            filter_fraction = 1.0
        else:
            filter_fraction = 1.0 - np.exp(
                -delta_time / self.config.filter_time_constant
            )

        filtered_position = self._last_target.position + filter_fraction * (
            desired_position - self._last_target.position
        )
        filtered_position = self._last_target.position + _clip_vector(
            filtered_position - self._last_target.position,
            self.config.max_position_step,
        )
        filtered_quaternion = slerp(
            self._last_target.quaternion, desired_quaternion, filter_fraction
        )
        rotation_step = quat_to_rotvec(
            quat_multiply(
                filtered_quaternion, quat_conjugate(self._last_target.quaternion)
            )
        )
        rotation_step = _clip_vector(rotation_step, self.config.max_rotation_step)
        filtered_quaternion = quat_multiply(
            quat_from_axis_angle(rotation_step, float(np.linalg.norm(rotation_step))),
            self._last_target.quaternion,
        )

        self._last_target = Pose(filtered_position, filtered_quaternion)
        self._last_update_time = now
        return MappingOutput(
            target=_copy_pose(self._last_target),
            active=True,
            clutch_state=self.clutch_state,
            activated=False,
            deactivated=False,
            stale=False,
        )
