"""Immutable Cartesian workspace validation for guarded Gen3 motion."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .pose_mapping import (
    Pose,
    normalize_quat,
    quat_conjugate,
    quat_multiply,
    quat_to_rotvec,
)


@dataclass(frozen=True, slots=True)
class WorkspaceDecision:
    """Result of evaluating one target pose against explicit XYZ bounds."""

    accepted: bool
    reason: str = ""


@dataclass(frozen=True, slots=True)
class AnchorEnvelope:
    """Inclusive translation-axis and shortest-arc rotation limits."""

    maximum_translation_axis_m: tuple[float, float, float]
    maximum_rotation_rad: float

    def __post_init__(self) -> None:
        translation = _xyz(
            self.maximum_translation_axis_m,
            "maximum_translation_axis_m",
        )
        try:
            rotation = float(self.maximum_rotation_rad)
        except (TypeError, ValueError) as error:
            raise ValueError(
                "anchor envelope limits must be positive finite values"
            ) from error
        if any(limit <= 0.0 for limit in translation) or not (
            np.isfinite(rotation) and rotation > 0.0
        ):
            raise ValueError("anchor envelope limits must be positive finite values")
        object.__setattr__(self, "maximum_translation_axis_m", translation)
        object.__setattr__(self, "maximum_rotation_rad", rotation)

    def evaluate(self, anchor: Pose, target: Pose) -> WorkspaceDecision:
        """Evaluate a target relative to an immutable control anchor."""

        anchor_components = _pose_components(anchor)
        if anchor_components is None:
            return WorkspaceDecision(False, "anchor pose is invalid")
        target_components = _pose_components(target)
        if target_components is None:
            return WorkspaceDecision(False, "target pose is invalid")
        anchor_position, anchor_quaternion = anchor_components
        target_position, target_quaternion = target_components

        delta = np.abs(target_position - anchor_position)
        translation_limits = np.asarray(self.maximum_translation_axis_m)
        translation_scale = np.maximum.reduce(
            (np.abs(anchor_position), np.abs(target_position), translation_limits)
        )
        translation_tolerance = 4.0 * np.spacing(translation_scale)
        translation_overrun = delta > translation_limits + translation_tolerance
        if np.any(translation_overrun):
            return WorkspaceDecision(
                False,
                "target outside anchor translation envelope",
            )
        error = quat_multiply(
            target_quaternion,
            quat_conjugate(anchor_quaternion),
        )
        rotation = float(np.linalg.norm(quat_to_rotvec(error)))
        rotation_tolerance = 16.0 * np.spacing(
            max(rotation, self.maximum_rotation_rad)
        )
        if rotation > self.maximum_rotation_rad + rotation_tolerance:
            return WorkspaceDecision(False, "target outside anchor rotation envelope")
        return WorkspaceDecision(True, "")


def _xyz(value: Any, label: str) -> tuple[float, float, float]:
    try:
        values = tuple(float(item) for item in value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label} must contain three finite values") from error
    if len(values) != 3 or not np.isfinite(np.asarray(values, dtype=float)).all():
        raise ValueError(f"{label} must contain three finite values")
    return values  # type: ignore[return-value]


def _pose_components(pose: Pose) -> tuple[np.ndarray, np.ndarray] | None:
    try:
        position = np.asarray(pose.position, dtype=float)
        quaternion = normalize_quat(pose.quaternion)
    except (AttributeError, TypeError, ValueError):
        return None
    if position.shape != (3,) or not np.isfinite(position).all():
        return None
    return position, quaternion


@dataclass(frozen=True, slots=True)
class WorkspaceLimits:
    """Explicit inclusive Cartesian bounds in the Kortex base frame."""

    minimum_xyz: tuple[float, float, float]
    maximum_xyz: tuple[float, float, float]

    def __post_init__(self) -> None:
        minimum = _xyz(self.minimum_xyz, "minimum_xyz")
        maximum = _xyz(self.maximum_xyz, "maximum_xyz")
        if any(lower > upper for lower, upper in zip(minimum, maximum)):
            raise ValueError("minimum_xyz must not exceed maximum_xyz")
        object.__setattr__(self, "minimum_xyz", minimum)
        object.__setattr__(self, "maximum_xyz", maximum)

    def evaluate(self, target: Pose) -> WorkspaceDecision:
        """Return a fail-closed decision without reading robot feedback."""

        try:
            position = np.asarray(target.position, dtype=float)
        except (AttributeError, TypeError, ValueError):
            return WorkspaceDecision(False, "target pose is non-finite")
        if position.shape != (3,) or not np.isfinite(position).all():
            return WorkspaceDecision(False, "target pose is non-finite")
        inside = np.all(position >= np.asarray(self.minimum_xyz)) and np.all(
            position <= np.asarray(self.maximum_xyz)
        )
        return WorkspaceDecision(
            bool(inside), "" if inside else "target outside workspace"
        )


def validate_target_pose(target: Pose, limits: WorkspaceLimits) -> WorkspaceDecision:
    """Small functional seam used by adapters and tests."""

    if not isinstance(limits, WorkspaceLimits):
        raise TypeError("limits must be a WorkspaceLimits")
    return limits.evaluate(target)
