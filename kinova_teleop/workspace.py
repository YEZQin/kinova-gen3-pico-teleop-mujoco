"""Immutable Cartesian workspace validation for guarded Gen3 motion."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .pose_mapping import Pose


@dataclass(frozen=True, slots=True)
class WorkspaceDecision:
    """Result of evaluating one target pose against explicit XYZ bounds."""

    accepted: bool
    reason: str = ""


def _xyz(value: Any, label: str) -> tuple[float, float, float]:
    try:
        values = tuple(float(item) for item in value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label} must contain three finite values") from error
    if len(values) != 3 or not np.isfinite(np.asarray(values, dtype=float)).all():
        raise ValueError(f"{label} must contain three finite values")
    return values  # type: ignore[return-value]


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
