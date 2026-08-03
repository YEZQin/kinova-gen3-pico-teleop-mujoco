from __future__ import annotations

import numpy as np

from kinova_teleop.pose_mapping import Pose
from kinova_teleop.workspace import WorkspaceLimits, validate_target_pose


def pose_at(x: float, y: float, z: float) -> Pose:
    return Pose(np.array([x, y, z], dtype=float), np.array([1.0, 0.0, 0.0, 0.0]))


def test_workspace_rejects_nonfinite_and_outside_targets() -> None:
    limits = WorkspaceLimits((-0.2, -0.2, 0.1), (0.2, 0.2, 0.6))
    assert limits.evaluate(pose_at(0.0, 0.0, 0.3)).accepted
    assert limits.evaluate(pose_at(0.21, 0.0, 0.3)).reason == "target outside workspace"
    assert limits.evaluate(pose_at(float("nan"), 0.0, 0.3)).reason == "target pose is non-finite"
    assert validate_target_pose(pose_at(0.0, 0.0, 0.3), limits).accepted


def test_workspace_rejects_invalid_bounds_and_malformed_pose() -> None:
    import pytest

    with pytest.raises(ValueError):
        WorkspaceLimits((0.0, 0.0, 1.0), (0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        WorkspaceLimits((0.0, 0.0), (0.0, 0.0, 1.0))
    limits = WorkspaceLimits((-1.0, -1.0, -1.0), (1.0, 1.0, 1.0))
    assert limits.evaluate(None).reason == "target pose is non-finite"
    assert limits.evaluate(Pose(np.zeros(2), np.array([1.0, 0.0, 0.0, 0.0]))).reason == "target pose is non-finite"
    with pytest.raises(TypeError):
        validate_target_pose(pose_at(0.0, 0.0, 0.0), object())
