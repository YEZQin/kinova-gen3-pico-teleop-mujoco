from __future__ import annotations

import math

import numpy as np
import pytest

from kinova_teleop.pose_mapping import Pose, quat_from_axis_angle
from kinova_teleop.workspace import (
    AnchorEnvelope,
    WorkspaceLimits,
    validate_target_pose,
)


def pose_at(x: float, y: float, z: float) -> Pose:
    return Pose(np.array([x, y, z], dtype=float), np.array([1.0, 0.0, 0.0, 0.0]))


def test_workspace_rejects_nonfinite_and_outside_targets() -> None:
    limits = WorkspaceLimits((-0.2, -0.2, 0.1), (0.2, 0.2, 0.6))
    assert limits.evaluate(pose_at(0.0, 0.0, 0.3)).accepted
    assert limits.evaluate(pose_at(0.21, 0.0, 0.3)).reason == "target outside workspace"
    assert limits.evaluate(pose_at(float("nan"), 0.0, 0.3)).reason == "target pose is non-finite"
    assert validate_target_pose(pose_at(0.0, 0.0, 0.3), limits).accepted


def test_workspace_rejects_invalid_bounds_and_malformed_pose() -> None:
    with pytest.raises(ValueError):
        WorkspaceLimits((0.0, 0.0, 1.0), (0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        WorkspaceLimits((0.0, 0.0), (0.0, 0.0, 1.0))
    limits = WorkspaceLimits((-1.0, -1.0, -1.0), (1.0, 1.0, 1.0))
    assert limits.evaluate(None).reason == "target pose is non-finite"
    assert limits.evaluate(Pose(np.zeros(2), np.array([1.0, 0.0, 0.0, 0.0]))).reason == "target pose is non-finite"
    with pytest.raises(TypeError):
        validate_target_pose(pose_at(0.0, 0.0, 0.0), object())


def test_anchor_envelope_accepts_boundary_and_rejects_translation_overrun() -> None:
    envelope = AnchorEnvelope((0.02, 0.02, 0.02), math.radians(5.0))
    anchor = pose_at(0.10, -0.20, 0.30)

    assert envelope.evaluate(anchor, pose_at(0.12, -0.18, 0.28)).accepted
    rejected = envelope.evaluate(anchor, pose_at(0.120001, -0.20, 0.30))

    assert not rejected.accepted
    assert rejected.reason == "target outside anchor translation envelope"


def test_expanded_anchor_envelope_accepts_fifty_millimetres_and_rejects_overrun() -> None:
    envelope = AnchorEnvelope((0.05, 0.05, 0.05), math.radians(5.0))
    anchor = pose_at(0.10, -0.20, 0.30)

    assert envelope.evaluate(anchor, pose_at(0.15, -0.20, 0.30)).accepted
    rejected = envelope.evaluate(anchor, pose_at(0.150001, -0.20, 0.30))

    assert not rejected.accepted
    assert rejected.reason == "target outside anchor translation envelope"


def test_anchor_envelope_uses_shortest_arc_orientation() -> None:
    envelope = AnchorEnvelope((0.02, 0.02, 0.02), math.radians(5.0))
    anchor = Pose(
        np.zeros(3),
        quat_from_axis_angle(np.array([0, 0, 1]), 0.0),
    )
    inside = Pose(
        np.zeros(3),
        quat_from_axis_angle(np.array([0, 0, 1]), math.radians(355.0)),
    )
    outside = Pose(
        np.zeros(3),
        quat_from_axis_angle(np.array([0, 0, 1]), math.radians(5.01)),
    )

    assert envelope.evaluate(anchor, inside).accepted
    rejected = envelope.evaluate(anchor, outside)
    assert not rejected.accepted
    assert rejected.reason == "target outside anchor rotation envelope"


def test_anchor_envelope_rejects_invalid_limits_and_poses() -> None:
    with pytest.raises(ValueError, match="positive finite"):
        AnchorEnvelope((0.02, 0.0, 0.02), math.radians(5.0))
    with pytest.raises(ValueError, match="positive finite"):
        AnchorEnvelope((0.02, 0.02, 0.02), float("nan"))

    envelope = AnchorEnvelope((0.02, 0.02, 0.02), math.radians(5.0))
    invalid_anchor = Pose(np.array([0.0, np.nan, 0.0]), np.array([1, 0, 0, 0]))
    invalid_target = Pose(np.zeros(3), np.zeros(4))

    assert not envelope.evaluate(invalid_anchor, pose_at(0.0, 0.0, 0.0)).accepted
    assert not envelope.evaluate(pose_at(0.0, 0.0, 0.0), invalid_target).accepted
