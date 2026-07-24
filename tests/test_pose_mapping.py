from types import SimpleNamespace

import numpy as np
import pytest

from kinova_teleop.pose_mapping import (
    PICO_TO_WORLD,
    MappingConfig,
    Pose,
    RelativePoseMapper,
    matrix_to_quat,
    quat_from_axis_angle,
    quat_multiply,
    quat_to_matrix,
    quat_to_rotvec,
    transform_controller_pose,
)


def sample(
    position,
    quaternion=(0.0, 0.0, 0.0, 1.0),
    grip=0.0,
    stamp=1,
    received=0.0,
    valid=True,
):
    return SimpleNamespace(
        position=np.asarray(position, dtype=np.float64),
        quaternion_xyzw=np.asarray(quaternion, dtype=np.float64),
        grip=float(grip),
        timestamp_ns=int(stamp),
        received_monotonic=float(received),
        valid=bool(valid),
    )


def identity_pose(position=(0.0, 0.0, 0.0)) -> Pose:
    return Pose(
        position=np.asarray(position, dtype=np.float64),
        quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
    )


def unfiltered_config(**overrides) -> MappingConfig:
    values = {
        "filter_time_constant": 0.0,
        "max_position_step": 1.0,
        "max_rotation_step": np.pi,
    }
    values.update(overrides)
    return MappingConfig(**values)


def test_pico_translation_uses_documented_basis() -> None:
    pose = transform_controller_pose(
        np.array([1.0, 2.0, 3.0]),
        np.array([0.0, 0.0, 0.0, 1.0]),
    )

    np.testing.assert_allclose(pose.position, PICO_TO_WORLD @ [1.0, 2.0, 3.0])


def test_controller_orientation_uses_basis_conjugation() -> None:
    source_wxyz = quat_from_axis_angle(np.array([0.0, 1.0, 0.0]), np.pi / 2)
    pose = transform_controller_pose(np.zeros(3), np.roll(source_wxyz, -1))
    expected = PICO_TO_WORLD @ quat_to_matrix(source_wxyz) @ PICO_TO_WORLD.T

    np.testing.assert_allclose(quat_to_matrix(pose.quaternion), expected, atol=1e-8)


def test_quaternion_matrix_round_trip_and_composition() -> None:
    qx = quat_from_axis_angle(np.array([1.0, 0.0, 0.0]), 0.2)
    qy = quat_from_axis_angle(np.array([0.0, 1.0, 0.0]), -0.3)
    combined = quat_multiply(qx, qy)

    np.testing.assert_allclose(
        quat_to_matrix(combined),
        quat_to_matrix(qx) @ quat_to_matrix(qy),
        atol=1e-8,
    )
    round_trip = matrix_to_quat(quat_to_matrix(combined))
    np.testing.assert_allclose(quat_to_matrix(round_trip), quat_to_matrix(combined), atol=1e-8)


def test_grip_rising_edge_has_no_target_jump() -> None:
    mapper = RelativePoseMapper(unfiltered_config())
    ee = identity_pose((0.2, -0.1, 0.5))

    output = mapper.update(sample([1, 2, 3], grip=1.0, received=10.0), ee, now=10.0)

    assert output.activated
    assert output.active
    np.testing.assert_allclose(output.target.position, ee.position)
    np.testing.assert_allclose(output.target.quaternion, ee.quaternion)


def test_held_grip_maps_full_relative_pose() -> None:
    mapper = RelativePoseMapper(unfiltered_config(translation_scale=1.0))
    ee = identity_pose()
    mapper.update(sample([0, 0, 0], grip=1.0, stamp=1, received=1.0), ee, now=1.0)
    moved_wxyz = quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), 0.2)

    output = mapper.update(
        sample([0.1, 0, 0], np.roll(moved_wxyz, -1), 1.0, 2, 1.01),
        ee,
        now=1.01,
    )

    np.testing.assert_allclose(output.target.position, PICO_TO_WORLD @ [0.1, 0, 0])
    expected_rotation = PICO_TO_WORLD @ quat_to_matrix(moved_wxyz) @ PICO_TO_WORLD.T
    np.testing.assert_allclose(quat_to_matrix(output.target.quaternion), expected_rotation)
    assert np.linalg.norm(quat_to_rotvec(output.target.quaternion)) == pytest.approx(0.2)


def test_release_holds_last_target() -> None:
    mapper = RelativePoseMapper(unfiltered_config())
    ee = identity_pose()
    mapper.update(sample([0, 0, 0], grip=1.0, stamp=1, received=1.0), ee, now=1.0)
    moved = mapper.update(
        sample([0.1, 0, 0], grip=1.0, stamp=2, received=1.01),
        ee,
        now=1.01,
    )

    released = mapper.update(
        sample([9, 9, 9], grip=0.0, stamp=3, received=1.02),
        ee,
        now=1.02,
    )

    assert released.deactivated
    assert not released.active
    np.testing.assert_allclose(released.target.position, moved.target.position)
    np.testing.assert_allclose(released.target.quaternion, moved.target.quaternion)


def test_repeated_timestamp_eventually_becomes_stale_and_holds() -> None:
    mapper = RelativePoseMapper(unfiltered_config(stale_timeout=0.2))
    ee = identity_pose()
    first = mapper.update(
        sample([0, 0, 0], grip=1.0, stamp=7, received=1.0),
        ee,
        now=1.0,
    )
    duplicate = mapper.update(
        sample([1, 1, 1], grip=1.0, stamp=7, received=1.1),
        ee,
        now=1.1,
    )
    stale = mapper.update(
        sample([2, 2, 2], grip=1.0, stamp=7, received=1.3),
        ee,
        now=1.3,
    )

    assert duplicate.active
    np.testing.assert_allclose(duplicate.target.position, first.target.position)
    assert stale.stale
    assert stale.deactivated
    assert not stale.active
    np.testing.assert_allclose(stale.target.position, first.target.position)


def test_invalid_pose_deactivates_and_holds() -> None:
    mapper = RelativePoseMapper(unfiltered_config())
    ee = identity_pose()
    first = mapper.update(
        sample([0, 0, 0], grip=1.0, stamp=1, received=1.0),
        ee,
        now=1.0,
    )

    invalid = mapper.update(
        sample([np.nan, 0, 0], grip=1.0, stamp=2, received=1.01),
        ee,
        now=1.01,
    )

    assert invalid.deactivated
    assert not invalid.active
    np.testing.assert_allclose(invalid.target.position, first.target.position)


def test_position_and_rotation_steps_are_limited() -> None:
    mapper = RelativePoseMapper(
        unfiltered_config(max_position_step=0.01, max_rotation_step=0.05)
    )
    ee = identity_pose()
    mapper.update(sample([0, 0, 0], grip=1.0, stamp=1, received=1.0), ee, now=1.0)
    rotated = quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), 0.5)

    output = mapper.update(
        sample([1, 0, 0], np.roll(rotated, -1), 1.0, 2, 1.01),
        ee,
        now=1.01,
    )

    assert np.linalg.norm(output.target.position - ee.position) == pytest.approx(0.01)
    assert np.linalg.norm(quat_to_rotvec(output.target.quaternion)) == pytest.approx(0.05)
