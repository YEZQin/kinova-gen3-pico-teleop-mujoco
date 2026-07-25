from types import SimpleNamespace

import numpy as np
import pytest

from kinova_teleop.pose_mapping import (
    ClutchState,
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


@pytest.mark.parametrize(
    ("thresholds", "expected_press", "expected_release"),
    [
        pytest.param({}, 0.9, 0.8, id="defaults"),
        pytest.param(
            {"grip_press_threshold": 0.75, "grip_release_threshold": 0.25},
            0.75,
            0.25,
            id="valid-custom-hysteresis",
        ),
    ],
)
def test_mapping_config_accepts_and_preserves_valid_grip_thresholds(
    thresholds, expected_press, expected_release
) -> None:
    config = MappingConfig(**thresholds)

    assert config.grip_press_threshold == expected_press
    assert config.grip_release_threshold == expected_release


@pytest.mark.parametrize(
    "thresholds",
    [
        pytest.param({"grip_press_threshold": np.nan}, id="press-nan"),
        pytest.param({"grip_release_threshold": np.inf}, id="release-infinity"),
    ],
)
def test_mapping_config_rejects_nonfinite_grip_thresholds(thresholds) -> None:
    with pytest.raises(ValueError):
        MappingConfig(**thresholds)


@pytest.mark.parametrize(
    "thresholds",
    [
        pytest.param({"grip_press_threshold": -0.01}, id="press-below-range"),
        pytest.param({"grip_press_threshold": 1.01}, id="press-above-range"),
        pytest.param({"grip_release_threshold": -0.01}, id="release-below-range"),
        pytest.param({"grip_release_threshold": 1.01}, id="release-above-range"),
    ],
)
def test_mapping_config_rejects_out_of_range_grip_thresholds(thresholds) -> None:
    with pytest.raises(ValueError):
        MappingConfig(**thresholds)


@pytest.mark.parametrize(
    "thresholds",
    [
        pytest.param(
            {"grip_press_threshold": 0.8, "grip_release_threshold": 0.8},
            id="equal-thresholds",
        ),
        pytest.param(
            {"grip_press_threshold": 0.5, "grip_release_threshold": 0.8},
            id="inverted-thresholds",
        ),
    ],
)
def test_mapping_config_requires_release_below_press(thresholds) -> None:
    with pytest.raises(ValueError):
        MappingConfig(**thresholds)


def release_then_press(
    mapper: RelativePoseMapper,
    ee: Pose,
    *,
    start: float = 1.0,
) -> None:
    mapper.update(
        sample([0, 0, 0], grip=0.0, stamp=1, received=start),
        ee,
        now=start,
    )
    mapper.update(
        sample([0, 0, 0], grip=1.0, stamp=2, received=start + 0.01),
        ee,
        now=start + 0.01,
    )


def test_startup_pressed_grip_waits_for_release_before_activation() -> None:
    mapper = RelativePoseMapper(unfiltered_config())
    ee = identity_pose()

    pressed = mapper.update(
        sample([0, 0, 0], grip=1.0, stamp=1, received=1.0),
        ee,
        now=1.0,
    )
    released = mapper.update(
        sample([0, 0, 0], grip=0.0, stamp=2, received=1.01),
        ee,
        now=1.01,
    )
    activated = mapper.update(
        sample([0, 0, 0], grip=1.0, stamp=3, received=1.02),
        ee,
        now=1.02,
    )

    assert pressed.clutch_state is ClutchState.WAITING_FOR_RELEASE
    assert not pressed.active
    assert released.clutch_state is ClutchState.READY
    assert activated.activated and activated.active


def test_grip_hysteresis_avoids_chatter() -> None:
    mapper = RelativePoseMapper(unfiltered_config())
    ee = identity_pose()

    mapper.update(sample([0, 0, 0], grip=0.0, stamp=1), ee, now=0.0)
    mapper.update(sample([0, 0, 0], grip=1.0, stamp=2), ee, now=0.01)
    middle = mapper.update(sample([0, 0, 0], grip=0.85, stamp=3), ee, now=0.02)
    released = mapper.update(sample([0, 0, 0], grip=0.79, stamp=4), ee, now=0.03)

    assert middle.active
    assert released.clutch_state is ClutchState.READY
    assert not released.active


def test_invalid_input_requires_new_release_before_reclutch() -> None:
    mapper = RelativePoseMapper(unfiltered_config())
    ee = identity_pose()

    mapper.update(sample([0, 0, 0], grip=0.0, stamp=1), ee, now=0.0)
    mapper.update(sample([0, 0, 0], grip=1.0, stamp=2), ee, now=0.01)
    mapper.update(
        sample([0, 0, 0], grip=1.0, stamp=3, valid=False),
        ee,
        now=0.02,
    )
    still_pressed = mapper.update(
        sample([1, 0, 0], grip=1.0, stamp=4), ee, now=0.03
    )

    assert still_pressed.clutch_state is ClutchState.WAITING_FOR_RELEASE
    assert not still_pressed.active


def test_invalid_pose_forces_release_gate_even_with_released_grip() -> None:
    mapper = RelativePoseMapper(unfiltered_config())
    ee = identity_pose()
    release_then_press(mapper, ee)

    invalid = mapper.update(
        sample([np.nan, 0, 0], grip=0.0, stamp=3, received=1.02),
        ee,
        now=1.02,
    )

    assert invalid.clutch_state is ClutchState.WAITING_FOR_RELEASE
    assert not invalid.active


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

    mapper.update(sample([0, 0, 0], grip=0.0, stamp=1, received=10.0), ee, now=10.0)
    output = mapper.update(
        sample([1, 2, 3], grip=1.0, stamp=2, received=10.01), ee, now=10.01
    )

    assert output.activated
    assert output.active
    np.testing.assert_allclose(output.target.position, ee.position)
    np.testing.assert_allclose(output.target.quaternion, ee.quaternion)


def test_held_grip_maps_full_relative_pose() -> None:
    mapper = RelativePoseMapper(unfiltered_config(translation_scale=1.0))
    ee = identity_pose()
    release_then_press(mapper, ee)
    moved_wxyz = quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), 0.2)

    output = mapper.update(
        sample([0.1, 0, 0], np.roll(moved_wxyz, -1), 1.0, 3, 1.02),
        ee,
        now=1.02,
    )

    np.testing.assert_allclose(output.target.position, PICO_TO_WORLD @ [0.1, 0, 0])
    expected_rotation = PICO_TO_WORLD @ quat_to_matrix(moved_wxyz) @ PICO_TO_WORLD.T
    np.testing.assert_allclose(quat_to_matrix(output.target.quaternion), expected_rotation)
    assert np.linalg.norm(quat_to_rotvec(output.target.quaternion)) == pytest.approx(0.2)


def test_release_holds_last_target() -> None:
    mapper = RelativePoseMapper(unfiltered_config())
    ee = identity_pose()
    release_then_press(mapper, ee)
    moved = mapper.update(
        sample([0.1, 0, 0], grip=1.0, stamp=3, received=1.02),
        ee,
        now=1.02,
    )

    released = mapper.update(
        sample([9, 9, 9], grip=0.0, stamp=4, received=1.03),
        ee,
        now=1.03,
    )

    assert released.deactivated
    assert not released.active
    np.testing.assert_allclose(released.target.position, moved.target.position)
    np.testing.assert_allclose(released.target.quaternion, moved.target.quaternion)


def test_repeated_timestamp_eventually_becomes_stale_and_holds() -> None:
    mapper = RelativePoseMapper(unfiltered_config(stale_timeout=0.2))
    ee = identity_pose()
    release_then_press(mapper, ee, start=0.0)
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
    release_then_press(mapper, ee)
    first = mapper.update(
        sample([0, 0, 0], grip=1.0, stamp=3, received=1.02),
        ee,
        now=1.02,
    )

    invalid = mapper.update(
        sample([np.nan, 0, 0], grip=1.0, stamp=4, received=1.03),
        ee,
        now=1.03,
    )

    assert invalid.deactivated
    assert not invalid.active
    np.testing.assert_allclose(invalid.target.position, first.target.position)


def test_restarted_source_timestamp_can_release_and_rearm_after_loss() -> None:
    """A lower process-relative timestamp must recover through the release gate."""
    mapper = RelativePoseMapper(unfiltered_config(stale_timeout=0.2))
    ee = identity_pose()
    mapper.update(
        sample([0, 0, 0], grip=0.0, stamp=100, received=1.0),
        ee,
        now=1.0,
    )
    mapper.update(
        sample([0, 0, 0], grip=1.0, stamp=101, received=1.01),
        ee,
        now=1.01,
    )
    lost = mapper.update(
        sample([0, 0, 0], grip=1.0, stamp=0, received=1.02, valid=False),
        ee,
        now=1.02,
    )
    assert lost.clutch_state is ClutchState.WAITING_FOR_RELEASE

    held = mapper.update(
        sample([0, 0, 0], grip=1.0, stamp=1, received=2.0),
        ee,
        now=2.0,
    )
    assert not held.stale
    assert held.clutch_state is ClutchState.WAITING_FOR_RELEASE
    assert not held.active

    released = mapper.update(
        sample([0, 0, 0], grip=0.0, stamp=2, received=2.01),
        ee,
        now=2.01,
    )
    assert released.clutch_state is ClutchState.READY

    repressed = mapper.update(
        sample([0, 0, 0], grip=1.0, stamp=3, received=2.02),
        ee,
        now=2.02,
    )
    assert repressed.activated
    assert repressed.clutch_state is ClutchState.ACTIVE


def test_position_and_rotation_steps_are_limited() -> None:
    mapper = RelativePoseMapper(
        unfiltered_config(max_position_step=0.01, max_rotation_step=0.05)
    )
    ee = identity_pose()
    release_then_press(mapper, ee)
    rotated = quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), 0.5)

    output = mapper.update(
        sample([1, 0, 0], np.roll(rotated, -1), 1.0, 3, 1.02),
        ee,
        now=1.02,
    )

    assert np.linalg.norm(output.target.position - ee.position) == pytest.approx(0.01)
    assert np.linalg.norm(quat_to_rotvec(output.target.quaternion)) == pytest.approx(0.05)
