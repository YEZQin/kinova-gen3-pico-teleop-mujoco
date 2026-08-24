from types import SimpleNamespace

import numpy as np
import pytest

from kinova_teleop.pose_mapping import (
    ClutchState,
    InputFault,
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
    invalid_reason="",
):
    return SimpleNamespace(
        position=np.asarray(position, dtype=np.float64),
        quaternion_xyzw=np.asarray(quaternion, dtype=np.float64),
        grip=float(grip),
        timestamp_ns=int(stamp),
        received_monotonic=float(received),
        valid=bool(valid),
        invalid_reason=str(invalid_reason),
    )


@pytest.mark.parametrize(
    ("reason", "expected"),
    [
        pytest.param("stream is stale", InputFault.STALE, id="stale"),
        pytest.param("source changed", InputFault.SOURCE_CHANGED, id="source-changed"),
        pytest.param("controller is untracked", InputFault.INVALID, id="invalid"),
    ],
)
def test_invalid_sample_reason_maps_to_stable_input_fault(reason, expected) -> None:
    mapper = RelativePoseMapper(unfiltered_config())
    ee = identity_pose()
    mapper.reset(ee)

    output = mapper.update(
        sample([0.0, 0.0, 0.0], valid=False, invalid_reason=reason),
        ee,
        now=0.0,
    )

    assert output.input_fault is expected


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


CALIBRATED_TRANSLATION_ROTATION = (
    (0.861288411785, 0.469277688168, -0.194835117758),
    (-0.470679958298, 0.881303368425, 0.042008923554),
    (0.191422696095, 0.055523186053, 0.979935981190),
)


def move_after_anchor(
    config: MappingConfig,
    raw_delta: list[float],
    ee: Pose,
) -> Pose:
    mapper = RelativePoseMapper(config)
    release_then_press(mapper, ee)
    return mapper.update(
        sample(raw_delta, grip=1.0, stamp=3, received=1.02),
        ee,
        now=1.02,
    ).target


@pytest.mark.parametrize(
    ("raw_delta", "expected_base"),
    [
        pytest.param(
            [0.881303368425, -0.042008923554, -0.470679958298],
            [0.0, -1.0, 0.0],
            id="right",
        ),
        pytest.param(
            [-0.055523186053, 0.979935981190, -0.191422696095],
            [0.0, 0.0, 1.0],
            id="up",
        ),
        pytest.param(
            [0.469277688168, 0.194835117758, 0.861288411785],
            [-1.0, 0.0, 0.0],
            id="forward",
        ),
    ],
)
def test_operator_calibration_maps_measured_directions(
    raw_delta, expected_base
) -> None:
    ee = Pose(
        np.array([0.4, -0.2, 0.3], dtype=np.float64),
        quat_from_axis_angle(np.array([1.0, 0.0, 0.0]), 0.4),
    )

    target = move_after_anchor(
        unfiltered_config(
            orientation_enabled=False,
            translation_rotation=CALIBRATED_TRANSLATION_ROTATION,
        ),
        raw_delta,
        ee,
    )

    np.testing.assert_allclose(
        (target.position - ee.position) / np.linalg.norm(target.position - ee.position),
        expected_base,
        atol=0.11,
    )
    np.testing.assert_allclose(
        quat_to_matrix(target.quaternion), quat_to_matrix(ee.quaternion), atol=1e-8
    )


def test_translation_axis_gain_boosts_calibrated_base_x_only() -> None:
    ee = identity_pose((0.4, -0.2, 0.3))
    raw_forward = [0.046927768817, 0.019483511776, 0.086128841179]
    baseline = move_after_anchor(
        unfiltered_config(
            orientation_enabled=False,
            translation_rotation=CALIBRATED_TRANSLATION_ROTATION,
        ),
        raw_forward,
        ee,
    )
    boosted = move_after_anchor(
        unfiltered_config(
            orientation_enabled=False,
            translation_rotation=CALIBRATED_TRANSLATION_ROTATION,
            translation_axis_gain=(2.0, 1.0, 1.0),
        ),
        raw_forward,
        ee,
    )

    baseline_delta = baseline.position - ee.position
    boosted_delta = boosted.position - ee.position
    np.testing.assert_allclose(
        boosted_delta,
        baseline_delta * np.array([2.0, 1.0, 1.0]),
        atol=1e-9,
    )


@pytest.mark.parametrize(
    "translation_rotation",
    [
        pytest.param(((1.0, 0.0), (0.0, 1.0)), id="wrong-shape"),
        pytest.param(((np.nan, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)), id="nonfinite"),
        pytest.param(((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 2.0)), id="nonorthogonal"),
        pytest.param(((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0)), id="reflection"),
    ],
)
def test_mapping_config_rejects_invalid_translation_rotation(
    translation_rotation,
) -> None:
    with pytest.raises(ValueError):
        MappingConfig(translation_rotation=translation_rotation)


def test_inversion_is_applied_after_operator_translation_rotation() -> None:
    ee = identity_pose((0.4, -0.2, 0.3))
    target = move_after_anchor(
        unfiltered_config(
            orientation_enabled=False,
            invert_translation=True,
            translation_rotation=CALIBRATED_TRANSLATION_ROTATION,
        ),
        [0.881303368425, -0.042008923554, -0.470679958298],
        ee,
    )

    direction = target.position - ee.position
    direction /= np.linalg.norm(direction)
    np.testing.assert_allclose(direction, [0.0, 1.0, 0.0], atol=1e-9)


@pytest.mark.parametrize(
    "mutable_rotation",
    [
        pytest.param(
            [list(row) for row in CALIBRATED_TRANSLATION_ROTATION],
            id="nested-list",
        ),
        pytest.param(np.array(CALIBRATED_TRANSLATION_ROTATION), id="ndarray"),
    ],
)
def test_mapping_config_snapshots_mutable_translation_rotation(
    mutable_rotation,
) -> None:
    mapper = RelativePoseMapper(
        unfiltered_config(
            orientation_enabled=False,
            translation_rotation=mutable_rotation,
        )
    )
    mutable_rotation[0][0] = 0.0
    ee = identity_pose()

    target = move_after_anchor(
        mapper.config,
        [0.881303368425, -0.042008923554, -0.470679958298],
        ee,
    )

    assert isinstance(mapper.config.translation_rotation, tuple)
    assert all(isinstance(row, tuple) for row in mapper.config.translation_rotation)
    np.testing.assert_allclose(
        target.position - ee.position,
        [0.0, -1.0, 0.0],
        atol=1e-9,
    )


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


def test_translation_only_mapping_keeps_anchor_orientation() -> None:
    mapper = RelativePoseMapper(
        unfiltered_config(translation_scale=1.0, orientation_enabled=False)
    )
    ee = Pose(
        np.array([0.4, -0.2, 0.3], dtype=np.float64),
        quat_from_axis_angle(np.array([1.0, 0.0, 0.0]), 0.4),
    )
    release_then_press(mapper, ee)
    moved_wxyz = quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), 0.6)

    output = mapper.update(
        sample([0.1, 0.0, 0.0], np.roll(moved_wxyz, -1), 1.0, 3, 1.02),
        ee,
        now=1.02,
    )

    np.testing.assert_allclose(
        output.target.position,
        ee.position + PICO_TO_WORLD @ [0.1, 0.0, 0.0],
    )
    np.testing.assert_allclose(
        quat_to_matrix(output.target.quaternion),
        quat_to_matrix(ee.quaternion),
        atol=1e-8,
    )


@pytest.mark.parametrize(
    "controller_delta",
    (
        pytest.param([0.1, 0.0, 0.0], id="pico-x"),
        pytest.param([0.0, 0.1, 0.0], id="pico-y"),
        pytest.param([0.0, 0.0, 0.1], id="pico-z"),
    ),
)
def test_inverted_translation_negates_every_mapped_axis_and_keeps_anchor_orientation(
    controller_delta,
) -> None:
    mapper = RelativePoseMapper(
        unfiltered_config(
            translation_scale=0.8,
            orientation_enabled=False,
            invert_translation=True,
        )
    )
    ee = Pose(
        np.array([0.4, -0.2, 0.3], dtype=np.float64),
        quat_from_axis_angle(np.array([1.0, 0.0, 0.0]), 0.4),
    )
    release_then_press(mapper, ee)

    output = mapper.update(
        sample(controller_delta, grip=1.0, stamp=3, received=1.02),
        ee,
        now=1.02,
    )

    expected_position = ee.position - 0.8 * (
        PICO_TO_WORLD @ np.asarray(controller_delta, dtype=np.float64)
    )
    np.testing.assert_allclose(output.target.position, expected_position)
    np.testing.assert_allclose(
        quat_to_matrix(output.target.quaternion),
        quat_to_matrix(ee.quaternion),
        atol=1e-8,
    )


def test_waiting_for_release_requires_configured_distinct_released_samples() -> None:
    mapper = RelativePoseMapper(unfiltered_config(release_stability_samples=3))
    ee = identity_pose()
    mapper.reset(ee)

    first = mapper.update(sample([0, 0, 0], grip=0.0, stamp=1), ee, now=0.00)
    duplicate = mapper.update(sample([0, 0, 0], grip=0.0, stamp=1), ee, now=0.01)
    second = mapper.update(sample([0, 0, 0], grip=0.0, stamp=2), ee, now=0.02)
    pressed = mapper.update(sample([0, 0, 0], grip=1.0, stamp=3), ee, now=0.03)
    restarted = mapper.update(sample([0, 0, 0], grip=0.0, stamp=4), ee, now=0.04)
    mapper.update(sample([0, 0, 0], grip=0.0, stamp=5), ee, now=0.05)
    ready = mapper.update(sample([0, 0, 0], grip=0.0, stamp=6), ee, now=0.06)

    assert first.clutch_state is ClutchState.WAITING_FOR_RELEASE
    assert duplicate.clutch_state is ClutchState.WAITING_FOR_RELEASE
    assert second.clutch_state is ClutchState.WAITING_FOR_RELEASE
    assert pressed.clutch_state is ClutchState.WAITING_FOR_RELEASE
    assert restarted.clutch_state is ClutchState.WAITING_FOR_RELEASE
    assert ready.clutch_state is ClutchState.READY


def test_confirmed_boundary_stop_requires_release_before_reanchoring() -> None:
    mapper = RelativePoseMapper(unfiltered_config(release_stability_samples=3))
    original_anchor = identity_pose((0.1, -0.2, 0.3))
    mapper.reset(original_anchor)

    for timestamp in (1, 2, 3):
        mapper.update(
            sample([0, 0, 0], grip=0.0, stamp=timestamp, received=float(timestamp)),
            original_anchor,
            now=float(timestamp),
        )
    mapper.update(
        sample([0, 0, 0], grip=1.0, stamp=4, received=4.0),
        original_anchor,
        now=4.0,
    )

    output = mapper.require_release()

    assert output.clutch_state is ClutchState.WAITING_FOR_RELEASE
    assert output.active is False
    assert output.deactivated is True

    pressed = mapper.update(
        sample([9, 9, 9], grip=1.0, stamp=5, received=5.0),
        original_anchor,
        now=5.0,
    )
    assert pressed.clutch_state is ClutchState.WAITING_FOR_RELEASE
    assert not pressed.active

    for timestamp in (6, 7):
        waiting = mapper.update(
            sample([9, 9, 9], grip=0.0, stamp=timestamp, received=float(timestamp)),
            original_anchor,
            now=float(timestamp),
        )
        assert waiting.clutch_state is ClutchState.WAITING_FOR_RELEASE
    ready = mapper.update(
        sample([9, 9, 9], grip=0.0, stamp=8, received=8.0),
        original_anchor,
        now=8.0,
    )
    assert ready.clutch_state is ClutchState.READY

    reanchored_pose = identity_pose((0.4, 0.5, 0.6))
    activation = mapper.update(
        sample([5, 5, 5], grip=1.0, stamp=9, received=9.0),
        reanchored_pose,
        now=9.0,
    )
    moved = mapper.update(
        sample([5.01, 5, 5], grip=1.0, stamp=10, received=10.0),
        reanchored_pose,
        now=10.0,
    )

    assert activation.activated
    np.testing.assert_allclose(activation.target.position, reanchored_pose.position)
    np.testing.assert_allclose(
        moved.target.position,
        reanchored_pose.position + np.array([0.0, -0.01, 0.0]),
    )


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


class CountingAnchor:
    """Callable anchor that records how often activation resolves it."""

    def __init__(self, pose, error=None):
        self.pose = pose
        self.error = error
        self.calls = 0

    def __call__(self):
        self.calls += 1
        if self.error is not None:
            raise self.error
        return self.pose


def test_deferred_anchor_resolves_exactly_once_per_activation() -> None:
    mapper = RelativePoseMapper(unfiltered_config())
    mapper.reset(identity_pose())
    anchor = CountingAnchor(identity_pose((0.4, -0.2, 0.3)))

    mapper.update(sample([0, 0, 0], grip=0.0, stamp=1), anchor, now=0.0)
    assert anchor.calls == 0

    activated = mapper.update(sample([0, 0, 0], grip=1.0, stamp=2), anchor, now=0.01)
    assert activated.activated
    assert anchor.calls == 1
    np.testing.assert_allclose(activated.target.position, [0.4, -0.2, 0.3])

    mapper.update(sample([0.01, 0, 0], grip=1.0, stamp=3), anchor, now=0.02)
    mapper.update(sample([0.01, 0, 0], grip=1.0, stamp=4), anchor, now=0.03)
    assert anchor.calls == 1

    # Release and re-clutch: the anchor resolves exactly once more.
    mapper.update(sample([0.01, 0, 0], grip=0.0, stamp=5), anchor, now=0.04)
    mapper.update(sample([0.01, 0, 0], grip=1.0, stamp=6), anchor, now=0.05)
    assert anchor.calls == 2


def test_deferred_anchor_failure_leaves_clutch_ready_and_retryable() -> None:
    mapper = RelativePoseMapper(unfiltered_config())
    mapper.reset(identity_pose())
    failing = CountingAnchor(None, error=RuntimeError("begin_control rejected"))

    mapper.update(sample([0, 0, 0], grip=0.0, stamp=1), failing, now=0.0)
    with pytest.raises(RuntimeError, match="begin_control rejected"):
        mapper.update(sample([0, 0, 0], grip=1.0, stamp=2), failing, now=0.01)

    # The clutch state machine must be untouched: still READY, no half
    # captured references, and a later working anchor can activate.
    assert mapper.clutch_state is ClutchState.READY
    assert mapper._controller_reference is None
    assert mapper._ee_reference is None

    working = CountingAnchor(identity_pose((0.1, 0.2, 0.3)))
    activated = mapper.update(sample([0, 0, 0], grip=1.0, stamp=3), working, now=0.02)
    assert activated.activated
    np.testing.assert_allclose(activated.target.position, [0.1, 0.2, 0.3])


def test_deferred_anchor_requires_reset_before_update() -> None:
    mapper = RelativePoseMapper(unfiltered_config())
    anchor = CountingAnchor(identity_pose())

    with pytest.raises(RuntimeError, match="reset"):
        mapper.update(sample([0, 0, 0], grip=0.0, stamp=1), anchor, now=0.0)
    assert anchor.calls == 0
