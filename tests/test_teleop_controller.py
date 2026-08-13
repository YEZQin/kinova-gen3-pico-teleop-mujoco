from collections.abc import Iterable

import mujoco
import numpy as np
import pytest

from kinova_teleop.mujoco_backend import MuJoCoBackend
from kinova_teleop.pose_mapping import ClutchState, Pose
from kinova_teleop.teleop_controller import (
    TeleopConfig,
    TeleopController,
    TeleopSafetyError,
)
from kinova_teleop.xr_input import ControllerSample, DryRunXrInput


class ScriptedInput:
    def __init__(self, samples: Iterable[ControllerSample]) -> None:
        self._samples = iter(samples)
        self.close_calls = 0

    def read(self) -> ControllerSample:
        return next(self._samples)

    def close(self) -> None:
        self.close_calls += 1

    @property
    def closed(self) -> bool:
        return self.close_calls > 0


class RecordingBackend:
    def __init__(self) -> None:
        self.pose = Pose(
            np.array([0.0, 0.0, 0.5]),
            np.array([1.0, 0.0, 0.0, 0.0]),
        )
        self.begin_calls = 0
        self.commands = []
        self.holds = 0
        self.steps = 0
        self.close_calls = 0

    def current_pose(self):
        return self.pose

    def begin_control(self):
        self.begin_calls += 1

    def command_pose(self, target):
        from kinova_teleop.backend import BackendResult

        self.commands.append(target)
        return BackendResult(True, True, 0.0, 0.0, "")

    def hold(self):
        self.holds += 1

    def step(self):
        self.steps += 1

    def close(self):
        self.close_calls += 1

    @property
    def closed(self) -> bool:
        return self.close_calls > 0


def sample(
    position: list[float],
    grip: float,
    timestamp_ns: int,
    received: float,
    *,
    valid: bool = True,
    invalid_reason: str = "",
) -> ControllerSample:
    return ControllerSample(
        position=np.asarray(position, dtype=np.float64),
        quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        grip=grip,
        timestamp_ns=timestamp_ns,
        received_monotonic=received,
        valid=valid,
        invalid_reason=invalid_reason,
    )


def test_hardware_stale_input_stops_and_escapes_loop() -> None:
    backend = RecordingBackend()
    source = ScriptedInput(
        [
            sample([0.0, 0.0, 0.0], 0.0, 1, 1.00),
            sample([0.0, 0.0, 0.0], 1.0, 2, 1.01),
            sample(
                [0.0, 0.0, 0.0],
                0.0,
                0,
                1.02,
                valid=False,
                invalid_reason="stream is stale",
            ),
        ]
    )
    controller = TeleopController(
        TeleopConfig(realtime=False, fatal_input_faults=True),
        source,
        backend,
    )

    controller.step_once()
    controller.step_once()
    with pytest.raises(TeleopSafetyError, match="stale"):
        controller.step_once()

    assert backend.holds == 1


def test_recoverable_stale_stops_once_and_requires_fresh_release_streak() -> None:
    backend = RecordingBackend()
    source = ScriptedInput(
        [
            sample([0.0, 0.0, 0.0], 0.0, 1, 1.00),
            sample([0.0, 0.0, 0.0], 0.0, 2, 1.01),
            sample([0.0, 0.0, 0.0], 0.0, 3, 1.02),
            sample([0.0, 0.0, 0.0], 1.0, 4, 1.03),
            sample([0.1, 0.0, 0.0], 1.0, 5, 1.04),
            sample([0.0, 0.0, 0.0], 0.0, 0, 1.05, valid=False, invalid_reason="stream is stale"),
            sample([0.0, 0.0, 0.0], 0.0, 0, 1.06, valid=False, invalid_reason="stream is stale"),
            sample([0.0, 0.0, 0.0], 0.0, 10, 1.07),
            sample([0.0, 0.0, 0.0], 0.0, 10, 1.08),
            sample([0.0, 0.0, 0.0], 0.0, 11, 1.09),
            sample([0.0, 0.0, 0.0], 1.0, 12, 1.10),
            sample([0.0, 0.0, 0.0], 0.0, 13, 1.11),
            sample([0.0, 0.0, 0.0], 0.0, 14, 1.12),
            sample([0.0, 0.0, 0.0], 0.0, 15, 1.13),
        ]
    )
    controller = TeleopController(
        TeleopConfig(
            realtime=False,
            fatal_input_faults=True,
            recover_stale_input=True,
            recovery_release_samples=3,
        ),
        source,
        backend,
    )

    diagnostics = [controller.step_once() for _ in range(14)]

    assert backend.holds == 1
    assert diagnostics[5].stale is True
    assert diagnostics[6].clutch_state is ClutchState.WAITING_FOR_RELEASE
    assert diagnostics[9].clutch_state is ClutchState.WAITING_FOR_RELEASE
    assert diagnostics[10].clutch_state is ClutchState.WAITING_FOR_RELEASE
    assert diagnostics[12].clutch_state is ClutchState.WAITING_FOR_RELEASE
    assert diagnostics[13].clutch_state is ClutchState.READY


def test_stale_recovery_reanchors_before_any_new_motion_command() -> None:
    backend = RecordingBackend()
    source = ScriptedInput(
        [
            sample([0.0, 0.0, 0.0], 0.0, 1, 1.00),
            sample([0.0, 0.0, 0.0], 0.0, 2, 1.01),
            sample([0.0, 0.0, 0.0], 0.0, 3, 1.02),
            sample([0.0, 0.0, 0.0], 1.0, 4, 1.03),
            sample([0.1, 0.0, 0.0], 1.0, 5, 1.04),
            sample([0.0, 0.0, 0.0], 0.0, 0, 1.05, valid=False, invalid_reason="stream is stale"),
            sample([5.0, 5.0, 5.0], 0.0, 10, 1.06),
            sample([5.0, 5.0, 5.0], 0.0, 11, 1.07),
            sample([5.0, 5.0, 5.0], 0.0, 12, 1.08),
            sample([5.0, 5.0, 5.0], 1.0, 13, 1.09),
            sample([5.01, 5.0, 5.0], 1.0, 14, 1.10),
        ]
    )
    controller = TeleopController(
        TeleopConfig(
            realtime=False,
            fatal_input_faults=True,
            recover_stale_input=True,
            recovery_release_samples=3,
        ),
        source,
        backend,
    )

    for _ in range(9):
        controller.step_once()
    backend.pose = Pose(
        np.array([0.2, -0.1, 0.6]),
        np.array([1.0, 0.0, 0.0, 0.0]),
    )
    activation = controller.step_once()
    moved = controller.step_once()

    assert activation.clutch_state is ClutchState.ACTIVE
    assert backend.begin_calls == 2
    assert len(backend.commands) == 2
    reanchored_target = backend.commands[-1].position
    assert reanchored_target[0] == pytest.approx(backend.pose.position[0])
    assert reanchored_target[2] == pytest.approx(backend.pose.position[2])
    assert backend.pose.position[1] - 0.005 < reanchored_target[1] < backend.pose.position[1]
    assert moved.active


def test_normal_release_remains_recoverable_in_hardware_policy() -> None:
    backend = RecordingBackend()
    source = ScriptedInput(
        [
            sample([0.0, 0.0, 0.0], 0.0, 1, 1.00),
            sample([0.0, 0.0, 0.0], 1.0, 2, 1.01),
            sample([0.0, 0.0, 0.0], 0.0, 3, 1.02),
            sample([0.0, 0.0, 0.0], 1.0, 4, 1.03),
        ]
    )
    controller = TeleopController(
        TeleopConfig(realtime=False, fatal_input_faults=True),
        source,
        backend,
    )

    controller.step_once()
    controller.step_once()
    released = controller.step_once()

    assert released.clutch_state is ClutchState.READY
    assert backend.holds == 1
    assert controller.step_once().active is True


@pytest.mark.parametrize(
    "invalid_reason",
    (
        pytest.param("source changed", id="source-changed"),
        pytest.param("controller is untracked", id="untracked"),
    ),
)
def test_nonstale_input_fault_is_fatal_during_stale_recovery(
    invalid_reason: str,
) -> None:
    invalid = sample(
        [0.0, 0.0, 0.0],
        0.0,
        0,
        1.0,
        valid=False,
        invalid_reason=invalid_reason,
    )
    recoverable = TeleopController(
        TeleopConfig(realtime=False),
        ScriptedInput([invalid]),
        RecordingBackend(),
    )

    assert recoverable.step_once().active is False

    fatal_backend = RecordingBackend()
    fatal = TeleopController(
        TeleopConfig(
            realtime=False,
            fatal_input_faults=True,
            recover_stale_input=True,
        ),
        ScriptedInput([invalid]),
        fatal_backend,
    )
    expected_reason = "source_changed" if invalid_reason == "source changed" else "invalid"
    with pytest.raises(TeleopSafetyError, match=expected_reason):
        fatal.step_once()
    assert fatal_backend.holds == 1


def test_fatal_input_reports_unconfirmed_stop_failure() -> None:
    class FailingHoldBackend(RecordingBackend):
        def hold(self):
            self.holds += 1
            raise RuntimeError("stop RPC failed")

    invalid = sample(
        [0.0, 0.0, 0.0],
        0.0,
        0,
        1.0,
        valid=False,
        invalid_reason="stream is stale",
    )
    backend = FailingHoldBackend()
    controller = TeleopController(
        TeleopConfig(
            realtime=False,
            fatal_input_faults=True,
            recover_stale_input=True,
        ),
        ScriptedInput(
            [
                sample([0.0, 0.0, 0.0], 0.0, 1, 0.98),
                sample([0.0, 0.0, 0.0], 1.0, 2, 0.99),
                invalid,
            ]
        ),
        backend,
    )

    controller.step_once()
    controller.step_once()
    with pytest.raises(
        TeleopSafetyError,
        match="Stop attempted but unconfirmed",
    ) as captured:
        controller.step_once()

    assert isinstance(captured.value.__cause__, RuntimeError)
    assert backend.holds == 1


def test_headless_dry_run_stays_finite(teleop_model_path) -> None:
    source = DryRunXrInput(control_hz=100.0)
    backend = MuJoCoBackend(teleop_model_path, control_hz=100.0)
    controller = TeleopController(
        TeleopConfig(realtime=False),
        source,
        backend,
    )
    try:
        controller.run(max_steps=500)
        assert np.isfinite(backend.data.qpos).all()
        assert np.isfinite(backend.data.qvel).all()
        assert np.isfinite(backend.data.ctrl).all()
        assert controller.steps == 500
    finally:
        controller.close()


def test_release_holds_last_actuator_target(teleop_model_path) -> None:
    samples = [
        sample([0.0, 0.0, 0.0], 0.0, 1, 1.00),
        sample([0.0, 0.0, 0.0], 1.0, 2, 1.01),
        sample([0.05, 0.0, 0.0], 1.0, 3, 1.02),
        sample([1.0, 1.0, 1.0], 0.0, 4, 1.03),
        sample([2.0, 2.0, 2.0], 0.0, 5, 1.04),
    ]
    source = ScriptedInput(samples)
    backend = MuJoCoBackend(teleop_model_path, control_hz=100.0)
    controller = TeleopController(
        TeleopConfig(realtime=False),
        source,
        backend,
    )
    try:
        initial = backend.data.ctrl.copy()
        controller.step_once()
        controller.step_once()
        controller.step_once()
        held = backend.data.ctrl.copy()
        assert not np.allclose(held, initial)
        controller.step_once()
        controller.step_once()
        np.testing.assert_allclose(backend.data.ctrl, held)
    finally:
        controller.close()
    assert source.closed


def test_target_marker_tracks_mapping_target(teleop_model_path) -> None:
    source = ScriptedInput([sample([0.0, 0.0, 0.0], 1.0, 1, 1.0)])
    backend = MuJoCoBackend(teleop_model_path, control_hz=100.0)
    controller = TeleopController(
        TeleopConfig(realtime=False),
        source,
        backend,
    )
    try:
        expected_target = backend.data.site_xpos[backend.contract.site_id].copy()
        controller.step_once()
        mocap_body_id = mujoco.mj_name2id(
            backend.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "teleop_target",
        )
        mocap_id = backend.model.body_mocapid[mocap_body_id]
        np.testing.assert_allclose(
            backend.data.mocap_pos[mocap_id],
            expected_target,
        )
    finally:
        controller.close()


def test_controller_uses_only_backend_contract_and_owns_resources() -> None:
    source = ScriptedInput(
        [
            sample([0.0, 0.0, 0.0], 0.0, 1, 1.00),
            sample([0.0, 0.0, 0.0], 1.0, 2, 1.01),
            sample([0.01, 0.0, 0.0], 1.0, 3, 1.02),
            sample([0.01, 0.0, 0.0], 0.0, 4, 1.03),
        ],
    )
    backend = RecordingBackend()
    controller = TeleopController(
        TeleopConfig(realtime=False),
        source,
        backend,
    )

    diagnostics = [controller.step_once() for _ in range(4)]
    controller.close()
    controller.close()

    assert backend.begin_calls == 1
    assert len(backend.commands) == 1
    # hold() fires only on the deactivation edge so idle cycles do not spam
    # hardware backends with Stop commands.
    assert backend.holds == 1
    assert backend.steps == 4
    assert [diagnostic.active for diagnostic in diagnostics] == [
        False,
        True,
        True,
        False,
    ]
    assert source.close_calls == 1
    assert backend.close_calls == 1


def test_controller_passes_translation_rotation_to_mapper() -> None:
    rotation = (
        (0.0, -1.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    controller = TeleopController(
        TeleopConfig(realtime=False, translation_rotation=rotation),
        ScriptedInput([]),
        RecordingBackend(),
    )

    try:
        assert controller.mapper.config.translation_rotation == rotation
    finally:
        controller.close()


def test_run_reports_only_meaningful_state_and_reason_transitions() -> None:
    class RejectingBackend(RecordingBackend):
        def command_pose(self, target):
            from kinova_teleop.backend import BackendResult

            self.commands.append(target)
            return BackendResult(False, False, 0.01, 0.02, "IK did not converge")

    source = ScriptedInput(
        [
            sample([0.0, 0.0, 0.0], 1.0, 1, 1.00),
            sample([0.0, 0.0, 0.0], 1.0, 2, 1.01),
            sample([0.0, 0.0, 0.0], 0.0, 3, 1.02),
            sample([0.0, 0.0, 0.0], 1.0, 4, 1.03),
            sample([0.0, 0.0, 0.0], 1.0, 5, 1.04),
            sample([0.0, 0.0, 0.0], 0.0, 6, 1.05),
        ],
    )
    backend = RejectingBackend()
    controller = TeleopController(TeleopConfig(realtime=False), source, backend)
    events = []

    controller.run(max_steps=6, on_status=events.append)

    assert [
        (event.clutch_state.value, event.stale, event.reason)
        for event in events
    ] == [
        ("waiting_for_release", False, ""),
        ("ready", False, ""),
        ("active", False, ""),
        ("active", False, "IK did not converge"),
        ("ready", False, ""),
    ]
