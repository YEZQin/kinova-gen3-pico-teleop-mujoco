from collections.abc import Iterable

import mujoco
import numpy as np

from kinova_teleop.ik_solver import IKResult
from kinova_teleop.teleop_controller import TeleopConfig, TeleopController
from kinova_teleop.xr_input import ControllerSample, DryRunXrInput


class ScriptedInput:
    def __init__(self, samples: Iterable[ControllerSample]) -> None:
        self._samples = iter(samples)
        self.closed = False

    def read(self) -> ControllerSample:
        return next(self._samples)

    def close(self) -> None:
        self.closed = True


def sample(
    position: list[float],
    grip: float,
    timestamp_ns: int,
    received: float,
) -> ControllerSample:
    return ControllerSample(
        position=np.asarray(position, dtype=np.float64),
        quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        grip=grip,
        timestamp_ns=timestamp_ns,
        received_monotonic=received,
    )


def test_headless_dry_run_stays_finite(teleop_model_path) -> None:
    source = DryRunXrInput(control_hz=100.0)
    controller = TeleopController(
        TeleopConfig(model_path=teleop_model_path, realtime=False),
        source,
    )
    try:
        controller.run(headless=True, max_steps=500)
        assert np.isfinite(controller.data.qpos).all()
        assert np.isfinite(controller.data.qvel).all()
        assert np.isfinite(controller.data.ctrl).all()
        assert controller.steps == 500
    finally:
        controller.close()


def test_release_holds_last_actuator_target(teleop_model_path) -> None:
    samples = [
        sample([0.0, 0.0, 0.0], 1.0, 1, 1.00),
        sample([0.01, 0.0, 0.0], 1.0, 2, 1.01),
        sample([1.0, 1.0, 1.0], 0.0, 3, 1.02),
        sample([2.0, 2.0, 2.0], 0.0, 4, 1.03),
    ]
    source = ScriptedInput(samples)
    controller = TeleopController(
        TeleopConfig(model_path=teleop_model_path, realtime=False),
        source,
    )
    try:
        controller.step_once()
        controller.step_once()
        held = controller.data.ctrl.copy()
        controller.step_once()
        controller.step_once()
        np.testing.assert_allclose(controller.data.ctrl, held)
    finally:
        controller.close()
    assert source.closed


def test_target_marker_tracks_mapping_target(teleop_model_path) -> None:
    source = ScriptedInput([sample([0.0, 0.0, 0.0], 1.0, 1, 1.0)])
    controller = TeleopController(
        TeleopConfig(model_path=teleop_model_path, realtime=False),
        source,
    )
    try:
        expected_target = controller.data.site_xpos[controller.contract.site_id].copy()
        controller.step_once()
        mocap_body_id = mujoco.mj_name2id(
            controller.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "teleop_target",
        )
        mocap_id = controller.model.body_mocapid[mocap_body_id]
        np.testing.assert_allclose(
            controller.data.mocap_pos[mocap_id],
            expected_target,
        )
    finally:
        controller.close()


def test_nonconverged_ik_holds_previous_target(teleop_model_path) -> None:
    source = ScriptedInput([sample([0.0, 0.0, 0.0], 1.0, 1, 1.0)])
    controller = TeleopController(
        TeleopConfig(model_path=teleop_model_path, realtime=False),
        source,
    )

    class NonconvergedIK:
        def solve(self, *_args, **_kwargs) -> IKResult:
            return IKResult(
                qpos=np.ones(7),
                converged=False,
                position_error=0.1,
                rotation_error=0.2,
                iterations=40,
            )

    controller.ik = NonconvergedIK()
    held = controller.data.ctrl.copy()
    try:
        diagnostics = controller.step_once()
        assert not diagnostics.ik_converged
        np.testing.assert_allclose(controller.data.ctrl, held)
    finally:
        controller.close()
