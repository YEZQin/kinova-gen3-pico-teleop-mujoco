"""MuJoCo end-effector target backend."""

from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np

from .backend import BackendResult
from .ik_solver import DampedLeastSquaresIK, IKConfig
from .model_contract import load_and_validate_model
from .pose_mapping import Pose, matrix_to_quat


class MuJoCoBackend:
    """MuJoCo adapter for the validated seven-joint Kinova Gen3 model."""

    def __init__(
        self,
        model_path: Path,
        *,
        control_hz: float = 100.0,
    ) -> None:
        if not math.isfinite(control_hz) or control_hz <= 0.0:
            raise ValueError("control_hz must be positive and finite")

        self.model, self.contract = load_and_validate_model(model_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(
            self.model,
            self.data,
            self.contract.home_key_id,
        )
        mujoco.mj_forward(self.model, self.data)

        self.ik = DampedLeastSquaresIK(self.model, self.contract, IKConfig())
        self.posture_reference = self._joint_positions()
        self.last_valid_target = self.posture_reference.copy()
        self._set_controls(self.last_valid_target)
        self._mocap_id = self._find_target_mocap()
        self._update_target_marker(self.current_pose())

        simulation_interval = 1.0 / control_hz
        self._simulation_substeps = max(
            1,
            int(round(simulation_interval / float(self.model.opt.timestep))),
        )
        self._viewer = None
        self._closed = False

    def _joint_positions(self) -> np.ndarray:
        return self.data.qpos[list(self.contract.qpos_addresses)].copy()

    def current_pose(self) -> Pose:
        return Pose(
            self.data.site_xpos[self.contract.site_id].copy(),
            matrix_to_quat(
                self.data.site_xmat[self.contract.site_id].reshape(3, 3),
            ),
        )

    def _set_controls(self, q_target: np.ndarray) -> None:
        for actuator_id, value in zip(self.contract.actuator_ids, q_target):
            self.data.ctrl[actuator_id] = value

    def _find_target_mocap(self) -> int | None:
        body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "teleop_target",
        )
        if body_id < 0:
            return None
        mocap_id = int(self.model.body_mocapid[body_id])
        return mocap_id if mocap_id >= 0 else None

    def _update_target_marker(self, target: Pose) -> None:
        if self._mocap_id is None:
            return
        self.data.mocap_pos[self._mocap_id] = target.position
        self.data.mocap_quat[self._mocap_id] = target.quaternion

    def begin_control(self) -> None:
        self.posture_reference = self._joint_positions()

    def command_pose(self, target: Pose) -> BackendResult:
        self._update_target_marker(target)
        result = self.ik.solve(
            self._joint_positions(),
            target,
            self.posture_reference,
        )
        errors_are_finite = bool(
            np.isfinite(result.position_error)
            and np.isfinite(result.rotation_error)
        )
        joints_are_finite = bool(np.isfinite(result.qpos).all())

        if not result.converged:
            reason = "IK did not converge"
        elif not joints_are_finite:
            reason = "IK returned non-finite joint target"
        elif not errors_are_finite:
            reason = "IK returned non-finite error"
        else:
            self.last_valid_target = result.qpos.copy()
            self._set_controls(self.last_valid_target)
            return BackendResult(
                accepted=True,
                converged=True,
                position_error=result.position_error,
                rotation_error=result.rotation_error,
                reason="",
            )

        return BackendResult(
            accepted=False,
            converged=result.converged,
            position_error=result.position_error,
            rotation_error=result.rotation_error,
            reason=reason,
        )

    def hold(self) -> None:
        self._set_controls(self.last_valid_target)

    def state_is_finite(self) -> bool:
        return all(
            np.isfinite(array).all()
            for array in (self.data.qpos, self.data.qvel, self.data.ctrl)
        )

    def _assert_finite_state(self) -> None:
        arrays = {
            "qpos": self.data.qpos,
            "qvel": self.data.qvel,
            "ctrl": self.data.ctrl,
        }
        bad = [name for name, array in arrays.items() if not np.isfinite(array).all()]
        if bad:
            raise FloatingPointError(
                f"MuJoCo state became non-finite: {', '.join(bad)}",
            )

    def step(self) -> None:
        for _ in range(self._simulation_substeps):
            mujoco.mj_step(self.model, self.data)
        self._assert_finite_state()

    def launch_viewer(self):
        if self._closed:
            raise RuntimeError("Cannot launch a viewer for a closed backend")
        if self._viewer is None:
            import mujoco.viewer

            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
        return self._viewer

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
