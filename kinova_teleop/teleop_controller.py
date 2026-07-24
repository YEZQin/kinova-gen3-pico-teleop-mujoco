"""Teleoperation orchestration for the MuJoCo Kinova Gen3 model."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import time

import mujoco
import numpy as np

from .ik_solver import DampedLeastSquaresIK, IKConfig
from .model_contract import load_and_validate_model
from .pose_mapping import (
    MappingConfig,
    Pose,
    RelativePoseMapper,
    matrix_to_quat,
)
from .xr_input import XrInputSource


@dataclass(frozen=True)
class TeleopConfig:
    model_path: Path
    control_hz: float = 100.0
    realtime: bool = True
    translation_scale: float = 1.0
    stale_timeout: float = 0.2


@dataclass(frozen=True)
class StepDiagnostics:
    active: bool
    stale: bool
    ik_converged: bool
    position_error: float
    rotation_error: float


class TeleopController:
    def __init__(self, config: TeleopConfig, source: XrInputSource) -> None:
        if not math.isfinite(config.control_hz) or config.control_hz <= 0.0:
            raise ValueError("control_hz must be positive and finite")
        if not math.isfinite(config.translation_scale) or config.translation_scale <= 0:
            raise ValueError("translation_scale must be positive and finite")

        self.config = config
        self.source = source
        self.model, self.contract = load_and_validate_model(config.model_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(
            self.model,
            self.data,
            self.contract.home_key_id,
        )
        mujoco.mj_forward(self.model, self.data)

        self.mapper = RelativePoseMapper(
            MappingConfig(
                translation_scale=config.translation_scale,
                stale_timeout=config.stale_timeout,
            ),
        )
        self.mapper.reset(self._site_pose())
        self.ik = DampedLeastSquaresIK(self.model, self.contract, IKConfig())
        self.posture_reference = self._joint_positions()
        self.last_valid_target = self.posture_reference.copy()
        self._set_controls(self.last_valid_target)
        self.steps = 0
        self._closed = False
        self._mocap_id = self._find_target_mocap()
        self._update_target_marker(self._site_pose())

        simulation_interval = 1.0 / config.control_hz
        self._simulation_substeps = max(
            1,
            int(round(simulation_interval / float(self.model.opt.timestep))),
        )

    def _joint_positions(self) -> np.ndarray:
        return self.data.qpos[list(self.contract.qpos_addresses)].copy()

    def _site_pose(self) -> Pose:
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

    def step_once(self) -> StepDiagnostics:
        sample = self.source.read()
        now = (
            time.monotonic()
            if self.config.realtime
            else float(sample.received_monotonic)
        )
        mapping = self.mapper.update(sample, self._site_pose(), now)
        self._update_target_marker(mapping.target)

        ik_converged = False
        position_error = 0.0
        rotation_error = 0.0
        if mapping.activated:
            self.posture_reference = self._joint_positions()

        if mapping.active:
            result = self.ik.solve(
                self._joint_positions(),
                mapping.target,
                self.posture_reference,
            )
            ik_converged = result.converged
            position_error = result.position_error
            rotation_error = result.rotation_error
            if (
                np.isfinite(result.qpos).all()
                and np.isfinite(result.position_error)
                and np.isfinite(result.rotation_error)
            ):
                self.last_valid_target = result.qpos.copy()
                self._set_controls(self.last_valid_target)

        for _ in range(self._simulation_substeps):
            mujoco.mj_step(self.model, self.data)
        self._assert_finite_state()
        self.steps += 1
        return StepDiagnostics(
            active=mapping.active,
            stale=mapping.stale,
            ik_converged=ik_converged,
            position_error=position_error,
            rotation_error=rotation_error,
        )

    def run(self, headless: bool = False, max_steps: int | None = None) -> None:
        if headless and (max_steps is None or max_steps <= 0):
            raise ValueError("headless runs require a positive max_steps")
        if max_steps is not None and max_steps <= 0:
            raise ValueError("max_steps must be positive")

        viewer = None
        try:
            if not headless:
                import mujoco.viewer

                viewer = mujoco.viewer.launch_passive(self.model, self.data)

            next_deadline = time.monotonic()
            while (
                (max_steps is None or self.steps < max_steps)
                and (viewer is None or viewer.is_running())
            ):
                self.step_once()
                if viewer is not None:
                    viewer.sync()
                if self.config.realtime:
                    next_deadline += 1.0 / self.config.control_hz
                    delay = next_deadline - time.monotonic()
                    if delay > 0.0:
                        time.sleep(delay)
                    elif delay < -1.0:
                        next_deadline = time.monotonic()
        finally:
            if viewer is not None:
                viewer.close()
            self.close()

    def close(self) -> None:
        if not self._closed:
            self.source.close()
            self._closed = True
