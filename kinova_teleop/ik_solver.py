"""MuJoCo-native damped least-squares inverse kinematics."""

from __future__ import annotations

from dataclasses import dataclass
import math

import mujoco
import numpy as np

from .model_contract import ModelContract
from .pose_mapping import (
    Pose,
    matrix_to_quat,
    quat_conjugate,
    quat_multiply,
    quat_to_rotvec,
)


@dataclass(frozen=True)
class IKConfig:
    max_iterations: int = 40
    position_tolerance: float = 1e-3
    rotation_tolerance: float = 1e-2
    position_gain: float = 1.0
    rotation_gain: float = 0.7
    base_damping: float = 1e-3
    singular_value_threshold: float = 0.08
    singular_damping_gain: float = 0.2
    nullspace_gain: float = 0.03
    max_joint_step: float = 0.08


@dataclass(frozen=True)
class IKResult:
    qpos: np.ndarray
    converged: bool
    position_error: float
    rotation_error: float
    iterations: int


def nearest_angle_delta(
    reference: np.ndarray,
    current: np.ndarray,
    model: mujoco.MjModel,
    contract: ModelContract,
) -> np.ndarray:
    """Return ``reference-current``, wrapping unlimited hinge joints."""

    reference = np.asarray(reference, dtype=np.float64)
    current = np.asarray(current, dtype=np.float64)
    if reference.shape != (7,) or current.shape != (7,):
        raise ValueError("reference and current must each contain seven joints")

    delta = reference - current
    for index, joint_id in enumerate(contract.joint_ids):
        if (
            not bool(model.jnt_limited[joint_id])
            and int(model.jnt_type[joint_id]) == int(mujoco.mjtJoint.mjJNT_HINGE)
        ):
            delta[index] = math.atan2(
                math.sin(delta[index]),
                math.cos(delta[index]),
            )
    return delta


class DampedLeastSquaresIK:
    """Iterative 6DoF IK with adaptive damping and posture nullspace bias."""

    def __init__(
        self,
        model: mujoco.MjModel,
        contract: ModelContract,
        config: IKConfig,
    ) -> None:
        if config.max_iterations <= 0:
            raise ValueError("max_iterations must be positive")
        self.model = model
        self.contract = contract
        self.config = config
        self.data = mujoco.MjData(model)
        self._qpos_addresses = list(contract.qpos_addresses)
        self._dof_addresses = list(contract.dof_addresses)

    def _pose_error(self, target: Pose) -> tuple[np.ndarray, np.ndarray]:
        position = self.data.site_xpos[self.contract.site_id].copy()
        current_quaternion = matrix_to_quat(
            self.data.site_xmat[self.contract.site_id].reshape(3, 3),
        )
        position_error = np.asarray(target.position) - position
        rotation_error = quat_to_rotvec(
            quat_multiply(
                target.quaternion,
                quat_conjugate(current_quaternion),
            ),
        )
        return position_error, rotation_error

    def _set_position(self, q: np.ndarray) -> None:
        self.data.qpos[self._qpos_addresses] = q
        self.data.qvel[:] = 0.0
        mujoco.mj_fwdPosition(self.model, self.data)

    def _apply_limits(
        self,
        candidate: np.ndarray,
        wrap_reference: np.ndarray,
    ) -> np.ndarray:
        result = np.asarray(candidate, dtype=np.float64).copy()
        for index, joint_id in enumerate(self.contract.joint_ids):
            if bool(self.model.jnt_limited[joint_id]):
                low, high = self.model.jnt_range[joint_id]
                result[index] = np.clip(result[index], low, high)
            elif int(self.model.jnt_type[joint_id]) == int(
                mujoco.mjtJoint.mjJNT_HINGE,
            ):
                offset = result[index] - wrap_reference[index]
                result[index] = wrap_reference[index] + math.atan2(
                    math.sin(offset),
                    math.cos(offset),
                )
        return result

    def solve(
        self,
        q_start: np.ndarray,
        target: Pose,
        posture_reference: np.ndarray,
    ) -> IKResult:
        q_start = np.asarray(q_start, dtype=np.float64)
        posture_reference = np.asarray(posture_reference, dtype=np.float64)
        if q_start.shape != (7,) or posture_reference.shape != (7,):
            raise ValueError("q_start and posture_reference must each contain seven joints")
        if not np.isfinite(q_start).all() or not np.isfinite(posture_reference).all():
            raise ValueError("IK joint inputs must be finite")

        q = self._apply_limits(q_start, q_start)
        position_error = np.full(3, np.inf)
        rotation_error = np.full(3, np.inf)

        for iteration in range(1, self.config.max_iterations + 1):
            self._set_position(q)
            position_error, rotation_error = self._pose_error(target)
            position_norm = float(np.linalg.norm(position_error))
            rotation_norm = float(np.linalg.norm(rotation_error))
            if (
                position_norm < self.config.position_tolerance
                and rotation_norm < self.config.rotation_tolerance
            ):
                return IKResult(
                    q.copy(),
                    True,
                    position_norm,
                    rotation_norm,
                    iteration,
                )

            jacobian_position = np.zeros((3, self.model.nv))
            jacobian_rotation = np.zeros((3, self.model.nv))
            mujoco.mj_jacSite(
                self.model,
                self.data,
                jacobian_position,
                jacobian_rotation,
                self.contract.site_id,
            )
            jacobian = np.vstack(
                (
                    jacobian_position[:, self._dof_addresses],
                    jacobian_rotation[:, self._dof_addresses],
                ),
            )
            error = np.concatenate(
                (
                    self.config.position_gain * position_error,
                    self.config.rotation_gain * rotation_error,
                ),
            )
            singular_values = np.linalg.svd(jacobian, compute_uv=False)
            sigma_min = float(singular_values[-1])
            deficit = max(
                0.0,
                self.config.singular_value_threshold - sigma_min,
            )
            damping = (
                self.config.base_damping
                + self.config.singular_damping_gain * deficit * deficit
            )
            regularized = (
                jacobian @ jacobian.T
                + damping * damping * np.eye(6)
            )
            pseudo_inverse = jacobian.T @ np.linalg.solve(
                regularized,
                np.eye(6),
            )
            null_projector = np.eye(7) - pseudo_inverse @ jacobian
            posture_delta = nearest_angle_delta(
                posture_reference,
                q,
                self.model,
                self.contract,
            )
            joint_delta = (
                pseudo_inverse @ error
                + null_projector
                @ (self.config.nullspace_gain * posture_delta)
            )
            joint_delta = np.clip(
                joint_delta,
                -self.config.max_joint_step,
                self.config.max_joint_step,
            )
            if not np.isfinite(joint_delta).all():
                break
            q = self._apply_limits(q + joint_delta, q_start)

        self._set_position(q)
        position_error, rotation_error = self._pose_error(target)
        return IKResult(
            q.copy(),
            False,
            float(np.linalg.norm(position_error)),
            float(np.linalg.norm(rotation_error)),
            self.config.max_iterations,
        )
