import mujoco
import numpy as np

from kinova_teleop.ik_solver import (
    DampedLeastSquaresIK,
    IKConfig,
    nearest_angle_delta,
)
from kinova_teleop.model_contract import load_and_validate_model
from kinova_teleop.pose_mapping import (
    Pose,
    matrix_to_quat,
    quat_from_axis_angle,
    quat_multiply,
)


def home_state(model, contract) -> tuple[np.ndarray, Pose]:
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, contract.home_key_id)
    mujoco.mj_forward(model, data)
    q = data.qpos[list(contract.qpos_addresses)].copy()
    pose = Pose(
        data.site_xpos[contract.site_id].copy(),
        matrix_to_quat(data.site_xmat[contract.site_id].reshape(3, 3)),
    )
    return q, pose


def test_ik_converges_for_reachable_6dof_target(model_path) -> None:
    model, contract = load_and_validate_model(model_path)
    q, pose = home_state(model, contract)
    target = Pose(
        pose.position + np.array([0.015, -0.01, 0.01]),
        pose.quaternion,
    )

    result = DampedLeastSquaresIK(model, contract, IKConfig()).solve(q, target, q)

    assert result.converged
    assert result.position_error < 1e-3
    assert result.rotation_error < 1e-2
    assert np.isfinite(result.qpos).all()


def test_ik_respects_limited_joints_and_stays_finite(model_path) -> None:
    model, contract = load_and_validate_model(model_path)
    q, pose = home_state(model, contract)
    target = Pose(
        pose.position + np.array([0.5, 0.5, 0.5]),
        pose.quaternion,
    )

    result = DampedLeastSquaresIK(
        model,
        contract,
        IKConfig(max_iterations=10),
    ).solve(q, target, q)

    assert np.isfinite(result.qpos).all()
    assert np.isfinite(result.position_error)
    assert np.isfinite(result.rotation_error)
    for joint_id, value in zip(contract.joint_ids, result.qpos):
        if model.jnt_limited[joint_id]:
            low, high = model.jnt_range[joint_id]
            assert low <= value <= high


def test_ik_converges_for_reachable_rotation(model_path) -> None:
    model, contract = load_and_validate_model(model_path)
    q, pose = home_state(model, contract)
    delta = quat_from_axis_angle([0.0, 0.0, 1.0], 0.05)
    target = Pose(pose.position, quat_multiply(delta, pose.quaternion))

    result = DampedLeastSquaresIK(model, contract, IKConfig()).solve(q, target, q)

    assert result.converged
    assert result.rotation_error < 1e-2
    assert result.position_error < 1e-3


def test_extended_configuration_remains_finite(model_path) -> None:
    model, contract = load_and_validate_model(model_path)
    q = np.zeros(7)
    data = mujoco.MjData(model)
    data.qpos[list(contract.qpos_addresses)] = q
    mujoco.mj_forward(model, data)
    pose = Pose(
        data.site_xpos[contract.site_id].copy(),
        matrix_to_quat(data.site_xmat[contract.site_id].reshape(3, 3)),
    )
    target = Pose(
        pose.position + np.array([0.001, 0.0, 0.001]),
        pose.quaternion,
    )

    result = DampedLeastSquaresIK(model, contract, IKConfig()).solve(q, target, q)

    assert np.isfinite(result.qpos).all()
    assert np.isfinite(result.position_error)
    assert np.isfinite(result.rotation_error)


def test_nearest_angle_delta_wraps_unlimited_hinges(model_path) -> None:
    model, contract = load_and_validate_model(model_path)
    reference = np.zeros(7)
    current = np.zeros(7)
    reference[0] = -np.pi + 0.1
    current[0] = np.pi - 0.1

    delta = nearest_angle_delta(reference, current, model, contract)

    np.testing.assert_allclose(delta[0], 0.2, atol=1e-12)
