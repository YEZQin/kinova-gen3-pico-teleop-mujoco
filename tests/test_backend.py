import mujoco
import mujoco.viewer
import numpy as np

from kinova_teleop.backend import MuJoCoBackend


def test_mujoco_backend_reports_current_pose_and_finite_state(
    teleop_model_path,
) -> None:
    backend = MuJoCoBackend(teleop_model_path, control_hz=100.0)
    try:
        pose = backend.current_pose()
        assert pose.position.shape == (3,)
        assert pose.quaternion.shape == (4,)
        assert backend.state_is_finite()
    finally:
        backend.close()


def test_mujoco_backend_rejects_nonconverged_ik_without_changing_controls(
    teleop_model_path,
) -> None:
    backend = MuJoCoBackend(teleop_model_path, control_hz=100.0)
    backend.begin_control()
    held = backend.data.ctrl.copy()

    class FailedIK:
        def solve(self, *_args, **_kwargs):
            from kinova_teleop.ik_solver import IKResult

            return IKResult(np.ones(7), False, 0.1, 0.2, 40)

    backend.ik = FailedIK()
    result = backend.command_pose(backend.current_pose())
    assert not result.accepted and not result.converged
    assert result.reason == "IK did not converge"
    np.testing.assert_allclose(backend.data.ctrl, held)
    backend.close()


def test_mujoco_backend_owns_and_idempotently_closes_launched_viewer(
    teleop_model_path,
    monkeypatch,
) -> None:
    class RecordingViewer:
        def __init__(self) -> None:
            self.close_calls = 0

        def close(self) -> None:
            self.close_calls += 1

    viewer = RecordingViewer()
    monkeypatch.setattr(
        mujoco.viewer,
        "launch_passive",
        lambda *_args: viewer,
    )
    backend = MuJoCoBackend(teleop_model_path, control_hz=100.0)

    assert backend.launch_viewer() is viewer
    backend.close()
    backend.close()

    assert viewer.close_calls == 1
