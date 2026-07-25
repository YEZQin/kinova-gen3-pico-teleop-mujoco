from collections.abc import Iterable

import numpy as np

from kinova_teleop.backend import BackendResult
from kinova_teleop.pose_mapping import Pose
from kinova_teleop.teleop_controller import TeleopConfig, TeleopController
from kinova_teleop.xr_input import ControllerSample


class ScriptedInput:
    def __init__(self, samples: Iterable[ControllerSample]) -> None:
        self._samples = iter(samples)
        self.close_calls = 0

    def read(self) -> ControllerSample:
        return next(self._samples)

    def close(self) -> None:
        self.close_calls += 1


class RecordingBackend:
    def __init__(self) -> None:
        self.pose = Pose(
            np.array([0.4, -0.2, 0.3], dtype=np.float64),
            np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        )
        self.events: list[str] = []
        self.targets: list[Pose] = []
        self.steps = 0
        self.close_calls = 0

    def current_pose(self) -> Pose:
        return self.pose

    def begin_control(self) -> None:
        self.events.append("begin_control")

    def command_pose(self, target: Pose) -> BackendResult:
        self.events.append("command_pose")
        self.targets.append(target)
        return BackendResult(converged=True, position_error=0.01, rotation_error=0.02)

    def hold(self) -> None:
        self.events.append("hold")

    def step(self) -> None:
        self.events.append("step")
        self.steps += 1

    def close(self) -> None:
        self.close_calls += 1


def sample(position: list[float], grip: float, timestamp_ns: int, received: float) -> ControllerSample:
    return ControllerSample(
        position=np.asarray(position, dtype=np.float64),
        quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        grip=grip,
        timestamp_ns=timestamp_ns,
        received_monotonic=received,
    )


def test_controller_orchestrates_backend_clutch_lifecycle() -> None:
    """Removing clutch transitions or per-cycle stepping must break this test."""

    source = ScriptedInput(
        [
            sample([0.0, 0.0, 0.0], 1.0, 1, 1.00),
            sample([0.01, 0.0, 0.0], 1.0, 2, 1.01),
            sample([0.01, 0.0, 0.0], 0.0, 3, 1.02),
            sample([0.01, 0.0, 0.0], 0.0, 4, 1.03),
        ],
    )
    backend = RecordingBackend()
    controller = TeleopController(
        TeleopConfig(model_path=None, realtime=False),
        source,
        backend,
    )

    for _ in range(4):
        controller.step_once()

    assert backend.events == [
        "begin_control",
        "command_pose",
        "step",
        "command_pose",
        "step",
        "hold",
        "step",
        "step",
    ]
    assert len(backend.targets) == 2
    assert backend.targets[0].position.tolist() == [0.4, -0.2, 0.3]
    assert controller.steps == 4

    controller.close()
    controller.close()

    assert backend.close_calls == 1
    assert source.close_calls == 1


def test_controller_runs_generic_backend_without_a_mujoco_viewer() -> None:
    """Reintroducing a controller-level viewer/model dependency must fail here."""

    source = ScriptedInput([sample([0.0, 0.0, 0.0], 1.0, 1, 1.00)])
    backend = RecordingBackend()
    controller = TeleopController(
        TeleopConfig(model_path=None, realtime=False),
        source,
        backend,
    )

    controller.run(max_steps=1)

    assert backend.events == ["begin_control", "command_pose", "step"]
    assert backend.close_calls == 1
    assert source.close_calls == 1
