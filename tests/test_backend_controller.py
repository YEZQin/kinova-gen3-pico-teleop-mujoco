"""Controller-to-backend contract tests focused on hardware-safe RPC ordering."""

from collections.abc import Iterable
from dataclasses import replace

import numpy as np
import pytest

from kinova_teleop.backend import BackendResult
import kinova_teleop.kortex_backend as kortex_backend_module
from kinova_teleop.kortex_backend import KortexBackend
from kinova_teleop.pose_mapping import Pose
from kinova_teleop.teleop_controller import TeleopConfig, TeleopController
from kinova_teleop.workspace import AnchorEnvelope
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
        return BackendResult(
            accepted=True,
            converged=True,
            position_error=0.01,
            rotation_error=0.02,
            reason="",
        )

    def hold(self) -> None:
        self.events.append("hold")

    def step(self) -> None:
        self.events.append("step")
        self.steps += 1

    def close(self) -> None:
        self.close_calls += 1


class RpcOrderingBackend(RecordingBackend):
    def current_pose(self) -> Pose:
        self.events.append("current_pose")
        return self.pose


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

    for _ in range(4):
        controller.step_once()

    assert backend.events == [
        "step",
        "begin_control",
        "step",
        "command_pose",
        "step",
        "hold",
        "step",
    ]
    assert len(backend.targets) == 1
    assert controller.steps == 4

    controller.close()
    controller.close()

    assert backend.close_calls == 1
    assert source.close_calls == 1


def test_unconfirmed_boundary_stop_error_remains_terminal() -> None:
    from kinova_teleop.kortex_backend import KortexSafetyError

    class StopFailureBackend(RecordingBackend):
        def command_pose(self, target: Pose) -> BackendResult:
            self.events.append("command_pose")
            self.targets.append(target)
            raise KortexSafetyError("Stop attempted but unconfirmed")

    backend = StopFailureBackend()
    controller = TeleopController(
        TeleopConfig(realtime=False),
        ScriptedInput(
            [
                sample([0.0, 0.0, 0.0], 0.0, 1, 1.00),
                sample([0.0, 0.0, 0.0], 1.0, 2, 1.01),
                sample([0.01, 0.0, 0.0], 1.0, 3, 1.02),
            ]
        ),
        backend,
    )

    controller.step_once()
    controller.step_once()
    with pytest.raises(KortexSafetyError, match="Stop attempted but unconfirmed"):
        controller.step_once()

    assert backend.events == ["step", "begin_control", "step", "command_pose"]


def test_controller_close_orders_backend_before_source_and_is_idempotent() -> None:
    """Closing XR before the motion backend must break this test."""

    events: list[str] = []

    class OrderedSource(ScriptedInput):
        def close(self) -> None:
            events.append("source.close")
            super().close()

    class OrderedBackend(RecordingBackend):
        def close(self) -> None:
            events.append("backend.close")
            super().close()

    source = OrderedSource([])
    backend = OrderedBackend()
    controller = TeleopController(
        TeleopConfig(realtime=False),
        source,
        backend,
    )

    controller.close()
    controller.close()

    assert events == ["backend.close", "source.close"]
    assert backend.close_calls == 1
    assert source.close_calls == 1


def test_controller_close_attempts_source_after_backend_failure() -> None:
    """A backend close failure must not strand the XR input resource."""

    events: list[str] = []

    class OrderedSource(ScriptedInput):
        def close(self) -> None:
            events.append("source.close")
            super().close()

    class FailingBackend(RecordingBackend):
        def close(self) -> None:
            events.append("backend.close")
            raise RuntimeError("backend cleanup failed")

    controller = TeleopController(
        TeleopConfig(realtime=False),
        OrderedSource([]),
        FailingBackend(),
    )

    with pytest.raises(RuntimeError, match="backend cleanup failed"):
        controller.close()

    assert events == ["backend.close", "source.close"]


def test_first_grip_reads_anchor_after_begin_and_active_cycle_adds_no_feedback() -> None:
    """Eager or per-active-cycle controller feedback must break this test."""

    source = ScriptedInput(
        [
            sample([0.0, 0.0, 0.0], 0.0, 1, 1.00),
            sample([0.0, 0.0, 0.0], 1.0, 2, 1.01),
            sample([0.01, 0.0, 0.0], 1.0, 3, 1.02),
        ],
    )
    backend = RpcOrderingBackend()
    controller = TeleopController(
        TeleopConfig(realtime=False),
        source,
        backend,
    )

    # Construction seeds the mapper with exactly one feedback read.
    assert backend.events == ["current_pose"]
    backend.events.clear()

    controller.step_once()
    assert backend.events == ["step"]

    backend.events.clear()
    controller.step_once()
    assert backend.events == [
        "begin_control",
        "current_pose",
        "step",
    ]
    assert backend.targets == []

    backend.events.clear()
    controller.step_once()
    assert backend.events == ["command_pose", "step"]


def test_first_press_anchors_without_twist_when_feedback_changes(
    monkeypatch,
) -> None:
    """A feedback change after anchoring must not create activation motion."""

    class Twist:
        linear_x = linear_y = linear_z = 0.0
        angular_x = angular_y = angular_z = 0.0

    class TwistCommand:
        def __init__(self) -> None:
            self.reference_frame = None
            self.duration = None
            self.twist = Twist()

    class ServoingModeInformation:
        def __init__(self) -> None:
            self.servoing_mode = None

    class ManualWatchdog:
        def __init__(self, *, target, args=(), name, daemon) -> None:
            self.args = args

        def start(self) -> None:
            self.args[0].set()

        def join(self, timeout=None) -> None:
            pass

    class Base:
        def __init__(self) -> None:
            self.sent = []

        def SetServoingMode(self, _mode, *, options=None) -> None:
            pass

        def GetArmState(self, *, options=None):
            return type("ArmState", (), {"active_state": 31})()

        def SendTwistCommand(self, command, *, options=None) -> None:
            self.sent.append(command)

        def Stop(self, *, options=None) -> None:
            pass

    class Cyclic:
        def __init__(self) -> None:
            self.feedback_calls = 0

        def RefreshFeedback(self, *, options=None):
            self.feedback_calls += 1
            # The third read is the vulnerable command-time read: it differs
            # from the second read used to establish the Grip anchor.
            tool_pose_x = 0.001 if self.feedback_calls >= 3 else 0.0
            return type(
                "Feedback",
                (),
                {"base": type("Pose", (), {
                    "tool_pose_x": tool_pose_x,
                    "tool_pose_y": 0.0,
                    "tool_pose_z": 0.0,
                    "tool_pose_theta_x": 0.0,
                    "tool_pose_theta_y": 0.0,
                    "tool_pose_theta_z": 0.0,
                })()},
            )()

    class Connection:
        def __init__(self) -> None:
            self.base = Base()
            self.base_cyclic = Cyclic()
            self.base_pb2 = type(
                "BasePb2",
                (),
                {
                    "TwistCommand": TwistCommand,
                    "ServoingModeInformation": ServoingModeInformation,
                    "CARTESIAN_REFERENCE_FRAME_BASE": 1,
                    "SINGLE_LEVEL_SERVOING": 2,
                    "ARMSTATE_SERVOING_READY": 31,
                },
            )()

        def rpc_options(self):
            return object()

        def close(self) -> bool:
            return True

    monkeypatch.setattr(kortex_backend_module, "_watchdog_thread_factory", ManualWatchdog)
    connection = Connection()
    backend = KortexBackend(
        connection,
        anchor_envelope=AnchorEnvelope((0.02, 0.02, 0.02), 0.1),
        max_linear_speed=0.005,
        max_angular_speed_deg=2.0,
    )
    controller = TeleopController(
        TeleopConfig(realtime=False, translation_scale=0.25),
        ScriptedInput(
            [
                sample([0.0, 0.0, 0.0], 0.0, 1, 1.00),
                sample([0.0, 0.0, 0.0], 1.0, 2, 1.01),
                sample([0.04, 0.0, 0.0], 1.0, 3, 1.02),
            ]
        ),
        backend,
    )

    controller.step_once()
    controller.step_once()

    # Activation is an anchor-only cycle. Even a changed feedback frame must
    # not be converted into a compensating Twist command.
    assert connection.base.sent == []

    controller.step_once()

    sent = [
        np.array(
            [
                command.twist.linear_x,
                command.twist.linear_y,
                command.twist.linear_z,
                command.twist.angular_x,
                command.twist.angular_y,
                command.twist.angular_z,
            ]
        )
        for command in connection.base.sent
    ]
    assert len(sent) == 1
    assert 0.0 < np.linalg.norm(sent[0][:3]) <= 0.005
    np.testing.assert_allclose(sent[0][3:], np.zeros(3))
    assert max(np.linalg.norm(command[:3]) for command in sent) <= 0.005
    assert max(np.linalg.norm(command[3:]) for command in sent) <= 2.0

    controller.close()


@pytest.mark.parametrize(
    "unsafe_sample",
    [
        sample([1.0, 2.0, 3.0], 0.0, 3, 1.02),
        replace(sample([1.0, 2.0, 3.0], 1.0, 3, 1.02), valid=False),
        sample([1.0, 2.0, 3.0], 1.0, 2, 1.22),
    ],
    ids=("released", "invalid", "stale"),
)
def test_unsafe_input_holds_without_preceding_feedback(
    unsafe_sample: ControllerSample,
) -> None:
    """Release, invalidity, or staleness must Stop before any feedback RPC."""

    source = ScriptedInput(
        [
            sample([0.0, 0.0, 0.0], 0.0, 1, 1.00),
            sample([0.0, 0.0, 0.0], 1.0, 2, 1.01),
            unsafe_sample,
        ],
    )
    backend = RpcOrderingBackend()
    controller = TeleopController(
        TeleopConfig(realtime=False),
        source,
        backend,
    )
    controller.step_once()
    controller.step_once()
    backend.events.clear()

    controller.step_once()

    assert backend.events == ["hold", "step"]


def test_controller_runs_generic_backend_without_a_mujoco_viewer() -> None:
    """Reintroducing a controller-level viewer/model dependency must fail here."""

    source = ScriptedInput(
        [
            sample([0.0, 0.0, 0.0], 0.0, 1, 1.00),
            sample([0.0, 0.0, 0.0], 1.0, 2, 1.01),
        ],
    )
    backend = RecordingBackend()
    controller = TeleopController(
        TeleopConfig(realtime=False),
        source,
        backend,
    )

    controller.run(max_steps=2)

    assert backend.events == ["step", "begin_control", "step"]
    assert backend.close_calls == 1
    assert source.close_calls == 1


class TriggerRejectingBackend(RecordingBackend):
    def command_gripper(self, _position: float) -> bool:
        raise AssertionError("PICO trigger must remain telemetry only")


def test_controller_never_actuates_pico_trigger() -> None:
    source = ScriptedInput(
        [
            ControllerSample(
                position=np.zeros(3, dtype=np.float64),
                quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                grip=0.0,
                timestamp_ns=1,
                received_monotonic=1.0,
                trigger=0.3,
            ),
            ControllerSample(
                position=np.zeros(3, dtype=np.float64),
                quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                grip=1.0,
                timestamp_ns=2,
                received_monotonic=1.01,
                trigger=0.7,
            ),
        ],
    )
    backend = TriggerRejectingBackend()
    controller = TeleopController(TeleopConfig(realtime=False), source, backend)

    controller.step_once()
    controller.step_once()

    assert backend.events == [
        "step",
        "begin_control",
        "step",
    ]
