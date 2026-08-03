from __future__ import annotations

from pathlib import Path
import json
import pytest

import numpy as np

from kinova_teleop.backend import BackendResult
from kinova_teleop.fixed_trajectory import (
    FixedTrajectoryRunner,
    FixedTrajectorySpec,
    RelativeSegment,
    load_trajectory,
    validate_trajectory,
)
from kinova_teleop.pose_mapping import Pose


def test_first_trial_trajectory_has_ten_return_to_anchor_segments():
    spec = load_trajectory(Path("configs/gen3_micro_axes.json"))
    assert len(spec.segments) == 10
    assert all(segment.offsets_xyz[-1] == (0.0, 0.0, 0.0) for segment in spec.segments)
    assert max(abs(value) for segment in spec.segments for point in segment.offsets_xyz for value in point) <= 0.005


class RejectingBackend:
    def __init__(self, reject_at_command: int):
        self.reject_at_command = reject_at_command
        self.commands = 0
        self.hold_calls = 0

    def begin_control(self):
        return None

    def current_pose(self):
        return Pose(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))

    def command_pose(self, target):
        self.commands += 1
        if self.commands == self.reject_at_command:
            return BackendResult(False, False, 0.0, 0.0, "rejected")
        return BackendResult(True, True, 0.0, 0.0, "")

    def hold(self):
        self.hold_calls += 1

    def step(self):
        return None


def test_runner_stops_on_first_rejection_and_never_runs_next_segment():
    spec = load_trajectory(Path("configs/gen3_micro_axes.json"))
    spec = spec.__class__(spec.schema_version, "two", spec.max_offset_m, spec.segments[:2], spec.control_hz, spec.max_speed_mps, spec.operator_approval_each_segment)
    backend = RejectingBackend(reject_at_command=3)
    result = FixedTrajectoryRunner(
        backend,
        control_hz=25.0,
        approve_segment=lambda *_: True,
    ).run(spec)
    assert result.completed_segments == 0
    assert backend.hold_calls == 1


def test_runner_fails_closed_without_segment_approval_callback():
    spec = load_trajectory(Path("configs/gen3_micro_axes.json"))
    backend = RejectingBackend(reject_at_command=999)
    events: list[tuple[str, str, dict[str, object]]] = []
    result = FixedTrajectoryRunner(
        backend,
        event_sink=lambda kind, state, payload: events.append((kind, state, dict(payload))),
    ).run(spec)

    assert result.completed_segments == 0
    assert result.reason == "operator approval callback is required"
    assert backend.commands == 0
    assert backend.hold_calls == 1
    assert [kind for kind, _state, _payload in events] == ["command_rejected"]
    assert not any(kind == "physical_stop_observed" for kind, _state, _payload in events)


def test_trajectory_loader_rejects_bad_json_and_extra_fields(tmp_path):
    bad = tmp_path / "bad.json"
    bad.write_text("[]", encoding="utf-8")
    with pytest.raises(ValueError, match="object"):
        load_trajectory(bad)
    bad.write_text(json.dumps({"schema_version": "1.0"}), encoding="utf-8")
    with pytest.raises(ValueError, match="missing"):
        load_trajectory(bad)
    bad.write_text(json.dumps({"schema_version": "1.0", "name": "x", "max_offset_m": 0.005, "segments": [], "extra": 1}), encoding="utf-8")
    with pytest.raises(ValueError, match="unexpected"):
        load_trajectory(bad)


def test_trajectory_validation_rejects_bad_segments_and_anchor():
    with pytest.raises(ValueError):
        RelativeSegment("bad", ((0.0, 0.0, 0.0),))
    with pytest.raises(ValueError):
        RelativeSegment("bad", ((0.0, 0.0, 0.0), (float("nan"), 0.0, 0.0)))
    with pytest.raises(ValueError):
        FixedTrajectorySpec("1.0", "x", 0.005, ())
    with pytest.raises(ValueError):
        FixedTrajectorySpec("2.0", "x", 0.005, (RelativeSegment("x", ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))),))
    with pytest.raises(ValueError):
        FixedTrajectorySpec("1.0", "x", 0.005, (  # does not return
            RelativeSegment("x", ((0.0, 0.0, 0.0), (0.001, 0.0, 0.0))),
        ))
    spec = load_trajectory(Path("configs/gen3_micro_axes.json"))
    with pytest.raises(ValueError):
        validate_trajectory(spec, anchor=Pose(np.array([float("nan"), 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0])))


class ConfirmingBackend(RejectingBackend):
    def __init__(self, result=True):
        super().__init__(999)
        self.result = result

    def confirm_stationary(self):
        return self.result


def test_runner_operator_denial_and_stationary_failure_stop_once():
    spec = load_trajectory(Path("configs/gen3_micro_axes.json"))
    denied = FixedTrajectoryRunner(ConfirmingBackend(), approve_segment=lambda *_: False).run(spec)
    assert denied.completed_segments == 0
    failed = FixedTrajectoryRunner(
        ConfirmingBackend(False),
        approve_segment=lambda *_: True,
    ).run(spec)
    assert failed.reason == "stationary confirmation failed"
    assert failed.completed_segments == 0


def test_runner_emits_segment_completion_only_after_software_stationary():
    spec = load_trajectory(Path("configs/gen3_micro_axes.json"))
    spec = spec.__class__(
        spec.schema_version,
        "one",
        spec.max_offset_m,
        spec.segments[:1],
        spec.control_hz,
        spec.max_speed_mps,
        spec.operator_approval_each_segment,
    )
    events: list[str] = []
    result = FixedTrajectoryRunner(
        ConfirmingBackend(True),
        approve_segment=lambda *_: True,
        event_sink=lambda kind, _state, _payload: events.append(kind),
    ).run(spec)

    assert result.completed
    assert events == [
        "moving",
        "motion_command_completed",
        "device_stationary_confirmed",
    ]
    assert "physical_stop_observed" not in events


def test_runner_propagates_backend_exception_after_one_stop():
    class Exploding(RejectingBackend):
        def command_pose(self, target):
            self.commands += 1
            raise RuntimeError("command failed")

    spec = load_trajectory(Path("configs/gen3_micro_axes.json"))
    backend = Exploding(999)
    with pytest.raises(RuntimeError, match="command failed"):
        FixedTrajectoryRunner(backend, approve_segment=lambda *_: True).run(spec)
    assert backend.hold_calls == 1
