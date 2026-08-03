"""Validated relative Cartesian micro-trajectory execution."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Any

import numpy as np

from .backend import BackendResult
from .pose_mapping import Pose, normalize_quat


@dataclass(frozen=True, slots=True)
class RelativeSegment:
    name: str
    offsets_xyz: tuple[tuple[float, float, float], ...]
    orientation_xyzw: tuple[float, float, float, float] | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name:
            raise ValueError("trajectory segment name must be non-empty")
        if len(self.offsets_xyz) < 2:
            raise ValueError("trajectory segments need at least two offsets")
        normalized: list[tuple[float, float, float]] = []
        for point in self.offsets_xyz:
            try:
                values = tuple(float(value) for value in point)
            except (TypeError, ValueError) as error:
                raise ValueError("trajectory offsets must be finite XYZ triples") from error
            if len(values) != 3 or not np.isfinite(values).all():
                raise ValueError("trajectory offsets must be finite XYZ triples")
            normalized.append(values)  # type: ignore[arg-type]
        object.__setattr__(self, "offsets_xyz", tuple(normalized))
        if self.orientation_xyzw is not None:
            orientation = tuple(float(value) for value in self.orientation_xyzw)
            if len(orientation) != 4 or not np.isfinite(orientation).all():
                raise ValueError("trajectory orientation must be finite XYZW")
            object.__setattr__(self, "orientation_xyzw", orientation)  # type: ignore[arg-type]


@dataclass(frozen=True, slots=True)
class FixedTrajectorySpec:
    schema_version: str
    name: str
    max_offset_m: float
    segments: tuple[RelativeSegment, ...]
    control_hz: float = 25.0
    max_speed_mps: float = 0.01
    operator_approval_each_segment: bool = True

    def __post_init__(self) -> None:
        if self.schema_version != "1.0":
            raise ValueError("trajectory schema_version must be '1.0'")
        if not self.name:
            raise ValueError("trajectory name must be non-empty")
        values = np.asarray([self.max_offset_m, self.control_hz, self.max_speed_mps], dtype=float)
        if not np.isfinite(values).all() or self.max_offset_m <= 0 or self.control_hz <= 0 or self.max_speed_mps <= 0:
            raise ValueError("trajectory limits must be positive and finite")
        if self.control_hz > 40.0 or self.max_speed_mps > 0.03:
            raise ValueError("trajectory limits exceed Gen3 caps")
        if not self.segments:
            raise ValueError("trajectory must contain at least one segment")
        for segment in self.segments:
            if segment.offsets_xyz[-1] != (0.0, 0.0, 0.0):
                raise ValueError("each segment must return to its anchor")
            if max(abs(value) for point in segment.offsets_xyz for value in point) > self.max_offset_m + 1e-12:
                raise ValueError("trajectory offset exceeds max_offset_m")


@dataclass(frozen=True, slots=True)
class FixedTrajectoryResult:
    completed_segments: int
    total_segments: int
    rejected_segment: str | None = None
    reason: str = ""

    @property
    def completed(self) -> bool:
        return self.completed_segments == self.total_segments


def _strict_json(path: Path) -> Mapping[str, Any]:
    def reject_constant(value: str) -> None:
        raise ValueError(f"non-finite JSON constant is not allowed: {value}")

    try:
        payload = json.loads(path.read_text(encoding="utf-8"), parse_constant=reject_constant)
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as error:
        raise ValueError("trajectory JSON is invalid") from error
    if not isinstance(payload, Mapping):
        raise ValueError("trajectory JSON must be an object")
    return payload


def load_trajectory(path: str | Path) -> FixedTrajectorySpec:
    payload = _strict_json(Path(path))
    required = {"schema_version", "name", "max_offset_m", "segments"}
    missing = required - set(payload)
    if missing:
        raise ValueError(f"trajectory missing required fields: {', '.join(sorted(missing))}")
    allowed = required | {
        "control_hz", "max_speed_mps", "operator_approval_each_segment"
    }
    extra = set(payload) - allowed
    if extra:
        raise ValueError(f"trajectory has unexpected fields: {', '.join(sorted(map(str, extra)))}")
    segments_raw = payload["segments"]
    if not isinstance(segments_raw, Sequence) or isinstance(segments_raw, (str, bytes, bytearray)):
        raise ValueError("trajectory segments must be an array")
    segments: list[RelativeSegment] = []
    for index, raw in enumerate(segments_raw):
        if not isinstance(raw, Mapping):
            raise ValueError(f"trajectory segment {index} must be an object")
        if set(raw) - {"name", "offsets_xyz", "orientation_xyzw"}:
            raise ValueError(f"trajectory segment {index} has unexpected fields")
        offsets = raw.get("offsets_xyz")
        if not isinstance(offsets, Sequence) or isinstance(offsets, (str, bytes, bytearray)):
            raise ValueError(f"trajectory segment {index} offsets_xyz must be an array")
        segments.append(
            RelativeSegment(
                name=str(raw.get("name", f"segment-{index + 1}")),
                offsets_xyz=tuple(tuple(point) for point in offsets),
                orientation_xyzw=(tuple(raw["orientation_xyzw"]) if raw.get("orientation_xyzw") is not None else None),
            )
        )
    approval = payload.get("operator_approval_each_segment", True)
    if not isinstance(approval, bool):
        raise ValueError("operator_approval_each_segment must be a boolean")
    return FixedTrajectorySpec(
        schema_version=str(payload["schema_version"]),
        name=str(payload["name"]),
        max_offset_m=float(payload["max_offset_m"]),
        segments=tuple(segments),
        control_hz=float(payload.get("control_hz", 25.0)),
        max_speed_mps=float(payload.get("max_speed_mps", 0.01)),
        operator_approval_each_segment=approval,
    )


def validate_trajectory(spec: FixedTrajectorySpec, *, anchor: Pose | None = None) -> None:
    """Re-run pure validation, optionally checking absolute targets at anchor."""

    if not isinstance(spec, FixedTrajectorySpec):
        raise TypeError("spec must be a FixedTrajectorySpec")
    if anchor is None:
        return
    position = np.asarray(anchor.position, dtype=float)
    if position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError("anchor position must be finite")
    for segment in spec.segments:
        for offset in segment.offsets_xyz:
            if not np.isfinite(position + np.asarray(offset, dtype=float)).all():
                raise ValueError("trajectory target is non-finite")


class FixedTrajectoryRunner:
    """Run one bounded relative trajectory, stopping at the first rejection."""

    def __init__(
        self,
        backend: Any,
        *,
        control_hz: float = 25.0,
        sleep: Callable[[float], None] = time.sleep,
        approve_segment: Callable[[RelativeSegment, int], bool] | None = None,
        event_sink: Callable[[str, str, Mapping[str, object]], None] | None = None,
    ) -> None:
        if not math.isfinite(control_hz) or control_hz <= 0.0 or control_hz > 40.0:
            raise ValueError("control_hz must be in (0, 40]")
        self.backend = backend
        self.control_hz = float(control_hz)
        self.sleep = sleep
        self.approve_segment = approve_segment
        self.event_sink = event_sink

    def _emit(self, kind: str, state: str, payload: Mapping[str, object]) -> None:
        if self.event_sink is not None:
            self.event_sink(kind, state, dict(payload))

    def _stop_once(self, stopped: bool) -> bool:
        if stopped or getattr(self, "_stop_attempted", False):
            return True
        # Mark before invoking the backend so a raising Stop is not retried by
        # the outer exception path.
        self._stop_attempted = True
        self.backend.hold()
        return True

    def run(self, spec: FixedTrajectorySpec) -> FixedTrajectoryResult:
        validate_trajectory(spec)
        self._stop_attempted = False
        self.backend.begin_control()
        stopped = False
        try:
            anchor = self.backend.current_pose()
            validate_trajectory(spec, anchor=anchor)
            completed = 0
            for index, segment in enumerate(spec.segments):
                if spec.operator_approval_each_segment:
                    # A fixed trajectory is intentionally fail-closed: a
                    # caller must inject an explicit operator approval hook
                    # for every segment.  Silently treating a missing hook as
                    # approval would turn the checked-in micro-trajectory into
                    # an unattended motion sequence.
                    if self.approve_segment is None:
                        reason = "operator approval callback is required"
                        self._emit(
                            "command_rejected",
                            "STOPPING",
                            {"segment": segment.name, "reason": reason},
                        )
                        stopped = self._stop_once(stopped)
                        return FixedTrajectoryResult(
                            completed, len(spec.segments), segment.name, reason
                        )
                    if not self.approve_segment(segment, index):
                        reason = "operator approval denied"
                        self._emit(
                            "command_rejected",
                            "STOPPING",
                            {"segment": segment.name, "reason": reason},
                        )
                        stopped = self._stop_once(stopped)
                        return FixedTrajectoryResult(
                            completed, len(spec.segments), segment.name, reason
                        )
                self._emit(
                    "moving",
                    "MOVING",
                    {"segment": segment.name, "index": index},
                )
                for point in segment.offsets_xyz:
                    target = Pose(
                        np.asarray(anchor.position, dtype=float) + np.asarray(point, dtype=float),
                        np.asarray(anchor.quaternion, dtype=float).copy(),
                    )
                    result: BackendResult = self.backend.command_pose(target)
                    if not result.accepted:
                        self._emit("command_rejected", "STOPPING", {"segment": segment.name, "reason": result.reason})
                        stopped = self._stop_once(stopped)
                        return FixedTrajectoryResult(completed, len(spec.segments), segment.name, result.reason)
                    self.backend.step()
                confirmer = getattr(self.backend, "confirm_stationary", None)
                if not callable(confirmer):
                    reason = "stationary confirmation is unavailable"
                    self._emit(
                        "command_rejected",
                        "STOPPING",
                        {"segment": segment.name, "reason": reason},
                    )
                    stopped = self._stop_once(stopped)
                    return FixedTrajectoryResult(completed, len(spec.segments), segment.name, reason)
                if not bool(confirmer()):
                    reason = "stationary confirmation failed"
                    self._emit(
                        "command_rejected",
                        "STOPPING",
                        {"segment": segment.name, "reason": reason},
                    )
                    stopped = self._stop_once(stopped)
                    return FixedTrajectoryResult(completed, len(spec.segments), segment.name, reason)
                self._emit("motion_command_completed", "MOVING", {"segment": segment.name})
                if callable(confirmer):
                    self._emit(
                        "device_stationary_confirmed",
                        "STOPPING",
                        {"segment": segment.name},
                    )
                completed += 1
            stopped = self._stop_once(stopped)
            return FixedTrajectoryResult(completed, len(spec.segments))
        except BaseException:
            self._stop_once(stopped)
            raise
