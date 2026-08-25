"""Interactive, fail-closed capture of a public PICO operator calibration."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
import errno
import json
import math
import os
from pathlib import Path
import secrets
import stat
from typing import Any

import numpy as np

from .operator_calibration import _validate_capture
from .xr_input import XrInputSource


_POSE_SEQUENCE = ("neutral", "right", "neutral", "up", "neutral", "forward")
_GESTURE_LABELS = ("right", "up", "forward")
_MIN_DISPLACEMENT_M = 0.015
_MAX_RETURN_TO_NEUTRAL_M = 0.015
_MAX_RELEASED_GRIP = 0.8
_REPARSE_POINT = 0x400


@dataclass(frozen=True, slots=True)
class CalibrationPrompts:
    """The operator acknowledgement required before each stationary pose."""

    wait_for_pose: Callable[[str], None]

    @classmethod
    def interactive(cls) -> "CalibrationPrompts":
        def wait_for_pose(label: str) -> None:
            input(
                f"Hold the released left controller at {label}. "
                "Press Enter when steady: "
            )

        return cls(wait_for_pose=wait_for_pose)


@dataclass(frozen=True, slots=True)
class _PicoSourceAdmission:
    active_source: tuple[str, int]
    foreign: int


def _capture_error(detail: str) -> ValueError:
    return ValueError(f"operator calibration capture {detail}")


def _read_released_sample(
    source: XrInputSource,
    *,
    after_timestamp_ns: int,
    admission: _PicoSourceAdmission | None,
) -> tuple[np.ndarray, int, _PicoSourceAdmission]:
    if admission is not None:
        _verify_source_health(source, admission)
    try:
        sample = source.read()
    except StopIteration as error:
        raise _capture_error("input ended before a pose was complete") from error

    if not sample.valid:
        reason = sample.invalid_reason or "invalid controller sample"
        raise _capture_error(reason)
    try:
        position = np.asarray(sample.position, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise _capture_error("controller position must be a finite 3-vector") from error
    if position.shape != (3,) or not np.isfinite(position).all():
        raise _capture_error("controller position must be a finite 3-vector")
    if isinstance(sample.grip, bool):
        raise _capture_error("requires Grip released below 0.8")
    try:
        grip = float(sample.grip)
    except (TypeError, ValueError) as error:
        raise _capture_error("requires Grip released below 0.8") from error
    if not math.isfinite(grip) or grip >= _MAX_RELEASED_GRIP:
        raise _capture_error("requires Grip released below 0.8")
    timestamp = sample.timestamp_ns
    if (
        isinstance(timestamp, bool)
        or not isinstance(timestamp, int)
        or timestamp <= after_timestamp_ns
    ):
        raise _capture_error("sample timestamps must be positive and strictly increasing")
    admitted = _verify_source_health(source, admission)
    return np.array(position, dtype=np.float64, copy=True), timestamp, admitted


def _require_health_reader(source: XrInputSource) -> None:
    if not callable(getattr(source, "health", None)):
        raise _capture_error("source health is unavailable")


def _read_source_health(source: XrInputSource) -> _PicoSourceAdmission | None:
    _require_health_reader(source)
    health_reader = getattr(source, "health")
    try:
        health = health_reader()
        active_source = health.active_source
        foreign = health.foreign
    except Exception as error:
        raise _capture_error("source health is unavailable") from error
    if (
        isinstance(foreign, bool)
        or not isinstance(foreign, int)
        or foreign < 0
    ):
        raise _capture_error("source health is unavailable")
    if active_source is None:
        return None
    if (
        not isinstance(active_source, tuple)
        or len(active_source) != 2
        or not isinstance(active_source[0], str)
        or isinstance(active_source[1], bool)
        or not isinstance(active_source[1], int)
        or active_source[1] <= 0
    ):
        raise _capture_error("source health is unavailable")
    return _PicoSourceAdmission(active_source=active_source, foreign=foreign)


def _verify_source_health(
    source: XrInputSource,
    admission: _PicoSourceAdmission | None,
) -> _PicoSourceAdmission:
    observed = _read_source_health(source)
    if observed is None:
        raise _capture_error("source health is unavailable")
    if admission is None:
        return observed
    if observed.active_source != admission.active_source:
        raise _capture_error("source changed during capture")
    if observed.foreign != admission.foreign:
        raise _capture_error("foreign source count changed during capture")
    return admission


def _capture_pose(
    source: XrInputSource,
    prompts: CalibrationPrompts,
    label: str,
    samples_per_pose: int,
    *,
    after_timestamp_ns: int,
    admission: _PicoSourceAdmission | None,
) -> tuple[np.ndarray, int, int, _PicoSourceAdmission]:
    prompts.wait_for_pose(label)
    positions: list[np.ndarray] = []
    timestamps: list[int] = []
    timestamp = after_timestamp_ns
    for _ in range(samples_per_pose):
        position, timestamp, admission = _read_released_sample(
            source,
            after_timestamp_ns=timestamp,
            admission=admission,
        )
        positions.append(position)
        timestamps.append(timestamp)
    median_position = np.median(np.asarray(positions, dtype=np.float64), axis=0)
    median_timestamp = sorted(timestamps)[len(timestamps) // 2]
    return (
        np.array(median_position, dtype=np.float64, copy=True),
        median_timestamp,
        timestamp,
        admission,
    )


def _require_return_to_neutral(neutral: np.ndarray, returned: np.ndarray) -> None:
    if float(np.linalg.norm(returned - neutral)) > _MAX_RETURN_TO_NEUTRAL_M:
        raise _capture_error("return to neutral exceeds 0.015 m")


def _gesture(
    label: str,
    neutral: np.ndarray,
    endpoint: np.ndarray,
    endpoint_timestamp_ns: int,
) -> dict[str, object]:
    delta = endpoint - neutral
    magnitude = float(np.linalg.norm(delta))
    if magnitude < _MIN_DISPLACEMENT_M:
        raise _capture_error("gesture displacement must be at least 0.015 m")
    unit = delta / magnitude
    return {
        "label": label,
        "neutral_m": neutral.tolist(),
        "endpoint_m": endpoint.tolist(),
        "raw_delta_m": delta.tolist(),
        "unit_raw": unit.tolist(),
        "magnitude_m": magnitude,
        "endpoint_timestamp_ns": endpoint_timestamp_ns,
    }


def _as_payload(
    neutral: np.ndarray,
    neutral_timestamp_ns: int,
    gestures: list[dict[str, object]],
) -> dict[str, object]:
    units = [np.asarray(gesture["unit_raw"], dtype=np.float64) for gesture in gestures]
    basis = np.column_stack(units)
    dots = {
        "right_up": abs(float(np.dot(units[0], units[1]))),
        "right_forward": abs(float(np.dot(units[0], units[2]))),
        "up_forward": abs(float(np.dot(units[1], units[2]))),
    }
    payload: dict[str, object] = {
        "schema_version": "1.0",
        "captured_utc": datetime.now(timezone.utc).isoformat(timespec="microseconds").replace("+00:00", "Z"),
        "controller": "left",
        "grip_released": True,
        "shared_neutral_m": neutral.tolist(),
        "neutral_timestamp_ns": neutral_timestamp_ns,
        "gestures": gestures,
        "raw_operator_basis_columns_right_up_forward": basis.tolist(),
        "basis_determinant": float(np.linalg.det(basis)),
        "basis_condition_number": float(np.linalg.cond(basis)),
        "pairwise_abs_dots": dots,
    }
    try:
        _validate_capture(payload)
    except ValueError as error:
        raise _capture_error(str(error).removeprefix("operator calibration ")) from error
    return payload


def capture_operator_calibration(
    source: XrInputSource,
    prompts: CalibrationPrompts,
    *,
    samples_per_pose: int = 20,
) -> dict[str, object]:
    """Capture released left-controller poses and return a strict artifact payload."""

    if isinstance(samples_per_pose, bool) or not isinstance(samples_per_pose, int) or samples_per_pose <= 0:
        raise ValueError("samples_per_pose must be a positive integer")

    _require_health_reader(source)
    neutral, neutral_timestamp, last_timestamp, admission = _capture_pose(
        source,
        prompts,
        _POSE_SEQUENCE[0],
        samples_per_pose,
        after_timestamp_ns=0,
        admission=None,
    )
    gestures: list[dict[str, object]] = []
    for label in _GESTURE_LABELS:
        endpoint, endpoint_timestamp, last_timestamp, admission = _capture_pose(
            source,
            prompts,
            label,
            samples_per_pose,
            after_timestamp_ns=last_timestamp,
            admission=admission,
        )
        gestures.append(_gesture(label, neutral, endpoint, endpoint_timestamp))
        if label != "forward":
            returned, _returned_timestamp, last_timestamp, admission = _capture_pose(
                source,
                prompts,
                "neutral",
                samples_per_pose,
                after_timestamp_ns=last_timestamp,
                admission=admission,
            )
            _require_return_to_neutral(neutral, returned)
    return _as_payload(neutral, neutral_timestamp, gestures)


def _reject_reparse_or_link(metadata: Any, label: str) -> None:
    if stat.S_ISLNK(metadata.st_mode):
        raise ValueError(f"calibration output {label} must not be a symbolic link")
    if getattr(metadata, "st_file_attributes", 0) & _REPARSE_POINT:
        raise ValueError(f"calibration output {label} must not be a Windows reparse point")


def _validate_output_parent(path: Path) -> None:
    parent = path.parent.absolute()
    current = Path(parent.anchor)
    for component in parent.parts[1:]:
        current /= component
        try:
            metadata = os.lstat(current)
        except OSError as error:
            raise ValueError("calibration output parent must already exist") from error
        _reject_reparse_or_link(metadata, "parent")
        if not stat.S_ISDIR(metadata.st_mode):
            raise ValueError("calibration output parent must be a directory")


def _validate_absent_output(path: Path) -> None:
    try:
        metadata = os.lstat(path)
    except FileNotFoundError:
        return
    _reject_reparse_or_link(metadata, "path")
    raise FileExistsError(f"calibration output already exists: {path}")


def _strict_json_bytes(payload: Mapping[str, object]) -> bytes:
    try:
        serialized = json.dumps(dict(payload), ensure_ascii=False, allow_nan=False, sort_keys=True, indent=2)
        decoded = json.loads(serialized)
    except (TypeError, ValueError, json.JSONDecodeError) as error:
        raise ValueError("calibration payload is not strict JSON") from error
    if not isinstance(decoded, dict):
        raise ValueError("calibration payload must be an object")
    _validate_capture(decoded)
    return (serialized + "\n").encode("utf-8")


def write_new_calibration(path: Path, payload: Mapping[str, object]) -> None:
    """Atomically publish one validated calibration without replacing any path."""

    output = Path(path)
    if not output.name:
        raise ValueError("calibration output path must name a file")
    encoded = _strict_json_bytes(payload)
    _validate_output_parent(output)
    _validate_absent_output(output)

    temporary = output.with_name(f".{output.name}.{secrets.token_hex(16)}.tmp")
    descriptor: int | None = None
    try:
        descriptor = os.open(
            temporary,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_BINARY", 0),
            0o600,
        )
        with os.fdopen(descriptor, "wb") as artifact:
            descriptor = None
            artifact.write(encoded)
            artifact.flush()
            os.fsync(artifact.fileno())
        try:
            os.link(temporary, output)
        except OSError as error:
            if error.errno in (errno.EEXIST, errno.EACCES):
                _validate_absent_output(output)
            raise
    finally:
        if descriptor is not None:
            os.close(descriptor)
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Capture a released PICO operator-axis calibration.")
    parser.add_argument("--output", type=Path, default=Path("local-config/operator-axes.json"))
    parser.add_argument("--samples-per-pose", type=int, default=20)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.samples_per_pose <= 0:
        print("error: --samples-per-pose must be positive")
        return 2
    from .main import create_pico_udp_input

    source = create_pico_udp_input()
    try:
        payload = capture_operator_calibration(
            source, CalibrationPrompts.interactive(), samples_per_pose=args.samples_per_pose
        )
        write_new_calibration(args.output, payload)
    except (OSError, RuntimeError, ValueError) as error:
        print(f"error: {error}")
        return 2
    finally:
        source.close()
    print(f"saved released left-controller calibration: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
