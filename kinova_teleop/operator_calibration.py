"""Strict loader for recorded operator-axis calibration captures."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import stat
from typing import Any

import numpy as np

from kinova_teleop.pose_mapping import PICO_TO_WORLD


_TOP_LEVEL_FIELDS = frozenset({
    "schema_version", "captured_utc", "controller", "grip_released",
    "shared_neutral_m", "neutral_timestamp_ns", "gestures",
    "raw_operator_basis_columns_right_up_forward", "basis_determinant",
    "basis_condition_number", "pairwise_abs_dots",
})
_GESTURE_FIELDS = frozenset({
    "label", "neutral_m", "endpoint_m", "raw_delta_m", "unit_raw",
    "magnitude_m", "endpoint_timestamp_ns",
})
_LABELS = ("right", "up", "forward")
_PAIR_KEYS = ("right_up", "right_forward", "up_forward")
_REPARSE_POINT = 0x400
_ATOL = 1e-12


@dataclass(frozen=True, slots=True)
class OperatorAxisCalibration:
    translation_rotation: tuple[tuple[float, float, float], ...]
    source_sha256: str


def _fail(detail: str) -> ValueError:
    return ValueError(f"operator calibration {detail}")


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise _fail("contains duplicate JSON keys")
        result[key] = value
    return result


def _reject_non_finite_constant(value: str) -> None:
    raise _fail(f"contains non-finite JSON constant {value}")


def _load_strict_regular_json(path: Path) -> tuple[dict[str, Any], bytes]:
    try:
        if path.is_symlink():
            raise _fail("must not be a symbolic link")
        metadata = os.lstat(path)
        if getattr(metadata, "st_file_attributes", 0) & _REPARSE_POINT:
            raise _fail("must not be a Windows reparse point")
        if not stat.S_ISREG(metadata.st_mode):
            raise _fail("must be a regular file")
        raw = path.read_bytes()
        decoded = raw.decode("utf-8")
        payload = json.loads(
            decoded,
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_non_finite_constant,
        )
    except ValueError as exc:
        if str(exc).startswith("operator calibration"):
            raise
        raise _fail("is not valid strict JSON") from exc
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise _fail("is not a readable strict UTF-8 JSON file") from exc
    if not isinstance(payload, dict):
        raise _fail("top level must be an object")
    return payload, raw


def _finite_vector(value: Any, name: str) -> np.ndarray:
    if not isinstance(value, list) or len(value) != 3:
        raise _fail(f"{name} must be a 3-vector")
    if any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in value):
        raise _fail(f"{name} must contain numeric values")
    vector = np.asarray(value, dtype=np.float64)
    if not np.isfinite(vector).all():
        raise _fail(f"{name} must be finite")
    return vector


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value):
        raise _fail(f"{name} must be a finite number")
    return float(value)


def _validate_capture(payload: dict[str, Any]) -> np.ndarray:
    if set(payload) != _TOP_LEVEL_FIELDS:
        raise _fail("has missing or unexpected top-level fields")
    if payload["schema_version"] != "1.0" or not isinstance(payload["captured_utc"], str):
        raise _fail("has an unsupported schema")
    if payload["controller"] != "left" or payload["grip_released"] is not True:
        raise _fail("requires the released left controller")
    neutral_timestamp = payload["neutral_timestamp_ns"]
    if isinstance(neutral_timestamp, bool) or not isinstance(neutral_timestamp, int) or neutral_timestamp <= 0:
        raise _fail("neutral_timestamp_ns must be a positive integer")
    shared_neutral = _finite_vector(payload["shared_neutral_m"], "shared_neutral_m")
    gestures = payload["gestures"]
    if not isinstance(gestures, list) or len(gestures) != 3:
        raise _fail("must contain exactly three gestures")

    units: list[np.ndarray] = []
    last_timestamp = neutral_timestamp
    for expected_label, gesture in zip(_LABELS, gestures):
        if not isinstance(gesture, dict) or set(gesture) != _GESTURE_FIELDS:
            raise _fail("gesture has missing or unexpected fields")
        if gesture["label"] != expected_label:
            raise _fail("gesture labels must be right, up, forward in order")
        neutral = _finite_vector(gesture["neutral_m"], f"{expected_label}.neutral_m")
        endpoint = _finite_vector(gesture["endpoint_m"], f"{expected_label}.endpoint_m")
        delta = _finite_vector(gesture["raw_delta_m"], f"{expected_label}.raw_delta_m")
        unit = _finite_vector(gesture["unit_raw"], f"{expected_label}.unit_raw")
        magnitude = _finite_number(gesture["magnitude_m"], f"{expected_label}.magnitude_m")
        timestamp = gesture["endpoint_timestamp_ns"]
        if isinstance(timestamp, bool) or not isinstance(timestamp, int) or timestamp <= last_timestamp:
            raise _fail("gesture timestamps must be strictly increasing")
        if not np.allclose(neutral, shared_neutral, rtol=0.0, atol=_ATOL):
            raise _fail("gesture neutral does not match shared neutral")
        if not np.allclose(endpoint - neutral, delta, rtol=0.0, atol=_ATOL):
            raise _fail("endpoint minus neutral does not match raw delta")
        recomputed_magnitude = float(np.linalg.norm(delta))
        if magnitude < 0.015 or not np.isclose(recomputed_magnitude, magnitude, rtol=0.0, atol=_ATOL):
            raise _fail("gesture displacement must be at least 0.015 m and consistent")
        if not np.allclose(delta / recomputed_magnitude, unit, rtol=0.0, atol=_ATOL):
            raise _fail("gesture unit vector is inconsistent")
        units.append(unit)
        last_timestamp = timestamp

    matrix_value = payload["raw_operator_basis_columns_right_up_forward"]
    if not isinstance(matrix_value, list) or len(matrix_value) != 3:
        raise _fail("raw basis must be a 3-by-3 matrix")
    basis = np.asarray([_finite_vector(row, "raw basis row") for row in matrix_value], dtype=np.float64)
    if not np.allclose(basis, np.column_stack(units), rtol=0.0, atol=_ATOL):
        raise _fail("raw basis does not match gesture unit vectors")
    determinant = float(np.linalg.det(basis))
    condition = float(np.linalg.cond(basis))
    dots = {
        "right_up": abs(float(np.dot(units[0], units[1]))),
        "right_forward": abs(float(np.dot(units[0], units[2]))),
        "up_forward": abs(float(np.dot(units[1], units[2]))),
    }
    if determinant <= 0.0 or condition > 2.0 or max(dots.values()) > 0.35:
        raise _fail("raw basis fails quality thresholds")
    if not np.isclose(_finite_number(payload["basis_determinant"], "basis_determinant"), determinant, rtol=0.0, atol=_ATOL):
        raise _fail("basis determinant summary is inconsistent")
    if not np.isclose(_finite_number(payload["basis_condition_number"], "basis_condition_number"), condition, rtol=0.0, atol=_ATOL):
        raise _fail("basis condition summary is inconsistent")
    reported_dots = payload["pairwise_abs_dots"]
    if not isinstance(reported_dots, dict) or set(reported_dots) != set(_PAIR_KEYS):
        raise _fail("pairwise dot summary has invalid fields")
    for key, dot in dots.items():
        if not np.isclose(_finite_number(reported_dots[key], key), dot, rtol=0.0, atol=_ATOL):
            raise _fail("pairwise dot summary is inconsistent")
    return basis


def load_operator_axis_calibration(path: str | Path) -> OperatorAxisCalibration:
    """Load one audited operator-axis capture without trusting its summaries."""

    payload, raw = _load_strict_regular_json(Path(path))
    basis = _validate_capture(payload)
    u, _singular_values, vt = np.linalg.svd(basis)
    orthogonal_basis = u @ vt
    rotation = PICO_TO_WORLD @ orthogonal_basis.T @ PICO_TO_WORLD.T
    return OperatorAxisCalibration(
        tuple(tuple(float(value) for value in row) for row in rotation),
        hashlib.sha256(raw).hexdigest(),
    )
