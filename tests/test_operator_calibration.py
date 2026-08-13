from __future__ import annotations

import copy
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from kinova_teleop.operator_calibration import load_operator_axis_calibration


RIGHT_UP_FORWARD_CAPTURE = {
    "schema_version": "1.0",
    "captured_utc": "2026-08-13T05:15:04.103968Z",
    "controller": "left",
    "grip_released": True,
    "shared_neutral_m": [-0.5928683578968048, 0.7679366767406464, 0.5482306480407715],
    "neutral_timestamp_ns": 7512837463000,
    "gestures": [
        {"label": "right", "neutral_m": [-0.5928683578968048, 0.7679366767406464, 0.5482306480407715], "endpoint_m": [-0.5213789343833923, 0.7676919996738434, 0.5053293704986572], "raw_delta_m": [0.07148942351341248, -0.0002446770668029785, -0.04290127754211426], "unit_raw": [0.8574488911729563, -0.0029346729811900647, -0.5145607706780233], "magnitude_m": 0.08337455940448854, "endpoint_timestamp_ns": 7514004153000},
        {"label": "up", "neutral_m": [-0.5928683578968048, 0.7679366767406464, 0.5482306480407715], "endpoint_m": [-0.5892895758152008, 0.8773105442523956, 0.5339113175868988], "raw_delta_m": [0.003578782081604004, 0.10937386751174927, -0.01431933045387268], "unit_raw": [0.03242670396139248, 0.9910170393293299, -0.12974489057041716], "magnitude_m": 0.110365274431374, "endpoint_timestamp_ns": 7521748909000},
        {"label": "forward", "neutral_m": [-0.5928683578968048, 0.7679366767406464, 0.5482306480407715], "endpoint_m": [-0.5392515063285828, 0.8045488595962524, 0.6564490795135498], "raw_delta_m": [0.053616851568222046, 0.03661218285560608, 0.10821843147277832], "unit_raw": [0.4248560915097345, 0.2901123145934903, 0.8575150998252332], "magnitude_m": 0.12620003017420206, "endpoint_timestamp_ns": 7529016038000},
    ],
    "raw_operator_basis_columns_right_up_forward": [[0.8574488911729563, 0.03242670396139248, 0.4248560915097345], [-0.0029346729811900647, 0.9910170393293299, 0.2901123145934903], [-0.5145607706780233, -0.12974489057041716, 0.8575150998252332]],
    "basis_determinant": 0.9729984772748919,
    "basis_condition_number": 1.2582795878764326,
    "pairwise_abs_dots": {"right_up": 0.09165756131032059, "right_forward": 0.07780263083216421, "up_forward": 0.1900247269976951},
}


def write_capture(tmp_path: Path, payload: object, *, raw: str | None = None) -> Path:
    path = tmp_path / "capture.json"
    path.write_bytes((raw if raw is not None else json.dumps(payload)).encode("utf-8"))
    return path


def test_loader_derives_expected_proper_rotation(tmp_path: Path) -> None:
    path = write_capture(tmp_path, RIGHT_UP_FORWARD_CAPTURE)
    loaded = load_operator_axis_calibration(path)
    np.testing.assert_allclose(loaded.translation_rotation, [[0.861288411785, 0.469277688168, -0.194835117758], [-0.470679958298, 0.881303368425, 0.042008923554], [0.191422696095, 0.055523186053, 0.979935981190]], atol=1e-9)
    np.testing.assert_allclose(np.asarray(loaded.translation_rotation).T @ np.asarray(loaded.translation_rotation), np.eye(3), atol=1e-12)
    assert isinstance(loaded.translation_rotation, tuple)
    assert loaded.source_sha256 == "91623595616e559737cf0265284d7ae35b074e255a8b8e1a39da3515e79f4dec"


@pytest.mark.parametrize("raw", [
    '{"schema_version":"1.0","schema_version":"1.0"}',
    '{"schema_version": NaN}',
])
def test_loader_rejects_unsafe_json(tmp_path: Path, raw: str) -> None:
    with pytest.raises(ValueError, match="operator calibration"):
        load_operator_axis_calibration(write_capture(tmp_path, {}, raw=raw))


@pytest.mark.parametrize("mutate", [
    lambda value: value.pop("captured_utc"),
    lambda value: value.update(extra=True),
    lambda value: value.update(controller="right"),
    lambda value: value.update(grip_released=False),
    lambda value: value["gestures"][2].update(label="ahead"),
    lambda value: value["gestures"][0].update(magnitude_m=0.014),
    lambda value: value.update(raw_operator_basis_columns_right_up_forward=[[1, 0], [0, 1]]),
    lambda value: value["gestures"][1].update(unit_raw=value["gestures"][0]["unit_raw"]),
    lambda value: value.update(raw_operator_basis_columns_right_up_forward=[[1, 0, 0], [0, 1, 0], [0, 0, 0.49]]),
    lambda value: value.update(raw_operator_basis_columns_right_up_forward=[[1, 0, 0], [0, 1, 0], [0, 0, -1]]),
    lambda value: value.update(basis_determinant=1.0),
    lambda value: value.update(basis_condition_number=1.0),
    lambda value: value.update(pairwise_abs_dots={"right_up": 0.0, "right_forward": 0.0, "up_forward": 0.0}),
])
def test_loader_rejects_malformed_or_low_quality_capture(tmp_path: Path, mutate) -> None:
    payload = copy.deepcopy(RIGHT_UP_FORWARD_CAPTURE)
    mutate(payload)
    with pytest.raises(ValueError, match="operator calibration"):
        load_operator_axis_calibration(write_capture(tmp_path, payload))


def test_loader_rejects_symlink(tmp_path: Path) -> None:
    target = write_capture(tmp_path, RIGHT_UP_FORWARD_CAPTURE)
    symlink = tmp_path / "capture-link.json"
    try:
        symlink.symlink_to(target)
    except OSError:
        pytest.skip("symlinks unavailable")
    with pytest.raises(ValueError, match="operator calibration"):
        load_operator_axis_calibration(symlink)


def test_loader_rejects_windows_reparse_file(tmp_path: Path, monkeypatch) -> None:
    target = write_capture(tmp_path, RIGHT_UP_FORWARD_CAPTURE)
    monkeypatch.setattr(Path, "is_symlink", lambda _: False)
    monkeypatch.setattr("kinova_teleop.operator_calibration.os.lstat", lambda _: SimpleNamespace(st_file_attributes=0x400))
    with pytest.raises(ValueError, match="operator calibration"):
        load_operator_axis_calibration(target)
