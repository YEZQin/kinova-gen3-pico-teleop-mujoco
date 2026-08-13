from __future__ import annotations

import json
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from kinova_teleop.calibration_capture import (
    CalibrationPrompts,
    capture_operator_calibration,
    write_new_calibration,
)
from kinova_teleop.operator_calibration import load_operator_axis_calibration
from kinova_teleop.xr_input import ControllerSample


class ScriptedSource:
    def __init__(
        self,
        samples: list[ControllerSample],
        *,
        active_source: tuple[str, int] = ("192.0.2.10", 15031),
        foreign_after_reads: int | None = None,
        changed_source_after_reads: int | None = None,
    ) -> None:
        self._samples = iter(samples)
        self._active_source = active_source
        self._foreign_after_reads = foreign_after_reads
        self._changed_source_after_reads = changed_source_after_reads
        self._read_count = 0
        self.closed = False

    def read(self) -> ControllerSample:
        self._read_count += 1
        return next(self._samples)

    def health(self) -> SimpleNamespace:
        active_source = self._active_source
        if (
            self._changed_source_after_reads is not None
            and self._read_count >= self._changed_source_after_reads
        ):
            active_source = ("192.0.2.11", 15031)
        foreign = (
            1
            if self._foreign_after_reads is not None
            and self._read_count >= self._foreign_after_reads
            else 0
        )
        return SimpleNamespace(active_source=active_source, foreign=foreign)

    def close(self) -> None:
        self.closed = True


def sample(
    position: tuple[float, float, float],
    timestamp_ns: int,
    *,
    grip: float = 0.0,
    valid: bool = True,
    reason: str = "",
) -> ControllerSample:
    return ControllerSample(
        position=np.asarray(position, dtype=np.float64),
        quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        grip=grip,
        timestamp_ns=timestamp_ns,
        received_monotonic=float(timestamp_ns) / 1_000_000_000,
        valid=valid,
        invalid_reason=reason,
    )


def samples_at(
    position: tuple[float, float, float],
    timestamp_start: int,
    count: int = 3,
) -> list[ControllerSample]:
    return [
        sample(
            tuple(value + offset for value in position),
            timestamp_start + index,
        )
        for index, offset in enumerate((-0.001, 0.0, 0.001))
    ][:count]


def valid_capture_samples() -> list[ControllerSample]:
    neutral = (0.1, 0.2, 0.3)
    return [
        *samples_at(neutral, 1),
        *samples_at((0.13, 0.2, 0.3), 10),
        *samples_at((0.102, 0.199, 0.3), 20),
        *samples_at((0.1, 0.23, 0.3), 30),
        *samples_at((0.099, 0.201, 0.298), 40),
        *samples_at((0.1, 0.2, 0.33), 50),
    ]


def no_op_prompts() -> CalibrationPrompts:
    return CalibrationPrompts(wait_for_pose=lambda _label: None)


def test_capture_uses_medians_and_emits_the_strict_shared_neutral_schema() -> None:
    prompts_seen: list[str] = []
    payload = capture_operator_calibration(
        ScriptedSource(valid_capture_samples()),
        CalibrationPrompts(wait_for_pose=prompts_seen.append),
        samples_per_pose=3,
    )

    assert prompts_seen == ["neutral", "right", "neutral", "up", "neutral", "forward"]
    assert set(payload) == {
        "schema_version", "captured_utc", "controller", "grip_released",
        "shared_neutral_m", "neutral_timestamp_ns", "gestures",
        "raw_operator_basis_columns_right_up_forward", "basis_determinant",
        "basis_condition_number", "pairwise_abs_dots",
    }
    assert payload["shared_neutral_m"] == [0.1, 0.2, 0.3]
    assert payload["neutral_timestamp_ns"] == 2
    assert payload["controller"] == "left"
    assert payload["grip_released"] is True
    gestures = payload["gestures"]
    assert [gesture["label"] for gesture in gestures] == ["right", "up", "forward"]
    assert [gesture["endpoint_timestamp_ns"] for gesture in gestures] == [11, 31, 51]
    assert all(gesture["neutral_m"] == [0.1, 0.2, 0.3] for gesture in gestures)
    assert payload["basis_determinant"] > 0.0
    assert payload["basis_condition_number"] <= 2.0
    assert max(payload["pairwise_abs_dots"].values()) <= 0.35
    assert all(gesture["magnitude_m"] >= 0.015 for gesture in gestures)


def test_capture_output_loads_through_the_strict_loader(tmp_path: Path) -> None:
    payload = capture_operator_calibration(
        ScriptedSource(valid_capture_samples()), no_op_prompts(), samples_per_pose=3
    )
    output = tmp_path / "operator-axes.json"

    write_new_calibration(output, payload)

    assert load_operator_axis_calibration(output).translation_rotation


def test_capture_requires_stable_admitted_pico_source_health() -> None:
    payload = capture_operator_calibration(
        ScriptedSource(valid_capture_samples()), no_op_prompts(), samples_per_pose=3
    )

    assert payload["controller"] == "left"


@pytest.mark.parametrize(
    ("source", "error"),
    [
        (
            ScriptedSource(valid_capture_samples(), changed_source_after_reads=1),
            "source changed",
        ),
        (
            ScriptedSource(valid_capture_samples(), foreign_after_reads=1),
            "foreign",
        ),
        (ScriptedSource(valid_capture_samples(), active_source=None), "unavailable"),  # type: ignore[arg-type]
        (object(), "source health is unavailable"),
    ],
)
def test_capture_rejects_unavailable_or_changed_pico_source_health(
    source: object,
    error: str,
) -> None:
    with pytest.raises(ValueError, match=error):
        capture_operator_calibration(source, no_op_prompts(), samples_per_pose=3)  # type: ignore[arg-type]


def test_capture_factory_disables_stale_source_handoff_by_default(monkeypatch) -> None:
    created: list[dict[str, object]] = []
    monkeypatch.setattr(
        "kinova_teleop.main.PicoUdpInput",
        lambda **kwargs: created.append(kwargs) or object(),
    )
    monkeypatch.setattr("kinova_teleop.main.ContinuousInputBuffer", lambda source: source)
    from kinova_teleop.main import create_pico_udp_input

    create_pico_udp_input()

    assert created == [{
        "host": "0.0.0.0", "port": 15031, "stale_after": 0.2,
        "allow_stale_source_handoff": False,
    }]


@pytest.mark.parametrize(
    ("bad_sample", "error"),
    [
        (sample((0.0, 0.0, 0.0), 1, valid=False, reason="stream is stale"), "stale"),
        (sample((0.0, 0.0, 0.0), 1, valid=False, reason="source changed"), "source changed"),
        (sample((0.0, 0.0, 0.0), 1, grip=0.8), "Grip"),
    ],
)
def test_capture_rejects_invalid_or_pressed_samples(
    bad_sample: ControllerSample,
    error: str,
) -> None:
    with pytest.raises(ValueError, match=error):
        capture_operator_calibration(
            ScriptedSource([bad_sample]), no_op_prompts(), samples_per_pose=1
        )


def test_capture_rejects_duplicate_timestamps_and_early_eof() -> None:
    duplicate = valid_capture_samples()
    duplicate[1] = sample((0.1, 0.2, 0.3), 1)
    with pytest.raises(ValueError, match="timestamp"):
        capture_operator_calibration(ScriptedSource(duplicate), no_op_prompts(), samples_per_pose=3)

    with pytest.raises(ValueError, match="ended"):
        capture_operator_calibration(ScriptedSource([]), no_op_prompts(), samples_per_pose=1)


@pytest.mark.parametrize(
    ("samples", "error"),
    [
        (valid_capture_samples()[:3] + samples_at((0.11, 0.2, 0.3), 10) + valid_capture_samples()[6:], "0.015"),
        (valid_capture_samples()[:6] + samples_at((0.2, 0.2, 0.3), 20) + valid_capture_samples()[9:], "neutral"),
        (
            valid_capture_samples()[:9]
            + samples_at((0.13, 0.23, 0.3), 30)
            + valid_capture_samples()[12:],
            "condition|pairwise",
        ),
    ],
)
def test_capture_rejects_unsafe_displacement_return_or_mixed_axes(
    samples: list[ControllerSample],
    error: str,
) -> None:
    with pytest.raises(ValueError, match=error):
        capture_operator_calibration(ScriptedSource(samples), no_op_prompts(), samples_per_pose=3)


def test_write_is_exclusive_and_atomic(tmp_path: Path) -> None:
    payload = capture_operator_calibration(
        ScriptedSource(valid_capture_samples()), no_op_prompts(), samples_per_pose=3
    )
    output = tmp_path / "operator-axes.json"
    write_new_calibration(output, payload)
    original = output.read_bytes()

    with pytest.raises(FileExistsError):
        write_new_calibration(output, payload)

    assert output.read_bytes() == original
    assert json.loads(original) == payload
    assert not list(tmp_path.glob(".operator-axes.json.*.tmp"))


def test_write_refuses_symlink_output(tmp_path: Path) -> None:
    payload = capture_operator_calibration(
        ScriptedSource(valid_capture_samples()), no_op_prompts(), samples_per_pose=3
    )
    target = tmp_path / "target.json"
    target.write_text("do not replace", encoding="utf-8")
    symlink = tmp_path / "operator-axes.json"
    try:
        symlink.symlink_to(target)
    except OSError:
        pytest.skip("symbolic links are unavailable")

    with pytest.raises(ValueError, match="symbolic link"):
        write_new_calibration(symlink, payload)
    assert target.read_text(encoding="utf-8") == "do not replace"


def test_write_refuses_windows_reparse_output(tmp_path: Path, monkeypatch) -> None:
    payload = capture_operator_calibration(
        ScriptedSource(valid_capture_samples()), no_op_prompts(), samples_per_pose=3
    )
    reparse = tmp_path / "reparse.json"
    reparse.write_text("do not replace", encoding="utf-8")
    metadata = os.lstat(reparse)
    monkeypatch.setattr(
        "kinova_teleop.calibration_capture.os.lstat",
        lambda path: type("Metadata", (), {
            "st_mode": metadata.st_mode,
            "st_file_attributes": 0x400,
        })(),
    )
    with pytest.raises(ValueError, match="reparse"):
        write_new_calibration(reparse, payload)


def test_capture_script_has_no_robot_parameters_and_stays_in_local_config() -> None:
    script = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "capture_pico_operator_calibration.ps1"
    ).read_text(encoding="utf-8")

    assert ".venv-kortex\\Scripts\\python.exe" in script
    assert "never connects to Kortex" in script
    assert "OutputPath must be beneath local-config" in script
    assert "RobotIp" not in script
    assert "Password" not in script
