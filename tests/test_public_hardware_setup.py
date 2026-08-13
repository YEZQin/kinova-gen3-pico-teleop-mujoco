from __future__ import annotations

import json
from pathlib import Path

import pytest

from kinova_teleop.motion_lease import create_motion_lease, validate_motion_lease
from kinova_teleop.public_hardware_setup import main


PHYSICAL_FLAGS = (
    "workspace_clear",
    "physical_estop_reachable",
    "teach_pendant_stop_reachable",
    "second_observer_present",
    "cable_slack_checked",
    "device_fixture_checked",
    "speed_level_checked",
    "workspace_bounds_checked",
    "load_tcp_checked",
)


def _calibration_payload() -> dict[str, object]:
    gestures = []
    for index, (label, endpoint) in enumerate(
        (("right", [0.02, 0.0, 0.0]), ("up", [0.0, 0.02, 0.0]), ("forward", [0.0, 0.0, 0.02])),
        start=2,
    ):
        gestures.append(
            {
                "label": label,
                "neutral_m": [0.0, 0.0, 0.0],
                "endpoint_m": endpoint,
                "raw_delta_m": endpoint,
                "unit_raw": [value / 0.02 for value in endpoint],
                "magnitude_m": 0.02,
                "endpoint_timestamp_ns": index,
            }
        )
    return {
        "schema_version": "1.0",
        "captured_utc": "2026-08-13T00:00:00Z",
        "controller": "left",
        "grip_released": True,
        "shared_neutral_m": [0.0, 0.0, 0.0],
        "neutral_timestamp_ns": 1,
        "gestures": gestures,
        "raw_operator_basis_columns_right_up_forward": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        "basis_determinant": 1.0,
        "basis_condition_number": 1.0,
        "pairwise_abs_dots": {"right_up": 0.0, "right_forward": 0.0, "up_forward": 0.0},
    }


def _t0_payload() -> dict[str, object]:
    checks = [
        {"name": name, "status": "pass", "detail": detail}
        for name, detail in (
            ("code_revision", "present"),
            ("dirty_worktree", "clean worktree"),
            ("runtime", "present"),
            ("driver", "present"),
            ("firmware", "present"),
            ("transport", "present"),
            ("calibration", "calibration hashes present"),
            ("safety_limits", "workspace and speed limits present"),
            ("arm_state", "SERVOING_READY"),
            ("feedback_pose", "finite tool pose"),
            ("product_model", "MODEL_ID_L53"),
            ("degree_of_freedom", "7 DoF"),
            ("firmware_version", "2.8.0-5"),
            ("operating_mode", "RUN_MODE"),
            ("servoing_mode", "SINGLE_LEVEL_SERVOING"),
        )
    ]
    return {
        "schema_version": "1.0",
        "device": "gen3",
        "timestamp_utc": "2026-08-13T00:00:00Z",
        "code_revision": "a" * 40,
        "dirty_worktree": False,
        "runtime": {"python": "3.11.9"},
        "driver": {"name": "kortex-api", "version": "2.8.0.post5"},
        "firmware": {"version": "2.8.0-5"},
        "transport": {"kind": "tcp", "host": "192.168.1.10", "port": 10000},
        "calibration": [{"name": "t0-only", "sha256": "b" * 64}],
        "safety_limits": {"workspace": "T0 only", "max_linear_speed": 0.02},
        "checks": checks,
        "physical_checks": {name: False for name in PHYSICAL_FLAGS},
        "passed": False,
        "reference_pose_m": [0.4, -0.1, 0.3],
    }


def _package_argv(t0: Path, calibration: Path, output_dir: Path) -> list[str]:
    return [
        "package", "--t0", str(t0), "--calibration", str(calibration),
        "--workspace-min", "0.0", "-0.3", "0.05",
        "--workspace-max", "1.0", "0.3", "0.6",
        "--linear-speed", "0.02", "--owner", "operator-1",
        "--output-dir", str(output_dir),
    ] + [f"--{name.replace('_', '-')}" for name in PHYSICAL_FLAGS]


def test_create_motion_lease_is_deterministic_and_validates_as_a_regular_artifact(tmp_path: Path) -> None:
    """Removing nonce-bound construction would make the emitted lease unverifiable."""
    lease = create_motion_lease("run-1", "operator-1", "2026-08-13T00:00:00Z", "nonce")
    path = tmp_path / "lease.json"
    path.write_text(json.dumps(lease.to_mapping()), encoding="utf-8")

    assert lease.lease_id == create_motion_lease("run-1", "operator-1", "2026-08-13T00:00:00Z", "nonce").lease_id
    assert validate_motion_lease(path, "run-1", "operator-1") == lease


def test_package_writes_bound_self_validating_artifacts_without_external_access(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Dropping T0/code/calibration binding would let a copied package pass."""
    import kinova_teleop.public_hardware_setup as setup

    output = tmp_path / "local-config"
    output.mkdir()
    t0 = output / "t0.json"
    calibration = output / "operator-axes.json"
    t0.write_text(json.dumps(_t0_payload()), encoding="utf-8")
    calibration.write_text(json.dumps(_calibration_payload()), encoding="utf-8")
    monkeypatch.setattr(setup, "_current_clean_code_revision", lambda: "a" * 40)
    monkeypatch.setattr(setup.secrets, "token_hex", lambda _size: "c" * 32)
    monkeypatch.setattr(setup.uuid, "uuid4", lambda: type("Id", (), {"hex": "d" * 32})())

    assert main(_package_argv(t0, calibration, output)) == 0

    profile = json.loads((output / "teleop-profile.json").read_text(encoding="utf-8"))
    report = json.loads((output / "reviewed-preflight.json").read_text(encoding="utf-8"))
    assert profile["workspace_midpoint_m"] == [0.5, 0.0, 0.325]
    assert profile["reference_pose_m"] == [0.4, -0.1, 0.3]
    assert profile["calibration_sha256"] != "b" * 64
    assert report["passed"] is True
    assert set(profile) == {
        "schema_version", "device", "robot_ip", "robot_user", "motion_lease",
        "preflight_report", "calibration", "workspace_min_m", "workspace_max_m",
        "max_linear_speed_m_s", "run_id", "lease_owner", "code_revision",
        "calibration_sha256", "driver_sha256", "reference_pose_m", "workspace_midpoint_m",
    }


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (("reference_pose_m", [float("nan"), 0.0, 0.3]), "finite"),
        (("transport", {"kind": "tcp", "host": "8.8.8.8", "port": 10000}), "private"),
        (("checks", []), "T0"),
    ],
)
def test_package_rejects_untrusted_t0_before_writing(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, mutation, message: str) -> None:
    """Accepting a stale or malformed T0 would manufacture motion authorization."""
    import kinova_teleop.public_hardware_setup as setup

    output = tmp_path / "local-config"
    output.mkdir()
    t0 = output / "t0.json"
    payload = _t0_payload()
    payload[mutation[0]] = mutation[1]
    t0.write_text(json.dumps(payload, allow_nan=True), encoding="utf-8")
    calibration = output / "operator-axes.json"
    calibration.write_text(json.dumps(_calibration_payload()), encoding="utf-8")
    monkeypatch.setattr(setup, "_current_clean_code_revision", lambda: "a" * 40)

    assert main(_package_argv(t0, calibration, output)) == 2
    assert not (output / "teleop-profile.json").exists()


@pytest.mark.parametrize(
    ("minimum", "maximum", "speed"),
    [((0, 0, -0.01), (1, 0.3, 0.6), 0.02), ((0, 0, 0.05), (1.21, 0.3, 0.6), 0.02), ((0, 0, 0.05), (1, 0.3, 0.6), 0.021)],
)
def test_package_rejects_unsafe_bounds_or_speed(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, minimum, maximum, speed: float) -> None:
    """Removing calibrated envelope checks would permit an unsafe profile."""
    import kinova_teleop.public_hardware_setup as setup

    output = tmp_path / "local-config"
    output.mkdir()
    t0 = output / "t0.json"
    calibration = output / "operator-axes.json"
    t0.write_text(json.dumps(_t0_payload()), encoding="utf-8")
    calibration.write_text(json.dumps(_calibration_payload()), encoding="utf-8")
    monkeypatch.setattr(setup, "_current_clean_code_revision", lambda: "a" * 40)
    args = _package_argv(t0, calibration, output)
    start = args.index("--workspace-min") + 1
    args[start:start + 3] = [str(value) for value in minimum]
    start = args.index("--workspace-max") + 1
    args[start:start + 3] = [str(value) for value in maximum]
    args[args.index("--linear-speed") + 1] = str(speed)

    assert main(args) == 2
