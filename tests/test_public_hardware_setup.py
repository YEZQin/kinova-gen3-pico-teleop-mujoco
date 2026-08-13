from __future__ import annotations

from datetime import datetime, timezone
import hashlib
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


def _t0_observation_digest(payload: dict[str, object]) -> str:
    fields = (
        "device", "timestamp_utc", "code_revision", "dirty_worktree", "runtime",
        "driver", "firmware", "transport", "safety_limits", "checks",
        "physical_checks", "passed", "reference_pose_m",
    )
    evidence = {field: payload[field] for field in fields}
    encoded = json.dumps(evidence, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def _t0_payload(*, arm_detail: str = "SERVOING_READY", robot_user: str = "operator") -> dict[str, object]:
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
            ("arm_state", arm_detail),
            ("feedback_pose", "finite tool pose"),
            ("product_model", "MODEL_ID_L53"),
            ("degree_of_freedom", "7 DoF"),
            ("firmware_version", "2.8.0-5"),
            ("operating_mode", "RUN_MODE"),
            ("servoing_mode", "SINGLE_LEVEL_SERVOING"),
        )
    ]
    checks.extend(
        {"name": f"physical.{name}", "status": "fail", "detail": "not confirmed"}
        for name in PHYSICAL_FLAGS
    )
    payload: dict[str, object] = {
        "schema_version": "1.0",
        "device": "gen3",
        "timestamp_utc": "2026-08-13T00:00:00Z",
        "code_revision": "a" * 40,
        "dirty_worktree": False,
        "runtime": {"python": "3.11.9", "kortex_api": "2.8.0.post5", "protobuf": "3.20.0"},
        "driver": {"name": "kortex-api", "version": "2.8.0.post5"},
        "firmware": {"version": "2.8.0-5"},
        "transport": {"kind": "tcp", "host": "192.168.1.10", "port": 10000, "robot_user": robot_user},
        "calibration": [{"name": "t0-observation", "sha256": ""}],
        "safety_limits": {"workspace": "read-only-t0", "max_linear_speed": 0.02},
        "checks": checks,
        "physical_checks": {name: False for name in PHYSICAL_FLAGS},
        "passed": False,
        "reference_pose_m": [0.4, -0.1, 0.3],
    }
    payload["calibration"] = [{"name": "t0-observation", "sha256": _t0_observation_digest(payload)}]
    return payload


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
    monkeypatch.setattr(setup, "_now_utc", lambda: datetime(2026, 8, 13, tzinfo=timezone.utc))
    monkeypatch.setattr(setup.secrets, "token_hex", lambda _size: "c" * 32)
    monkeypatch.setattr(setup.uuid, "uuid4", lambda: type("Id", (), {"hex": "d" * 32})())

    assert main(_package_argv(t0, calibration, output)) == 0

    profile = json.loads((output / "teleop-profile.json").read_text(encoding="utf-8"))
    report = json.loads((output / "reviewed-preflight.json").read_text(encoding="utf-8"))
    assert profile["workspace_midpoint_m"] == [0.5, 0.0, 0.325]
    assert profile["reference_pose_m"] == [0.4, -0.1, 0.3]
    assert profile["robot_user"] == "operator"
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
    monkeypatch.setattr(setup, "_now_utc", lambda: datetime(2026, 8, 13, tzinfo=timezone.utc))

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
    monkeypatch.setattr(setup, "_now_utc", lambda: datetime(2026, 8, 13, tzinfo=timezone.utc))
    args = _package_argv(t0, calibration, output)
    start = args.index("--workspace-min") + 1
    args[start:start + 3] = [str(value) for value in minimum]
    start = args.index("--workspace-max") + 1
    args[start:start + 3] = [str(value) for value in maximum]
    args[args.index("--linear-speed") + 1] = str(speed)

    assert main(args) == 2


@pytest.mark.parametrize("arm_detail", ("SERVOING_READY", "SERVOING_MANUALLY_CONTROLLED"))
def test_t0_accepts_only_observed_manual_control_or_ready_state(arm_detail: str) -> None:
    """Changing the admitted state set would allow an unobserved servoing mode."""
    import kinova_teleop.public_hardware_setup as setup

    robot_ip, robot_user, reference = setup._validate_t0(
        _t0_payload(arm_detail=arm_detail),
        "a" * 40,
        now=datetime(2026, 8, 13, tzinfo=timezone.utc),
    )

    assert (robot_ip, robot_user, reference) == ("192.168.1.10", "operator", (0.4, -0.1, 0.3))


@pytest.mark.parametrize(
    ("mutation", "value"),
    [
        (("passed",), True),
        (("physical_checks", "workspace_clear"), True),
        (("runtime", "protobuf"), "unknown"),
        (("firmware", "version"), "expected-2.8.0-5"),
        (("transport", "robot_user"), ""),
        (("calibration",), [{"name": "t0-observation", "sha256": "a" * 64}]),
        (("checks", 8, "detail"), "ARMSTATE_SERVOING_AUTONOMOUS"),
        (("checks", 8, "status"), "unknown"),
    ],
)
def test_t0_rejects_non_observational_or_copied_evidence(mutation, value) -> None:
    """Relaxing exact T0 evidence would turn a read-only observation into authorization."""
    import kinova_teleop.public_hardware_setup as setup

    payload = _t0_payload()
    target: object = payload
    for key in mutation[:-1]:
        target = target[key]  # type: ignore[index]
    target[mutation[-1]] = value  # type: ignore[index]
    if mutation[0] not in {"calibration", "checks"}:
        payload["calibration"] = [{"name": "t0-observation", "sha256": _t0_observation_digest(payload)}]

    with pytest.raises(ValueError):
        setup._validate_t0(payload, "a" * 40, now=datetime(2026, 8, 13, tzinfo=timezone.utc))


def test_package_publish_failure_leaves_no_final_artifact(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Publishing one artifact before another must not leave a partially valid package."""
    import kinova_teleop.public_hardware_setup as setup

    output = tmp_path / "local-config"
    output.mkdir()
    t0, calibration = output / "t0.json", output / "operator-axes.json"
    t0.write_text(json.dumps(_t0_payload()), encoding="utf-8")
    calibration.write_text(json.dumps(_calibration_payload()), encoding="utf-8")
    monkeypatch.setattr(setup, "_current_clean_code_revision", lambda: "a" * 40)
    monkeypatch.setattr(setup, "_now_utc", lambda: datetime(2026, 8, 13, tzinfo=timezone.utc))
    real_link = setup.os.link
    calls = 0

    def fail_second_publish(source, destination, *args, **kwargs):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise OSError("simulated publish failure")
        return real_link(source, destination, *args, **kwargs)

    monkeypatch.setattr(setup.os, "link", fail_second_publish)

    assert main(_package_argv(t0, calibration, output)) == 2
    assert [path.name for path in output.iterdir()] == ["operator-axes.json", "t0.json"]


def test_package_rejects_missing_confirmations_and_existing_output(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Removing either gate would overwrite a reviewed artifact or bypass onsite checks."""
    import kinova_teleop.public_hardware_setup as setup

    output = tmp_path / "local-config"
    output.mkdir()
    t0, calibration = output / "t0.json", output / "operator-axes.json"
    t0.write_text(json.dumps(_t0_payload()), encoding="utf-8")
    calibration.write_text(json.dumps(_calibration_payload()), encoding="utf-8")
    monkeypatch.setattr(setup, "_current_clean_code_revision", lambda: "a" * 40)
    monkeypatch.setattr(setup, "_now_utc", lambda: datetime(2026, 8, 13, tzinfo=timezone.utc))
    missing = _package_argv(t0, calibration, output)
    missing.remove("--load-tcp-checked")
    assert main(missing) == 2
    existing = output / "teleop-profile.json"
    existing.write_text('{"user":"artifact"}\n', encoding="utf-8")
    before = existing.read_bytes()
    assert main(_package_argv(t0, calibration, output)) == 2
    assert existing.read_bytes() == before


@pytest.mark.parametrize(
    ("failure_at", "error"),
    ((1, KeyboardInterrupt()), (2, SystemExit(17)), (3, KeyboardInterrupt()), (2, OSError("windows publish failure"))),
)
def test_package_publish_baseexception_never_leaves_created_final_files(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, failure_at: int, error: BaseException
) -> None:
    """Interrupting any publish boundary must roll back only this transaction's links."""
    import kinova_teleop.public_hardware_setup as setup

    output = tmp_path / "local-config"
    output.mkdir()
    t0, calibration = output / "t0.json", output / "operator-axes.json"
    t0.write_text(json.dumps(_t0_payload()), encoding="utf-8")
    calibration.write_text(json.dumps(_calibration_payload()), encoding="utf-8")
    monkeypatch.setattr(setup, "_current_clean_code_revision", lambda: "a" * 40)
    monkeypatch.setattr(setup, "_now_utc", lambda: datetime(2026, 8, 13, tzinfo=timezone.utc))
    real_link = setup.os.link
    calls = 0

    def interrupt_after_link(source, destination, *args, **kwargs):
        nonlocal calls
        calls += 1
        real_link(source, destination, *args, **kwargs)
        if calls == failure_at:
            raise error

    monkeypatch.setattr(setup.os, "link", interrupt_after_link)
    with pytest.raises(type(error)):
        setup._run_package(setup.build_parser().parse_args(_package_argv(t0, calibration, output)))
    assert sorted(path.name for path in output.iterdir()) == ["operator-axes.json", "t0.json"]
