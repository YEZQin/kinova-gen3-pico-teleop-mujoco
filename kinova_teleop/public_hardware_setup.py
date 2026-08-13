"""Offline-safe Gen3 T0 capture and robot-specific package generation."""

from __future__ import annotations

import argparse
from collections.abc import Mapping, Sequence
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import stat
import sys
import uuid
from typing import Any

from .hardware_profile import (
    CALIBRATED_RESPONSIVE_TRANSLATION_WORKSPACE_HALF_WIDTH_AXIS_M,
    EXPANDED_TRANSLATION_ONLY_ANCHOR_AXIS_M,
    FIRST_HARDWARE_PROFILE,
    validate_private_robot_ipv4,
)
from .main import _current_clean_code_revision
from .motion_lease import create_motion_lease
from .operator_calibration import load_operator_axis_calibration
from .preflight import (
    _GEN3_PHYSICAL_KEYS,
    PreflightContext,
    build_reviewed_gen3_preflight_report,
    load_passing_preflight_report,
    run_kortex_readonly_preflight,
)
from .workspace import WorkspaceLimits


_T0_FIELDS = frozenset({
    "schema_version", "device", "timestamp_utc", "code_revision", "dirty_worktree",
    "runtime", "driver", "firmware", "transport", "calibration", "safety_limits",
    "checks", "physical_checks", "passed", "reference_pose_m",
})
_REPARSE = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0)
_MAX_T0_AGE = timedelta(hours=24)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _strict_json(path: Path) -> dict[str, Any]:
    try:
        metadata = path.lstat()
    except OSError as error:
        raise ValueError("artifact file is unavailable") from error
    if path.is_symlink() or bool(getattr(metadata, "st_file_attributes", 0) & _REPARSE) or not path.is_file():
        raise ValueError("artifact must be a regular file")

    def reject_constant(value: str) -> None:
        raise ValueError(f"non-finite JSON constant is not allowed: {value}")

    def unique_pairs(pairs: list[tuple[str, object]]) -> dict[str, object]:
        result: dict[str, object] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("duplicate JSON key is not allowed")
            result[key] = value
        return result

    try:
        payload = json.loads(path.read_text(encoding="utf-8"), parse_constant=reject_constant, object_pairs_hook=unique_pairs)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
        raise ValueError("artifact JSON is invalid") from error
    if not isinstance(payload, dict):
        raise ValueError("artifact JSON must be an object")
    return payload


def _finite_vector(value: object, name: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{name} must be a three-value vector")
    if any(isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item) for item in value):
        raise ValueError(f"{name} must contain finite values")
    return tuple(float(item) for item in value)


def _validate_t0(payload: Mapping[str, object], expected_revision: str) -> tuple[str, tuple[float, float, float]]:
    if set(payload) != _T0_FIELDS or payload.get("schema_version") != "1.0" or payload.get("device") != "gen3":
        raise ValueError("T0 schema or device is invalid")
    if payload.get("dirty_worktree") is not False or payload.get("code_revision") != expected_revision:
        raise ValueError("T0 is not from the current clean code revision")
    timestamp = payload.get("timestamp_utc")
    if not isinstance(timestamp, str) or not timestamp.endswith("Z"):
        raise ValueError("T0 timestamp is invalid")
    try:
        observed = datetime.fromisoformat(timestamp[:-1] + "+00:00")
    except ValueError as error:
        raise ValueError("T0 timestamp is invalid") from error
    age = datetime.now(timezone.utc) - observed
    if age > _MAX_T0_AGE or age < timedelta(minutes=-5):
        raise ValueError("T0 is stale or has a future timestamp")
    transport = payload.get("transport")
    if not isinstance(transport, Mapping) or transport.get("kind") != "tcp" or transport.get("port") != 10000:
        raise ValueError("T0 transport is invalid")
    robot_ip = transport.get("host")
    if not isinstance(robot_ip, str):
        raise ValueError("T0 robot IP is invalid")
    validate_private_robot_ipv4(robot_ip)
    reference = _finite_vector(payload.get("reference_pose_m"), "T0 feedback pose")
    checks = payload.get("checks")
    if not isinstance(checks, list):
        raise ValueError("T0 checks are invalid")
    passed = {item.get("name"): item for item in checks if isinstance(item, Mapping) and item.get("status") == "pass"}
    expected = {
        "arm_state": "SERVOING_READY", "product_model": "MODEL_ID_L53",
        "degree_of_freedom": "7 DoF", "firmware_version": "2.8.0-5",
        "operating_mode": "RUN_MODE", "servoing_mode": "SINGLE_LEVEL_SERVOING",
        "feedback_pose": "finite tool pose",
    }
    for name, detail in expected.items():
        if not isinstance(passed.get(name), Mapping) or passed[name].get("detail") != detail:
            raise ValueError(f"T0 is missing required {name} evidence")
    return robot_ip, reference


def _validate_output_directory(path: Path) -> Path:
    if path.name != "local-config":
        raise ValueError("output directory must be named local-config")
    try:
        metadata = path.lstat()
    except OSError as error:
        raise ValueError("output directory must already exist") from error
    if path.is_symlink() or bool(getattr(metadata, "st_file_attributes", 0) & _REPARSE) or not path.is_dir():
        raise ValueError("output directory must be a regular local-config directory")
    return path.resolve()


def _write_new_json(path: Path, payload: Mapping[str, object]) -> None:
    if os.path.lexists(path):
        raise ValueError(f"refusing to overwrite output artifact: {path.name}")
    encoded = (json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode("utf-8")
    try:
        descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_BINARY", 0), 0o600)
        with os.fdopen(descriptor, "wb") as artifact:
            artifact.write(encoded)
            artifact.flush()
            os.fsync(artifact.fileno())
    except FileExistsError as error:
        raise ValueError(f"refusing to overwrite output artifact: {path.name}") from error


def _driver_sha256() -> str:
    return hashlib.sha256((Path(__file__).with_name("kortex_backend.py")).read_bytes()).hexdigest()


def _motion_contract(minimum: tuple[float, float, float], maximum: tuple[float, float, float], linear_speed: float, calibration_sha256: str) -> dict[str, object]:
    return {
        "workspace_min_m": list(minimum), "workspace_max_m": list(maximum),
        "max_linear_speed_m_s": linear_speed, "translation_scale": 0.8,
        "translation_only": True, "expanded_translation_envelope": True,
        "responsive_translation_profile": True, "operator_axis_calibration": True,
        "operator_calibration_sha256": calibration_sha256, "recover_stale_input": True,
        "stale_timeout_s": 0.2, "control_hz": 40.0,
        "max_angular_speed_deg_s": FIRST_HARDWARE_PROFILE.max_angular_speed_deg_s,
        "anchor_translation_axis_m": list(EXPANDED_TRANSLATION_ONLY_ANCHOR_AXIS_M),
        "anchor_rotation_deg": FIRST_HARDWARE_PROFILE.anchor_rotation_deg,
    }


def _validate_workspace(minimum: tuple[float, float, float], maximum: tuple[float, float, float], reference: tuple[float, float, float], linear_speed: float) -> WorkspaceLimits:
    if any(low >= high for low, high in zip(minimum, maximum, strict=True)):
        raise ValueError("workspace bounds must be ordered")
    if minimum[2] < 0.0:
        raise ValueError("workspace lower Z must not be below zero")
    if any(value < low or value > high for value, low, high in zip(reference, minimum, maximum, strict=True)):
        raise ValueError("workspace bounds must contain the measured T0 pose")
    if not math.isfinite(linear_speed) or not 0.0 < linear_speed <= 0.02:
        raise ValueError("linear speed must be in (0, 0.02]")
    limits = WorkspaceLimits(minimum, maximum)
    spans = tuple(high - low for low, high in zip(minimum, maximum, strict=True))
    caps = tuple(2.0 * value for value in CALIBRATED_RESPONSIVE_TRANSLATION_WORKSPACE_HALF_WIDTH_AXIS_M)
    if any(span > cap for span, cap in zip(spans, caps, strict=True)):
        raise ValueError("workspace span exceeds calibrated per-axis cap")
    return limits


def _run_package(args: argparse.Namespace) -> int:
    if args.gripper or args.orientation:
        raise ValueError("package generation permits translation-only motion without gripper or orientation")
    physical = {name: bool(getattr(args, name)) for name in _GEN3_PHYSICAL_KEYS}
    if not all(physical.values()):
        raise ValueError("every physical checklist flag must be explicitly confirmed")
    output = _validate_output_directory(args.output_dir)
    t0 = _strict_json(args.t0)
    revision = _current_clean_code_revision()
    robot_ip, reference = _validate_t0(t0, revision)
    if Path(args.calibration).parent.resolve() != output:
        raise ValueError("calibration must be a regular file directly under output local-config")
    calibration = load_operator_axis_calibration(args.calibration)
    minimum = _finite_vector(args.workspace_min, "workspace minimum")
    maximum = _finite_vector(args.workspace_max, "workspace maximum")
    _validate_workspace(minimum, maximum, reference, args.linear_speed)
    contract = _motion_contract(minimum, maximum, args.linear_speed, calibration.source_sha256)
    run_id = f"run-{uuid.uuid4().hex}"
    now = _utc_now()
    lease = create_motion_lease(run_id, args.owner, now, secrets.token_hex(32))
    report = build_reviewed_gen3_preflight_report(
        t0, code_revision=revision, calibration_sha256=calibration.source_sha256,
        driver_sha256=_driver_sha256(), safety_limits=contract,
        physical_checks=physical, timestamp_utc=now,
    )
    midpoint = [float((low + high) / 2.0) for low, high in zip(minimum, maximum, strict=True)]
    profile = {
        "schema_version": "1.0", "device": "gen3", "robot_ip": robot_ip,
        "robot_user": "admin", "motion_lease": "motion-lease.json",
        "preflight_report": "reviewed-preflight.json", "calibration": Path(args.calibration).name,
        "workspace_min_m": list(minimum), "workspace_max_m": list(maximum),
        "max_linear_speed_m_s": args.linear_speed, "run_id": run_id, "lease_owner": args.owner,
        "code_revision": revision, "calibration_sha256": calibration.source_sha256,
        "driver_sha256": _driver_sha256(), "reference_pose_m": list(reference),
        "workspace_midpoint_m": midpoint,
    }
    lease_path, report_path, profile_path = output / "motion-lease.json", output / "reviewed-preflight.json", output / "teleop-profile.json"
    for candidate in (lease_path, report_path, profile_path):
        if os.path.lexists(candidate):
            raise ValueError(f"refusing to overwrite output artifact: {candidate.name}")
    _write_new_json(lease_path, lease.to_mapping())
    try:
        _write_new_json(report_path, report.to_mapping())
        load_passing_preflight_report(report_path, expected_safety_limits=contract, expected_code_revision=revision, expected_calibration_sha256=calibration.source_sha256)
        _write_new_json(profile_path, profile)
    except Exception:
        # Generated artifacts are intentionally never overwritten or removed.
        raise
    print(f"generated guarded Gen3 package in {output}")
    return 0


def _run_t0(args: argparse.Namespace) -> int:
    """Execute only the existing read-only Kortex inspection boundary."""

    validate_private_robot_ipv4(args.robot_ip)
    output = Path(args.output)
    _validate_output_directory(output.parent)
    if os.path.lexists(output):
        raise ValueError("refusing to overwrite T0 output")
    password = os.environ.get("KINOVA_PASSWORD")
    if not password:
        raise ValueError("KINOVA_PASSWORD must be supplied only for the T0 child process")
    from .hardware_profile import validate_kortex_runtime
    from .kortex_transport import KortexConfig
    from .main import _close_resource, _create_kortex_connection

    versions = validate_kortex_runtime()
    connection = _create_kortex_connection(KortexConfig(args.robot_ip, args.robot_user, password), read_only=True)
    try:
        context = PreflightContext(
            code_revision=_current_clean_code_revision(),
            dirty_worktree=False,
            runtime={"python": versions.python},
            driver={"name": "kortex-api", "version": versions.kortex_api},
            firmware={"expected": "2.8.0-5"},
            transport={"kind": "tcp", "host": args.robot_ip, "port": 10000},
            calibration=({"name": "unbound-at-t0", "sha256": _driver_sha256()},),
            safety_limits={"workspace": "t0-read-only", "max_linear_speed": 0.02},
            physical_checks={name: False for name in _GEN3_PHYSICAL_KEYS},
        )
        report = run_kortex_readonly_preflight(connection, context)
        feedback = connection.base_cyclic.RefreshFeedback(options=connection.readonly_rpc_options())
        base = getattr(feedback, "base", feedback)
        reference = [float(base.tool_pose_x), float(base.tool_pose_y), float(base.tool_pose_z)]
        _finite_vector(reference, "T0 feedback pose")
    finally:
        if not _close_resource(connection, hardware=False, send_stop=False):
            raise RuntimeError("read-only Kortex cleanup failed")
    payload = report.to_mapping()
    payload["code_revision"] = _current_clean_code_revision()
    payload["dirty_worktree"] = False
    payload["reference_pose_m"] = reference
    _write_new_json(output, payload)
    print(f"read-only T0 saved: {output}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate local, guarded Gen3 hardware artifacts.")
    phases = parser.add_subparsers(dest="phase", required=True)
    t0 = phases.add_parser("t0", help="run the read-only Gen3 T0 observation")
    t0.add_argument("--robot-ip", required=True)
    t0.add_argument("--robot-user", default="admin")
    t0.add_argument("--output", type=Path, required=True)
    package = phases.add_parser("package", help="offline build of a robot-specific motion package")
    package.add_argument("--t0", type=Path, required=True)
    package.add_argument("--calibration", type=Path, required=True)
    package.add_argument("--workspace-min", nargs=3, type=float, required=True)
    package.add_argument("--workspace-max", nargs=3, type=float, required=True)
    package.add_argument("--linear-speed", type=float, required=True)
    package.add_argument("--owner", required=True)
    package.add_argument("--output-dir", type=Path, required=True)
    package.add_argument("--gripper", action="store_true")
    package.add_argument("--orientation", action="store_true")
    for name in _GEN3_PHYSICAL_KEYS:
        package.add_argument(f"--{name.replace('_', '-')}", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return _run_t0(args) if args.phase == "t0" else _run_package(args)
    except (OSError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
