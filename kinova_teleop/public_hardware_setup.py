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
import re
import secrets
import shutil
import stat
import sys
import tempfile
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
from .motion_lease import validate_motion_lease
from .operator_calibration import load_operator_axis_calibration
from .preflight import (
    _GEN3_PHYSICAL_KEYS,
    PreflightContext,
    build_reviewed_gen3_preflight_report,
    load_passing_preflight_report,
    decode_firmware_version,
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
_T0_SOFTWARE_CHECKS = {
    "code_revision": "present", "dirty_worktree": "clean worktree",
    "runtime": "present", "driver": "present", "firmware": "present",
    "transport": "present", "calibration": "calibration hashes present",
    "safety_limits": "workspace and speed limits present",
    "product_model": "MODEL_ID_L53", "degree_of_freedom": "7 DoF",
    "firmware_version": "2.8.0-5", "operating_mode": "RUN_MODE",
    "servoing_mode": "SINGLE_LEVEL_SERVOING", "feedback_pose": "finite tool pose",
}
_USER = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}\Z")


def _utc_now() -> str:
    return _now_utc().isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _now_utc() -> datetime:
    return datetime.now(timezone.utc)


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


def _t0_observation_digest(payload: Mapping[str, object]) -> str:
    fields = (
        "device", "timestamp_utc", "code_revision", "dirty_worktree", "runtime",
        "driver", "firmware", "transport", "safety_limits", "checks",
        "physical_checks", "passed", "reference_pose_m",
    )
    encoded = json.dumps(
        {field: payload[field] for field in fields},
        sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _validate_t0(
    payload: Mapping[str, object],
    expected_revision: str,
    *,
    now: datetime | None = None,
) -> tuple[str, str, tuple[float, float, float]]:
    if set(payload) != _T0_FIELDS or payload.get("schema_version") != "1.0" or payload.get("device") != "gen3":
        raise ValueError("T0 schema or device is invalid")
    if payload.get("dirty_worktree") is not False or payload.get("code_revision") != expected_revision:
        raise ValueError("T0 is not from the current clean code revision")
    if payload.get("passed") is not False:
        raise ValueError("T0 must remain a read-only non-passing observation")
    physical = payload.get("physical_checks")
    if not isinstance(physical, Mapping) or set(physical) != set(_GEN3_PHYSICAL_KEYS) or any(value is not False for value in physical.values()):
        raise ValueError("T0 physical checks must all remain explicitly false")
    timestamp = payload.get("timestamp_utc")
    if not isinstance(timestamp, str) or not timestamp.endswith("Z"):
        raise ValueError("T0 timestamp is invalid")
    try:
        observed = datetime.fromisoformat(timestamp[:-1] + "+00:00")
    except ValueError as error:
        raise ValueError("T0 timestamp is invalid") from error
    age = (now or _now_utc()) - observed
    if age > _MAX_T0_AGE or age < timedelta(minutes=-5):
        raise ValueError("T0 is stale or has a future timestamp")
    transport = payload.get("transport")
    if not isinstance(transport, Mapping) or set(transport) != {"kind", "host", "port", "robot_user"} or transport.get("kind") != "tcp" or transport.get("port") != 10000:
        raise ValueError("T0 transport is invalid")
    robot_ip = transport.get("host")
    if not isinstance(robot_ip, str):
        raise ValueError("T0 robot IP is invalid")
    validate_private_robot_ipv4(robot_ip)
    robot_user = transport.get("robot_user")
    if not isinstance(robot_user, str) or _USER.fullmatch(robot_user) is None:
        raise ValueError("T0 robot user is invalid")
    runtime = payload.get("runtime")
    if not isinstance(runtime, Mapping) or set(runtime) != {"python", "kortex_api", "protobuf"} or not isinstance(runtime.get("python"), str) or not runtime["python"].startswith("3.11.") or runtime.get("kortex_api") != "2.8.0.post5" or runtime.get("protobuf") != "3.20.0":
        raise ValueError("T0 runtime evidence is invalid")
    driver = payload.get("driver")
    if not isinstance(driver, Mapping) or dict(driver) != {"name": "kortex-api", "version": "2.8.0.post5"}:
        raise ValueError("T0 driver evidence is invalid")
    if payload.get("firmware") != {"version": "2.8.0-5"}:
        raise ValueError("T0 firmware evidence is invalid")
    if payload.get("safety_limits") != {"workspace": "read-only-t0", "max_linear_speed": 0.02}:
        raise ValueError("T0 safety evidence is invalid")
    calibration = payload.get("calibration")
    if not isinstance(calibration, list) or len(calibration) != 1 or not isinstance(calibration[0], Mapping) or set(calibration[0]) != {"name", "sha256"} or calibration[0].get("name") != "t0-observation" or not isinstance(calibration[0].get("sha256"), str) or calibration[0].get("sha256") != _t0_observation_digest(payload):
        raise ValueError("T0 observation hash is invalid or copied")
    reference = _finite_vector(payload.get("reference_pose_m"), "T0 feedback pose")
    checks = payload.get("checks")
    if not isinstance(checks, list):
        raise ValueError("T0 checks are invalid")
    expected_names = set(_T0_SOFTWARE_CHECKS) | {"arm_state"} | {f"physical.{name}" for name in _GEN3_PHYSICAL_KEYS}
    if len(checks) != len(expected_names):
        raise ValueError("T0 checks have missing or extra evidence")
    by_name: dict[str, Mapping[str, object]] = {}
    for item in checks:
        if not isinstance(item, Mapping) or set(item) != {"name", "status", "detail"} or not isinstance(item.get("name"), str) or item["name"] in by_name:
            raise ValueError("T0 checks are invalid")
        by_name[item["name"]] = item
    if set(by_name) != expected_names:
        raise ValueError("T0 checks have missing or extra evidence")
    for name, detail in _T0_SOFTWARE_CHECKS.items():
        if by_name[name].get("status") != "pass" or by_name[name].get("detail") != detail:
            raise ValueError(f"T0 is missing required {name} evidence")
    if by_name["arm_state"].get("status") != "pass" or by_name["arm_state"].get("detail") not in {"SERVOING_READY", "SERVOING_MANUALLY_CONTROLLED"}:
        raise ValueError("T0 arm state is not observed manual-control-ready")
    for name in _GEN3_PHYSICAL_KEYS:
        item = by_name[f"physical.{name}"]
        if item.get("status") != "fail" or item.get("detail") != "not confirmed":
            raise ValueError("T0 physical check evidence is invalid")
    return robot_ip, robot_user, reference


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
    robot_ip, robot_user, reference = _validate_t0(t0, revision, now=_now_utc())
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
        "robot_user": robot_user, "motion_lease": "motion-lease.json",
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
    transaction = Path(tempfile.mkdtemp(prefix=".gen3-package-", dir=output))
    staged_pairs = tuple(
        zip(
            (transaction / lease_path.name, transaction / report_path.name, transaction / profile_path.name),
            (lease_path, report_path, profile_path),
            strict=True,
        )
    )
    completed = False
    try:
        (staged_lease, _), (staged_report, _), (staged_profile, _) = staged_pairs
        _write_new_json(staged_lease, lease.to_mapping())
        _write_new_json(staged_report, report.to_mapping())
        _write_new_json(staged_profile, profile)
        validate_motion_lease(staged_lease, run_id, args.owner)
        load_passing_preflight_report(staged_report, expected_safety_limits=contract, expected_code_revision=revision, expected_calibration_sha256=calibration.source_sha256)
        if _strict_json(staged_profile) != profile:
            raise ValueError("generated profile self-validation failed")
        for staged, target in staged_pairs:
            os.link(staged, target)
        completed = True
    finally:
        if not completed:
            # A link can complete immediately before an interrupt reaches us.
            # Remove only a target that still has the staged file identity; an
            # independently-created user file is never unlinked.
            for staged, target in reversed(staged_pairs):
                try:
                    if target.exists() and os.path.samefile(staged, target):
                        target.unlink()
                except OSError:
                    pass
        shutil.rmtree(transaction, ignore_errors=True)
    print(f"generated guarded Gen3 package in {output}")
    return 0


def _run_t0(args: argparse.Namespace) -> int:
    """Execute only the existing read-only Kortex inspection boundary."""

    validate_private_robot_ipv4(args.robot_ip)
    if _USER.fullmatch(args.robot_user) is None:
        raise ValueError("robot user must be a non-empty path-safe identifier")
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
            runtime={"python": versions.python, "kortex_api": versions.kortex_api, "protobuf": versions.protobuf},
            driver={"name": "kortex-api", "version": versions.kortex_api},
            firmware={"version": "2.8.0-5"},
            transport={"kind": "tcp", "host": args.robot_ip, "port": 10000, "robot_user": args.robot_user},
            calibration=({"name": "t0-observation", "sha256": "0" * 64},),
            safety_limits={"workspace": "read-only-t0", "max_linear_speed": 0.02},
            physical_checks={name: False for name in _GEN3_PHYSICAL_KEYS},
        )
        report = run_kortex_readonly_preflight(connection, context)
        firmware = connection.device_config.GetFirmwareVersion(options=connection.readonly_rpc_options())
        raw_firmware = getattr(firmware, "firmware_version", None)
        observed_firmware = decode_firmware_version(raw_firmware) if isinstance(raw_firmware, int) else "not-observed"
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
    payload["runtime"] = {"python": versions.python, "kortex_api": versions.kortex_api, "protobuf": versions.protobuf}
    payload["driver"] = {"name": "kortex-api", "version": versions.kortex_api}
    payload["firmware"] = {"version": observed_firmware}
    payload["transport"] = {"kind": "tcp", "host": args.robot_ip, "port": 10000, "robot_user": args.robot_user}
    payload["safety_limits"] = {"workspace": "read-only-t0", "max_linear_speed": 0.02}
    payload["reference_pose_m"] = reference
    payload["calibration"] = [{"name": "t0-observation", "sha256": _t0_observation_digest(payload)}]
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
