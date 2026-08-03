"""Read-only input and Kortex Gen3 preflight reports.

This module deliberately has no Kortex SDK imports.  A connected object is
duck-typed at the RPC boundary so every check can run against a fake object in
an offline test environment.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
import math
from types import MappingProxyType
from typing import Any, Literal

import numpy as np

from .xr_input import XrInputSource


CheckStatus = Literal["pass", "fail", "unknown"]
_GEN3_PHYSICAL_KEYS = (
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


def _freeze(value: Any) -> Any:
    if isinstance(value, Mapping):
        return MappingProxyType({str(key): _freeze(item) for key, item in value.items()})
    if isinstance(value, (tuple, list)):
        return tuple(_freeze(item) for item in value)
    return value


def _plain(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_plain(item) for item in value]
    return value


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _unknown(value: object) -> bool:
    if value is None or value == "" or value == {} or value == ():
        return True
    if isinstance(value, str) and value.strip().lower() in {"unknown", "n/a", "not available"}:
        return True
    if isinstance(value, Mapping):
        return any(_unknown(item) for item in value.values())
    if isinstance(value, (tuple, list)):
        return any(_unknown(item) for item in value)
    return False


@dataclass(frozen=True, slots=True)
class PreflightCheck:
    name: str
    status: CheckStatus
    detail: str

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name:
            raise ValueError("preflight check name must be non-empty")
        if self.status not in ("pass", "fail", "unknown"):
            raise ValueError("preflight check status is invalid")
        if not isinstance(self.detail, str) or not self.detail:
            raise ValueError("preflight check detail must be non-empty")

    def to_mapping(self) -> dict[str, object]:
        return {"name": self.name, "status": self.status, "detail": self.detail}


@dataclass(frozen=True, slots=True)
class PreflightContext:
    """Build metadata and human-confirmed physical checks for one report."""

    code_revision: str | None = None
    dirty_worktree: bool | None = None
    runtime: Mapping[str, object] = ()
    driver: Mapping[str, object] = ()
    firmware: Mapping[str, object] = ()
    transport: Mapping[str, object] = ()
    calibration: Sequence[Mapping[str, object]] = ()
    safety_limits: Mapping[str, object] = ()
    physical_checks: Mapping[str, bool] = ()
    timestamp_utc: str | None = None
    device: Literal["gen3"] = "gen3"

    def __post_init__(self) -> None:
        if self.device != "gen3":
            raise ValueError("Gen3 preflight context device must be gen3")
        for name in ("runtime", "driver", "firmware", "transport", "safety_limits", "physical_checks"):
            value = getattr(self, name)
            if not isinstance(value, Mapping):
                value = {}
            object.__setattr__(self, name, _freeze(value))
        object.__setattr__(self, "calibration", _freeze(tuple(self.calibration)))
        if self.timestamp_utc is None:
            object.__setattr__(self, "timestamp_utc", _utc_now())


@dataclass(frozen=True, slots=True)
class PreflightReport:
    schema_version: str
    device: Literal["gen3"]
    timestamp_utc: str
    code_revision: str
    dirty_worktree: bool
    runtime: Mapping[str, object]
    driver: Mapping[str, object]
    firmware: Mapping[str, object]
    transport: Mapping[str, object]
    calibration: tuple[Mapping[str, object], ...]
    safety_limits: Mapping[str, object]
    physical_checks: Mapping[str, bool]
    checks: tuple[PreflightCheck, ...]
    passed: bool

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "device": self.device,
            "timestamp_utc": self.timestamp_utc,
            "code_revision": self.code_revision,
            "dirty_worktree": self.dirty_worktree,
            "runtime": _plain(self.runtime),
            "driver": _plain(self.driver),
            "firmware": _plain(self.firmware),
            "transport": _plain(self.transport),
            "calibration": _plain(self.calibration),
            "safety_limits": _plain(self.safety_limits),
            "checks": [check.to_mapping() for check in self.checks],
            "physical_checks": _plain(self.physical_checks),
            "passed": self.passed,
        }


def _context_checks(context: PreflightContext) -> list[PreflightCheck]:
    checks: list[PreflightCheck] = []

    def metadata(name: str, value: object) -> None:
        if _unknown(value):
            checks.append(PreflightCheck(name, "unknown", f"{name} is missing"))
        else:
            checks.append(PreflightCheck(name, "pass", "present"))

    metadata("code_revision", context.code_revision)
    if isinstance(context.dirty_worktree, bool):
        checks.append(
            PreflightCheck(
                "dirty_worktree",
                "pass" if not context.dirty_worktree else "fail",
                "clean worktree" if not context.dirty_worktree else "worktree is dirty",
            )
        )
    else:
        checks.append(PreflightCheck("dirty_worktree", "unknown", "dirty state is missing"))
    for name, value in (
        ("runtime", context.runtime),
        ("driver", context.driver),
        ("firmware", context.firmware),
        ("transport", context.transport),
    ):
        metadata(name, value)
    if not context.calibration:
        checks.append(PreflightCheck("calibration", "unknown", "calibration hashes are missing"))
    else:
        valid = all(
            isinstance(item, Mapping)
            and bool(item.get("name"))
            and bool(item.get("sha256"))
            and not _unknown(item.get("sha256"))
            for item in context.calibration
        )
        checks.append(
            PreflightCheck(
                "calibration",
                "pass" if valid else "unknown",
                "calibration hashes present" if valid else "calibration hashes are incomplete",
            )
        )
    limits = context.safety_limits
    if not limits:
        checks.append(PreflightCheck("safety_limits", "unknown", "workspace and speed limits are missing"))
    else:
        has_workspace = any("workspace" in str(key).lower() for key in limits)
        has_speed = any("speed" in str(key).lower() for key in limits)
        status: CheckStatus = "pass" if has_workspace and has_speed else "unknown"
        detail = "workspace and speed limits present" if status == "pass" else "workspace or speed limits are missing"
        checks.append(PreflightCheck("safety_limits", status, detail))
    for name in _GEN3_PHYSICAL_KEYS:
        value = context.physical_checks.get(name)
        if isinstance(value, bool):
            checks.append(
                PreflightCheck(
                    f"physical.{name}",
                    "pass" if value else "fail",
                    "confirmed" if value else "not confirmed",
                )
            )
        else:
            checks.append(PreflightCheck(f"physical.{name}", "unknown", "physical check is missing"))
    return checks


def _make_report(context: PreflightContext, checks: Sequence[PreflightCheck]) -> PreflightReport:
    checks_tuple = tuple(checks)
    physical = {
        name: bool(context.physical_checks.get(name))
        for name in _GEN3_PHYSICAL_KEYS
    }
    passed = bool(
        checks_tuple
        and all(check.status == "pass" for check in checks_tuple)
        and all(physical.values())
    )
    return PreflightReport(
        schema_version="1.0",
        device="gen3",
        timestamp_utc=str(context.timestamp_utc),
        code_revision=context.code_revision or "unknown",
        dirty_worktree=bool(context.dirty_worktree) if isinstance(context.dirty_worktree, bool) else False,
        runtime=context.runtime,
        driver=context.driver,
        firmware=context.firmware,
        transport=context.transport,
        calibration=tuple(context.calibration),
        safety_limits=context.safety_limits,
        physical_checks=MappingProxyType(physical),
        checks=checks_tuple,
        passed=passed,
    )


def _feedback_values(feedback: Any) -> tuple[np.ndarray, np.ndarray]:
    base = getattr(feedback, "base", feedback)
    position = np.asarray(
        [base.tool_pose_x, base.tool_pose_y, base.tool_pose_z], dtype=float
    )
    angles = np.asarray(
        [base.tool_pose_theta_x, base.tool_pose_theta_y, base.tool_pose_theta_z], dtype=float
    )
    return position, angles


def run_kortex_readonly_preflight(
    connection: Any,
    context: PreflightContext,
    *,
    monotonic: Callable[[], float] | None = None,
) -> PreflightReport:
    """Read arm state and one feedback frame; never clear faults or servo."""

    checks = _context_checks(context)
    try:
        state_response = connection.base.GetArmState(options=connection.rpc_options())
        active_state = getattr(state_response, "active_state", None)
        ready_state = getattr(connection.base_pb2, "ARMSTATE_SERVOING_READY", None)
        if active_state == ready_state or active_state == "ARMSTATE_SERVOING_READY":
            checks.append(PreflightCheck("arm_state", "pass", "SERVOING_READY"))
        elif ready_state is None:
            checks.append(PreflightCheck("arm_state", "unknown", "ready state constant is missing"))
        else:
            checks.append(PreflightCheck("arm_state", "fail", f"arm state is {active_state!r}"))
        feedback = connection.base_cyclic.RefreshFeedback(options=connection.rpc_options())
        position, angles = _feedback_values(feedback)
        if np.isfinite(position).all() and np.isfinite(angles).all():
            checks.append(PreflightCheck("feedback_pose", "pass", "finite tool pose"))
        else:
            checks.append(PreflightCheck("feedback_pose", "fail", "tool pose is non-finite"))
    except Exception as error:
        # Do not expose arbitrary SDK text: it may contain credentials or host details.
        checks.append(PreflightCheck("kortex_rpc", "fail", f"read-only RPC failed: {type(error).__name__}"))
    return _make_report(context, checks)


def run_input_preflight(
    source: XrInputSource,
    context: PreflightContext,
    *,
    samples: int = 10,
    monotonic: Callable[[], float] = lambda: 0.0,
) -> PreflightReport:
    """Verify finite, advancing, releasable input without starting motion."""

    if isinstance(samples, bool) or samples <= 0:
        raise ValueError("samples must be positive")
    checks = _context_checks(context)
    valid_count = 0
    release_seen = False
    previous_timestamp: int | None = None
    failures: list[str] = []
    for _ in range(samples):
        try:
            sample = source.read()
        except Exception as error:
            failures.append(type(error).__name__)
            continue
        try:
            position = np.asarray(sample.position, dtype=float)
            quaternion = np.asarray(sample.quaternion_xyzw, dtype=float)
            timestamp = int(sample.timestamp_ns)
            grip = float(sample.grip)
            valid = bool(sample.valid)
        except (AttributeError, TypeError, ValueError, OverflowError):
            failures.append("malformed sample")
            continue
        if not valid or position.shape != (3,) or quaternion.shape != (4,):
            failures.append("invalid sample")
            continue
        if not np.isfinite(position).all() or not np.isfinite(quaternion).all() or not math.isfinite(grip):
            failures.append("non-finite sample")
            continue
        if previous_timestamp is not None and timestamp <= previous_timestamp:
            failures.append("timestamps did not advance")
            continue
        previous_timestamp = timestamp
        valid_count += 1
        release_seen = release_seen or grip < 0.8
    checks.append(
        PreflightCheck(
            "input_samples",
            "pass" if valid_count == samples and not failures else "fail",
            f"{valid_count}/{samples} fresh finite samples" if not failures else "; ".join(failures),
        )
    )
    checks.append(
        PreflightCheck(
            "input_release",
            "pass" if release_seen else "fail",
            "Grip release observed" if release_seen else "no Grip release observed",
        )
    )
    return _make_report(context, checks)


def build_schema_compatible_report(
    context: PreflightContext,
    checks: Sequence[PreflightCheck],
) -> PreflightReport:
    """Public construction seam for adapters that add pure checks."""

    return _make_report(context, checks)
