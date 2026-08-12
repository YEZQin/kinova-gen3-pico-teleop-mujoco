"""Command-line entry point for Kinova Gen3 PICO teleoperation."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any

from .pico_udp_input import PicoUdpInput
from .evidence_log import EvidenceLogger, logger_event_sink
from .fixed_trajectory import FixedTrajectoryRunner, load_trajectory
from .hardware_profile import (
    FIRST_HARDWARE_PROFILE,
    validate_kortex_runtime,
    validate_private_robot_ipv4,
    validate_workspace_span,
)
from .hardware_admission import (
    confirm_move,
    verify_released_now,
    wait_for_fresh_released_input,
)
from .motion_lease import validate_motion_lease
from .preflight import (
    PreflightContext,
    load_passing_preflight_report,
    require_live_kortex_ready,
    run_kortex_readonly_preflight,
)
from .teleop_controller import StepDiagnostics, TeleopConfig, TeleopController
from .workspace import AnchorEnvelope, WorkspaceLimits
from .xr_input import (
    ContinuousInputBuffer,
    DryRunXrInput,
    SdkXrInput,
    XrInputSource,
)


DEFAULT_MODEL = (
    Path(__file__).resolve().parents[1]
    / "kinova_gen3_mujoco"
    / "teleop_scene.xml"
)

DEFAULT_MUJOCO_CONTROL_HZ = 100.0
DEFAULT_KORTEX_ROBOT_IP = "192.168.1.10"
DEFAULT_KORTEX_ROBOT_USER = "admin"
DEFAULT_MUJOCO_TRANSLATION_SCALE = 0.5
DEFAULT_MUJOCO_STALE_TIMEOUT = 0.2
DEFAULT_MOTION_RUN_ID = "gen3-first-hardware"
DEFAULT_MOTION_OWNER = "kinova-teleop"


class _BackendDefaultsParser(argparse.ArgumentParser):
    """Resolve legacy MuJoCo presentation defaults after backend selection."""

    def parse_known_args(
        self,
        args: Sequence[str] | None = None,
        namespace: argparse.Namespace | None = None,
    ) -> tuple[argparse.Namespace, list[str]]:
        parsed, extras = super().parse_known_args(args, namespace)
        if parsed.backend == "mujoco" and parsed.scale is None:
            parsed = argparse.Namespace(
                **(
                    vars(parsed)
                    | {"scale": DEFAULT_MUJOCO_TRANSLATION_SCALE}
                ),
            )
        return parsed, extras


def _create_kortex_connection(config: Any, *, read_only: bool = False) -> Any:
    """Construct the optional Kortex transport after the hardware gate passes."""

    from .kortex_transport import KortexConnection

    return KortexConnection(config).connect(send_stop_on_failure=not read_only)


def _create_kortex_backend(connection: Any, **kwargs: Any) -> Any:
    """Construct the optional Kortex backend after the hardware gate passes."""

    from .kortex_backend import KortexBackend

    return KortexBackend(connection, **kwargs)


def build_parser() -> argparse.ArgumentParser:
    parser = _BackendDefaultsParser(
        description=(
            "Use the PICO left controller to command a Kinova Gen3 arm in "
            "MuJoCo or, with explicit hardware gating, over Kortex."
        ),
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=DEFAULT_MODEL,
        help="MuJoCo scene path (default: kinova_gen3_mujoco/teleop_scene.xml)",
    )
    parser.add_argument(
        "--controller",
        choices=("left",),
        default="left",
        help="XR controller to use; this project intentionally supports left only",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=None,
        help="controller-to-robot translation scale (backend-specific default)",
    )
    parser.add_argument(
        "--translation-only",
        action="store_true",
        help="map controller translation while holding the anchored tool orientation",
    )
    parser.add_argument(
        "--control-hz",
        type=float,
        default=None,
        help=(
            "teleoperation update rate "
            f"(default: {DEFAULT_MUJOCO_CONTROL_HZ:g} for MuJoCo, "
            f"{FIRST_HARDWARE_PROFILE.control_hz:g} for Kortex; "
            f"Kortex maximum: {FIRST_HARDWARE_PROFILE.control_hz:g})"
        ),
    )
    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=None,
        help="seconds before unchanged XR timestamps release the clutch (backend-specific default)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="use deterministic synthetic 6DoF controller input",
    )
    parser.add_argument(
        "--input",
        choices=("pico-udp", "xrobotoolkit", "none"),
        default="pico-udp",
        help="controller input source (default: pico-udp; none for fixed trajectory)",
    )
    parser.add_argument(
        "--pico-host",
        default="0.0.0.0",
        help="PICO UDP bind host (default: 0.0.0.0 for network discovery)",
    )
    parser.add_argument(
        "--pico-port",
        type=int,
        default=15031,
        help="PICO UDP bind port (default: 15031)",
    )
    parser.add_argument(
        "--check-input",
        action="store_true",
        help="preflight the selected input without loading MuJoCo",
    )
    parser.add_argument(
        "--check-xr",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--check-kortex",
        action="store_true",
        help="run read-only Kortex state/feedback preflight and exit",
    )
    parser.add_argument(
        "--preflight-json",
        type=Path,
        help="write the read-only preflight report to this JSON path",
    )
    parser.add_argument(
        "--preflight-report",
        type=Path,
        help="supervisor-reviewed passing preflight JSON required for Kortex motion",
    )
    parser.add_argument(
        "--check-timeout",
        type=float,
        default=15.0,
        help="seconds to wait for each input preflight (default: 15)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="run without the MuJoCo Viewer",
    )
    parser.add_argument(
        "--steps",
        type=int,
        help="stop simulation after this many control steps",
    )
    parser.add_argument(
        "--samples",
        type=int,
        help="stop --check-input after this many samples",
    )
    parser.add_argument(
        "--backend",
        choices=("mujoco", "kortex"),
        default="mujoco",
        help="teleoperation backend (default: mujoco)",
    )
    parser.add_argument(
        "--enable-hardware",
        action="store_true",
        help="allow an explicitly selected Kortex hardware backend",
    )
    parser.add_argument(
        "--robot-ip",
        default=DEFAULT_KORTEX_ROBOT_IP,
        help=f"Kortex robot address (default: {DEFAULT_KORTEX_ROBOT_IP})",
    )
    parser.add_argument(
        "--robot-user",
        default=DEFAULT_KORTEX_ROBOT_USER,
        help=f"Kortex username (default: {DEFAULT_KORTEX_ROBOT_USER})",
    )
    parser.add_argument(
        "--max-linear-speed",
        type=float,
        default=None,
        help="Kortex linear-speed limit in m/s (maximum: 0.005)",
    )
    parser.add_argument(
        "--max-angular-speed-deg",
        type=float,
        default=None,
        help="Kortex angular-speed limit in deg/s (maximum: 2)",
    )
    parser.add_argument(
        "--gripper",
        action="store_true",
        help="deprecated first-hardware option; always rejected",
    )
    parser.add_argument("--workspace-min", nargs=3, type=float, metavar=("X", "Y", "Z"), help="explicit workspace minimum XYZ (m)")
    parser.add_argument("--workspace-max", nargs=3, type=float, metavar=("X", "Y", "Z"), help="explicit workspace maximum XYZ (m)")
    parser.add_argument("--motion-lease", type=Path, help="supervisor-issued motion lease JSON")
    parser.add_argument("--run-id", default=DEFAULT_MOTION_RUN_ID, help="run id expected in the motion lease")
    parser.add_argument("--lease-owner", default=DEFAULT_MOTION_OWNER, help="owner expected in the motion lease")
    parser.add_argument("--fixed-trajectory", type=Path, help="validated relative trajectory JSON")
    parser.add_argument("--evidence-jsonl", type=Path, help="append-only trial event JSONL path")
    return parser


def resolve_control_hz(args: argparse.Namespace) -> float:
    if args.control_hz is not None:
        return float(args.control_hz)
    if args.backend == "kortex":
        return FIRST_HARDWARE_PROFILE.control_hz
    return DEFAULT_MUJOCO_CONTROL_HZ


def resolve_translation_scale(args: argparse.Namespace) -> float:
    """Resolve the backend-specific controller-to-robot translation scale."""

    if args.scale is not None:
        return float(args.scale)
    if args.backend == "kortex":
        return FIRST_HARDWARE_PROFILE.translation_scale
    return DEFAULT_MUJOCO_TRANSLATION_SCALE


def _resolve_stale_timeout(args: argparse.Namespace) -> float:
    if args.stale_timeout is not None:
        return float(args.stale_timeout)
    if args.backend == "kortex":
        return FIRST_HARDWARE_PROFILE.stale_timeout_s
    return DEFAULT_MUJOCO_STALE_TIMEOUT


def _resolve_max_linear_speed(args: argparse.Namespace) -> float:
    if args.max_linear_speed is not None:
        return float(args.max_linear_speed)
    return FIRST_HARDWARE_PROFILE.max_linear_speed_mps


def _resolve_max_angular_speed_deg(args: argparse.Namespace) -> float:
    if args.max_angular_speed_deg is not None:
        return float(args.max_angular_speed_deg)
    return FIRST_HARDWARE_PROFILE.max_angular_speed_deg_s


def create_input(args: argparse.Namespace) -> XrInputSource:
    """Construct the selected input without requiring a fixed PICO address."""
    control_hz = resolve_control_hz(args)
    if args.input == "none":
        raise RuntimeError("input source is disabled for fixed trajectory mode")
    if args.dry_run:
        return DryRunXrInput(control_hz=control_hz)
    if args.input == "pico-udp":
        return ContinuousInputBuffer(
            PicoUdpInput(
                host=args.pico_host,
                port=args.pico_port,
                stale_after=_resolve_stale_timeout(args),
            ),
        )
    return SdkXrInput()


def check_input(
    source: XrInputSource,
    samples: int | None,
    check_timeout: float,
) -> int:
    """Verify that the selected input supplies fresh valid, releasable poses."""
    read_count = 0
    last_timestamp: int | None = None
    saw_release = False
    deadline = time.monotonic() + check_timeout
    try:
        while samples is None or read_count < samples:
            if samples is not None and time.monotonic() >= deadline:
                print(
                    "error: timed out waiting for "
                    f"{samples} distinct valid input samples; "
                    "verify the selected input source is connected and "
                    "delivering tracked controller data",
                    file=sys.stderr,
                )
                return 2
            sample = source.read()
            if not sample.valid:
                time.sleep(0.01)
                continue
            if last_timestamp is not None and sample.timestamp_ns <= last_timestamp:
                time.sleep(0.01)
                continue
            values = " ".join(f"{value:+.5f}" for value in sample.position)
            quaternion = " ".join(
                f"{value:+.5f}" for value in sample.quaternion_xyzw
            )
            print(
                f"left position=[{values}] quat_xyzw=[{quaternion}] "
                f"grip={sample.grip:.3f} trigger={sample.trigger:.3f} "
                f"timestamp_ns={sample.timestamp_ns}",
                flush=True,
            )
            last_timestamp = sample.timestamp_ns
            read_count += 1
            saw_release = saw_release or sample.grip < 0.8
            if samples is None:
                time.sleep(0.01)
        if not saw_release:
            print(
                "error: no Grip release observed; release the Grip below 0.8 "
                "before starting teleoperation",
                file=sys.stderr,
            )
            return 2
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"error: input check failed: {exc}", file=sys.stderr)
        return 2
    finally:
        source.close()
    return 0


def _validate_args(args: argparse.Namespace) -> str | None:
    # This is intentionally first: the bare-arm profile never permits a
    # gripper flag, regardless of backend or any other malformed option.
    if args.gripper:
        return "--gripper is disabled for the first-hardware profile"
    if args.control_hz is not None and (
        not math.isfinite(args.control_hz) or args.control_hz <= 0.0
    ):
        return "--control-hz must be positive"
    if not math.isfinite(resolve_translation_scale(args)) or resolve_translation_scale(args) <= 0.0:
        return "--scale must be positive"
    if not math.isfinite(_resolve_stale_timeout(args)) or _resolve_stale_timeout(args) <= 0.0:
        return "--stale-timeout must be positive"
    if not 1 <= args.pico_port <= 65535:
        return "--pico-port must be between 1 and 65535"
    if not math.isfinite(args.check_timeout) or args.check_timeout <= 0.0:
        return "--check-timeout must be positive"
    if args.backend != "kortex" and args.headless and (
        args.steps is None or args.steps <= 0
    ):
        return "--headless requires a positive --steps"
    if args.steps is not None and args.steps <= 0:
        return "--steps must be positive"
    if args.samples is not None and args.samples <= 0:
        return "--samples must be positive"
    if args.check_input and args.samples is None:
        return "--check-input requires a positive --samples"
    if args.samples is not None and not (args.check_input or args.check_xr):
        return "--samples is only valid with --check-input"
    if args.check_kortex and args.backend != "kortex":
        return "--check-kortex requires --backend kortex"
    if args.preflight_json is not None and not args.check_kortex:
        return "--preflight-json requires --check-kortex"
    if args.preflight_report is not None and args.check_kortex:
        return "--preflight-report cannot be combined with --check-kortex"
    if args.preflight_report is not None and args.backend != "kortex":
        return "--preflight-report requires --backend kortex"
    if args.check_kortex and args.fixed_trajectory:
        return "--check-kortex cannot be combined with --fixed-trajectory"
    if args.fixed_trajectory and args.input != "none":
        return "--fixed-trajectory requires --input none"
    if args.fixed_trajectory and not args.backend == "kortex":
        return "--fixed-trajectory requires --backend kortex"
    return None


def _workspace_from_args(args: argparse.Namespace) -> WorkspaceLimits | None:
    if args.workspace_min is None or args.workspace_max is None:
        return None
    return WorkspaceLimits(tuple(args.workspace_min), tuple(args.workspace_max))


def _validate_kortex_args(
    args: argparse.Namespace,
    password: str | None,
    *,
    check_password: bool = True,
) -> str | None:
    """Validate hardware gates in their fail-closed order before SDK import."""

    if not args.enable_hardware:
        return "--backend kortex requires --enable-hardware"
    if args.dry_run:
        return "--dry-run cannot be used with --backend kortex"
    if args.check_kortex and args.fixed_trajectory:
        return "--check-kortex cannot be combined with --fixed-trajectory"
    if args.fixed_trajectory and args.input != "none":
        return "--fixed-trajectory requires --input none"
    if _resolve_stale_timeout(args) > FIRST_HARDWARE_PROFILE.stale_timeout_s:
        return "--stale-timeout must not exceed 0.2 for --backend kortex"
    if not args.robot_ip.strip():
        return "--robot-ip must not be empty"
    if not args.robot_user.strip():
        return "--robot-user must not be empty"
    try:
        validate_private_robot_ipv4(args.robot_ip)
    except ValueError as error:
        return str(error)
    if resolve_translation_scale(args) > FIRST_HARDWARE_PROFILE.translation_scale:
        return "--scale must not exceed 0.25 for --backend kortex"
    if resolve_control_hz(args) > FIRST_HARDWARE_PROFILE.control_hz:
        return (
            f"--control-hz must not exceed {FIRST_HARDWARE_PROFILE.control_hz:g} "
            "for --backend kortex"
        )
    max_linear_speed = _resolve_max_linear_speed(args)
    if not math.isfinite(max_linear_speed) or not (
        0.0 < max_linear_speed <= FIRST_HARDWARE_PROFILE.max_linear_speed_mps
    ):
        return "--max-linear-speed must be in (0, 0.005]"
    max_angular_speed_deg = _resolve_max_angular_speed_deg(args)
    if not math.isfinite(max_angular_speed_deg) or not (
        0.0 < max_angular_speed_deg <= FIRST_HARDWARE_PROFILE.max_angular_speed_deg_s
    ):
        return "--max-angular-speed-deg must be in (0, 2]"

    # Read-only inspection does not authorize motion, so workspace/lease gates
    # are intentionally scoped to the motion paths only.
    if not args.check_kortex:
        try:
            limits = _workspace_from_args(args)
        except ValueError as error:
            return str(error)
        if limits is None:
            return "--workspace-min and --workspace-max are required for Kortex motion"
        try:
            validate_workspace_span(limits)
        except ValueError as error:
            return str(error)
        if args.motion_lease is None:
            return "--motion-lease is required for Kortex motion"
        try:
            validate_motion_lease(args.motion_lease, args.run_id, args.lease_owner)
        except (OSError, ValueError) as error:
            return f"invalid motion lease: {error}"
        if args.preflight_report is None:
            return "--preflight-report is required for Kortex motion"
        try:
            load_passing_preflight_report(args.preflight_report)
        except (OSError, ValueError) as error:
            return f"invalid preflight report: {error}"
        if args.fixed_trajectory is not None:
            try:
                load_trajectory(args.fixed_trajectory)
            except (OSError, ValueError) as error:
                return f"invalid fixed trajectory: {error}"
    if check_password and not password:
        return "KINOVA_PASSWORD must be set for --backend kortex"
    return None


def _print_status(diagnostics: StepDiagnostics) -> None:
    """Print a status transition supplied by ``TeleopController.run``."""
    fields = [
        f"clutch={diagnostics.clutch_state.value}",
        f"stale={str(diagnostics.stale).lower()}",
        f"ik_converged={str(diagnostics.ik_converged).lower()}",
    ]
    if diagnostics.reason:
        fields.append(f"reason={diagnostics.reason}")
    print(f"status: {' '.join(fields)}", flush=True)


def _approve_fixed_segment(segment: Any, index: int) -> bool:
    """Require a fresh exact MOVE confirmation before each fixed segment."""

    try:
        confirmation = input(
            f"Type MOVE to authorize fixed segment {index + 1} "
            f"({segment.name}): "
        )
    except KeyboardInterrupt:
        raise
    except Exception:
        return False
    return confirmation == "MOVE"


def _preflight_context(
    args: argparse.Namespace,
    versions: Any | None = None,
) -> PreflightContext:
    limits = _workspace_from_args(args)
    limit_mapping: dict[str, object] = {
        "max_linear_speed": _resolve_max_linear_speed(args),
        "max_angular_speed_deg": _resolve_max_angular_speed_deg(args),
    }
    if limits is not None:
        limit_mapping["workspace_min"] = list(limits.minimum_xyz)
        limit_mapping["workspace_max"] = list(limits.maximum_xyz)
    return PreflightContext(
        code_revision="working-tree",
        dirty_worktree=False,
        runtime={"python": getattr(versions, "python", sys.version.split()[0])},
        driver={"name": "kortex", "version": getattr(versions, "kortex_api", "unknown")},
        firmware={"version": "read-only-unverified"},
        transport={"kind": "tcp", "host": args.robot_ip, "port": 10000},
        calibration=({"name": "reported-by-kortex", "sha256": "read-only-unverified"},),
        safety_limits=limit_mapping,
        # Physical checks are an onsite operator concern; read-only inspection
        # must remain failed-closed until an operator records every check.
        physical_checks={
            "workspace_clear": False,
            "physical_estop_reachable": False,
            "teach_pendant_stop_reachable": False,
            "second_observer_present": False,
            "cable_slack_checked": False,
            "device_fixture_checked": False,
            "speed_level_checked": False,
            "workspace_bounds_checked": False,
            "load_tcp_checked": False,
        },
    )


def _run_kortex_readonly_check(args: argparse.Namespace, password: str) -> int:
    """Connect only after exact CONNECT and perform no motion setup."""

    try:
        confirmation = input("Type CONNECT to run read-only Kortex preflight: ")
    except KeyboardInterrupt:
        print("stopped by user", file=sys.stderr)
        return 130
    except Exception:
        print("error: Kortex confirmation failed", file=sys.stderr)
        return 2
    if confirmation != "CONNECT":
        print("error: read-only Kortex preflight was not confirmed", file=sys.stderr)
        return 2
    connection = None
    exit_code = 2
    try:
        from .kortex_transport import KortexConfig

        connection = _create_kortex_connection(
            KortexConfig(args.robot_ip, args.robot_user, password),
            read_only=True,
        )
        report = run_kortex_readonly_preflight(connection, _preflight_context(args))
        if args.preflight_json is not None:
            args.preflight_json.parent.mkdir(parents=True, exist_ok=True)
            args.preflight_json.write_text(
                json.dumps(report.to_mapping(), ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
        # A read-only RPC probe can succeed while the physical checklist is
        # intentionally unconfirmed; only a motion mode may gate on `passed`.
        exit_code = 0
    except Exception as error:
        print(f"error: Kortex read-only preflight failed ({type(error).__name__})", file=sys.stderr)
    finally:
        if connection is not None:
            if not _close_resource(connection, hardware=False, send_stop=False):
                exit_code = 2
    return exit_code


def _close_resource(
    resource: Any,
    *,
    hardware: bool,
    send_stop: bool | None = None,
) -> bool:
    """Attempt one close operation and report whether it completed safely."""

    try:
        result = resource.close() if send_stop is None else resource.close(send_stop=send_stop)
    except BaseException:
        message = (
            "error: Kortex cleanup failed; motion stop may be unconfirmed"
            if hardware
            else "error: cleanup failed"
        )
        print(message, file=sys.stderr)
        return False
    if result is False:
        message = (
            "error: Kortex cleanup failed; motion stop may be unconfirmed"
            if hardware
            else "error: cleanup failed"
        )
        print(message, file=sys.stderr)
        return False
    return True


def _cleanup_resources(
    controller: Any,
    source: Any,
    backend: Any,
    connection: Any,
    *,
    hardware: bool,
) -> bool:
    """Close every remaining resource, preferring the aggregate controller."""

    if controller is not None and _close_resource(controller, hardware=hardware):
        return True

    resources = (backend, connection, source)
    succeeded = True
    for resource in resources:
        if resource is not None:
            succeeded = _close_resource(resource, hardware=hardware) and succeeded
    return succeeded


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    error = _validate_args(args)
    if error is not None:
        print(f"error: {error}", file=sys.stderr)
        return 2
    if args.check_input or args.check_xr:
        try:
            return check_input(
                create_input(args),
                args.samples,
                args.check_timeout,
            )
        except Exception as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 2

    password: str | None = None
    if args.backend == "kortex":
        # Fail closed on non-secret gates before even looking up the password.
        error = _validate_kortex_args(args, None, check_password=False)
        if error is not None:
            print(f"error: {error}", file=sys.stderr)
            return 2
        password = os.getenv("KINOVA_PASSWORD")
        error = _validate_kortex_args(args, password)
        if error is not None:
            print(f"error: {error}", file=sys.stderr)
            return 2
        if args.check_kortex:
            return _run_kortex_readonly_check(args, password)

    control_hz = resolve_control_hz(args)
    source = None
    connection = None
    backend = None
    controller = None
    evidence_logger = None
    event_sink = None
    exit_code = 2
    completion: str | None = None
    try:
        if args.fixed_trajectory is None and args.backend != "kortex":
            source = create_input(args)
        if args.backend == "kortex":
            from .kortex_transport import KortexConfig

            workspace_limits = _workspace_from_args(args)
            if args.evidence_jsonl is not None:
                evidence_logger = EvidenceLogger(
                    args.evidence_jsonl,
                    run_id=args.run_id,
            )
            event_sink = logger_event_sink(evidence_logger) if evidence_logger is not None else None

            # Nothing that can transmit motion is constructed until runtime
            # compatibility and current-session robot state are admitted.  A
            # live PICO teleoperation path additionally requires stable
            # released input before and after confirmation.  The read-only
            # connection is never reused for motion.
            versions = validate_kortex_runtime()
            if args.fixed_trajectory is None:
                source = create_input(args)
                admitted = wait_for_fresh_released_input(
                    source,
                    sample_count=10,
                    timeout_s=args.check_timeout,
                )
            readonly = _create_kortex_connection(
                KortexConfig(args.robot_ip, args.robot_user, password),
                read_only=True,
            )
            try:
                live_report = run_kortex_readonly_preflight(
                    readonly,
                    _preflight_context(args, versions),
                )
                require_live_kortex_ready(live_report)
            finally:
                if not _close_resource(readonly, hardware=False, send_stop=False):
                    raise RuntimeError("read-only Kortex cleanup failed")
            confirm_move()
            if source is not None:
                verify_released_now(
                    source,
                    after_timestamp_ns=admitted.last_timestamp_ns,
                    admission=admitted,
                    timeout_s=0.5,
                )

            connection = _create_kortex_connection(
                KortexConfig(args.robot_ip, args.robot_user, password),
            )
            backend = _create_kortex_backend(
                connection,
                max_linear_speed=_resolve_max_linear_speed(args),
                max_angular_speed_deg=_resolve_max_angular_speed_deg(args),
                workspace_limits=workspace_limits,
                anchor_envelope=AnchorEnvelope(
                    FIRST_HARDWARE_PROFILE.anchor_translation_axis_m,
                    math.radians(FIRST_HARDWARE_PROFILE.anchor_rotation_deg),
                ),
                event_sink=event_sink,
            )
        else:
            from .mujoco_backend import MuJoCoBackend

            backend = MuJoCoBackend(args.model, control_hz=control_hz)

        if args.fixed_trajectory is not None:
            spec = load_trajectory(args.fixed_trajectory)
            result = FixedTrajectoryRunner(
                backend,
                control_hz=spec.control_hz,
                approve_segment=_approve_fixed_segment,
                event_sink=event_sink if args.backend == "kortex" else None,
            ).run(spec)
            exit_code = 0 if result.completed else 2
            completion = (
                f"completed fixed_segments={result.completed_segments}/{result.total_segments}"
            )
        else:
            controller_kwargs: dict[str, object] = {}
            if event_sink is not None:
                controller_kwargs["event_sink"] = event_sink
            controller = TeleopController(
                TeleopConfig(
                    control_hz=control_hz,
                    # Hardware must always run against the wall clock; free-running
                    # a headless loop against a real arm floods the robot with RPCs.
                    realtime=args.backend == "kortex" or not args.headless,
                    translation_scale=resolve_translation_scale(args),
                    orientation_enabled=not args.translation_only,
                    stale_timeout=_resolve_stale_timeout(args),
                    fatal_input_faults=args.backend == "kortex",
                ),
                source,
                backend,
                **controller_kwargs,
            )
            if args.backend == "kortex" or args.headless:
                controller.run(max_steps=args.steps, on_status=_print_status)
            else:
                viewer = backend.launch_viewer()
                controller.run(
                    max_steps=args.steps,
                    should_continue=viewer.is_running,
                    on_step=viewer.sync,
                    on_status=_print_status,
                )
            if args.backend == "mujoco":
                finite = backend.state_is_finite()
                exit_code = 0 if finite else 2
                completion = (
                    f"completed steps={controller.steps} "
                    f"finite_state={str(finite).lower()}"
                )
            else:
                exit_code = 0
                completion = f"completed steps={controller.steps}"
    except KeyboardInterrupt:
        print("stopped by user", file=sys.stderr)
        exit_code = 130
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        exit_code = 2
    finally:
        cleanup_succeeded = _cleanup_resources(
            controller,
            source,
            backend,
            connection,
            hardware=args.backend == "kortex",
        )
        # A failed close is fatal for every backend; Ctrl+C keeps 130 so the
        # interruption stays visible in the exit status.
        if not cleanup_succeeded and exit_code != 130:
            exit_code = 2

    # Never print a completion line when cleanup failed: for hardware that
    # would contradict a possible "motion stop may be unconfirmed" error.
    if completion is not None and cleanup_succeeded and exit_code in (0, 130):
        print(completion)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
