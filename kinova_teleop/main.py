"""Command-line entry point for Kinova Gen3 PICO teleoperation."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
import math
import os
from pathlib import Path
import sys
import time
from typing import Any

from .pico_udp_input import PicoUdpInput
from .teleop_controller import StepDiagnostics, TeleopConfig, TeleopController
from .xr_input import DryRunXrInput, SdkXrInput, XrInputSource


DEFAULT_MODEL = (
    Path(__file__).resolve().parents[1]
    / "kinova_gen3_mujoco"
    / "teleop_scene.xml"
)

DEFAULT_MUJOCO_CONTROL_HZ = 100.0
DEFAULT_KORTEX_CONTROL_HZ = 25.0
MAX_KORTEX_CONTROL_HZ = 40.0
DEFAULT_KORTEX_ROBOT_IP = "192.168.1.10"
DEFAULT_KORTEX_ROBOT_USER = "admin"
DEFAULT_KORTEX_LINEAR_SPEED = 0.03
DEFAULT_KORTEX_ANGULAR_SPEED_DEG = 5.0
MAX_KORTEX_SCALE = 0.5
MAX_KORTEX_STALE_TIMEOUT = 0.2


def _create_kortex_connection(config: Any) -> Any:
    """Construct the optional Kortex transport after the hardware gate passes."""

    from .kortex_transport import KortexConnection

    return KortexConnection(config).connect()


def _create_kortex_backend(connection: Any, **kwargs: Any) -> Any:
    """Construct the optional Kortex backend after the hardware gate passes."""

    from .kortex_backend import KortexBackend

    return KortexBackend(connection, **kwargs)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
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
        default=0.5,
        help="controller-to-robot translation scale (default: 0.5)",
    )
    parser.add_argument(
        "--control-hz",
        type=float,
        default=None,
        help=(
            "teleoperation update rate "
            f"(default: {DEFAULT_MUJOCO_CONTROL_HZ:g} for MuJoCo, "
            f"{DEFAULT_KORTEX_CONTROL_HZ:g} for Kortex; "
            f"Kortex maximum: {MAX_KORTEX_CONTROL_HZ:g})"
        ),
    )
    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=0.2,
        help="seconds before unchanged XR timestamps release the clutch (default: 0.2)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="use deterministic synthetic 6DoF controller input",
    )
    parser.add_argument(
        "--input",
        choices=("pico-udp", "xrobotoolkit"),
        default="pico-udp",
        help="controller input source (default: pico-udp)",
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
        default=DEFAULT_KORTEX_LINEAR_SPEED,
        help="Kortex linear-speed limit in m/s (maximum: 0.03)",
    )
    parser.add_argument(
        "--max-angular-speed-deg",
        type=float,
        default=DEFAULT_KORTEX_ANGULAR_SPEED_DEG,
        help="Kortex angular-speed limit in deg/s (maximum: 5)",
    )
    parser.add_argument(
        "--gripper",
        action="store_true",
        help=(
            "drive the Kortex gripper from the controller trigger "
            "(requires --backend kortex and a Unity bridge sending "
            "protocol V2 packets)"
        ),
    )
    return parser


def resolve_control_hz(args: argparse.Namespace) -> float:
    if args.control_hz is not None:
        return float(args.control_hz)
    if args.backend == "kortex":
        return DEFAULT_KORTEX_CONTROL_HZ
    return DEFAULT_MUJOCO_CONTROL_HZ


def create_input(args: argparse.Namespace) -> XrInputSource:
    """Construct the selected input without requiring a fixed PICO address."""
    control_hz = resolve_control_hz(args)
    if args.dry_run:
        return DryRunXrInput(control_hz=control_hz)
    if args.input == "pico-udp":
        return PicoUdpInput(
            host=args.pico_host,
            port=args.pico_port,
            stale_after=args.stale_timeout,
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
    if args.control_hz is not None and (
        not math.isfinite(args.control_hz) or args.control_hz <= 0.0
    ):
        return "--control-hz must be positive"
    if not math.isfinite(args.scale) or args.scale <= 0.0:
        return "--scale must be positive"
    if not math.isfinite(args.stale_timeout) or args.stale_timeout <= 0.0:
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
    if args.gripper and args.backend != "kortex":
        return "--gripper requires --backend kortex"
    return None


def _validate_kortex_args(args: argparse.Namespace, password: str | None) -> str | None:
    """Validate every hardware input before importing or connecting Kortex."""

    if not args.enable_hardware:
        return "--backend kortex requires --enable-hardware"
    if args.dry_run:
        return "--dry-run cannot be used with --backend kortex"
    if args.stale_timeout > MAX_KORTEX_STALE_TIMEOUT:
        return "--stale-timeout must not exceed 0.2 for --backend kortex"
    if not args.robot_ip.strip():
        return "--robot-ip must not be empty"
    if not args.robot_user.strip():
        return "--robot-user must not be empty"
    if not password:
        return "KINOVA_PASSWORD must be set for --backend kortex"
    if args.scale > MAX_KORTEX_SCALE:
        return "--scale must not exceed 0.5 for --backend kortex"
    if args.control_hz is not None and args.control_hz > MAX_KORTEX_CONTROL_HZ:
        return (
            f"--control-hz must not exceed {MAX_KORTEX_CONTROL_HZ:g} "
            "for --backend kortex"
        )
    if not math.isfinite(args.max_linear_speed) or not (
        0.0 < args.max_linear_speed <= DEFAULT_KORTEX_LINEAR_SPEED
    ):
        return "--max-linear-speed must be in (0, 0.03]"
    if not math.isfinite(args.max_angular_speed_deg) or not (
        0.0 < args.max_angular_speed_deg <= DEFAULT_KORTEX_ANGULAR_SPEED_DEG
    ):
        return "--max-angular-speed-deg must be in (0, 5]"
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


def _close_resource(resource: Any, *, hardware: bool) -> bool:
    """Attempt one close operation and report whether it completed safely."""

    try:
        result = resource.close()
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
        password = os.getenv("KINOVA_PASSWORD")
        error = _validate_kortex_args(args, password)
        if error is not None:
            print(f"error: {error}", file=sys.stderr)
            return 2
        try:
            confirmation = input("Type MOVE to enable Kortex motion: ")
        except KeyboardInterrupt:
            print("stopped by user", file=sys.stderr)
            return 130
        except Exception:
            print("error: hardware confirmation failed", file=sys.stderr)
            return 2
        if confirmation != "MOVE":
            print("error: hardware motion was not confirmed", file=sys.stderr)
            return 2

    control_hz = resolve_control_hz(args)
    source = None
    connection = None
    backend = None
    controller = None
    exit_code = 2
    completion: str | None = None
    try:
        source = create_input(args)
        if args.backend == "kortex":
            from .kortex_transport import KortexConfig

            connection = _create_kortex_connection(
                KortexConfig(args.robot_ip, args.robot_user, password),
            )
            backend = _create_kortex_backend(
                connection,
                max_linear_speed=args.max_linear_speed,
                max_angular_speed_deg=args.max_angular_speed_deg,
            )
        else:
            from .mujoco_backend import MuJoCoBackend

            backend = MuJoCoBackend(args.model, control_hz=control_hz)

        controller = TeleopController(
            TeleopConfig(
                control_hz=control_hz,
                # Hardware must always run against the wall clock; free-running
                # a headless loop against a real arm floods the robot with RPCs.
                realtime=args.backend == "kortex" or not args.headless,
                translation_scale=args.scale,
                stale_timeout=args.stale_timeout,
                gripper=args.gripper,
            ),
            source,
            backend,
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
            completion = (
                f"completed steps={controller.steps} "
                f"finite_state={str(finite).lower()}"
            )
            exit_code = 0 if finite else 2
        else:
            completion = f"completed steps={controller.steps}"
            exit_code = 0
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
        if args.backend == "kortex" and not cleanup_succeeded and exit_code != 130:
            exit_code = 2

    if completion is not None and exit_code in (0, 130):
        print(completion)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
