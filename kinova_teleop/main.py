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

import numpy as np

from .teleop_controller import TeleopConfig, TeleopController
from .xr_input import DryRunXrInput, SdkXrInput


DEFAULT_MODEL = (
    Path(__file__).resolve().parents[1]
    / "kinova_gen3_mujoco"
    / "teleop_scene.xml"
)

DEFAULT_KORTEX_ROBOT_IP = "192.168.1.10"
DEFAULT_KORTEX_ROBOT_USER = "admin"
DEFAULT_KORTEX_LINEAR_SPEED = 0.03
DEFAULT_KORTEX_ANGULAR_SPEED_DEG = 5.0


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
            "Use the PICO left controller to command a Kinova Gen3 arm in MuJoCo."
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
        help="translation scale (default: 1.0 for MuJoCo, 0.5 for Kortex)",
    )
    parser.add_argument(
        "--control-hz",
        type=float,
        default=100.0,
        help="teleoperation update rate (default: 100)",
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
        "--check-xr",
        action="store_true",
        help="read and print XR samples without loading MuJoCo",
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
        help="stop --check-xr after this many samples",
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
    return parser


def check_xr(samples: int | None) -> int:
    if samples is not None and samples <= 0:
        print("error: --samples must be positive", file=sys.stderr)
        return 2

    try:
        source = SdkXrInput()
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    read_count = 0
    last_timestamp: int | None = None
    try:
        while samples is None or read_count < samples:
            sample = source.read()
            if not sample.valid:
                print(
                    "error: received an invalid left-controller pose",
                    file=sys.stderr,
                )
                return 2
            timestamp_state = (
                "new"
                if last_timestamp is None or sample.timestamp_ns > last_timestamp
                else "unchanged"
            )
            values = " ".join(f"{value:+.5f}" for value in sample.position)
            quaternion = " ".join(
                f"{value:+.5f}" for value in sample.quaternion_xyzw
            )
            print(
                f"left position=[{values}] quat_xyzw=[{quaternion}] "
                f"grip={sample.grip:.3f} timestamp_ns={sample.timestamp_ns} "
                f"({timestamp_state})",
                flush=True,
            )
            last_timestamp = sample.timestamp_ns
            read_count += 1
            if samples is None:
                time.sleep(0.01)
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"error: XR input failed: {exc}", file=sys.stderr)
        return 2
    finally:
        source.close()
    return 0


def _validate_args(args: argparse.Namespace) -> str | None:
    if not math.isfinite(args.control_hz) or args.control_hz <= 0.0:
        return "--control-hz must be positive and finite"
    if args.scale is not None and (
        not math.isfinite(args.scale) or args.scale <= 0.0
    ):
        return "--scale must be positive and finite"
    if not math.isfinite(args.stale_timeout) or args.stale_timeout <= 0.0:
        return "--stale-timeout must be positive and finite"
    if args.headless and (args.steps is None or args.steps <= 0):
        return "--headless requires a positive --steps"
    if args.steps is not None and args.steps <= 0:
        return "--steps must be positive"
    if args.samples is not None and args.samples <= 0:
        return "--samples must be positive"
    if args.samples is not None and not args.check_xr:
        return "--samples is only valid with --check-xr"
    return None


def _validate_kortex_args(args: argparse.Namespace, password: str | None) -> str | None:
    """Validate every hardware input before importing or connecting Kortex."""

    if not args.enable_hardware:
        return "--backend kortex requires --enable-hardware"
    if args.dry_run:
        return "--dry-run cannot be used with --backend kortex"
    if not password:
        return "KINOVA_PASSWORD must be set for --backend kortex"
    if args.scale is not None and args.scale > 0.5:
        return "--scale must not exceed 0.5 for --backend kortex"
    if not math.isfinite(args.max_linear_speed) or not (
        0.0 < args.max_linear_speed <= DEFAULT_KORTEX_LINEAR_SPEED
    ):
        return "--max-linear-speed must be in (0, 0.03]"
    if not math.isfinite(args.max_angular_speed_deg) or not (
        0.0 < args.max_angular_speed_deg <= DEFAULT_KORTEX_ANGULAR_SPEED_DEG
    ):
        return "--max-angular-speed-deg must be in (0, 5]"
    return None


def _safe_close(resource: Any) -> None:
    """Release a partially constructed resource without masking the main result."""

    try:
        resource.close()
    except BaseException:
        print("error: cleanup failed", file=sys.stderr)


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    error = _validate_args(args)
    if error is not None:
        print(f"error: {error}", file=sys.stderr)
        return 2
    if args.check_xr:
        return check_xr(args.samples)

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

    source = None
    connection = None
    backend = None
    controller = None
    try:
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
            source = SdkXrInput()
        else:
            source = (
                DryRunXrInput(control_hz=args.control_hz)
                if args.dry_run
                else SdkXrInput()
            )

        scale = args.scale if args.scale is not None else (
            0.5 if args.backend == "kortex" else 1.0
        )
        controller = TeleopController(
            TeleopConfig(
                model_path=args.model,
                control_hz=args.control_hz,
                realtime=not args.headless,
                translation_scale=scale,
                stale_timeout=args.stale_timeout,
            ),
            source,
            backend,
        )
        controller.run(headless=args.headless, max_steps=args.steps)
        if args.backend == "mujoco":
            finite = all(
                np.isfinite(array).all()
                for array in (
                    controller.data.qpos,
                    controller.data.qvel,
                    controller.data.ctrl,
                )
            )
            print(
                f"completed steps={controller.steps} "
                f"finite_state={str(finite).lower()}",
            )
            return 0 if finite else 2
        print(f"completed steps={controller.steps}")
        return 0
    except KeyboardInterrupt:
        print("stopped by user", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    finally:
        if controller is not None:
            _safe_close(controller)
        else:
            if source is not None:
                _safe_close(source)
            if backend is not None:
                _safe_close(backend)
            elif connection is not None:
                _safe_close(connection)


if __name__ == "__main__":
    raise SystemExit(main())
