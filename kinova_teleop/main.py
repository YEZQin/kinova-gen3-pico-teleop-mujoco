"""Command-line entry point for Kinova Gen3 PICO teleoperation."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from pathlib import Path
import sys
import time

import numpy as np

from .teleop_controller import TeleopConfig, TeleopController
from .xr_input import DryRunXrInput, SdkXrInput


DEFAULT_MODEL = (
    Path(__file__).resolve().parents[1]
    / "kinova_gen3_mujoco"
    / "teleop_scene.xml"
)


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
        default=1.0,
        help="controller-to-robot translation scale (default: 1.0)",
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
    if args.control_hz <= 0.0:
        return "--control-hz must be positive"
    if args.scale <= 0.0:
        return "--scale must be positive"
    if args.stale_timeout <= 0.0:
        return "--stale-timeout must be positive"
    if args.headless and (args.steps is None or args.steps <= 0):
        return "--headless requires a positive --steps"
    if args.steps is not None and args.steps <= 0:
        return "--steps must be positive"
    if args.samples is not None and args.samples <= 0:
        return "--samples must be positive"
    if args.samples is not None and not args.check_xr:
        return "--samples is only valid with --check-xr"
    return None


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    error = _validate_args(args)
    if error is not None:
        print(f"error: {error}", file=sys.stderr)
        return 2
    if args.check_xr:
        return check_xr(args.samples)

    source = None
    try:
        source = (
            DryRunXrInput(control_hz=args.control_hz)
            if args.dry_run
            else SdkXrInput()
        )
        controller = TeleopController(
            TeleopConfig(
                model_path=args.model,
                control_hz=args.control_hz,
                realtime=not args.headless,
                translation_scale=args.scale,
                stale_timeout=args.stale_timeout,
            ),
            source,
        )
        controller.run(headless=args.headless, max_steps=args.steps)
        finite = all(
            np.isfinite(array).all()
            for array in (
                controller.data.qpos,
                controller.data.qvel,
                controller.data.ctrl,
            )
        )
        print(
            f"completed steps={controller.steps} finite_state={str(finite).lower()}",
        )
        return 0 if finite else 2
    except KeyboardInterrupt:
        print("stopped by user", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    finally:
        if source is not None:
            source.close()


if __name__ == "__main__":
    raise SystemExit(main())
