"""Command-line entry point for Kinova Gen3 PICO teleoperation."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
import math
from pathlib import Path
import sys
import time

from .backend import MuJoCoBackend
from .pico_udp_input import PicoUdpInput
from .teleop_controller import StepDiagnostics, TeleopConfig, TeleopController
from .xr_input import DryRunXrInput, SdkXrInput, XrInputSource


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
        default=0.5,
        help="controller-to-robot translation scale (default: 0.5)",
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
        help="stop --check-xr after this many samples",
    )
    return parser


def create_input(args: argparse.Namespace) -> XrInputSource:
    """Construct the selected input without requiring a fixed PICO address."""
    if args.dry_run:
        return DryRunXrInput(control_hz=args.control_hz)
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
                f"grip={sample.grip:.3f} timestamp_ns={sample.timestamp_ns}",
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
    if not math.isfinite(args.control_hz) or args.control_hz <= 0.0:
        return "--control-hz must be positive"
    if not math.isfinite(args.scale) or args.scale <= 0.0:
        return "--scale must be positive"
    if not math.isfinite(args.stale_timeout) or args.stale_timeout <= 0.0:
        return "--stale-timeout must be positive"
    if not 1 <= args.pico_port <= 65535:
        return "--pico-port must be between 1 and 65535"
    if not math.isfinite(args.check_timeout) or args.check_timeout <= 0.0:
        return "--check-timeout must be positive"
    if args.headless and (args.steps is None or args.steps <= 0):
        return "--headless requires a positive --steps"
    if args.steps is not None and args.steps <= 0:
        return "--steps must be positive"
    if args.samples is not None and args.samples <= 0:
        return "--samples must be positive"
    if args.check_input and args.samples is None:
        return "--check-input requires a positive --samples"
    if args.samples is not None and not (args.check_input or args.check_xr):
        return "--samples is only valid with --check-input"
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

    source = None
    backend = None
    controller = None
    try:
        source = create_input(args)
        backend = MuJoCoBackend(args.model, control_hz=args.control_hz)
        controller = TeleopController(
            TeleopConfig(
                control_hz=args.control_hz,
                realtime=not args.headless,
                translation_scale=args.scale,
                stale_timeout=args.stale_timeout,
            ),
            source,
            backend,
        )
        if args.headless:
            controller.run(max_steps=args.steps, on_status=_print_status)
        else:
            viewer = backend.launch_viewer()
            controller.run(
                max_steps=args.steps,
                should_continue=viewer.is_running,
                on_step=viewer.sync,
                on_status=_print_status,
            )
        finite = backend.state_is_finite()
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
        if controller is not None:
            controller.close()
        else:
            if source is not None:
                source.close()
            if backend is not None:
                backend.close()


if __name__ == "__main__":
    raise SystemExit(main())
