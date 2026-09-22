"""Independent PICO tracker -> relative translation -> Kortex Cartesian PID.

No hardware connection unless --enable-hardware is explicitly supplied.
Once admitted, follow automatically without Grip or an additional MOVE prompt.
"""

import argparse
import csv
import getpass
import importlib
import math
from pathlib import Path
import sys
import time

import numpy as np

from .pose_mapping import PICO_TO_WORLD
from .tracker_follow import TrackerFollower
from .workspace import WorkspaceLimits


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    modes = result.add_mutually_exclusive_group()
    modes.add_argument('--list-trackers', action='store_true')
    modes.add_argument('--check-input', action='store_true')
    modes.add_argument('--dry-run', action='store_true')
    modes.add_argument('--enable-hardware', action='store_true')
    result.add_argument('--tracker-serial')
    result.add_argument('--robot-ip')
    result.add_argument('--robot-user', default='admin')
    result.add_argument('--workspace-min', type=float, nargs=3)
    result.add_argument('--workspace-max', type=float, nargs=3)
    result.add_argument('--rotation', type=float, nargs=9, default=PICO_TO_WORLD.flatten().tolist(),
                        help='row-major proper rotation from tracker coordinates to robot base')
    result.add_argument('--scale', type=float, default=1.)
    result.add_argument('--kp', type=float, default=3.3)
    result.add_argument('--ki', type=float, default=.15)
    result.add_argument('--kd', type=float, default=.02)
    result.add_argument('--max-linear-speed', type=float, default=.05)
    result.add_argument('--control-hz', type=float, default=40.)
    result.add_argument('--timeout', type=float, default=15.)
    result.add_argument('--samples', type=int, default=100)
    result.add_argument('--log', type=Path, help='new CSV path; never overwrites a prior run')
    return result


def key():
    import msvcrt
    if msvcrt.kbhit():
        char = msvcrt.getwch()
        if char in ('\x00', '\xe0'):
            if msvcrt.kbhit():
                msvcrt.getwch()
            return None
        return char
    return None


def wait_tracker(source, timeout):
    end = time.monotonic() + timeout
    previous = None
    count = 0
    while time.monotonic() < end:
        if key() in ('\x1b', '\x03'):
            raise KeyboardInterrupt
        observation = source.read()
        if observation.valid and observation.timestamp_ns != previous:
            count += 1
            previous = observation.timestamp_ns
            if count >= 10:
                return
        elif not observation.valid:
            count = 0
        time.sleep(.01)
    raise RuntimeError('No stable fresh tracker stream; enable Object Tracking and Send')


def benchmark(args, pid):
    """Ideal velocity-driven plant only; does not model Kinova physics or transport."""
    dt = 1. / args.control_hz
    current = np.zeros(3)
    target = np.array([.01, 0., 0.])
    t90 = None
    for step in range(int(3 * args.control_hz)):
        velocity = pid.update(target, current, step * dt)
        current = current + velocity * dt
        if t90 is None and current[0] >= .009:
            t90 = (step + 1) * dt
    print(f'ideal_10mm_step_t90_s={t90} final_error_mm={np.linalg.norm(target-current)*1000:.4f}')
    print('IDEAL MODEL ONLY: hardware response and network latency are not measured.')
    return 0


def hardware_backend(args, pid):
    from .hardware_profile import validate_kortex_runtime, validate_private_robot_ipv4
    from .kortex_backend import KortexBackend
    from .kortex_transport import KortexConfig, KortexConnection
    from .preflight import PreflightContext, run_kortex_readonly_preflight, require_live_kortex_ready
    validate_private_robot_ipv4(args.robot_ip)
    validate_kortex_runtime()
    config = KortexConfig(args.robot_ip, args.robot_user, getpass.getpass('Kortex password: '))
    readonly = KortexConnection(config)
    try:
        readonly.connect(send_stop_on_failure=False)
        report = run_kortex_readonly_preflight(readonly, PreflightContext())
        require_live_kortex_ready(report)
    finally:
        if not readonly.close(send_stop=False):
            raise RuntimeError('Read-only connection cleanup failed')
    connection = KortexConnection(config)
    try:
        connection.connect()
        backend = KortexBackend(connection, anchor_envelope=None,
                                workspace_limits=WorkspaceLimits(tuple(args.workspace_min), tuple(args.workspace_max)),
                                advanced_translation=True, max_linear_speed=args.max_linear_speed,
                                max_angular_speed_deg=2., translation_pid=pid)
        pose = backend.current_pose()
        if np.any(pose.position < args.workspace_min) or np.any(pose.position > args.workspace_max):
            raise ValueError('Current TCP lies outside the supplied workspace')
        return backend
    except BaseException:
        if not connection.close():
            raise RuntimeError('Hardware startup cleanup/Stop unconfirmed')
        raise


def validate(args):
    for name in ('control_hz', 'timeout', 'scale', 'max_linear_speed'):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be positive and finite')
    if args.control_hz > 40 or args.scale > 1 or args.max_linear_speed > .05:
        raise ValueError('Maximums: control_hz=40, scale=1, max_linear_speed=.05')
    if args.samples <= 0:
        raise ValueError('samples must be positive')
    rotation = np.array(args.rotation).reshape(3, 3)
    TrackerFollower(None, rotation, args.scale)
    if args.enable_hardware:
        from .hardware_profile import validate_private_robot_ipv4
        validate_private_robot_ipv4(args.robot_ip or '')
        if args.workspace_min is None or args.workspace_max is None:
            raise ValueError('Hardware requires explicit --workspace-min and --workspace-max')
        WorkspaceLimits(tuple(args.workspace_min), tuple(args.workspace_max))
        if np.any(np.array(args.workspace_min) >= args.workspace_max) or args.workspace_min[2] < 0:
            raise ValueError('Workspace bounds must be strictly ordered and lower Z >= 0')
        if not sys.stdin.isatty() or sys.platform != 'win32':
            raise ValueError('Hardware keyboard mode requires an interactive native Windows terminal')
    return rotation


def run_follow(args, source, backend, rotation, stream):
    follower = TrackerFollower(backend, rotation, args.scale)
    writer = csv.writer(stream)
    writer.writerow(['host_monotonic_s', 'tracker_timestamp_ns', 'state',
                     'target_x_m', 'target_y_m', 'target_z_m',
                     'actual_x_m', 'actual_y_m', 'actual_z_m', 'tracker_time_source'])
    last_status = None
    print('AUTO FOLLOW after anchoring. Space=pause/resume; Esc/Ctrl+C=stop. Keep this terminal focused.', flush=True)
    while True:
        start = time.monotonic()
        pressed = key()
        if pressed in ('\x1b', '\x03'):
            break
        if pressed == ' ':
            follower.toggle_pause()
        sample = source.read()
        follower.tick(sample)
        if follower.status != last_status:
            print(f'state={follower.status}', flush=True)
            last_status = follower.status
        if follower.last_target is not None and not follower.paused:
            actual = backend.current_pose()
            writer.writerow([time.monotonic(), sample.timestamp_ns, follower.status,
                             *follower.last_target.position, *actual.position,
                             getattr(sample, 'time_source', 'device_motion')])
            stream.flush()
        time.sleep(max(0., 1. / args.control_hz - (time.monotonic()-start)))


def main(argv=None):
    args = parser().parse_args(argv)
    source = backend = stream = None
    code = 0
    try:
        rotation = validate(args)
        from .cartesian_pid import CartesianPID
        pid = CartesianPID(kp=args.kp, ki=args.ki, kd=args.kd, max_speed=args.max_linear_speed)
        if args.dry_run or not (args.enable_hardware or args.check_input or args.list_trackers):
            return benchmark(args, pid)
        if args.list_trackers:
            sdk = importlib.import_module('xrobotoolkit_sdk')
            sdk.init()
            try:
                time.sleep(1.)
                print('trackers=', sdk.get_motion_tracker_serial_numbers())
            finally:
                sdk.close()
            return 0
        if not args.tracker_serial or not args.tracker_serial.strip():
            raise ValueError('--tracker-serial is required; use --list-trackers first')
        from .motion_tracker import MotionTrackerInput
        source = MotionTrackerInput(args.tracker_serial)
        if args.check_input:
            end = time.monotonic() + args.timeout
            count, previous = 0, None
            while count < args.samples and time.monotonic() < end:
                sample = source.read()
                if sample.valid and sample.timestamp_ns != previous:
                    print(f'position={sample.position} timestamp_ns={sample.timestamp_ns} time_source={getattr(sample, "time_source", "device_motion")}')
                    previous = sample.timestamp_ns
                    count += 1
                time.sleep(.025)
            if count < args.samples:
                raise RuntimeError('Tracker input check timed out')
        else:
            wait_tracker(source, args.timeout)
            log_path = args.log or Path('sessions') / f'tracker-{time.time_ns()}.csv'
            log_path.parent.mkdir(parents=True, exist_ok=True)
            stream = log_path.open('x', newline='', encoding='utf-8')
            backend = hardware_backend(args, pid)
            wait_tracker(source, args.timeout)
            run_follow(args, source, backend, rotation, stream)
    except KeyboardInterrupt:
        print('Stopped by keyboard')
    except Exception as error:
        print(f'error: {error}', file=sys.stderr)
        code = 2
    finally:
        for name, resource in (('robot', backend), ('tracker', source), ('log', stream)):
            if resource is not None:
                try:
                    result = resource.close()
                    if result is False:
                        raise RuntimeError('cleanup returned failure')
                except BaseException:
                    print(f'error: {name} cleanup failed; Stop may be unconfirmed', file=sys.stderr)
                    code = 2
    return code


if __name__ == '__main__':
    raise SystemExit(main())
