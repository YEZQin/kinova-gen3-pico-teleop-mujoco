# Kinova Gen3 first-hardware quickstart

This branch is implementation-ready for an onsite validation; it is not a
claim of physical stopping performance. The first-visit profile is bare-arm
only and keeps all motion behind explicit, independently checked gates.

## Hard constraints

- Use a Kortex wheel matched to the robot firmware, platform, and supported
  Python version. Keep the wheel in an offline wheelhouse and verify
  `import kortex_api` in the target virtual environment.
- Use trusted wired TCP to the robot on port `10000`; do not authorize motion
  when the route is unreachable.
- Read the password only from `KINOVA_PASSWORD`; never print, persist, or put
  it on a command line.
- The first-hardware profile rejects `--gripper` before password lookup,
  imports, prompts, or a connection. It never runs `ClearFaults`, Home/action
  playback, low-level cyclic control, or a watchdog bypass.
- Keep the documented caps: 40 Hz, scale 0.5, stale timeout 0.2 s, linear
  speed 0.03 m/s, and angular speed 5 deg/s. Start at 25 Hz, 0.01 m/s, and
  2 deg/s.

## Offline gates

```powershell
python -m pytest -q
python -m kinova_teleop.main --dry-run --headless --steps 2000
python -m kinova_teleop.main --check-xr --samples 100 --input pico-udp
```

The dry run must report a finite state. None of these commands imports the
Kortex SDK or connects to a robot.

## Read-only Kortex check

Set the password without echoing it, then run the read-only preflight. The
program requires the exact `CONNECT` confirmation and reads only arm state and
one feedback frame. It does not set servoing mode or issue motion commands.

```powershell
$env:KINOVA_PASSWORD = Read-Host 'Kortex password'
python -m kinova_teleop.main `
  --backend kortex --enable-hardware --check-kortex `
  --robot-ip 192.168.1.10 --robot-user admin `
  --preflight-json results\gen3-preflight.json
```

Inspect the JSON report and the Kinova Web App for firmware, faults, payload,
limits, and the physical checklist. A read-only RPC success does not certify
the physical checklist.

## Guarded motion

Motion requires an external supervisor lease. The adapter validates the lease
read-only; it never acquires, deletes, or transfers it. Both explicit inclusive
XYZ bounds and a matching lease are required before the password is read or
the exact `MOVE` prompt is shown.

```powershell
python -m kinova_teleop.main `
  --backend kortex --enable-hardware `
  --workspace-min -0.20 -0.20 0.10 `
  --workspace-max 0.20 0.20 0.60 `
  --motion-lease C:\path\to\motion.lock `
  --run-id gen3-first-hardware --lease-owner kinova-teleop `
  --max-linear-speed 0.01 --max-angular-speed-deg 2
```

Grip must be released below 0.8 before motion authorization and pressed above
0.9 to activate the clutch. The first valid press anchors the current
controller pose to the measured end-effector pose; it must not command a jump.

## Fixed micro-trajectory

The checked-in trajectory contains ten independent, orientation-preserving,
millimetre-scale excursions that return to the measured anchor. It is not a
replacement for the physical checklist and still requires lease, bounds,
password, and `MOVE`:

```powershell
python -m kinova_teleop.main `
  --backend kortex --enable-hardware --input none `
  --fixed-trajectory configs\gen3_micro_axes.json `
  --workspace-min -0.20 -0.20 0.10 `
  --workspace-max 0.20 0.20 0.60 `
  --motion-lease C:\path\to\motion.lock `
  --run-id gen3-first-hardware --lease-owner kinova-teleop `
  --max-linear-speed 0.01 --max-angular-speed-deg 2
```

Each segment stops on the first rejection and requires a fresh low-velocity
feedback observation before another segment is authorized. The program asks
for an exact `MOVE` confirmation before every segment; a missing confirmation
hook is fail-closed. Normal RPC return or cleanup is not evidence of physical
stillness, and the adapter never fabricates `physical_stop_observed`.

## Stop semantics and observations

Grip release, stale XR input, an RPC error, watchdog expiry, and program exit
request Stop through the existing backend generation/lock path. The 0.2 s
property is a host stop-request/admission boundary, never a physical-stop
guarantee. Evidence distinguishes `host_stop_requested`, `stop_rpc_returned`,
and `stop_unconfirmed`; a failed Stop requires the physical E-stop or Kinova
Web App intervention before relaunch.

Only a responsible onsite operator with a reachable physical E-stop may run a
motion trial. Record direction, anchor behavior, and observed stopping for Grip
release, stale input, network loss, and Ctrl+C. Label evidence as offline,
read-only, or hardware-observed.
