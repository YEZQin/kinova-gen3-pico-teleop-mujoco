# Kinova Gen3 first-hardware quickstart

This branch is **offline verified** only. It is not a claim of physical
stopping performance. A system becomes **read-only verified** only after a
current-session T0 check, and **hardware-observed** only after T0--T3 are
recorded for the same code revision and device session.

The tested first-hardware stack is Python 3.11.15, Kortex `2.8.0.post5`,
protobuf `3.20.0`, and firmware `2.8.0-5`. Use only a matching offline
wheelhouse; do not substitute an unreviewed SDK or firmware version.

## Non-negotiable constraints

- Use a trusted wired private IPv4 route to the robot on port `10000`.
- Read credentials only from `KINOVA_PASSWORD`; never put them in command
  lines, evidence, logs, or launcher parameters.
- The first-hardware profile is fixed at 40 Hz, scale 0.25, stale timeout
  0.2 s, 0.005 m/s linear speed, and 2 deg/s angular speed.
- The anchor-relative envelope is ±0.02 m on each translation axis and 5 deg
  in rotation. It is additional to tight, operator-reviewed, inclusive
  absolute XYZ bounds; never use a broad reusable workspace example.
- Gripper and vision writes are disabled. The software does not clear faults,
  home the arm, play actions, use low-level cyclic control, bypass the
  watchdog, or reconnect automatically.
- Grip release is recoverable. Stale, invalid, source-change, RPC, workspace,
  and watchdog faults latch and exit. A Stop RPC return is not proof that the
  arm is physically still.

## 1. Offline evidence

Run the offline suite before travel. These commands neither import the Kortex
SDK nor connect to the robot:

```powershell
python -m pytest -q
python -m kinova_teleop.main --dry-run --headless --steps 2000
python -m kinova_teleop.main --check-xr --samples 100 --input pico-udp
```

Record these results as **offline verified**. The PICO must later be fresh and
released before a live read-only connection and again before the exact `MOVE`
confirmation. A reviewed report and a supervisor lease are necessary motion
artifacts, but neither replaces these live checks.

## 2. T0: current read-only evidence

Before connecting, verify a reachable physical E-stop, second observer, clear
workspace, fixture, cables, payload/TCP, and Kinova Web App status. Release
Grip and ensure the PICO is producing fresh input. Set the environment
credential without echoing it, then run only the exact read-only path with
`CONNECT` and save a new, non-overwriting report:

```powershell
& .\scripts\assert_no_untracked_python_startup_hooks.ps1 `
  -ProjectRoot (Get-Location).Path
$securePassword = Read-Host 'Kortex password' -AsSecureString
try {
  $env:KINOVA_PASSWORD = [System.Net.NetworkCredential]::new('', $securePassword).Password
  python -m kinova_teleop.main `
    --backend kortex --enable-hardware --check-kortex `
    --robot-ip 192.168.1.10 --robot-user admin `
    --preflight-json results\gen3-t0-read-only.json
} finally {
  Remove-Item Env:KINOVA_PASSWORD -ErrorAction SilentlyContinue
  $securePassword.Dispose()
}
```

The conversion exists only while the Python process runs; the value is neither
printed nor persisted, and the environment entry is removed even if the
read-only check fails.

Confirm L53 / 7 DoF, firmware `2.8.0-5`, Running, Single Level Servoing,
`SERVOING_READY`, and a finite pose. Do not continue if read-only cleanup
fails. The `--check-kortex` JSON is read-only evidence: it intentionally does
not become an accepted motion report. A separate central-schema report must be
reviewed with every software check, physical check, and `passed=true` before
it can be supplied as `--preflight-report`.

## Mandatory pause after T0

Stop here and present T0 to the onsite operator. Do not construct or present a
motion invocation until they authorize continuing, confirm that onsite safety
conditions still hold, review the actual anchor pose and tight absolute XYZ
bounds, validate the separately reviewed report and lease paths, and confirm
that Grip is released. The approved `scripts/start_kortex_pico_teleop.ps1`
launcher accepts the reviewed artifacts and has no credential parameter; it
fixes the first-hardware profile rather than accepting arbitrary limits.

## 3. Hardware-observed trials

After the mandatory pause, keep an evidence log and label each observation by
the following gates:

- **T1 — arm/no-jump:** verify no motion before Grip press, first press only
  anchors the controller to measured end-effector pose, and release requests
  Stop. If Stop is unconfirmed, use physical E-stop/Web intervention and do
  not relaunch.
- **T2 — six-axis micro-motion:** test X/Y/Z separately at millimetre scale,
  then Roll/Pitch/Yaw separately at small angle. Record commanded and observed
  direction, peak speed, envelope status, and Stop reason. Any sign or
  reference-frame mismatch ends the trial.
- **T3 — stop paths:** one at a time test Grip release, PICO stale,
  controlled source/session loss, `Ctrl+C`, and watchdog stall. Record both
  event-to-host-Stop-request time and feedback-to-stationary time. Automatic
  recovery or continued motion is a failure.

The checked-in `configs/gen3_micro_axes.json` is limited to
orientation-preserving millimetre excursions and returns to the measured
anchor. It does not replace T0, the required lease, the reviewed report, fresh
released PICO input, tight absolute bounds, or the operator's `MOVE`
confirmation. Do not relaunch after a latched fault; resolve it through the
onsite safety process first.
