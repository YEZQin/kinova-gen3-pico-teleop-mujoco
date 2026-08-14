# PICO-to-Kinova Gen3 guarded teleoperation

[中文说明](README_CN.md) | [Evidence levels](docs/evidence-levels.md) | [PICO detail](docs/pico-udp-quickstart.md) | [Kortex detail](docs/kortex-hardware-quickstart.md)

This public walkthrough starts from clean Windows and ends at a deliberately guarded first motion workflow. It supports a **Kinova Gen3 L53, 7 DoF, firmware 2.8.0-5** with the **Kortex 2.8 / `kortex_api` 2.8.0.post5** stack, plus a PICO 4 or PICO 4 Ultra left controller. The default and recommended path is MuJoCo.

> **Safety boundary.** This is not safety-rated, production-safe, unattended, or universally hardware-validated. Keep a reachable physical E-stop/Web Stop and a second observer for first motion. A returned Stop RPC is not proof of physical stationarity. The public first motion is translation-only: no vision control and no gripper writes. The final expanded/asymmetric profile is **offline verified and not hardware-validated**; do not claim otherwise.

## What is connected

`PICO left controller → Unity OpenXR Bridge → UDP 15031 discovery → Python admission/Grip clutch → MuJoCo (default) or explicitly gated Kortex Cartesian Twist`.

Grip is a clutch: release below `0.8`, then press above `0.9` to create a new anchor. In guarded hardware mode, control is 40 Hz, stale timeout is at most `0.2 s`, angular cap is `2 deg/s`, and your selected linear cap must not exceed `0.02 m/s`. A stale, invalid, changed-source, watchdog, workspace, or RPC fault latches Stop; resolve it onsite before another run.

## Before you begin

Use native Windows 10/11 PowerShell 5.1+ (not WSL), Git, CPython **3.11.x**, and Android platform tools/ADB for USB installation. To build the required APK locally, install Unity `2022.3.62f3c1` with Android Build Support, SDK/NDK, and OpenJDK. The bridge project obtains PICO OpenXR SDK commit `3aa3e62bff41df618529eeb60ff02c29a515dafe` from its [official source](https://github.com/Pico-Developer/PICO-Unity-OpenXR-SDK/tree/3aa3e62bff41df618529eeb60ff02c29a515dafe); review its [official license](https://github.com/Pico-Developer/PICO-Unity-OpenXR-SDK/blob/3aa3e62bff41df618529eeb60ff02c29a515dafe/LICENSE.md) before use.

<!-- RELEASE-ASSET-STATE: published -->

The Kortex wheel is the only binary declared in [release/public-release-assets.json](release/public-release-assets.json); bootstrap hash-checks that `v0.2.0-rc.1` asset before use. **APK intentionally not redistributed due to PICO SDK terms; build locally.** Neither the APK nor PICO SDK binaries are GitHub release assets. The PICO terms are proprietary, not an OSI-approved license.

Put the headset and PC on the same trusted LAN/VLAN. Do not set fixed PICO addresses. Permit only inbound UDP `15031` for Python on that trusted profile if Windows Firewall asks; do not add broad rules, expose the port to public networks, use `adb tcpip`, `adb connect`, or ADB reverse. The scripts never create a firewall rule.

## Ordered zero-to-run commands

Run these from a new PowerShell window. Replace only the clearly local variables with values measured on **your** robot. Do not copy another robot's workspace, calibration, report, lease, or evidence files.

```powershell
# LIFECYCLE: clone
git clone --branch codex/public-gen3-pico-hardware-teleop https://github.com/YEZQin/kinova-gen3-pico-teleop-mujoco.git kinova-gen3-pico-teleop
Set-Location .\kinova-gen3-pico-teleop
```

If you cannot use Git, download the ZIP from that same publication branch and extract it to a new short local directory. Do not copy a development workspace. The named branch/tag and release are publication targets; verify they exist before relying on them.

```powershell
# LIFECYCLE: bootstrap
.\scripts\bootstrap_public_teleop.ps1
```

Bootstrap validates the tracked release manifest, downloads and verifies only the Kortex wheel, creates `.venv-kortex`, installs the wheel before the project, pins protobuf 3.20.0, and performs import/offline checks. It never downloads or installs an APK, contacts a robot, retains credentials, or alters firewall rules.

```powershell
# LIFECYCLE: apk-install
$unityPath = 'C:\Program Files\Unity\Hub\Editor\2022.3.62f3c1\Editor\Unity.exe'
.\scripts\build_pico_udp_bridge.ps1 -UnityPath $unityPath -Install
```

This tracked script obtains the pinned SDK from official upstream through Unity, builds the APK locally, and installs only the newly built regular artifact. `-Install` requires exactly one authorized ADB device; omit it to build without touching a device. Never upload or redistribute the resulting APK unless you independently obtain redistribution permission.

Open **Kinova PICO Bridge** on the headset. If ADB cannot start it, start it from the headset library; do not change network topology to make ADB work.

```powershell
# LIFECYCLE: pico-gate
.\.venv-kortex\Scripts\python.exe -m kinova_teleop.main --input pico-udp --check-input --samples 10 --check-timeout 15
```

Move the tracked left controller and fully release Grip during this gate. It must report fresh samples and a release. This gate opens no robot connection.

```powershell
# LIFECYCLE: mujoco-finite
.\.venv-kortex\Scripts\python.exe -m kinova_teleop.main --dry-run --headless --steps 2000
```

Continue only when output contains `finite_state=true`. This deterministic check is simulation evidence, not hardware evidence.

```powershell
# LIFECYCLE: calibration
.\scripts\capture_pico_operator_calibration.ps1 -OutputPath local-config\operator-axes.json
```

Follow neutral/right/neutral/up/neutral/forward prompts with Grip released. Confirm the resulting local calibration maps right, up, and forward as instructed. It uses PICO UDP only and never connects to Kortex.

```powershell
# LIFECYCLE: t0
$robotIp = Read-Host 'Private IPv4 address of this Gen3'
$robotUser = Read-Host 'Kortex user name for this Gen3'
.\scripts\prepare_gen3_hardware.ps1 -RobotIp $robotIp -RobotUser $robotUser -Output local-config\t0.json
```

The last command uses a masked password prompt for a read-only current-session T0 observation and then clears the child credential. `admin` may be a device's user name; it is never a password. Confirm L53/7 DoF, firmware `2.8.0-5`, ready/manual-control state, and finite TCP pose. Stop here if any observation differs.

Measure and physically inspect the complete swept volume around **your measured T0 TCP pose**. Choose tight, inclusive absolute XYZ bounds that contain it; never reuse broad or somebody else's bounds.

```powershell
# LIFECYCLE: local-package
$workspaceMin = @(<your-measured-x-min-m>, <your-measured-y-min-m>, <your-measured-z-min-m>)
$workspaceMax = @(<your-measured-x-max-m>, <your-measured-y-max-m>, <your-measured-z-max-m>)
$owner = Read-Host 'Responsible operator identifier'
.\.venv-kortex\Scripts\python.exe -m kinova_teleop.public_hardware_setup package --t0 local-config\t0.json --calibration local-config\operator-axes.json --workspace-min $workspaceMin --workspace-max $workspaceMax --linear-speed 0.005 --owner $owner --output-dir local-config --workspace-clear --physical-estop-reachable --teach-pendant-stop-reachable --second-observer-present --cable-slack-checked --device-fixture-checked --speed-level-checked --workspace-bounds-checked --load-tcp-checked
```

The package generator is offline: it accepts no password and opens no Kortex connection. It creates new local artifacts only after all nine physical-check flags are true. Do not overwrite a prior package; correct the condition and use a new local checkout/configuration when necessary.

```powershell
# LIFECYCLE: offline-validation
$profile = Get-Content local-config\teleop-profile.json -Raw | ConvertFrom-Json
$profileRoot = Split-Path -Parent (Resolve-Path local-config\teleop-profile.json)
& .\.venv-kortex\Scripts\python.exe -m kinova_teleop.main --backend kortex --enable-hardware --input pico-udp --robot-ip $profile.robot_ip --robot-user $profile.robot_user --workspace-min $profile.workspace_min_m --workspace-max $profile.workspace_max_m --max-linear-speed $profile.max_linear_speed_m_s --max-angular-speed-deg 2 --translation-only --expanded-translation-envelope --responsive-translation-profile --recover-stale-input --stale-timeout 0.2 --control-hz 40 --scale 0.8 --operator-calibration (Join-Path $profileRoot $profile.calibration) --motion-lease (Join-Path $profileRoot $profile.motion_lease) --preflight-report (Join-Path $profileRoot $profile.preflight_report) --run-id $profile.run_id --lease-owner $profile.lease_owner --validate-motion-package
```

This command validates the package offline: it reads no password and makes no PICO or Kortex connection. Do not proceed to the next section unless it succeeds.

## Evidence matrix

| Evidence level | Public claim | Boundary |
| --- | --- | --- |
| Source/code-level | Source review, unit/integration contracts, manifest validation, and guarded admission logic are automated evidence. | They do not prove a headset, robot connection, physical stopping distance, or workspace is safe. |
| Simulator | The deterministic 2000-step MuJoCo finite check is simulator evidence. | It does not prove Kortex compatibility or physical arm behavior. |
| Hardware observed | A fresh local PICO gate, read-only T0, and recorded onsite trial are hardware observed only for that installation/session. | They do not generalize to another robot, code revision, or future unattended run. |
| Release final profile | The final expanded/asymmetric profile is offline verified and not hardware-validated in this release. | It requires new onsite evidence before any hardware claim. |

## Onsite first motion: nine checks, launch, clutch, Stop

Before the launcher may proceed, physically verify all nine conditions represented by the package command: clear workspace; E-stop reachable; teach-pendant/Web Stop reachable; second observer; cable slack; device/fixture; speed level; inspected bounds; load/TCP. Recheck them after T0 and after any interruption. Position the arm safely, keep Grip released, and test only one translation axis at a time at millimetre scale.

<!-- LIFECYCLE: physical-checklist -->

### Operator checklist before launching

<!-- LAUNCHER-OPERATOR-SEQUENCE: grip-release -> HARDWARE-READY -> masked-password -> MOVE -> grip-clutch -> stop -->

1. Confirm all nine onsite checks above and keep the physical E-stop/Web Stop reachable.
2. Keep PICO Grip fully released (`< 0.8`) while the launcher validates the package and PICO input.
3. In the launcher, type `HARDWARE-READY` only after those checks still hold; enter the masked password when prompted.
4. In the Python process, type exactly `MOVE`; only then press Grip (`> 0.9`) once to clutch and anchor.
5. Stop by releasing Grip. For unexpected motion, use `Ctrl+C`, Web Stop, or the physical E-stop; do not relaunch a latched fault.

```powershell
# LIFECYCLE: hardware-launcher
.\scripts\start_generated_gen3_teleop.ps1 -Profile local-config\teleop-profile.json -PythonPath .\.venv-kortex\Scripts\python.exe
```

<!-- LIFECYCLE: grip-release -->
<!-- LIFECYCLE: move -->
<!-- LIFECYCLE: stop-troubleshooting -->

The launcher sequence above is the operational instruction, not a shell command to simulate. Press Grip above `0.9` once to anchor; that first press must not jump. Move one axis a few millimetres, observe direction and speed, then release Grip. Gripper writes and vision control are intentionally unavailable in this first-run path.

For a stale-input, UDP, source-change, workspace, watchdog, or Stop failure: keep hands clear, use physical Stop when indicated, do not relaunch after a latched fault, inspect cables/network/fixture, and repeat from the applicable gate. For PICO failure check headset app foreground state, left-controller tracking, trusted LAN/VLAN, VPN/AP isolation, and the narrow UDP 15031 rule. For Kortex failure, do not change firmware or SDK ad hoc: confirm the required wheel/firmware pair and repeat a fresh read-only T0.

## Evidence, licenses, and development

Read [docs/evidence-levels.md](docs/evidence-levels.md) before interpreting any result. It distinguishes automated/offline, installation-local PICO, read-only T0, and onsite observed evidence. The final expanded/asymmetric profile remains offline verified and not hardware-validated.

Project-owned source is MIT licensed ([LICENSE](LICENSE)). Kortex redistribution notices, the exact upstream identity, and PICO/Unity notice references are in [release/THIRD_PARTY_NOTICES.txt](release/THIRD_PARTY_NOTICES.txt). Do not redistribute SDKs, APKs, firmware, logs, private device data, or dependencies without checking their licenses.

For development-only checks (no hardware launcher):

```powershell
.\.venv-kortex\Scripts\python.exe -m pytest -q
.\.venv-kortex\Scripts\python.exe -m kinova_teleop.main --dry-run --headless --steps 2000
git diff --check
```

Quick recap: clone → bootstrap the wheel → build/install the APK locally → open it → PICO gate → finite MuJoCo → calibrate → read-only T0 → measure/package locally → offline validate → nine physical checks → guarded launcher → release/press Grip → exact `MOVE` → Stop and investigate every fault.
