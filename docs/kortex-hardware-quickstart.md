# Gen3 Kortex first-hardware details

Use this alongside the ordered [root README](../README.md), not as a shortcut around its gates. The supported public target is a Gen3 L53 7 DoF with firmware `2.8.0-5`, Kortex 2.8, `kortex_api` `2.8.0.post5`, CPython 3.11, and protobuf 3.20.0.

The only public first-motion route is `scripts/start_generated_gen3_teleop.ps1` with a locally generated `teleop-profile.json`. It is translation-only, 40 Hz, stale timeout `0.2 s`, angular cap `2 deg/s`, and a locally selected linear speed no greater than `0.02 m/s`. It does not command a gripper or vision system.

First collect a fresh read-only T0 through `scripts/prepare_gen3_hardware.ps1`, then measure the TCP and inspect the full swept volume. The package CLI requires all nine physical flags and writes new local artifacts; package validation is offline and does not read a password or connect to Kortex. The guarded launcher repeats package validation and PICO admission before masked credential entry and the program's `MOVE` confirmation.

## 2. T0: current read-only evidence

The root README uses the preferred wrapper. The equivalent explicit boundary below shows that the startup-hook gate precedes the masked prompt. Supply only your own private IPv4 address and user name; this command writes a new local observation and performs no motion.

```powershell
& .\scripts\assert_no_untracked_python_startup_hooks.ps1 -ProjectRoot (Get-Location).Path
$securePassword = Read-Host 'Kortex password' -AsSecureString
try {
  $env:KINOVA_PASSWORD = [System.Net.NetworkCredential]::new('', $securePassword).Password
  .\.venv-kortex\Scripts\python.exe -m kinova_teleop.main --backend kortex --enable-hardware --check-kortex --robot-ip <your-private-ipv4> --robot-user <your-kortex-user> --preflight-json local-config\t0-observation.json
} finally {
  Remove-Item Env:KINOVA_PASSWORD -ErrorAction SilentlyContinue
  $securePassword.Dispose()
}
```

Treat this as read-only T0 evidence only. It is not a passing motion report and cannot authorize motion by itself.

## Mandatory pause after T0

Stop after T0. Present the observation to the onsite operator, recheck all physical conditions, inspect the measured pose and tight bounds, and confirm that Grip is released before creating a local package.

Use only a trusted private IPv4 route to TCP port 10000. Never put a credential on a command line or in a profile. A Stop return is not proof of physical stationarity. Keep a physical E-stop/Web Stop and a second observer. After Grip release, stale input, any source change, watchdog, workspace, RPC, or unexpected motion, use the onsite safety process; do not relaunch a latched fault.

Evidence terminology and the explicit limitation for the final expanded/asymmetric profile are in [evidence levels](evidence-levels.md). That profile is offline verified and not hardware-validated.
