# PICO ADB Transport Failover Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a tested Windows connection supervisor that tunnels XRoboToolkit ports through USB or wireless ADB, automatically survives USB removal, and launches the existing WSL/MuJoCo teleoperation workflow.

**Architecture:** A pure Python ADB domain layer parses devices, IPs, Reverse rules, and state transitions behind an injectable command runner. A Windows CLI performs idempotent prepare/status/monitor operations, while thin PowerShell scripts configure the user PATH and launch the Windows plus WSL pieces. The WSL teleoperation loop publishes a throttled XR heartbeat so the Windows supervisor can distinguish a healthy Wi‑Fi handoff from a transport that needs one controlled PICO-app restart.

**Tech Stack:** Python 3.10+, dataclasses, `subprocess`, `argparse`, Windows PowerShell 5.1+, Android Platform Tools 36, pytest, WSL2 Ubuntu 22.04, XRoboToolkit PC Service, MuJoCo.

## Global Constraints

- ADB executable is `C:\adb\adb.exe`; append `C:\adb` only to the current Windows user's PATH and never replace existing entries.
- Target PICO package is `com.xrobotoolkit.client`; do not select `com.xrobotoolkit.pico.tcp1` or `com.yezqin.l20picobridge`.
- XRoboToolkit Reverse ports are TCP `12345` and `13579`; ADB-over-Wi‑Fi uses TCP `5555`.
- The PICO application connects only to `127.0.0.1`; runtime PICO Wi‑Fi addresses must never be hardcoded.
- WSL SDK-to-PC-Service gRPC remains TCP `60061` and is not sent through the PICO Reverse rules.
- Handoff may reconnect but must recover within 5 seconds; MuJoCo holds the last valid target while XR data is stale.
- Multiple USB Android devices are an error unless an exact serial is supplied.
- Local state and heartbeat files must be ignored by Git.
- All production behavior is preceded by a failing test.
- Existing Kinova teleoperation tests must continue to pass.

---

### Task 1: Safe Windows ADB PATH Configuration

**Files:**
- Create: `scripts/configure_adb_path.ps1`
- Create: `tests/test_windows_scripts.py`

**Interfaces:**
- Consumes: optional `-AdbDirectory`, `-PathValue`, `-DryRun`.
- Produces: one JSON object with `changed`, `adbDirectory`, and `path`; with no `-DryRun`, persists the result to the current user's `Path`.

- [ ] **Step 1: Write the failing PowerShell script tests**

```python
from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "configure_adb_path.ps1"


@pytest.mark.skipif(sys.platform != "win32", reason="PowerShell PATH test is Windows-only")
def test_adb_path_dry_run_appends_without_overwriting() -> None:
    result = subprocess.run(
        [
            "powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass",
            "-File", str(SCRIPT),
            "-DryRun",
            "-PathValue", r"C:\Tools;C:\Python",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(result.stdout)
    assert payload["changed"] is True
    assert payload["path"] == r"C:\Tools;C:\Python;C:\adb"


@pytest.mark.skipif(sys.platform != "win32", reason="PowerShell PATH test is Windows-only")
def test_adb_path_dry_run_is_case_insensitively_idempotent() -> None:
    result = subprocess.run(
        [
            "powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass",
            "-File", str(SCRIPT),
            "-DryRun",
            "-PathValue", r"C:\Tools;c:\ADB\\",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(result.stdout)
    assert payload["changed"] is False
    assert payload["path"] == r"C:\Tools;c:\ADB\\"
```

- [ ] **Step 2: Run the tests and confirm RED**

Run:

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_windows_scripts.py -q
```

Expected: both tests fail because `scripts/configure_adb_path.ps1` is absent.

- [ ] **Step 3: Implement the safe PATH script**

```powershell
[CmdletBinding()]
param(
    [string]$AdbDirectory = 'C:\adb',
    [string]$PathValue,
    [switch]$DryRun
)

$ErrorActionPreference = 'Stop'
$adbExe = Join-Path $AdbDirectory 'adb.exe'
if (-not (Test-Path -LiteralPath $adbExe -PathType Leaf)) {
    throw "adb.exe not found: $adbExe"
}

$providedPath = $PSBoundParameters.ContainsKey('PathValue')
$currentPath = if ($providedPath) {
    $PathValue
} else {
    [Environment]::GetEnvironmentVariable('Path', 'User')
}
if ($null -eq $currentPath) {
    $currentPath = ''
}

$normalizedAdb = $AdbDirectory.TrimEnd('\')
$entries = @($currentPath -split ';' | Where-Object { $_ -ne '' })
$containsAdb = $false
foreach ($entry in $entries) {
    if ($entry.TrimEnd('\').Equals(
        $normalizedAdb,
        [System.StringComparison]::OrdinalIgnoreCase
    )) {
        $containsAdb = $true
        break
    }
}
$newPath = if ($containsAdb) {
    $currentPath
} elseif ([string]::IsNullOrWhiteSpace($currentPath)) {
    $AdbDirectory
} else {
    "$($currentPath.TrimEnd(';'));$AdbDirectory"
}

if (-not $DryRun) {
    [Environment]::SetEnvironmentVariable('Path', $newPath, 'User')
    $env:Path = "$newPath;$([Environment]::GetEnvironmentVariable('Path', 'Machine'))"
}

[ordered]@{
    changed = -not $containsAdb
    adbDirectory = $AdbDirectory
    path = $newPath
} | ConvertTo-Json -Compress
```

- [ ] **Step 4: Run the tests and apply the real user PATH change**

Run:

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_windows_scripts.py -q
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\scripts\configure_adb_path.ps1
```

Expected: tests pass and JSON contains `"path":` with `C:\\adb`.

- [ ] **Step 5: Verify a fresh process resolves ADB**

Run:

```powershell
powershell.exe -NoProfile -Command `
  '$env:Path = [Environment]::GetEnvironmentVariable("Path", "User") + ";" + [Environment]::GetEnvironmentVariable("Path", "Machine"); Get-Command adb; adb version'
```

Expected: command source is `C:\adb\adb.exe` and Platform Tools version is `36.0.2-14143358`.

- [ ] **Step 6: Commit**

```powershell
git add scripts/configure_adb_path.ps1 tests/test_windows_scripts.py
git commit -m "build: configure ADB user path safely"
```

---

### Task 2: ADB Parsing, Device Selection, and Transport State

**Files:**
- Create: `kinova_teleop/adb_transport.py`
- Create: `tests/test_adb_transport.py`

**Interfaces:**
- Produces:
  - `AdbDevice(serial: str, state: str, attributes: Mapping[str, str])`
  - `TransportPhase` enum
  - `TransportSnapshot`
  - `parse_adb_devices(text: str) -> tuple[AdbDevice, ...]`
  - `parse_wlan_ipv4(text: str) -> str | None`
  - `parse_reverse_list(text: str) -> frozenset[tuple[str, str]]`
  - `select_usb_device(devices, requested_serial=None) -> AdbDevice | None`
  - `classify_snapshot(*, usb_online: bool, wifi_online: bool, usb_rules_complete: bool, wifi_rules_complete: bool, unauthorized: bool) -> TransportPhase`

- [ ] **Step 1: Write failing parser and selection tests**

```python
from __future__ import annotations

import pytest

from kinova_teleop.adb_transport import (
    TransportPhase,
    classify_snapshot,
    parse_adb_devices,
    parse_reverse_list,
    parse_wlan_ipv4,
    select_usb_device,
)


DEVICES = """List of devices attached
PA921BMGK7300311G device product:sparrow model:A9210 transport_id:1
192.168.1.4:5555 device product:sparrow model:A9210 transport_id:2
"""


def test_parse_devices_classifies_usb_and_network_serials() -> None:
    devices = parse_adb_devices(DEVICES)
    assert devices[0].is_network is False
    assert devices[1].is_network is True
    assert devices[1].host == "192.168.1.4"


def test_parse_wlan_ipv4_ignores_loopback_and_broadcast() -> None:
    text = """15: wlan0: <UP>
        inet 192.168.1.4/24 brd 192.168.1.255 scope global wlan0
    """
    assert parse_wlan_ipv4(text) == "192.168.1.4"


def test_parse_reverse_rules() -> None:
    rules = parse_reverse_list(
        "192.168.1.4:5555 tcp:12345 tcp:12345\n"
        "192.168.1.4:5555 tcp:13579 tcp:13579\n"
    )
    assert ("tcp:12345", "tcp:12345") in rules
    assert ("tcp:13579", "tcp:13579") in rules


def test_multiple_usb_devices_require_exact_serial() -> None:
    devices = parse_adb_devices(
        "List of devices attached\nAAA device\nBBB device\n"
    )
    with pytest.raises(ValueError, match="Multiple USB"):
        select_usb_device(devices)
    assert select_usb_device(devices, "BBB").serial == "BBB"


def test_phase_is_dual_ready_only_when_both_transports_have_rules() -> None:
    phase = classify_snapshot(
        usb_online=True,
        wifi_online=True,
        usb_rules_complete=True,
        wifi_rules_complete=True,
        unauthorized=False,
    )
    assert phase is TransportPhase.DUAL_READY
```

- [ ] **Step 2: Run the tests and confirm RED**

Run:

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_adb_transport.py -q
```

Expected: import fails because `kinova_teleop.adb_transport` does not exist.

- [ ] **Step 3: Implement immutable device and state types**

```python
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import ipaddress
import re
from typing import Mapping, Sequence


REVERSE_PORTS = (12345, 13579)
ADB_TCP_PORT = 5555
PICO_PACKAGE = "com.xrobotoolkit.client"
PICO_ACTIVITY = "com.unity3d.player.UnityPlayerActivity"


@dataclass(frozen=True)
class AdbDevice:
    serial: str
    state: str
    attributes: Mapping[str, str]

    @property
    def is_network(self) -> bool:
        return bool(re.fullmatch(r"\d+\.\d+\.\d+\.\d+:\d+", self.serial))

    @property
    def host(self) -> str | None:
        return self.serial.rsplit(":", 1)[0] if self.is_network else None


class TransportPhase(str, Enum):
    NO_DEVICE = "NO_DEVICE"
    USB_ONLY = "USB_ONLY"
    DUAL_READY = "DUAL_READY"
    WIFI_ONLY = "WIFI_ONLY"
    DEGRADED = "DEGRADED"
    UNAUTHORIZED = "UNAUTHORIZED"
    AMBIGUOUS = "AMBIGUOUS"


@dataclass(frozen=True)
class TransportSnapshot:
    phase: TransportPhase
    usb_serial: str | None
    wifi_serial: str | None
    pico_ip: str | None
    usb_rules_complete: bool
    wifi_rules_complete: bool
    message: str
```

- [ ] **Step 4: Implement parsing and deterministic classification**

```python
def parse_adb_devices(text: str) -> tuple[AdbDevice, ...]:
    devices = []
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("List of devices attached") or line.startswith("*"):
            continue
        fields = line.split()
        if len(fields) < 2:
            continue
        attributes = {}
        for field in fields[2:]:
            if ":" in field:
                key, value = field.split(":", 1)
                attributes[key] = value
        devices.append(AdbDevice(fields[0], fields[1], attributes))
    return tuple(devices)


def parse_wlan_ipv4(text: str) -> str | None:
    for match in re.finditer(r"\binet\s+(\d+\.\d+\.\d+\.\d+)/\d+", text):
        address = ipaddress.ip_address(match.group(1))
        if address.version == 4 and not address.is_loopback:
            return str(address)
    return None


def parse_reverse_list(text: str) -> frozenset[tuple[str, str]]:
    rules = set()
    for line in text.splitlines():
        fields = line.split()
        if len(fields) >= 2:
            rules.add((fields[-2], fields[-1]))
    return frozenset(rules)


def select_usb_device(
    devices: Sequence[AdbDevice],
    requested_serial: str | None = None,
) -> AdbDevice | None:
    candidates = [device for device in devices if not device.is_network]
    if requested_serial is not None:
        matches = [device for device in candidates if device.serial == requested_serial]
        if len(matches) != 1:
            raise ValueError(f"USB device not found: {requested_serial}")
        return matches[0]
    if len(candidates) > 1:
        raise ValueError("Multiple USB devices detected; pass --serial")
    return candidates[0] if candidates else None
```

Implement `classify_snapshot` with the exact precedence:

1. unauthorized USB → `UNAUTHORIZED`;
2. no online transport → `NO_DEVICE`;
3. both online and both rule sets complete → `DUAL_READY`;
4. USB only and its rules complete → `USB_ONLY`;
5. Wi‑Fi only and its rules complete → `WIFI_ONLY`;
6. otherwise → `DEGRADED`.

- [ ] **Step 5: Run tests and commit**

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_adb_transport.py -q
git add kinova_teleop/adb_transport.py tests/test_adb_transport.py
git commit -m "feat: model PICO ADB transport state"
```

---

### Task 3: Idempotent ADB Prepare and Monitor Manager

**Files:**
- Modify: `kinova_teleop/adb_transport.py`
- Modify: `tests/test_adb_transport.py`

**Interfaces:**
- Consumes: `CommandRunner.run(args: Sequence[str], timeout: float) -> CommandResult`.
- Produces:
  - `CommandResult(returncode: int, stdout: str, stderr: str)`
  - `SubprocessRunner`
  - `AdbTransportManager.inspect() -> TransportSnapshot`
  - `AdbTransportManager.prepare(requested_serial=None) -> TransportSnapshot`
  - `AdbTransportManager.reconcile(requested_serial=None, last_ip=None) -> TransportSnapshot`
  - `AdbTransportManager.ensure_pico_app(serial: str, restart: bool=False) -> None`

- [ ] **Step 1: Add failing fake-runner manager tests**

```python
class FakeRunner:
    def __init__(self, responses):
        self.responses = list(responses)
        self.calls = []

    def run(self, args, timeout=10.0):
        self.calls.append(tuple(args))
        if not self.responses:
            raise AssertionError(f"Unexpected command: {args}")
        return self.responses.pop(0)


def result(stdout="", returncode=0, stderr=""):
    return CommandResult(returncode, stdout, stderr)


def test_prepare_discovers_ip_connects_wifi_and_reverses_both_ports() -> None:
    runner = FakeRunner([
        result(DEVICES.split("192.168", 1)[0]),
        result("package:/data/app/base.apk\n"),
        result("inet 192.168.1.4/24 scope global wlan0\n"),
        result("restarting in TCP mode port: 5555\n"),
        result("connected to 192.168.1.4:5555\n"),
        result(DEVICES),
        result(), result(), result(), result(),
        result("PA921BMGK7300311G tcp:12345 tcp:12345\n"
               "PA921BMGK7300311G tcp:13579 tcp:13579\n"),
        result("192.168.1.4:5555 tcp:12345 tcp:12345\n"
               "192.168.1.4:5555 tcp:13579 tcp:13579\n"),
    ])
    manager = AdbTransportManager(runner, adb_executable="adb")
    snapshot = manager.prepare()
    assert snapshot.phase is TransportPhase.DUAL_READY
    assert ("adb", "-s", "192.168.1.4:5555", "reverse",
            "tcp:12345", "tcp:12345") in runner.calls
    assert ("adb", "-s", "192.168.1.4:5555", "reverse",
            "tcp:13579", "tcp:13579") in runner.calls


def test_reconcile_wifi_only_reapplies_missing_rule_without_tcpip() -> None:
    runner = FakeRunner([
        result("List of devices attached\n"
               "192.168.1.4:5555 device transport_id:2\n"),
        result("192.168.1.4:5555 tcp:12345 tcp:12345\n"),
        result(),
        result("List of devices attached\n"
               "192.168.1.4:5555 device transport_id:2\n"),
        result("192.168.1.4:5555 tcp:12345 tcp:12345\n"
               "192.168.1.4:5555 tcp:13579 tcp:13579\n"),
    ])
    manager = AdbTransportManager(runner, adb_executable="adb")
    snapshot = manager.reconcile()
    assert snapshot.phase is TransportPhase.WIFI_ONLY
    assert not any("tcpip" in call for call in runner.calls)


def test_prepare_rejects_unauthorized_usb_before_mutating() -> None:
    runner = FakeRunner([
        result("List of devices attached\nPA921 unauthorized transport_id:1\n")
    ])
    manager = AdbTransportManager(runner, adb_executable="adb")
    with pytest.raises(RuntimeError, match="RSA"):
        manager.prepare()
    assert len(runner.calls) == 1
```

- [ ] **Step 2: Run manager tests and confirm RED**

Run:

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_adb_transport.py -q
```

Expected: collection fails on missing `CommandResult` or `AdbTransportManager`.

- [ ] **Step 3: Implement runner and checked ADB commands**

```python
from pathlib import Path
import subprocess
from typing import Protocol


@dataclass(frozen=True)
class CommandResult:
    returncode: int
    stdout: str
    stderr: str


class CommandRunner(Protocol):
    def run(self, args: Sequence[str], timeout: float = 10.0) -> CommandResult:
        raise NotImplementedError


class SubprocessRunner:
    def run(self, args: Sequence[str], timeout: float = 10.0) -> CommandResult:
        completed = subprocess.run(
            list(args),
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
        return CommandResult(
            completed.returncode,
            completed.stdout,
            completed.stderr,
        )
```

`AdbTransportManager._run_adb` must prepend the configured executable, raise an actionable `RuntimeError` on nonzero status, and include stderr without exposing unrelated environment variables.

- [ ] **Step 4: Implement prepare, inspect, rule repair, and package control**

Use these exact command shapes:

```python
("devices", "-l")
("-s", usb_serial, "shell", "pm", "path", PICO_PACKAGE)
("-s", usb_serial, "shell", "ip", "-4", "addr", "show", "wlan0")
("-s", usb_serial, "tcpip", "5555")
("connect", f"{pico_ip}:5555")
("-s", transport_serial, "reverse", f"tcp:{port}", f"tcp:{port}")
("-s", transport_serial, "reverse", "--list")
("-s", transport_serial, "shell", "am", "force-stop", PICO_PACKAGE)
("-s", transport_serial, "shell", "am", "start", "-n",
 f"{PICO_PACKAGE}/{PICO_ACTIVITY}")
```

Implementation rules:

- `prepare` requires a usable USB device and verifies the official package before changing adbd.
- Retry `adb devices -l` for up to 5 seconds after `tcpip`.
- Treat `adb connect` output containing `connected to` or `already connected` as success.
- Add Reverse rules unconditionally; the operation is idempotent.
- `inspect` reads each transport's own `reverse --list`.
- `reconcile` never runs `tcpip` without USB; it reconnects the last known IP if available and repairs missing rules.
- `ensure_pico_app(restart=False)` starts the activity only if `pidof` is empty.
- `ensure_pico_app(restart=True)` performs exactly one force-stop/start sequence.

- [ ] **Step 5: Run the manager tests and the complete existing suite**

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_adb_transport.py -q
.\.venv\Scripts\python.exe -m pytest -q
```

Expected: manager tests and all existing tests pass.

- [ ] **Step 6: Commit**

```powershell
git add kinova_teleop/adb_transport.py tests/test_adb_transport.py
git commit -m "feat: manage USB and wireless ADB transports"
```

---

### Task 4: ADB CLI, Local State, and Continuous Monitor

**Files:**
- Create: `kinova_teleop/adb_cli.py`
- Create: `tests/test_adb_cli.py`
- Modify: `.gitignore`
- Modify: `pyproject.toml`

**Interfaces:**
- Produces:
  - `build_parser() -> argparse.ArgumentParser`
  - `main(argv: Sequence[str] | None = None) -> int`
  - subcommands `prepare`, `status`, `monitor`
- Local state schema:

```json
{
  "usb_serial": "PA921BMGK7300311G",
  "pico_ip": "192.168.1.4",
  "wifi_serial": "192.168.1.4:5555",
  "updated_at": 0.0
}
```

- [ ] **Step 1: Write failing CLI tests**

```python
from kinova_teleop.adb_cli import main


DUAL_READY_SNAPSHOT = TransportSnapshot(
    phase=TransportPhase.DUAL_READY,
    usb_serial="USB1",
    wifi_serial="192.168.1.4:5555",
    pico_ip="192.168.1.4",
    usb_rules_complete=True,
    wifi_rules_complete=True,
    message="ready",
)
WIFI_ONLY_SNAPSHOT = TransportSnapshot(
    phase=TransportPhase.WIFI_ONLY,
    usb_serial=None,
    wifi_serial="192.168.1.4:5555",
    pico_ip="192.168.1.4",
    usb_rules_complete=False,
    wifi_rules_complete=True,
    message="wifi ready",
)


class FakeManager:
    def __init__(self, snapshot):
        self.snapshot = snapshot
        self.reconcile_calls = 0

    def inspect(self):
        return self.snapshot

    def prepare(self, requested_serial=None):
        return self.snapshot

    def reconcile(self, requested_serial=None, last_ip=None):
        self.reconcile_calls += 1
        return self.snapshot


class RaisingManager(FakeManager):
    def __init__(self, error):
        self.error = error

    def prepare(self, requested_serial=None):
        raise self.error


def test_status_json_uses_injected_manager(monkeypatch, capsys) -> None:
    monkeypatch.setattr(
        "kinova_teleop.adb_cli.make_manager",
        lambda _args: FakeManager(DUAL_READY_SNAPSHOT),
    )
    assert main(["status", "--json"]) == 0
    assert '"phase": "DUAL_READY"' in capsys.readouterr().out


def test_prepare_returns_two_for_unauthorized(monkeypatch, capsys) -> None:
    monkeypatch.setattr(
        "kinova_teleop.adb_cli.make_manager",
        lambda _args: RaisingManager(RuntimeError("confirm PICO RSA")),
    )
    assert main(["prepare"]) == 2
    assert "RSA" in capsys.readouterr().err


def test_monitor_once_reconciles_and_saves_state(monkeypatch, tmp_path) -> None:
    manager = FakeManager(WIFI_ONLY_SNAPSHOT)
    monkeypatch.setattr("kinova_teleop.adb_cli.make_manager", lambda _args: manager)
    state = tmp_path / "state.json"
    assert main(["--state-file", str(state), "monitor", "--once"]) == 0
    assert manager.reconcile_calls == 1
    assert json.loads(state.read_text())["pico_ip"] == "192.168.1.4"
```

- [ ] **Step 2: Run CLI tests and confirm RED**

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_adb_cli.py -q
```

Expected: import fails because `kinova_teleop.adb_cli` is absent.

- [ ] **Step 3: Implement CLI and atomic local state**

Parser:

```python
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Prepare PICO XR ADB failover")
    parser.add_argument("--adb", default=None)
    parser.add_argument("--serial")
    parser.add_argument(
        "--state-file",
        type=Path,
        default=Path(".pico-adb-state.json"),
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    for name in ("prepare", "status"):
        command = subparsers.add_parser(name)
        command.add_argument("--json", action="store_true")
    monitor = subparsers.add_parser("monitor")
    monitor.add_argument("--interval", type=float, default=1.0)
    monitor.add_argument("--once", action="store_true")
    monitor.add_argument("--json", action="store_true")
    return parser
```

Resolve ADB in this order:

1. explicit `--adb`;
2. `shutil.which("adb")`;
3. `C:\adb\adb.exe` on Windows;
4. actionable error.

Write state atomically using `Path.with_suffix(".tmp")` plus `replace`. Reject monitor intervals below `0.1`.

Monitor behavior:

- load last IP from state;
- call `reconcile`;
- save successful snapshots;
- print only phase changes unless `--json`;
- on `DUAL_READY → WIFI_ONLY`, record the transition time for Task 5 health handling;
- catch Ctrl+C and return `0`;
- connection errors print once per changed error message and continue.

- [ ] **Step 4: Add packaging and ignore rules**

Add:

```toml
[project.scripts]
kinova-pico-teleop = "kinova_teleop.main:main"
kinova-pico-adb = "kinova_teleop.adb_cli:main"
```

Append to `.gitignore`:

```gitignore
.pico-adb-state.json
.pico-xr-health.json
.pico-adb-monitor.pid
```

- [ ] **Step 5: Run CLI tests, help, and commit**

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_adb_cli.py -q
.\.venv\Scripts\python.exe -m kinova_teleop.adb_cli --help
git add .gitignore pyproject.toml kinova_teleop/adb_cli.py tests/test_adb_cli.py
git commit -m "feat: add PICO ADB supervisor CLI"
```

---

### Task 5: XR Heartbeat and Controlled Post-Handoff Restart

**Files:**
- Create: `kinova_teleop/xr_health.py`
- Create: `tests/test_xr_health.py`
- Modify: `kinova_teleop/teleop_controller.py`
- Modify: `kinova_teleop/main.py`
- Modify: `kinova_teleop/adb_cli.py`
- Modify: `tests/test_teleop_controller.py`
- Modify: `tests/test_adb_cli.py`

**Interfaces:**
- Produces:
  - `XrHealthReporter(path: Path, minimum_interval: float = 0.2)`
  - `XrHealthReporter.update(timestamp_ns: int, received_monotonic: float) -> None`
  - `read_xr_health(path: Path) -> XrHealth | None`
- Adds `TeleopConfig.health_file: Path | None = None`.
- Adds teleop CLI `--health-file PATH`.
- Adds supervisor monitor options `--health-file PATH` and `--handoff-timeout 5.0`.
- Adds `run_monitor(manager, *, state_file: Path, health_file: Path | None, handoff_timeout: float, interval: float, max_iterations: int | None = None, monotonic=time.monotonic, sleeper=time.sleep) -> int`.

- [ ] **Step 1: Write failing heartbeat tests**

```python
def test_reporter_writes_latest_timestamp_atomically(tmp_path) -> None:
    path = tmp_path / "health.json"
    reporter = XrHealthReporter(path, minimum_interval=0.0)
    reporter.update(123, 4.5)
    health = read_xr_health(path)
    assert health.timestamp_ns == 123
    assert health.received_monotonic == pytest.approx(4.5)


def test_reporter_throttles_disk_writes(tmp_path) -> None:
    path = tmp_path / "health.json"
    reporter = XrHealthReporter(path, minimum_interval=0.2)
    reporter.update(1, 1.0)
    reporter.update(2, 1.1)
    assert read_xr_health(path).timestamp_ns == 1
    reporter.update(3, 1.21)
    assert read_xr_health(path).timestamp_ns == 3


def test_controller_reports_valid_xr_timestamp(teleop_model_path, tmp_path) -> None:
    health_file = tmp_path / "health.json"
    controller = TeleopController(
        TeleopConfig(
            model_path=teleop_model_path,
            realtime=False,
            health_file=health_file,
        ),
        ScriptedInput([sample([0, 0, 0], 1.0, 7, 1.0)]),
    )
    try:
        controller.step_once()
        assert read_xr_health(health_file).timestamp_ns == 7
    finally:
        controller.close()
```

- [ ] **Step 2: Run heartbeat tests and confirm RED**

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_xr_health.py tests/test_teleop_controller.py -q
```

Expected: missing `xr_health` module and `TeleopConfig.health_file`.

- [ ] **Step 3: Implement throttled atomic heartbeat**

```python
@dataclass(frozen=True)
class XrHealth:
    timestamp_ns: int
    received_monotonic: float
    written_wall_time: float


class XrHealthReporter:
    def __init__(self, path: Path, minimum_interval: float = 0.2):
        self.path = Path(path)
        self.minimum_interval = minimum_interval
        self._last_write_monotonic: float | None = None

    def update(self, timestamp_ns: int, received_monotonic: float) -> None:
        if (
            self._last_write_monotonic is not None
            and received_monotonic - self._last_write_monotonic
            < self.minimum_interval
        ):
            return
        payload = {
            "timestamp_ns": int(timestamp_ns),
            "received_monotonic": float(received_monotonic),
            "written_wall_time": time.time(),
        }
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text(json.dumps(payload), encoding="utf-8")
        temporary.replace(self.path)
        self._last_write_monotonic = received_monotonic
```

`read_xr_health` returns `None` for a missing, malformed, non-finite, or negative-timestamp file.

- [ ] **Step 4: Integrate heartbeat into the teleoperation loop**

In `TeleopController.__init__`, create a reporter only when `health_file` is not `None`. In `step_once`, after `sample = self.source.read()`, call reporter only when `sample.valid` and `timestamp_ns > 0`.

Add CLI:

```python
parser.add_argument(
    "--health-file",
    type=Path,
    help="write throttled XR timestamp health for the Windows ADB supervisor",
)
```

Pass it into `TeleopConfig`.

- [ ] **Step 5: Add a failing monitor handoff test and implement one restart**

Test exact behavior:

```python
class SequenceManager:
    def __init__(self, snapshots):
        self.snapshots = iter(snapshots)
        self.restart_calls = []

    def reconcile(self, requested_serial=None, last_ip=None):
        return next(self.snapshots)

    def ensure_pico_app(self, serial, restart=False):
        if restart:
            self.restart_calls.append(serial)


class FakeClock:
    def __init__(self, values):
        self.values = iter(values)

    def __call__(self):
        return next(self.values)


def test_monitor_restarts_app_once_when_health_stalls_after_usb_removal(
    tmp_path,
) -> None:
    manager = SequenceManager([
        DUAL_READY_SNAPSHOT,
        WIFI_ONLY_SNAPSHOT,
        WIFI_ONLY_SNAPSHOT,
    ])
    clock = FakeClock([0.0, 0.1, 6.0])
    health_file = tmp_path / "health.json"
    health_file.write_text(
        json.dumps({
            "timestamp_ns": 10,
            "received_monotonic": 1.0,
            "written_wall_time": 1.0,
        }),
        encoding="utf-8",
    )
    code = run_monitor(
        manager,
        state_file=tmp_path / "state.json",
        health_file=health_file,
        handoff_timeout=5.0,
        interval=0.0,
        max_iterations=3,
        monotonic=clock,
        sleeper=lambda _: None,
    )
    assert code == 0
    assert manager.restart_calls == ["192.168.1.4:5555"]
```

Implementation rules:

- A restart is eligible only after an observed `DUAL_READY → WIFI_ONLY`.
- Snapshot the heartbeat timestamp at transition.
- If the timestamp advances, clear the pending restart.
- At timeout, restart the official app exactly once on the wireless serial.
- Never restart repeatedly for the same transition.
- If the wireless transport disappears, cancel the pending restart.

- [ ] **Step 6: Run full tests and commit**

```powershell
.\.venv\Scripts\python.exe -m pytest -q
git add kinova_teleop/xr_health.py kinova_teleop/teleop_controller.py `
  kinova_teleop/main.py kinova_teleop/adb_cli.py tests
git commit -m "feat: recover XR data after ADB handoff"
```

---

### Task 6: Windows Supervisor and One-Command Teleoperation Scripts

**Files:**
- Create: `scripts/pico_adb_transport.ps1`
- Create: `scripts/start_pico_teleop.ps1`
- Modify: `tests/test_windows_scripts.py`

**Interfaces:**
- `pico_adb_transport.ps1` supports `-Prepare`, `-Status`, `-Monitor`, `-Serial`, and `-Json`.
- `start_pico_teleop.ps1` supports `-Serial`, `-Scale`, `-SkipXrCheck`, and `-HeadlessSteps`.

- [ ] **Step 1: Add failing dry-run wrapper tests**

```python
@pytest.mark.skipif(sys.platform != "win32", reason="PowerShell wrapper test")
@pytest.mark.parametrize(
    ("script_name", "expected"),
    [
        ("pico_adb_transport.ps1", "kinova_teleop.adb_cli"),
        ("start_pico_teleop.ps1", "kinova_teleop.main"),
    ],
)
def test_windows_wrapper_dry_run_shows_commands(script_name, expected) -> None:
    result = subprocess.run(
        [
            "powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass",
            "-File", str(ROOT / "scripts" / script_name),
            "-DryRun",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    assert expected in result.stdout
```

- [ ] **Step 2: Run wrapper tests and confirm RED**

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_windows_scripts.py -q
```

Expected: wrapper files are absent.

- [ ] **Step 3: Implement the thin transport wrapper**

The script must:

- resolve repository root from `$PSScriptRoot`;
- use `.\.venv\Scripts\python.exe` when present, otherwise `python`;
- map exactly one mode to the corresponding Python subcommand;
- forward `-Serial`;
- use `Start-Process -WindowStyle Hidden` only when the caller explicitly requests background monitor operation;
- print the exact command and exit without executing it for `-DryRun`.

Core command construction:

```powershell
$arguments = @('-m', 'kinova_teleop.adb_cli')
if ($Serial) {
    $arguments += @('--serial', $Serial)
}
$arguments += $mode
if ($Json) {
    $arguments += '--json'
}
```

- [ ] **Step 4: Implement one-command teleoperation startup**

The startup script must:

1. run `adb_cli prepare`;
2. require returned phase `DUAL_READY`;
3. start `adb_cli monitor` hidden and write its PID to `.pico-adb-monitor.pid`;
4. test Windows `127.0.0.1:60061`;
5. derive the WSL Windows gateway using `ip route show default`;
6. write `PXREASetting.ini`;
7. run WSL `--check-xr --samples 20` unless skipped;
8. launch WSL teleop with:

```bash
python -m kinova_teleop.main \
  --model kinova_gen3_mujoco/teleop_scene.xml \
  --scale <value> \
  --health-file /mnt/d/yezq/26.7.24_Kinova_controll/.pico-xr-health.json
```

For `-HeadlessSteps`, append `--dry-run --headless --steps <N>` and do not require PC Service.

The WSL command must activate `~/.venvs/kinova-pico-verify` and export the documented XR SDK `LD_LIBRARY_PATH`.

- [ ] **Step 5: Run wrapper tests and dry-run output**

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_windows_scripts.py -q
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\scripts\pico_adb_transport.ps1 -DryRun
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\scripts\start_pico_teleop.ps1 -DryRun
```

Expected: tests pass and both commands print without changing ADB state.

- [ ] **Step 6: Commit**

```powershell
git add scripts/pico_adb_transport.ps1 scripts/start_pico_teleop.ps1 `
  tests/test_windows_scripts.py
git commit -m "feat: add Windows PICO teleop launcher"
```

---

### Task 7: Documentation, Real Device Validation, and Publication

**Files:**
- Modify: `README.md`
- Create: `docs/pico-wired-wifi-quickstart.md`
- Modify only if validation exposes a tested defect.

**Interfaces:**
- Documents the one-time official client setting and the daily one-command flow.

- [ ] **Step 1: Write the focused local quick-start**

Document the detected local facts:

- PICO A9210 and USB serial `PA921BMGK7300311G`;
- official package `com.xrobotoolkit.client`;
- official client address `127.0.0.1`, data `12345`, command `13579`;
- PC Service must run on Windows;
- one-time PATH command;
- daily command:

```powershell
.\scripts\start_pico_teleop.ps1 -Scale 0.5
```

- transport status:

```powershell
.\scripts\pico_adb_transport.ps1 -Status
```

- [ ] **Step 2: Run complete automated verification**

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".[dev]"
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe -m kinova_teleop.main `
  --dry-run --headless --steps 5000
wsl.exe -d Ubuntu-22.04 bash -lc `
  "source ~/.venvs/kinova-pico-verify/bin/activate && \
   cd /mnt/d/yezq/26.7.24_Kinova_controll && \
   python -m pytest -q && \
   python -m kinova_teleop.main --dry-run --headless --steps 2000"
```

Expected: zero failures and both dry runs report `finite_state=true`.

- [ ] **Step 3: Validate the real USB preparation**

With USB connected and RSA authorized:

```powershell
adb devices -l
.\scripts\pico_adb_transport.ps1 -Prepare -Serial PA921BMGK7300311G
.\scripts\pico_adb_transport.ps1 -Status -Json
adb -s PA921BMGK7300311G reverse --list
```

Expected:

- USB and `<discovered-ip>:5555` are both `device`;
- phase is `DUAL_READY`;
- both transports contain `tcp:12345` and `tcp:13579`.

- [ ] **Step 4: Validate USB removal and Wi‑Fi takeover**

Start monitor:

```powershell
.\scripts\pico_adb_transport.ps1 -Monitor -Serial PA921BMGK7300311G
```

Unplug USB, then in another PowerShell:

```powershell
adb devices -l
.\scripts\pico_adb_transport.ps1 -Status -Json
adb -s 192.168.1.4:5555 reverse --list
```

Use the discovered IP printed by status rather than assuming `192.168.1.4`.

Expected:

- only wireless transport remains;
- phase becomes `WIFI_ONLY`;
- both Reverse rules remain;
- after PC Service and official client are active, XR heartbeat advances again within 5 seconds.

- [ ] **Step 5: Commit documentation and any tested corrections**

```powershell
git add README.md docs/pico-wired-wifi-quickstart.md
git commit -m "docs: add wired-to-WiFi PICO quick start"
```

- [ ] **Step 6: Audit and push**

```powershell
git status -sb
git diff --check
git ls-files | Select-String -Pattern `
  'pico-adb-state|pico-xr-health|PXREASetting.ini|setting.ini'
git push origin main
gh repo view YEZQin/kinova-gen3-pico-teleop-mujoco `
  --json visibility,url,defaultBranchRef
```

Expected:

- working tree clean;
- no runtime state/config file tracked;
- private GitHub repository default branch remains `main`;
- remote `main` matches local `HEAD`.
