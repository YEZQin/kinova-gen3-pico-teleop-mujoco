"""Contracts for the repeatable, hardware-observed Gen3/PICO preset launcher."""

from __future__ import annotations

import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = ROOT / "scripts" / "start_gen3_pico_tuned_session.ps1"


def _ps_literal(value: Path | str) -> str:
    return "'" + str(value).replace("'", "''") + "'"


def test_tuned_launcher_exposes_the_hardware_observed_defaults() -> None:
    source = LAUNCHER.read_text(encoding="utf-8")

    for contract in (
        "[double]$Scale = 1.0",
        "[double]$MaxLinearSpeed = 0.05",
        "[double]$LinearGain = 1.5",
        "[double[]]$TranslationAxisGain = @(-2.0, 1.0, 1.0)",
        "[double]$GripperBinaryThreshold = 0.9",
        "-EnableGripper",
        "-ConfirmPhysicalChecks",
        "start_gen3_pico_teleop.ps1",
        "prepare_gen3_hardware.ps1",
    ):
        assert contract in source, contract

    assert "InvertTranslation" not in source
    assert "SetShouldExit" not in source
    assert "Remove-Item" not in source


def test_tuned_launcher_parses_with_windows_powershell_51_ast() -> None:
    command = (
        "$errors=@();[void][System.Management.Automation.Language.Parser]::ParseFile("
        f"{_ps_literal(LAUNCHER)},[ref]$null,[ref]$errors);"
        "if($errors.Count){$errors | ForEach-Object {Write-Error $_};exit 1}"
    )
    result = subprocess.run(
        ["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", command],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr


def _write_fake_project(project: Path, log: Path) -> Path:
    scripts = project / "scripts"
    package = project / "kinova_teleop"
    scripts.mkdir(parents=True)
    package.mkdir()
    shutil.copy2(LAUNCHER, scripts / LAUNCHER.name)
    (scripts / "assert_no_untracked_python_startup_hooks.ps1").write_text(
        "param([string]$ProjectRoot)\n",
        encoding="utf-8",
    )
    (package / "__init__.py").write_text("", encoding="utf-8")
    (package / "public_hardware_setup.py").write_text(
        """from __future__ import annotations
import json
import os
from pathlib import Path
import sys

args = sys.argv[1:]
if not args or args[0] != 'package':
    raise SystemExit(91)
def value(flag: str) -> str:
    return args[args.index(flag) + 1]
output = Path(value('--output-dir'))
profile = {
    'run_id': 'run-repeatable-test',
    'lease_owner': value('--owner'),
}
for name, payload in (
    ('motion-lease.json', {}),
    ('reviewed-preflight.json', {}),
    ('teleop-profile.json', profile),
):
    (output / name).write_text(json.dumps(payload), encoding='utf-8')
record = {'stage': 'package', 'args': args, 'output': str(output)}
with Path(os.environ['TUNED_TEST_LOG']).open('a', encoding='utf-8') as stream:
    stream.write(json.dumps(record) + '\\n')
""",
        encoding="utf-8",
    )
    (scripts / "prepare_gen3_hardware.ps1").write_text(
        r"""[CmdletBinding()]
param(
  [Parameter(Mandatory=$true)][string]$RobotIp,
  [string]$RobotUser = 'admin',
  [Parameter(Mandatory=$true)][string]$Output,
  [string]$PythonPath
)
$record = @{stage='t0'; robot_ip=$RobotIp; robot_user=$RobotUser; output=$Output}
[IO.File]::AppendAllText($env:TUNED_TEST_LOG, (($record | ConvertTo-Json -Compress) + [Environment]::NewLine), [Text.UTF8Encoding]::new($false))
[IO.File]::WriteAllText($Output, '{}', [Text.UTF8Encoding]::new($false))
""",
        encoding="utf-8",
    )
    (scripts / "start_gen3_pico_teleop.ps1").write_text(
        r"""[CmdletBinding()]
param(
  [string]$RobotHost,
  [string]$RobotUser,
  [double]$Scale,
  [double]$MaxLinearSpeed,
  [double[]]$WorkspaceMin,
  [double[]]$WorkspaceMax,
  [string]$MotionLease,
  [string]$PreflightReport,
  [string]$OperatorCalibration,
  [switch]$EnableGripper,
  [string]$RunId,
  [string]$LeaseOwner,
  [double]$GripperBinaryThreshold,
  [double]$LinearGain,
  [double[]]$TranslationAxisGain,
  [string]$PythonPath
)
$record = @{
  stage='teleop'; robot_host=$RobotHost; robot_user=$RobotUser; scale=$Scale
  max_linear_speed=$MaxLinearSpeed; workspace_min=$WorkspaceMin; workspace_max=$WorkspaceMax
  motion_lease=$MotionLease; preflight_report=$PreflightReport
  calibration=$OperatorCalibration; enable_gripper=[bool]$EnableGripper
  run_id=$RunId; lease_owner=$LeaseOwner; threshold=$GripperBinaryThreshold
  linear_gain=$LinearGain; axis_gain=$TranslationAxisGain; python=$PythonPath
}
[IO.File]::AppendAllText($env:TUNED_TEST_LOG, (($record | ConvertTo-Json -Compress) + [Environment]::NewLine), [Text.UTF8Encoding]::new($false))
""",
        encoding="utf-8",
    )
    calibration = project / "saved-operator-axes.json"
    calibration.write_text('{"calibration":"test"}\n', encoding="utf-8")
    return calibration


def _run_fake_tuned_session(project: Path, calibration: Path, sessions: Path, log: Path) -> subprocess.CompletedProcess[str]:
    launcher = project / "scripts" / LAUNCHER.name
    command = (
        f"& {_ps_literal(launcher)} "
        "-RobotHost '192.0.2.10' -RobotUser 'test-operator' "
        f"-OperatorCalibration {_ps_literal(calibration)} "
        "-WorkspaceMin ([double[]]@(0.10,-0.30,0.05)) "
        "-WorkspaceMax ([double[]]@(0.90,0.30,0.60)) "
        "-ConfirmPhysicalChecks "
        f"-PythonPath {_ps_literal(Path(sys.executable))} "
        f"-SessionRoot {_ps_literal(sessions)}"
    )
    environment = dict(__import__("os").environ)
    environment["TUNED_TEST_LOG"] = str(log)
    return subprocess.run(
        ["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", command],
        cwd=project,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )


def test_tuned_launcher_creates_a_fresh_bound_package_on_every_run(tmp_path: Path) -> None:
    project = tmp_path / "project"
    log = tmp_path / "calls.jsonl"
    sessions = project / "local-sessions"
    calibration = _write_fake_project(project, log)

    first = _run_fake_tuned_session(project, calibration, sessions, log)
    second = _run_fake_tuned_session(project, calibration, sessions, log)

    assert first.returncode == second.returncode == 0, first.stderr + second.stderr
    records = [json.loads(line) for line in log.read_text(encoding="utf-8").splitlines()]
    assert [record["stage"] for record in records] == [
        "t0", "package", "teleop", "t0", "package", "teleop"
    ]
    package_outputs = [Path(record["output"]) for record in records if record["stage"] == "package"]
    assert package_outputs[0] != package_outputs[1]
    assert all(path.name == "local-config" for path in package_outputs)
    assert all((path / "operator-axes.json").is_file() for path in package_outputs)

    for record in (item for item in records if item["stage"] == "teleop"):
        assert record["scale"] == 1.0
        assert record["max_linear_speed"] == 0.05
        assert record["linear_gain"] == 1.5
        assert record["axis_gain"] == [-2.0, 1.0, 1.0]
        assert record["threshold"] == 0.9
        assert record["enable_gripper"] is True
        assert record["run_id"] == "run-repeatable-test"
        assert Path(record["motion_lease"]).parent.name == "local-config"


def test_tuned_launcher_rejects_missing_confirmation_before_creating_a_session(tmp_path: Path) -> None:
    source = LAUNCHER.read_text(encoding="utf-8")
    assert "if (-not $ConfirmPhysicalChecks)" in source
    assert source.index("if (-not $ConfirmPhysicalChecks)") < source.index(
        "$sessionRootResolved = Resolve-OrCreateRegularDirectory"
    )


def test_tuned_launcher_rejects_a_session_root_below_a_junction(tmp_path: Path) -> None:
    project = tmp_path / "project"
    log = tmp_path / "calls.jsonl"
    calibration = _write_fake_project(project, log)
    target = tmp_path / "session-target"
    junction = tmp_path / "session-junction"
    target.mkdir()
    result = subprocess.run(
        [
            "powershell.exe",
            "-NoProfile",
            "-NonInteractive",
            "-Command",
            f"New-Item -ItemType Junction -Path {_ps_literal(junction)} "
            f"-Target {_ps_literal(target)} | Out-Null",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        pytest.skip("junctions are unavailable")

    nested_session_root = junction / "nested-sessions"
    launch = _run_fake_tuned_session(
        project,
        calibration,
        nested_session_root,
        log,
    )

    assert launch.returncode != 0
    assert not nested_session_root.exists()
    assert not log.exists()
