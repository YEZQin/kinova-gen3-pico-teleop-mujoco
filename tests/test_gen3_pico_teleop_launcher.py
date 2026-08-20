from __future__ import annotations

import ast
import json
import os
from pathlib import Path
import re
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = ROOT / "scripts" / "start_gen3_pico_teleop.ps1"


def _ps_literal(value: Path | str) -> str:
    return "'" + str(value).replace("'", "''") + "'"


def _write_fake_python(path: Path) -> None:
    path.write_text(
        """$childArguments = [string[]]@($args)
$stage = if ($childArguments -contains '--validate-motion-package') {
    'validate'
} elseif ($childArguments -contains '--check-input' -or ($childArguments.Count -gt 0 -and $childArguments[0] -eq '-c')) {
    'pico'
} else {
    'motion'
}
$record = [ordered]@{
    stage = $stage
    args = $childArguments
    password = [string]$env:KINOVA_PASSWORD
}
$record | ConvertTo-Json -Compress -Depth 5 | Add-Content -LiteralPath $env:KINOVA_FAKE_LOG -Encoding UTF8
$exitCode = if ($stage -eq 'pico') {
    [int]$env:KINOVA_FAKE_PICO_EXIT
} elseif ($stage -eq 'motion') {
    [int]$env:KINOVA_FAKE_MOTION_EXIT
} elseif ($stage -eq 'validate') {
    [int]$env:KINOVA_FAKE_VALIDATION_EXIT
} else {
    0
}
& $env:ComSpec /d /c "exit $exitCode"
""",
        encoding="utf-8",
    )


@pytest.fixture
def launcher_fixture(tmp_path: Path) -> dict[str, Path]:
    paths = {
        "lease": tmp_path / "motion-lease.json",
        "report": tmp_path / "reviewed-preflight.json",
        "calibration": tmp_path / "operator-axes.json",
        "python": tmp_path / "fake-python.ps1",
        "log": tmp_path / "fake-child.jsonl",
        "prompt": tmp_path / "password-prompt.txt",
    }
    for key in ("lease", "report", "calibration"):
        paths[key].write_text("{}\n", encoding="utf-8")
    _write_fake_python(paths["python"])
    return paths


def _launcher_call(
    paths: dict[str, Path],
    *,
    scale: str = "1.0",
    max_linear_speed: str = "0.05",
    workspace_min: str = "([double[]]@(0.10,-0.30,0.05))",
    workspace_max: str = "([double[]]@(0.90,0.30,0.60))",
    lease: Path | None = None,
    report: Path | None = None,
    calibration: Path | None = None,
    enable_gripper: str = "-EnableGripper",
) -> str:
    resolved_lease = paths["lease"] if lease is None else lease
    resolved_report = paths["report"] if report is None else report
    resolved_calibration = paths["calibration"] if calibration is None else calibration
    return (
        f"& {_ps_literal(LAUNCHER)} "
        f"-MotionLease {_ps_literal(resolved_lease)} "
        f"-PreflightReport {_ps_literal(resolved_report)} "
        f"-OperatorCalibration {_ps_literal(resolved_calibration)} "
        f"-WorkspaceMin {workspace_min} -WorkspaceMax {workspace_max} "
        f"-RobotHost '192.0.2.10' -RobotUser 'test-operator' "
        f"-Scale {scale} -MaxLinearSpeed {max_linear_speed} "
        f"{enable_gripper} -PythonPath {_ps_literal(paths['python'])}"
    )


def _run_launcher(
    paths: dict[str, Path],
    *,
    validation_exit: int = 0,
    pico_exit: int = 0,
    motion_exit: int = 0,
    **call_overrides: object,
) -> subprocess.CompletedProcess[str]:
    command = (
        f"$env:KINOVA_FAKE_LOG={_ps_literal(paths['log'])};"
        f"$env:KINOVA_FAKE_VALIDATION_EXIT='{validation_exit}';"
        f"$env:KINOVA_FAKE_PICO_EXIT='{pico_exit}';"
        f"$env:KINOVA_FAKE_MOTION_EXIT='{motion_exit}';"
        "$env:KINOVA_PASSWORD=$null;"
        "function Read-Host {param([string]$Prompt,[switch]$AsSecureString)"
        f"[IO.File]::WriteAllText({_ps_literal(paths['prompt'])},"
        "$(if($AsSecureString){'secure'}else{'plain'}));"
        "if(-not $AsSecureString){throw 'unexpected unmasked confirmation prompt'};"
        "$secure=[Security.SecureString]::new();"
        "'test-only-password'.ToCharArray() | ForEach-Object {$secure.AppendChar($_)};"
        "$secure.MakeReadOnly();$secure};"
        "$caught=$null;try{"
        + _launcher_call(paths, **call_overrides)
        + "}catch{$caught=$_;$host.SetShouldExit(1)};"
        "if(Test-Path Env:KINOVA_PASSWORD){$host.SetShouldExit(98)};"
        "if($caught){Write-Error $caught -ErrorAction Continue}"
    )
    return subprocess.run(
        [
            "powershell.exe",
            "-NoProfile",
            "-NonInteractive",
            "-ExecutionPolicy",
            "Bypass",
            "-Command",
            command,
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )


def _records(path: Path) -> list[dict[str, object]]:
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8-sig").splitlines()]


def _value_after(arguments: list[str], flag: str) -> str:
    return arguments[arguments.index(flag) + 1]


def _normalized(value: str) -> str:
    return os.path.normcase(os.path.abspath(value))


def test_launcher_declares_the_guarded_single_confirmation_contract() -> None:
    """Making an artifact or gripper opt-in optional must fail this contract."""
    source = LAUNCHER.read_text(encoding="utf-8")

    for parameter in (
        "MotionLease",
        "PreflightReport",
        "OperatorCalibration",
        "WorkspaceMin",
        "WorkspaceMax",
        "EnableGripper",
    ):
        assert re.search(
            rf"\[Parameter\(Mandatory=\$true\)\][^\r\n]*\${parameter}\b", source
        )
    assert re.search(r"\[double\[\]\]\$WorkspaceMin\b", source)
    assert re.search(r"\[double\[\]\]\$WorkspaceMax\b", source)
    assert "[string]$RobotHost = '192.168.1.10'" in source
    assert "[string]$RobotUser = 'admin'" in source
    assert "[double]$Scale = 1.0" in source
    assert "[double]$MaxLinearSpeed = 0.05" in source
    assert "HARDWARE-READY" not in source
    assert re.search(r"\bMOVE\b", source) is None
    assert "Read-Host" in source and "AsSecureString" in source
    assert "finally" in source


def test_launcher_orders_startup_offline_pico_and_password_gates() -> None:
    """Reordering these gates could connect or disclose a password too early."""
    source = LAUNCHER.read_text(encoding="utf-8")

    startup = source.index("assert_no_untracked_python_startup_hooks.ps1")
    offline = source.index("--validate-motion-package")
    pico_v2 = source.index("trigger_available")
    password = source.index("Read-Host")
    assert startup < offline < pico_v2 < password


def test_launcher_parses_with_windows_powershell_51_ast() -> None:
    """A PS5.1 syntax regression would prevent every safety gate from running."""
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


def test_fake_child_observes_exact_gated_arguments_and_child_only_password(
    launcher_fixture: dict[str, Path],
) -> None:
    """Dropping a gate/argument or leaking the password into argv must fail."""
    result = _run_launcher(launcher_fixture)

    assert result.returncode == 0, result.stderr
    records = _records(launcher_fixture["log"])
    assert [record["stage"] for record in records] == ["validate", "pico", "motion"]
    assert [record["password"] for record in records] == ["", "", "test-only-password"]
    validate_args = records[0]["args"]
    pico_args = records[1]["args"]
    motion_args = records[2]["args"]
    assert isinstance(validate_args, list)
    assert isinstance(pico_args, list)
    assert isinstance(motion_args, list)
    assert validate_args == motion_args + ["--validate-motion-package"]

    expected_paths = {
        "--motion-lease": launcher_fixture["lease"],
        "--preflight-report": launcher_fixture["report"],
        "--operator-calibration": launcher_fixture["calibration"],
    }
    for flag, expected in expected_paths.items():
        assert _normalized(_value_after(motion_args, flag)) == _normalized(str(expected))
    for required in (
        "--backend",
        "kortex",
        "--enable-hardware",
        "--advanced-pico-teleop",
        "--input",
        "pico-udp",
        "--translation-only",
        "--responsive-translation-profile",
        "--recover-stale-input",
        "--gripper",
    ):
        assert required in motion_args
    assert _value_after(motion_args, "--robot-ip") == "192.0.2.10"
    assert _value_after(motion_args, "--robot-user") == "test-operator"
    assert _value_after(motion_args, "--scale") == "1"
    assert _value_after(motion_args, "--max-linear-speed") == "0.05"
    min_index = motion_args.index("--workspace-min")
    max_index = motion_args.index("--workspace-max")
    assert motion_args[min_index + 1 : min_index + 4] == ["0.1", "-0.3", "0.05"]
    assert motion_args[max_index + 1 : max_index + 4] == ["0.9", "0.3", "0.6"]
    assert all(
        "test-only-password" not in argument
        for record in records
        for argument in record["args"]
    )

    pico_program = "\n".join(pico_args)
    ast.parse(pico_args[1])
    for capability_check in (
        "sample.valid",
        "sample.timestamp_ns",
        "sample.grip",
        "sample.trigger_available",
        "math.isfinite(sample.trigger)",
    ):
        assert capability_check in pico_program
    assert launcher_fixture["prompt"].read_text(encoding="utf-8") == "secure"


def test_fake_child_failure_cleans_password_and_propagates_exact_exit(
    launcher_fixture: dict[str, Path],
) -> None:
    """A failed motion child must leave no secret and retain its exit status."""
    result = _run_launcher(launcher_fixture, motion_exit=37)

    assert result.returncode == 37, result.stderr
    records = _records(launcher_fixture["log"])
    assert [record["stage"] for record in records] == ["validate", "pico", "motion"]
    assert records[-1]["password"] == "test-only-password"


def test_failed_pico_v2_gate_never_prompts_or_runs_motion(
    launcher_fixture: dict[str, Path],
) -> None:
    """A V1/unavailable Trigger gate failure must precede password and motion."""
    result = _run_launcher(launcher_fixture, pico_exit=23)

    assert result.returncode != 0
    assert [record["stage"] for record in _records(launcher_fixture["log"])] == [
        "validate",
        "pico",
    ]
    assert not launcher_fixture["prompt"].exists()


def test_failed_offline_validation_never_reaches_pico_or_password(
    launcher_fixture: dict[str, Path],
) -> None:
    """An invalid package must stop before opening PICO input or asking a secret."""
    result = _run_launcher(launcher_fixture, validation_exit=19)

    assert result.returncode != 0
    assert [record["stage"] for record in _records(launcher_fixture["log"])] == [
        "validate"
    ]
    assert not launcher_fixture["prompt"].exists()


def test_each_launch_uses_a_new_absent_evidence_path(
    launcher_fixture: dict[str, Path],
) -> None:
    """Reusing an evidence path could append one trial to another."""
    first = _run_launcher(launcher_fixture)
    first_records = _records(launcher_fixture["log"])
    first_evidence = _value_after(first_records[-1]["args"], "--evidence-jsonl")
    launcher_fixture["log"].unlink()

    second = _run_launcher(launcher_fixture)
    second_records = _records(launcher_fixture["log"])
    second_evidence = _value_after(second_records[-1]["args"], "--evidence-jsonl")

    assert first.returncode == second.returncode == 0
    assert first_evidence != second_evidence
    assert not Path(first_evidence).exists()
    assert not Path(second_evidence).exists()


@pytest.mark.parametrize(
    ("overrides", "break_name"),
    (
        ({"scale": "0"}, "zero scale"),
        ({"scale": "([double]::NaN)"}, "non-finite scale"),
        ({"max_linear_speed": "([double]::PositiveInfinity)"}, "non-finite speed"),
        ({"workspace_min": "([double[]]@(0.10,-0.30))"}, "short minimum"),
        (
            {"workspace_max": "([double[]]@(0.90,[double]::NaN,0.60))"},
            "non-finite maximum",
        ),
        (
            {
                "workspace_min": "([double[]]@(0.10,-0.30,0.60))",
                "workspace_max": "([double[]]@(0.90,0.30,0.60))",
            },
            "equal workspace axis",
        ),
        (
            {
                "workspace_min": "([double[]]@(0.90,-0.30,0.05))",
                "workspace_max": "([double[]]@(0.10,0.30,0.60))",
            },
            "reversed workspace axis",
        ),
        ({"enable_gripper": "-EnableGripper:$false"}, "disabled gripper"),
    ),
)
def test_invalid_numeric_or_advanced_values_fail_before_any_python_child(
    launcher_fixture: dict[str, Path],
    overrides: dict[str, str],
    break_name: str,
) -> None:
    """Invalid runtime bounds or rates must fail before offline validation."""
    result = _run_launcher(launcher_fixture, **overrides)

    assert result.returncode != 0, break_name
    assert _records(launcher_fixture["log"]) == [], break_name
    assert not launcher_fixture["prompt"].exists(), break_name


@pytest.mark.parametrize("artifact", ("lease", "report", "calibration"))
def test_missing_or_non_file_artifact_fails_before_python(
    launcher_fixture: dict[str, Path], artifact: str, tmp_path: Path
) -> None:
    """Each reviewed local artifact must resolve to a regular file."""
    missing = tmp_path / f"missing-{artifact}.json"
    result = _run_launcher(launcher_fixture, **{artifact: missing})
    assert result.returncode != 0
    assert _records(launcher_fixture["log"]) == []

    directory = tmp_path / f"directory-{artifact}"
    directory.mkdir()
    result = _run_launcher(launcher_fixture, **{artifact: directory})
    assert result.returncode != 0
    assert _records(launcher_fixture["log"]) == []


def test_reparse_artifact_is_rejected_before_python(
    launcher_fixture: dict[str, Path], tmp_path: Path
) -> None:
    """A junction could swap a reviewed artifact after validation."""
    target = tmp_path / "calibration-target"
    target.mkdir()
    target_calibration = target / "operator-axes.json"
    target_calibration.write_text("{}\n", encoding="utf-8")
    junction = tmp_path / "calibration-junction"
    junction_result = subprocess.run(
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
    assert junction_result.returncode == 0, junction_result.stderr
    linked_calibration = junction / target_calibration.name

    result = _run_launcher(launcher_fixture, calibration=linked_calibration)

    assert result.returncode != 0
    assert _records(launcher_fixture["log"]) == []
    assert not launcher_fixture["prompt"].exists()
