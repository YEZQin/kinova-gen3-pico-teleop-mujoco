from __future__ import annotations

import json
import hashlib
from pathlib import Path
import subprocess


def test_prepare_script_only_hands_password_to_the_readonly_t0_child() -> None:
    """Moving secret handling into package generation would persist credentials."""
    script = Path("scripts/prepare_gen3_hardware.ps1").read_text(encoding="utf-8")

    assert "Read-Host -AsSecureString" in script
    assert "KINOVA_PASSWORD" in script
    assert "finally" in script
    assert "package" not in script.split("Read-Host -AsSecureString", 1)[0]


def test_generated_launcher_validates_offline_before_pico_or_password() -> None:
    """Reordering the gates could connect or disclose a secret for an invalid package."""
    script = Path("scripts/start_generated_gen3_teleop.ps1").read_text(encoding="utf-8")

    offline_gate = script.index("--validate-motion-package")
    pico_gate = script.index("--check-input")
    password_gate = script.index("Read-Host -AsSecureString")
    assert offline_gate < pico_gate < password_gate
    assert "HARDWARE-READY" in script
    assert "Type MOVE" not in script
    assert "--translation-only" in script
    assert "--expanded-translation-envelope" in script
    assert "--responsive-translation-profile" in script
    assert "--recover-stale-input" in script
    assert "'--scale','0.8'" in script
    assert "'--control-hz','40'" in script
    assert "--gripper" not in script
    assert "finally" in script


def test_guarded_scripts_parse_in_windows_powershell() -> None:
    """A syntax change would prevent startup gates from running at all."""
    command = ""
    for script in ("scripts/prepare_gen3_hardware.ps1", "scripts/start_generated_gen3_teleop.ps1"):
        command += (
            "$errors=@();[void][System.Management.Automation.Language.Parser]::ParseFile("
            f"'{script}',[ref]$null,[ref]$errors);if($errors.Count){{exit 1}};"
        )
    result = subprocess.run(
        ["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", command],
        check=False, capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stderr


def test_prepare_child_failure_clears_password_with_fake_python(tmp_path: Path) -> None:
    """A failing T0 child must not leave its password in the caller environment."""
    fake = tmp_path / "fake-python.cmd"
    fake.write_text("@exit /b 2\r\n", encoding="utf-8")
    output = tmp_path / "local-config"
    output.mkdir()
    command = (
        "$env:KINOVA_PASSWORD=$null;function Read-Host {param([string]$Prompt,[switch]$AsSecureString)"
        "ConvertTo-SecureString 'test-only' -AsPlainText -Force};"
        "try{& 'scripts/prepare_gen3_hardware.ps1' -RobotIp '192.168.1.10' -Output '"
        + str(output / "t0.json").replace("\\", "\\\\")
        + "' -PythonPath '" + str(fake).replace("\\", "\\\\")
        + "'}catch{};if($env:KINOVA_PASSWORD){exit 3}else{exit 0}"
    )
    result = subprocess.run(
        ["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", command],
        check=False, capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stderr


def test_generated_launcher_child_failure_clears_password_and_rejects_evidence_collision(tmp_path: Path) -> None:
    """A failed motion child or reused evidence path must not retain credentials."""
    revision = subprocess.run(
        ["git", "rev-parse", "HEAD"], check=True, capture_output=True, text=True
    ).stdout.strip()
    calibration = tmp_path / "operator-axes.json"
    calibration.write_text("{}\n", encoding="utf-8")
    driver = Path("kinova_teleop/kortex_backend.py")
    profile = tmp_path / "teleop-profile.json"
    profile.write_text(json.dumps({
        "schema_version": "1.0", "device": "gen3", "robot_ip": "192.168.1.10", "robot_user": "operator",
        "motion_lease": "motion-lease.json", "preflight_report": "reviewed-preflight.json", "calibration": calibration.name,
        "workspace_min_m": [0, -0.3, 0.05], "workspace_max_m": [1, 0.3, 0.6], "max_linear_speed_m_s": 0.02,
        "run_id": "run-test", "lease_owner": "operator", "code_revision": revision,
        "calibration_sha256": hashlib.sha256(calibration.read_bytes()).hexdigest(),
        "driver_sha256": hashlib.sha256(driver.read_bytes()).hexdigest(),
        "reference_pose_m": [0.4, -0.1, 0.3], "workspace_midpoint_m": [0.5, 0, 0.325],
    }), encoding="utf-8")
    for name in ("motion-lease.json", "reviewed-preflight.json"):
        (tmp_path / name).write_text("{}\n", encoding="utf-8")
    fake = tmp_path / "fake-python.cmd"
    fake.write_text(
        "@echo off\r\necho %* | findstr /C:\"--validate-motion-package\" >nul && exit /b 0\r\n"
        "echo %* | findstr /C:\"--check-input\" >nul && exit /b 0\r\nexit /b 2\r\n",
        encoding="utf-8",
    )
    command = (
        "$env:KINOVA_PASSWORD=$null;function Read-Host {param([string]$Prompt,[switch]$AsSecureString)"
        "if($AsSecureString){ConvertTo-SecureString 'test-only' -AsPlainText -Force}else{'HARDWARE-READY'}};"
        "try{& 'scripts/start_generated_gen3_teleop.ps1' -Profile '"
        + str(profile).replace("\\", "/") + "' -PythonPath '" + str(fake).replace("\\", "/")
        + "'}catch{};if($env:KINOVA_PASSWORD){exit 3}else{exit 0}"
    )
    result = subprocess.run(
        ["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", command],
        check=False, capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stderr
    script = Path("scripts/start_generated_gen3_teleop.ps1").read_text(encoding="utf-8")
    assert "Test-Path -LiteralPath $evidence" in script
