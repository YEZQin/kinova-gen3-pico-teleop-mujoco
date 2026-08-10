import re
from pathlib import Path


def test_kortex_launcher_hardcodes_only_the_approved_profile() -> None:
    script = Path("scripts/start_kortex_pico_teleop.ps1").read_text(encoding="utf-8")
    assert "--control-hz', '40'" in script
    assert "--scale', '0.25'" in script
    assert "--stale-timeout', '0.2'" in script
    assert "--max-linear-speed', '0.005'" in script
    assert "--max-angular-speed-deg', '2'" in script
    assert "Password" not in script
    assert "KINOVA_PASSWORD=" not in script


def test_launcher_requires_external_bounds_lease_and_reviewed_report() -> None:
    script = Path("scripts/start_kortex_pico_teleop.ps1").read_text(encoding="utf-8")
    for parameter in ("WorkspaceMin", "WorkspaceMax", "MotionLease", "PreflightReport"):
        declaration = rf"\[Parameter\(Mandatory=\$true\)\][^\r\n]*\${parameter}\b"
        assert re.search(declaration, script)


def test_launcher_validates_paths_and_invokes_python_with_an_argument_array() -> None:
    script = Path("scripts/start_kortex_pico_teleop.ps1").read_text(encoding="utf-8")
    assert "Resolve-Path -LiteralPath" in script
    assert "Test-Path -LiteralPath $resolvedLease -PathType Leaf" in script
    assert "Test-Path -LiteralPath $resolvedPreflight -PathType Leaf" in script
    assert "& $resolvedPython @launchArguments" in script
    assert "--enable-hardware" in script
