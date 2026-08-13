import re
from pathlib import Path
import subprocess

import pytest


STARTUP_GATE = Path("scripts/assert_no_untracked_python_startup_hooks.ps1")


def _initialize_git_fixture(project: Path) -> None:
    project.mkdir()
    subprocess.run(["git", "init", "--quiet"], cwd=project, check=True)
    subprocess.run(
        [
            "git",
            "-c",
            "user.name=Kinova Test",
            "-c",
            "user.email=kinova-test@example.invalid",
            "commit",
            "--allow-empty",
            "--quiet",
            "-m",
            "fixture",
        ],
        cwd=project,
        check=True,
    )


def _run_startup_gate(project: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            "powershell",
            "-NoProfile",
            "-NonInteractive",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(STARTUP_GATE.resolve()),
            "-ProjectRoot",
            str(project),
        ],
        check=False,
        capture_output=True,
        text=True,
    )


@pytest.mark.parametrize(
    "relative_hook",
    (
        "sitecustomize.py",
        "usercustomize.py",
        "sitecustomize/__init__.py",
        "usercustomize/__init__.py",
        "sitecustomize.pyc",
        "usercustomize.cp311-win_amd64.pyd",
    ),
)
def test_prelaunch_gate_rejects_untracked_python_startup_hook(
    tmp_path: Path,
    relative_hook: str,
) -> None:
    project = tmp_path / "repo"
    _initialize_git_fixture(project)
    hook = project / relative_hook
    hook.parent.mkdir(parents=True, exist_ok=True)
    hook.write_text("unreviewed\n", encoding="utf-8")

    result = _run_startup_gate(project)

    assert result.returncode != 0
    assert "Untracked Python startup hook" in result.stderr
    assert relative_hook.replace("/", "\\") in result.stderr


def test_prelaunch_gate_allows_ordinary_untracked_trial_artifacts(tmp_path: Path) -> None:
    project = tmp_path / "repo"
    _initialize_git_fixture(project)
    (project / "results").mkdir()
    (project / "results" / "trial.json").write_text("{}\n", encoding="utf-8")
    (project / ".pytest-local").mkdir()
    (project / ".pytest-local" / "state").write_text("ok\n", encoding="utf-8")

    result = _run_startup_gate(project)

    assert result.returncode == 0, result.stderr


def test_prelaunch_gate_allows_only_clean_tracked_startup_hook(tmp_path: Path) -> None:
    project = tmp_path / "repo"
    _initialize_git_fixture(project)
    hook = project / "sitecustomize.py"
    hook.write_text("reviewed = True\n", encoding="utf-8")
    subprocess.run(["git", "add", "sitecustomize.py"], cwd=project, check=True)
    subprocess.run(
        [
            "git",
            "-c",
            "user.name=Kinova Test",
            "-c",
            "user.email=kinova-test@example.invalid",
            "commit",
            "--quiet",
            "-m",
            "track hook",
        ],
        cwd=project,
        check=True,
    )

    assert _run_startup_gate(project).returncode == 0

    hook.write_text("reviewed = False\n", encoding="utf-8")
    result = _run_startup_gate(project)
    assert result.returncode != 0
    assert "Dirty Python startup hook" in result.stderr


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


def test_launcher_runs_startup_hook_gate_before_python() -> None:
    script = Path("scripts/start_kortex_pico_teleop.ps1").read_text(encoding="utf-8")
    gate_call = "assert_no_untracked_python_startup_hooks.ps1"

    assert gate_call in script
    assert script.index(gate_call) < script.index("& $resolvedPython @launchArguments")


def test_readonly_quickstart_runs_startup_hook_gate_before_password() -> None:
    guide = Path("docs/kortex-hardware-quickstart.md").read_text(encoding="utf-8")
    t0 = guide[guide.index("## 2. T0") : guide.index("## Mandatory pause after T0")]

    assert "assert_no_untracked_python_startup_hooks.ps1" in t0
    assert t0.index("assert_no_untracked_python_startup_hooks.ps1") < t0.index(
        "Read-Host 'Kortex password'"
    )
