from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest


BOOTSTRAP = Path("scripts/bootstrap_public_teleop.ps1")


@pytest.fixture
def bootstrap_source() -> str:
    return BOOTSTRAP.read_text(encoding="utf-8")


def test_public_bootstrap_exposes_the_safe_windows_cli(bootstrap_source: str) -> None:
    for parameter in (
        "PythonExe",
        "ReleaseManifest",
        "DownloadDirectory",
        "VenvDirectory",
        "ApkPath",
        "KortexWheel",
        "InstallApk",
        "DeviceSerial",
        "SkipOfflineTests",
    ):
        assert f"${parameter}" in bootstrap_source

    assert "Windows_NT" in bootstrap_source
    assert "3.11" in bootstrap_source
    assert "Resolve-Path -LiteralPath" in bootstrap_source


def test_public_bootstrap_verifies_only_manifested_github_assets(
    bootstrap_source: str,
) -> None:
    assert "release_assets.load_release_manifest" in bootstrap_source
    assert "release_assets.verify_release_asset" in bootstrap_source
    assert "Invoke-WebRequest" in bootstrap_source
    assert "-MaximumRedirection 0" in bootstrap_source
    assert "GitHub release asset" in bootstrap_source
    assert "Get-FileHash" in bootstrap_source


def test_public_bootstrap_uses_argument_arrays_and_required_install_order(
    bootstrap_source: str,
) -> None:
    for invocation in (
        "& $resolvedPython @manifestLoadArguments",
        "& $resolvedPython @assetVerificationArguments",
        "& $venvPython @kortexInstallArguments",
        "& $venvPython @projectInstallArguments",
        "& $venvPython @protobufInstallArguments",
        "& $venvPython @pytestArguments",
        "& $venvPython @dryRunArguments",
    ):
        assert invocation in bootstrap_source

    markers = (
        "assert_no_untracked_python_startup_hooks.ps1",
        "@manifestLoadArguments",
        "Invoke-WebRequest",
        "@assetVerificationArguments",
        "-m', 'venv'",
        "@kortexInstallArguments",
        "@projectInstallArguments",
        "import kortex_api",
        "@pytestArguments",
        "@dryRunArguments",
        "install-receipt.json",
    )
    positions = [bootstrap_source.index(marker) for marker in markers]
    assert positions == sorted(positions)


def test_public_bootstrap_never_touches_a_device_without_the_explicit_switch(
    bootstrap_source: str,
) -> None:
    install_guard = bootstrap_source.index("if ($InstallApk)")
    adb_commands = bootstrap_source.index("adb devices")
    assert install_guard < adb_commands
    assert "adb -s" in bootstrap_source
    assert "install -r" in bootstrap_source
    assert "adb tcpip" not in bootstrap_source
    assert "adb connect" not in bootstrap_source
    assert "New-NetFirewallRule" not in bootstrap_source
    assert "netsh advfirewall" not in bootstrap_source


def test_public_bootstrap_does_not_read_or_record_a_kinova_password(
    bootstrap_source: str,
) -> None:
    assert "KINOVA_PASSWORD" not in bootstrap_source
    assert "password" not in bootstrap_source.lower()
    assert "repository_commit" in bootstrap_source
    assert "asset_sha256" in bootstrap_source


def _initialize_git_fixture(project: Path) -> None:
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


def _find_csharp_compiler() -> str:
    compiler = shutil.which("csc.exe")
    if compiler:
        return compiler
    windows_directory = Path(os.environ["WINDIR"])
    candidates = sorted(
        (windows_directory / "Microsoft.NET").glob("Framework*/v4.0.30319/csc.exe")
    )
    if not candidates:
        pytest.skip("a local C# compiler is required for the fake python executable")
    return str(candidates[-1])


def _build_fake_python(tmp_path: Path) -> Path:
    source = tmp_path / "fake_python.cs"
    executable = tmp_path / "python.exe"
    source.write_text(
        """
using System;
using System.Diagnostics;
using System.IO;

public static class FakePython {
    public static int Main(string[] args) {
        File.AppendAllText(Environment.GetEnvironmentVariable("FAKE_PYTHON_LOG"),
            String.Join("|", args) + Environment.NewLine);
        if (args.Length > 0 && args[0] == "--version") {
            Console.WriteLine(Environment.GetEnvironmentVariable("FAKE_PYTHON_VERSION") ?? "Python 3.11.9");
            return 0;
        }
        if (args.Length > 1 && args[0] == "-c" &&
            Environment.GetEnvironmentVariable("FAKE_MANIFEST_FAILURE") == "1" &&
            args[1].Contains("load_release_manifest")) return 1;
        if (args.Length > 1 && args[0] == "-c" &&
            Environment.GetEnvironmentVariable("FAKE_ASSET_FAILURE") == "1") return 1;
        if (args.Length > 2 && args[0] == "-m" && args[1] == "venv") {
            string scripts = Path.Combine(args[2], "Scripts");
            Directory.CreateDirectory(scripts);
            File.Copy(Process.GetCurrentProcess().MainModule.FileName,
                Path.Combine(scripts, "python.exe"), true);
        }
        return 0;
    }
}
""".strip(),
        encoding="utf-8",
    )
    subprocess.run(
        [_find_csharp_compiler(), "/nologo", f"/out:{executable}", str(source)],
        check=True,
        capture_output=True,
        text=True,
    )
    return executable


def _write_release_manifest(project: Path) -> None:
    downloads = project / "downloads"
    downloads.mkdir()
    assets = {
        "kinova-pico-udp-bridge.apk": b"apk",
        "kortex_api-2.8.0.post5-py3-none-any.whl": b"wheel",
    }
    release = "v0.2.0-rc.1"
    prefix = "https://github.com/YEZQin/kinova-gen3-pico-teleop-mujoco/releases/download"
    manifest_assets = []
    for name, contents in assets.items():
        (downloads / name).write_bytes(contents)
        manifest_assets.append(
            {
                "name": name,
                "sha256": hashlib.sha256(contents).hexdigest(),
                "size_bytes": len(contents),
                "download_url": f"{prefix}/{release}/{name}",
            }
        )
    release_directory = project / "release"
    release_directory.mkdir()
    (release_directory / "public-release-assets.json").write_text(
        json.dumps({"schema_version": "1.0", "release": release, "assets": manifest_assets}),
        encoding="utf-8",
    )


def _prepare_offline_project(tmp_path: Path) -> tuple[Path, Path, dict[str, str]]:
    project = tmp_path / "repo"
    scripts = project / "scripts"
    scripts.mkdir(parents=True)
    shutil.copy2(BOOTSTRAP, scripts / BOOTSTRAP.name)
    shutil.copy2(
        "scripts/assert_no_untracked_python_startup_hooks.ps1",
        scripts / "assert_no_untracked_python_startup_hooks.ps1",
    )
    _initialize_git_fixture(project)
    _write_release_manifest(project)
    fake_python = _build_fake_python(tmp_path)
    environment = {
        **os.environ,
        "FAKE_PYTHON_LOG": str(tmp_path / "fake-python.log"),
        "FAKE_ADB_LOG": str(tmp_path / "fake-adb.log"),
        "PATH": f"{tmp_path}{os.pathsep}{os.environ['PATH']}",
    }
    return project, fake_python, environment


def _run_bootstrap(
    project: Path, fake_python: Path, environment: dict[str, str], *arguments: str
) -> subprocess.CompletedProcess[str]:
    command = [
        "powershell",
        "-NoProfile",
        "-NonInteractive",
        "-ExecutionPolicy",
        "Bypass",
        "-File",
        str(project / "scripts" / BOOTSTRAP.name),
        "-PythonExe",
        str(fake_python),
        *arguments,
    ]
    return subprocess.run(
        command,
        cwd=project,
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )


def test_public_bootstrap_offline_fixture_uses_fake_python_and_never_invokes_adb(
    tmp_path: Path,
) -> None:
    project, fake_python, environment = _prepare_offline_project(tmp_path)
    python_log = Path(environment["FAKE_PYTHON_LOG"])
    adb_log = Path(environment["FAKE_ADB_LOG"])
    adb_shim = tmp_path / "adb.cmd"
    adb_shim.write_text(
        "@echo %*>>\"%FAKE_ADB_LOG%\"\r\n@echo List of devices attached\r\n@echo fake-pico device\r\n",
        encoding="utf-8",
    )
    result = _run_bootstrap(project, fake_python, environment, "-SkipOfflineTests")

    assert result.returncode == 0, result.stderr
    calls = python_log.read_text(encoding="utf-8")
    assert "-m|venv" in calls
    assert "-m|pip|install" in calls
    assert "protobuf==3.20.0" in calls
    assert not adb_log.exists()
    receipt = json.loads((project / "local-config" / "install-receipt.json").read_text())
    assert receipt["repository_commit"]
    assert receipt["asset_sha256"]["apk"] == hashlib.sha256(b"apk").hexdigest()


def test_public_bootstrap_rejects_non_python_311_before_manifest_work(tmp_path: Path) -> None:
    project, fake_python, environment = _prepare_offline_project(tmp_path)
    environment["FAKE_PYTHON_VERSION"] = "Python 3.13.0"

    result = _run_bootstrap(project, fake_python, environment, "-SkipOfflineTests")

    assert result.returncode != 0
    assert "PythonExe must be Python 3.11" in result.stderr
    assert not (project / "local-config").exists()


def test_public_bootstrap_rejects_non_windows_before_any_tool_invocation(tmp_path: Path) -> None:
    project, fake_python, environment = _prepare_offline_project(tmp_path)
    environment["OS"] = "Linux"

    result = _run_bootstrap(project, fake_python, environment, "-SkipOfflineTests")

    assert result.returncode != 0
    assert "must run on Windows" in result.stderr
    assert not Path(environment["FAKE_PYTHON_LOG"]).exists()


def test_public_bootstrap_rejects_untracked_startup_hook_before_python(tmp_path: Path) -> None:
    project, fake_python, environment = _prepare_offline_project(tmp_path)
    (project / "sitecustomize.py").write_text("unreviewed\n", encoding="utf-8")

    result = _run_bootstrap(project, fake_python, environment, "-SkipOfflineTests")

    assert result.returncode != 0
    assert "Untracked Python startup hook" in result.stderr
    assert not Path(environment["FAKE_PYTHON_LOG"]).exists()


@pytest.mark.parametrize("failure_variable", ["FAKE_MANIFEST_FAILURE", "FAKE_ASSET_FAILURE"])
def test_public_bootstrap_rejects_manifest_or_asset_integrity_failure(
    tmp_path: Path, failure_variable: str
) -> None:
    project, fake_python, environment = _prepare_offline_project(tmp_path)
    environment[failure_variable] = "1"

    result = _run_bootstrap(project, fake_python, environment, "-SkipOfflineTests")

    assert result.returncode != 0
    assert "failed" in result.stderr.lower()
    assert not (project / ".venv-kortex").exists()


def test_public_bootstrap_rejects_existing_non_venv_target(tmp_path: Path) -> None:
    project, fake_python, environment = _prepare_offline_project(tmp_path)
    target = project / "not-a-venv"
    target.mkdir()

    result = _run_bootstrap(
        project, fake_python, environment, "-SkipOfflineTests", "-VenvDirectory", str(target)
    )

    assert result.returncode != 0
    assert "already exists but is not a Python virtual environment" in result.stderr


def test_public_bootstrap_rejects_download_redirect_outside_github_hosts(tmp_path: Path) -> None:
    project, fake_python, environment = _prepare_offline_project(tmp_path)
    manifest_path = project / "release" / "public-release-assets.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["assets"][0]["download_url"] = "https://example.invalid/asset.apk"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    (project / "downloads" / manifest["assets"][0]["name"]).unlink()

    result = _run_bootstrap(project, fake_python, environment, "-SkipOfflineTests")

    assert result.returncode != 0
    assert "approved HTTPS GitHub release URL" in result.stderr


def test_public_bootstrap_documents_the_bounded_github_redirect_allow_list(
    bootstrap_source: str,
) -> None:
    for host in (
        "github.com",
        "objects.githubusercontent.com",
        "release-assets.githubusercontent.com",
        "github-releases.githubusercontent.com",
    ):
        assert f"'{host}'" in bootstrap_source
    assert "$maximumRedirects = 5" in bootstrap_source


@pytest.mark.parametrize("device_lines, extra_arguments", [
    ("one device\ntwo device", ()),
    ("one device", ("-DeviceSerial", "not-one")),
])
def test_public_bootstrap_rejects_ambiguous_or_unauthorized_adb_device(
    tmp_path: Path, device_lines: str, extra_arguments: tuple[str, ...]
) -> None:
    project, fake_python, environment = _prepare_offline_project(tmp_path)
    (tmp_path / "adb.cmd").write_text(
        "@echo %*>>\"%FAKE_ADB_LOG%\"\r\n@echo List of devices attached\r\n"
        + "\r\n".join(f"@echo {line}" for line in device_lines.split("\n"))
        + "\r\n",
        encoding="utf-8",
    )

    result = _run_bootstrap(
        project, fake_python, environment, "-SkipOfflineTests", "-InstallApk", *extra_arguments
    )

    assert result.returncode != 0
    assert "authorized ADB device" in result.stderr
    assert "install -r" not in Path(environment["FAKE_ADB_LOG"]).read_text(encoding="utf-8")
