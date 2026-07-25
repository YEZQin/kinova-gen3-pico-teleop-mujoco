"""Process-level contracts for the native Windows PICO UDP helpers."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess


REPOSITORY = Path(__file__).resolve().parents[1]
POWERSHELL = "powershell.exe"


def run_script(script: str, *arguments: str, cwd: Path) -> subprocess.CompletedProcess[str]:
    """Run a helper in a deliberately unrelated current directory."""
    return subprocess.run(
        [
            POWERSHELL,
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(REPOSITORY / "scripts" / script),
            *arguments,
        ],
        cwd=cwd,
        text=True,
        capture_output=True,
        check=False,
    )


def test_build_dry_run_prints_the_pinned_editmode_test_and_apk_build(
    tmp_path: Path,
) -> None:
    """Removing the reproducible Unity test/build sequence must fail this test."""
    unity = r"C:\\Unity\\2022.3.62f3c1\\Editor\\Unity.exe"
    build = run_script(
        "build_pico_udp_bridge.ps1", "-UnityPath", unity, "-DryRun", cwd=tmp_path
    )

    assert build.returncode == 0, build.stderr
    assert "2022.3.62f3c1" in build.stdout
    assert unity in build.stdout
    assert "runTests" in build.stdout
    assert "EditMode" in build.stdout
    assert "BuildKinovaPicoBridge.Build" in build.stdout
    assert "kinova-pico-udp-bridge.apk" in build.stdout
    assert "C:\\adb\\adb.exe" not in build.stdout


def test_build_requires_an_explicit_unity_path(tmp_path: Path) -> None:
    """A repository must not embed one developer's local Unity installation path."""
    build = run_script("build_pico_udp_bridge.ps1", "-DryRun", cwd=tmp_path)

    assert build.returncode != 0
    assert "-UnityPath" in build.stderr


def test_build_dry_run_prints_serial_scoped_install_without_running_adb(
    tmp_path: Path,
) -> None:
    """Dropping an explicitly selected serial must fail this safety contract."""
    build = run_script(
        "build_pico_udp_bridge.ps1",
        "-Install",
        "-DeviceSerial",
        "PICO-USB-42",
        "-UnityPath",
        r"C:\\Unity\\2022.3.62f3c1\\Editor\\Unity.exe",
        "-DryRun",
        cwd=tmp_path,
    )

    assert build.returncode == 0, build.stderr
    assert "C:\\adb\\adb.exe" in build.stdout
    assert "-s PICO-USB-42 install -r" in build.stdout


def test_build_can_isolate_all_artifacts_from_the_default_output(
    tmp_path: Path,
) -> None:
    """Fake Unity validation must never replace a developer's real APK/XML."""
    unity = tmp_path / "isolated-unity.cmd"
    unity.write_text(
        "@echo off\n"
        "set \"SCRIPT_DIR=%~dp0\"\n"
        ":loop\n"
        "if \"%1\"==\"\" goto done\n"
        "if \"%1\"==\"-testResults\" (\n"
        "  >\"%~2\" echo ^<test-run result=\"Passed\" failed=\"0\" /^>\n"
        ")\n"
        "shift\n"
        "goto loop\n"
        ":done\n"
        ">\"%SCRIPT_DIR%successful.apk\" echo isolated-apk\n"
        "exit /b 0\n",
        encoding="utf-8",
    )
    built_apk = tmp_path / "successful.apk"
    isolated_artifacts = tmp_path / "artifacts"
    default_apk = REPOSITORY / "artifacts" / "kinova-pico-udp-bridge.apk"
    default_xml = REPOSITORY / "artifacts" / "kinova-pico-editmode-results.xml"
    before_apk = default_apk.read_bytes() if default_apk.exists() else None
    before_xml = default_xml.read_bytes() if default_xml.exists() else None

    build = run_script(
        "build_pico_udp_bridge.ps1",
        "-UnityPath",
        str(unity),
        "-BuiltApkPath",
        str(built_apk),
        "-ArtifactsPath",
        str(isolated_artifacts),
        cwd=tmp_path,
    )

    assert build.returncode == 0, build.stderr
    assert (isolated_artifacts / "kinova-pico-udp-bridge.apk").read_text(
        encoding="utf-8"
    ).strip() == "isolated-apk"
    assert (isolated_artifacts / "kinova-pico-editmode-results.xml").exists()
    assert (default_apk.read_bytes() if default_apk.exists() else None) == before_apk
    assert (default_xml.read_bytes() if default_xml.exists() else None) == before_xml


def test_build_rejects_stale_test_results_after_zero_exit(tmp_path: Path) -> None:
    """A prior passing XML/APK must not validate a Unity process that wrote nothing."""
    unity = tmp_path / "zero-exit-unity.cmd"
    unity.write_text("@echo off\nexit /b 0\n", encoding="utf-8")
    artifacts = tmp_path / "artifacts"
    artifacts.mkdir()
    (artifacts / "kinova-pico-editmode-results.xml").write_text(
        '<test-run result="Passed" failed="0" />', encoding="utf-8"
    )
    built_apk = tmp_path / "KinovaPicoBridge-development.apk"
    built_apk.write_bytes(b"stale-apk")

    build = run_script(
        "build_pico_udp_bridge.ps1",
        "-UnityPath",
        str(unity),
        "-BuiltApkPath",
        str(built_apk),
        "-ArtifactsPath",
        str(artifacts),
        cwd=tmp_path,
    )

    assert build.returncode != 0
    assert "test results were not written" in build.stderr


def test_build_rejects_stale_apk_after_zero_exit(tmp_path: Path) -> None:
    """A fresh Unity XML is insufficient when the build leaves an old APK behind."""
    unity = tmp_path / "xml-only-unity.cmd"
    unity.write_text(
        "@echo off\n"
        ":loop\n"
        "if \"%1\"==\"\" goto done\n"
        "if \"%1\"==\"-testResults\" (\n"
        "  >\"%~2\" echo ^<test-run result=\"Passed\" failed=\"0\" /^>\n"
        ")\n"
        "shift\n"
        "goto loop\n"
        ":done\n"
        "exit /b 0\n",
        encoding="utf-8",
    )
    built_apk = tmp_path / "KinovaPicoBridge-development.apk"
    built_apk.write_bytes(b"stale-apk")

    build = run_script(
        "build_pico_udp_bridge.ps1",
        "-UnityPath",
        str(unity),
        "-BuiltApkPath",
        str(built_apk),
        "-ArtifactsPath",
        str(tmp_path / "artifacts"),
        cwd=tmp_path,
    )

    assert build.returncode != 0
    assert "nonempty APK" in build.stderr


def test_build_allows_unity_test_runner_and_batch_build_to_own_exit(
    tmp_path: Path,
) -> None:
    """A premature Unity -quit must not bypass tests or the batch build."""
    unity = tmp_path / "unity-requires-owned-exit.cmd"
    unity.write_text(
        "@echo off\n"
        "set \"SCRIPT_DIR=%~dp0\"\n"
        "for %%A in (%*) do if \"%%~A\"==\"-quit\" exit /b 86\n"
        ":loop\n"
        "if \"%1\"==\"\" goto done\n"
        "if \"%1\"==\"-testResults\" (\n"
        "  >\"%~2\" echo ^<test-run result=\"Passed\" failed=\"0\" /^>\n"
        ")\n"
        "shift\n"
        "goto loop\n"
        ":done\n"
        ">\"%SCRIPT_DIR%successful.apk\" echo apk\n"
        "exit /b 0\n",
        encoding="utf-8",
    )
    built_apk = tmp_path / "successful.apk"

    build = run_script(
        "build_pico_udp_bridge.ps1",
        "-UnityPath",
        str(unity),
        "-BuiltApkPath",
        str(built_apk),
        "-ArtifactsPath",
        str(tmp_path / "artifacts"),
        cwd=tmp_path,
    )

    assert build.returncode == 0, build.stderr
    assert built_apk.stat().st_size > 0


def test_build_waits_for_the_unity_gui_process_before_inspecting_outputs(
    tmp_path: Path,
) -> None:
    """Returning early from the Windows GUI editor must not race its outputs."""
    source = tmp_path / "DelayedUnity.cs"
    source.write_text(
        "using System;\n"
        "using System.IO;\n"
        "using System.Threading;\n"
        "public static class DelayedUnity {\n"
        "  [STAThread]\n"
        "  public static int Main(string[] args) {\n"
        "    Thread.Sleep(250);\n"
        "    for (var i = 0; i < args.Length; i++) {\n"
        "      if (args[i] == \"-testResults\" && i + 1 < args.Length)\n"
        "        File.WriteAllText(args[i + 1],\n"
        "          \"<test-run result=\\\"Passed\\\" failed=\\\"0\\\" />\");\n"
        "      if (args[i] == \"-executeMethod\")\n"
        "        File.WriteAllText(Path.Combine(AppDomain.CurrentDomain.BaseDirectory,\n"
        "          \"successful.apk\"), \"apk\");\n"
        "    }\n"
        "    return 0;\n"
        "  }\n"
        "}\n",
        encoding="utf-8",
    )
    unity = tmp_path / "delayed-unity.exe"
    compiler = (
        Path(os.environ["WINDIR"])
        / "Microsoft.NET"
        / "Framework64"
        / "v4.0.30319"
        / "csc.exe"
    )
    compile_result = subprocess.run(
        [
            str(compiler),
            "/nologo",
            "/target:winexe",
            f"/out:{unity}",
            str(source),
        ],
        text=True,
        capture_output=True,
        check=False,
    )
    assert compile_result.returncode == 0, compile_result.stderr
    built_apk = tmp_path / "successful.apk"

    build = run_script(
        "build_pico_udp_bridge.ps1",
        "-UnityPath",
        str(unity),
        "-BuiltApkPath",
        str(built_apk),
        "-ArtifactsPath",
        str(tmp_path / "artifacts"),
        cwd=tmp_path,
    )

    assert build.returncode == 0, build.stderr
    assert built_apk.stat().st_size > 0


def test_start_dry_run_uses_native_pico_preflight_and_safe_defaults(
    tmp_path: Path,
) -> None:
    """Changing the launcher input, preflight, or defaults must fail this test."""
    start = run_script("start_pico_udp_teleop.ps1", "-DryRun", cwd=tmp_path)

    assert start.returncode == 0, start.stderr
    assert "com.yezqin.kinovapicobridge" in start.stdout
    assert "--input pico-udp" in start.stdout
    assert "--check-input" in start.stdout
    assert "--samples 20" in start.stdout
    assert "--scale 0.5" in start.stdout
    assert "--check-timeout 30" in start.stdout
    assert "15031" in start.stdout
    for forbidden in (
        "wsl.exe",
        "xrobotoolkit",
        "reverse tcp",
        "adb tcpip",
        "adb connect",
    ):
        assert forbidden not in start.stdout.lower()


def test_start_dry_run_honors_manual_start_and_explicit_serial(tmp_path: Path) -> None:
    """Ignoring manual mode or a requested serial must fail this test."""
    manual = run_script(
        "start_pico_udp_teleop.ps1", "-ManualPicoStart", "-DryRun", cwd=tmp_path
    )
    serial = run_script(
        "start_pico_udp_teleop.ps1",
        "-DeviceSerial",
        "PICO-USB-42",
        "-DryRun",
        cwd=tmp_path,
    )

    assert manual.returncode == 0, manual.stderr
    assert "adb.exe" not in manual.stdout
    assert serial.returncode == 0, serial.stderr
    assert "-s PICO-USB-42 shell monkey" in serial.stdout


def test_start_uses_one_detected_pico_usb_device_before_preflight(tmp_path: Path) -> None:
    """Breaking PICO model detection or USB selector choice must fail this test."""
    adb = tmp_path / "fake-adb.cmd"
    adb.write_text(
        "@echo off\n"
        "if \"%1\"==\"devices\" goto devices\n"
        "if \"%1\"==\"-d\" goto direct\n"
        "echo ADB %*\n"
        "exit /b 0\n"
        ":devices\n"
        "echo List of devices attached\n"
        "echo PICO-USB-42 device product:PICO_4 model:PICO_4 transport_id:1\n"
        "exit /b 0\n"
        ":direct\n"
        "if \"%2\"==\"get-state\" echo device\n"
        "if \"%2\"==\"get-state\" exit /b 0\n"
        "echo ADB %*\n"
        "exit /b 0\n",
        encoding="utf-8",
    )
    python = tmp_path / "fake-python.cmd"
    python.write_text("@echo off\necho PYTHON %*\n", encoding="utf-8")

    start = run_script(
        "start_pico_udp_teleop.ps1",
        "-AdbPath",
        str(adb),
        "-PythonPath",
        str(python),
        cwd=tmp_path,
    )

    assert start.returncode == 0, start.stderr
    assert "ADB -d shell monkey -p com.yezqin.kinovapicobridge" in start.stdout
    assert start.stdout.count("PYTHON -m kinova_teleop.main") == 2


def test_start_recognizes_the_authorized_a9210_adb_identity(tmp_path: Path) -> None:
    """The A9210 product codename must not force an unnecessary manual start."""
    adb = tmp_path / "a9210-adb.cmd"
    adb.write_text(
        "@echo off\n"
        "if \"%1\"==\"devices\" goto devices\n"
        "if \"%1\"==\"-d\" goto direct\n"
        "echo ADB %*\n"
        "exit /b 0\n"
        ":devices\n"
        "echo List of devices attached\n"
        "echo PICO_TEST_SERIAL_001 device product:sparrow model:A9210 device:sparrow transport_id:1\n"
        "exit /b 0\n"
        ":direct\n"
        "if \"%2\"==\"get-state\" echo device\n"
        "if \"%2\"==\"get-state\" exit /b 0\n"
        "echo ADB %*\n"
        "exit /b 0\n",
        encoding="utf-8",
    )
    python = tmp_path / "fake-python.cmd"
    python.write_text("@echo off\necho PYTHON %*\n", encoding="utf-8")

    start = run_script(
        "start_pico_udp_teleop.ps1",
        "-AdbPath",
        str(adb),
        "-PythonPath",
        str(python),
        cwd=tmp_path,
    )

    assert start.returncode == 0, start.stderr
    assert "ADB -d shell monkey -p com.yezqin.kinovapicobridge" in start.stdout
    assert "Start Kinova PICO Bridge manually" not in start.stdout + start.stderr


def test_start_falls_back_to_manual_launch_when_fake_adb_has_no_device(
    tmp_path: Path,
) -> None:
    """An empty ADB list must keep the UDP preflight usable via manual launch."""
    adb = tmp_path / "empty-adb.cmd"
    adb.write_text(
        "@echo off\n"
        "echo List of devices attached\n"
        "exit /b 0\n",
        encoding="utf-8",
    )
    python = tmp_path / "fake-python.cmd"
    python.write_text("@echo off\necho PYTHON %*\n", encoding="utf-8")

    start = run_script(
        "start_pico_udp_teleop.ps1",
        "-AdbPath",
        str(adb),
        "-PythonPath",
        str(python),
        cwd=tmp_path,
    )

    output = start.stdout + start.stderr
    assert start.returncode == 0, output
    assert "Start Kinova PICO Bridge manually" in output
    assert "shell monkey" not in output
    assert start.stdout.count("PYTHON -m kinova_teleop.main") == 2


def test_start_refuses_ambiguous_authorized_devices_without_a_serial(
    tmp_path: Path,
) -> None:
    """Choosing one of multiple authorized devices must fail rather than guess."""
    adb = tmp_path / "ambiguous-adb.cmd"
    adb.write_text(
        "@echo off\n"
        "echo List of devices attached\n"
        "echo PICO-USB-42 device product:PICO_4 model:PICO_4 transport_id:1\n"
        "echo OTHER-USB-7 device product:pixel model:Pixel_7 transport_id:2\n",
        encoding="utf-8",
    )
    python = tmp_path / "fake-python.cmd"
    python.write_text("@echo off\necho PYTHON %*\n", encoding="utf-8")

    start = run_script(
        "start_pico_udp_teleop.ps1",
        "-AdbPath",
        str(adb),
        "-PythonPath",
        str(python),
        cwd=tmp_path,
    )

    assert start.returncode != 0
    assert "PYTHON" not in start.stdout


def test_build_preserves_a_unity_test_failure_exit_code(tmp_path: Path) -> None:
    """Flattening a Unity test failure to exit 1 must fail this test."""
    unity = tmp_path / "failing-unity.cmd"
    unity.write_text("@echo off\nexit /b 17\n", encoding="utf-8")

    build = run_script(
        "build_pico_udp_bridge.ps1",
        "-UnityPath",
        str(unity),
        "-ArtifactsPath",
        str(tmp_path / "artifacts"),
        cwd=tmp_path,
    )

    assert build.returncode == 17


def test_build_preserves_an_adb_install_failure_exit_code(tmp_path: Path) -> None:
    """Flattening an ADB install failure to exit 1 must fail this test."""
    unity = tmp_path / "successful-unity.cmd"
    unity.write_text(
        "@echo off\n"
        "set \"SCRIPT_DIR=%~dp0\"\n"
        ":loop\n"
        "if \"%1\"==\"\" goto done\n"
        "if \"%1\"==\"-testResults\" (\n"
        "  >\"%~2\" echo ^<test-run result=\"Passed\" failed=\"0\" /^>\n"
        ")\n"
        "shift\n"
        "goto loop\n"
        ":done\n"
        ">\"%SCRIPT_DIR%successful.apk\" echo apk\n"
        "exit /b 0\n",
        encoding="utf-8",
    )
    adb = tmp_path / "failing-adb.cmd"
    adb.write_text(
        "@echo off\n"
        "if \"%1\"==\"devices\" goto devices\n"
        "if \"%1\"==\"-d\" goto direct\n"
        "exit /b 29\n"
        ":devices\n"
        "echo List of devices attached\n"
        "echo PICO-USB-42 device product:PICO_4 model:PICO_4 transport_id:1\n"
        "exit /b 0\n"
        ":direct\n"
        "if \"%2\"==\"get-state\" echo device\n"
        "if \"%2\"==\"get-state\" exit /b 0\n"
        "exit /b 29\n",
        encoding="utf-8",
    )
    built_apk = tmp_path / "successful.apk"

    build = run_script(
        "build_pico_udp_bridge.ps1",
        "-UnityPath",
        str(unity),
        "-AdbPath",
        str(adb),
        "-BuiltApkPath",
        str(built_apk),
        "-ArtifactsPath",
        str(tmp_path / "artifacts"),
        "-Install",
        cwd=tmp_path,
    )

    assert build.returncode == 29


def test_start_preserves_preflight_exit_code_and_prints_guidance(tmp_path: Path) -> None:
    """Terminating guidance must not replace a Python preflight exit code."""
    python = tmp_path / "failing-python.cmd"
    python.write_text("@echo off\nexit /b 23\n", encoding="utf-8")

    start = run_script(
        "start_pico_udp_teleop.ps1",
        "-ManualPicoStart",
        "-PythonPath",
        str(python),
        cwd=tmp_path,
    )

    assert start.returncode == 23
    assert "same LAN/VLAN" in start.stdout + start.stderr


def test_start_honors_an_explicit_authorized_serial_without_pico_label(
    tmp_path: Path,
) -> None:
    """Rejecting a user-selected authorized codename-only headset must fail."""
    adb = tmp_path / "codename-adb.cmd"
    adb.write_text(
        "@echo off\n"
        "if \"%1\"==\"devices\" (\n"
        "  echo List of devices attached\n"
        "  echo CODENAME-42 device product:vrheadset model:orca transport_id:1\n"
        "  exit /b 0\n"
        ")\n"
        "echo ADB %*\n",
        encoding="utf-8",
    )
    python = tmp_path / "fake-python.cmd"
    python.write_text("@echo off\necho PYTHON %*\n", encoding="utf-8")

    start = run_script(
        "start_pico_udp_teleop.ps1",
        "-AdbPath",
        str(adb),
        "-PythonPath",
        str(python),
        "-DeviceSerial",
        "CODENAME-42",
        cwd=tmp_path,
    )

    assert start.returncode == 0, start.stderr
    assert "ADB -s CODENAME-42 shell monkey" in start.stdout


def test_runtime_artifacts_are_ignored_by_git_behavior() -> None:
    """Removing an actual generated-path ignore must fail this configuration test."""
    ignored_paths = (
        "artifacts/example.apk",
        "pico_unity_bridge/Library/example",
        "Temp/example",
        "Logs/example",
        "obj/example",
        "UserSettings/example",
    )

    for path in ignored_paths:
        result = subprocess.run(
            ["git", "check-ignore", "--quiet", path],
            cwd=REPOSITORY,
            text=True,
            capture_output=True,
            check=False,
        )
        assert result.returncode == 0, f"expected git to ignore {path}: {result.stderr}"
