from pathlib import Path
import json
import subprocess


def test_wide_workspace_passes_exact_observed_bounds(tmp_path):
    fake = tmp_path / 'python.ps1'
    output = tmp_path / 'argv.json'
    fake.write_text("$args | ConvertTo-Json | Set-Content -LiteralPath '" +
                    str(output).replace("'", "''") + "'\n$global:LASTEXITCODE=0\n", encoding='utf-8')
    command = ("& './scripts/start_gen3_tracker.ps1' -TrackerSerial test -RobotHost 192.0.2.10 "
               "-WideWorkspace -EnableHardware -PythonPath '" + str(fake).replace("'", "''") + "'")
    result = subprocess.run(['powershell.exe', '-NoProfile', '-Command', command],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    args = json.loads(output.read_text(encoding='utf-8-sig'))
    low, high = args.index('--workspace-min'), args.index('--workspace-max')
    assert args[low+1:low+4] == ['-10', '-10', '0']
    assert args[high+1:high+4] == ['10', '10', '10']
    assert args[args.index('--max-linear-speed')+1] == '0.05'


def test_conflicting_wide_and_explicit_bounds_are_rejected():
    result = subprocess.run(['powershell.exe', '-NoProfile', '-Command',
        "& './scripts/start_gen3_tracker.ps1' -TrackerSerial test -WideWorkspace "
        "-WorkspaceMin @(0,0,0) -PythonPath (Get-Command powershell.exe).Source"],
        capture_output=True, text=True)
    assert result.returncode != 0
    assert 'cannot be combined' in result.stderr
