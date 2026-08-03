from pathlib import Path


def test_environment_script_requires_explicit_existing_inputs():
    text = Path("scripts/build_kortex_offline_env.ps1").read_text(encoding="utf-8")
    for parameter in ("PythonExe", "KortexWheel", "Wheelhouse", "TargetDir"):
        assert f"[Parameter(Mandatory=$true)][string]${parameter}" in text
    assert "Test-Path -LiteralPath" in text
    assert "pip install --no-index" in text
    assert "Get-FileHash" in text
