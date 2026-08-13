"""Public documentation contracts for the zero-to-Kinova handoff."""

from __future__ import annotations

import re
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
README_PATHS = (ROOT / "README.md", ROOT / "README_CN.md")
LIFECYCLE = (
    "clone", "bootstrap", "apk-install", "pico-gate", "mujoco-finite",
    "calibration", "t0", "local-package", "offline-validation",
    "physical-checklist", "hardware-launcher", "grip-release", "move",
    "stop-troubleshooting",
)


def _powershell_blocks(document: str) -> list[str]:
    return re.findall(r"```powershell\s*\n(.*?)```", document, flags=re.DOTALL | re.IGNORECASE)


def _lifecycle_markers(document: str) -> tuple[str, ...]:
    return tuple(
        re.findall(r"^\s*#\s*LIFECYCLE:\s*([a-z0-9-]+)\s*$", "\n".join(_powershell_blocks(document)), flags=re.MULTILINE)
    )


def _lifecycle_commands(document: str) -> tuple[str, ...]:
    commands: list[str] = []
    for block in _powershell_blocks(document):
        if re.match(r"\s*#\s*LIFECYCLE:\s*[a-z0-9-]+\s*$", block.splitlines()[0]):
            commands.append(block.strip())
    return tuple(commands)


def _tracked_files() -> set[str]:
    result = subprocess.run(
        ["git", "-C", str(ROOT), "ls-files"], check=True, text=True, capture_output=True
    )
    return set(result.stdout.splitlines())


def test_readmes_have_matching_ordered_lifecycle_commands() -> None:
    documents = [path.read_text(encoding="utf-8") for path in README_PATHS]
    manifests = [_lifecycle_markers(document) for document in documents]
    assert manifests[0] == LIFECYCLE
    assert manifests[1] == LIFECYCLE
    assert manifests[0] == manifests[1]
    assert _lifecycle_commands(documents[0]) == _lifecycle_commands(documents[1])


def test_public_commands_reference_tracked_scripts_or_release_assets() -> None:
    tracked = _tracked_files()
    for path in README_PATHS:
        for command in _powershell_blocks(path.read_text(encoding="utf-8")):
            for script in re.findall(r"(?:\.\\|\.\/)(scripts[\\/][\w.-]+\.ps1)", command):
                normalized = script.replace("\\", "/")
                assert normalized in tracked, f"{path.name} references untracked script {normalized}"
    manifest = (ROOT / "release" / "public-release-assets.json").read_text(encoding="utf-8")
    for asset in ("kinova-pico-udp-bridge.apk", "kortex_api-2.8.0.post5-py3-none-any.whl"):
        assert asset in manifest


def test_public_docs_exclude_private_paths_secrets_and_laboratory_artifacts() -> None:
    forbidden = (
        r"D:\\yezq", r"results\\", r"results/", r"motion-lease\.json",
        r"reviewed-preflight\.json", r"evidence-[^\s`]+\.jsonl",
        r"KINOVA_PASSWORD\s*=\s*['\"]", r"password\s*[:=]\s*['\"]",
        r"admin\s+as\s+(?:the\s+)?password", r"copy.*(?:entire|whole|broad).*workspace",
    )
    for path in (*README_PATHS, ROOT / "docs" / "kortex-hardware-quickstart.md", ROOT / "docs" / "pico-udp-quickstart.md"):
        text = path.read_text(encoding="utf-8")
        for pattern in forbidden:
            assert re.search(pattern, text, flags=re.IGNORECASE) is None, f"{path.name}: {pattern}"


def test_evidence_licenses_and_final_profile_limit_are_explicit() -> None:
    english = (ROOT / "README.md").read_text(encoding="utf-8").lower()
    chinese = (ROOT / "README_CN.md").read_text(encoding="utf-8").lower()
    evidence = (ROOT / "docs" / "evidence-levels.md").read_text(encoding="utf-8").lower()
    notices = (ROOT / "release" / "THIRD_PARTY_NOTICES.txt").read_text(encoding="utf-8").lower()
    for document in (english, chinese):
        assert "evidence" in document
        assert "v0.2.0-rc.1" in document
        assert "offline verified" in document
        assert "not hardware-validated" in document
    assert "final expanded" in evidence and "offline verified" in evidence
    assert "kinova kortex api sdk" in notices and "pico openxr sdk" in notices


def test_packaging_metadata_points_to_public_handoff() -> None:
    metadata = (ROOT / "pyproject.toml").read_text(encoding="utf-8")
    assert "guarded Kortex hardware teleoperation" in metadata
    for name in ('Repository', 'Documentation', 'Issues', 'Releases'):
        assert f'{name} =' in metadata
