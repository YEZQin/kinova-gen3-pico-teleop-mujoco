"""Public documentation contracts for the zero-to-Kinova handoff."""

from __future__ import annotations

import re
import subprocess
import json
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
    markers = re.findall(
        r"(?m)^\s*#\s*LIFECYCLE:\s*([a-z0-9-]+)\s*$|<!--\s*LIFECYCLE:\s*([a-z0-9-]+)\s*-->",
        document,
    )
    return tuple(code or comment for code, comment in markers)


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


def test_readmes_embed_equivalent_evidence_matrices() -> None:
    required_rows = ("source/code-level", "simulator", "hardware observed", "release final profile")
    for path in README_PATHS:
        tables = re.findall(r"(?ms)^\|[^\n]+\|\s*\n\|(?:\s*:?-{3,}:?\s*\|)+\s*\n(?:\|[^\n]+\|\s*\n)+", path.read_text(encoding="utf-8"))
        rows = "\n".join(tables).lower()
        for label in required_rows:
            assert label in rows, f"{path.name} is missing evidence row {label}"
        assert "offline verified" in rows and "not hardware-validated" in rows


def test_hardware_lifecycle_is_an_operator_sequence_not_inert_commands() -> None:
    expected_sequence = "grip-release -> HARDWARE-READY -> masked-password -> MOVE -> grip-clutch -> stop"
    for path in README_PATHS:
        document = path.read_text(encoding="utf-8")
        launcher = document.index("# LIFECYCLE: hardware-launcher")
        sequence = document.index(f"<!-- LAUNCHER-OPERATOR-SEQUENCE: {expected_sequence} -->")
        assert sequence < launcher, f"{path.name} must explain the launcher sequence before launch"
        lifecycle_blocks = [block for block in _powershell_blocks(document) if "LIFECYCLE:" in block]
        assert all("Write-Host" not in block for block in lifecycle_blocks)


def test_bootstrap_is_gated_by_the_release_asset_state() -> None:
    manifest = json.loads((ROOT / "release" / "public-release-assets.json").read_text(encoding="utf-8"))
    for path in README_PATHS:
        _assert_release_asset_documentation_state(path.read_text(encoding="utf-8"), manifest)


def _assert_release_asset_documentation_state(document: str, manifest: dict[str, object]) -> None:
    assets = manifest["assets"]
    assert isinstance(assets, list)
    apk = next(asset for asset in assets if asset["name"] == "kinova-pico-udp-bridge.apk")
    assert isinstance(apk, dict)
    fixture = apk["sha256"] == "a" * 64 and apk["size_bytes"] == 3
    normalized = document.lower()
    bootstrap = next(block.lower() for block in _powershell_blocks(document) if "lifecycle: bootstrap" in block.lower())
    prepublication_terms = (
        "prepublication-fixture", "not yet published", "do not run until published",
        "post-release only", "publication fixture", "not an installable binary",
        "task 6 replaces", "cannot be installed", "不能安装", "发布占位",
    )
    if fixture:
        assert "<!-- release-asset-state: prepublication-fixture -->" in normalized
        assert "v0.2.0-rc.1 is not yet published" in normalized
        assert "do not run until published" in normalized
        assert "post-release only" in bootstrap
    else:
        assert "<!-- release-asset-state: published -->" in normalized
        assert all(term not in normalized for term in prepublication_terms)


def test_real_manifest_branch_rejects_all_prepublication_wording() -> None:
    real_manifest = {
        "assets": [{
            "name": "kinova-pico-udp-bridge.apk", "sha256": "b" * 64, "size_bytes": 4,
        }]
    }
    for path in README_PATHS:
        with __import__("pytest").raises(AssertionError):
            _assert_release_asset_documentation_state(path.read_text(encoding="utf-8"), real_manifest)


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
