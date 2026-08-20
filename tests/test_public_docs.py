"""Public documentation contracts for the zero-to-Kinova handoff."""

from __future__ import annotations

import re
import subprocess
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
README_PATHS = (ROOT / "README.md", ROOT / "README_CN.md")
ADVANCED_GUIDE = ROOT / "docs" / "advanced-gripper-teleoperation.md"
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


def test_readmes_link_to_the_separate_advanced_gripper_guide() -> None:
    """The public first-motion path stays gripper-free; advanced use is separate."""
    english, chinese = (path.read_text(encoding="utf-8") for path in README_PATHS)

    assert "[Advanced PICO gripper teleoperation](docs/advanced-gripper-teleoperation.md)" in english
    assert "[高级 PICO 夹爪遥操](docs/advanced-gripper-teleoperation.md)" in chinese


def test_advanced_gripper_guide_documents_the_guarded_parameterized_contract() -> None:
    """A public advanced command must remain artifact-bound and safety-qualified."""
    document = ADVANCED_GUIDE.read_text(encoding="utf-8")

    for required in (
        "-MotionLease",
        "-PreflightReport",
        "-OperatorCalibration",
        "-EnableGripper",
        "-Scale 1.0",
        "-MaxLinearSpeed 0.05",
        "PICO V2",
        "MOVE",
        "Trigger `0.0`",
        "Trigger `1.0`",
        "Grip",
        "projection",
        "clamp",
        "physical E-stop",
        "Web Stop",
        "not hardware-observed",
        "not hardware-validated",
    ):
        assert required in document, required

    assert re.search(r"\$workspaceMin\s*=\s*@\(", document)
    assert re.search(r"\$workspaceMax\s*=\s*@\(", document)
    assert "start_gen3_pico_teleop.ps1" in document

    for required in (
        "Grip activation establishes an anchor frame only",
        "sends neither an arm motion command nor a gripper command",
        "Proportional Trigger commands begin only on subsequent active samples",
        "Grip release holds the last/current gripper command",
        "recoverable stale input, workspace boundary projection/contact, and shutdown/cleanup",
        "send no automatic gripper command and hold the current gripper position",
        "No automatic open occurs in any of these cases",
        "For the first gripper trial, use no payload, begin with a small Trigger value, and keep the physical E-stop and Web Stop reachable.",
        "首次夹爪试验必须空载，从较小的 Trigger 值开始，并保持实体 E-stop 与 Web Stop 可达。",
    ):
        assert required in document, required

    forbidden = (
        r"D:[\\/]", r"C:[\\/]Users", r"KINOVA_PASSWORD\s*=",
        r"download[^\n]*apk", r"hardware-observed evidence",
    )
    for pattern in forbidden:
        assert re.search(pattern, document, flags=re.IGNORECASE) is None, pattern


def test_bootstrap_is_gated_by_the_release_asset_state() -> None:
    manifest = json.loads((ROOT / "release" / "public-release-assets.json").read_text(encoding="utf-8"))
    for path in README_PATHS:
        _assert_release_asset_documentation_state(path.read_text(encoding="utf-8"), manifest)


def _assert_release_asset_documentation_state(document: str, manifest: dict[str, object]) -> None:
    assets = manifest["assets"]
    assert isinstance(assets, list)
    assert [asset["name"] for asset in assets] == [
        "kortex_api-2.8.0.post5-py3-none-any.whl"
    ]
    normalized = document.lower()
    bootstrap = next(block.lower() for block in _powershell_blocks(document) if "lifecycle: bootstrap" in block.lower())
    prepublication_terms = (
        "prepublication-fixture", "uninstallable", "not yet published",
        "do not run until published", "post-release only", "publication fixture",
        "not an installable binary", "task 6 replaces", "cannot be installed",
        "task 6 将以真实资产替换", "不能安装", "发布占位", "未发布",
    )
    assert "<!-- release-asset-state: published -->" in normalized
    assert re.search(r"(?<!device-)(?<!device/)(?<!network/)fixture\b", normalized) is None
    assert all(term not in normalized for term in prepublication_terms)
    assert "installapk" not in bootstrap and "apkpath" not in bootstrap


def _published_document(document: str) -> str:
    state_and_warning = r"<!-- RELEASE-ASSET-STATE: prepublication-fixture -->\n\n[^\n]*\n"
    published = re.sub(
        state_and_warning,
        "<!-- RELEASE-ASSET-STATE: published -->\n",
        document,
    )
    return re.sub(r"^# POST-RELEASE ONLY:.*\n", "", published, flags=re.MULTILINE)


def test_real_manifest_branch_accepts_clean_published_readmes() -> None:
    real_manifest = {
        "assets": [{
            "name": "kortex_api-2.8.0.post5-py3-none-any.whl",
            "sha256": "b" * 64,
            "size_bytes": 4,
        }]
    }
    for path in README_PATHS:
        _assert_release_asset_documentation_state(
            _published_document(path.read_text(encoding="utf-8")), real_manifest
        )


@pytest.mark.parametrize(
    "stale_phrase",
    (
        "fixture", "uninstallable", "Task 6 replaces this fixture",
        "not yet published", "do not run until published", "post-release only",
        "publication fixture", "not an installable binary", "cannot be installed",
        "Task 6 将以真实资产替换", "发布占位", "不能安装", "未发布",
    ),
)
def test_real_manifest_branch_rejects_each_stale_prepublication_phrase(stale_phrase: str) -> None:
    real_manifest = {
        "assets": [{
            "name": "kortex_api-2.8.0.post5-py3-none-any.whl",
            "sha256": "b" * 64,
            "size_bytes": 4,
        }]
    }
    for path in README_PATHS:
        published = _published_document(path.read_text(encoding="utf-8"))
        with pytest.raises(AssertionError):
            _assert_release_asset_documentation_state(f"{published}\n{stale_phrase}\n", real_manifest)


def test_public_commands_reference_tracked_scripts_or_release_assets() -> None:
    tracked = _tracked_files()
    for path in README_PATHS:
        for command in _powershell_blocks(path.read_text(encoding="utf-8")):
            for script in re.findall(r"(?:\.\\|\.\/)(scripts[\\/][\w.-]+\.ps1)", command):
                normalized = script.replace("\\", "/")
                assert normalized in tracked, f"{path.name} references untracked script {normalized}"
    manifest = (ROOT / "release" / "public-release-assets.json").read_text(encoding="utf-8")
    assert "kortex_api-2.8.0.post5-py3-none-any.whl" in manifest
    assert ".apk" not in manifest.lower()


def test_public_docs_build_the_nonredistributed_apk_locally() -> None:
    source_url = "https://github.com/pico-developer/pico-unity-openxr-sdk/tree/"
    license_url = "https://github.com/pico-developer/pico-unity-openxr-sdk/blob/"
    for path in README_PATHS:
        document = path.read_text(encoding="utf-8")
        normalized = document.lower()
        apk_block = next(
            block.lower()
            for block in _powershell_blocks(document)
            if "lifecycle: apk-install" in block.lower()
        )
        assert "build_pico_udp_bridge.ps1" in apk_block
        assert "bootstrap_public_teleop.ps1" not in apk_block
        assert "2022.3.62f3c1" in document
        assert "apk intentionally not redistributed" in normalized
        assert "build locally" in normalized
        assert source_url in normalized and license_url in normalized
        assert "open source license" not in normalized
        assert re.search(r"releases/download/[^\s)]+\.apk", normalized) is None


def test_publication_design_and_plan_have_no_stale_apk_release_contract() -> None:
    internal_docs = (
        ROOT / "docs" / "superpowers" / "specs" / "2026-08-13-public-zero-to-kinova-teleop-design.md",
        ROOT / "docs" / "superpowers" / "plans" / "2026-08-13-public-zero-to-kinova-teleop.md",
    )
    stale_terms = (
        "supplied APK",
        "APK and Kortex wheel are release assets",
        "exact APK hash/size",
        "four verified assets",
    )
    for path in internal_docs:
        document = path.read_text(encoding="utf-8")
        for stale_term in stale_terms:
            assert stale_term not in document, f"{path.name}: {stale_term}"


def test_public_docs_exclude_private_paths_secrets_and_laboratory_artifacts() -> None:
    forbidden = (
        r"D:\\yezq", r"results\\", r"results/", r"motion-lease\.json",
        r"reviewed-preflight\.json", r"evidence-[^\s`]+\.jsonl",
        r"KINOVA_PASSWORD\s*=\s*['\"]", r"password\s*[:=]\s*['\"]",
        r"admin\s+as\s+(?:the\s+)?password", r"copy.*(?:entire|whole|broad).*workspace",
    )
    for path in (
        *README_PATHS,
        ADVANCED_GUIDE,
        ROOT / "docs" / "kortex-hardware-quickstart.md",
        ROOT / "docs" / "pico-udp-quickstart.md",
    ):
        text = path.read_text(encoding="utf-8")
        for pattern in forbidden:
            assert re.search(pattern, text, flags=re.IGNORECASE) is None, f"{path.name}: {pattern}"


def test_all_tracked_markdown_excludes_developer_machine_paths() -> None:
    for relative_path in _tracked_files():
        if not relative_path.lower().endswith(".md"):
            continue
        text = (ROOT / relative_path).read_text(encoding="utf-8")
        assert re.search(r"(?:D:[\\/]+yezq|C:[\\/]+Users[\\/]+qqxx)", text, re.IGNORECASE) is None, relative_path


def test_public_source_archive_excludes_internal_plans() -> None:
    for internal_path in (
        "docs/superpowers/specs/2026-08-10-gen3-pico-kortex-hardware-teleop-design.md",
        ".superpowers/sdd/2026-08-10-gen3-pico-kortex-hardware-completion/final-fix-report.md",
    ):
        result = subprocess.run(
            ["git", "-C", str(ROOT), "check-attr", "export-ignore", "--", internal_path],
            check=True,
            text=True,
            capture_output=True,
        )
        assert result.stdout.rstrip().endswith(": export-ignore: set")


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
