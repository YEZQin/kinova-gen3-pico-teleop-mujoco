from __future__ import annotations

import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
UNITY_PROJECT = ROOT / "pico_unity_bridge"
ANDROID_NS = "{http://schemas.android.com/apk/res/android}"
PICO_SDK_COMMIT = "3aa3e62bff41df618529eeb60ff02c29a515dafe"


def test_unity_manifest_pins_l20_verified_openxr_stack() -> None:
    manifest = json.loads(
        (UNITY_PROJECT / "Packages/manifest.json").read_text(encoding="utf-8")
    )
    deps = manifest["dependencies"]

    assert deps["com.unity.xr.management"] == "4.4.0"
    assert deps["com.unity.xr.openxr"] == "1.8.2"
    assert deps["com.unity.xr.openxr.picoxr"].endswith(f"#{PICO_SDK_COMMIT}")


def test_packages_lock_resolves_the_pinned_pico_commit() -> None:
    package_lock = json.loads(
        (UNITY_PROJECT / "Packages/packages-lock.json").read_text(encoding="utf-8")
    )
    pico = package_lock["dependencies"]["com.unity.xr.openxr.picoxr"]

    assert pico["source"] == "git"
    assert pico["hash"] == PICO_SDK_COMMIT
    assert pico["version"].endswith(f"#{PICO_SDK_COMMIT}")


def test_project_version_is_the_verified_unity_editor() -> None:
    project_version = (
        UNITY_PROJECT / "ProjectSettings/ProjectVersion.txt"
    ).read_text(encoding="utf-8")
    match = re.search(r"^m_EditorVersion:\s*(\S+)\s*$", project_version, re.MULTILINE)

    assert match is not None
    assert match.group(1) == "2022.3.62f3c1"


def test_android_manifest_exposes_only_the_bridge_application() -> None:
    root = ET.parse(
        UNITY_PROJECT / "Assets/Plugins/Android/AndroidManifest.xml"
    ).getroot()
    uses_permissions = {
        item.attrib[f"{ANDROID_NS}name"] for item in root.findall("uses-permission")
    }
    application = root.find("application")

    assert "android.permission.INTERNET" in uses_permissions
    assert application is not None
    activities = application.findall("activity")
    assert len(activities) == 1
    assert (
        activities[0].attrib[f"{ANDROID_NS}name"]
        == "com.unity3d.player.UnityPlayerActivity"
    )
