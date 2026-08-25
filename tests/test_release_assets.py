from __future__ import annotations

import hashlib
import json
import os
import stat
from pathlib import Path
from types import SimpleNamespace

import pytest

from kinova_teleop.release_assets import (
    ReleaseAsset,
    load_release_manifest,
    verify_release_asset,
)


RELEASE = "v0.2.0-rc.1"
DOWNLOAD_PREFIX = (
    "https://github.com/YEZQin/kinova-gen3-pico-teleop-mujoco/releases/download/"
)


def valid_payload() -> dict[str, object]:
    return {
        "schema_version": "1.0",
        "release": RELEASE,
        "assets": [
            {
                "name": "kortex_api-2.8.0.post5-py3-none-any.whl",
                "sha256": "5796425d48f0ab70c56ce9ecf06b8ab91f0b951319266e5d4c0b179cdb9e99cf",
                "size_bytes": 161265,
                "download_url": (
                    f"{DOWNLOAD_PREFIX}{RELEASE}/"
                    "kortex_api-2.8.0.post5-py3-none-any.whl"
                ),
            },
        ],
    }


def write_manifest(tmp_path: Path, payload: object) -> Path:
    manifest_path = tmp_path / "public-release-assets.json"
    manifest_path.write_text(json.dumps(payload), encoding="utf-8")
    return manifest_path


def test_load_release_manifest_accepts_the_public_release_contract(tmp_path: Path) -> None:
    manifest = load_release_manifest(write_manifest(tmp_path, valid_payload()))

    assert manifest.schema_version == "1.0"
    assert manifest.release == RELEASE
    assert manifest.assets == (
        ReleaseAsset(
            name="kortex_api-2.8.0.post5-py3-none-any.whl",
            sha256="5796425d48f0ab70c56ce9ecf06b8ab91f0b951319266e5d4c0b179cdb9e99cf",
            size_bytes=161265,
            download_url=(
                f"{DOWNLOAD_PREFIX}{RELEASE}/kortex_api-2.8.0.post5-py3-none-any.whl"
            ),
        ),
    )


@pytest.mark.parametrize(
    "name",
    (
        "kinova-pico-udp-bridge.apk",
        "unrelated-py3-none-any.whl",
        "THIRD_PARTY_NOTICES.txt",
    ),
)
def test_load_release_manifest_rejects_non_kortex_public_assets(
    tmp_path: Path, name: str
) -> None:
    payload = valid_payload()
    payload["assets"] = [
        {
            "name": name,
            "sha256": "a" * 64,
            "size_bytes": 3,
            "download_url": f"{DOWNLOAD_PREFIX}{RELEASE}/{name}",
        }
    ]

    with pytest.raises(ValueError, match="exactly the Kortex wheel"):
        load_release_manifest(write_manifest(tmp_path, payload))


def test_load_release_manifest_rejects_an_extra_public_asset(tmp_path: Path) -> None:
    payload = valid_payload()
    payload["assets"].append(  # type: ignore[union-attr]
        {
            "name": "kinova-pico-udp-bridge.apk",
            "sha256": "a" * 64,
            "size_bytes": 3,
            "download_url": f"{DOWNLOAD_PREFIX}{RELEASE}/kinova-pico-udp-bridge.apk",
        }
    )

    with pytest.raises(ValueError, match="exactly the Kortex wheel"):
        load_release_manifest(write_manifest(tmp_path, payload))


def test_load_release_manifest_rejects_duplicate_json_keys(tmp_path: Path) -> None:
    manifest_path = tmp_path / "duplicate.json"
    manifest_path.write_text(
        '{"schema_version":"1.0","schema_version":"1.0","release":"v0.2.0-rc.1","assets":[]}',
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="duplicate JSON key"):
        load_release_manifest(manifest_path)


@pytest.mark.parametrize(
    ("mutation", "error"),
    [
        (
            lambda payload: payload.update({"extra": "not allowed"}),
            "unexpected manifest fields",
        ),
        (
            lambda payload: payload["assets"][0].update({"extra": "not allowed"}),  # type: ignore[index]
            "unexpected asset fields",
        ),
        (
            lambda payload: payload["assets"][0].update({"download_url": "http://github.com/example"}),  # type: ignore[index]
            "download URL",
        ),
        (
            lambda payload: payload["assets"][0].update({"download_url": "https://example.com/a.apk"}),  # type: ignore[index]
            "download URL",
        ),
        (
            lambda payload: payload["assets"][0].update({"name": "../a.apk"}),  # type: ignore[index]
            "asset name",
        ),
        (
            lambda payload: payload["assets"][0].update({"name": "dir\\a.apk"}),  # type: ignore[index]
            "asset name",
        ),
        (
            lambda payload: payload["assets"][0].update({"sha256": "A" * 64}),  # type: ignore[index]
            "SHA-256",
        ),
        (
            lambda payload: payload["assets"][0].update({"sha256": "a" * 63}),  # type: ignore[index]
            "SHA-256",
        ),
        (
            lambda payload: payload["assets"][0].update({"size_bytes": 0}),  # type: ignore[index]
            "size_bytes",
        ),
        (
            lambda payload: payload["assets"][0].update({"size_bytes": True}),  # type: ignore[index]
            "size_bytes",
        ),
    ],
)
def test_load_release_manifest_rejects_invalid_fields(
    tmp_path: Path,
    mutation: object,
    error: str,
) -> None:
    payload = valid_payload()
    mutation(payload)  # type: ignore[operator]

    with pytest.raises(ValueError, match=error):
        load_release_manifest(write_manifest(tmp_path, payload))


def test_load_release_manifest_rejects_non_finite_json_values(tmp_path: Path) -> None:
    manifest_path = tmp_path / "non-finite.json"
    manifest_path.write_text(
        '{"schema_version":"1.0","release":"v0.2.0-rc.1","assets":NaN}',
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="non-finite"):
        load_release_manifest(manifest_path)


def expected_asset(contents: bytes, name: str = "asset.bin") -> ReleaseAsset:
    return ReleaseAsset(
        name=name,
        sha256=hashlib.sha256(contents).hexdigest(),
        size_bytes=len(contents),
        download_url=f"{DOWNLOAD_PREFIX}{RELEASE}/{name}",
    )


def test_verify_release_asset_accepts_matching_regular_file(tmp_path: Path) -> None:
    contents = b"abc"
    asset_path = tmp_path / "asset.bin"
    asset_path.write_bytes(contents)

    assert verify_release_asset(asset_path, expected_asset(contents)) is None


def test_verify_release_asset_rejects_mismatched_local_filename(tmp_path: Path) -> None:
    contents = b"abc"
    asset_path = tmp_path / "other.bin"
    asset_path.write_bytes(contents)

    with pytest.raises(ValueError, match="filename"):
        verify_release_asset(asset_path, expected_asset(contents, name="expected.bin"))


def test_verify_release_asset_rejects_symlink(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    asset_path = tmp_path / "asset.bin"
    asset_path.write_bytes(b"abc")
    monkeypatch.setattr(
        os,
        "lstat",
        lambda _: SimpleNamespace(st_mode=stat.S_IFLNK | 0o777, st_file_attributes=0),
    )

    with pytest.raises(ValueError, match="symlink or reparse"):
        verify_release_asset(asset_path, expected_asset(b"abc"))


def test_verify_release_asset_rejects_reparse_point(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    asset_path = tmp_path / "asset.bin"
    asset_path.write_bytes(b"abc")
    reparse_flag = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400)
    monkeypatch.setattr(
        os,
        "lstat",
        lambda _: SimpleNamespace(
            st_mode=stat.S_IFREG | 0o644,
            st_file_attributes=reparse_flag,
        ),
    )

    with pytest.raises(ValueError, match="symlink or reparse"):
        verify_release_asset(asset_path, expected_asset(b"abc"))


def test_verify_release_asset_rejects_wrong_size_and_hash(tmp_path: Path) -> None:
    asset_path = tmp_path / "asset.bin"
    asset_path.write_bytes(b"abc")

    with pytest.raises(ValueError, match="size"):
        verify_release_asset(asset_path, expected_asset(b"abcd"))
    with pytest.raises(ValueError, match="SHA-256"):
        verify_release_asset(asset_path, expected_asset(b"abd"))


def test_verify_release_asset_rejects_changed_file_identity(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    contents = b"abc"
    asset_path = tmp_path / "asset.bin"
    asset_path.write_bytes(contents)
    original_fstat = os.fstat
    calls = 0

    def changed_after_read(file_descriptor: int) -> os.stat_result:
        nonlocal calls
        calls += 1
        result = original_fstat(file_descriptor)
        if calls == 2:
            values = list(result)
            values[stat.ST_INO] += 1
            return os.stat_result(values)
        return result

    monkeypatch.setattr(os, "fstat", changed_after_read)

    with pytest.raises(ValueError, match="identity changed"):
        verify_release_asset(asset_path, expected_asset(contents))


def test_verify_release_asset_rejects_replaced_path_after_read(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    contents = b"abc"
    asset_path = tmp_path / "asset.bin"
    asset_path.write_bytes(contents)
    original_lstat = os.lstat
    calls = 0

    def replaced_after_read(path: Path) -> os.stat_result:
        nonlocal calls
        calls += 1
        result = original_lstat(path)
        if calls == 2:
            values = list(result)
            values[stat.ST_INO] += 1
            return os.stat_result(values)
        return result

    monkeypatch.setattr(os, "lstat", replaced_after_read)

    with pytest.raises(ValueError, match="identity changed"):
        verify_release_asset(asset_path, expected_asset(contents))
