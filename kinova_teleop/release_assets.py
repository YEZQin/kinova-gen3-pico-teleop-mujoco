"""Strict, offline validation for public release asset manifests."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path, PureWindowsPath
import re
import stat
from typing import Any


GITHUB_RELEASE_PREFIX = (
    "https://github.com/YEZQin/kinova-gen3-pico-teleop-mujoco/releases/download/"
)
_MANIFEST_FIELDS = frozenset({"schema_version", "release", "assets"})
_ASSET_FIELDS = frozenset({"name", "sha256", "size_bytes", "download_url"})
_SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")
_RELEASE_PATTERN = re.compile(r"v[0-9]+\.[0-9]+\.[0-9]+(?:-[0-9A-Za-z.]+)?")
_READ_CHUNK_BYTES = 1024 * 1024


@dataclass(frozen=True, slots=True)
class ReleaseAsset:
    """An immutable declaration of a downloadable public release asset."""

    name: str
    sha256: str
    size_bytes: int
    download_url: str


@dataclass(frozen=True, slots=True)
class ReleaseManifest:
    """An immutable public release manifest."""

    schema_version: str
    release: str
    assets: tuple[ReleaseAsset, ...]


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_non_finite_json_value(value: str) -> None:
    raise ValueError(f"non-finite JSON value: {value}")


def _validate_exact_fields(
    value: object, expected: frozenset[str], label: str
) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a JSON object")
    fields = frozenset(value)
    unexpected = fields - expected
    if unexpected:
        raise ValueError(f"unexpected {label} fields: {sorted(unexpected)}")
    missing = expected - fields
    if missing:
        raise ValueError(f"missing {label} fields: {sorted(missing)}")
    return value


def _validate_release(release: object) -> str:
    if not isinstance(release, str) or not _RELEASE_PATTERN.fullmatch(release):
        raise ValueError("release must be a versioned GitHub release tag")
    return release


def _validate_asset_name(name: object) -> str:
    if not isinstance(name, str) or not name:
        raise ValueError("asset name must be a non-empty filename")
    if (
        Path(name).name != name
        or PureWindowsPath(name).name != name
        or name in {".", ".."}
    ):
        raise ValueError("asset name must not contain a path traversal or separator")
    return name


def _validate_asset(
    value: object, release: str | None = None
) -> ReleaseAsset:
    fields = _validate_exact_fields(value, _ASSET_FIELDS, "asset")
    name = _validate_asset_name(fields["name"])
    sha256 = fields["sha256"]
    if not isinstance(sha256, str) or not _SHA256_PATTERN.fullmatch(sha256):
        raise ValueError("SHA-256 must be exactly 64 lowercase hexadecimal characters")
    size_bytes = fields["size_bytes"]
    if isinstance(size_bytes, bool) or not isinstance(size_bytes, int) or size_bytes <= 0:
        raise ValueError("size_bytes must be a positive integer")
    download_url = fields["download_url"]
    if not isinstance(download_url, str):
        raise ValueError("download URL must be an HTTPS GitHub release URL")

    if release is None:
        relative_url = download_url.removeprefix(GITHUB_RELEASE_PREFIX)
        url_release, separator, url_name = relative_url.partition("/")
        if (
            not separator
            or not _RELEASE_PATTERN.fullmatch(url_release)
            or url_name != name
            or download_url != f"{GITHUB_RELEASE_PREFIX}{url_release}/{name}"
        ):
            raise ValueError("download URL must be an HTTPS GitHub release URL")
    elif download_url != f"{GITHUB_RELEASE_PREFIX}{release}/{name}":
        raise ValueError("download URL must match the GitHub release asset URL")

    return ReleaseAsset(name, sha256, size_bytes, download_url)


def load_release_manifest(path: Path) -> ReleaseManifest:
    """Load a manifest without permitting ambiguous JSON or extra fields."""
    try:
        source = path.read_bytes().decode("utf-8")
    except UnicodeDecodeError as error:
        raise ValueError("release manifest must be valid UTF-8") from error
    try:
        payload = json.loads(
            source,
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_non_finite_json_value,
        )
    except json.JSONDecodeError as error:
        raise ValueError("release manifest must contain valid JSON") from error

    fields = _validate_exact_fields(payload, _MANIFEST_FIELDS, "manifest")
    schema_version = fields["schema_version"]
    if schema_version != "1.0":
        raise ValueError("schema_version must be 1.0")
    release = _validate_release(fields["release"])
    raw_assets = fields["assets"]
    if not isinstance(raw_assets, list) or not raw_assets:
        raise ValueError("assets must be a non-empty JSON array")
    assets = tuple(_validate_asset(asset, release) for asset in raw_assets)
    if len({asset.name for asset in assets}) != len(assets):
        raise ValueError("asset names must be unique")
    return ReleaseManifest(schema_version, release, assets)


def _is_symlink_or_reparse(file_stat: os.stat_result) -> bool:
    reparse_flag = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0)
    attributes = getattr(file_stat, "st_file_attributes", 0) or 0
    return stat.S_ISLNK(file_stat.st_mode) or bool(attributes & reparse_flag)


def _file_identity(file_stat: os.stat_result) -> tuple[int, int]:
    return file_stat.st_dev, file_stat.st_ino


def verify_release_asset(path: Path, asset: ReleaseAsset) -> None:
    """Verify a regular file without following links or accepting file swaps."""
    if not isinstance(asset, ReleaseAsset):
        raise TypeError("asset must be a ReleaseAsset")
    _validate_asset(
        {
            "name": asset.name,
            "sha256": asset.sha256,
            "size_bytes": asset.size_bytes,
            "download_url": asset.download_url,
        }
    )
    if path.name != asset.name:
        raise ValueError("release asset filename does not match the manifest")
    before_open = os.lstat(path)
    if _is_symlink_or_reparse(before_open):
        raise ValueError("release asset must not be a symlink or reparse point")
    if not stat.S_ISREG(before_open.st_mode):
        raise ValueError("release asset must be a regular file")

    open_flags = os.O_RDONLY | getattr(os, "O_BINARY", 0)
    if hasattr(os, "O_NOFOLLOW"):
        open_flags |= os.O_NOFOLLOW
    descriptor = os.open(path, open_flags)
    try:
        opened = os.fstat(descriptor)
        if _is_symlink_or_reparse(opened) or not stat.S_ISREG(opened.st_mode):
            raise ValueError("release asset must be a regular non-reparse file")
        if _file_identity(before_open) != _file_identity(opened):
            raise ValueError("release asset identity changed before verification")
        if opened.st_size != asset.size_bytes:
            raise ValueError("release asset size does not match the manifest")

        digest = hashlib.sha256()
        bytes_read = 0
        while chunk := os.read(descriptor, _READ_CHUNK_BYTES):
            bytes_read += len(chunk)
            digest.update(chunk)

        after_read = os.fstat(descriptor)
        if _file_identity(opened) != _file_identity(after_read):
            raise ValueError("release asset identity changed during verification")
        after_path = os.lstat(path)
        if _is_symlink_or_reparse(after_path):
            raise ValueError("release asset identity changed during verification")
        if _file_identity(opened) != _file_identity(after_path):
            raise ValueError("release asset identity changed during verification")
        if after_read.st_size != asset.size_bytes or bytes_read != asset.size_bytes:
            raise ValueError("release asset size does not match the manifest")
        if digest.hexdigest() != asset.sha256:
            raise ValueError("release asset SHA-256 does not match the manifest")
    finally:
        os.close(descriptor)
