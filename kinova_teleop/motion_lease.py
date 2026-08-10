"""Read-only validation of a supervisor-issued Gen3 motion lease."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from datetime import datetime
import json
from pathlib import Path
import re
import stat
from typing import Any, cast


_LEASE_FIELDS = frozenset(("lease_id", "device", "run_id", "owner", "acquired_utc"))
_LEASE_ID = re.compile(r"[0-9a-f]{32}\Z")
_SAFE_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}\Z")
_UTC = re.compile(r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d{1,6})?Z\Z")
_WINDOWS_REPARSE_POINT = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0)


@dataclass(frozen=True, slots=True)
class MotionLease:
    lease_id: str
    device: str
    run_id: str
    owner: str
    acquired_utc: str

    def __post_init__(self) -> None:
        if _LEASE_ID.fullmatch(self.lease_id) is None:
            raise ValueError("lease_id must be exactly 32 lowercase hexadecimal characters")
        if self.device != "gen3":
            raise ValueError("device must be gen3")
        for value, label in ((self.run_id, "run_id"), (self.owner, "owner")):
            if not isinstance(value, str) or _SAFE_ID.fullmatch(value) is None:
                raise ValueError(f"{label} must be a path-safe identifier")
        if not isinstance(self.acquired_utc, str) or _UTC.fullmatch(self.acquired_utc) is None:
            raise ValueError("acquired_utc must be a UTC ISO 8601 timestamp ending in Z")
        try:
            datetime.fromisoformat(self.acquired_utc[:-1] + "+00:00")
        except ValueError as error:
            raise ValueError("acquired_utc must be a valid UTC timestamp") from error

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any]) -> "MotionLease":
        if not isinstance(payload, Mapping):
            raise ValueError("motion lease must be a mapping")
        actual = set(payload)
        missing = _LEASE_FIELDS - actual
        extra = actual - _LEASE_FIELDS
        if missing:
            raise ValueError(f"motion lease missing required fields: {', '.join(sorted(missing))}")
        if extra:
            raise ValueError(f"motion lease has unexpected fields: {', '.join(sorted(map(str, extra)))}")
        return cls(
            lease_id=cast(str, payload["lease_id"]),
            device=cast(str, payload["device"]),
            run_id=cast(str, payload["run_id"]),
            owner=cast(str, payload["owner"]),
            acquired_utc=cast(str, payload["acquired_utc"]),
        )

    def to_mapping(self) -> dict[str, str]:
        return {
            "lease_id": self.lease_id,
            "device": self.device,
            "run_id": self.run_id,
            "owner": self.owner,
            "acquired_utc": self.acquired_utc,
        }


def _reject_constant(value: str) -> None:
    raise ValueError(f"non-finite JSON constant is not allowed: {value}")


def _unique_pairs(pairs: list[tuple[str, object]]) -> dict[str, object]:
    output: dict[str, object] = {}
    for key, value in pairs:
        if key in output:
            raise ValueError("duplicate JSON key is not allowed")
        output[key] = value
    return output


def validate_motion_lease(path: str | Path, run_id: str, owner: str) -> MotionLease:
    """Validate an existing lease without acquiring, changing, or deleting it."""

    candidate = Path(path)
    try:
        metadata = candidate.lstat()
    except OSError as error:
        raise ValueError("motion lease file is unavailable") from error
    is_reparse_point = bool(
        getattr(metadata, "st_file_attributes", 0) & _WINDOWS_REPARSE_POINT
    )
    if candidate.is_symlink() or is_reparse_point or not candidate.is_file():
        raise ValueError("motion lease must be a regular file")
    try:
        raw = candidate.read_bytes()
    except OSError as error:
        raise ValueError("motion lease file cannot be read") from error
    try:
        payload = json.loads(
            raw.decode("utf-8"),
            parse_constant=_reject_constant,
            object_pairs_hook=_unique_pairs,
        )
        lease = MotionLease.from_mapping(payload)
    except (UnicodeError, json.JSONDecodeError, TypeError, ValueError) as error:
        if isinstance(error, ValueError) and str(error).startswith(("motion lease", "lease_id", "device", "run_id", "owner", "acquired_utc")):
            raise
        raise ValueError("motion lease JSON is invalid") from error
    if lease.device != "gen3":
        raise ValueError("motion lease device must be gen3")
    if lease.run_id != run_id:
        raise ValueError("motion lease run_id does not match")
    if lease.owner != owner:
        raise ValueError("motion lease owner does not match")
    # Keep a read-only contract visible to callers and make accidental mutation
    # of the path observable during debugging without writing it.
    _ = metadata
    return lease


load_motion_lease = validate_motion_lease
