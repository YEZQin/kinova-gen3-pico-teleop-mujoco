from __future__ import annotations

import json

import pytest

from kinova_teleop.motion_lease import validate_motion_lease


def _write(path, **overrides):
    payload = {
        "lease_id": "a" * 32,
        "device": "gen3",
        "run_id": "run-1",
        "owner": "operator-1",
        "acquired_utc": "2026-08-03T00:00:00Z",
    }
    payload.update(overrides)
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_validate_motion_lease_requires_matching_gen3_run_and_owner(tmp_path):
    path = tmp_path / "motion.lock"
    _write(path)
    lease = validate_motion_lease(path, "run-1", "operator-1")
    assert lease.device == "gen3"
    with pytest.raises(ValueError, match="run_id"):
        validate_motion_lease(path, "other", "operator-1")
    with pytest.raises(ValueError, match="owner"):
        validate_motion_lease(path, "run-1", "other")


def test_validate_motion_lease_is_read_only(tmp_path):
    path = tmp_path / "motion.lock"
    _write(path)
    before = path.read_bytes()
    validate_motion_lease(path, "run-1", "operator-1")
    assert path.read_bytes() == before


@pytest.mark.parametrize(
    "payload",
    [
        {},
        {"lease_id": "bad"},
        {"lease_id": "a" * 32, "device": "ur5e", "run_id": "run-1", "owner": "operator-1", "acquired_utc": "2026-08-03T00:00:00Z"},
        {"lease_id": "a" * 32, "device": "gen3", "run_id": "run-1", "owner": "operator-1", "acquired_utc": "not-a-date"},
    ],
)
def test_validate_motion_lease_rejects_invalid_payloads(tmp_path, payload):
    path = tmp_path / "motion.lock"
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError):
        validate_motion_lease(path, "run-1", "operator-1")


def test_validate_motion_lease_rejects_non_file_and_invalid_json(tmp_path):
    with pytest.raises(ValueError, match="regular file"):
        validate_motion_lease(tmp_path, "run-1", "operator-1")
    path = tmp_path / "motion.lock"
    path.write_text("{", encoding="utf-8")
    with pytest.raises(ValueError, match="invalid"):
        validate_motion_lease(path, "run-1", "operator-1")
