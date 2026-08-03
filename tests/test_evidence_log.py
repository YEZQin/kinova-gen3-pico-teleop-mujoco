from __future__ import annotations

import json
import math

import pytest

from kinova_teleop.evidence_log import EvidenceLogger


def test_evidence_logger_redacts_secret_keys(tmp_path):
    path = tmp_path / "events.jsonl"
    logger = EvidenceLogger(path, run_id="g-001")
    with pytest.raises(ValueError, match="secret field"):
        logger.event("connected", {"KINOVA_PASSWORD": "never-write-this"})
    assert not path.exists() or "never-write-this" not in path.read_text()


def test_evidence_logger_writes_schema_compatible_records(tmp_path):
    path = tmp_path / "events.jsonl"
    logger = EvidenceLogger(path, run_id="g-001", monotonic_ns=lambda: 10)
    logger.event("connected", "CONNECTED_READ_ONLY", {"attempt": 1})
    record = json.loads(path.read_text(encoding="utf-8"))
    assert record["schema_version"] == "1.0"
    assert record["run_id"] == "g-001"
    assert record["kind"] == "connected"
    assert record["state"] == "CONNECTED_READ_ONLY"


def test_evidence_logger_monotonic_order_is_strict(tmp_path):
    values = iter((3, 3))
    logger = EvidenceLogger(tmp_path / "events.jsonl", run_id="g-001", monotonic_ns=lambda: next(values))
    logger.event("connected", "CONNECTED_READ_ONLY", {})
    logger.event("preflight_passed", "PREFLIGHT_PASSED", {})
    records = [json.loads(line) for line in logger.path.read_text().splitlines()]
    assert records[1]["monotonic_ns"] > records[0]["monotonic_ns"]


def test_evidence_logger_rejects_invalid_event_and_payload(tmp_path):
    logger = EvidenceLogger(tmp_path / "events.jsonl", run_id="g-001")
    with pytest.raises(ValueError, match="unsupported event kind"):
        logger.event("not-a-kind", "CONNECTED_READ_ONLY", {})
    with pytest.raises(ValueError, match="unsupported trial state"):
        logger.event("connected", "not-a-state", {})
    with pytest.raises(TypeError):
        logger.event("connected", "CONNECTED_READ_ONLY", [])
    with pytest.raises(ValueError, match="non-JSON"):
        logger.event("connected", "CONNECTED_READ_ONLY", {"obj": object()})
    with pytest.raises(ValueError, match="finite"):
        logger.event("connected", "CONNECTED_READ_ONLY", {"value": math.nan})


def test_evidence_logger_sample_stream_and_secret_values(tmp_path):
    logger = EvidenceLogger(tmp_path / "events.jsonl", run_id="g-001")
    record = logger.sample({"position": [0.0, 0.0, 0.3]})
    assert record["kind"] == "sample"
    assert logger.samples_path.exists()
    with pytest.raises(ValueError, match="secret field"):
        logger.sample({"access_token": "secret"})
