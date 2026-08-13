"""Append-only, schema-compatible trial evidence for the Gen3 adapter."""

from __future__ import annotations

from collections.abc import Callable, Mapping
import json
import os
from pathlib import Path
import re
import time
from typing import Any


EventSink = Callable[[str, str, Mapping[str, object]], None]

_EVENT_KINDS = frozenset(
    {
        "connected",
        "preflight_passed",
        "lease_acquired",
        "armed",
        "control_started",
        "moving",
        "input_release",
        "input_stale",
        "command_rejected",
        "workspace_rejected",
        "anchor_rejected",
        "host_stop_requested",
        "stop_rpc_returned",
        "controller_cancel_confirmed",
        "stop_unconfirmed",
        "motion_command_completed",
        "device_stationary_confirmed",
        "physical_stop_observed",
        "no_motion_verified",
        "protective_stop",
        "intervention",
        "timeout",
        "faulted",
        "cleanup_completed",
        "attachment_registered",
        "completed",
    }
)
_TRIAL_STATES = frozenset(
    {
        "DISCONNECTED",
        "CONNECTED_READ_ONLY",
        "PREFLIGHT_PASSED",
        "ARMED",
        "MOVING",
        "STOPPING",
        "FAULTED",
        "COMPLETED",
    }
)
_DEFAULT_STATES = {
    "connected": "CONNECTED_READ_ONLY",
    "preflight_passed": "PREFLIGHT_PASSED",
    "lease_acquired": "ARMED",
    "armed": "ARMED",
    "control_started": "MOVING",
    "moving": "MOVING",
    "input_release": "STOPPING",
    "input_stale": "STOPPING",
    "command_rejected": "STOPPING",
    "workspace_rejected": "STOPPING",
    "anchor_rejected": "STOPPING",
    "host_stop_requested": "STOPPING",
    "stop_rpc_returned": "STOPPING",
    "controller_cancel_confirmed": "STOPPING",
    "stop_unconfirmed": "FAULTED",
    "motion_command_completed": "MOVING",
    "device_stationary_confirmed": "STOPPING",
    "physical_stop_observed": "STOPPING",
    "no_motion_verified": "CONNECTED_READ_ONLY",
    "protective_stop": "FAULTED",
    "intervention": "FAULTED",
    "timeout": "STOPPING",
    "faulted": "FAULTED",
    "cleanup_completed": "COMPLETED",
    "attachment_registered": "COMPLETED",
    "completed": "COMPLETED",
}

_SECRET_KEY = re.compile(r"(?:password|passwd|secret|token|credential|api[_-]?key)", re.I)


def _reject_secret_keys(value: object, path: str = "payload") -> None:
    if isinstance(value, Mapping):
        for key, item in value.items():
            if isinstance(key, str) and _SECRET_KEY.search(key):
                # Never include the key's value in the exception text.
                raise ValueError(f"secret field is not permitted: {path}.{key}")
            _reject_secret_keys(item, f"{path}.{key}")
    elif isinstance(value, (tuple, list)):
        for index, item in enumerate(value):
            _reject_secret_keys(item, f"{path}[{index}]")


def reject_secret_keys(payload: Mapping[str, object]) -> None:
    """Reject password-like keys before serialisation."""

    if not isinstance(payload, Mapping):
        raise TypeError("payload must be a mapping")
    _reject_secret_keys(payload)


def _json_safe(value: object) -> object:
    if value is None or isinstance(value, (str, bool, int, float)):
        if isinstance(value, float) and not (value == value and abs(value) != float("inf")):
            raise ValueError("payload numbers must be finite")
        return value
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_safe(item) for item in value]
    raise ValueError("payload contains a non-JSON value")


class EvidenceLogger:
    """Write LF-terminated JSONL records and fsync each append."""

    def __init__(
        self,
        events_path: str | Path,
        *,
        run_id: str,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        samples_path: str | Path | None = None,
    ) -> None:
        if not isinstance(run_id, str) or not run_id:
            raise ValueError("run_id must be a non-empty string")
        self.path = Path(events_path)
        self.events_path = self.path
        self.samples_path = Path(samples_path) if samples_path is not None else self.path.with_name("samples.jsonl")
        self.run_id = run_id
        self._monotonic_ns = monotonic_ns
        self._last_monotonic_ns: int | None = None

    def _next_monotonic_ns(self) -> int:
        try:
            candidate = int(self._monotonic_ns())
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError("monotonic clock must return an integer") from error
        if candidate < 0:
            raise ValueError("monotonic clock must be non-negative")
        if self._last_monotonic_ns is not None and candidate <= self._last_monotonic_ns:
            candidate = self._last_monotonic_ns + 1
        self._last_monotonic_ns = candidate
        return candidate

    @staticmethod
    def _append(path: Path, record: Mapping[str, object]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        encoded = (json.dumps(record, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")
        with path.open("ab") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())

    def event(
        self,
        kind: str,
        state: str | Mapping[str, object],
        payload: Mapping[str, object] | None = None,
    ) -> dict[str, object]:
        """Append one event.

        For compatibility with early callers, ``event(kind, payload)`` uses a
        schema-defined default state; production callers should pass all three
        arguments explicitly.
        """

        if kind not in _EVENT_KINDS:
            raise ValueError(f"unsupported event kind: {kind}")
        if payload is None:
            if not isinstance(state, Mapping):
                raise TypeError("event payload must be a mapping")
            payload = state
            state_value = _DEFAULT_STATES[kind]
        else:
            if not isinstance(state, str):
                raise TypeError("event state must be a string")
            state_value = state
        if state_value not in _TRIAL_STATES:
            raise ValueError(f"unsupported trial state: {state_value}")
        reject_secret_keys(payload)
        safe_payload = _json_safe(payload)
        assert isinstance(safe_payload, dict)
        record: dict[str, object] = {
            "schema_version": "1.0",
            "run_id": self.run_id,
            "monotonic_ns": self._next_monotonic_ns(),
            "kind": kind,
            "state": state_value,
            "payload": safe_payload,
        }
        self._append(self.events_path, record)
        return dict(record)

    def sample(
        self,
        sample: Mapping[str, object],
        *,
        kind: str = "sample",
    ) -> dict[str, object]:
        """Append a controller/feedback sample to a separate JSONL stream."""

        reject_secret_keys(sample)
        safe_payload = _json_safe(sample)
        assert isinstance(safe_payload, dict)
        record: dict[str, object] = {
            "schema_version": "1.0",
            "run_id": self.run_id,
            "monotonic_ns": self._next_monotonic_ns(),
            "kind": kind,
            "payload": safe_payload,
        }
        self._append(self.samples_path, record)
        return dict(record)


def logger_event_sink(logger: EvidenceLogger) -> EventSink:
    """Return the callback shape consumed by KortexBackend/TeleopController."""

    return lambda kind, state, payload: logger.event(kind, state, payload)
