"""Fail-closed admission checks for live hardware motion input."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable

import numpy as np

from .xr_input import ControllerSample, XrInputSource


_GRIP_RELEASE_THRESHOLD = 0.8
_POLL_INTERVAL_S = 0.01


class InputAdmissionError(RuntimeError):
    """Raised when live input is not safe to admit for hardware motion."""


@dataclass(frozen=True)
class InputAdmissionResult:
    """Evidence collected before hardware motion is admitted."""

    accepted_samples: int
    last_timestamp_ns: int
    grip_released: bool
    active_source: tuple[str, int] | None
    foreign_count: int | None = None


def confirm_move(*, input_fn: Callable[[str], str] = input) -> None:
    """Require an exact, deliberate operator confirmation before motion."""

    if input_fn("Type MOVE to enable Kortex motion: ") != "MOVE":
        raise InputAdmissionError("hardware motion requires exact MOVE confirmation")


def wait_for_fresh_released_input(
    source: XrInputSource,
    *,
    sample_count: int = 3,
    timeout_s: float = 1.0,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> InputAdmissionResult:
    """Require a fresh, stable run of released controller observations.

    This function observes only the supplied source.  It deliberately does
    not open, close, or otherwise interact with the robot transport.
    """

    _validate_window_arguments(sample_count, timeout_s)
    deadline = monotonic() + timeout_s
    previous_timestamp: int | None = None
    active_source: tuple[str, int] | None = None
    foreign_count: int | None = None
    accepted_samples = 0

    while accepted_samples < sample_count:
        if monotonic() >= deadline:
            raise InputAdmissionError("timed out waiting for fresh controller input")

        candidate = _read_sample(source)
        if previous_timestamp is None and _is_initial_stale_placeholder(candidate):
            sleep(_POLL_INTERVAL_S)
            continue
        sample = _validate_finite_sample(candidate)
        timestamp = _sample_timestamp(sample)
        if float(sample.grip) >= _GRIP_RELEASE_THRESHOLD:
            raise InputAdmissionError("Grip must remain released during admission")

        observed_source, observed_foreign = _observe_source_health(source)
        if observed_source is not None or observed_foreign is not None:
            if observed_source is None:
                raise InputAdmissionError("active controller source is unavailable")
            if active_source is None:
                active_source = observed_source
                foreign_count = observed_foreign
            elif observed_source != active_source:
                raise InputAdmissionError("controller source changed during admission")
            elif observed_foreign != foreign_count:
                raise InputAdmissionError("foreign packet count changed during admission")

        if previous_timestamp is not None:
            if timestamp < previous_timestamp:
                raise InputAdmissionError("controller timestamps must not regress")
            if timestamp == previous_timestamp:
                sleep(_POLL_INTERVAL_S)
                continue

        previous_timestamp = timestamp
        accepted_samples += 1

        if accepted_samples < sample_count:
            sleep(_POLL_INTERVAL_S)

    return InputAdmissionResult(
        accepted_samples=accepted_samples,
        last_timestamp_ns=previous_timestamp,
        grip_released=True,
        active_source=active_source,
        foreign_count=foreign_count,
    )


def verify_released_now(
    source: XrInputSource,
    *,
    after_timestamp_ns: int,
    admission: InputAdmissionResult | None = None,
    timeout_s: float = 1.0,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> ControllerSample:
    """Require a newer released sample immediately after confirmation."""

    _validate_timestamp(after_timestamp_ns, name="after_timestamp_ns")
    _validate_timeout(timeout_s)
    deadline = monotonic() + timeout_s
    while monotonic() < deadline:
        sample = _read_finite_sample(source)
        _verify_source_continuity(source, admission)
        if float(sample.grip) >= _GRIP_RELEASE_THRESHOLD:
            raise InputAdmissionError("Grip must remain released after confirmation")
        timestamp = _sample_timestamp(sample)
        if timestamp < after_timestamp_ns:
            raise InputAdmissionError("controller timestamp regressed after admission")
        if timestamp == after_timestamp_ns:
            sleep(_POLL_INTERVAL_S)
            continue
        return sample
    raise InputAdmissionError("timed out waiting for a newer controller sample")


def _validate_window_arguments(sample_count: int, timeout_s: float) -> None:
    if isinstance(sample_count, bool) or not isinstance(sample_count, int) or sample_count <= 0:
        raise ValueError("sample_count must be a positive integer")
    _validate_timeout(timeout_s)


def _validate_timeout(timeout_s: float) -> None:
    if not math.isfinite(timeout_s) or timeout_s <= 0.0:
        raise ValueError("timeout_s must be positive and finite")


def _read_sample(source: XrInputSource) -> ControllerSample:
    try:
        return source.read()
    except Exception as error:
        raise InputAdmissionError("unable to read controller input") from error


def _is_initial_stale_placeholder(sample: ControllerSample) -> bool:
    """Recognize only the UDP adapter's pre-first-packet sentinel."""

    try:
        return (
            sample.valid is False
            and sample.timestamp_ns == 0
            and sample.invalid_reason == "stream is stale"
        )
    except AttributeError:
        return False


def _validate_finite_sample(sample: ControllerSample) -> ControllerSample:
    try:
        position = np.asarray(sample.position, dtype=np.float64)
        quaternion = np.asarray(sample.quaternion_xyzw, dtype=np.float64)
        grip = float(sample.grip)
        valid = bool(sample.valid)
    except (AttributeError, TypeError, ValueError, OverflowError) as error:
        raise InputAdmissionError("controller sample must be finite valid input") from error
    if (
        not valid
        or position.shape != (3,)
        or quaternion.shape != (4,)
        or not np.isfinite(position).all()
        or not np.isfinite(quaternion).all()
        or not math.isfinite(grip)
    ):
        raise InputAdmissionError("controller sample must be finite valid input")
    _sample_timestamp(sample)
    return sample


def _read_finite_sample(source: XrInputSource) -> ControllerSample:
    return _validate_finite_sample(_read_sample(source))


def _sample_timestamp(sample: ControllerSample) -> int:
    try:
        return _validate_timestamp(sample.timestamp_ns, name="controller timestamp")
    except (AttributeError, TypeError, ValueError, OverflowError) as error:
        raise InputAdmissionError("controller timestamp must be finite") from error


def _validate_timestamp(value: object, *, name: str) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be an integer")
    timestamp = float(value)
    if not math.isfinite(timestamp) or not timestamp.is_integer():
        raise ValueError(f"{name} must be a finite integer")
    return int(timestamp)


def _observe_source_health(
    source: XrInputSource,
) -> tuple[tuple[str, int] | None, int | None]:
    health_reader = getattr(source, "health", None)
    if not callable(health_reader):
        return None, None
    try:
        health = health_reader()
        active_source = health.active_source
        foreign_count = health.foreign
    except Exception as error:
        raise InputAdmissionError("controller health is unavailable") from error
    if active_source is not None and not isinstance(active_source, tuple):
        raise InputAdmissionError("active controller source is unavailable")
    if isinstance(foreign_count, bool) or not isinstance(foreign_count, int) or foreign_count < 0:
        raise InputAdmissionError("controller health is unavailable")
    return active_source, foreign_count


def _verify_source_continuity(
    source: XrInputSource,
    admission: InputAdmissionResult | None,
) -> None:
    has_health = callable(getattr(source, "health", None))
    if has_health and admission is None:
        raise InputAdmissionError(
            "prior admission is required for controller health recheck"
        )
    active_source, foreign_count = _observe_source_health(source)
    if active_source is None and has_health:
        raise InputAdmissionError("active controller source is unavailable")
    if admission is None:
        return
    if active_source != admission.active_source:
        raise InputAdmissionError("controller source changed after admission")
    if foreign_count != admission.foreign_count:
        raise InputAdmissionError("foreign packet count changed after admission")
