"""XR controller input adapters.

The production adapter deliberately imports XRoboToolkit lazily so that the
MuJoCo dry run and the automated tests do not require its native extension.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import importlib
import math
import threading
import time
from typing import Callable, Protocol

import numpy as np


_MAX_PENDING_INVALIDS = 256
_RECEIVER_STOP_GRACE_S = 0.1
_RECEIVER_INTERRUPT_GRACE_S = 1.0


@dataclass(frozen=True)
class ControllerSample:
    """One left-controller observation in XR tracking coordinates."""

    position: np.ndarray
    quaternion_xyzw: np.ndarray
    grip: float
    timestamp_ns: int
    received_monotonic: float
    valid: bool = True
    trigger: float = 0.0
    trigger_available: bool = True
    invalid_reason: str = ""


class XrInputSource(Protocol):
    def read(self) -> ControllerSample:
        """Return promptly with the newest available controller sample."""

    def close(self) -> None:
        """Release source resources. Implementations must be idempotent."""


class ContinuousInputBuffer:
    """Continuously ingest a prompt, close-interruptible input source."""

    def __init__(
        self,
        source: XrInputSource,
        *,
        poll_interval_s: float = 0.005,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        if not math.isfinite(poll_interval_s) or poll_interval_s <= 0.0:
            raise ValueError("poll_interval_s must be positive and finite")
        self._source = source
        self._poll_interval_s = float(poll_interval_s)
        self._monotonic = monotonic
        self._source_lock = threading.Lock()
        self._sample_lock = threading.Lock()
        self._close_lock = threading.Lock()
        self._stop = threading.Event()
        self._latest: ControllerSample | None = None
        self._pending_invalids: deque[ControllerSample] = deque()
        self._error: Exception | None = None
        self._closed = False
        self._thread = threading.Thread(
            target=self._receive_loop,
            name="pico-input-receiver",
            daemon=True,
        )
        try:
            self._thread.start()
        except BaseException as start_error:
            self._stop.set()
            try:
                self._source.close()
            except Exception as close_error:
                start_error.add_note(
                    f"input cleanup also failed: {close_error!r}"
                )
            self._closed = True
            raise

    def read(self) -> ControllerSample:
        with self._sample_lock:
            error = self._error
            if self._pending_invalids:
                sample = self._pending_invalids.popleft()
            else:
                sample = self._latest
        if error is not None:
            raise RuntimeError("continuous controller input failed") from error
        if sample is None:
            return ControllerSample(
                position=np.zeros(3, dtype=np.float64),
                quaternion_xyzw=np.array(
                    [0.0, 0.0, 0.0, 1.0],
                    dtype=np.float64,
                ),
                grip=0.0,
                timestamp_ns=0,
                received_monotonic=self._monotonic(),
                valid=False,
                trigger=0.0,
                trigger_available=False,
                invalid_reason="stream is stale",
            )
        return ControllerSample(
            position=np.array(sample.position, dtype=np.float64, copy=True),
            quaternion_xyzw=np.array(
                sample.quaternion_xyzw,
                dtype=np.float64,
                copy=True,
            ),
            grip=float(sample.grip),
            timestamp_ns=int(sample.timestamp_ns),
            received_monotonic=float(sample.received_monotonic),
            valid=bool(sample.valid),
            trigger=float(sample.trigger),
            trigger_available=bool(sample.trigger_available),
            invalid_reason=str(sample.invalid_reason),
        )

    def health(self):
        with self._source_lock:
            health_reader = getattr(self._source, "health", None)
            if not callable(health_reader):
                raise RuntimeError("controller health is unavailable")
            return health_reader()

    def close(self) -> None:
        with self._close_lock:
            if self._closed:
                return
            self._stop.set()
            self._thread.join(timeout=_RECEIVER_STOP_GRACE_S)
            if self._thread.is_alive():
                try:
                    self._source.close()
                except Exception as close_error:
                    raise RuntimeError(
                        "continuous controller input could not be interrupted"
                    ) from close_error
                self._thread.join(timeout=_RECEIVER_INTERRUPT_GRACE_S)
                if self._thread.is_alive():
                    raise RuntimeError("continuous controller input did not stop")
            else:
                with self._source_lock:
                    self._source.close()
            self._closed = True

    def _receive_loop(self) -> None:
        while not self._stop.is_set():
            try:
                with self._source_lock:
                    sample = self._source.read()
            except Exception as error:
                with self._sample_lock:
                    self._latest = None
                    self._error = error
                self._stop.set()
                return
            with self._sample_lock:
                if not sample.valid and self._queue_new_invalid(sample):
                    if len(self._pending_invalids) >= _MAX_PENDING_INVALIDS:
                        self._latest = None
                        self._error = RuntimeError(
                            "continuous controller invalid queue overflow"
                        )
                        self._stop.set()
                        return
                    self._pending_invalids.append(sample)
                self._latest = sample
            self._stop.wait(self._poll_interval_s)

    def _queue_new_invalid(self, sample: ControllerSample) -> bool:
        if not self._pending_invalids:
            return True
        previous = self._pending_invalids[-1]
        return (
            previous.timestamp_ns != sample.timestamp_ns
            or previous.invalid_reason != sample.invalid_reason
        )


class SdkXrInput:
    """Adapter for the XRoboToolkit ``xrobotoolkit_sdk`` Python module."""

    def __init__(
        self,
        sdk: object | None = None,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        if sdk is None:
            try:
                sdk = importlib.import_module("xrobotoolkit_sdk")
            except (ImportError, OSError) as exc:
                raise RuntimeError(
                    "xrobotoolkit_sdk is unavailable. Follow README.md to build "
                    "the XRoboToolkit Python SDK in WSL."
                ) from exc

        self._sdk = sdk
        self._monotonic = monotonic
        self._closed = False
        try:
            self._sdk.init()
        except Exception as exc:
            raise RuntimeError(
                "xrobotoolkit_sdk.init() failed. Check PC Service, "
                "PXREASetting.ini, firewall, and native library dependencies."
            ) from exc

    def read(self) -> ControllerSample:
        raw = np.asarray(
            self._sdk.get_left_controller_pose(),
            dtype=np.float64,
        )
        valid = bool(raw.shape == (7,) and np.isfinite(raw).all())
        if valid:
            position = raw[:3].copy()
            quaternion = raw[3:].copy()
            quaternion_norm = float(np.linalg.norm(quaternion))
            valid = quaternion_norm > 1e-12
        if not valid:
            position = np.zeros(3, dtype=np.float64)
            quaternion = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)

        try:
            grip = float(self._sdk.get_left_grip())
            timestamp_ns = int(self._sdk.get_time_stamp_ns())
        except (TypeError, ValueError, OverflowError):
            grip = 0.0
            timestamp_ns = 0
            valid = False

        if not math.isfinite(grip):
            grip = 0.0
            valid = False

        # The trigger channel is optional; missing or bad values degrade to
        # 0.0 without invalidating the pose sample.
        trigger = 0.0
        trigger_getter = getattr(self._sdk, "get_left_trigger", None)
        if trigger_getter is not None:
            try:
                trigger = float(trigger_getter())
            except (TypeError, ValueError, OverflowError):
                trigger = 0.0
            if not math.isfinite(trigger):
                trigger = 0.0
            trigger = min(1.0, max(0.0, trigger))

        return ControllerSample(
            position=position,
            quaternion_xyzw=quaternion,
            grip=grip,
            timestamp_ns=timestamp_ns,
            received_monotonic=self._monotonic(),
            valid=valid,
            trigger=trigger,
        )

    def close(self) -> None:
        if not self._closed:
            self._sdk.close()
            self._closed = True


class DryRunXrInput:
    """Deterministic synthetic input for tests and installation checks."""

    def __init__(self, control_hz: float = 100.0) -> None:
        if not math.isfinite(control_hz) or control_hz <= 0.0:
            raise ValueError("control_hz must be positive and finite")
        self._control_hz = float(control_hz)
        self._step = 0

    def read(self) -> ControllerSample:
        self._step += 1
        elapsed = self._step / self._control_hz
        angle = 0.08 * math.sin(0.7 * elapsed)
        half_angle = 0.5 * angle
        position = np.array(
            [
                0.025 * math.sin(0.8 * elapsed),
                0.018 * math.sin(0.5 * elapsed),
                0.012 * (math.cos(0.6 * elapsed) - 1.0),
            ],
            dtype=np.float64,
        )
        quaternion_xyzw = np.array(
            [0.0, math.sin(half_angle), 0.0, math.cos(half_angle)],
            dtype=np.float64,
        )

        # The initial release tests the clutch edge; a short periodic release
        # exercises hold and re-clutch in longer headless runs.
        phase = self._step % 500
        grip = 1.0 if self._step > 10 and not 400 <= phase < 420 else 0.0
        return ControllerSample(
            position=position,
            quaternion_xyzw=quaternion_xyzw,
            grip=grip,
            timestamp_ns=int(round(elapsed * 1_000_000_000)),
            received_monotonic=elapsed,
            valid=True,
        )

    def close(self) -> None:
        return None
