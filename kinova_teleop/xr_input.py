"""XR controller input adapters.

The production adapter deliberately imports XRoboToolkit lazily so that the
MuJoCo dry run and the automated tests do not require its native extension.
"""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import math
import time
from typing import Callable, Protocol

import numpy as np


@dataclass(frozen=True)
class ControllerSample:
    """One left-controller observation in XR tracking coordinates."""

    position: np.ndarray
    quaternion_xyzw: np.ndarray
    grip: float
    timestamp_ns: int
    received_monotonic: float
    valid: bool = True


class XrInputSource(Protocol):
    def read(self) -> ControllerSample:
        """Return the newest available controller sample."""

    def close(self) -> None:
        """Release source resources. Implementations must be idempotent."""


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

        return ControllerSample(
            position=position,
            quaternion_xyzw=quaternion,
            grip=grip,
            timestamp_ns=timestamp_ns,
            received_monotonic=self._monotonic(),
            valid=valid,
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
