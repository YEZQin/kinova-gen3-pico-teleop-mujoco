"""Serial-bound, fail-closed XRoboToolkit Motion Tracker input.

No robot connection is made here. Native SDK loading and initialization occur
only on the first read. A valid sample is not authorization to move hardware.
"""
from __future__ import annotations

import importlib
import time
import sys
from dataclasses import dataclass
from numbers import Integral
from typing import Callable

import numpy as np


@dataclass(frozen=True)
class TrackerSample:
    position: np.ndarray
    timestamp_ns: int
    valid: bool
    reason: str
    time_source: str = 'device_motion'

    def __post_init__(self) -> None:
        position = np.array(self.position, dtype=float, copy=True)
        position.setflags(write=False)
        object.__setattr__(self, "position", position)


class MotionTrackerInput:
    """Read one explicitly named tracker; never select an available substitute.

    Startup and recovery require a baseline followed by a timestamp advance.
    Repeated frames are cached for at most ``stale_timeout`` local seconds.
    Consumers must disarm on invalid samples and explicitly rearm after recovery.
    The injected SDK follows the native module's init/close lifecycle.
    """

    def __init__(
        self, serial: str, sdk=None,
        monotonic: Callable[[], float] = time.monotonic,
        stale_timeout: float = 0.2,
    ) -> None:
        if not isinstance(serial, str) or not serial.strip():
            raise ValueError("An explicit tracker serial is required")
        if not np.isfinite(stale_timeout) or stale_timeout <= 0:
            raise ValueError("stale_timeout must be finite and positive")
        self.serial = serial
        self._sdk = sdk
        self._monotonic = monotonic
        self._stale_timeout = float(stale_timeout)
        self._initialized = False
        self._closed = False
        self._timestamp: int | None = None
        self._advanced_at: float | None = None
        self._sample: TrackerSample | None = None
        self._time_source = 'device_motion'

    def _invalid(self, reason: str) -> TrackerSample:
        self._timestamp = None
        self._advanced_at = None
        self._sample = None
        return TrackerSample(np.zeros(3), 0, False, reason)

    @staticmethod
    def _positive_timestamp(value) -> int:
        if isinstance(value, bool) or not isinstance(value, Integral) or value <= 0:
            raise ValueError("Motion timestamp must be a positive integer")
        return int(value)

    def _snapshot(self) -> tuple[int, np.ndarray]:
        if hasattr(self._sdk, 'get_motion_snapshot'):
            frame = self._sdk.get_motion_snapshot()
            age = self._monotonic() - frame['received_ns'] / 1e9
            if not 0 <= age < self._stale_timeout:
                raise ValueError('Motion packet reception is stale')
            source = frame['time_source']
            if source != self._time_source:
                self._invalid('Timestamp clock changed; awaiting new baseline')
                self._time_source = source
            serials = frame['serials']
            if serials.count(self.serial) != 1:
                raise ValueError('Required tracker serial missing or ambiguous')
            return self._positive_timestamp(frame['timestamp_ns']), np.array(frame['poses'][serials.index(self.serial)][:3])
        before = self._positive_timestamp(self._sdk.get_motion_timestamp_ns())
        count = self._sdk.num_motion_data_available()
        serials = list(self._sdk.get_motion_tracker_serial_numbers())
        poses = np.asarray(self._sdk.get_motion_tracker_pose(), dtype=float)
        after = self._positive_timestamp(self._sdk.get_motion_timestamp_ns())
        if before != after:
            raise ValueError("Motion frame changed during snapshot")
        if (isinstance(count, bool) or not isinstance(count, Integral)
                or count <= 0 or count != len(serials) or poses.shape != (count, 7)):
            raise ValueError("Tracker count, serial list and pose rows disagree")
        if any(not isinstance(serial, str) for serial in serials):
            raise ValueError("Tracker serials must be strings")
        if len(set(serials)) != len(serials) or self.serial not in serials:
            raise ValueError("Required tracker serial missing or ambiguous")
        pose = poses[serials.index(self.serial)]
        if not np.isfinite(pose).all() or not np.any(pose[3:] != 0):
            raise ValueError("Tracker pose must be finite with a nonzero quaternion")
        return after, pose[:3].copy()

    def read(self) -> TrackerSample:
        if self._closed:
            return self._invalid("Tracker input is closed")
        try:
            if not self._initialized:
                if self._sdk is None:
                    if sys.platform == 'win32':
                        from .motion_packets import PacketMotionSdk
                        self._sdk = PacketMotionSdk(serial=self.serial)
                    else:
                        self._sdk = importlib.import_module("xrobotoolkit_sdk")
                self._sdk.init()
                self._initialized = True
            timestamp, position = self._snapshot()
            now = float(self._monotonic())
            if not np.isfinite(now):
                return self._invalid("Local monotonic clock is invalid")
        except Exception as error:
            return self._invalid(f"Tracker input unavailable: {type(error).__name__}: {error}")
        return self._accept_frame(timestamp, position, now)

    def _accept_frame(self, timestamp: int, position: np.ndarray, now: float) -> TrackerSample:
        if self._advanced_at is not None:
            elapsed = now - self._advanced_at
            if elapsed < 0 or elapsed >= self._stale_timeout:
                return self._invalid("Tracker data is stale or local clock moved backwards")
        if self._timestamp is None or timestamp < self._timestamp:
            self._timestamp = timestamp
            self._advanced_at = now
            self._sample = TrackerSample(position, timestamp, False, "Awaiting advancing tracker frame", self._time_source)
        elif timestamp > self._timestamp:
            self._timestamp = timestamp
            self._advanced_at = now
            self._sample = TrackerSample(position, timestamp, True, "ok", self._time_source)
        # A repeated timestamp cannot update position or refresh local freshness.
        assert self._sample is not None
        return self._sample

    def close(self) -> None:
        """Release initialized native input once; never reopen after closing."""
        if self._closed:
            return
        self._closed = True
        self._invalid("Tracker input is closed")
        if self._initialized:
            self._sdk.close()
