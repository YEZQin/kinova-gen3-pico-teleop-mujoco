"""PICO UDP input adapter with source locking and a stale-data watchdog."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable

import numpy as np

from .pico_protocol import PicoControllerFrame, sequence_delta, sequence_is_newer
from .pico_receiver import PicoUdpReceiver
from .xr_input import ControllerSample


_MAX_DRAIN_ATTEMPTS = 1024


@dataclass(frozen=True)
class PicoStreamHealth:
    active_source: tuple[str, int] | None
    accepted: int
    rejected: int
    dropped: int
    foreign: int
    stale: bool
    last_error: str


class PicoUdpInput:
    """Convert newest safe PICO UDP data into :class:`ControllerSample`."""

    def __init__(
        self,
        receiver: PicoUdpReceiver | None = None,
        *,
        host: str = "0.0.0.0",
        port: int = 15031,
        stale_after: float = 0.2,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        if not math.isfinite(stale_after) or stale_after <= 0.0:
            raise ValueError("stale_after must be positive and finite")
        self._receiver = (
            receiver
            if receiver is not None
            else PicoUdpReceiver(host, port, monotonic=monotonic)
        )
        self._stale_after = float(stale_after)
        self._clock = monotonic
        self._active_source: tuple[str, int] | None = None
        self._last_sequence: int | None = None
        self._last_valid: PicoControllerFrame | None = None
        self._accepted = 0
        self._rejected = 0
        self._dropped = 0
        self._foreign = 0
        self._last_error = ""
        self._closed = False
        self._stale_boundary_reported = False

    def read(self) -> ControllerSample:
        now = self._clock()
        recovering_from_stale = (
            self._last_valid is not None and self._is_stale(now)
        )
        emit_stale_boundary = (
            recovering_from_stale and not self._stale_boundary_reported
        )
        if emit_stale_boundary:
            self._stale_boundary_reported = True
        had_invalid = emit_stale_boundary

        for _ in range(_MAX_DRAIN_ATTEMPTS):
            try:
                frame = self._receiver.receive_once(timeout=0.0)
            except BlockingIOError:
                break
            except OSError as error:
                self._rejected += 1
                self._last_error = str(error)
                had_invalid = True
                break
            except ValueError as error:
                self._rejected += 1
                self._last_error = str(error)
                had_invalid = True
                continue

            if frame is None:
                continue
            if not self._accept(
                frame,
                recovering_from_stale=recovering_from_stale,
            ):
                had_invalid = True
        else:
            self._last_error = "receiver drain limit reached"
            had_invalid = True

        now = self._clock()
        stale = self._is_stale(now)
        if stale and not self._last_error:
            self._last_error = "stream is stale"
        if had_invalid or stale or self._last_valid is None:
            return self._inactive_sample(now)
        return self._sample_from(self._last_valid)

    def health(self) -> PicoStreamHealth:
        return PicoStreamHealth(
            active_source=self._active_source,
            accepted=self._accepted,
            rejected=self._rejected,
            dropped=self._dropped,
            foreign=self._foreign,
            stale=self._is_stale(self._clock()),
            last_error=self._last_error,
        )

    def close(self) -> None:
        if not self._closed:
            self._receiver.close()
            self._closed = True

    def _accept(
        self,
        frame: PicoControllerFrame,
        *,
        recovering_from_stale: bool,
    ) -> bool:
        session_boundary = False
        if self._active_source is not None and frame.source != self._active_source:
            if (
                not recovering_from_stale
                and frame.source[0] != self._active_source[0]
            ):
                self._foreign += 1
                return True
            session_boundary = not recovering_from_stale
            self._clear_session()

        if not frame.tracked:
            self._rejected += 1
            self._last_error = "controller is untracked"
            return False

        if self._last_sequence is not None:
            if not sequence_is_newer(frame.sequence, self._last_sequence):
                restarted_same_endpoint = (
                    recovering_from_stale
                    and self._last_valid is not None
                    and frame.source == self._active_source
                    and frame.sequence < self._last_sequence
                    and frame.source_time_us < self._last_valid.source_time_us
                )
                if restarted_same_endpoint:
                    self._last_sequence = None
                    self._last_valid = None
                else:
                    self._rejected += 1
                    self._last_error = "duplicate or out-of-order sequence"
                    return False
            if self._last_sequence is not None:
                self._dropped += (
                    sequence_delta(frame.sequence, self._last_sequence) - 1
                )

        self._active_source = frame.source
        self._last_sequence = frame.sequence
        self._last_valid = frame
        self._accepted += 1
        self._last_error = ""
        self._stale_boundary_reported = False
        return not session_boundary

    def _clear_session(self) -> None:
        self._active_source = None
        self._last_sequence = None
        self._last_valid = None

    def _is_stale(self, now: float) -> bool:
        return self._last_valid is None or now - self._last_valid.received_at > self._stale_after

    @staticmethod
    def _inactive_sample(now: float) -> ControllerSample:
        return ControllerSample(
            position=np.zeros(3, dtype=np.float64),
            quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
            grip=0.0,
            timestamp_ns=0,
            received_monotonic=now,
            valid=False,
            trigger=0.0,
        )

    @staticmethod
    def _sample_from(frame: PicoControllerFrame) -> ControllerSample:
        return ControllerSample(
            position=np.array(frame.position, dtype=np.float64, copy=True),
            quaternion_xyzw=np.array(frame.quaternion_xyzw, dtype=np.float64, copy=True),
            grip=float(frame.grip),
            timestamp_ns=frame.source_time_us * 1000,
            received_monotonic=frame.received_at,
            valid=True,
            trigger=float(frame.trigger),
        )
