"""Single-socket UDP receiver for PICO controller packets."""

from __future__ import annotations

import socket
import time
from typing import Callable

from .pico_protocol import (
    PICO_DISCOVER,
    PICO_MAGIC,
    PICO_READY,
    PicoControllerFrame,
    decode_pico_packet,
)


class PicoUdpReceiver:
    """Receive one PICO datagram at a time without a worker thread."""

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 15031,
        *,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self._clock = monotonic
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((host, port))
        self._closed = False

    @property
    def address(self) -> tuple[str, int]:
        host, port = self._socket.getsockname()
        return str(host), int(port)

    def receive_once(self, *, timeout: float | None = None) -> PicoControllerFrame | None:
        self._socket.settimeout(timeout)
        payload, source = self._socket.recvfrom(512)
        endpoint = str(source[0]), int(source[1])
        if payload == PICO_DISCOVER:
            self._socket.sendto(PICO_READY, endpoint)
            return None
        if not payload.startswith(PICO_MAGIC):
            return None
        return decode_pico_packet(
            payload,
            received_at=self._clock(),
            source=endpoint,
        )

    def close(self) -> None:
        if not self._closed:
            self._socket.close()
            self._closed = True

    def __enter__(self) -> PicoUdpReceiver:
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        self.close()
