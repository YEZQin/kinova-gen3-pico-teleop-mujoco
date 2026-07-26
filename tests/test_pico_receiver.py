import socket

import pytest

from kinova_teleop.pico_protocol import (
    PICO_DISCOVER,
    PICO_READY,
    PicoControllerFrame,
    encode_pico_packet,
)
from kinova_teleop.pico_receiver import PicoUdpReceiver


def make_frame(**overrides) -> PicoControllerFrame:
    values = {
        "sequence": 42,
        "tracked": True,
        "source_time_us": 1_234_567,
        "position": (0.1, 1.2, -0.3),
        "quaternion_xyzw": (0.0, 0.0, 0.0, 1.0),
        "grip": 0.75,
        "received_at": 0.0,
        "source": ("192.168.1.4", 34000),
    }
    values.update(overrides)
    return PicoControllerFrame(**values)


def test_receiver_replies_to_discovery_and_does_not_return_a_frame() -> None:
    with PicoUdpReceiver("127.0.0.1", 0) as receiver:
        address = receiver.address
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.settimeout(1.0)
            sender.sendto(PICO_DISCOVER, address)
            assert receiver.receive_once(timeout=1.0) is None
            reply, source = sender.recvfrom(128)
    assert reply == PICO_READY
    assert source == address


def test_receiver_decodes_binary_frame_with_local_source_and_time() -> None:
    with PicoUdpReceiver("127.0.0.1", 0, monotonic=lambda: 7.0) as receiver:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(encode_pico_packet(make_frame()), receiver.address)
            sender_port = sender.getsockname()[1]
            result = receiver.receive_once(timeout=1.0)
    assert result is not None
    assert result.received_at == 7.0
    assert result.source == ("127.0.0.1", sender_port)


def test_receiver_ignores_unrelated_ascii_and_propagates_bad_protocol() -> None:
    with PicoUdpReceiver("127.0.0.1", 0) as receiver:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(b"unrelated", receiver.address)
            assert receiver.receive_once(timeout=1.0) is None
            sender.sendto(b"KINVPICO", receiver.address)
            with pytest.raises(ValueError, match="56 or 60 bytes"):
                receiver.receive_once(timeout=1.0)
