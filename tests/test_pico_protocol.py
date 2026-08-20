import math
import struct

import pytest

from kinova_teleop.pico_protocol import (
    PICO_DISCOVER,
    PICO_PACKET_SIZE,
    PICO_READY,
    PicoControllerFrame,
    decode_pico_packet,
    encode_pico_packet,
    sequence_delta,
    sequence_is_newer,
)


def frame(**overrides) -> PicoControllerFrame:
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


def test_protocol_constants_and_exact_layout() -> None:
    payload = encode_pico_packet(frame(trigger=0.25))
    assert PICO_DISCOVER == b"KINOVA_DISCOVER_V1"
    assert PICO_READY == b"KINOVA_READY_V1"
    assert PICO_PACKET_SIZE == 60
    assert len(payload) == 60
    assert payload[:8] == b"KINVPICO"
    assert payload[8] == 2
    assert struct.unpack_from("<I", payload, 12)[0] == 42
    assert struct.unpack_from("<Q", payload, 16)[0] == 1_234_567
    assert struct.unpack_from("<f", payload, 52)[0] == pytest.approx(0.75)
    assert struct.unpack_from("<f", payload, 56)[0] == pytest.approx(0.25)


def test_decode_accepts_legacy_v1_packet_with_zero_trigger() -> None:
    v2_payload = encode_pico_packet(frame())
    v1_payload = (
        v2_payload[:8] + b"\x01" + v2_payload[9:56]
    )
    assert len(v1_payload) == 56

    decoded = decode_pico_packet(
        v1_payload,
        received_at=2.0,
        source=("10.0.0.9", 999),
    )

    assert decoded.tracked
    assert decoded.protocol_version == 1
    assert decoded.grip == pytest.approx(0.75)
    assert decoded.trigger == 0.0


def test_decode_rejects_version_size_mismatch() -> None:
    v2_payload = encode_pico_packet(frame())
    v1_sized_with_v2_version = v2_payload[:56]
    with pytest.raises(ValueError, match="version"):
        decode_pico_packet(
            v1_sized_with_v2_version,
            received_at=1.0,
            source=("127.0.0.1", 1),
        )

    v2_sized_with_v1_version = v2_payload[:8] + b"\x01" + v2_payload[9:]
    with pytest.raises(ValueError, match="version"):
        decode_pico_packet(
            v2_sized_with_v1_version,
            received_at=1.0,
            source=("127.0.0.1", 1),
        )


def test_round_trip_preserves_controller_values_and_receive_metadata() -> None:
    decoded = decode_pico_packet(
        encode_pico_packet(frame()),
        received_at=9.5,
        source=("10.0.0.3", 4567),
    )
    assert decoded.sequence == 42
    assert decoded.protocol_version == 2
    assert decoded.tracked
    assert decoded.position == pytest.approx((0.1, 1.2, -0.3))
    assert decoded.quaternion_xyzw == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert decoded.grip == pytest.approx(0.75)
    assert decoded.received_at == 9.5
    assert decoded.source == ("10.0.0.3", 4567)


@pytest.mark.parametrize(
    ("mutator", "message"),
    [
        (lambda data: data[:-1], "60 bytes"),
        (lambda data: b"BADMAGIC" + data[8:], "magic"),
        (lambda data: data[:8] + b"\x03" + data[9:], "version"),
        (lambda data: data[:10] + b"\x01\x00" + data[12:], "reserved"),
        (lambda data: data[:9] + b"\x80" + data[10:], "flags"),
    ],
)
def test_decode_rejects_invalid_envelope(mutator, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        decode_pico_packet(
            mutator(encode_pico_packet(frame())),
            received_at=1.0,
            source=("127.0.0.1", 1),
        )


@pytest.mark.parametrize(
    "overrides",
    [
        {"position": (math.nan, 0.0, 0.0)},
        {"quaternion_xyzw": (0.0, 0.0, 0.0, 0.0)},
        {"grip": math.inf},
        {"grip": -0.01},
        {"grip": 1.01},
        {"trigger": math.inf},
        {"trigger": -0.01},
        {"trigger": 1.01},
    ],
)
def test_encode_rejects_invalid_numeric_values(overrides) -> None:
    with pytest.raises(ValueError):
        encode_pico_packet(frame(**overrides))


def test_untracked_frame_encodes_neutral_controller_payload() -> None:
    payload = encode_pico_packet(
        frame(
            tracked=False,
            position=(math.nan, 3.0, -4.0),
            quaternion_xyzw=(0.0, 0.0, 0.0, 0.0),
            grip=-1.0,
        )
    )

    assert struct.unpack_from("<8f", payload, 24) == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)


@pytest.mark.parametrize(
    ("offset", "value", "message"),
    [
        (24, math.nan, "non-finite"),
        (52, 1.01, "grip"),
        (56, 1.01, "trigger"),
        (36, 0.0, "quaternion"),
    ],
)
def test_decode_rejects_invalid_controller_values(offset: int, value: float, message: str) -> None:
    payload = bytearray(encode_pico_packet(frame()))
    if message == "quaternion":
        for quaternion_offset in (36, 40, 44, 48):
            struct.pack_into("<f", payload, quaternion_offset, value)
    else:
        struct.pack_into("<f", payload, offset, value)

    with pytest.raises(ValueError, match=message):
        decode_pico_packet(bytes(payload), received_at=1.0, source=("127.0.0.1", 1))


def test_decode_normalizes_valid_nonunit_quaternion() -> None:
    payload = bytearray(encode_pico_packet(frame()))
    struct.pack_into("<f", payload, 48, 2.0)

    decoded = decode_pico_packet(bytes(payload), received_at=1.0, source=("127.0.0.1", 1))

    assert decoded.quaternion_xyzw == pytest.approx((0.0, 0.0, 0.0, 1.0))


@pytest.mark.parametrize(
    "overrides",
    [
        {"sequence": -1},
        {"sequence": 0x1_0000_0000},
        {"source_time_us": -1},
        {"source_time_us": 0x1_0000_0000_0000_0000},
    ],
)
def test_encode_rejects_out_of_range_unsigned_fields(overrides) -> None:
    with pytest.raises(ValueError):
        encode_pico_packet(frame(**overrides))


def test_encode_rejects_non_vector_position_with_value_error() -> None:
    with pytest.raises(ValueError, match="position"):
        encode_pico_packet(frame(position=None))


def test_encode_rejects_overflowing_position_with_value_error() -> None:
    with pytest.raises(ValueError, match="position"):
        encode_pico_packet(frame(position=(10**10000, 0.0, 0.0)))


def test_encode_rejects_missing_grip_with_value_error() -> None:
    with pytest.raises(ValueError, match="grip"):
        encode_pico_packet(frame(grip=None))


def test_modular_sequence_arithmetic_accepts_wrap_and_rejects_old_values() -> None:
    assert sequence_delta(0, 0xFFFFFFFF) == 1
    assert sequence_is_newer(0, 0xFFFFFFFF)
    assert sequence_delta(8, 5) == 3
    assert not sequence_is_newer(5, 5)
    assert not sequence_is_newer(4, 5)
