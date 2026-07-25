from __future__ import annotations

from dataclasses import dataclass
import math
import struct

PICO_DISCOVER = b"KINOVA_DISCOVER_V1"
PICO_READY = b"KINOVA_READY_V1"
PICO_MAGIC = b"KINVPICO"
PICO_VERSION = 1
TRACKED_FLAG = 0x01
_PACKET = struct.Struct("<8sBBHIQ8f")
PICO_PACKET_SIZE = _PACKET.size


@dataclass(frozen=True)
class PicoControllerFrame:
    sequence: int
    tracked: bool
    source_time_us: int
    position: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    grip: float
    received_at: float
    source: tuple[str, int]


def sequence_delta(candidate: int, previous: int) -> int:
    return (int(candidate) - int(previous)) & 0xFFFFFFFF


def sequence_is_newer(candidate: int, previous: int) -> bool:
    delta = sequence_delta(candidate, previous)
    return 0 < delta < 0x80000000


def _validate_unsigned(value: int, maximum: int, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or not 0 <= value <= maximum:
        raise ValueError(f"{name} must be an unsigned integer in range")
    return value


def _validate_vector(values: object, length: int, name: str) -> tuple[float, ...]:
    try:
        vector = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{name} must contain {length} numeric values") from error
    if len(vector) != length:
        raise ValueError(f"{name} must contain {length} values")
    if not all(math.isfinite(value) for value in vector):
        raise ValueError(f"{name} must contain only finite values")
    return vector


def _validate_frame_values(frame: PicoControllerFrame) -> tuple[tuple[float, float, float], tuple[float, float, float, float], float]:
    _validate_unsigned(frame.sequence, 0xFFFFFFFF, "sequence")
    _validate_unsigned(frame.source_time_us, 0xFFFFFFFFFFFFFFFF, "source_time_us")
    if not isinstance(frame.tracked, bool):
        raise ValueError("tracked must be a bool")
    if not frame.tracked:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0.0

    position = _validate_vector(frame.position, 3, "position")
    quaternion = _validate_vector(frame.quaternion_xyzw, 4, "quaternion_xyzw")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm == 0.0:
        raise ValueError("quaternion_xyzw must have nonzero norm")
    try:
        grip = float(frame.grip)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("grip must be a finite value within [0, 1]") from error
    if not math.isfinite(grip) or not 0.0 <= grip <= 1.0:
        raise ValueError("grip must be finite and within [0, 1]")
    return position, quaternion, grip


def encode_pico_packet(frame: PicoControllerFrame) -> bytes:
    position, quaternion, grip = _validate_frame_values(frame)
    flags = TRACKED_FLAG if frame.tracked else 0
    return _PACKET.pack(
        PICO_MAGIC,
        PICO_VERSION,
        flags,
        0,
        frame.sequence,
        frame.source_time_us,
        *position,
        *quaternion,
        grip,
    )


def decode_pico_packet(
    payload: bytes, *, received_at: float, source: tuple[str, int]
) -> PicoControllerFrame:
    if len(payload) != PICO_PACKET_SIZE:
        raise ValueError(f"PICO packet must be exactly {PICO_PACKET_SIZE} bytes")
    magic, version, flags, reserved, sequence, source_time_us, *values = _PACKET.unpack(payload)
    if magic != PICO_MAGIC:
        raise ValueError("invalid PICO packet magic")
    if version != PICO_VERSION:
        raise ValueError("unsupported PICO packet version")
    if reserved != 0:
        raise ValueError("PICO packet reserved bytes must be zero")
    if flags & ~TRACKED_FLAG:
        raise ValueError("PICO packet contains undefined flags")

    position = tuple(values[:3])
    quaternion = tuple(values[3:7])
    grip = values[7]
    if not all(math.isfinite(value) for value in (*position, *quaternion, grip)):
        raise ValueError("PICO packet contains non-finite controller values")
    if not 0.0 <= grip <= 1.0:
        raise ValueError("PICO packet grip must be within [0, 1]")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm == 0.0:
        raise ValueError("PICO packet quaternion must have nonzero norm")
    normalized_quaternion = tuple(value / norm for value in quaternion)
    return PicoControllerFrame(
        sequence=sequence,
        tracked=bool(flags & TRACKED_FLAG),
        source_time_us=source_time_us,
        position=position,
        quaternion_xyzw=normalized_quaternion,
        grip=grip,
        received_at=received_at,
        source=source,
    )
