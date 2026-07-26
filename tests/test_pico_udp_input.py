from collections import deque
import socket

import numpy as np
import pytest

from kinova_teleop.pico_protocol import PicoControllerFrame, encode_pico_packet
from kinova_teleop.pico_udp_input import PicoUdpInput
from kinova_teleop.pose_mapping import (
    ClutchState,
    MappingConfig,
    Pose,
    RelativePoseMapper,
)


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


class FakeReceiver:
    def __init__(self, items=()):
        self.items = deque(items)
        self.closed = False

    def receive_once(self, *, timeout=0.0):
        assert timeout == 0.0
        if not self.items:
            raise BlockingIOError
        item = self.items.popleft()
        if isinstance(item, BaseException):
            raise item
        return item

    def close(self):
        self.closed = True


class RepeatedOsErrorReceiver:
    def __init__(self) -> None:
        self.calls = 0

    def receive_once(self, *, timeout=0.0):
        assert timeout == 0.0
        self.calls += 1
        if self.calls > 100:
            raise BlockingIOError
        raise OSError("receiver failed")

    def close(self):
        pass


def identity_pose() -> Pose:
    return Pose(
        position=np.zeros(3, dtype=np.float64),
        quaternion=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
    )


def test_input_drains_to_newest_frame_and_maps_timestamp_to_nanoseconds() -> None:
    receiver = FakeReceiver([
        make_frame(sequence=1, source_time_us=10, received_at=5.0),
        make_frame(sequence=2, source_time_us=20, received_at=5.01),
    ])
    source = PicoUdpInput(receiver=receiver, monotonic=lambda: 5.01)
    sample = source.read()
    assert sample.valid
    assert sample.timestamp_ns == 20_000
    assert sample.grip == 0.75


def test_owned_receiver_uses_input_injected_clock_for_watchdog() -> None:
    source = PicoUdpInput(host="127.0.0.1", port=0, monotonic=lambda: 123.0)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(encode_pico_packet(make_frame(received_at=0.0)), source._receiver.address)
        sample = source.read()
        assert sample.valid
        assert sample.received_monotonic == 123.0
    finally:
        source.close()


def test_input_allows_packet_gap_and_counts_loss() -> None:
    receiver = FakeReceiver([
        make_frame(sequence=5, received_at=5.0),
        make_frame(sequence=8, received_at=5.01),
    ])
    source = PicoUdpInput(receiver=receiver, monotonic=lambda: 5.01)
    assert source.read().valid
    assert source.health().dropped == 2


def test_duplicate_or_out_of_order_frame_invalidates_current_input() -> None:
    receiver = FakeReceiver([
        make_frame(sequence=5, received_at=5.0),
        make_frame(sequence=5, received_at=5.01),
    ])
    source = PicoUdpInput(receiver=receiver, monotonic=lambda: 5.01)
    assert not source.read().valid
    assert source.health().rejected == 1


def test_other_source_is_ignored_until_active_source_is_stale() -> None:
    first = make_frame(sequence=1, position=(1.0, 0.0, 0.0), received_at=5.0, source=("10.0.0.2", 3000))
    other = make_frame(sequence=1, position=(2.0, 0.0, 0.0), received_at=5.05, source=("10.0.0.3", 4000))
    receiver = FakeReceiver([first, other])
    now = [5.05]
    source = PicoUdpInput(receiver=receiver, monotonic=lambda: now[0])
    assert source.read().position.tolist() == list(first.position)
    assert source.health().foreign == 1
    now[0] = 5.21
    receiver.items.append(make_frame(sequence=2, received_at=5.21, source=("10.0.0.3", 4000)))
    assert not source.read().valid
    assert source.health().active_source == ("10.0.0.3", 4000)
    assert source.read().valid


def test_stale_handoff_uses_current_read_time_not_datagram_time() -> None:
    receiver = FakeReceiver([make_frame(sequence=1, received_at=5.0, source=("10.0.0.2", 3000))])
    now = [5.0]
    source = PicoUdpInput(receiver=receiver, monotonic=lambda: now[0])
    assert source.read().valid
    now[0] = 5.21
    receiver.items.append(make_frame(sequence=1, received_at=5.05, source=("10.0.0.3", 4000)))
    source.read()
    assert source.health().active_source == ("10.0.0.3", 4000)


def test_untracked_or_stale_stream_returns_invalid_sample() -> None:
    now = [5.0]
    receiver = FakeReceiver([make_frame(sequence=1, received_at=5.0)])
    source = PicoUdpInput(receiver=receiver, monotonic=lambda: now[0])
    assert source.read().valid
    now[0] = 5.201
    assert not source.read().valid
    receiver.items.append(make_frame(sequence=2, tracked=False, received_at=5.202))
    now[0] = 5.202
    assert not source.read().valid


def test_healthy_same_ip_new_port_emits_boundary_and_requires_release() -> None:
    """Removing full-endpoint locking would bypass the mapper release gate."""
    now = [5.0]
    receiver = FakeReceiver([
        make_frame(
            sequence=10,
            source_time_us=100,
            grip=0.0,
            received_at=5.0,
            source=("10.0.0.2", 3000),
        ),
    ])
    source = PicoUdpInput(receiver=receiver, monotonic=lambda: now[0])
    mapper = RelativePoseMapper(
        MappingConfig(
            filter_time_constant=0.0,
            max_position_step=1.0,
            max_rotation_step=np.pi,
        )
    )
    ee = identity_pose()

    released = source.read()
    assert mapper.update(released, ee, now=5.0).clutch_state is ClutchState.READY
    now[0] = 5.01
    receiver.items.append(
        make_frame(
            sequence=11,
            source_time_us=110,
            grip=1.0,
            received_at=5.01,
            source=("10.0.0.2", 3000),
        )
    )
    pressed = source.read()
    assert mapper.update(pressed, ee, now=5.01).clutch_state is ClutchState.ACTIVE

    now[0] = 5.02
    receiver.items.append(
        make_frame(
            sequence=12,
            source_time_us=120,
            grip=1.0,
            received_at=5.02,
            source=("10.0.0.2", 4000),
        )
    )
    boundary = source.read()
    assert not boundary.valid
    assert source.health().active_source == ("10.0.0.2", 4000)
    assert (
        mapper.update(boundary, ee, now=5.02).clutch_state
        is ClutchState.WAITING_FOR_RELEASE
    )

    held = source.read()
    held_mapping = mapper.update(held, ee, now=5.02)
    assert held.valid
    assert held.grip == 1.0
    assert held_mapping.clutch_state is ClutchState.WAITING_FOR_RELEASE
    assert not held_mapping.active

    now[0] = 5.03
    receiver.items.append(
        make_frame(
            sequence=13,
            source_time_us=130,
            grip=0.0,
            received_at=5.03,
            source=("10.0.0.2", 4000),
        )
    )
    rearmed = mapper.update(source.read(), ee, now=5.03)
    assert rearmed.clutch_state is ClutchState.READY


def test_stale_handoff_queued_before_read_emits_boundary_before_valid_frame() -> None:
    """Draining a recovery frame must not hide the already-stale session."""
    receiver = FakeReceiver([
        make_frame(
            sequence=100,
            source_time_us=1000,
            received_at=5.0,
            source=("10.0.0.2", 3000),
        ),
    ])
    now = [5.0]
    source = PicoUdpInput(receiver=receiver, monotonic=lambda: now[0])
    assert source.read().valid

    now[0] = 5.21
    receiver.items.append(
        make_frame(
            sequence=101,
            source_time_us=1010,
            received_at=5.21,
            source=("10.0.0.2", 4000),
        )
    )

    assert not source.read().valid
    assert source.health().active_source == ("10.0.0.2", 4000)
    assert source.read().valid
    assert source.health().rejected == 0


def test_same_endpoint_restart_requires_stale_sequence_and_source_time_reset() -> None:
    """A restarted process may reset both clocks only after a stale boundary."""
    endpoint = ("10.0.0.2", 3000)
    receiver = FakeReceiver([
        make_frame(
            sequence=500,
            source_time_us=50_000,
            received_at=5.0,
            source=endpoint,
        ),
    ])
    now = [5.0]
    source = PicoUdpInput(receiver=receiver, monotonic=lambda: now[0])
    assert source.read().valid

    now[0] = 5.21
    receiver.items.append(
        make_frame(
            sequence=0,
            source_time_us=10,
            received_at=5.21,
            source=endpoint,
        )
    )

    boundary = source.read()
    assert not boundary.valid
    assert source.health().accepted == 2
    assert source.health().rejected == 0
    restarted = source.read()
    assert restarted.valid
    assert restarted.timestamp_ns == 10_000


@pytest.mark.parametrize("candidate", [10, 9])
def test_healthy_same_endpoint_duplicate_or_out_of_order_stays_rejected(
    candidate: int,
) -> None:
    endpoint = ("10.0.0.2", 3000)
    receiver = FakeReceiver([
        make_frame(
            sequence=10,
            source_time_us=100,
            received_at=5.0,
            source=endpoint,
        ),
    ])
    now = [5.0]
    source = PicoUdpInput(receiver=receiver, monotonic=lambda: now[0])
    assert source.read().valid

    now[0] = 5.01
    receiver.items.append(
        make_frame(
            sequence=candidate,
            source_time_us=90,
            received_at=5.01,
            source=endpoint,
        )
    )

    assert not source.read().valid
    assert source.health().accepted == 1
    assert source.health().rejected == 1
    assert source.health().active_source == endpoint


def test_persistent_receiver_os_error_stops_drain_and_returns_inactive() -> None:
    """Continuing after an OS failure would spin forever in one control step."""
    receiver = RepeatedOsErrorReceiver()
    source = PicoUdpInput(receiver=receiver, monotonic=lambda: 5.0)

    sample = source.read()

    assert not sample.valid
    assert receiver.calls == 1
    assert source.health().rejected == 1
    assert source.health().last_error == "receiver failed"


def test_decode_error_invalidates_input_and_is_reported() -> None:
    receiver = FakeReceiver([ValueError("bad packet")])
    source = PicoUdpInput(receiver=receiver, monotonic=lambda: 5.0)
    assert not source.read().valid
    assert source.health().rejected == 1
    assert source.health().last_error == "bad packet"


def test_sequence_wraparound_is_newer() -> None:
    receiver = FakeReceiver([
        make_frame(sequence=0xFFFFFFFF, received_at=5.0),
        make_frame(sequence=0, received_at=5.01),
    ])
    source = PicoUdpInput(receiver=receiver, monotonic=lambda: 5.01)
    assert source.read().valid
    assert source.health().dropped == 0


def test_v2_trigger_propagates_into_controller_sample() -> None:
    receiver = FakeReceiver([
        make_frame(sequence=1, trigger=0.6, received_at=5.0),
    ])
    source = PicoUdpInput(receiver=receiver, monotonic=lambda: 5.0)
    sample = source.read()
    assert sample.valid
    assert sample.trigger == pytest.approx(0.6)


def test_legacy_frame_without_trigger_yields_zero_trigger() -> None:
    receiver = FakeReceiver([
        make_frame(sequence=1, trigger=0.9, received_at=5.0),
        make_frame(sequence=2, received_at=5.01),
    ])
    source = PicoUdpInput(receiver=receiver, monotonic=lambda: 5.01)
    sample = source.read()
    assert sample.valid
    assert sample.trigger == 0.0


def test_inactive_sample_has_zero_trigger() -> None:
    source = PicoUdpInput(receiver=FakeReceiver(), monotonic=lambda: 5.0)
    sample = source.read()
    assert not sample.valid
    assert sample.trigger == 0.0


def test_untracked_burst_is_not_counted_as_network_loss() -> None:
    receiver = FakeReceiver([
        make_frame(sequence=1, received_at=5.0),
        make_frame(sequence=2, tracked=False, received_at=5.01),
        make_frame(sequence=3, tracked=False, received_at=5.02),
        make_frame(sequence=4, tracked=False, received_at=5.03),
        make_frame(sequence=5, received_at=5.04),
    ])
    source = PicoUdpInput(receiver=receiver, monotonic=lambda: 5.04)
    # The untracked frames in the drain invalidate this read (safety
    # boundary); the subject here is only the health counters.
    source.read()
    health = source.health()
    assert health.dropped == 0
    assert health.rejected == 3


def test_real_gap_before_untracked_frame_still_counts_as_loss() -> None:
    receiver = FakeReceiver([
        make_frame(sequence=1, received_at=5.0),
        make_frame(sequence=4, tracked=False, received_at=5.01),
        make_frame(sequence=5, received_at=5.02),
    ])
    source = PicoUdpInput(receiver=receiver, monotonic=lambda: 5.02)
    source.read()
    assert source.health().dropped == 2
