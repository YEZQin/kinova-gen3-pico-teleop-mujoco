import json
import pytest

from kinova_teleop.motion_packets import MotionPacketBuffer


def packet(timestamp=None):
    motion = {'joints': [{'p': '1,2,3,0,0,0,1', 'sn': 'right'}], 'len': 1}
    if timestamp is not None:
        motion['timeStampNs'] = timestamp
    return json.dumps({'value': json.dumps({'Motion': motion})})


def test_missing_timestamp_uses_packet_arrival_only():
    buffer = MotionPacketBuffer()
    buffer.ingest(packet(), 1_000_000_000)
    first = buffer.snapshot()
    assert first['timestamp_ns'] == 1_000_000_000
    assert first['time_source'] == 'host_packet_arrival'
    assert buffer.snapshot() == first
    buffer.ingest(json.dumps({'value': '{}'}), 1_100_000_000)
    assert buffer.snapshot() == first  # headset-only traffic cannot refresh tracker
    buffer.ingest(packet(), 1_200_000_000)
    assert buffer.snapshot()['timestamp_ns'] == 1_200_000_000


def test_device_timestamp_is_preserved_when_available():
    buffer = MotionPacketBuffer()
    buffer.ingest(packet(42), 1000)
    assert buffer.snapshot()['timestamp_ns'] == 42
    buffer.ingest(packet(42), 2000)
    assert buffer.snapshot()['timestamp_ns'] == 42


@pytest.mark.parametrize('value', ['bad json', json.dumps({'value': '{bad'}),
                                  json.dumps({'value': []}),
                                  json.dumps({'value': json.dumps({'Motion': {'joints': []}})})])
def test_bad_packet_clears_prior_data(value):
    buffer = MotionPacketBuffer()
    buffer.ingest(packet(), 1000)
    buffer.ingest(value, 2000)
    with pytest.raises(ValueError):
        buffer.snapshot()


def test_disconnect_invalidates_cached_packet():
    buffer = MotionPacketBuffer()
    buffer.ingest(packet(), 1000)
    buffer.invalidate()
    with pytest.raises(ValueError):
        buffer.snapshot()


def test_adapter_packet_clock_cannot_refresh_from_reading_cache():
    from kinova_teleop.motion_tracker import MotionTrackerInput
    buffer = MotionPacketBuffer()
    class Sdk:
        def init(self): pass
        def close(self): pass
        def get_motion_snapshot(self): return buffer.snapshot()
    now = [1.]
    source = MotionTrackerInput('right', sdk=Sdk(), monotonic=lambda: now[0])
    buffer.ingest(packet(), 1_000_000_000)
    assert not source.read().valid
    now[0] = 1.01
    buffer.ingest(packet(), 1_010_000_000)
    assert source.read().valid
    assert source.read().time_source == 'host_packet_arrival'
    now[0] = 1.22
    assert not source.read().valid
    # Reading this cached packet again must not establish a fresh baseline.
    assert not source.read().valid


def test_unchanged_valid_stationary_packets_remain_fresh():
    from kinova_teleop.motion_tracker import MotionTrackerInput
    buffer = MotionPacketBuffer()
    class Sdk:
        def init(self): pass
        def close(self): pass
        def get_motion_snapshot(self): return buffer.snapshot()
    now = [1.]
    source = MotionTrackerInput('right', sdk=Sdk(), monotonic=lambda: now[0])
    for index in range(30):
        ns = 1_000_000_000 + index * 10_000_000
        now[0] = ns / 1e9
        buffer.ingest(packet(), ns)
        assert source.read().valid == (index > 0)


def test_zero_len_with_old_cached_joints_is_not_a_new_tracker_frame():
    buffer = MotionPacketBuffer()
    value = json.loads(json.loads(packet())['value'])
    value['Motion']['len'] = 0
    buffer.ingest(json.dumps({'value': json.dumps(value)}), 1000)
    with pytest.raises(ValueError): buffer.snapshot()


def test_native_callback_binds_only_valid_motion_from_required_tracker():
    import ctypes as c
    from kinova_teleop.motion_packets import PacketMotionSdk, _DeviceState
    sdk = PacketMotionSdk(serial='right')
    def send(device, raw):
        state = _DeviceState(device, raw.encode())
        sdk._receive(None, 1 << 25, 0, c.addressof(state))
    send(b'wrong', '{bad')
    assert sdk._device is None
    send(b'wrong', packet().replace('right', 'other'))
    assert sdk._device is None
    send(b'right-headset', packet())
    assert sdk._device == b'right-headset'
    assert sdk.get_motion_snapshot()['serials'] == ('right',)
