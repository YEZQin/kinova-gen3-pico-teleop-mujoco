"""Motion tracker boundary tests; never load the real native SDK."""
import importlib
from dataclasses import FrozenInstanceError

import numpy as np
import pytest


class FakeSdk:
    def __init__(self):
        self.timestamp = 100
        self.serials = ["other", "chosen"]
        self.poses = [[9, 8, 7, 0, 0, 0, 1], [1, 2, 3, 0, 0, 0, 1]]
        self.count = 2
        self.closed = 0
        self.initialized = 0

    def init(self):
        self.initialized += 1

    def close(self):
        self.closed += 1

    def num_motion_data_available(self):
        return self.count

    def get_motion_tracker_serial_numbers(self):
        return self.serials

    def get_motion_tracker_pose(self):
        return self.poses

    def get_motion_timestamp_ns(self):
        return self.timestamp


@pytest.fixture
def adapter():
    # The assertion is an explicit RED when the new module is absent.
    assert importlib.util.find_spec("kinova_teleop.motion_tracker") is not None
    return importlib.import_module("kinova_teleop.motion_tracker").MotionTrackerInput


def ready(adapter):
    sdk = FakeSdk()
    clock = [10.0]
    source = adapter("chosen", sdk=sdk, monotonic=lambda: clock[0])
    assert not source.read().valid
    sdk.timestamp += 1
    assert source.read().valid
    return source, sdk, clock


def test_selects_required_serial_and_returns_immutable_copy(adapter):
    source, sdk, _ = ready(adapter)
    sample = source.read()
    np.testing.assert_array_equal(sample.position, [1, 2, 3])
    sdk.poses[1][0] = 99
    np.testing.assert_array_equal(sample.position, [1, 2, 3])
    with pytest.raises(ValueError):
        sample.position[0] = 10
    with pytest.raises(FrozenInstanceError):
        sample.valid = False


def test_freshness_uses_local_elapsed_time_not_headset_epoch(adapter):
    source, sdk, clock = ready(adapter)
    clock[0] += 0.19
    assert source.read().valid
    clock[0] += 0.02
    assert not source.read().valid
    sdk.timestamp += 1
    assert not source.read().valid  # Recovery frame establishes a new baseline.
    sdk.timestamp += 1
    assert source.read().valid


def test_missing_serial_never_falls_back_and_reappearance_requires_advance(adapter):
    source, sdk, _ = ready(adapter)
    sdk.serials = ["other"]
    sdk.poses = sdk.poses[:1]
    sdk.count = 1
    assert not source.read().valid
    sdk.serials = ["chosen"]
    sdk.timestamp += 1
    assert not source.read().valid
    assert not source.read().valid
    sdk.timestamp += 1
    assert source.read().valid


def test_timestamp_restart_needs_another_advancing_frame(adapter):
    source, sdk, _ = ready(adapter)
    sdk.timestamp = 1
    assert not source.read().valid
    assert not source.read().valid
    sdk.timestamp = 2
    assert source.read().valid


@pytest.mark.parametrize("timestamp", [0, -1, 1.5, float("nan"), True])
def test_invalid_timestamps_fail_closed(adapter, timestamp):
    source, sdk, _ = ready(adapter)
    sdk.timestamp = timestamp
    assert not source.read().valid


@pytest.mark.parametrize("pose", [
    [1, 2, 3, 0, 0, 0, 0], [1, 2, 3, 0, 0, 0, float("nan")],
    [float("inf"), 2, 3, 0, 0, 0, 1], [1, 2, 3],
])
def test_invalid_poses_fail_closed(adapter, pose):
    source, sdk, _ = ready(adapter)
    sdk.poses[1] = pose
    assert not source.read().valid


@pytest.mark.parametrize("count,serials", [(1, ["other", "chosen"]), (2, ["chosen", "chosen"]), (0, [])])
def test_inconsistent_or_ambiguous_identity_fails_closed(adapter, count, serials):
    source, sdk, _ = ready(adapter)
    sdk.count, sdk.serials = count, serials
    assert not source.read().valid


def test_mixed_timestamp_snapshot_fails_closed(adapter):
    source, sdk, _ = ready(adapter)
    def changing_timestamp():
        sdk.timestamp += 1
        return sdk.timestamp
    sdk.get_motion_timestamp_ns = changing_timestamp
    assert not source.read().valid


def test_sdk_errors_fail_closed(adapter):
    source, sdk, _ = ready(adapter)
    def broken():
        raise RuntimeError("disconnected")
    sdk.get_motion_tracker_pose = broken
    assert not source.read().valid


def test_repeated_timestamp_cannot_change_position(adapter):
    source, sdk, _ = ready(adapter)
    sdk.poses[1][0] = 999
    np.testing.assert_array_equal(source.read().position, [1, 2, 3])


def test_long_poll_gap_disarms_even_if_timestamp_advanced(adapter):
    source, sdk, clock = ready(adapter)
    sdk.timestamp += 1
    clock[0] += 1
    assert not source.read().valid


@pytest.mark.parametrize("now", [float("nan"), 1.0])
def test_invalid_local_clock_fails_closed(adapter, now):
    source, _, clock = ready(adapter)
    clock[0] = now
    assert not source.read().valid


def test_close_before_read_does_not_initialize_sdk(adapter):
    sdk = FakeSdk()
    source = adapter("chosen", sdk=sdk)
    source.close()
    assert not source.read().valid
    assert sdk.initialized == 0


def test_lazy_native_initialization_and_idempotent_close(adapter, monkeypatch):
    monkeypatch.setattr('kinova_teleop.motion_tracker.sys.platform', 'linux')
    sdk = FakeSdk()
    monkeypatch.setattr("kinova_teleop.motion_tracker.importlib.import_module", lambda name: sdk)
    source = adapter("chosen")
    assert sdk.initialized == 0
    source.read()
    assert sdk.initialized == 1
    source.close()
    source.close()
    assert sdk.closed == 1
    assert not source.read().valid


@pytest.mark.parametrize("serial,timeout", [("", .2), (" ", .2), ("chosen", 0), ("chosen", float("nan"))])
def test_invalid_configuration_rejected(adapter, serial, timeout):
    with pytest.raises(ValueError):
        adapter(serial, stale_timeout=timeout)
