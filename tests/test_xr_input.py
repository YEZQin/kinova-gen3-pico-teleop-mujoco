import importlib
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from kinova_teleop.xr_input import (
    ContinuousInputBuffer,
    ControllerSample,
    DryRunXrInput,
    SdkXrInput,
)


xr_input_module = importlib.import_module("kinova_teleop.xr_input")


class FakeSdk:
    def __init__(self) -> None:
        self.initialized = False
        self.closed = False

    def init(self) -> None:
        self.initialized = True

    def get_left_controller_pose(self) -> np.ndarray:
        return np.array([1, 2, 3, 0, 0, 0, 1], dtype=float)

    def get_left_grip(self) -> float:
        return 0.95

    def get_time_stamp_ns(self) -> int:
        return 123

    def close(self) -> None:
        self.closed = True


def test_sdk_adapter_reads_left_controller() -> None:
    sdk = FakeSdk()
    source = SdkXrInput(sdk=sdk, monotonic=lambda: 5.0)

    sample = source.read()

    assert sdk.initialized
    np.testing.assert_allclose(sample.position, [1, 2, 3])
    np.testing.assert_allclose(sample.quaternion_xyzw, [0, 0, 0, 1])
    assert sample.grip == pytest.approx(0.95)
    assert sample.timestamp_ns == 123
    assert sample.received_monotonic == 5.0
    assert sample.valid
    source.close()
    source.close()
    assert sdk.closed


def test_sdk_import_error_is_actionable(monkeypatch: pytest.MonkeyPatch) -> None:
    def fail_import(_: str):
        raise ModuleNotFoundError

    monkeypatch.setattr(importlib, "import_module", fail_import)
    with pytest.raises(RuntimeError, match="xrobotoolkit_sdk"):
        SdkXrInput()


def test_sdk_adapter_marks_malformed_pose_invalid() -> None:
    sdk = FakeSdk()
    sdk.get_left_controller_pose = lambda: np.array([1.0, 2.0])
    source = SdkXrInput(sdk=sdk)

    sample = source.read()

    assert not sample.valid
    np.testing.assert_allclose(sample.position, np.zeros(3))
    np.testing.assert_allclose(sample.quaternion_xyzw, [0, 0, 0, 1])
    source.close()


def test_dry_run_is_deterministic_and_exercises_grip() -> None:
    first = DryRunXrInput(control_hz=100.0)
    second = DryRunXrInput(control_hz=100.0)
    samples = [first.read() for _ in range(150)]
    matching_samples = [second.read() for _ in range(150)]

    assert samples[0].timestamp_ns < samples[-1].timestamp_ns
    assert any(sample.grip > 0.9 for sample in samples)
    assert any(sample.grip < 0.9 for sample in samples)
    assert all(sample.position.shape == (3,) for sample in samples)
    for left, right in zip(samples, matching_samples):
        np.testing.assert_allclose(left.position, right.position)
        np.testing.assert_allclose(left.quaternion_xyzw, right.quaternion_xyzw)
        assert left.grip == right.grip
        assert left.timestamp_ns == right.timestamp_ns


def test_continuous_buffer_ingests_while_consumer_is_idle() -> None:
    five_samples = threading.Event()

    class Source:
        def __init__(self) -> None:
            self.reads = 0
            self.closed = 0

        def read(self) -> ControllerSample:
            self.reads += 1
            if self.reads >= 5:
                five_samples.set()
            return ControllerSample(
                position=np.array([float(self.reads), 0.0, 0.0]),
                quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                grip=0.0,
                timestamp_ns=self.reads,
                received_monotonic=float(self.reads),
            )

        def health(self):
            return SimpleNamespace(active_source=("192.168.1.2", 5000), foreign=0)

        def close(self) -> None:
            self.closed += 1

    source = Source()
    buffered = ContinuousInputBuffer(source, poll_interval_s=0.001)
    try:
        assert five_samples.wait(0.2)
        sample = buffered.read()
        assert sample.valid
        assert sample.timestamp_ns >= 5
        assert buffered.health().active_source == ("192.168.1.2", 5000)
    finally:
        buffered.close()
        buffered.close()
    assert source.closed == 1


def test_continuous_buffer_surfaces_receiver_failure() -> None:
    failed = threading.Event()

    class Source:
        def read(self) -> ControllerSample:
            failed.set()
            raise OSError("receiver failed")

        def close(self) -> None:
            pass

    buffered = ContinuousInputBuffer(Source(), poll_interval_s=0.001)
    try:
        assert failed.wait(0.2)
        with pytest.raises(RuntimeError, match="continuous controller input failed"):
            buffered.read()
    finally:
        buffered.close()


def test_continuous_buffer_does_not_mask_stale_source_sample() -> None:
    stale_emitted = threading.Event()

    class Source:
        def __init__(self) -> None:
            self.reads = 0

        def read(self) -> ControllerSample:
            self.reads += 1
            if self.reads == 1:
                return ControllerSample(
                    position=np.zeros(3, dtype=float),
                    quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                    grip=0.0,
                    timestamp_ns=1,
                    received_monotonic=1.0,
                )
            stale_emitted.set()
            return ControllerSample(
                position=np.zeros(3, dtype=float),
                quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                grip=0.0,
                timestamp_ns=0,
                received_monotonic=2.0,
                valid=False,
                invalid_reason="stream is stale",
            )

        def close(self) -> None:
            pass

    buffered = ContinuousInputBuffer(Source(), poll_interval_s=0.001)
    try:
        assert stale_emitted.wait(0.2)
        sample = buffered.read()
        assert not sample.valid
        assert sample.invalid_reason == "stream is stale"
    finally:
        buffered.close()


def test_continuous_buffer_latches_transient_invalid_sample() -> None:
    recovered = threading.Event()

    class Source:
        def __init__(self) -> None:
            self.reads = 0

        def read(self) -> ControllerSample:
            self.reads += 1
            if self.reads == 2:
                return ControllerSample(
                    position=np.zeros(3, dtype=float),
                    quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                    grip=0.0,
                    timestamp_ns=0,
                    received_monotonic=2.0,
                    valid=False,
                    invalid_reason="source changed",
                )
            if self.reads >= 3:
                recovered.set()
            return ControllerSample(
                position=np.zeros(3, dtype=float),
                quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                grip=0.0,
                timestamp_ns=self.reads,
                received_monotonic=float(self.reads),
            )

        def close(self) -> None:
            pass

    buffered = ContinuousInputBuffer(Source(), poll_interval_s=0.001)
    try:
        assert recovered.wait(0.2)
        sample = buffered.read()
        assert not sample.valid
        assert sample.invalid_reason == "source changed"
    finally:
        buffered.close()


def test_continuous_buffer_preserves_ordered_invalid_samples() -> None:
    recovered = threading.Event()

    class Source:
        def __init__(self) -> None:
            self.reads = 0

        def read(self) -> ControllerSample:
            self.reads += 1
            if self.reads == 1:
                return ControllerSample(
                    position=np.zeros(3, dtype=float),
                    quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                    grip=0.0,
                    timestamp_ns=0,
                    received_monotonic=1.0,
                    valid=False,
                    invalid_reason="stream is stale",
                )
            if self.reads == 2:
                return ControllerSample(
                    position=np.zeros(3, dtype=float),
                    quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                    grip=0.0,
                    timestamp_ns=0,
                    received_monotonic=2.0,
                    valid=False,
                    invalid_reason="controller is untracked",
                )
            recovered.set()
            return ControllerSample(
                position=np.zeros(3, dtype=float),
                quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                grip=0.0,
                timestamp_ns=self.reads,
                received_monotonic=float(self.reads),
            )

        def close(self) -> None:
            pass

    buffered = ContinuousInputBuffer(Source(), poll_interval_s=0.001)
    try:
        assert recovered.wait(0.2)
        first = buffered.read()
        second = buffered.read()
        assert (first.valid, first.invalid_reason) == (False, "stream is stale")
        assert (second.valid, second.invalid_reason) == (
            False,
            "controller is untracked",
        )
    finally:
        buffered.close()


def test_continuous_buffer_closes_source_when_thread_start_fails(
    monkeypatch,
) -> None:
    class Source:
        def __init__(self) -> None:
            self.closed = 0

        def read(self) -> ControllerSample:
            raise AssertionError("receiver thread must not run")

        def close(self) -> None:
            self.closed += 1

    source = Source()
    monkeypatch.setattr(
        threading.Thread,
        "start",
        lambda _thread: (_ for _ in ()).throw(RuntimeError("start failed")),
    )

    with pytest.raises(RuntimeError, match="start failed"):
        ContinuousInputBuffer(source)

    assert source.closed == 1


def test_continuous_buffer_close_interrupts_blocked_source() -> None:
    reading = threading.Event()
    release = threading.Event()

    class Source:
        def __init__(self) -> None:
            self.closed = 0

        def read(self) -> ControllerSample:
            reading.set()
            release.wait()
            raise OSError("source closed")

        def close(self) -> None:
            self.closed += 1
            release.set()

    source = Source()
    buffered = ContinuousInputBuffer(source, poll_interval_s=0.001)
    assert reading.wait(0.2)

    buffered.close()

    assert source.closed == 1


def test_continuous_buffer_fails_closed_on_invalid_queue_overflow(
    monkeypatch,
) -> None:
    overflowed = threading.Event()
    monkeypatch.setattr(xr_input_module, "_MAX_PENDING_INVALIDS", 2, raising=False)

    class Source:
        def __init__(self) -> None:
            self.reads = 0

        def read(self) -> ControllerSample:
            self.reads += 1
            if self.reads >= 3:
                overflowed.set()
            return ControllerSample(
                position=np.zeros(3, dtype=float),
                quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                grip=0.0,
                timestamp_ns=0,
                received_monotonic=float(self.reads),
                valid=False,
                invalid_reason=f"invalid-{self.reads}",
            )

        def close(self) -> None:
            pass

    buffered = ContinuousInputBuffer(Source(), poll_interval_s=0.001)
    try:
        assert overflowed.wait(0.2)
        with pytest.raises(
            RuntimeError,
            match="continuous controller input failed",
        ) as caught:
            buffered.read()
        assert caught.value.__cause__ is not None
        assert "invalid queue overflow" in str(caught.value.__cause__)
    finally:
        buffered.close()
