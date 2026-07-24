import importlib

import numpy as np
import pytest

from kinova_teleop.xr_input import DryRunXrInput, SdkXrInput


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
