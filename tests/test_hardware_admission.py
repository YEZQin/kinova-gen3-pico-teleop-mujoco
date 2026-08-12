from __future__ import annotations

from collections import deque

import numpy as np
import pytest

from kinova_teleop.hardware_admission import (
    InputAdmissionError,
    confirm_move,
    verify_released_now,
    wait_for_fresh_released_input,
)
from kinova_teleop.xr_input import ControllerSample


def sample(
    *,
    timestamp_ns: int,
    grip: float,
    valid: bool = True,
    invalid_reason: str = "",
) -> ControllerSample:
    return ControllerSample(
        position=np.zeros(3, dtype=np.float64),
        quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
        grip=grip,
        timestamp_ns=timestamp_ns,
        received_monotonic=0.0,
        valid=valid,
        invalid_reason=invalid_reason,
    )


class ScriptedSource:
    def __init__(self, samples: list[ControllerSample]) -> None:
        self._samples = deque(samples)
        self.closed = False

    def read(self) -> ControllerSample:
        if not self._samples:
            raise RuntimeError("sample stream exhausted")
        return self._samples.popleft()

    def close(self) -> None:
        self.closed = True


class HealthScriptedSource(ScriptedSource):
    def __init__(self, *, source_changes_during_window: bool = False) -> None:
        super().__init__([
            sample(timestamp_ns=1, grip=0.0),
            sample(timestamp_ns=2, grip=0.0),
            sample(timestamp_ns=3, grip=0.0),
        ])
        self._source_changes_during_window = source_changes_during_window
        self._reads = 0

    def read(self) -> ControllerSample:
        self._reads += 1
        return super().read()

    def health(self):
        source = ("pico", 1 if self._source_changes_during_window and self._reads >= 2 else 0)
        return type("Health", (), {"active_source": source, "foreign": 0})()


class MutableHealthSource(ScriptedSource):
    def __init__(self, samples: list[ControllerSample]) -> None:
        super().__init__(samples)
        self.active_source = ("pico", 0)
        self.foreign = 0

    def health(self):
        return type(
            "Health",
            (),
            {"active_source": self.active_source, "foreign": self.foreign},
        )()


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, duration: float) -> None:
        self.now += duration


def test_admission_requires_advancing_finite_samples_and_current_release() -> None:
    source = ScriptedSource([
        sample(timestamp_ns=1, grip=0.0),
        sample(timestamp_ns=2, grip=0.0),
        sample(timestamp_ns=3, grip=0.0),
    ])

    result = wait_for_fresh_released_input(
        source,
        sample_count=3,
        timeout_s=1.0,
        monotonic=FakeClock().monotonic,
        sleep=lambda _: None,
    )

    assert result.last_timestamp_ns == 3
    assert result.grip_released is True
    assert source.closed is False


def test_admission_waits_for_initial_stale_placeholder_before_valid_window() -> None:
    clock = FakeClock()
    source = ScriptedSource([
        sample(
            timestamp_ns=0,
            grip=0.0,
            valid=False,
            invalid_reason="stream is stale",
        ),
        sample(timestamp_ns=1, grip=0.0),
        sample(timestamp_ns=2, grip=0.0),
        sample(timestamp_ns=3, grip=0.0),
    ])

    result = wait_for_fresh_released_input(
        source,
        sample_count=3,
        timeout_s=1.0,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    assert result.accepted_samples == 3
    assert result.last_timestamp_ns == 3


def test_admission_waits_past_cached_timestamp_for_distinct_samples() -> None:
    clock = FakeClock()
    source = ScriptedSource([
        sample(timestamp_ns=1, grip=0.0),
        sample(timestamp_ns=1, grip=0.0),
        sample(timestamp_ns=2, grip=0.0),
        sample(timestamp_ns=3, grip=0.0),
    ])

    result = wait_for_fresh_released_input(
        source,
        sample_count=3,
        timeout_s=1.0,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    assert result.accepted_samples == 3
    assert result.last_timestamp_ns == 3


def test_admission_rejects_source_change_without_connecting_robot() -> None:
    source = HealthScriptedSource(source_changes_during_window=True)

    with pytest.raises(InputAdmissionError, match="source changed"):
        wait_for_fresh_released_input(source, sample_count=3, timeout_s=1.0)

    assert source.closed is False


def test_admission_rejects_foreign_packet_count_change() -> None:
    class ForeignSource(HealthScriptedSource):
        def health(self):
            return type(
                "Health",
                (),
                {"active_source": ("pico", 0), "foreign": self._reads // 2},
            )()

    with pytest.raises(InputAdmissionError, match="foreign packet count changed"):
        wait_for_fresh_released_input(ForeignSource(), sample_count=3, timeout_s=1.0)


@pytest.mark.parametrize(
    "invalid_sample",
    [
        sample(timestamp_ns=1, grip=float("nan")),
        sample(timestamp_ns=1, grip=0.0, valid=False),
    ],
)
def test_admission_rejects_non_finite_or_invalid_samples(
    invalid_sample: ControllerSample,
) -> None:
    with pytest.raises(InputAdmissionError, match="finite valid"):
        wait_for_fresh_released_input(
            ScriptedSource([invalid_sample]), sample_count=1, timeout_s=1.0
        )


def test_admission_rejects_regressing_timestamps_and_pressed_final_sample() -> None:
    with pytest.raises(InputAdmissionError, match="timestamps"):
        wait_for_fresh_released_input(
            ScriptedSource([
                sample(timestamp_ns=2, grip=0.0),
                sample(timestamp_ns=1, grip=0.0),
            ]),
            sample_count=2,
            timeout_s=1.0,
        )
    with pytest.raises(InputAdmissionError, match="Grip must remain released"):
        wait_for_fresh_released_input(
            ScriptedSource([sample(timestamp_ns=1, grip=0.8)]),
            sample_count=1,
            timeout_s=1.0,
        )


def test_admission_rejects_pressed_grip_anywhere_in_release_window() -> None:
    with pytest.raises(InputAdmissionError, match="Grip must remain released"):
        wait_for_fresh_released_input(
            ScriptedSource([
                sample(timestamp_ns=1, grip=1.0),
                sample(timestamp_ns=2, grip=0.0),
                sample(timestamp_ns=3, grip=0.0),
            ]),
            sample_count=3,
            timeout_s=1.0,
        )


def test_move_confirmation_is_exact_and_requires_released_recheck() -> None:
    assert confirm_move(input_fn=lambda _: "MOVE") is None
    with pytest.raises(InputAdmissionError, match="exact MOVE"):
        confirm_move(input_fn=lambda _: "move")
    with pytest.raises(InputAdmissionError, match="Grip must remain released"):
        verify_released_now(
            ScriptedSource([sample(timestamp_ns=4, grip=1.0)]),
            after_timestamp_ns=3,
            timeout_s=0.5,
        )


def test_release_recheck_requires_a_newer_finite_sample() -> None:
    clock = FakeClock()
    with pytest.raises(InputAdmissionError, match="timed out"):
        verify_released_now(
            ScriptedSource([
                sample(timestamp_ns=3, grip=0.0),
                sample(timestamp_ns=3, grip=0.0),
            ]),
            after_timestamp_ns=3,
            timeout_s=0.02,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        )
    result = verify_released_now(
        ScriptedSource([sample(timestamp_ns=4, grip=0.0)]),
        after_timestamp_ns=3,
        timeout_s=0.5,
    )
    assert result.timestamp_ns == 4


def test_release_recheck_waits_past_cached_sample_for_newer_frame() -> None:
    clock = FakeClock()
    result = verify_released_now(
        ScriptedSource([
            sample(timestamp_ns=3, grip=0.0),
            sample(timestamp_ns=3, grip=0.0),
            sample(timestamp_ns=4, grip=0.0),
        ]),
        after_timestamp_ns=3,
        timeout_s=0.5,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    assert result.timestamp_ns == 4


def test_release_recheck_rejects_source_change_after_admission() -> None:
    source = MutableHealthSource([
        sample(timestamp_ns=1, grip=0.0),
        sample(timestamp_ns=2, grip=0.0),
        sample(timestamp_ns=3, grip=0.0),
        sample(timestamp_ns=4, grip=0.0),
    ])
    admission = wait_for_fresh_released_input(
        source,
        sample_count=3,
        timeout_s=1.0,
    )
    source.active_source = ("pico", 1)

    with pytest.raises(InputAdmissionError, match="source changed"):
        verify_released_now(
            source,
            after_timestamp_ns=admission.last_timestamp_ns,
            admission=admission,
            timeout_s=0.5,
        )


def test_release_recheck_rejects_foreign_count_change_after_admission() -> None:
    source = MutableHealthSource([
        sample(timestamp_ns=1, grip=0.0),
        sample(timestamp_ns=2, grip=0.0),
        sample(timestamp_ns=3, grip=0.0),
        sample(timestamp_ns=4, grip=0.0),
    ])
    admission = wait_for_fresh_released_input(
        source,
        sample_count=3,
        timeout_s=1.0,
    )
    source.foreign = 1

    with pytest.raises(InputAdmissionError, match="foreign packet count changed"):
        verify_released_now(
            source,
            after_timestamp_ns=admission.last_timestamp_ns,
            admission=admission,
            timeout_s=0.5,
        )


def test_release_recheck_requires_admission_for_health_capable_source() -> None:
    source = MutableHealthSource([sample(timestamp_ns=4, grip=0.0)])

    with pytest.raises(InputAdmissionError, match="prior admission"):
        verify_released_now(
            source,
            after_timestamp_ns=3,
            timeout_s=0.5,
        )
