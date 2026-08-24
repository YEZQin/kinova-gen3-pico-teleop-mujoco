"""Validated proportional mapping from PICO Trigger to gripper position."""

from __future__ import annotations

from dataclasses import dataclass
import math


GRIPPER_POSITION_MIN = 0.01
GRIPPER_POSITION_MAX = 0.99


@dataclass(frozen=True)
class GripperTriggerMapping:
    """Map a calibrated Trigger interval onto the usable gripper travel."""

    trigger_min: float = 0.0
    trigger_max: float = 1.0

    def __post_init__(self) -> None:
        if (
            not math.isfinite(self.trigger_min)
            or not math.isfinite(self.trigger_max)
            or self.trigger_min < 0.0
            or self.trigger_max > 1.0
            or self.trigger_min >= self.trigger_max
        ):
            raise ValueError(
                "gripper trigger range must be finite, ordered, and within [0, 1]"
            )

    def map(self, trigger: float) -> float:
        if not math.isfinite(trigger):
            raise ValueError("gripper trigger must be finite")
        normalized = (float(trigger) - self.trigger_min) / (
            self.trigger_max - self.trigger_min
        )
        clamped = min(1.0, max(0.0, normalized))
        return GRIPPER_POSITION_MIN + clamped * (
            GRIPPER_POSITION_MAX - GRIPPER_POSITION_MIN
        )
