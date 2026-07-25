"""Simulator-independent teleoperation orchestration."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
import time

from .backend import BackendResult, EndEffectorTargetBackend
from .pose_mapping import ClutchState, MappingConfig, RelativePoseMapper
from .xr_input import XrInputSource


@dataclass(frozen=True)
class TeleopConfig:
    control_hz: float = 100.0
    realtime: bool = True
    translation_scale: float = 0.5
    stale_timeout: float = 0.2


@dataclass(frozen=True)
class StepDiagnostics:
    active: bool
    stale: bool
    ik_converged: bool
    position_error: float
    rotation_error: float
    clutch_state: ClutchState
    reason: str


class TeleopController:
    def __init__(
        self,
        config: TeleopConfig,
        source: XrInputSource,
        backend: EndEffectorTargetBackend,
    ) -> None:
        if not math.isfinite(config.control_hz) or config.control_hz <= 0.0:
            raise ValueError("control_hz must be positive and finite")
        if not math.isfinite(config.translation_scale) or config.translation_scale <= 0:
            raise ValueError("translation_scale must be positive and finite")

        self.config = config
        self.source = source
        self.backend = backend
        self.mapper = RelativePoseMapper(
            MappingConfig(
                translation_scale=config.translation_scale,
                stale_timeout=config.stale_timeout,
            ),
        )
        self.mapper.reset(backend.current_pose())
        self.steps = 0
        self._closed = False

    def step_once(self) -> StepDiagnostics:
        sample = self.source.read()
        now = (
            time.monotonic()
            if self.config.realtime
            else float(sample.received_monotonic)
        )
        mapping = self.mapper.update(sample, self.backend.current_pose(), now)
        if mapping.activated:
            self.backend.begin_control()

        if mapping.active:
            result = self.backend.command_pose(mapping.target)
        else:
            self.backend.hold()
            result = BackendResult(False, False, 0.0, 0.0, "")

        self.backend.step()
        self.steps += 1
        return StepDiagnostics(
            active=mapping.active,
            stale=mapping.stale,
            ik_converged=result.converged,
            position_error=result.position_error,
            rotation_error=result.rotation_error,
            clutch_state=mapping.clutch_state,
            reason=result.reason,
        )

    def run(
        self,
        *,
        max_steps: int | None = None,
        should_continue: Callable[[], bool] | None = None,
        on_step: Callable[[], None] | None = None,
        on_status: Callable[[StepDiagnostics], None] | None = None,
    ) -> None:
        if max_steps is not None and max_steps <= 0:
            raise ValueError("max_steps must be positive")

        try:
            next_deadline = time.monotonic()
            last_status: tuple[ClutchState, bool, bool, str] | None = None
            while (
                (max_steps is None or self.steps < max_steps)
                and (should_continue is None or should_continue())
            ):
                diagnostics = self.step_once()
                status = (
                    diagnostics.clutch_state,
                    diagnostics.stale,
                    diagnostics.ik_converged,
                    diagnostics.reason,
                )
                if on_status is not None and status != last_status:
                    on_status(diagnostics)
                last_status = status
                if on_step is not None:
                    on_step()
                if self.config.realtime:
                    next_deadline += 1.0 / self.config.control_hz
                    delay = next_deadline - time.monotonic()
                    if delay > 0.0:
                        time.sleep(delay)
                    elif delay < -1.0:
                        next_deadline = time.monotonic()
        finally:
            self.close()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self.source.close()
        finally:
            self.backend.close()
