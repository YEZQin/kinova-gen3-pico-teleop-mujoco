"""Simulator-independent teleoperation orchestration."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
import time
from collections.abc import Mapping

from .backend import BackendResult, EndEffectorTargetBackend
from .pose_mapping import ClutchState, MappingConfig, Pose, RelativePoseMapper
from .xr_input import XrInputSource


@dataclass(frozen=True)
class TeleopConfig:
    control_hz: float = 100.0
    realtime: bool = True
    translation_scale: float = 0.5
    stale_timeout: float = 0.2
    gripper: bool = False


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
        *,
        event_sink: Callable[[str, str, Mapping[str, object]], None] | None = None,
    ) -> None:
        if not math.isfinite(config.control_hz) or config.control_hz <= 0.0:
            raise ValueError("control_hz must be positive and finite")
        if not math.isfinite(config.translation_scale) or config.translation_scale <= 0:
            raise ValueError("translation_scale must be positive and finite")
        if config.gripper and getattr(backend, "command_gripper", None) is None:
            raise ValueError("The selected backend does not support a gripper")

        self.config = config
        self.source = source
        self.backend = backend
        self.event_sink = event_sink
        self.mapper = RelativePoseMapper(
            MappingConfig(
                translation_scale=config.translation_scale,
                stale_timeout=config.stale_timeout,
            ),
        )
        self.mapper.reset(backend.current_pose())
        self.steps = 0
        self._closed = False
        self._backend_closed = False
        self._source_closed = False

    def _emit(self, kind: str, state: str, payload: Mapping[str, object] | None = None) -> None:
        if self.event_sink is not None:
            try:
                self.event_sink(kind, state, dict(payload or {}))
            except Exception:
                return

    def _begin_anchor_transaction(self) -> Pose:
        """Run the backend's begin-control transaction and return the anchor.

        ``begin_control`` must precede ``current_pose`` so that hardware
        backends can re-arm their feedback gate before the anchor read.
        """

        self.backend.begin_control()
        return self.backend.current_pose()

    def step_once(self) -> StepDiagnostics:
        sample = self.source.read()
        now = (
            time.monotonic()
            if self.config.realtime
            else float(sample.received_monotonic)
        )
        mapping = self.mapper.update(sample, self._begin_anchor_transaction, now)

        if mapping.active:
            result = self.backend.command_pose(mapping.target)
        else:
            if mapping.deactivated:
                self._emit("input_release", "STOPPING", {})
                self.backend.hold()
            result = BackendResult(False, False, 0.0, 0.0, "")

        if mapping.stale:
            self._emit("input_stale", "STOPPING", {})

        if self.config.gripper and mapping.active and sample.valid:
            self.backend.command_gripper(float(sample.trigger))

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
        """Close the backend first so hardware motion stops before the input.

        Both resources are always attempted; the first error is re-raised
        after both close attempts complete.
        """

        if self._closed:
            return

        first_error: BaseException | None = None
        if not self._backend_closed:
            try:
                self.backend.close()
            except BaseException as error:
                first_error = error
            else:
                self._backend_closed = True
        if not self._source_closed:
            try:
                self.source.close()
            except BaseException as error:
                if first_error is None:
                    first_error = error
            else:
                self._source_closed = True

        self._closed = self._backend_closed and self._source_closed
        if first_error is not None:
            raise first_error
