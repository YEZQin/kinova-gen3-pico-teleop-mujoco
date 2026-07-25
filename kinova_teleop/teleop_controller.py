"""Backend-neutral teleoperation orchestration."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import time
from typing import Any

from .backend import EndEffectorTargetBackend
from .mujoco_backend import MuJoCoBackend
from .pose_mapping import MappingConfig, RelativePoseMapper
from .xr_input import XrInputSource


@dataclass(frozen=True)
class TeleopConfig:
    model_path: Path
    control_hz: float = 100.0
    realtime: bool = True
    translation_scale: float = 1.0
    stale_timeout: float = 0.2


@dataclass(frozen=True)
class StepDiagnostics:
    active: bool
    stale: bool
    ik_converged: bool
    position_error: float
    rotation_error: float


class TeleopController:
    def __init__(
        self,
        config: TeleopConfig,
        source: XrInputSource,
        backend: EndEffectorTargetBackend | None = None,
    ) -> None:
        if not math.isfinite(config.control_hz) or config.control_hz <= 0.0:
            raise ValueError("control_hz must be positive and finite")
        if not math.isfinite(config.translation_scale) or config.translation_scale <= 0:
            raise ValueError("translation_scale must be positive and finite")

        self.config = config
        self.source = source
        self.backend = (
            MuJoCoBackend(config.model_path, config.control_hz)
            if backend is None
            else backend
        )
        self.mapper = RelativePoseMapper(
            MappingConfig(
                translation_scale=config.translation_scale,
                stale_timeout=config.stale_timeout,
            ),
        )
        self.mapper.reset(self.backend.current_pose())
        self._steps = 0
        self._closed = False

    @property
    def model(self) -> Any:
        return self.backend.model

    @property
    def data(self) -> Any:
        return self.backend.data

    @property
    def contract(self) -> Any:
        return self.backend.contract

    @property
    def ik(self) -> Any:
        return self.backend.ik

    @ik.setter
    def ik(self, value: Any) -> None:
        self.backend.ik = value

    @property
    def steps(self) -> int:
        return self._steps

    def step_once(self) -> StepDiagnostics:
        sample = self.source.read()
        now = (
            time.monotonic()
            if self.config.realtime
            else float(sample.received_monotonic)
        )
        mapping = self.mapper.update(sample, self.backend.current_pose(), now)

        result = None
        if mapping.activated:
            self.backend.begin_control()
        if mapping.active:
            result = self.backend.command_pose(mapping.target)
        elif mapping.deactivated:
            self.backend.hold()

        self.backend.step()
        self._steps += 1
        return StepDiagnostics(
            active=mapping.active,
            stale=mapping.stale,
            ik_converged=False if result is None else result.converged,
            position_error=0.0 if result is None else result.position_error,
            rotation_error=0.0 if result is None else result.rotation_error,
        )

    def run(self, headless: bool = False, max_steps: int | None = None) -> None:
        if headless and (max_steps is None or max_steps <= 0):
            raise ValueError("headless runs require a positive max_steps")
        if max_steps is not None and max_steps <= 0:
            raise ValueError("max_steps must be positive")

        viewer = None
        try:
            if not headless:
                import mujoco.viewer

                viewer = mujoco.viewer.launch_passive(self.model, self.data)

            next_deadline = time.monotonic()
            while (
                (max_steps is None or self.steps < max_steps)
                and (viewer is None or viewer.is_running())
            ):
                self.step_once()
                if viewer is not None:
                    viewer.sync()
                if self.config.realtime:
                    next_deadline += 1.0 / self.config.control_hz
                    delay = next_deadline - time.monotonic()
                    if delay > 0.0:
                        time.sleep(delay)
                    elif delay < -1.0:
                        next_deadline = time.monotonic()
        finally:
            if viewer is not None:
                viewer.close()
            self.close()

    def close(self) -> None:
        if not self._closed:
            try:
                self.source.close()
            finally:
                self.backend.close()
                self._closed = True
