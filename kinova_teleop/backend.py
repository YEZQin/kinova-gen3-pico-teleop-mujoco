"""Backend interface for end-effector target teleoperation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from .pose_mapping import Pose


@dataclass(frozen=True)
class BackendResult:
    converged: bool = True
    position_error: float = 0.0
    rotation_error: float = 0.0


class EndEffectorTargetBackend(Protocol):
    def current_pose(self) -> Pose: ...

    def begin_control(self) -> None: ...

    def command_pose(self, target: Pose) -> BackendResult: ...

    def hold(self) -> None: ...

    def step(self) -> None: ...

    def close(self) -> None: ...
