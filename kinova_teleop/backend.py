"""Backend protocol shared by simulation and hardware end-effector targets.

This module intentionally has no simulator or vendor SDK imports so that
safety-critical hardware code can depend on it without pulling in MuJoCo.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from .pose_mapping import Pose


@dataclass(frozen=True)
class BackendResult:
    accepted: bool
    converged: bool
    position_error: float
    rotation_error: float
    reason: str
    reanchor_required: bool = False


class EndEffectorTargetBackend(Protocol):
    def current_pose(self) -> Pose: ...

    def begin_control(self) -> None: ...

    def command_pose(self, target: Pose) -> BackendResult: ...

    def hold(self) -> None: ...

    def step(self) -> None: ...

    def close(self) -> None: ...


@runtime_checkable
class ProportionalGripperBackend(Protocol):
    def command_gripper(self, position: float) -> bool: ...
