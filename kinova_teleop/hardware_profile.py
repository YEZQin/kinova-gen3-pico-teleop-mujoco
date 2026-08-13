"""Offline first-hardware limits and compatibility checks for Kortex."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import importlib.metadata
import ipaddress
import math
import sys

import numpy as np

from .workspace import WorkspaceLimits


_RFC1918_NETWORKS = (
    ipaddress.ip_network("10.0.0.0/8"),
    ipaddress.ip_network("172.16.0.0/12"),
    ipaddress.ip_network("192.168.0.0/16"),
)


@dataclass(frozen=True)
class HardwareProfile:
    """Immutable envelope approved for the first physical-hardware run."""

    control_hz: float
    translation_scale: float
    stale_timeout_s: float
    max_linear_speed_mps: float
    max_angular_speed_deg_s: float
    anchor_translation_axis_m: tuple[float, float, float]
    anchor_rotation_deg: float


FIRST_HARDWARE_PROFILE = HardwareProfile(
    control_hz=40.0,
    translation_scale=0.25,
    stale_timeout_s=0.2,
    max_linear_speed_mps=0.005,
    max_angular_speed_deg_s=2.0,
    anchor_translation_axis_m=(0.02, 0.02, 0.02),
    anchor_rotation_deg=5.0,
)

EXPANDED_TRANSLATION_ONLY_ANCHOR_AXIS_M = (0.05, 0.05, 0.05)

MAX_TRANSLATION_ONLY_SCALE = 0.5
RESPONSIVE_TRANSLATION_MAX_SCALE = 0.8
RESPONSIVE_TRANSLATION_MAX_LINEAR_SPEED_MPS = 0.01
RESPONSIVE_TRANSLATION_WORKSPACE_HALF_WIDTH_AXIS_M = (0.1, 0.1, 0.1)
CALIBRATED_RESPONSIVE_TRANSLATION_MAX_LINEAR_SPEED_MPS = 0.02
CALIBRATED_RESPONSIVE_TRANSLATION_WORKSPACE_HALF_WIDTH_AXIS_M = (0.6, 0.6, 0.32)


@dataclass(frozen=True)
class KortexRuntimeVersions:
    """Versions observed by the Kortex runtime compatibility gate."""

    python: str
    kortex_api: str
    protobuf: str


def validate_kortex_runtime(
    *,
    python_version: tuple[int, int, int] | None = None,
    distribution_version: Callable[[str], str] = importlib.metadata.version,
) -> KortexRuntimeVersions:
    """Require the Python and SDK versions validated for first hardware use."""

    observed = python_version or sys.version_info[:3]
    if observed[:2] != (3, 11):
        raise RuntimeError("first-hardware Kortex runtime requires Python 3.11.x")
    kortex = distribution_version("kortex-api")
    protobuf = distribution_version("protobuf")
    if kortex != "2.8.0.post5":
        raise RuntimeError("first-hardware runtime requires kortex-api 2.8.0.post5")
    if protobuf != "3.20.0":
        raise RuntimeError("first-hardware runtime requires protobuf 3.20.0")
    return KortexRuntimeVersions(".".join(map(str, observed)), kortex, protobuf)


def validate_private_robot_ipv4(host: str) -> None:
    """Allow robot endpoints only within the three RFC1918 IPv4 blocks."""

    try:
        address = ipaddress.ip_address(host)
    except ValueError as error:
        raise ValueError("robot endpoint must be a private IPv4 address") from error
    if (
        not isinstance(address, ipaddress.IPv4Address)
        or not any(address in network for network in _RFC1918_NETWORKS)
    ):
        raise ValueError("robot endpoint must be a private IPv4 address")


def validate_workspace_span(
    limits: WorkspaceLimits,
    maximum_translation_axis_m: tuple[float, float, float] = (
        FIRST_HARDWARE_PROFILE.anchor_translation_axis_m
    ),
) -> None:
    """Require absolute motion bounds to fit within the supplied span."""

    if len(maximum_translation_axis_m) != 3 or not all(
        math.isfinite(value) and value > 0.0
        for value in maximum_translation_axis_m
    ):
        raise ValueError("maximum translation axis must contain three positive finite values")
    span = np.asarray(limits.maximum_xyz) - np.asarray(limits.minimum_xyz)
    maximum = 2.0 * np.asarray(maximum_translation_axis_m)
    tolerance = 4.0 * np.spacing(np.maximum(np.abs(span), maximum))
    if np.any(span > maximum + tolerance):
        formatted_maximum = ", ".join(f"{value:g}" for value in maximum)
        raise ValueError(
            "workspace span must not exceed per-axis maxima "
            f"[{formatted_maximum}] m"
        )
