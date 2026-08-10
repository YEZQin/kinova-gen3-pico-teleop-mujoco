"""Offline first-hardware limits and compatibility checks for Kortex."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import importlib.metadata
import ipaddress
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
    profile: HardwareProfile = FIRST_HARDWARE_PROFILE,
) -> None:
    """Require absolute motion bounds to fit within the first-hardware span."""

    span = np.asarray(limits.maximum_xyz) - np.asarray(limits.minimum_xyz)
    maximum = 2.0 * np.asarray(profile.anchor_translation_axis_m)
    if np.any(span > maximum):
        raise ValueError("first-hardware workspace span must not exceed 0.04 m per axis")
