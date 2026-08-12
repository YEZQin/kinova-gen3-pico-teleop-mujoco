import pytest

from kinova_teleop.hardware_profile import (
    FIRST_HARDWARE_PROFILE,
    MAX_TRANSLATION_ONLY_SCALE,
    validate_kortex_runtime,
    validate_private_robot_ipv4,
    validate_workspace_span,
)
from kinova_teleop.main import build_parser, resolve_control_hz, resolve_translation_scale
from kinova_teleop.workspace import WorkspaceLimits


def test_first_hardware_profile_is_the_approved_envelope() -> None:
    profile = FIRST_HARDWARE_PROFILE
    assert profile.control_hz == 40.0
    assert profile.translation_scale == 0.25
    assert profile.stale_timeout_s == 0.2
    assert profile.max_linear_speed_mps == 0.005
    assert profile.max_angular_speed_deg_s == 2.0
    assert profile.anchor_translation_axis_m == (0.02, 0.02, 0.02)
    assert profile.anchor_rotation_deg == 5.0
    assert MAX_TRANSLATION_ONLY_SCALE == 0.5


def test_backend_specific_defaults_preserve_mujoco() -> None:
    parser = build_parser()
    mujoco = parser.parse_args([])
    kortex = parser.parse_args(["--backend", "kortex"])
    assert resolve_control_hz(mujoco) == 100.0
    assert resolve_translation_scale(mujoco) == 0.5
    assert resolve_control_hz(kortex) == 40.0
    assert resolve_translation_scale(kortex) == 0.25


def test_runtime_gate_requires_the_tested_sdk_stack() -> None:
    versions = {"kortex-api": "2.8.0.post5", "protobuf": "3.20.0"}
    observed = validate_kortex_runtime(
        python_version=(3, 11, 15),
        distribution_version=versions.__getitem__,
    )
    assert observed.kortex_api == "2.8.0.post5"
    with pytest.raises(RuntimeError, match="protobuf 3.20.0"):
        validate_kortex_runtime(
            python_version=(3, 11, 15),
            distribution_version=lambda name: "5.0.0" if name == "protobuf" else "2.8.0.post5",
        )


@pytest.mark.parametrize(
    "host",
    [
        "8.8.8.8",
        "127.0.0.1",
        "169.254.1.1",
        "192.0.2.1",
        "198.18.0.1",
        "198.51.100.1",
        "203.0.113.1",
        "240.0.0.1",
        "255.255.255.255",
        "::1",
        "robot.local",
    ],
)
def test_robot_endpoint_rejects_non_rfc1918_ipv4(host: str) -> None:
    with pytest.raises(ValueError, match="private IPv4"):
        validate_private_robot_ipv4(host)


@pytest.mark.parametrize(
    "host",
    [
        "10.0.0.1",
        "10.255.255.254",
        "172.16.0.1",
        "172.31.255.254",
        "192.168.0.1",
        "192.168.255.254",
    ],
)
def test_robot_endpoint_accepts_only_rfc1918_ipv4(host: str) -> None:
    assert validate_private_robot_ipv4(host) is None


def test_first_hardware_workspace_cannot_exceed_four_centimetres_per_axis() -> None:
    limits = WorkspaceLimits(
        (0.10, -0.20, 0.30),
        (0.140001, -0.16, 0.34),
    )
    with pytest.raises(ValueError, match="span must not exceed 0.04 m"):
        validate_workspace_span(limits, FIRST_HARDWARE_PROFILE)
