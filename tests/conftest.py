from pathlib import Path

import pytest


@pytest.fixture
def model_path() -> Path:
    return Path(__file__).resolve().parents[1] / "kinova_gen3_mujoco" / "scene.xml"


@pytest.fixture
def teleop_model_path() -> Path:
    return (
        Path(__file__).resolve().parents[1]
        / "kinova_gen3_mujoco"
        / "teleop_scene.xml"
    )
