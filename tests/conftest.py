from pathlib import Path

import pytest


@pytest.fixture
def model_path() -> Path:
    return Path(__file__).resolve().parents[1] / "kinova_gen3_mujoco" / "scene.xml"
