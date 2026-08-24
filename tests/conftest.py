from collections.abc import MutableMapping
import os
from pathlib import Path

import pytest


# Git exports repository-local variables to hooks.  If a pre-push hook starts
# pytest with those values intact, a fixture's `git init`/`git commit` can act
# on the parent repository despite setting cwd to a temporary directory.
GIT_LOCAL_ENVIRONMENT_VARIABLES = (
    "GIT_ALTERNATE_OBJECT_DIRECTORIES",
    "GIT_CONFIG",
    "GIT_CONFIG_PARAMETERS",
    "GIT_CONFIG_COUNT",
    "GIT_OBJECT_DIRECTORY",
    "GIT_DIR",
    "GIT_WORK_TREE",
    "GIT_IMPLICIT_WORK_TREE",
    "GIT_GRAFT_FILE",
    "GIT_INDEX_FILE",
    "GIT_NO_REPLACE_OBJECTS",
    "GIT_REPLACE_REF_BASE",
    "GIT_PREFIX",
    "GIT_SHALLOW_FILE",
    "GIT_COMMON_DIR",
)


def clear_inherited_git_local_environment(
    environment: MutableMapping[str, str],
) -> None:
    """Remove only Git's repository-local hook context from an environment."""

    for name in GIT_LOCAL_ENVIRONMENT_VARIABLES:
        environment.pop(name, None)


def pytest_sessionstart(session: pytest.Session) -> None:
    """Keep all temporary Git fixtures isolated when pytest runs from a hook."""

    del session
    clear_inherited_git_local_environment(os.environ)


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
