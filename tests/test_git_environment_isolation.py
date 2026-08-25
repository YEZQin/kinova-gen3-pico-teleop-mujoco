"""Regression tests for Git fixture isolation under repository hooks."""

from __future__ import annotations

from pathlib import Path
import subprocess

from conftest import GIT_LOCAL_ENVIRONMENT_VARIABLES
from conftest import clear_inherited_git_local_environment


def test_git_local_environment_cleanup_preserves_unrelated_values() -> None:
    environment = {
        **{name: "poisoned" for name in GIT_LOCAL_ENVIRONMENT_VARIABLES},
        "PATH": "keep-this-value",
        "UNRELATED": "also-keep",
    }

    clear_inherited_git_local_environment(environment)

    assert all(name not in environment for name in GIT_LOCAL_ENVIRONMENT_VARIABLES)
    assert environment == {"PATH": "keep-this-value", "UNRELATED": "also-keep"}


def test_cleaned_environment_initializes_the_requested_fixture_repository(
    tmp_path: Path,
) -> None:
    fixture = tmp_path / "repo"
    fixture.mkdir()
    environment = {
        "PATH": __import__("os").environ["PATH"],
        "GIT_DIR": str(Path.cwd() / ".git"),
        "GIT_WORK_TREE": str(Path.cwd()),
    }
    clear_inherited_git_local_environment(environment)

    subprocess.run(["git", "init", "--quiet"], cwd=fixture, env=environment, check=True)
    subprocess.run(
        [
            "git",
            "-c",
            "user.name=Kinova Test",
            "-c",
            "user.email=kinova-test@example.invalid",
            "commit",
            "--allow-empty",
            "--quiet",
            "-m",
            "fixture",
        ],
        cwd=fixture,
        env=environment,
        check=True,
    )

    root = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        cwd=fixture,
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    assert Path(root).resolve() == fixture.resolve()
