from pathlib import Path

import numpy as np
import pytest

from kinova_teleop.main import build_parser, main


def test_dry_run_headless_cli_returns_success(teleop_model_path: Path) -> None:
    code = main(
        [
            "--model",
            str(teleop_model_path),
            "--dry-run",
            "--headless",
            "--steps",
            "100",
        ],
    )
    assert code == 0


def test_check_xr_reports_missing_sdk(monkeypatch, capsys) -> None:
    def unavailable():
        raise RuntimeError("xrobotoolkit_sdk is unavailable")

    monkeypatch.setattr("kinova_teleop.main.SdkXrInput", unavailable)
    assert main(["--check-xr", "--samples", "1"]) == 2
    assert "xrobotoolkit_sdk" in capsys.readouterr().err


def test_headless_requires_positive_steps(capsys) -> None:
    assert main(["--dry-run", "--headless"]) == 2
    assert "--steps" in capsys.readouterr().err


def test_only_left_controller_is_accepted() -> None:
    parser = build_parser()
    assert parser.parse_args(["--controller", "left"]).controller == "left"


def test_source_closes_if_controller_initialization_fails(monkeypatch) -> None:
    class FakeSource:
        closed = False

        def close(self) -> None:
            self.closed = True

    source = FakeSource()
    monkeypatch.setattr("kinova_teleop.main.SdkXrInput", lambda: source)

    def fail_controller(*_args, **_kwargs):
        raise RuntimeError("model rejected")

    monkeypatch.setattr("kinova_teleop.main.TeleopController", fail_controller)

    assert main([]) == 2
    assert source.closed


def _install_unreachable_connection_factory(monkeypatch):
    calls: list[object] = []

    def factory(*args, **kwargs):
        calls.append((args, kwargs))
        raise AssertionError("Kortex connection must not be created")

    monkeypatch.setattr(
        "kinova_teleop.main._create_kortex_connection",
        factory,
        raising=False,
    )
    return calls


@pytest.mark.parametrize(
    ("arguments", "password", "confirmation"),
    [
        (["--backend", "kortex"], "secret", "MOVE"),
        (["--backend", "kortex", "--enable-hardware"], None, "MOVE"),
        (["--backend", "kortex", "--enable-hardware"], "secret", "move"),
        (
            [
                "--backend",
                "kortex",
                "--enable-hardware",
                "--max-linear-speed",
                "0.031",
            ],
            "secret",
            "MOVE",
        ),
        (
            [
                "--backend",
                "kortex",
                "--enable-hardware",
                "--max-angular-speed-deg",
                "5.1",
            ],
            "secret",
            "MOVE",
        ),
        (
            ["--backend", "kortex", "--enable-hardware", "--dry-run"],
            "secret",
            "MOVE",
        ),
        (
            ["--backend", "kortex", "--enable-hardware", "--scale", "0.6"],
            "secret",
            "MOVE",
        ),
    ],
)
def test_kortex_rejections_happen_before_connection(
    monkeypatch,
    arguments: list[str],
    password: str | None,
    confirmation: str,
) -> None:
    """Removing any hardware pre-connect gate must make this test fail."""

    calls = _install_unreachable_connection_factory(monkeypatch)
    if password is None:
        monkeypatch.delenv("KINOVA_PASSWORD", raising=False)
    else:
        monkeypatch.setenv("KINOVA_PASSWORD", password)
    monkeypatch.setattr("builtins.input", lambda _prompt: confirmation)

    assert main(arguments) == 2
    assert calls == []


@pytest.mark.parametrize("option", ("--robot-ip", "--robot-user"))
def test_kortex_rejects_blank_connection_identity_before_prompt_or_import(
    monkeypatch,
    option: str,
) -> None:
    """Blank Kortex host or user must not reach confirmation, imports, or connect."""

    calls = _install_unreachable_connection_factory(monkeypatch)
    prompts: list[str] = []
    imports: list[str] = []
    monkeypatch.setenv("KINOVA_PASSWORD", "secret")
    monkeypatch.setattr("builtins.input", lambda prompt: prompts.append(prompt))

    import builtins

    original_import = builtins.__import__

    def forbid_kortex_import(name, *args, **kwargs):
        if "kortex" in name:
            imports.append(name)
            raise AssertionError("Kortex import must not occur")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", forbid_kortex_import)

    assert main(["--backend", "kortex", "--enable-hardware", option, "   "]) == 2
    assert prompts == []
    assert imports == []
    assert calls == []


def test_mujoco_default_does_not_construct_kortex_connection(monkeypatch) -> None:
    """The default path remains independent of Kortex credentials and SDKs."""

    calls = _install_unreachable_connection_factory(monkeypatch)

    class FakeSource:
        def __init__(self, **_kwargs) -> None:
            pass

        def close(self) -> None:
            pass

    class FakeController:
        steps = 1
        data = type(
            "State",
            (),
            {"qpos": np.zeros(1), "qvel": np.zeros(1), "ctrl": np.zeros(1)},
        )()

        def __init__(self, *_args, **_kwargs) -> None:
            pass

        def run(self, **_kwargs) -> None:
            pass

        def close(self) -> None:
            pass

    monkeypatch.setattr("kinova_teleop.main.DryRunXrInput", FakeSource)
    monkeypatch.setattr("kinova_teleop.main.TeleopController", FakeController)

    def forbid_kortex_import(name, *args, **kwargs):
        if name.startswith("kinova_teleop.kortex"):
            raise AssertionError("MuJoCo must not import Kortex")
        return original_import(name, *args, **kwargs)

    import builtins

    original_import = builtins.__import__
    monkeypatch.setattr(builtins, "__import__", forbid_kortex_import)
    monkeypatch.setattr(
        "kinova_teleop.main.os.getenv",
        lambda _name: (_ for _ in ()).throw(AssertionError("password read")),
    )

    assert main(["--dry-run", "--headless", "--steps", "1"]) == 0
    assert calls == []


def test_valid_kortex_path_connects_and_uses_hardware_scale(monkeypatch) -> None:
    """The Kortex path is constructed only after exact MOVE confirmation."""

    created: dict[str, object] = {}

    class FakeConnection:
        def close(self) -> None:
            pass

    class FakeBackend:
        def __init__(self, connection, **kwargs) -> None:
            self.connection = connection
            self.kwargs = kwargs
            self.closed = False

        def close(self) -> None:
            self.closed = True

    class FakeSource:
        def close(self) -> None:
            pass

    class FakeController:
        steps = 7

        def __init__(self, config, source, backend) -> None:
            created["controller_config"] = config
            created["source"] = source
            created["backend"] = backend

        def run(self, **_kwargs) -> None:
            pass

        def close(self) -> None:
            created["backend"].close()

    def factory(config):
        connection = FakeConnection()
        created["connection_config"] = config
        created["connection"] = connection
        return connection

    monkeypatch.setattr(
        "kinova_teleop.main._create_kortex_connection",
        factory,
        raising=False,
    )
    monkeypatch.setattr("kinova_teleop.main._create_kortex_backend", FakeBackend, raising=False)
    monkeypatch.setattr("kinova_teleop.main.SdkXrInput", FakeSource)
    monkeypatch.setattr("kinova_teleop.main.TeleopController", FakeController)
    monkeypatch.setenv("KINOVA_PASSWORD", "secret")
    monkeypatch.setattr("builtins.input", lambda _prompt: "MOVE")

    assert main(["--backend", "kortex", "--enable-hardware"]) == 0
    config = created["connection_config"]
    backend = created["backend"]
    connection = created["connection"]
    assert config.host == "192.168.1.10"
    assert config.username == "admin"
    assert config.password == "secret"
    assert backend.connection is connection
    assert backend.kwargs == {
        "max_linear_speed": 0.03,
        "max_angular_speed_deg": 5.0,
    }
    assert created["controller_config"].translation_scale == 0.5
    assert backend.closed


@pytest.mark.parametrize(
    ("error", "expected_code"),
    [(RuntimeError("controller failed"), 2), (KeyboardInterrupt(), 130)],
)
def test_kortex_errors_close_controller_and_backend(
    monkeypatch,
    error: BaseException,
    expected_code: int,
) -> None:
    """All controller exits release Kortex resources through the same cleanup path."""

    events: list[str] = []

    class FakeConnection:
        def close(self) -> None:
            events.append("connection.close")

    class FakeBackend:
        def __init__(self, *_args, **_kwargs) -> None:
            pass

        def close(self) -> None:
            events.append("backend.close")

    class FakeSource:
        def __init__(self) -> None:
            pass

        def close(self) -> None:
            events.append("source.close")

    class FakeController:
        steps = 0

        def __init__(self, _config, source, backend) -> None:
            self.source = source
            self.backend = backend

        def run(self, **_kwargs) -> None:
            raise error

        def close(self) -> None:
            events.append("controller.close")
            self.source.close()
            self.backend.close()

    monkeypatch.setattr(
        "kinova_teleop.main._create_kortex_connection",
        lambda _config: FakeConnection(),
        raising=False,
    )
    monkeypatch.setattr("kinova_teleop.main._create_kortex_backend", FakeBackend, raising=False)
    monkeypatch.setattr("kinova_teleop.main.SdkXrInput", FakeSource)
    monkeypatch.setattr("kinova_teleop.main.TeleopController", FakeController)
    monkeypatch.setenv("KINOVA_PASSWORD", "secret")
    monkeypatch.setattr("builtins.input", lambda _prompt: "MOVE")

    assert main(["--backend", "kortex", "--enable-hardware"]) == expected_code
    assert events == ["controller.close", "source.close", "backend.close"]


def test_kortex_cleanup_failure_blocks_success_and_closes_remaining_resources(
    monkeypatch,
    capsys,
) -> None:
    """A Stop-unconfirmed cleanup failure must not be reported as completion."""

    from kinova_teleop.kortex_backend import KortexSafetyError

    events: list[str] = []

    class FakeConnection:
        def close(self) -> None:
            events.append("connection.close")

    class FakeBackend:
        def close(self) -> None:
            events.append("backend.close")
            raise KortexSafetyError("Stop attempted but unconfirmed")

    class FakeSource:
        def close(self) -> None:
            events.append("source.close")

    class FakeController:
        steps = 4

        def __init__(self, _config, source, backend) -> None:
            self.source = source
            self.backend = backend

        def run(self, **_kwargs) -> None:
            pass

        def close(self) -> None:
            events.append("controller.close")
            self.backend.close()

    monkeypatch.setattr(
        "kinova_teleop.main._create_kortex_connection",
        lambda _config: FakeConnection(),
    )
    monkeypatch.setattr(
        "kinova_teleop.main._create_kortex_backend",
        lambda *_args, **_kwargs: FakeBackend(),
    )
    monkeypatch.setattr("kinova_teleop.main.SdkXrInput", FakeSource)
    monkeypatch.setattr("kinova_teleop.main.TeleopController", FakeController)
    monkeypatch.setenv("KINOVA_PASSWORD", "secret")
    monkeypatch.setattr("builtins.input", lambda _prompt: "MOVE")

    assert main(["--backend", "kortex", "--enable-hardware"]) == 2
    captured = capsys.readouterr()
    assert "completed" not in captured.out
    assert "Kortex cleanup failed" in captured.err
    assert "secret" not in captured.err
    assert events == [
        "controller.close",
        "backend.close",
        "source.close",
        "backend.close",
        "connection.close",
    ]
