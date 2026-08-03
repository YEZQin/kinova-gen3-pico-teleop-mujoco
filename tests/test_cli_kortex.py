"""CLI tests for the gated Kortex hardware path and its cleanup ordering."""

from __future__ import annotations

import builtins
from types import SimpleNamespace

import pytest

from kinova_teleop.main import _approve_fixed_segment, _cleanup_resources, main


def _motion_gate_args(arguments: list[str]) -> list[str]:
    return arguments + [
        "--workspace-min", "-1", "-1", "-1",
        "--workspace-max", "1", "1", "1",
        "--motion-lease", "fixture-motion.lock",
    ]


class _FakeSource:
    def __init__(self, *args, **kwargs) -> None:
        self.closed = False

    def read(self):
        raise AssertionError("CLI tests never read input samples")

    def close(self) -> None:
        self.closed = True


def test_fallback_cleanup_orders_motion_resources_first_and_attempts_every_close(
    capsys,
) -> None:
    """Fallback cleanup must try backend, connection, then XR despite failures."""

    events: list[str] = []

    class FailingController:
        def close(self) -> None:
            events.append("controller.close")
            raise RuntimeError("aggregate close failed")

    class Resource:
        def __init__(self, name: str, *, fails: bool) -> None:
            self.name = name
            self.fails = fails

        def close(self) -> None:
            events.append(f"{self.name}.close")
            if self.fails:
                raise RuntimeError(f"{self.name} close failed")

    assert not _cleanup_resources(
        FailingController(),
        Resource("source", fails=False),
        Resource("backend", fails=True),
        Resource("connection", fails=True),
        hardware=True,
    )
    assert events == [
        "controller.close",
        "backend.close",
        "connection.close",
        "source.close",
    ]
    assert "Kortex cleanup failed" in capsys.readouterr().err


def test_explicit_false_hardware_close_makes_cleanup_fail_secret_free(
    capsys,
) -> None:
    """Treating an explicit False close result as success must break this test."""

    class FalseConnection:
        def close(self) -> bool:
            return False

    assert not _cleanup_resources(
        None, None, None, FalseConnection(), hardware=True
    )
    error = capsys.readouterr().err
    assert "motion stop may be unconfirmed" in error
    assert "secret" not in error


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
        (
            ["--backend", "kortex", "--enable-hardware", "--control-hz", "40.1"],
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


def test_gripper_requires_kortex_backend(monkeypatch, capsys) -> None:
    calls = _install_unreachable_connection_factory(monkeypatch)

    assert main(["--gripper"]) == 2
    assert "--gripper is disabled" in capsys.readouterr().err
    assert calls == []


def test_fixed_segment_approval_requires_exact_move(monkeypatch) -> None:
    segment = SimpleNamespace(name="axis-x-plus")
    monkeypatch.setattr("builtins.input", lambda _prompt: "move")
    assert _approve_fixed_segment(segment, 0) is False
    monkeypatch.setattr("builtins.input", lambda _prompt: "MOVE")
    assert _approve_fixed_segment(segment, 0) is True


def test_first_hardware_gripper_gate_precedes_password_lookup(monkeypatch) -> None:
    calls = _install_unreachable_connection_factory(monkeypatch)
    monkeypatch.delenv("KINOVA_PASSWORD", raising=False)
    monkeypatch.setattr(
        "kinova_teleop.main.os.getenv",
        lambda _name: (_ for _ in ()).throw(AssertionError("password read")),
    )
    assert main(["--backend", "kortex", "--enable-hardware", "--gripper"]) == 2
    assert calls == []


def test_workspace_gate_precedes_password_and_prompt(monkeypatch) -> None:
    calls = _install_unreachable_connection_factory(monkeypatch)
    monkeypatch.setattr(
        "kinova_teleop.main.os.getenv",
        lambda _name: (_ for _ in ()).throw(AssertionError("password read")),
    )
    monkeypatch.setattr("builtins.input", lambda _prompt: (_ for _ in ()).throw(AssertionError("prompted")))
    assert main(["--backend", "kortex", "--enable-hardware"]) == 2
    assert calls == []


def test_check_kortex_uses_connect_and_read_only_rpc(monkeypatch) -> None:
    calls: list[str] = []

    class Base:
        def GetArmState(self, *, options=None):
            calls.append("GetArmState")
            return SimpleNamespace(active_state=31)

    class Cyclic:
        def RefreshFeedback(self, *, options=None):
            calls.append("RefreshFeedback")
            return SimpleNamespace(base=SimpleNamespace(
                tool_pose_x=0.0, tool_pose_y=0.0, tool_pose_z=0.3,
                tool_pose_theta_x=0.0, tool_pose_theta_y=0.0, tool_pose_theta_z=0.0,
            ))

    class Connection:
        base = Base()
        base_cyclic = Cyclic()
        base_pb2 = SimpleNamespace(ARMSTATE_SERVOING_READY=31, ARMSTATE_IN_FAULT=32)

        def rpc_options(self):
            return SimpleNamespace(timeout_ms=100)

        def close(self):
            calls.append("close")

    monkeypatch.setenv("KINOVA_PASSWORD", "secret")
    monkeypatch.setattr("builtins.input", lambda prompt: "CONNECT")
    monkeypatch.setattr("kinova_teleop.main._create_kortex_connection", lambda _config: Connection())
    assert main(["--backend", "kortex", "--enable-hardware", "--check-kortex"]) == 0
    assert calls[:2] == ["GetArmState", "RefreshFeedback"]
    assert "close" in calls


def test_kortex_rejects_stale_timeout_above_200_ms_before_prompt_import_or_connect(
    monkeypatch,
) -> None:
    """Allowing 0.200001 s to reach any hardware gate side effect is unsafe."""

    calls = _install_unreachable_connection_factory(monkeypatch)
    prompts: list[str] = []
    imports: list[str] = []
    monkeypatch.setenv("KINOVA_PASSWORD", "secret")
    monkeypatch.setattr(
        "builtins.input",
        lambda prompt: prompts.append(prompt) or "MOVE",
    )

    original_import = builtins.__import__

    def forbid_kortex_import(name, *args, **kwargs):
        if name.startswith("kinova_teleop.kortex"):
            imports.append(name)
            raise AssertionError("Kortex import must not occur")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", forbid_kortex_import)

    assert main(
        [
            "--backend",
            "kortex",
            "--enable-hardware",
            "--stale-timeout",
            "0.200001",
        ],
    ) == 2
    assert prompts == []
    assert imports == []
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


def test_mujoco_default_does_not_read_password_or_construct_kortex(
    monkeypatch,
) -> None:
    """The default path remains independent of Kortex credentials and SDKs."""

    calls = _install_unreachable_connection_factory(monkeypatch)

    class FakeController:
        steps = 1

        def __init__(self, *_args, **_kwargs) -> None:
            pass

        def run(self, **_kwargs) -> None:
            pass

        def close(self) -> None:
            pass

    monkeypatch.setattr("kinova_teleop.main.DryRunXrInput", _FakeSource)
    monkeypatch.setattr("kinova_teleop.main.TeleopController", FakeController)

    original_import = builtins.__import__

    def forbid_kortex_import(name, *args, **kwargs):
        if name.startswith("kinova_teleop.kortex"):
            raise AssertionError("MuJoCo must not import Kortex")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", forbid_kortex_import)
    monkeypatch.setattr(
        "kinova_teleop.main.os.getenv",
        lambda _name: (_ for _ in ()).throw(AssertionError("password read")),
    )

    assert main(["--dry-run", "--headless", "--steps", "1"]) == 0
    assert calls == []


def _install_valid_kortex_fakes(monkeypatch, created: dict) -> None:
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
    monkeypatch.setattr(
        "kinova_teleop.main._create_kortex_backend",
        FakeBackend,
        raising=False,
    )
    monkeypatch.setattr("kinova_teleop.main.TeleopController", FakeController)
    monkeypatch.setattr("kinova_teleop.main.validate_motion_lease", lambda *_args: object())
    monkeypatch.setenv("KINOVA_PASSWORD", "secret")
    monkeypatch.setattr("builtins.input", lambda _prompt: "MOVE")


def test_valid_kortex_path_connects_and_uses_hardware_defaults(monkeypatch) -> None:
    """The Kortex path is constructed only after exact MOVE confirmation."""

    created: dict[str, object] = {}
    _install_valid_kortex_fakes(monkeypatch, created)
    monkeypatch.setattr("kinova_teleop.main.SdkXrInput", _FakeSource)

    assert main(_motion_gate_args(
        ["--backend", "kortex", "--enable-hardware", "--input", "xrobotoolkit"],
    )) == 0
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
        "workspace_limits": backend.kwargs["workspace_limits"],
        "event_sink": None,
    }
    controller_config = created["controller_config"]
    assert controller_config.translation_scale == 0.5
    # Hardware defaults: reduced RPC rate, wall-clock pacing, gripper off.
    assert controller_config.control_hz == 25.0
    assert controller_config.realtime is True
    assert controller_config.gripper is False
    assert backend.closed


def test_kortex_default_input_is_pico_udp(monkeypatch) -> None:
    created: dict[str, object] = {}
    pico_kwargs: dict[str, object] = {}
    _install_valid_kortex_fakes(monkeypatch, created)

    class FakePicoInput(_FakeSource):
        def __init__(self, *args, **kwargs) -> None:
            super().__init__()
            pico_kwargs.update(kwargs)

    monkeypatch.setattr("kinova_teleop.main.PicoUdpInput", FakePicoInput)

    assert main(_motion_gate_args(["--backend", "kortex", "--enable-hardware"])) == 0
    assert isinstance(created["source"], FakePicoInput)
    assert pico_kwargs == {
        "host": "0.0.0.0",
        "port": 15031,
        "stale_after": 0.2,
    }


def test_kortex_gripper_flag_reaches_controller_config(monkeypatch) -> None:
    created: dict[str, object] = {}
    _install_valid_kortex_fakes(monkeypatch, created)
    monkeypatch.setattr("kinova_teleop.main.PicoUdpInput", _FakeSource)

    assert main(_motion_gate_args(["--backend", "kortex", "--enable-hardware", "--gripper"])) == 2
    assert "controller_config" not in created


def test_kortex_headless_stays_realtime_and_needs_no_steps(monkeypatch) -> None:
    created: dict[str, object] = {}
    _install_valid_kortex_fakes(monkeypatch, created)
    monkeypatch.setattr("kinova_teleop.main.PicoUdpInput", _FakeSource)

    assert main(_motion_gate_args(["--backend", "kortex", "--enable-hardware", "--headless"])) == 0
    assert created["controller_config"].realtime is True


def test_backend_construction_fallback_reports_false_connection_cleanup(
    monkeypatch,
    capsys,
) -> None:
    """Backend-construction failure must retain a False connection close result."""

    events: list[str] = []

    class FalseConnection:
        def close(self) -> bool:
            events.append("connection.close")
            return False

    monkeypatch.setattr("kinova_teleop.main.PicoUdpInput", _FakeSource)
    monkeypatch.setattr(
        "kinova_teleop.main._create_kortex_connection",
        lambda _config: FalseConnection(),
    )
    monkeypatch.setattr(
        "kinova_teleop.main._create_kortex_backend",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            RuntimeError("servoing setup failed")
        ),
    )
    monkeypatch.setenv("KINOVA_PASSWORD", "top-secret")
    monkeypatch.setattr("kinova_teleop.main.validate_motion_lease", lambda *_args: object())
    monkeypatch.setattr("builtins.input", lambda _prompt: "MOVE")

    assert main(_motion_gate_args(["--backend", "kortex", "--enable-hardware"])) == 2
    error = capsys.readouterr().err
    assert "motion stop may be unconfirmed" in error
    assert "top-secret" not in error
    assert events == ["connection.close"]


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

    class EventSource(_FakeSource):
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
            self.backend.close()
            self.source.close()

    monkeypatch.setattr(
        "kinova_teleop.main._create_kortex_connection",
        lambda _config: FakeConnection(),
        raising=False,
    )
    monkeypatch.setattr(
        "kinova_teleop.main._create_kortex_backend",
        FakeBackend,
        raising=False,
    )
    monkeypatch.setattr("kinova_teleop.main.PicoUdpInput", EventSource)
    monkeypatch.setattr("kinova_teleop.main.TeleopController", FakeController)
    monkeypatch.setenv("KINOVA_PASSWORD", "secret")
    monkeypatch.setattr("kinova_teleop.main.validate_motion_lease", lambda *_args: object())
    monkeypatch.setattr("builtins.input", lambda _prompt: "MOVE")

    assert main(_motion_gate_args(["--backend", "kortex", "--enable-hardware"])) == expected_code
    assert events == ["controller.close", "backend.close", "source.close"]


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

    class EventSource(_FakeSource):
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
    monkeypatch.setattr("kinova_teleop.main.PicoUdpInput", EventSource)
    monkeypatch.setattr("kinova_teleop.main.TeleopController", FakeController)
    monkeypatch.setenv("KINOVA_PASSWORD", "secret")
    monkeypatch.setattr("kinova_teleop.main.validate_motion_lease", lambda *_args: object())
    monkeypatch.setattr("builtins.input", lambda _prompt: "MOVE")

    assert main(_motion_gate_args(["--backend", "kortex", "--enable-hardware"])) == 2
    captured = capsys.readouterr()
    assert "completed" not in captured.out
    assert "Kortex cleanup failed" in captured.err
    assert "secret" not in captured.err
    assert events == [
        "controller.close",
        "backend.close",
        "backend.close",
        "connection.close",
        "source.close",
    ]
