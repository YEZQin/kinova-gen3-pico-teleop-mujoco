"""CLI tests for the gated Kortex hardware path and its cleanup ordering."""

from __future__ import annotations

import builtins
import importlib
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from kinova_teleop.main import _approve_fixed_segment, _cleanup_resources, main
from kinova_teleop.xr_input import ControllerSample


main_module = importlib.import_module("kinova_teleop.main")


def _motion_gate_args(arguments: list[str]) -> list[str]:
    return arguments + [
        "--workspace-min", "0", "0", "0",
        "--workspace-max", "0.04", "0.04", "0.04",
        "--motion-lease", "fixture-motion.lock",
        "--preflight-report", "fixture-preflight.json",
    ]


def _expanded_motion_gate_args(arguments: list[str]) -> list[str]:
    return arguments + [
        "--workspace-min", "0", "0", "0",
        "--workspace-max", "0.10", "0.10", "0.10",
        "--motion-lease", "fixture-motion.lock",
        "--preflight-report", "fixture-preflight.json",
    ]


def _valid_motion_argv() -> list[str]:
    return _motion_gate_args(["--backend", "kortex", "--enable-hardware"])


def _valid_fixed_motion_argv() -> list[str]:
    return _motion_gate_args(
        [
            "--backend",
            "kortex",
            "--enable-hardware",
            "--input",
            "none",
            "--fixed-trajectory",
            "fixture-trajectory.json",
        ]
    )


def _install_valid_motion_gate_files(monkeypatch) -> None:
    monkeypatch.setattr(main_module, "validate_motion_lease", lambda *_args: object())
    monkeypatch.setattr(
        main_module,
        "load_passing_preflight_report",
        lambda *_args: object(),
    )
    monkeypatch.setenv("KINOVA_PASSWORD", "test-password")


def _admitted() -> SimpleNamespace:
    return SimpleNamespace(last_timestamp_ns=10)


def _ready_report() -> SimpleNamespace:
    return SimpleNamespace()


class _FakeReleasedPico:
    def close(self) -> None:
        pass


class _FakeBackend:
    def close(self) -> None:
        pass


class _FakeController:
    steps = 1

    def __init__(self, *_args, **_kwargs) -> None:
        pass

    def run(self, **_kwargs) -> None:
        pass

    def close(self) -> None:
        pass


def _readonly_then_motion_connection(events: list[str]):
    class Connection:
        def __init__(self, name: str) -> None:
            self.name = name

        def close(self, *, send_stop: bool = True) -> bool:
            assert send_stop is (self.name == "motion")
            events.append(f"{self.name}_close")
            return True

    def factory(_config, *, read_only: bool = False):
        name = "readonly" if read_only else "motion"
        events.append(f"{name}_connect")
        return Connection(name)

    return factory


def _install_passing_motion_admission(monkeypatch) -> None:
    monkeypatch.setattr(main_module, "validate_kortex_runtime", lambda: object())
    monkeypatch.setattr(
        main_module,
        "wait_for_fresh_released_input",
        lambda *_args, **_kwargs: _admitted(),
    )
    monkeypatch.setattr(
        main_module,
        "run_kortex_readonly_preflight",
        lambda *_args, **_kwargs: _ready_report(),
    )
    monkeypatch.setattr(main_module, "require_live_kortex_ready", lambda _report: None)
    monkeypatch.setattr(main_module, "confirm_move", lambda: None)
    monkeypatch.setattr(main_module, "verify_released_now", lambda *_args, **_kwargs: None)


def test_motion_startup_order_is_input_readonly_move_recheck_then_backend(
    monkeypatch,
) -> None:
    """Kortex motion must admit input and live readiness before MOVE/connect."""

    events: list[str] = []
    _install_valid_motion_gate_files(monkeypatch)
    monkeypatch.setattr(main_module, "validate_kortex_runtime", lambda: events.append("runtime") or object())
    monkeypatch.setattr(main_module, "create_input", lambda _args: events.append("source") or _FakeReleasedPico())
    monkeypatch.setattr(
        main_module,
        "wait_for_fresh_released_input",
        lambda _source, **_kwargs: events.append("input_admit") or _admitted(),
    )
    monkeypatch.setattr(
        main_module,
        "_create_kortex_connection",
        _readonly_then_motion_connection(events),
    )
    monkeypatch.setattr(
        main_module,
        "run_kortex_readonly_preflight",
        lambda *_args, **_kwargs: events.append("readonly_preflight") or _ready_report(),
    )
    monkeypatch.setattr(
        main_module,
        "require_live_kortex_ready",
        lambda _report: events.append("readonly_accept"),
    )
    monkeypatch.setattr(main_module, "confirm_move", lambda: events.append("MOVE"))
    monkeypatch.setattr(
        main_module,
        "verify_released_now",
        lambda *_args, **_kwargs: events.append("release_recheck"),
    )
    monkeypatch.setattr(
        main_module,
        "_create_kortex_backend",
        lambda *_args, **_kwargs: events.append("backend") or _FakeBackend(),
    )
    monkeypatch.setattr(main_module, "TeleopController", _FakeController)

    assert main(_valid_motion_argv()) == 0
    assert events[:10] == [
        "runtime",
        "source",
        "input_admit",
        "readonly_connect",
        "readonly_preflight",
        "readonly_accept",
        "readonly_close",
        "MOVE",
        "release_recheck",
        "motion_connect",
    ]
    assert events.index("backend") > events.index("motion_connect")


@pytest.mark.parametrize(
    "failed_stage",
    [
        "runtime",
        "input_admit",
        "pressed_grip",
        "source_changed",
        "firmware",
        "operating_mode",
        "servoing_mode",
        "readonly_cleanup",
        "confirmation",
        "release_recheck",
    ],
)
def test_kortex_admission_failures_cannot_reach_motion_connect(
    monkeypatch,
    failed_stage: str,
) -> None:
    """Every admission failure is terminal before a motion transport exists."""

    events: list[str] = []
    _install_valid_motion_gate_files(monkeypatch)
    monkeypatch.setattr(main_module, "validate_kortex_runtime", lambda: object())
    monkeypatch.setattr(main_module, "create_input", lambda _args: _FakeReleasedPico())
    monkeypatch.setattr(main_module, "wait_for_fresh_released_input", lambda *_args, **_kwargs: _admitted())
    monkeypatch.setattr(main_module, "_create_kortex_connection", _readonly_then_motion_connection(events))
    monkeypatch.setattr(main_module, "run_kortex_readonly_preflight", lambda *_args, **_kwargs: _ready_report())
    monkeypatch.setattr(main_module, "require_live_kortex_ready", lambda _report: None)
    monkeypatch.setattr(main_module, "confirm_move", lambda: None)
    monkeypatch.setattr(main_module, "verify_released_now", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(main_module, "_create_kortex_backend", lambda *_args, **_kwargs: _FakeBackend())
    monkeypatch.setattr(main_module, "TeleopController", _FakeController)

    failure = RuntimeError(f"{failed_stage} rejected")
    if failed_stage == "runtime":
        monkeypatch.setattr(main_module, "validate_kortex_runtime", lambda: (_ for _ in ()).throw(failure))
    elif failed_stage in {"input_admit", "pressed_grip", "source_changed"}:
        monkeypatch.setattr(main_module, "wait_for_fresh_released_input", lambda *_args, **_kwargs: (_ for _ in ()).throw(failure))
    elif failed_stage in {"firmware", "operating_mode", "servoing_mode"}:
        monkeypatch.setattr(main_module, "require_live_kortex_ready", lambda _report: (_ for _ in ()).throw(failure))
    elif failed_stage == "readonly_cleanup":
        def cleanup_fails(_config, *, read_only: bool = False):
            if not read_only:
                events.append("motion_connect")
            return SimpleNamespace(close=lambda **_kwargs: False)
        monkeypatch.setattr(main_module, "_create_kortex_connection", cleanup_fails)
    elif failed_stage == "confirmation":
        monkeypatch.setattr(main_module, "confirm_move", lambda: (_ for _ in ()).throw(failure))
    elif failed_stage == "release_recheck":
        monkeypatch.setattr(main_module, "verify_released_now", lambda *_args, **_kwargs: (_ for _ in ()).throw(failure))

    assert main(_valid_motion_argv()) == 2
    assert events.count("motion_connect") == 0


@pytest.mark.parametrize("failed_stage", ("runtime", "live_readiness"))
def test_fixed_trajectory_admission_failure_cannot_reach_motion_or_send_twist(
    monkeypatch,
    failed_stage: str,
) -> None:
    """Fixed Kortex motion shares runtime and live-readiness admission gates."""

    events: list[str] = []
    twists: list[object] = []
    _install_valid_motion_gate_files(monkeypatch)
    monkeypatch.setattr(main_module, "load_trajectory", lambda _path: object())
    monkeypatch.setattr(main_module, "validate_kortex_runtime", lambda: object())
    monkeypatch.setattr(
        main_module,
        "create_input",
        lambda _args: (_ for _ in ()).throw(AssertionError("fixed trajectory has no PICO input")),
    )

    class ReadonlyConnection:
        base = SimpleNamespace(SendTwistCommand=lambda *args, **kwargs: twists.append(args))

        def close(self, **_kwargs) -> bool:
            return True

    def connection_factory(_config, *, read_only: bool = False):
        events.append("readonly_connect" if read_only else "motion_connect")
        return ReadonlyConnection()

    monkeypatch.setattr(main_module, "_create_kortex_connection", connection_factory)
    monkeypatch.setattr(main_module, "run_kortex_readonly_preflight", lambda *_args: _ready_report())
    monkeypatch.setattr(main_module, "require_live_kortex_ready", lambda _report: None)
    monkeypatch.setattr(main_module, "confirm_move", lambda: events.append("MOVE"))

    failure = RuntimeError(f"{failed_stage} rejected")
    if failed_stage == "runtime":
        monkeypatch.setattr(
            main_module,
            "validate_kortex_runtime",
            lambda: (_ for _ in ()).throw(failure),
        )
    else:
        monkeypatch.setattr(
            main_module,
            "require_live_kortex_ready",
            lambda _report: (_ for _ in ()).throw(failure),
        )

    assert main(_valid_fixed_motion_argv()) == 2
    assert events.count("motion_connect") == 0
    assert twists == []


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
    "arguments",
    [
        ["--expanded-translation-envelope"],
        ["--backend", "kortex", "--enable-hardware", "--expanded-translation-envelope"],
        [
            "--backend",
            "kortex",
            "--enable-hardware",
            "--translation-only",
            "--check-kortex",
            "--expanded-translation-envelope",
        ],
        [
            "--backend",
            "kortex",
            "--enable-hardware",
            "--translation-only",
            "--input",
            "none",
            "--fixed-trajectory",
            "fixture-trajectory.json",
            "--expanded-translation-envelope",
        ],
    ],
)
def test_expanded_envelope_invalid_modes_reject_before_connection(
    monkeypatch, capsys, arguments: list[str]
) -> None:
    calls = _install_unreachable_connection_factory(monkeypatch)
    monkeypatch.setenv("KINOVA_PASSWORD", "secret")

    assert main(_motion_gate_args(arguments)) == 2
    assert calls == []
    assert "--expanded-translation-envelope requires" in capsys.readouterr().err


@pytest.mark.parametrize("input_check", ("--check-input", "--check-xr"))
def test_expanded_envelope_input_checks_reject_before_hardware_side_effects(
    monkeypatch,
    capsys,
    input_check: str,
) -> None:
    """Input-only preflight must not be admitted as hardware teleoperation."""

    calls = _install_unreachable_connection_factory(monkeypatch)
    prompts: list[str] = []
    monkeypatch.setattr(
        "kinova_teleop.main.os.getenv",
        lambda _name: (_ for _ in ()).throw(AssertionError("password read")),
    )
    monkeypatch.setattr(
        "kinova_teleop.main.validate_kortex_runtime",
        lambda: (_ for _ in ()).throw(AssertionError("runtime checked")),
    )
    monkeypatch.setattr(
        "kinova_teleop.main.create_input",
        lambda _args: (_ for _ in ()).throw(AssertionError("input preflight ran")),
    )
    monkeypatch.setattr("builtins.input", lambda prompt: prompts.append(prompt) or "MOVE")

    assert main(
        [
            "--backend",
            "kortex",
            "--enable-hardware",
            "--translation-only",
            "--expanded-translation-envelope",
            "--input",
            "xrobotoolkit",
            input_check,
            "--samples",
            "1",
        ]
    ) == 2
    assert calls == []
    assert prompts == []
    assert "--expanded-translation-envelope requires" in capsys.readouterr().err


def test_expanded_envelope_disabled_input_rejects_before_hardware_side_effects(
    monkeypatch,
    capsys,
) -> None:
    """An input-disabled motion path is not live hardware teleoperation."""

    calls = _install_unreachable_connection_factory(monkeypatch)
    prompts: list[str] = []
    monkeypatch.setattr(main_module, "validate_motion_lease", lambda *_args: object())
    monkeypatch.setattr(
        main_module,
        "load_passing_preflight_report",
        lambda *_args: object(),
    )
    monkeypatch.setattr(
        "kinova_teleop.main.os.getenv",
        lambda _name: (_ for _ in ()).throw(AssertionError("password read")),
    )
    monkeypatch.setattr(
        "kinova_teleop.main.validate_kortex_runtime",
        lambda: (_ for _ in ()).throw(AssertionError("runtime checked")),
    )
    monkeypatch.setattr(
        "kinova_teleop.main.create_input",
        lambda _args: (_ for _ in ()).throw(AssertionError("input constructed")),
    )
    monkeypatch.setattr("builtins.input", lambda prompt: prompts.append(prompt) or "MOVE")

    assert main(
        _expanded_motion_gate_args(
            [
                "--backend",
                "kortex",
                "--enable-hardware",
                "--translation-only",
                "--expanded-translation-envelope",
                "--input",
                "none",
            ]
        )
    ) == 2
    assert calls == []
    assert prompts == []
    assert "--expanded-translation-envelope requires" in capsys.readouterr().err


def test_expanded_envelope_workspace_overrun_rejects_before_hardware_side_effects(
    monkeypatch,
    capsys,
) -> None:
    """A representable span overrun must fail before any hardware setup."""

    calls = _install_unreachable_connection_factory(monkeypatch)
    prompts: list[str] = []
    monkeypatch.setattr(
        "kinova_teleop.main.os.getenv",
        lambda _name: (_ for _ in ()).throw(AssertionError("password read")),
    )
    monkeypatch.setattr(
        "kinova_teleop.main.validate_kortex_runtime",
        lambda: (_ for _ in ()).throw(AssertionError("runtime checked")),
    )
    monkeypatch.setattr("builtins.input", lambda prompt: prompts.append(prompt) or "MOVE")
    argv = _expanded_motion_gate_args(
        [
            "--backend",
            "kortex",
            "--enable-hardware",
            "--input",
            "xrobotoolkit",
            "--translation-only",
            "--expanded-translation-envelope",
        ]
    )
    workspace_max_index = argv.index("--workspace-max")
    argv[workspace_max_index + 1] = "0.100001"

    assert main(argv) == 2
    assert calls == []
    assert prompts == []
    assert "workspace span must not exceed 0.1 m per axis" in capsys.readouterr().err


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
                "0.0051",
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
                "2.1",
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
            ["--backend", "kortex", "--enable-hardware", "--scale", "0.26"],
            "secret",
            "MOVE",
        ),
        (
            [
                "--backend",
                "kortex",
                "--enable-hardware",
                "--translation-only",
                "--scale",
                "0.500001",
            ],
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


def test_deprecated_gripper_flag_rejects_before_hardware_setup(monkeypatch, capsys) -> None:
    sdk_imports = 0
    robot_connections = 0
    move_prompts = 0
    original_import = builtins.__import__

    def count_sdk_imports(name, *args, **kwargs):
        nonlocal sdk_imports
        if name.startswith("kortex_api"):
            sdk_imports += 1
        return original_import(name, *args, **kwargs)

    def count_robot_connections(*_args, **_kwargs):
        nonlocal robot_connections
        robot_connections += 1
        raise AssertionError("robot connection must not be created")

    def count_move_prompts(_prompt: str) -> str:
        nonlocal move_prompts
        move_prompts += 1
        raise AssertionError("motion confirmation must not be prompted")

    monkeypatch.setattr(builtins, "__import__", count_sdk_imports)
    monkeypatch.setattr(
        "kinova_teleop.main._create_kortex_connection",
        count_robot_connections,
        raising=False,
    )
    monkeypatch.setattr("builtins.input", count_move_prompts)

    assert main(["--gripper"]) == 2
    assert "--gripper is disabled" in capsys.readouterr().err
    assert sdk_imports == 0
    assert robot_connections == 0
    assert move_prompts == 0


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


def test_preflight_report_gate_precedes_password_and_prompt(monkeypatch) -> None:
    calls = _install_unreachable_connection_factory(monkeypatch)
    monkeypatch.setattr("kinova_teleop.main.validate_motion_lease", lambda *_args: object())
    monkeypatch.setattr(
        "kinova_teleop.main.os.getenv",
        lambda _name: (_ for _ in ()).throw(AssertionError("password read")),
    )
    monkeypatch.setattr(
        "builtins.input",
        lambda _prompt: (_ for _ in ()).throw(AssertionError("prompted")),
    )
    args = [
        "--backend",
        "kortex",
        "--enable-hardware",
        "--workspace-min",
        "0",
        "0",
        "0",
        "--workspace-max",
        "0.04",
        "0.04",
        "0.04",
        "--motion-lease",
        "fixture-motion.lock",
    ]
    assert main(args) == 2
    assert calls == []


def test_check_kortex_uses_connect_and_read_only_rpc(monkeypatch) -> None:
    calls: list[str] = []
    factory_kwargs: dict[str, object] = {}

    class Base:
        def GetArmState(self, *, options=None):
            calls.append("GetArmState")
            return SimpleNamespace(active_state=31)

        def GetProductConfiguration(self, *, options=None):
            return SimpleNamespace(model="MODEL_ID_L53", degree_of_freedom=7)

        def GetOperatingMode(self, *, options=None):
            return SimpleNamespace(operating_mode="RUN_MODE")

        def GetServoingMode(self, *, options=None):
            return SimpleNamespace(servoing_mode="SINGLE_LEVEL_SERVOING")

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
        base_pb2 = SimpleNamespace(
            ARMSTATE_SERVOING_READY=31,
            ARMSTATE_IN_FAULT=32,
            RUN_MODE="RUN_MODE",
            SINGLE_LEVEL_SERVOING="SINGLE_LEVEL_SERVOING",
        )
        device_config = SimpleNamespace(
            GetFirmwareVersion=lambda **_: SimpleNamespace(firmware_version=0x05020800),
        )

        def rpc_options(self):
            return SimpleNamespace(timeout_ms=100)

        def readonly_rpc_options(self):
            return SimpleNamespace(timeout_ms=5_000)

        def close(self, *, send_stop=True):
            calls.append(f"close:{send_stop}")

    monkeypatch.setenv("KINOVA_PASSWORD", "secret")
    monkeypatch.setattr("builtins.input", lambda prompt: "CONNECT")
    def create_connection(_config, **kwargs):
        factory_kwargs.update(kwargs)
        return Connection()

    monkeypatch.setattr("kinova_teleop.main._create_kortex_connection", create_connection)
    assert main(["--backend", "kortex", "--enable-hardware", "--check-kortex"]) == 0
    assert factory_kwargs == {"read_only": True}
    assert calls[:2] == ["GetArmState", "RefreshFeedback"]
    assert "close:False" in calls


def test_check_kortex_cleanup_failure_is_fatal(monkeypatch, capsys) -> None:
    class Connection:
        base = SimpleNamespace(
            GetArmState=lambda **_: SimpleNamespace(active_state=31),
        )
        base_cyclic = SimpleNamespace(
            RefreshFeedback=lambda **_: SimpleNamespace(base=SimpleNamespace(
                tool_pose_x=0.0,
                tool_pose_y=0.0,
                tool_pose_z=0.3,
                tool_pose_theta_x=0.0,
                tool_pose_theta_y=0.0,
                tool_pose_theta_z=0.0,
            )),
        )
        base_pb2 = SimpleNamespace(ARMSTATE_SERVOING_READY=31, ARMSTATE_IN_FAULT=32)

        def rpc_options(self):
            return SimpleNamespace(timeout_ms=100)

        def close(self, *, send_stop=True):
            return False

    monkeypatch.setenv("KINOVA_PASSWORD", "secret")
    monkeypatch.setattr("builtins.input", lambda _prompt: "CONNECT")
    monkeypatch.setattr(
        "kinova_teleop.main._create_kortex_connection",
        lambda _config, **_kwargs: Connection(),
    )
    assert main(["--backend", "kortex", "--enable-hardware", "--check-kortex"]) == 2
    assert "error: cleanup failed" in capsys.readouterr().err


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
        def close(self, *, send_stop: bool = True) -> bool:
            return True

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

    def factory(config, *, read_only: bool = False):
        connection = FakeConnection()
        if not read_only:
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
    monkeypatch.setattr("kinova_teleop.main.load_passing_preflight_report", lambda *_args: object())
    monkeypatch.setenv("KINOVA_PASSWORD", "secret")
    monkeypatch.setattr("builtins.input", lambda _prompt: "MOVE")
    _install_passing_motion_admission(monkeypatch)


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
        "max_linear_speed": 0.005,
        "max_angular_speed_deg": 2.0,
        "workspace_limits": backend.kwargs["workspace_limits"],
        "anchor_envelope": backend.kwargs["anchor_envelope"],
        "event_sink": None,
    }
    assert backend.kwargs["anchor_envelope"].maximum_translation_axis_m == (0.02, 0.02, 0.02)
    assert backend.kwargs["anchor_envelope"].maximum_rotation_rad == pytest.approx(0.0872664626)
    controller_config = created["controller_config"]
    assert controller_config.translation_scale == 0.25
    # Hardware defaults: reduced RPC rate and wall-clock pacing.
    assert controller_config.control_hz == 40.0
    assert controller_config.realtime is True
    assert controller_config.fatal_input_faults is True
    assert backend.closed


def test_translation_only_kortex_disables_orientation_mapping(monkeypatch) -> None:
    created: dict[str, object] = {}
    _install_valid_kortex_fakes(monkeypatch, created)
    monkeypatch.setattr("kinova_teleop.main.SdkXrInput", _FakeSource)

    assert main(_motion_gate_args([
        "--backend",
        "kortex",
        "--enable-hardware",
        "--input",
        "xrobotoolkit",
        "--translation-only",
    ])) == 0

    assert created["controller_config"].orientation_enabled is False
    assert (
        created["backend"].kwargs["anchor_envelope"].maximum_translation_axis_m
        == (0.02, 0.02, 0.02)
    )


def test_expanded_translation_only_wires_five_centimetre_anchor(monkeypatch) -> None:
    created: dict[str, object] = {}
    _install_valid_kortex_fakes(monkeypatch, created)
    monkeypatch.setattr("kinova_teleop.main.SdkXrInput", _FakeSource)
    argv = _expanded_motion_gate_args(
        [
            "--backend",
            "kortex",
            "--enable-hardware",
            "--input",
            "xrobotoolkit",
            "--translation-only",
            "--expanded-translation-envelope",
            "--scale",
            "0.5",
        ]
    )

    assert main(argv) == 0
    envelope = created["backend"].kwargs["anchor_envelope"]
    assert envelope.maximum_translation_axis_m == (0.05, 0.05, 0.05)
    assert created["controller_config"].orientation_enabled is False
    assert created["controller_config"].translation_scale == 0.5


def test_responsive_reversed_expanded_translation_wires_approved_values(monkeypatch) -> None:
    created: dict[str, object] = {}
    _install_valid_kortex_fakes(monkeypatch, created)
    monkeypatch.setattr("kinova_teleop.main.SdkXrInput", _FakeSource)
    argv = _expanded_motion_gate_args(
        [
            "--backend", "kortex",
            "--enable-hardware",
            "--input", "xrobotoolkit",
            "--translation-only",
            "--expanded-translation-envelope",
            "--responsive-translation-profile",
            "--invert-translation",
            "--recover-stale-input",
            "--scale", "0.8",
            "--max-linear-speed", "0.01",
        ]
    )

    assert main(argv) == 0
    assert created["backend"].kwargs["max_linear_speed"] == 0.01
    assert created["backend"].kwargs["anchor_envelope"].maximum_translation_axis_m == (0.05, 0.05, 0.05)
    controller_config = created["controller_config"]
    assert controller_config.translation_scale == 0.8
    assert controller_config.orientation_enabled is False
    assert controller_config.invert_translation is True
    assert controller_config.recover_stale_input is True
    assert controller_config.recovery_release_samples == 3


def test_responsive_translation_accepts_twenty_centimetre_absolute_workspace(
    monkeypatch,
) -> None:
    """Responsive motion may use a wider absolute box without widening Grip."""

    created: dict[str, object] = {}
    _install_valid_kortex_fakes(monkeypatch, created)
    monkeypatch.setattr("kinova_teleop.main.SdkXrInput", _FakeSource)
    argv = _expanded_motion_gate_args(
        [
            "--backend", "kortex",
            "--enable-hardware",
            "--input", "xrobotoolkit",
            "--translation-only",
            "--expanded-translation-envelope",
            "--responsive-translation-profile",
            "--invert-translation",
            "--recover-stale-input",
            "--scale", "0.8",
            "--max-linear-speed", "0.01",
        ]
    )
    workspace_min_index = argv.index("--workspace-min")
    argv[workspace_min_index + 1 : workspace_min_index + 4] = [
        "0.646914864",
        "-0.182188472",
        "-0.035961582",
    ]
    workspace_max_index = argv.index("--workspace-max")
    argv[workspace_max_index + 1 : workspace_max_index + 4] = [
        "0.846914864",
        "0.017811528",
        "0.164038418",
    ]

    assert main(argv) == 0
    backend = created["backend"]
    assert backend.kwargs["workspace_limits"].minimum_xyz == (
        0.646914864,
        -0.182188472,
        -0.035961582,
    )
    assert backend.kwargs["workspace_limits"].maximum_xyz == (
        0.846914864,
        0.017811528,
        0.164038418,
    )
    assert backend.kwargs["anchor_envelope"].maximum_translation_axis_m == (
        0.05,
        0.05,
        0.05,
    )


def test_responsive_workspace_overrun_rejects_before_hardware_side_effects(
    monkeypatch,
    capsys,
) -> None:
    """A responsive absolute span over 200 mm must fail before credentials."""

    calls = _install_unreachable_connection_factory(monkeypatch)
    prompts: list[str] = []
    monkeypatch.setattr(
        "kinova_teleop.main.os.getenv",
        lambda _name: (_ for _ in ()).throw(AssertionError("password read")),
    )
    monkeypatch.setattr(
        "kinova_teleop.main.validate_kortex_runtime",
        lambda: (_ for _ in ()).throw(AssertionError("runtime checked")),
    )
    monkeypatch.setattr("builtins.input", lambda prompt: prompts.append(prompt) or "MOVE")
    argv = _expanded_motion_gate_args(
        [
            "--backend", "kortex",
            "--enable-hardware",
            "--input", "xrobotoolkit",
            "--translation-only",
            "--expanded-translation-envelope",
            "--responsive-translation-profile",
            "--invert-translation",
            "--recover-stale-input",
            "--scale", "0.8",
            "--max-linear-speed", "0.01",
        ]
    )
    workspace_max_index = argv.index("--workspace-max")
    argv[workspace_max_index + 1] = "0.200001"
    argv[workspace_max_index + 2] = "0.20"
    argv[workspace_max_index + 3] = "0.20"

    assert main(argv) == 2
    assert calls == []
    assert prompts == []
    assert "workspace span must not exceed 0.2 m per axis" in capsys.readouterr().err


def test_responsive_calibrated_expanded_translation_wires_loaded_rotation_once(
    monkeypatch,
    tmp_path,
) -> None:
    """Changing the selected calibration must change controller-axis mapping."""

    created: dict[str, object] = {}
    _install_valid_kortex_fakes(monkeypatch, created)
    monkeypatch.setattr("kinova_teleop.main.SdkXrInput", _FakeSource)
    artifact = tmp_path / "operator-axis.json"
    expected_rotation = (
        (0.0, -1.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    loaded_paths: list[object] = []
    monkeypatch.setattr(
        main_module,
        "load_operator_axis_calibration",
        lambda path: loaded_paths.append(path) or SimpleNamespace(
            translation_rotation=expected_rotation,
        ),
    )
    argv = _expanded_motion_gate_args(
        [
            "--backend", "kortex",
            "--enable-hardware",
            "--input", "xrobotoolkit",
            "--translation-only",
            "--expanded-translation-envelope",
            "--responsive-translation-profile",
            "--operator-calibration", str(artifact),
            "--recover-stale-input",
            "--scale", "0.8",
            "--max-linear-speed", "0.01",
        ]
    )

    assert main(argv) == 0
    controller_config = created["controller_config"]
    assert controller_config.translation_scale == 0.8
    assert controller_config.orientation_enabled is False
    assert controller_config.recover_stale_input is True
    assert controller_config.invert_translation is False
    assert controller_config.translation_rotation == expected_rotation
    assert loaded_paths == [artifact]


def _forbid_hardware_setup_after_calibration_rejection(monkeypatch) -> list[object]:
    calls = _install_unreachable_connection_factory(monkeypatch)
    monkeypatch.setattr(
        main_module,
        "os",
        SimpleNamespace(
            getenv=lambda _name: (_ for _ in ()).throw(
                AssertionError("password read"),
            ),
        ),
    )
    monkeypatch.setattr(
        main_module,
        "validate_kortex_runtime",
        lambda: (_ for _ in ()).throw(AssertionError("SDK validated")),
    )
    monkeypatch.setattr(
        main_module,
        "create_input",
        lambda _args: (_ for _ in ()).throw(AssertionError("input created")),
    )
    monkeypatch.setattr(
        main_module,
        "_create_kortex_backend",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("backend created"),
        ),
    )
    monkeypatch.setattr(
        "builtins.input",
        lambda _prompt: (_ for _ in ()).throw(AssertionError("MOVE prompted")),
    )
    return calls


@pytest.mark.parametrize(
    ("arguments", "expected_error"),
    [
        (
            ["--operator-calibration", "scope.json"],
            "responsive translation profile requires",
        ),
        (
            [
                "--backend", "kortex",
                "--enable-hardware",
                "--translation-only",
                "--expanded-translation-envelope",
                "--responsive-translation-profile",
                "--operator-calibration", "scope.json",
                "--recover-stale-input",
                "--invert-translation",
            ],
            "exactly one of --invert-translation or --operator-calibration",
        ),
        (
            [
                "--backend", "kortex",
                "--enable-hardware",
                "--translation-only",
                "--expanded-translation-envelope",
                "--responsive-translation-profile",
                "--recover-stale-input",
            ],
            "exactly one of --invert-translation or --operator-calibration",
        ),
    ],
)
def test_invalid_responsive_calibration_scope_rejects_before_hardware_side_effects(
    monkeypatch,
    capsys,
    arguments: list[str],
    expected_error: str,
) -> None:
    """Invalid responsive mode must not inspect credentials or hardware."""

    calls = _forbid_hardware_setup_after_calibration_rejection(monkeypatch)

    assert main(_expanded_motion_gate_args(arguments)) == 2
    assert calls == []
    assert expected_error in capsys.readouterr().err


@pytest.mark.parametrize(
    ("artifact_name", "artifact_contents"),
    [
        ("missing.json", None),
        ("malformed.json", "not JSON"),
    ],
)
def test_unreadable_responsive_calibration_rejects_before_hardware_side_effects(
    monkeypatch,
    capsys,
    tmp_path,
    artifact_name: str,
    artifact_contents: str | None,
) -> None:
    """An unusable calibration artifact cannot fall through to hardware setup."""

    artifact = tmp_path / artifact_name
    if artifact_contents is not None:
        artifact.write_text(artifact_contents, encoding="utf-8")
    _install_valid_motion_gate_files(monkeypatch)
    calls = _forbid_hardware_setup_after_calibration_rejection(monkeypatch)
    argv = _expanded_motion_gate_args(
        [
            "--backend", "kortex",
            "--enable-hardware",
            "--translation-only",
            "--expanded-translation-envelope",
            "--responsive-translation-profile",
            "--operator-calibration", str(artifact),
            "--recover-stale-input",
        ]
    )

    assert main(argv) == 2
    assert calls == []
    assert "error: operator calibration" in capsys.readouterr().err


@pytest.mark.parametrize(
    "arguments",
    [
        ["--responsive-translation-profile", "--invert-translation", "--recover-stale-input"],
        ["--backend", "kortex", "--enable-hardware", "--responsive-translation-profile", "--invert-translation", "--recover-stale-input"],
        ["--backend", "kortex", "--enable-hardware", "--translation-only", "--responsive-translation-profile", "--invert-translation", "--recover-stale-input"],
    ],
)
def test_responsive_profile_rejects_invalid_scope_before_hardware_side_effects(
    monkeypatch, capsys, arguments,
) -> None:
    calls = _install_unreachable_connection_factory(monkeypatch)
    monkeypatch.setattr(
        "kinova_teleop.main.os.getenv",
        lambda _name: (_ for _ in ()).throw(AssertionError("password read")),
    )

    assert main(arguments) == 2
    assert calls == []
    assert "responsive translation profile requires" in capsys.readouterr().err


@pytest.mark.parametrize(
    "limit_arguments",
    (
        ["--scale", "0.800001"],
        ["--max-linear-speed", "0.010001"],
    ),
)
def test_responsive_profile_rejects_over_limit_before_connection(
    monkeypatch, limit_arguments,
) -> None:
    calls = _install_unreachable_connection_factory(monkeypatch)
    monkeypatch.setenv("KINOVA_PASSWORD", "secret")
    argv = _expanded_motion_gate_args(
        [
            "--backend", "kortex",
            "--enable-hardware",
            "--input", "xrobotoolkit",
            "--translation-only",
            "--expanded-translation-envelope",
            "--responsive-translation-profile",
            "--invert-translation",
            "--recover-stale-input",
            *limit_arguments,
        ]
    )

    assert main(argv) == 2
    assert calls == []


def test_translation_only_kortex_accepts_explicit_scale_half(monkeypatch) -> None:
    created: dict[str, object] = {}
    _install_valid_kortex_fakes(monkeypatch, created)
    monkeypatch.setattr("kinova_teleop.main.SdkXrInput", _FakeSource)

    assert main(_motion_gate_args([
        "--backend",
        "kortex",
        "--enable-hardware",
        "--input",
        "xrobotoolkit",
        "--translation-only",
        "--scale",
        "0.5",
    ])) == 0

    controller_config = created["controller_config"]
    assert controller_config.translation_scale == 0.5
    assert controller_config.orientation_enabled is False


def test_kortex_default_input_is_continuously_buffered_pico_udp(monkeypatch) -> None:
    created: dict[str, object] = {}
    pico_kwargs: dict[str, object] = {}
    _install_valid_kortex_fakes(monkeypatch, created)

    class FakePicoInput(_FakeSource):
        def __init__(self, *args, **kwargs) -> None:
            super().__init__()
            pico_kwargs.update(kwargs)

        def read(self) -> ControllerSample:
            return ControllerSample(
                position=np.zeros(3, dtype=float),
                quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                grip=0.0,
                timestamp_ns=1,
                received_monotonic=1.0,
            )

    monkeypatch.setattr("kinova_teleop.main.PicoUdpInput", FakePicoInput)

    assert main(_motion_gate_args(["--backend", "kortex", "--enable-hardware"])) == 0
    assert isinstance(created["source"], main_module.ContinuousInputBuffer)
    assert pico_kwargs == {
        "host": "0.0.0.0",
        "port": 15031,
        "stale_after": 0.2,
    }


def test_responsive_recovery_locks_pico_to_the_admitted_endpoint(monkeypatch) -> None:
    created: dict[str, object] = {}
    pico_kwargs: dict[str, object] = {}
    _install_valid_kortex_fakes(monkeypatch, created)

    class FakePicoInput(_FakeSource):
        def __init__(self, *args, **kwargs) -> None:
            super().__init__()
            pico_kwargs.update(kwargs)

        def read(self) -> ControllerSample:
            return ControllerSample(
                position=np.zeros(3, dtype=float),
                quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                grip=0.0,
                timestamp_ns=1,
                received_monotonic=1.0,
            )

    monkeypatch.setattr("kinova_teleop.main.PicoUdpInput", FakePicoInput)
    argv = _expanded_motion_gate_args(
        [
            "--backend", "kortex",
            "--enable-hardware",
            "--translation-only",
            "--expanded-translation-envelope",
            "--responsive-translation-profile",
            "--invert-translation",
            "--recover-stale-input",
            "--scale", "0.8",
            "--max-linear-speed", "0.01",
        ]
    )

    assert main(argv) == 0
    assert pico_kwargs["allow_stale_source_handoff"] is False


def test_kortex_pico_input_keeps_receiving_during_blocking_startup(
    monkeypatch,
) -> None:
    """A blocking Kortex setup phase must not pause PICO packet ingestion."""

    sampled = threading.Event()
    observed_during_setup: list[bool] = []

    class StreamingPico:
        def __init__(self, *args, **kwargs) -> None:
            self.reads = 0
            self.closed = False

        def read(self) -> ControllerSample:
            self.reads += 1
            if self.reads >= 3:
                sampled.set()
            return ControllerSample(
                position=np.zeros(3, dtype=float),
                quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                grip=0.0,
                timestamp_ns=self.reads,
                received_monotonic=float(self.reads),
            )

        def close(self) -> None:
            self.closed = True

    class Connection:
        def close(self, *, send_stop: bool = True) -> bool:
            return True

    def connect(_config, *, read_only: bool = False):
        if read_only:
            observed_during_setup.append(sampled.wait(0.1))
        return Connection()

    class Backend:
        def close(self) -> None:
            pass

    class Controller:
        steps = 0

        def __init__(self, _config, source, backend, **_kwargs) -> None:
            self.source = source
            self.backend = backend

        def run(self, **_kwargs) -> None:
            pass

        def close(self) -> None:
            self.backend.close()
            self.source.close()

    _install_valid_motion_gate_files(monkeypatch)
    _install_passing_motion_admission(monkeypatch)
    monkeypatch.setattr(main_module, "PicoUdpInput", StreamingPico)
    monkeypatch.setattr(main_module, "_create_kortex_connection", connect)
    monkeypatch.setattr(
        main_module,
        "_create_kortex_backend",
        lambda *_args, **_kwargs: Backend(),
    )
    monkeypatch.setattr(main_module, "TeleopController", Controller)

    assert main(_valid_motion_argv()) == 0
    assert observed_during_setup == [True]


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
        def close(self, **_kwargs) -> bool:
            events.append("connection.close")
            return False

    class ReadonlyConnection:
        def close(self, **_kwargs) -> bool:
            return True

    monkeypatch.setattr("kinova_teleop.main.PicoUdpInput", _FakeSource)
    monkeypatch.setattr(
        "kinova_teleop.main._create_kortex_connection",
        lambda _config, *, read_only=False: ReadonlyConnection() if read_only else FalseConnection(),
    )
    monkeypatch.setattr(
        "kinova_teleop.main._create_kortex_backend",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            RuntimeError("servoing setup failed")
        ),
    )
    monkeypatch.setenv("KINOVA_PASSWORD", "top-secret")
    monkeypatch.setattr("kinova_teleop.main.validate_motion_lease", lambda *_args: object())
    monkeypatch.setattr("kinova_teleop.main.load_passing_preflight_report", lambda *_args: object())
    monkeypatch.setattr("builtins.input", lambda _prompt: "MOVE")
    _install_passing_motion_admission(monkeypatch)

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
        def close(self, **_kwargs) -> bool:
            events.append("connection.close")
            return True

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
        lambda _config, *, read_only=False: (
            SimpleNamespace(close=lambda **_kwargs: True)
            if read_only
            else FakeConnection()
        ),
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
    monkeypatch.setattr("kinova_teleop.main.load_passing_preflight_report", lambda *_args: object())
    monkeypatch.setattr("builtins.input", lambda _prompt: "MOVE")
    _install_passing_motion_admission(monkeypatch)

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
        def close(self, **_kwargs) -> bool:
            events.append("connection.close")
            return True

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
        lambda _config, *, read_only=False: (
            SimpleNamespace(close=lambda **_kwargs: True)
            if read_only
            else FakeConnection()
        ),
    )
    monkeypatch.setattr(
        "kinova_teleop.main._create_kortex_backend",
        lambda *_args, **_kwargs: FakeBackend(),
    )
    monkeypatch.setattr("kinova_teleop.main.PicoUdpInput", EventSource)
    monkeypatch.setattr("kinova_teleop.main.TeleopController", FakeController)
    monkeypatch.setenv("KINOVA_PASSWORD", "secret")
    monkeypatch.setattr("kinova_teleop.main.validate_motion_lease", lambda *_args: object())
    monkeypatch.setattr("kinova_teleop.main.load_passing_preflight_report", lambda *_args: object())
    monkeypatch.setattr("builtins.input", lambda _prompt: "MOVE")
    _install_passing_motion_admission(monkeypatch)

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
