from pathlib import Path

import numpy as np

from kinova_teleop.main import build_parser, main
from kinova_teleop.xr_input import ControllerSample


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
    assert main(["--input", "xrobotoolkit", "--check-xr", "--samples", "1"]) == 2
    assert "xrobotoolkit_sdk" in capsys.readouterr().err


def test_headless_requires_positive_steps(capsys) -> None:
    assert main(["--dry-run", "--headless"]) == 2
    assert "--steps" in capsys.readouterr().err


def test_only_left_controller_is_accepted() -> None:
    parser = build_parser()
    assert parser.parse_args(["--controller", "left"]).controller == "left"


def test_operator_calibration_parser_preserves_artifact_path() -> None:
    parser = build_parser()

    assert parser.parse_args(
        ["--operator-calibration", "operator-axis.json"]
    ).operator_calibration == Path("operator-axis.json")


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

    assert main(["--input", "xrobotoolkit"]) == 2
    assert source.closed


def test_pico_udp_is_default_native_input() -> None:
    args = build_parser().parse_args([])
    assert args.input == "pico-udp"
    assert args.pico_host == "0.0.0.0"
    assert args.pico_port == 15031
    assert args.scale == 0.5


def test_create_input_selects_pico_udp(monkeypatch) -> None:
    created = []
    monkeypatch.setattr(
        "kinova_teleop.main.PicoUdpInput",
        lambda **kwargs: created.append(kwargs) or object(),
    )
    args = build_parser().parse_args(["--input", "pico-udp"])
    from kinova_teleop.main import create_input

    create_input(args)
    assert created == [{"host": "0.0.0.0", "port": 15031, "stale_after": 0.2}]


def test_dry_run_never_constructs_network_input(monkeypatch) -> None:
    monkeypatch.setattr(
        "kinova_teleop.main.PicoUdpInput",
        lambda **_kwargs: (_ for _ in ()).throw(AssertionError("network")),
    )
    assert main(["--dry-run", "--headless", "--steps", "20"]) == 0


class ScriptedSource:
    def __init__(self, samples: list[ControllerSample]) -> None:
        self._samples = iter(samples)
        self._last: ControllerSample | None = None
        self.closed = False

    def read(self) -> ControllerSample:
        try:
            self._last = next(self._samples)
        except StopIteration:
            assert self._last is not None
        return self._last

    def close(self) -> None:
        self.closed = True


def _sample(timestamp_ns: int, grip: float, *, valid: bool = True) -> ControllerSample:
    return ControllerSample(
        position=np.array([0.1, 0.2, 0.3]),
        quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        grip=grip,
        timestamp_ns=timestamp_ns,
        received_monotonic=float(timestamp_ns) / 1_000_000_000,
        valid=valid,
    )


def test_check_input_counts_distinct_valid_samples_and_requires_release(
    monkeypatch,
    capsys,
) -> None:
    source = ScriptedSource(
        [
            _sample(1, 0.0, valid=False),
            _sample(2, 1.0),
            _sample(2, 0.0),
            *[_sample(timestamp, 0.5 if timestamp == 10 else 1.0) for timestamp in range(3, 22)],
        ],
    )
    monkeypatch.setattr("kinova_teleop.main.create_input", lambda _args: source)

    assert main(["--check-input", "--samples", "20"]) == 0

    output = capsys.readouterr().out
    assert output.count("left position=") == 20
    assert "position=[+0.10000 +0.20000 +0.30000]" in output
    assert "quat_xyzw=[+0.00000 +0.00000 +0.00000 +1.00000]" in output
    assert "grip=0.500" in output
    assert source.closed


def test_check_input_reports_timeout_for_missing_valid_samples(monkeypatch, capsys) -> None:
    source = ScriptedSource([_sample(1, 0.0, valid=False)])
    monkeypatch.setattr("kinova_teleop.main.create_input", lambda _args: source)

    assert main(["--check-input", "--samples", "1", "--check-timeout", "0.001"]) == 2

    assert "timed out waiting for 1 distinct valid input samples" in capsys.readouterr().err
    assert source.closed


def test_check_input_requires_a_finite_sample_count(monkeypatch, capsys) -> None:
    monkeypatch.setattr(
        "kinova_teleop.main.create_input",
        lambda _args: (_ for _ in ()).throw(AssertionError("source constructed")),
    )

    assert main(["--check-input"]) == 2

    assert "--check-input requires a positive --samples" in capsys.readouterr().err


def test_check_input_timeout_guidance_is_source_neutral(monkeypatch, capsys) -> None:
    source = ScriptedSource([_sample(1, 0.0, valid=False)])
    monkeypatch.setattr("kinova_teleop.main.create_input", lambda _args: source)

    assert (
        main(
            [
                "--input",
                "xrobotoolkit",
                "--check-input",
                "--samples",
                "1",
                "--check-timeout",
                "0.001",
            ],
        )
        == 2
    )

    error = capsys.readouterr().err
    assert "verify the selected input source" in error
    assert "Unity" not in error
    assert "PICO" not in error


def test_check_input_reports_missing_release(monkeypatch, capsys) -> None:
    source = ScriptedSource([_sample(timestamp, 1.0) for timestamp in range(1, 21)])
    monkeypatch.setattr("kinova_teleop.main.create_input", lambda _args: source)

    assert main(["--check-input", "--samples", "20"]) == 2

    assert "release the Grip below 0.8" in capsys.readouterr().err
    assert source.closed


def test_check_xr_alias_uses_selected_input(monkeypatch, capsys) -> None:
    source = ScriptedSource([_sample(1, 0.0)])
    monkeypatch.setattr("kinova_teleop.main.create_input", lambda _args: source)

    assert main(["--input", "pico-udp", "--check-xr", "--samples", "1"]) == 0

    assert "left position=" in capsys.readouterr().out
    assert source.closed
