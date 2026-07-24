from pathlib import Path

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
