from __future__ import annotations

import json
from pathlib import Path
import stat
from types import SimpleNamespace

import numpy as np
import pytest

from kinova_teleop.preflight import (
    PreflightContext,
    load_passing_preflight_report,
    require_live_kortex_ready,
    run_input_preflight,
    run_kortex_readonly_preflight,
)
from kinova_teleop.xr_input import ControllerSample


class RecordingConnection:
    def __init__(
        self,
        arm_state: int = 31,
        *,
        model: str = "MODEL_ID_L53",
        degree_of_freedom: int = 7,
        firmware_raw: int = 0x05020800,
        operating_mode: str = "RUN_MODE",
        servoing_mode: str = "SINGLE_LEVEL_SERVOING",
    ) -> None:
        self.calls: list[str] = []
        self.base_pb2 = SimpleNamespace(
            ARMSTATE_SERVOING_READY=31,
            ARMSTATE_IN_FAULT=32,
            RUN_MODE=41,
            SINGLE_LEVEL_SERVOING=23,
        )
        self.base = SimpleNamespace(
            GetArmState=self._get_arm_state,
            GetProductConfiguration=self._get_product_configuration,
            GetOperatingMode=self._get_operating_mode,
            GetServoingMode=self._get_servoing_mode,
        )
        self.base_cyclic = SimpleNamespace(RefreshFeedback=self._refresh_feedback)
        self.device_config = SimpleNamespace(
            GetFirmwareVersion=self._get_firmware_version,
        )
        self.arm_state = arm_state
        self.model = model
        self.degree_of_freedom = degree_of_freedom
        self.firmware_raw = firmware_raw
        self.operating_mode = operating_mode
        self.servoing_mode = servoing_mode

    def rpc_options(self) -> SimpleNamespace:
        return SimpleNamespace(timeout_ms=100)

    def readonly_rpc_options(self) -> SimpleNamespace:
        return SimpleNamespace(timeout_ms=5_000)

    def _get_arm_state(self, *, options=None):
        self.calls.append("GetArmState")
        return SimpleNamespace(active_state=self.arm_state)

    def _refresh_feedback(self, *, options=None):
        self.calls.append("RefreshFeedback")
        return SimpleNamespace(base=SimpleNamespace(
            tool_pose_x=0.0,
            tool_pose_y=0.0,
            tool_pose_z=0.3,
            tool_pose_theta_x=0.0,
            tool_pose_theta_y=0.0,
            tool_pose_theta_z=0.0,
        ))

    def _get_product_configuration(self, *, options=None):
        self.calls.append("GetProductConfiguration")
        return SimpleNamespace(
            model=self.model,
            degree_of_freedom=self.degree_of_freedom,
            serial_number="PRIVATE-SERIAL-MUST-NOT-LEAK",
        )

    def _get_operating_mode(self, *, options=None):
        self.calls.append("GetOperatingMode")
        return SimpleNamespace(operating_mode=self.operating_mode)

    def _get_servoing_mode(self, *, options=None):
        self.calls.append("GetServoingMode")
        return SimpleNamespace(servoing_mode=self.servoing_mode)

    def _get_firmware_version(self, *, options=None):
        self.calls.append("GetFirmwareVersion")
        return SimpleNamespace(firmware_version=self.firmware_raw)


def preflight_context() -> PreflightContext:
    return PreflightContext(
        code_revision="ea389a5",
        dirty_worktree=False,
        runtime={"python": "3.13"},
        driver={"name": "kortex", "version": "2.6.0"},
        firmware={"version": "6.5.0"},
        transport={"kind": "tcp"},
        calibration=({"name": "gen3-arm", "sha256": "a" * 64},),
        safety_limits={"workspace": "explicit", "max_linear_speed": 0.01},
        physical_checks={
            "workspace_clear": True,
            "physical_estop_reachable": True,
            "teach_pendant_stop_reachable": True,
            "second_observer_present": True,
            "cable_slack_checked": True,
            "device_fixture_checked": True,
            "speed_level_checked": True,
            "workspace_bounds_checked": True,
            "load_tcp_checked": True,
        },
        timestamp_utc="2026-08-03T00:00:00Z",
    )


def test_readonly_preflight_reads_current_state_without_clear_or_servo_mode():
    connection = RecordingConnection()
    report = run_kortex_readonly_preflight(connection, preflight_context())
    assert report.passed is True
    assert connection.calls == [
        "GetArmState",
        "GetProductConfiguration",
        "GetOperatingMode",
        "GetServoingMode",
        "GetFirmwareVersion",
        "RefreshFeedback",
    ]


def test_readonly_preflight_requires_l53_7dof_running_single_level() -> None:
    connection = RecordingConnection(
        model="MODEL_ID_L53",
        degree_of_freedom=7,
        firmware_raw=0x05020800,
        operating_mode="RUN_MODE",
        servoing_mode="SINGLE_LEVEL_SERVOING",
    )

    report = run_kortex_readonly_preflight(connection, preflight_context())

    statuses = {check.name: check.status for check in report.checks}
    assert statuses["product_model"] == "pass"
    assert statuses["degree_of_freedom"] == "pass"
    assert statuses["firmware_version"] == "pass"
    assert statuses["operating_mode"] == "pass"
    assert statuses["servoing_mode"] == "pass"
    require_live_kortex_ready(report)


def test_readonly_preflight_uses_device_specific_enum_modules() -> None:
    """Base enums must not override the DeviceConfig/ProductConfiguration IDs."""

    connection = RecordingConnection(model=1, operating_mode=6)
    connection.base_pb2 = SimpleNamespace(
        ARMSTATE_SERVOING_READY=31,
        RUN_MODE=6,
        SINGLE_LEVEL_SERVOING=23,
    )
    connection.product_configuration_pb2 = SimpleNamespace(MODEL_ID_L53=1)

    report = run_kortex_readonly_preflight(connection, preflight_context())

    statuses = {check.name: check.status for check in report.checks}
    assert statuses["product_model"] == "pass"
    assert statuses["operating_mode"] == "pass"


def test_preflight_report_omits_unique_device_identifiers() -> None:
    mapping = run_kortex_readonly_preflight(
        RecordingConnection(), preflight_context()
    ).to_mapping()
    serialized = json.dumps(mapping).lower()
    assert "serial" not in serialized
    assert "mac" not in serialized


def test_report_mapping_has_central_preflight_fields():
    report = run_kortex_readonly_preflight(RecordingConnection(), preflight_context())
    assert set(report.to_mapping()) == {
        "schema_version", "device", "timestamp_utc", "code_revision", "dirty_worktree",
        "runtime", "driver", "firmware", "transport", "calibration", "safety_limits",
        "checks", "physical_checks", "passed",
    }


def test_passing_preflight_report_is_strict_and_read_only(tmp_path):
    payload = run_kortex_readonly_preflight(RecordingConnection(), preflight_context()).to_mapping()
    payload["passed"] = True
    report_path = tmp_path / "preflight.json"
    report_path.write_text(json.dumps(payload), encoding="utf-8")

    loaded = load_passing_preflight_report(report_path)
    assert loaded["device"] == "gen3"
    assert loaded["passed"] is True

    payload["passed"] = False
    report_path.write_text(json.dumps(payload), encoding="utf-8")
    import pytest

    with pytest.raises(ValueError, match="not passed"):
        load_passing_preflight_report(report_path)


def test_passing_preflight_report_rejects_windows_reparse_file(
    tmp_path,
    monkeypatch,
):
    payload = run_kortex_readonly_preflight(
        RecordingConnection(), preflight_context()
    ).to_mapping()
    payload["passed"] = True
    path = tmp_path / "preflight.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    monkeypatch.setattr(
        Path,
        "lstat",
        lambda self: SimpleNamespace(
            st_mode=stat.S_IFREG,
            st_file_attributes=stat.FILE_ATTRIBUTE_REPARSE_POINT,
        ),
    )

    with pytest.raises(ValueError, match="symlink or reparse point"):
        load_passing_preflight_report(path)


def test_passing_preflight_report_rejects_unknown_physical_check(tmp_path):
    payload = run_kortex_readonly_preflight(RecordingConnection(), preflight_context()).to_mapping()
    payload["passed"] = True
    payload["physical_checks"]["workspace_clear"] = False
    path = tmp_path / "preflight.json"
    path.write_text(json.dumps(payload), encoding="utf-8")

    import pytest

    with pytest.raises(ValueError, match="physical checklist"):
        load_passing_preflight_report(path)


@pytest.mark.parametrize(
    "required_check",
    (
        "feedback_pose",
        "product_model",
        "degree_of_freedom",
        "firmware_version",
        "operating_mode",
        "servoing_mode",
    ),
)
def test_passing_preflight_report_requires_complete_kortex_checks(tmp_path, required_check):
    payload = run_kortex_readonly_preflight(RecordingConnection(), preflight_context()).to_mapping()
    payload["passed"] = True
    payload["checks"] = [
        check for check in payload["checks"] if check["name"] != required_check
    ]
    path = tmp_path / "missing-check.json"
    path.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValueError, match="required software checks"):
        load_passing_preflight_report(path)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("driver", {"version": "read-only-unverified"}, "driver.*placeholder"),
        ("firmware", {"version": "read-only-unverified"}, "firmware.*placeholder"),
        (
            "calibration",
            [{"name": "reported-by-kortex", "sha256": "read-only-unverified"}],
            "calibration.*placeholder",
        ),
    ],
)
def test_passing_preflight_report_rejects_placeholder_evidence(
    tmp_path, field, value, message
):
    payload = run_kortex_readonly_preflight(RecordingConnection(), preflight_context()).to_mapping()
    payload["passed"] = True
    payload[field] = value
    path = tmp_path / f"placeholder-{field}.json"
    path.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValueError, match=message):
        load_passing_preflight_report(path)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("extra", "unexpected fields"),
        ("schema", "schema_version"),
        ("device", "device"),
        ("timestamp", "timestamp_utc"),
        ("dirty", "worktree"),
        ("runtime", "runtime"),
        ("calibration", "calibration"),
        ("workspace", "workspace limits"),
        ("checks", "checks"),
        ("check_shape", "check shape"),
        ("check_status", "non-passing check"),
        ("physical_shape", "physical checklist"),
        ("physical_value", "physical checklist"),
    ],
)
def test_passing_preflight_report_rejects_invalid_schema_fields(tmp_path, mutation, message):
    payload = run_kortex_readonly_preflight(RecordingConnection(), preflight_context()).to_mapping()
    payload["passed"] = True
    if mutation == "extra":
        payload["extra"] = 1
    elif mutation == "schema":
        payload["schema_version"] = "2.0"
    elif mutation == "device":
        payload["device"] = "ur5e"
    elif mutation == "timestamp":
        payload["timestamp_utc"] = ""
    elif mutation == "dirty":
        payload["dirty_worktree"] = True
    elif mutation == "runtime":
        payload["runtime"] = {}
    elif mutation == "calibration":
        payload["calibration"] = []
    elif mutation == "workspace":
        payload["safety_limits"] = {"max_speed": 0.01}
    elif mutation == "checks":
        payload["checks"] = []
    elif mutation == "check_shape":
        payload["checks"] = [{"name": "arm_state"}]
    elif mutation == "check_status":
        payload["checks"][0]["status"] = "fail"
    elif mutation == "physical_shape":
        payload["physical_checks"].pop("load_tcp_checked")
    elif mutation == "physical_value":
        payload["physical_checks"]["load_tcp_checked"] = False
    path = tmp_path / f"{mutation}.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match=message):
        load_passing_preflight_report(path)


class FakeInput:
    def __init__(self):
        self.step = 0

    def read(self):
        self.step += 1
        return ControllerSample(
            position=np.zeros(3),
            quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            grip=0.0,
            timestamp_ns=self.step,
            received_monotonic=float(self.step),
            valid=True,
        )

    def close(self):
        return None


def test_input_preflight_requires_fresh_releasable_samples():
    report = run_input_preflight(FakeInput(), preflight_context(), samples=3)
    assert report.passed


def test_preflight_fails_closed_for_unknown_context_and_bad_rpc():
    context = PreflightContext()

    class BadConnection:
        base = SimpleNamespace(GetArmState=lambda **_: (_ for _ in ()).throw(RuntimeError("bad")))
        base_cyclic = SimpleNamespace(RefreshFeedback=lambda **_: None)
        base_pb2 = SimpleNamespace(ARMSTATE_SERVOING_READY=31)

        def rpc_options(self):
            return SimpleNamespace(timeout_ms=100)

    report = run_kortex_readonly_preflight(BadConnection(), context)
    assert report.passed is False
    assert any(check.status in ("unknown", "fail") for check in report.checks)


def test_input_preflight_rejects_invalid_samples_and_missing_release():
    class BadInput:
        def __init__(self):
            self.count = 0

        def read(self):
            self.count += 1
            return ControllerSample(
                position=np.array([float("nan"), 0.0, 0.0]),
                quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                grip=1.0,
                timestamp_ns=1,
                received_monotonic=0.0,
                valid=False,
            )

        def close(self):
            return None

    report = run_input_preflight(BadInput(), preflight_context(), samples=1)
    assert report.passed is False
