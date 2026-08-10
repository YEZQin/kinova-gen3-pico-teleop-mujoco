# Gen3 PICO Kortex Hardware Completion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete the existing PICO-to-Kortex integration so the left controller can operate the Gen3 Ultra arm through a fail-closed, first-hardware 6DoF teleoperation profile.

**Architecture:** Preserve the existing PICO UDP, release-first relative pose mapper, MuJoCo backend, high-level Kortex twist backend, motion lease, preflight report, watchdog, and Stop machinery. Add an exact first-hardware profile, current-session input/robot admission before `MOVE`, an anchor-relative translation/orientation envelope, and explicit fatal-fault propagation; remove every reachable gripper write path from this phase.

**Tech Stack:** Windows PowerShell, Python 3.11, NumPy, MuJoCo, Kinova Kortex API `2.8.0.post5`, protobuf `3.20.0`, pytest, pytest-cov.

## Global Constraints

- Default backend remains MuJoCo; hardware requires both `--backend kortex` and `--enable-hardware`.
- First-hardware Kortex defaults and upper bounds are exactly: 40 Hz, translation scale 0.25, stale timeout 0.2 s, linear speed 0.005 m/s, angular speed 2 deg/s.
- The anchor-relative envelope is ±0.02 m on each translation axis and 5 deg total shortest-arc orientation error.
- The explicit absolute XYZ workspace must contain the live anchor and span no more than 0.04 m on each axis during the first-hardware profile.
- Kortex uses trusted private IPv4 and high-level TCP port 10000 with Single Level Servoing; no low-level cyclic command path is added.
- `KINOVA_PASSWORD` is the only password source and must never be printed, persisted, included on a command line, or copied into evidence.
- No automatic Home, Action, `ClearFaults`, automatic reconnect, watchdog bypass, visual-servo command, or gripper write is permitted.
- Fresh finite PICO samples, a currently released Grip, a locked input source, current read-only robot checks, a reviewed passing preflight report, a valid motion lease, the physical checklist, and exact `MOVE` must all precede creation of the motion backend.
- First Grip press only anchors the current controller pose to the current measured end-effector pose; that cycle sends no nonzero twist.
- Normal Grip release requests Stop and permits a healthy re-clutch. Invalid/stale/source-changed input, RPC/feedback failure, workspace/envelope violation, watchdog timeout, or unconfirmed Stop latches a fault and exits nonzero without reconnecting.
- The 0.2 s watchdog is a host Stop-request bound, not a physical stopping guarantee.
- Existing user files, untracked trial artifacts, and the dirty `codex/kinova-only-joint-speed` worktree are preserved.
- Production changes are test-first. The full suite must remain green and `kinova_teleop` line coverage must be at least 80%.
- Before every task commit, inspect the focused diff, run `git diff --cached --check`, scan the staged patch for credentials/unique identifiers, and resolve every correctness or security finding for that task.
- No real robot connection or motion is part of Tasks 1-8; onsite execution begins only at the post-implementation checkpoints.

---

## File responsibility map

- Create `kinova_teleop/hardware_profile.py`: immutable first-hardware constants, private-IPv4 validation, and exact local Kortex/Python/protobuf compatibility checks.
- Create `kinova_teleop/hardware_admission.py`: current-session PICO admission, released-Grip recheck, live read-only report admission, and exact `MOVE` confirmation.
- Modify `kinova_teleop/main.py`: resolve backend-specific defaults and enforce startup ordering without growing new safety algorithms in the CLI module.
- Modify `kinova_teleop/kortex_transport.py`: expose read-only/session RPC deadlines, DeviceConfig client construction, and read-only cleanup without a Stop write.
- Modify `kinova_teleop/preflight.py`: collect current product, firmware, operating mode, servoing mode, arm state, and finite feedback without unique device identifiers.
- Modify `kinova_teleop/workspace.py`: validate both absolute XYZ bounds and an immutable anchor-relative translation/orientation envelope.
- Modify `kinova_teleop/kortex_backend.py`: capture the re-clutch anchor, enforce both envelopes before twist, latch watchdog/workspace/Stop faults, and remove gripper writes.
- Modify `kinova_teleop/xr_input.py`, `kinova_teleop/pico_udp_input.py`, and `kinova_teleop/pose_mapping.py`: carry a stable invalid-input reason through the existing input and mapping boundary.
- Modify `kinova_teleop/teleop_controller.py`: make normal release recoverable while fatal hardware input/backend faults stop and escape the run loop.
- Create `scripts/start_kortex_pico_teleop.ps1`: a parameterized launcher that always supplies the first-hardware limits and never accepts a password parameter.
- Modify `configs/gen3_micro_axes.json` and `docs/kortex-hardware-quickstart.md`: align non-teleop trial limits and operator guidance with the approved profile.
- Modify `pyproject.toml`: add `pytest-cov` to the development test toolchain.

### Task 1: Exact first-hardware profile and runtime compatibility gate

**Files:**
- Create: `kinova_teleop/hardware_profile.py`
- Create: `tests/test_hardware_profile.py`
- Modify: `kinova_teleop/main.py:35-45,64-219,222-227,306-425`
- Modify: `tests/test_cli_kortex.py:130-340,470-510`

**Interfaces:**
- Produces `HardwareProfile`, `FIRST_HARDWARE_PROFILE`, `KortexRuntimeVersions`, `validate_kortex_runtime()`, `validate_private_robot_ipv4()`, and `validate_workspace_span()`.
- `main.resolve_control_hz(args)`, new `main.resolve_translation_scale(args)`, and Kortex CLI validation consume `FIRST_HARDWARE_PROFILE`.
- Later tasks rely on `FIRST_HARDWARE_PROFILE.anchor_translation_axis_m` and `anchor_rotation_deg`.

- [ ] **Step 1: Write failing profile and CLI-default tests**

```python
import pytest

from kinova_teleop.hardware_profile import (
    FIRST_HARDWARE_PROFILE,
    validate_kortex_runtime,
    validate_private_robot_ipv4,
    validate_workspace_span,
)
from kinova_teleop.main import build_parser, resolve_control_hz, resolve_translation_scale
from kinova_teleop.workspace import WorkspaceLimits


def test_first_hardware_profile_is_the_approved_envelope() -> None:
    profile = FIRST_HARDWARE_PROFILE
    assert profile.control_hz == 40.0
    assert profile.translation_scale == 0.25
    assert profile.stale_timeout_s == 0.2
    assert profile.max_linear_speed_mps == 0.005
    assert profile.max_angular_speed_deg_s == 2.0
    assert profile.anchor_translation_axis_m == (0.02, 0.02, 0.02)
    assert profile.anchor_rotation_deg == 5.0


def test_backend_specific_defaults_preserve_mujoco() -> None:
    parser = build_parser()
    mujoco = parser.parse_args([])
    kortex = parser.parse_args(["--backend", "kortex"])
    assert resolve_control_hz(mujoco) == 100.0
    assert resolve_translation_scale(mujoco) == 0.5
    assert resolve_control_hz(kortex) == 40.0
    assert resolve_translation_scale(kortex) == 0.25


def test_runtime_gate_requires_the_tested_sdk_stack() -> None:
    versions = {"kortex-api": "2.8.0.post5", "protobuf": "3.20.0"}
    observed = validate_kortex_runtime(
        python_version=(3, 11, 15),
        distribution_version=versions.__getitem__,
    )
    assert observed.kortex_api == "2.8.0.post5"
    with pytest.raises(RuntimeError, match="protobuf 3.20.0"):
        validate_kortex_runtime(
            python_version=(3, 11, 15),
            distribution_version=lambda name: "5.0.0" if name == "protobuf" else "2.8.0.post5",
        )


@pytest.mark.parametrize("host", ["8.8.8.8", "127.0.0.1", "::1", "robot.local"])
def test_robot_endpoint_requires_private_non_loopback_ipv4(host: str) -> None:
    with pytest.raises(ValueError, match="private IPv4"):
        validate_private_robot_ipv4(host)


def test_first_hardware_workspace_cannot_exceed_four_centimetres_per_axis() -> None:
    limits = WorkspaceLimits(
        (0.10, -0.20, 0.30),
        (0.140001, -0.16, 0.34),
    )
    with pytest.raises(ValueError, match="span must not exceed 0.04 m"):
        validate_workspace_span(limits, FIRST_HARDWARE_PROFILE)
```

- [ ] **Step 2: Run the focused tests and verify RED**

```powershell
$python = 'D:\yezq\26.7.24_Kinova_controll\.venv\Scripts\python.exe'
& $python -m pytest tests\test_hardware_profile.py tests\test_cli_kortex.py -q
```

Expected: collection fails because `kinova_teleop.hardware_profile` and `resolve_translation_scale` do not exist.

- [ ] **Step 3: Implement the immutable profile and pure validators**

```python
@dataclass(frozen=True)
class HardwareProfile:
    control_hz: float
    translation_scale: float
    stale_timeout_s: float
    max_linear_speed_mps: float
    max_angular_speed_deg_s: float
    anchor_translation_axis_m: tuple[float, float, float]
    anchor_rotation_deg: float


FIRST_HARDWARE_PROFILE = HardwareProfile(
    control_hz=40.0,
    translation_scale=0.25,
    stale_timeout_s=0.2,
    max_linear_speed_mps=0.005,
    max_angular_speed_deg_s=2.0,
    anchor_translation_axis_m=(0.02, 0.02, 0.02),
    anchor_rotation_deg=5.0,
)


@dataclass(frozen=True)
class KortexRuntimeVersions:
    python: str
    kortex_api: str
    protobuf: str


def validate_kortex_runtime(
    *,
    python_version: tuple[int, int, int] | None = None,
    distribution_version: Callable[[str], str] = importlib.metadata.version,
) -> KortexRuntimeVersions:
    observed = python_version or sys.version_info[:3]
    if observed[:2] != (3, 11):
        raise RuntimeError("first-hardware Kortex runtime requires Python 3.11.x")
    kortex = distribution_version("kortex-api")
    protobuf = distribution_version("protobuf")
    if kortex != "2.8.0.post5":
        raise RuntimeError("first-hardware runtime requires kortex-api 2.8.0.post5")
    if protobuf != "3.20.0":
        raise RuntimeError("first-hardware runtime requires protobuf 3.20.0")
    return KortexRuntimeVersions(".".join(map(str, observed)), kortex, protobuf)


def validate_workspace_span(
    limits: WorkspaceLimits,
    profile: HardwareProfile = FIRST_HARDWARE_PROFILE,
) -> None:
    span = np.asarray(limits.maximum_xyz) - np.asarray(limits.minimum_xyz)
    maximum = 2.0 * np.asarray(profile.anchor_translation_axis_m)
    if np.any(span > maximum):
        raise ValueError("first-hardware workspace span must not exceed 0.04 m per axis")
```

Implement `validate_private_robot_ipv4()` with `ipaddress.ip_address()`, requiring IPv4, `is_private`, and neither loopback nor unspecified. Change backend-dependent parser defaults to `None`, resolve them after parsing, and reject any Kortex override above the profile values before password lookup or SDK import. During Kortex motion validation, reject an absolute workspace whose per-axis span exceeds `2 * anchor_translation_axis_m`, before password lookup, prompt, import, or connection; the live-anchor containment check remains in the backend after the anchor is measured.

- [ ] **Step 4: Run focused and MuJoCo regression tests**

```powershell
& $python -m pytest tests\test_hardware_profile.py tests\test_cli.py tests\test_cli_kortex.py -q
& $python -m kinova_teleop.main --dry-run --headless --steps 500
```

Expected: all tests pass and the dry run ends with `finite_state=true`.

- [ ] **Step 5: Commit**

```powershell
git add kinova_teleop\hardware_profile.py kinova_teleop\main.py tests\test_hardware_profile.py tests\test_cli_kortex.py
git commit -m "feat: enforce Gen3 first-hardware profile"
```

### Task 2: Current-session read-only Kortex observation

**Files:**
- Modify: `kinova_teleop/kortex_transport.py:7-187`
- Modify: `kinova_teleop/preflight.py:252-291,381-532`
- Modify: `kinova_teleop/main.py:454-526`
- Modify: `tests/test_kortex_transport.py`
- Modify: `tests/test_preflight.py`
- Modify: `tests/test_cli_kortex.py`

**Interfaces:**
- `KortexFactories` adds `device_config_client: Callable[[Any], Any]`.
- `KortexConnection` adds `device_config`, `session_rpc_options()`, `readonly_rpc_options()`, and `close(*, send_stop: bool = True) -> bool`.
- `run_kortex_readonly_preflight(connection, context)` adds checks named `product_model`, `degree_of_freedom`, `firmware_version`, `operating_mode`, and `servoing_mode`.
- Produces `require_live_kortex_ready(report: PreflightReport) -> None` for Task 7.

- [ ] **Step 1: Write failing read-only observation tests**

```python
def test_readonly_preflight_requires_l53_7dof_running_single_level() -> None:
    connection = ready_connection(
        model="MODEL_ID_L53",
        degree_of_freedom=7,
        firmware_raw=0x05020800,
        operating_mode="RUN_MODE",
        servoing_mode="SINGLE_LEVEL_SERVOING",
    )
    report = run_kortex_readonly_preflight(connection, complete_context())
    statuses = {check.name: check.status for check in report.checks}
    assert statuses["product_model"] == "pass"
    assert statuses["degree_of_freedom"] == "pass"
    assert statuses["firmware_version"] == "pass"
    assert statuses["operating_mode"] == "pass"
    assert statuses["servoing_mode"] == "pass"
    require_live_kortex_ready(report)


def test_readonly_cleanup_never_sends_stop_or_motion() -> None:
    connection, events = connected_fake_connection()
    assert connection.close(send_stop=False)
    assert "base.Stop" not in events
    assert not any(name in events for name in ("SendTwistCommand", "ExecuteAction", "SendGripperCommand"))
    assert events[-3:] == ["session.CloseSession", "router.disable", "transport.disconnect"]


def test_preflight_report_omits_unique_device_identifiers() -> None:
    mapping = run_kortex_readonly_preflight(ready_connection(), complete_context()).to_mapping()
    serialized = json.dumps(mapping).lower()
    assert "serial" not in serialized
    assert "mac" not in serialized
```

- [ ] **Step 2: Run the focused tests and verify RED**

```powershell
& $python -m pytest tests\test_kortex_transport.py tests\test_preflight.py tests\test_cli_kortex.py -q
```

Expected: failures show missing read-only RPC options, missing `send_stop`, and missing robot-state checks.

- [ ] **Step 3: Extend the lazy SDK factories and read-only lifecycle**

```python
SESSION_RPC_TIMEOUT_MS = 5_000
READONLY_RPC_TIMEOUT_MS = 5_000


def session_rpc_options(self) -> Any:
    options = self.factories.create_send_options()
    options.timeout_ms = SESSION_RPC_TIMEOUT_MS
    return options


def readonly_rpc_options(self) -> Any:
    options = self.factories.create_send_options()
    options.timeout_ms = READONLY_RPC_TIMEOUT_MS
    return options


def close(self, *, send_stop: bool = True) -> bool:
    if self._closed:
        return self.stop_confirmed
    self._closed = True
    return self._cleanup(stop=send_stop)
```

Import `DeviceConfigClient` only inside `_sdk_factories()`, construct it after the session, and use the long session timeout for `CreateSession`/`CloseSession`. Keep motion and terminal Stop RPC options at 100 ms.

- [ ] **Step 4: Add exact current-state checks**

```python
def decode_firmware_version(raw: int) -> str:
    build = (raw >> 24) & 0xFF
    major = (raw >> 16) & 0xFF
    minor = (raw >> 8) & 0xFF
    patch = raw & 0xFF
    return f"{major}.{minor}.{patch}-{build}"


def require_live_kortex_ready(report: PreflightReport) -> None:
    required = {
        "arm_state",
        "feedback_pose",
        "product_model",
        "degree_of_freedom",
        "firmware_version",
        "operating_mode",
        "servoing_mode",
    }
    statuses = {check.name: check.status for check in report.checks}
    if any(statuses.get(name) != "pass" for name in required):
        raise ValueError("current Kortex read-only preflight is not motion-ready")
```

Call only `GetArmState`, `GetProductConfiguration`, `GetOperatingMode`, `GetServoingMode`, `GetFirmwareVersion`, and `RefreshFeedback`. Compare against `MODEL_ID_L53`, 7 DoF, firmware `2.8.0-5`, `RUN_MODE`, `SINGLE_LEVEL_SERVOING`, and `ARMSTATE_SERVOING_READY`. Sanitize exception output to the exception class name and omit serial/MAC fields.

Extend `_GEN3_KORTEX_REQUIRED_CHECKS` with the five new product/runtime-mode check names. Add a strict-loader test that removes each new check in turn and verifies the separately reviewed motion report is rejected; historical reports without current product/mode evidence cannot authorize a new motion session.

- [ ] **Step 5: Run the focused tests**

```powershell
& $python -m pytest tests\test_kortex_transport.py tests\test_preflight.py tests\test_cli_kortex.py -q
```

Expected: all focused tests pass and fake read-only cleanup contains no write RPC.

- [ ] **Step 6: Commit**

```powershell
git add kinova_teleop\kortex_transport.py kinova_teleop\preflight.py kinova_teleop\main.py tests\test_kortex_transport.py tests\test_preflight.py tests\test_cli_kortex.py
git commit -m "feat: verify current Gen3 state before motion"
```

### Task 3: Fresh PICO and released-Grip admission

**Files:**
- Create: `kinova_teleop/hardware_admission.py`
- Create: `tests/test_hardware_admission.py`
- Modify: `kinova_teleop/pico_udp_input.py:17-220`
- Modify: `tests/test_pico_udp_input.py`

**Interfaces:**
- Produces `InputAdmissionError`, `InputAdmissionResult`, `wait_for_fresh_released_input()`, `verify_released_now()`, and `confirm_move()`.
- Consumes `XrInputSource`; when `health()` exists, requires a stable non-null `active_source` and unchanged `foreign` count.
- Task 7 reuses the same open `PicoUdpInput` instance after admission; admission never closes it.

- [ ] **Step 1: Write failing admission tests**

```python
def test_admission_requires_advancing_finite_samples_and_current_release() -> None:
    source = ScriptedSource([
        sample(timestamp_ns=1, grip=0.0),
        sample(timestamp_ns=2, grip=0.0),
        sample(timestamp_ns=3, grip=0.0),
    ])
    result = wait_for_fresh_released_input(
        source,
        sample_count=3,
        timeout_s=1.0,
        monotonic=FakeClock().monotonic,
        sleep=lambda _: None,
    )
    assert result.last_timestamp_ns == 3
    assert result.grip_released is True


def test_admission_rejects_source_change_without_connecting_robot() -> None:
    source = HealthScriptedSource(source_changes_during_window=True)
    with pytest.raises(InputAdmissionError, match="source changed"):
        wait_for_fresh_released_input(source, sample_count=3, timeout_s=1.0)


def test_move_confirmation_is_exact_and_requires_released_recheck() -> None:
    assert confirm_move(input_fn=lambda _: "MOVE") is None
    with pytest.raises(InputAdmissionError, match="exact MOVE"):
        confirm_move(input_fn=lambda _: "move")
    with pytest.raises(InputAdmissionError, match="Grip must remain released"):
        verify_released_now(
            ScriptedSource([sample(timestamp_ns=4, grip=1.0)]),
            after_timestamp_ns=3,
            timeout_s=0.5,
        )
```

- [ ] **Step 2: Run the tests and verify RED**

```powershell
& $python -m pytest tests\test_hardware_admission.py tests\test_pico_udp_input.py -q
```

Expected: the admission module is missing.

- [ ] **Step 3: Implement pure admission helpers**

```python
@dataclass(frozen=True)
class InputAdmissionResult:
    accepted_samples: int
    last_timestamp_ns: int
    grip_released: bool
    active_source: tuple[str, int] | None


def confirm_move(*, input_fn: Callable[[str], str] = input) -> None:
    if input_fn("Type MOVE to enable Kortex motion: ") != "MOVE":
        raise InputAdmissionError("hardware motion requires exact MOVE confirmation")
```

`wait_for_fresh_released_input()` must reject malformed/non-finite samples, require strictly increasing timestamps, observe one stable source when health is available, and succeed only when the final accepted sample has Grip `< 0.8`. `verify_released_now()` requires one newer finite sample with Grip `< 0.8` after confirmation. Both use injected clock/sleep functions in tests and never log the source address.

- [ ] **Step 4: Make PICO session boundaries observable without exposing addresses**

When a same-device endpoint change clears a PICO session, set `last_error` to the stable string `source changed` and return one invalid sample. Continue ignoring unrelated foreign packets from a different IP while incrementing the health counter. Add tests proving both behaviors.

- [ ] **Step 5: Run the focused tests**

```powershell
& $python -m pytest tests\test_hardware_admission.py tests\test_pico_udp_input.py tests\test_pico_protocol.py -q
```

Expected: all tests pass.

- [ ] **Step 6: Commit**

```powershell
git add kinova_teleop\hardware_admission.py kinova_teleop\pico_udp_input.py tests\test_hardware_admission.py tests\test_pico_udp_input.py
git commit -m "feat: admit only fresh released PICO input"
```

### Task 4: Anchor-relative translation and orientation envelope

**Files:**
- Modify: `kinova_teleop/workspace.py:1-68`
- Modify: `kinova_teleop/kortex_backend.py:85-146,247-487`
- Modify: `tests/test_workspace.py`
- Modify: `tests/test_kortex_backend.py`

**Interfaces:**
- Produces `AnchorEnvelope(maximum_translation_axis_m, maximum_rotation_rad)` and `evaluate(anchor: Pose, target: Pose) -> WorkspaceDecision`.
- `KortexBackend.__init__` requires `anchor_envelope: AnchorEnvelope` for motion.
- `KortexBackend.current_pose()` records the current pose as the control anchor only while satisfying the existing re-arm feedback transaction.

- [ ] **Step 1: Write failing pure-envelope tests**

```python
def test_anchor_envelope_accepts_boundary_and_rejects_translation_overrun() -> None:
    envelope = AnchorEnvelope((0.02, 0.02, 0.02), math.radians(5.0))
    anchor = pose_at(0.10, -0.20, 0.30)
    assert envelope.evaluate(anchor, pose_at(0.12, -0.18, 0.28)).accepted
    rejected = envelope.evaluate(anchor, pose_at(0.120001, -0.20, 0.30))
    assert not rejected.accepted
    assert rejected.reason == "target outside anchor translation envelope"


def test_anchor_envelope_uses_shortest_arc_orientation() -> None:
    envelope = AnchorEnvelope((0.02, 0.02, 0.02), math.radians(5.0))
    anchor = Pose(np.zeros(3), quat_from_axis_angle(np.array([0, 0, 1]), 0.0))
    inside = Pose(np.zeros(3), quat_from_axis_angle(np.array([0, 0, 1]), math.radians(5.0)))
    outside = Pose(np.zeros(3), quat_from_axis_angle(np.array([0, 0, 1]), math.radians(5.01)))
    assert envelope.evaluate(anchor, inside).accepted
    assert not envelope.evaluate(anchor, outside).accepted
```

- [ ] **Step 2: Run tests and verify RED**

```powershell
& $python -m pytest tests\test_workspace.py tests\test_kortex_backend.py -q
```

Expected: `AnchorEnvelope` is missing and the backend does not capture an anchor.

- [ ] **Step 3: Implement immutable envelope validation**

```python
@dataclass(frozen=True)
class AnchorEnvelope:
    maximum_translation_axis_m: tuple[float, float, float]
    maximum_rotation_rad: float

    def evaluate(self, anchor: Pose, target: Pose) -> WorkspaceDecision:
        delta = np.abs(np.asarray(target.position) - np.asarray(anchor.position))
        if np.any(delta > np.asarray(self.maximum_translation_axis_m)):
            return WorkspaceDecision(False, "target outside anchor translation envelope")
        error = quat_multiply(target.quaternion, quat_conjugate(anchor.quaternion))
        if np.linalg.norm(quat_to_rotvec(error)) > self.maximum_rotation_rad:
            return WorkspaceDecision(False, "target outside anchor rotation envelope")
        return WorkspaceDecision(True, "")
```

Validate positive finite limits and finite poses in `__post_init__`/`evaluate()`. Preserve `WorkspaceLimits` as the independent absolute XYZ gate.

- [ ] **Step 4: Capture and enforce the anchor before any twist RPC**

During `current_pose()`, assign an immutable pose copy to `_control_anchor` only when `rearm_generation` succeeds. In `command_pose()`, evaluate absolute bounds first and the anchor envelope second, before requesting another feedback frame or constructing `TwistCommand`. A rejection must request Stop, latch the reason, and raise `KortexSafetyError`; it must never silently clamp the target or return to ACTIVE.

```python
decision = self.anchor_envelope.evaluate(self._control_anchor, target)
if not decision.accepted:
    self._latch_fault_and_stop(decision.reason)
    raise KortexSafetyError(decision.reason)
```

- [ ] **Step 5: Run focused tests**

```powershell
& $python -m pytest tests\test_workspace.py tests\test_kortex_backend.py -q
```

Expected: all tests pass; rejected targets cause one Stop attempt and zero twist sends.

- [ ] **Step 6: Commit**

```powershell
git add kinova_teleop\workspace.py kinova_teleop\kortex_backend.py tests\test_workspace.py tests\test_kortex_backend.py
git commit -m "feat: enforce anchored Gen3 pose envelope"
```

### Task 5: Fatal input and watchdog faults escape the hardware loop

**Files:**
- Modify: `kinova_teleop/xr_input.py:14-31`
- Modify: `kinova_teleop/pico_udp_input.py:63-220`
- Modify: `kinova_teleop/pose_mapping.py:20-64,242-445`
- Modify: `kinova_teleop/teleop_controller.py:14-189`
- Modify: `kinova_teleop/kortex_backend.py:43-48,120-146,225-247,559-712`
- Modify: `tests/test_pose_mapping.py`
- Modify: `tests/test_teleop_controller.py`
- Modify: `tests/test_kortex_backend.py`

**Interfaces:**
- Adds `ControllerSample.invalid_reason: str = ""`.
- Adds `InputFault(str, Enum)` with `NONE`, `INVALID`, `STALE`, and `SOURCE_CHANGED`.
- Adds `MappingOutput.input_fault: InputFault`.
- Adds `TeleopConfig.fatal_input_faults: bool = False` and `TeleopSafetyError`.
- Adds `KortexBackend.fault_reason: str | None`; a latched backend fault prevents `begin_control()` and makes the next foreground call raise.

- [ ] **Step 1: Write failing mapper/controller fault tests**

```python
def test_hardware_stale_input_stops_and_escapes_loop() -> None:
    backend = RecordingBackend()
    source = ScriptedInput([
        released_sample(1),
        pressed_sample(2),
        invalid_sample("stream is stale"),
    ])
    controller = TeleopController(
        TeleopConfig(realtime=False, fatal_input_faults=True),
        source,
        backend,
    )
    controller.step_once()
    controller.step_once()
    with pytest.raises(TeleopSafetyError, match="stale"):
        controller.step_once()
    assert backend.hold_calls == 1


def test_normal_release_remains_recoverable_in_hardware_policy() -> None:
    controller, backend = active_controller(fatal_input_faults=True)
    released = controller.step_once()
    assert released.clutch_state is ClutchState.READY
    assert backend.hold_calls == 1
    assert controller.step_once().active is True


def test_source_change_is_fatal_but_mujoco_default_keeps_existing_behavior() -> None:
    mapping = mapper_output_for(invalid_sample("source changed"))
    assert mapping.input_fault is InputFault.SOURCE_CHANGED
    controller = controller_for(invalid_sample("source changed"), fatal_input_faults=False)
    assert controller.step_once().active is False
```

- [ ] **Step 2: Write failing watchdog foreground-propagation test**

```python
def test_watchdog_latches_fault_and_next_foreground_call_raises() -> None:
    backend, clock, connection = active_backend_with_command()
    clock.advance(0.201)
    backend.check_watchdog()
    assert backend.fault_reason == "watchdog timeout"
    assert connection.base.stop_calls == 1
    with pytest.raises(KortexSafetyError, match="watchdog timeout"):
        backend.command_pose(valid_target())
    with pytest.raises(KortexSafetyError, match="latched"):
        backend.begin_control()
```

- [ ] **Step 3: Run tests and verify RED**

```powershell
& $python -m pytest tests\test_pose_mapping.py tests\test_teleop_controller.py tests\test_kortex_backend.py -q
```

Expected: missing input-fault fields and watchdog fault propagation failures.

- [ ] **Step 4: Implement stable fault classification and controller policy**

Map `stream is stale` to `STALE`, `source changed` to `SOURCE_CHANGED`, and every other invalid sample to `INVALID`. On a fatal hardware input fault, emit the reason, call `backend.hold()`, then raise `TeleopSafetyError`; if hold fails, raise `TeleopSafetyError("Stop attempted but unconfirmed")` chained from the hold failure. Grip release continues to call hold and return READY without latching.

```python
if mapping.input_fault is not InputFault.NONE and self.config.fatal_input_faults:
    self._emit("input_fault", "FAULTED", {"reason": mapping.input_fault.value})
    try:
        self.backend.hold()
    except BaseException as error:
        raise TeleopSafetyError("Stop attempted but unconfirmed") from error
    raise TeleopSafetyError(f"fatal input fault: {mapping.input_fault.value}")
```

- [ ] **Step 5: Latch background/backend faults**

Add `_fault_reason` under `_state_lock`. Workspace violation, watchdog expiry, non-finite target/feedback, RPC failure, and failed Stop set it once. `begin_control()` refuses every latched fault. `command_pose()`, `current_pose()`, and `step()` surface the stored reason to the foreground loop. Normal `hold()` does not set `_fault_reason`.

- [ ] **Step 6: Run focused and MuJoCo regression tests**

```powershell
& $python -m pytest tests\test_pose_mapping.py tests\test_teleop_controller.py tests\test_kortex_backend.py tests\test_backend_controller.py -q
& $python -m kinova_teleop.main --dry-run --headless --steps 500
```

- [ ] **Step 7: Commit**

```powershell
git add kinova_teleop\xr_input.py kinova_teleop\pico_udp_input.py kinova_teleop\pose_mapping.py kinova_teleop\teleop_controller.py kinova_teleop\kortex_backend.py tests\test_pose_mapping.py tests\test_teleop_controller.py tests\test_kortex_backend.py
git commit -m "feat: latch fatal Gen3 teleoperation faults"
```

### Task 6: Remove the first-phase gripper write surface

**Files:**
- Modify: `kinova_teleop/kortex_backend.py:25-37,140-146,487-557`
- Modify: `kinova_teleop/teleop_controller.py:17-22,47-51,103-107`
- Modify: `kinova_teleop/main.py:207-211,306-311,674-682`
- Modify: `tests/test_backend_controller.py:280-360`
- Modify: `tests/test_kortex_features.py:1-578`
- Modify: `tests/test_cli_kortex.py:160-195,470-520`

**Interfaces:**
- `KortexBackend` no longer exposes `command_gripper`.
- `TeleopConfig` no longer has a gripper field and `TeleopController` never reads trigger for actuation.
- CLI retains deprecated `--gripper` only to reject it before password lookup, import, prompt, or connection.

- [ ] **Step 1: Add the failing no-write-surface test**

```python
from pathlib import Path


def test_first_hardware_backend_exposes_no_gripper_write_surface() -> None:
    backend = _backend(_Connection())
    assert not hasattr(backend, "command_gripper")
    production = "\n".join(
        path.read_text(encoding="utf-8")
        for path in Path("kinova_teleop").glob("*.py")
    )
    assert "SendGripperCommand" not in production
```

- [ ] **Step 2: Run the focused tests and verify RED**

```powershell
& $python -m pytest tests\test_kortex_features.py tests\test_backend_controller.py tests\test_cli_kortex.py -q
```

Expected: the new test fails because `KortexBackend.command_gripper` still exists.

- [ ] **Step 3: Remove production gripper command code and contradictory tests**

Delete gripper constants, state, protobuf message construction, RPC calls, controller forwarding, and `TeleopConfig.gripper`. Replace old positive gripper tests with assertions that:

```python
assert main(["--gripper"]) == 2
assert sdk_imports == 0
assert robot_connections == 0
assert move_prompts == 0
```

Keep the PICO trigger field in the wire protocol for backward compatibility, but it remains telemetry only.

- [ ] **Step 4: Run focused tests**

```powershell
& $python -m pytest tests\test_kortex_features.py tests\test_backend_controller.py tests\test_cli_kortex.py tests\test_pico_protocol.py -q
```

- [ ] **Step 5: Commit**

```powershell
git add kinova_teleop\kortex_backend.py kinova_teleop\teleop_controller.py kinova_teleop\main.py tests\test_backend_controller.py tests\test_kortex_features.py tests\test_cli_kortex.py
git commit -m "fix: disable Gen3 gripper writes for first hardware"
```

### Task 7: Integrate admission ordering into the Kortex CLI path

**Files:**
- Modify: `kinova_teleop/main.py:48-62,454-733`
- Modify: `tests/test_cli_kortex.py`
- Modify: `tests/test_backend_controller.py`

**Interfaces:**
- Consumes `validate_kortex_runtime()`, `wait_for_fresh_released_input()`, `require_live_kortex_ready()`, `confirm_move()`, `verify_released_now()`, `AnchorEnvelope`, and the existing reviewed-report/motion-lease gates.
- Produces one fail-closed startup order and passes `fatal_input_faults=True` only for Kortex.

- [ ] **Step 1: Write the failing ordered-integration test**

```python
def test_motion_startup_order_is_input_readonly_move_recheck_then_backend(monkeypatch) -> None:
    events: list[str] = []
    install_valid_motion_gate_files(monkeypatch)
    monkeypatch.setattr(main_module, "validate_kortex_runtime", lambda: events.append("runtime"))
    monkeypatch.setattr(main_module, "create_input", lambda args: events.append("source") or FakeReleasedPico())
    monkeypatch.setattr(main_module, "wait_for_fresh_released_input", lambda source, **kw: events.append("input_admit") or admitted())
    monkeypatch.setattr(main_module, "_create_kortex_connection", readonly_then_motion_connection(events))
    monkeypatch.setattr(main_module, "run_kortex_readonly_preflight", lambda *a, **k: events.append("readonly_preflight") or ready_report())
    monkeypatch.setattr(main_module, "require_live_kortex_ready", lambda report: events.append("readonly_accept"))
    monkeypatch.setattr(main_module, "confirm_move", lambda: events.append("MOVE"))
    monkeypatch.setattr(main_module, "verify_released_now", lambda *a, **k: events.append("release_recheck"))
    monkeypatch.setattr(main_module, "_create_kortex_backend", lambda *a, **k: events.append("backend") or FakeBackend())

    assert main(valid_motion_argv()) == 0
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
```

- [ ] **Step 2: Add fail-before-connect tests for every admission error**

Parameterize runtime mismatch, no PICO samples, pressed Grip, PICO source change, live firmware mismatch, wrong operating/servoing mode, read-only cleanup failure, wrong confirmation, and failed released-Grip recheck. Each case must return 2 with `motion_connect == 0` and `SendTwistCommand == 0`.

- [ ] **Step 3: Run tests and verify RED**

```powershell
& $python -m pytest tests\test_cli_kortex.py tests\test_backend_controller.py -q
```

Expected: event ordering differs because the current code prompts `MOVE` before constructing and checking PICO input.

- [ ] **Step 4: Refactor `main()` into the approved order**

Keep `_validate_args()` and `_validate_kortex_args()` pure and before imports/network. For Kortex motion:

```python
versions = validate_kortex_runtime()
source = create_input(args)
admitted = wait_for_fresh_released_input(source, sample_count=10, timeout_s=args.check_timeout)
readonly = _create_kortex_connection(config)
try:
    live_report = run_kortex_readonly_preflight(readonly, _preflight_context(args, versions))
    require_live_kortex_ready(live_report)
finally:
    if not readonly.close(send_stop=False):
        raise RuntimeError("read-only Kortex cleanup failed")
confirm_move()
verify_released_now(source, after_timestamp_ns=admitted.last_timestamp_ns, timeout_s=0.5)
connection = _create_kortex_connection(config)
backend = _create_kortex_backend(
    connection,
    max_linear_speed=resolved_linear_speed,
    max_angular_speed_deg=resolved_angular_speed,
    workspace_limits=workspace_limits,
    anchor_envelope=AnchorEnvelope(
        FIRST_HARDWARE_PROFILE.anchor_translation_axis_m,
        math.radians(FIRST_HARDWARE_PROFILE.anchor_rotation_deg),
    ),
)
```

Pass `fatal_input_faults=True` to Kortex `TeleopConfig`. Do not reuse the read-only connection for motion. Do not reconnect in any exception handler. Preserve cleanup order and exit 130 for `Ctrl+C`; all other hardware admission/control failures exit 2.

- [ ] **Step 5: Verify first press cannot send a nonzero twist**

Add an integration test with released, pressed, then moved samples. Assert any twist emitted on the press/anchor cycle has six zero components; the first nonzero `SendTwistCommand` occurs only on the later moved sample and respects 0.005 m/s and 2 deg/s.

- [ ] **Step 6: Run CLI, controller, backend, and MuJoCo regressions**

```powershell
& $python -m pytest tests\test_cli.py tests\test_cli_kortex.py tests\test_backend_controller.py tests\test_kortex_backend.py -q
& $python -m kinova_teleop.main --dry-run --headless --steps 1000
```

- [ ] **Step 7: Commit**

```powershell
git add kinova_teleop\main.py tests\test_cli_kortex.py tests\test_backend_controller.py
git commit -m "feat: order guarded PICO Kortex admission"
```

### Task 8: Launcher, operator runbook, coverage, and whole-branch verification

**Files:**
- Create: `scripts/start_kortex_pico_teleop.ps1`
- Create: `tests/test_kortex_launch_script.py`
- Modify: `configs/gen3_micro_axes.json`
- Modify: `docs/kortex-hardware-quickstart.md`
- Modify: `pyproject.toml:14-19`
- Modify: `tests/test_offline_env_script.py`

**Interfaces:**
- The launcher accepts explicit `WorkspaceMin`, `WorkspaceMax`, `MotionLease`, `PreflightReport`, and optional `EvidenceJsonl`, `RobotIp`, `RobotUser`, and `PythonPath`.
- The launcher has no password parameter and supplies fixed first-hardware values to `kinova_teleop.main`.
- The runbook separates offline, read-only, and hardware-observed evidence and stops after T0 until the onsite operator authorizes motion.

- [ ] **Step 1: Write failing launcher contract tests**

```python
import re
from pathlib import Path


def test_kortex_launcher_hardcodes_only_the_approved_profile() -> None:
    script = Path("scripts/start_kortex_pico_teleop.ps1").read_text(encoding="utf-8")
    assert "--control-hz', '40'" in script
    assert "--scale', '0.25'" in script
    assert "--stale-timeout', '0.2'" in script
    assert "--max-linear-speed', '0.005'" in script
    assert "--max-angular-speed-deg', '2'" in script
    assert "Password" not in script
    assert "KINOVA_PASSWORD=" not in script


def test_launcher_requires_external_bounds_lease_and_reviewed_report() -> None:
    script = Path("scripts/start_kortex_pico_teleop.ps1").read_text(encoding="utf-8")
    for parameter in ("WorkspaceMin", "WorkspaceMax", "MotionLease", "PreflightReport"):
        declaration = rf"\[Parameter\(Mandatory=\$true\)\][^\r\n]*\${parameter}\b"
        assert re.search(declaration, script)
```

- [ ] **Step 2: Run the new test and verify RED**

```powershell
& $python -m pytest tests\test_kortex_launch_script.py -q
```

Expected: the launcher file does not exist.

- [ ] **Step 3: Implement the fail-closed PowerShell launcher**

Use parameter arrays typed as `[double[]]`, require exactly three finite values for each bound, resolve every file with `Resolve-Path -LiteralPath`, require the preflight and lease to be files, and invoke Python with an argument array rather than a constructed command string. The launcher must not read, set, print, or accept the password; Python performs the environment-only check.

```powershell
$launchArguments = @(
    '-m', 'kinova_teleop.main',
    '--backend', 'kortex', '--enable-hardware',
    '--input', 'pico-udp',
    '--robot-ip', $RobotIp, '--robot-user', $RobotUser,
    '--control-hz', '40', '--scale', '0.25', '--stale-timeout', '0.2',
    '--max-linear-speed', '0.005', '--max-angular-speed-deg', '2',
    '--workspace-min'
) + @($WorkspaceMin | ForEach-Object { $_.ToString('R', [Globalization.CultureInfo]::InvariantCulture) }) + @(
    '--workspace-max'
) + @($WorkspaceMax | ForEach-Object { $_.ToString('R', [Globalization.CultureInfo]::InvariantCulture) }) + @(
    '--motion-lease', $resolvedLease,
    '--preflight-report', $resolvedPreflight
)
& $resolvedPython @launchArguments
exit $LASTEXITCODE
```

- [ ] **Step 4: Align fixed micro-trajectory and documentation**

Set `configs/gen3_micro_axes.json` to `control_hz: 40.0` and `max_speed_mps: 0.005`. Rewrite the quickstart limits and startup order. Explicitly document:

- current tested stack: Python 3.11.15, Kortex `2.8.0.post5`, protobuf `3.20.0`, firmware `2.8.0-5`;
- fresh PICO/released Grip before live read-only connection and `MOVE`;
- reviewed report and lease are necessary but do not replace current live checks;
- anchor-relative ±0.02 m / 5 deg envelope plus explicit absolute XYZ bounds;
- gripper and vision writes disabled;
- release is recoverable, while stale/invalid/source/RPC/workspace/watchdog faults latch and exit;
- Stop RPC return is not proof of physical stillness;
- T0-T3 evidence labels and the mandatory pause between T0 and first motion.

Do not include a reusable broad workspace or a one-line unattended motion command.

- [ ] **Step 5: Add and run the 80% coverage gate**

Add `pytest-cov>=5.0` to `[project.optional-dependencies].dev`, update the offline-environment contract test to require the coverage tool in the wheelhouse installation, and run:

```powershell
& $python -m pip install -e ".[dev]"
$coverageTemp = Join-Path $env:TEMP ('gen3-coverage-' + [guid]::NewGuid().ToString('N'))
& $python -m pytest -q --basetemp=$coverageTemp --cov=kinova_teleop --cov-report=term-missing --cov-fail-under=80
```

Expected: all tests pass and total `kinova_teleop` coverage is at least 80%.

- [ ] **Step 6: Verify the exact Kortex runtime without network access**

```powershell
$kortexPython = 'D:\yezq\26.7.26_paper_prepare_CC\Kinova_Gen3\.venv-kortex-2.8\Scripts\python.exe'
& $kortexPython -c "from importlib.metadata import version; import kortex_api; assert version('kortex-api') == '2.8.0.post5'; assert version('protobuf') == '3.20.0'; print('Kortex offline import OK')"
```

Expected: `Kortex offline import OK`. This command imports the SDK locally and does not connect to the robot.

- [ ] **Step 7: Run full offline and formatting verification**

```powershell
& $python -m kinova_teleop.main --dry-run --headless --steps 2000
& $python -m kinova_teleop.main --help
git diff --check
git status --short
```

Expected: finite MuJoCo state, help text with safe Kortex defaults, no whitespace errors, and only intentional tracked changes plus pre-existing untracked artifacts.

- [ ] **Step 8: Perform required code and security reviews**

Review the complete range from `d812136` through the implementation HEAD. The correctness review must cover state transitions, first-press no-jump, quaternion units/reference frame, lock ordering, watchdog races, cleanup idempotence, and MuJoCo regression. The security review must cover password redaction, private endpoint validation, path/symlink validation, report schema strictness, zero pre-admission writes, zero gripper/vision writes, and absence of automatic reconnect. Fix every Critical/High finding and rerun Steps 5-7.

- [ ] **Step 9: Commit documentation and tooling**

```powershell
git add scripts\start_kortex_pico_teleop.ps1 tests\test_kortex_launch_script.py configs\gen3_micro_axes.json docs\kortex-hardware-quickstart.md pyproject.toml tests\test_offline_env_script.py
git commit -m "docs: add guarded Gen3 hardware trial workflow"
```

## Post-implementation onsite checkpoints

These checkpoints are execution gates, not claims produced by offline tests. The agent must use `operating-kinova-gen3-kortex` again immediately before any hardware connection.

1. **T0 — current read-only evidence:** Recheck physical E-stop reachability, second observer, clear workspace, fixture, cables, payload/TCP, and Web App status. Run only the exact `--check-kortex` path with `CONNECT`, save a new non-overwriting report, and verify L53/7DoF, firmware 2.8.0-5, Running, Single Level Servoing, SERVOING_READY, and finite pose. Do not proceed if read-only cleanup fails.
2. **Mandatory pause:** Present the T0 result to the user. Do not construct or present the session's motion invocation until the actual anchor pose and tight absolute workspace have been reviewed, the lease/report paths are validated, Grip is released, and the user confirms that the onsite safety conditions still hold.
3. **T1 — arm/no-jump:** With the approved launcher and exact `MOVE`, verify no motion before Grip press, first press only anchors, and release requests Stop. If Stop is unconfirmed, use physical E-stop/Web intervention and do not relaunch.
4. **T2 — six-axis micro-motion:** Test X/Y/Z separately at millimetre scale, then Roll/Pitch/Yaw separately at small angle. Record commanded direction, observed feedback direction, peak speed, envelope status, and Stop reason. Any sign/reference mismatch ends the trial.
5. **T3 — stop paths:** One at a time, verify Grip release, PICO stale, controlled source/session loss, `Ctrl+C`, and watchdog stall. Record event-to-host-Stop-request time separately from feedback-to-stationary time. Automatic recovery or continued motion is a failure.
6. **Completion claim:** Mark the system “hardware-observed” only when the same code revision and current device session have passing T0-T3 evidence. Until then, report only “offline verified” or “read-only verified”.
