# Gen3 PICO Proportional Gripper Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add explicit advanced PICO-to-Gen3 translation teleoperation with Grip-clutched motion, proportional Trigger gripper control, scale `1.0`, maximum linear speed `0.05 m/s`, and runtime absolute workspace projection.

**Architecture:** Keep one `TeleopController`, one `KortexBackend`, one transport, and one serialized Kortex RPC lock. Select the relaxed limits, no per-Grip translation envelope, and clamp-to-box behavior only through `--advanced-pico-teleop`; legacy/public-first-motion profiles keep their existing limits and Stop/re-anchor behavior.

**Tech Stack:** Python 3.11, NumPy, pytest, Coverage.py/pytest-cov, Kortex API `2.8.0.post5`, protobuf `3.20.0`, Windows PowerShell 5.1.

**Spec:** `docs/superpowers/specs/2026-08-20-gen3-pico-proportional-gripper-design.md`

## Global Constraints

- Advanced mode requires Kortex hardware, `pico-udp`, translation-only, responsive stale recovery, a validated operator-axis calibration, explicit `--advanced-pico-teleop`, and explicit `--gripper`.
- Defaults are scale `1.0`, maximum linear speed `0.05 m/s`, control `40 Hz`, stale timeout at most `0.2 s`, and angular cap `2 deg/s` while orientation is held.
- Absolute XYZ bounds remain mandatory, finite, strictly ordered, and must contain the current read-only TCP position. Only advanced mode skips the old span cap.
- Advanced targets are projected to the nearest workspace boundary before feedback/Twist preparation. They never reach `SendTwistCommand` outside the box, never request Stop merely for contact, and resume immediately when the hand moves inward.
- Legacy workspace and anchor rejection remain confirmed-Stop/release/re-anchor operations.
- Trigger `0.0/1.0` means open/closed. Grip release, stale recovery, boundary contact, shutdown, and cleanup send no automatic gripper command; the gripper holds its last value.
- Gripper commands use `GRIPPER_POSITION`, finger identifier `1`, clamp to `[0,1]`, deadband `0.02`, minimum interval `0.1 s`, the existing generation/RPC lock, and no Twist-watchdog deadline.
- PICO V1 packets have no Trigger channel and must be rejected before any advanced gripper motion; only V2 Trigger-capable input is admissible.
- Changed source, malformed/non-finite input, unconfirmed Stop, watchdog failure, Kortex fault, or arm/gripper RPC failure remains fatal.
- Advanced launch remains package-guarded: exact code revision, calibration hash, transport identity, passing checklist, motion lease, unique evidence, live read-only readiness, and current pose are required. Only historical report equality for runtime scale/speed/bounds is relaxed in advanced mode.
- Launch order is offline package validation -> PICO gate -> masked password -> live read-only Kortex readiness/pose -> one exact Python-owned `MOVE` -> motion connection. There is no `HARDWARE-READY` prompt.
- Do not run PICO, Kortex, a real launcher, or hardware while implementing. Preserve unrelated dirty/untracked user artifacts.

---

### Task 1: Stabilize Trigger Capability and Controller RED Contracts

**Files:**
- Modify: `kinova_teleop/pico_protocol.py`
- Modify: `kinova_teleop/pico_udp_input.py`
- Modify: `kinova_teleop/xr_input.py`
- Modify: `kinova_teleop/teleop_controller.py`
- Modify: `tests/test_pico_protocol.py`
- Modify: `tests/test_pico_udp_input.py`
- Modify: `tests/test_backend_controller.py`
- Modify: `tests/test_teleop_controller.py`

**Interfaces:**
- Produces `PicoControllerFrame.protocol_version: int = PICO_VERSION_2`.
- Produces `ControllerSample.trigger_available: bool = True`.
- Produces `TeleopConfig.gripper: bool = False` and optional callable `backend.command_gripper(position: float) -> bool`.

- [ ] **Step 1: Preserve and extend the existing uncommitted RED tests**

Keep the current `test_gripper_forwards_trigger_only_after_active_grip` and
`test_gripper_enabled_requires_backend_support`. Add tests asserting:

```python
def test_gripper_disabled_keeps_trigger_as_telemetry_only() -> None:
    backend = RecordingBackendWithGripper()
    source = ScriptedInput([
        replace(sample([0.0, 0.0, 0.0], 0.0, 1, 1.00), trigger=0.0),
        replace(sample([0.0, 0.0, 0.0], 1.0, 2, 1.01), trigger=1.0),
        replace(sample([0.01, 0.0, 0.0], 1.0, 3, 1.02), trigger=1.0),
    ])
    controller = TeleopController(TeleopConfig(realtime=False), source, backend)
    for _ in range(3):
        controller.step_once()
    assert backend.gripper_values == []


def test_release_stale_and_reclutch_do_not_send_automatic_open() -> None:
    samples = [
        replace(sample([0.0, 0.0, 0.0], 0.0, 1, 1.00), trigger=0.0),
        replace(sample([0.0, 0.0, 0.0], 1.0, 2, 1.01), trigger=0.7),
        replace(sample([0.01, 0.0, 0.0], 1.0, 3, 1.02), trigger=0.7),
        replace(sample([0.01, 0.0, 0.0], 0.0, 4, 1.03), trigger=0.0),
        replace(sample([0.01, 0.0, 0.0], 0.0, 4, 1.30), valid=False, invalid_reason="stream is stale", trigger=0.0),
        replace(sample([0.01, 0.0, 0.0], 0.0, 5, 1.31), trigger=0.0),
        replace(sample([0.01, 0.0, 0.0], 0.0, 6, 1.32), trigger=0.0),
        replace(sample([0.01, 0.0, 0.0], 0.0, 7, 1.33), trigger=0.0),
        replace(sample([0.01, 0.0, 0.0], 1.0, 8, 1.34), trigger=0.4),
        replace(sample([0.02, 0.0, 0.0], 1.0, 9, 1.35), trigger=0.4),
    ]
    backend = RecordingBackendWithGripper()
    controller = TeleopController(
        TeleopConfig(realtime=False, gripper=True, recover_stale_input=True, recovery_release_samples=3),
        ScriptedInput(samples),
        backend,
    )
    while controller.steps < len(samples):
        controller.step_once()
    assert controller.backend.gripper_values == [0.7, 0.4]


def test_nonfinite_or_unavailable_trigger_stops_before_pose_or_gripper_write() -> None:
    bad = replace(
        sample([0.0, 0.0, 0.0], 1.0, 2, 1.01),
        trigger=float("nan"),
        trigger_available=True,
    )
    backend = RecordingBackendWithGripper()
    controller = TeleopController(
        TeleopConfig(realtime=False, gripper=True),
        ScriptedInput([bad]),
        backend,
    )
    with pytest.raises(TeleopSafetyError, match="gripper trigger"):
        controller.step_once()
    assert backend.targets == []
    assert backend.gripper_values == []
    assert backend.events == ["hold"]
```

Add protocol tests proving V2 decodes `protocol_version == 2` and
`trigger_available is True`, while V1 decodes `protocol_version == 1`,
`trigger == 0.0`, and `trigger_available is False` after the UDP adapter.

- [ ] **Step 2: Run the focused suite and record RED**

```powershell
python -m pytest tests/test_pico_protocol.py tests/test_pico_udp_input.py tests/test_backend_controller.py tests/test_teleop_controller.py -k "trigger or gripper" -q -p no:cacheprovider --basetemp $env:TEMP\kinova-gripper-task1-red
```

Expected: capability and fail-closed Trigger tests fail on the current prototype.

- [ ] **Step 3: Implement capability propagation and controller ordering**

Add `protocol_version` to `PicoControllerFrame`; set it from the decoded header.
Add `trigger_available` to `ControllerSample`; set it in `PicoUdpInput` from
`frame.protocol_version >= 2`, and preserve it through `ContinuousInputBuffer`
copies. In `TeleopController.__init__`, validate gripper support with callable
`getattr` rather than making the base backend protocol mandatory.

Before mapper update or `command_pose`, when `config.gripper` is true, require
`sample.trigger_available` and finite Trigger. On failure call `backend.hold()`;
if Stop fails raise `TeleopSafetyError("Stop attempted but unconfirmed")`, else
raise `TeleopSafetyError("fatal input fault: gripper trigger is unavailable or non-finite")`.

Keep the Grip activation frame anchor-only for arm and gripper. On later active
frames call `command_pose` first; call `command_gripper(trigger)` only if the
result does not require re-anchor. Release/stale/close never call gripper.

- [ ] **Step 4: Run GREEN and commit**

```powershell
python -m pytest tests/test_pico_protocol.py tests/test_pico_udp_input.py tests/test_xr_input.py tests/test_backend_controller.py tests/test_teleop_controller.py tests/test_pose_mapping.py -q -p no:cacheprovider --basetemp $env:TEMP\kinova-gripper-task1-green
git diff --check
git add -- kinova_teleop/pico_protocol.py kinova_teleop/pico_udp_input.py kinova_teleop/xr_input.py kinova_teleop/teleop_controller.py tests/test_pico_protocol.py tests/test_pico_udp_input.py tests/test_backend_controller.py tests/test_teleop_controller.py
git commit -m "feat: gate proportional gripper on PICO Trigger"
```

### Task 2: Harden the Existing Kortex Gripper Prototype

**Files:**
- Modify: `kinova_teleop/backend.py`
- Modify: `kinova_teleop/kortex_backend.py`
- Modify: `tests/test_kortex_backend.py`
- Modify: `tests/test_kortex_features.py`

**Interfaces:**
- Produces separate runtime-checkable `ProportionalGripperBackend` protocol; `EndEffectorTargetBackend` remains unchanged.
- Produces `KortexBackend.command_gripper(position: float) -> bool`.

- [ ] **Step 1: Extend the existing RED coverage before production fixes**

Preserve the current command-shape, pre-begin, deadband/rate, clamp,
non-finite, send-failure, no-watchdog, and queued-Stop tests. Add:

```python
def _active_gripper_backend():
    connection = _Connection(_feedback())
    clock = _Clock()
    backend = _backend(connection, monotonic=clock)
    backend.begin_control()
    backend.current_pose()
    return backend, connection, clock


def test_gripper_is_rejected_after_stop_and_accepted_after_rearm() -> None:
    backend, connection, clock = _active_gripper_backend()
    assert backend.command_gripper(0.5) is True
    backend.hold()
    assert backend.command_gripper(0.9) is False
    backend.begin_control(); backend.current_pose(); clock.now += 0.11
    assert backend.command_gripper(0.9) is True


def test_gripper_rpc_lock_timeout_latches_stop_failure() -> None:
    backend, connection, _clock = _active_gripper_backend()
    backend._rpc_lock.acquire()
    try:
        with pytest.raises(KortexSafetyError, match="Stop attempted but unconfirmed"):
            backend.command_gripper(0.5)
    finally:
        backend._rpc_lock.release()
    assert connection.base.gripper_sent == []


def test_close_never_sends_gripper_open() -> None:
    backend, connection, _clock = _active_gripper_backend()
    backend.command_gripper(0.7)
    backend.close()
    assert [c.gripper.finger.entries[0].value for c in connection.base.gripper_sent] == [0.7]
```

Update failure tests to require a latched `fault_reason`, Stop attempted, and
confirmed/unconfirmed semantics rather than raw `ValueError`/`RuntimeError`.

- [ ] **Step 2: Run RED**

```powershell
python -m pytest tests/test_kortex_backend.py tests/test_kortex_features.py -k gripper -q -p no:cacheprovider --basetemp $env:TEMP\kinova-gripper-task2-red
```

- [ ] **Step 3: Implement the current-backend fault model**

Remove `command_gripper` from `EndEffectorTargetBackend`; add:

```python
@runtime_checkable
class ProportionalGripperBackend(Protocol):
    def command_gripper(self, position: float) -> bool: ...
```

Keep `GRIPPER_DEADBAND=0.02`, `GRIPPER_MIN_INTERVAL=0.1`, the positional Kortex
message, generation recheck, and RPC lock. Replace prototype calls to raw
`_request_stop` with `_latch_fault_and_stop(reason, generation=...)` so
non-finite, lock timeout, and send failure latch one stable fault and preserve
the current Stop-token model. Update last value/time only when the same
generation is still active after send. Never call `_ensure_watchdog_ready` or
write `_deadline`.

- [ ] **Step 4: Run GREEN, review locks, and commit**

```powershell
python -m pytest tests/test_kortex_backend.py tests/test_kortex_features.py -q -p no:cacheprovider --basetemp $env:TEMP\kinova-gripper-task2-green
git diff --check
git add -- kinova_teleop/backend.py kinova_teleop/kortex_backend.py tests/test_kortex_backend.py tests/test_kortex_features.py
git commit -m "feat: add guarded Gen3 gripper commands"
```

### Task 3: Make Workspace Projection an Explicit Advanced Policy

**Files:**
- Modify: `kinova_teleop/workspace.py`
- Modify: `kinova_teleop/kortex_backend.py`
- Modify: `kinova_teleop/evidence_log.py`
- Modify: `tests/test_workspace.py`
- Modify: `tests/test_kortex_backend.py`
- Modify: `tests/test_evidence_log.py`

**Interfaces:**
- Produces `WorkspaceProjection(target: Pose, projected: bool)` and `WorkspaceLimits.project(target) -> WorkspaceProjection`.
- Produces `KortexBackend(..., advanced_translation: bool = False)`.
- Advanced invariant: non-None workspace, `anchor_envelope is None`; legacy invariant: an `AnchorEnvelope` is required.

- [ ] **Step 1: Extend RED tests**

Keep the current projection test, then add all six faces, non-finite rejection,
quaternion preservation, inward-resume in the same generation, and constructor
invariant tests. Assert advanced boundary result reason is
`"target clamped to workspace"`, `reanchor_required is False`, Stop count is
zero, and an emitted `workspace_clamped` event is allowed. Assert the exact
legacy outside-workspace test still returns confirmed Stop/re-anchor.

- [ ] **Step 2: Run RED**

```powershell
python -m pytest tests/test_workspace.py tests/test_kortex_backend.py tests/test_evidence_log.py -k "project or clamp or workspace" -q -p no:cacheprovider --basetemp $env:TEMP\kinova-gripper-task3-red
```

- [ ] **Step 3: Implement explicit projection**

Replace the prototype inference `anchor_envelope is None` with explicit
`advanced_translation`. `WorkspaceLimits.project` returns immutable position
and quaternion copies plus a projected bit. In `command_pose`, advanced mode
projects before feedback/Twist preparation and uses the projected target for
all errors; legacy mode keeps `_reject_boundary_and_stop`. Skip anchor
evaluation only in explicit advanced mode. Emit `workspace_clamped/MOVING`
after a successful clamped send; do not Stop or re-anchor.

Keep the backend implementation ceiling `0.05`, while CLI scopes lower modes.

- [ ] **Step 4: Run GREEN and commit**

```powershell
python -m pytest tests/test_workspace.py tests/test_kortex_backend.py tests/test_kortex_features.py tests/test_evidence_log.py tests/test_backend_controller.py tests/test_teleop_controller.py -q -p no:cacheprovider --basetemp $env:TEMP\kinova-gripper-task3-green
git diff --check
git add -- kinova_teleop/workspace.py kinova_teleop/kortex_backend.py kinova_teleop/evidence_log.py tests/test_workspace.py tests/test_kortex_backend.py tests/test_evidence_log.py
git commit -m "feat: clamp advanced teleop workspace targets"
```

### Task 4: Close CLI, Preflight, and Live-Pose Gaps

**Files:**
- Modify: `kinova_teleop/hardware_profile.py`
- Modify: `kinova_teleop/preflight.py`
- Modify: `kinova_teleop/main.py`
- Modify: `tests/test_hardware_profile.py`
- Modify: `tests/test_preflight.py`
- Modify: `tests/test_cli_kortex.py`

**Interfaces:**
- Uses existing `--advanced-pico-teleop`; advanced requires `--gripper` and vice versa.
- Produces `read_kortex_tool_position(connection) -> tuple[float, float, float]`.
- Advanced report loader passes `expected_safety_limits=None` but retains exact revision, calibration, transport, passing software checks, and physical checklist.

- [ ] **Step 1: Write RED scope/default/binding tests**

Add tests for exact defaults scale `1.0` and speed `0.05`, rejection of
`1.000001`/`0.050001`, non-finite/equal/reversed/missing bounds, and rejection
without any of Kortex hardware, PICO UDP, translation-only, responsive profile,
recover-stale, operator calibration, advanced flag, or gripper flag. Every
failure must precede password, SDK/input construction, `MOVE`, and connection.

Add a report test capturing `load_passing_preflight_report` kwargs and asserting
advanced mode supplies `expected_safety_limits=None` plus exact HEAD,
calibration SHA, and `{kind:"tcp", host, port:10000, robot_user}`. Revision,
calibration, transport, non-passing checklist, absent motion lease, reused
evidence, or missing report must still reject.

Add live-pose ordering tests: outside current pose closes read-only connection
and never reaches `MOVE`/motion connect; inside pose orders
`readonly_connect -> readonly_pose -> readonly_close -> MOVE -> motion_connect`.

- [ ] **Step 2: Run RED**

```powershell
python -m pytest tests/test_hardware_profile.py tests/test_preflight.py tests/test_cli_kortex.py -k "advanced_pico or gripper or live_pose" -q -p no:cacheprovider --basetemp $env:TEMP\kinova-gripper-task4-red
```

- [ ] **Step 3: Implement fail-closed advanced validation**

Keep `ADVANCED_PICO_TELEOP_MAX_SCALE=1.0` and
`ADVANCED_PICO_TELEOP_MAX_LINEAR_SPEED_MPS=0.05`. Tighten `_validate_args` to
the exact scope above; remove the prototype's lease/report bypass. Always
validate motion lease, report, and unique evidence. Skip only
`validate_workspace_span` and only in advanced mode; require strict ordered
finite axes. For advanced report loading, set only `expected_safety_limits` to
`None`; retain every other binding.

Add `read_kortex_tool_position` in `preflight.py` using read-only
`RefreshFeedback`, finite shape validation, and immutable floats. After
`require_live_kortex_ready` and before read-only close/`MOVE`, require the live
position inside runtime limits. Wire backend with
`advanced_translation=True`, `anchor_envelope=None`, and controller
`gripper=True`; legacy modes remain unchanged.

- [ ] **Step 4: Run GREEN and commit**

```powershell
python -m pytest tests/test_hardware_profile.py tests/test_preflight.py tests/test_cli_kortex.py tests/test_kortex_backend.py tests/test_backend_controller.py tests/test_teleop_controller.py -q -p no:cacheprovider --basetemp $env:TEMP\kinova-gripper-task4-green
git diff --check
git add -- kinova_teleop/hardware_profile.py kinova_teleop/preflight.py kinova_teleop/main.py tests/test_hardware_profile.py tests/test_preflight.py tests/test_cli_kortex.py
git commit -m "feat: admit advanced PICO gripper profile"
```

### Task 5: Guarded Parameterized Launcher with One Confirmation

**Files:**
- Create: `scripts/start_gen3_pico_teleop.ps1`
- Create: `tests/test_gen3_pico_teleop_launcher.py`

**Interfaces:**
- Mandatory: `MotionLease`, `PreflightReport`, `OperatorCalibration`, `WorkspaceMin`, `WorkspaceMax`.
- Optional defaults: `RobotHost=192.168.1.10`, `RobotUser=admin`, `Scale=1.0`, `MaxLinearSpeed=0.05`, `PythonPath`.
- Switch: `EnableGripper` is mandatory for advanced mode.

- [ ] **Step 1: Replace the unsafe prototype launcher contract with RED tests**

Tests must assert startup-hook gate, offline `--validate-motion-package` before
PICO/password, exact lease/report/calibration arguments, PICO V2 Trigger gate,
unique absent evidence, SecureString child-only password, `finally` cleanup,
advanced/gripper/scale/speed/bounds argv, no credential in argv, no
`HARDWARE-READY`, and no launcher-owned `MOVE`. Fake-child execution must prove
credential cleanup and original nonzero exit propagation. Add PowerShell AST
parse and invalid numeric/path/reparse tests.

- [ ] **Step 2: Run RED**

```powershell
python -m pytest tests/test_gen3_pico_teleop_launcher.py -q -p no:cacheprovider --basetemp $env:TEMP\kinova-gripper-task5-red
```

- [ ] **Step 3: Implement the launcher**

Refactor the existing untracked script rather than deleting it. Require the
three local artifacts and pass them to both offline validation and motion.
Parse workspace as `[double[]]`, validate exactly three finite strictly ordered
values, validate positive finite scale/speed, resolve regular non-reparse files,
run the startup-hook gate, and use the existing generated launcher's credential
lifecycle. PICO gate must verify fresh released V2 Trigger-capable samples.
The only motion confirmation remains Python's exact `MOVE`.

- [ ] **Step 4: Run GREEN and commit**

```powershell
python -m pytest tests/test_gen3_pico_teleop_launcher.py tests/test_cli_kortex.py tests/test_pico_udp_scripts.py -q -p no:cacheprovider --basetemp $env:TEMP\kinova-gripper-task5-green
powershell.exe -NoProfile -NonInteractive -Command '$errors=@();[void][System.Management.Automation.Language.Parser]::ParseFile((Resolve-Path ".\scripts\start_gen3_pico_teleop.ps1"),[ref]$null,[ref]$errors);if($errors.Count){exit 1}'
git diff --check
git add -- scripts/start_gen3_pico_teleop.ps1 tests/test_gen3_pico_teleop_launcher.py
git commit -m "feat: add parameterized PICO gripper launcher"
```

### Task 6: Bilingual Advanced Documentation

**Files:**
- Modify: `README.md`
- Modify: `README_CN.md`
- Create: `docs/advanced-gripper-teleoperation.md`
- Modify: `tests/test_public_docs.py`

**Interfaces:**
- Public first-motion lifecycle and launcher remain unchanged.
- Advanced guide contains the direct artifact-bound command and exact Grip/Trigger/hold/boundary semantics.

- [ ] **Step 1: Write RED docs contracts**

Require EN/CN links to the advanced guide and require the guide to mention
`-MotionLease`, `-PreflightReport`, `-OperatorCalibration`, `-EnableGripper`,
`-Scale 1.0`, `-MaxLinearSpeed 0.05`, workspace arrays, PICO V2, one `MOVE`,
Trigger `0/1`, Grip hold, and boundary projection. Assert it does not claim
hardware validation, remove E-stop/Web Stop, embed developer paths/credentials,
or offer an APK download. Keep the public first-motion no-gripper and
`HARDWARE-READY` contract tests unchanged.

- [ ] **Step 2: Run RED, update docs, and run GREEN**

```powershell
python -m pytest tests/test_public_docs.py tests/test_gen3_pico_teleop_launcher.py -q -p no:cacheprovider --basetemp $env:TEMP\kinova-gripper-task6-red
```

Replace the current README prototype's reusable developer workspace literals
with measured-variable syntax and an artifact-bound launcher command. Document
from-zero prerequisites by linking the existing public setup, then the advanced
delta. Run:

```powershell
python -m pytest tests/test_public_docs.py tests/test_gen3_pico_teleop_launcher.py tests/test_generated_hardware_launcher.py tests/test_public_hardware_setup.py -q -p no:cacheprovider --basetemp $env:TEMP\kinova-gripper-task6-green
rg -n "developer-machine-path|KINOVA_PASSWORD\s*=|download.*apk|unbounded|safety-rated" README.md README_CN.md docs/advanced-gripper-teleoperation.md
git diff --check
git add -- README.md README_CN.md docs/advanced-gripper-teleoperation.md tests/test_public_docs.py
git commit -m "docs: explain advanced PICO gripper teleop"
```

### Task 7: Full Offline Verification and Independent Review

**Files:**
- Review all commits after design commit `878a145`.
- No tracked changes unless a finding receives a new RED test and fix commit.

**Interfaces:**
- Produces exact offline evidence and the final parameterized command template; performs no hardware action and no remote publication.

- [ ] **Step 1: Run focused and full verification**

```powershell
python -m pytest tests/test_pico_protocol.py tests/test_pico_udp_input.py tests/test_workspace.py tests/test_kortex_backend.py tests/test_kortex_features.py tests/test_backend_controller.py tests/test_teleop_controller.py tests/test_hardware_profile.py tests/test_preflight.py tests/test_cli_kortex.py tests/test_gen3_pico_teleop_launcher.py tests/test_public_docs.py -q -p no:cacheprovider --basetemp $env:TEMP\kinova-gripper-focused-final
python -m pytest -q -p no:cacheprovider --basetemp $env:TEMP\kinova-gripper-full-final
python -m coverage run -m pytest -q -p no:cacheprovider --basetemp $env:TEMP\kinova-gripper-coverage-final
python -m coverage report --fail-under=80
python -m compileall -q kinova_teleop tests
python -m kinova_teleop.main --dry-run --headless --steps 2000
```

- [ ] **Step 2: Run static gates**

```powershell
powershell.exe -NoProfile -NonInteractive -Command '$errors=@();Get-ChildItem scripts -Filter *.ps1|ForEach-Object{[void][System.Management.Automation.Language.Parser]::ParseFile($_.FullName,[ref]$null,[ref]$errors)};if($errors.Count){exit 1}'
git diff --check 878a145..HEAD
git status --short --untracked-files=no
rg -n "SendGripperCommand|SendTwistCommand|SetServoingMode|\.Stop\(" kinova_teleop
rg -n "developer-machine-path|BEGIN (RSA|OPENSSH|EC) PRIVATE KEY|ghp_[A-Za-z0-9]+|KINOVA_PASSWORD\s*=" kinova_teleop scripts README.md README_CN.md docs/advanced-gripper-teleoperation.md
```

Require coverage at least 80%, `finite_state=true`, clean tracked state, only
the intended Kortex write surface, and no secret/developer-path finding.

- [ ] **Step 3: Independent correctness and hardware-safety review**

Correctness review covers capability propagation, Grip/Trigger ordering,
projection/inward resume, defaults/scope, report binding, live pose, launcher
argv/exit codes, and legacy/public compatibility. Hardware-safety review covers
RPC lock/Stop ordering, watchdog independence, source/stale faults, no
auto-open, no out-of-box Twist, current-pose gate, credential lifetime, and
one-`MOVE` admission. Fix all Critical/High/Medium findings with RED/GREEN and
rerun Steps 1-2.

- [ ] **Step 4: Local handoff**

Report final HEAD, commits, pass counts, coverage, simulator result, and the
offline-only evidence boundary. Do not push, modify Draft PR #3, tag, or release
without separate explicit publication approval.

## Self-Review

- **Spec coverage:** Tasks 1-2 cover Trigger capability, proportional gripper,
  Grip hold, Stop/RPC/watchdog behavior; Tasks 3-4 cover workspace projection,
  scale/speed scope, live pose, and report identity; Task 5 covers the simplified
  parameterized launch; Task 6 covers bilingual zero-to-advanced use; Task 7
  covers all completion gates.
- **Placeholder scan:** No `TBD`, `TODO`, “implement later”, unnamed test, or
  blanket “add error handling” remains. Robot-specific bounds are intentionally
  runtime inputs, not copyable defaults.
- **Type consistency:** The plan consistently uses `TeleopConfig.gripper`,
  `ProportionalGripperBackend.command_gripper(float)->bool`,
  `ControllerSample.trigger_available`, `WorkspaceLimits.project`,
  `KortexBackend.advanced_translation`, and `--advanced-pico-teleop`.
