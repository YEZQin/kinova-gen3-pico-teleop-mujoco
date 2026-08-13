# Gen3 Recoverable Boundary and Asymmetric Workspace Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep calibrated Gen3 PICO teleoperation alive after a confirmed boundary Stop, and admit the operator-approved asymmetric workspace at 0.02 m/s without weakening fatal safety paths.

**Architecture:** Kortex remains the owner of target validation and Stop confirmation. A new immutable backend result bit reports only a successfully stopped, recoverable boundary; `TeleopController` then moves `RelativePoseMapper` to `waiting_for_release`, and a later Grip press uses the existing feedback re-arm transaction. CLI admission separately permits the exact asymmetric span and 0.02 m/s only for the calibrated responsive hardware profile.

**Tech Stack:** Python 3.11, pytest, NumPy, Kortex API 2.8.0.post5 fakes/offline tests, PowerShell launcher/static AST checks.

## Global Constraints

- Approved center: `[0.743218601, -0.070251882, 0.057065614]` m.
- Approved minimum: `[0.143218601, -0.670251882, 0.017065614]` m.
- Approved maximum: `[1.343218601, 0.529748118, 0.657065614]` m.
- Maximum total spans: X `1.2 m`, Y `1.2 m`, Z `0.64 m`.
- Per-Grip translation envelope remains `(0.05, 0.05, 0.05)` m; scale remains `0.8`.
- Maximum linear speed is `0.02 m/s` only for the responsive calibrated hardware profile; angular speed remains `2 deg/s`.
- Control rate remains `40 Hz`; stale timeout remains `0.2 s`; translation-only and operator calibration remain mandatory.
- Exact `MOVE`, source lock, post-MOVE release check, watchdog, Stop confirmation, motion lease, preflight, child-only password, and cleanup remain unchanged.
- Boundary recovery is allowed only after confirmed Stop. Unconfirmed Stop, invalid/source-changed input, RPC/watchdog/robot/cleanup faults remain fatal.
- No Home, stored actions, gripper command, low-level cyclic control, clamp-to-boundary, watchdog bypass, or unbounded workspace.

---

### Task 1: Return an explicit recoverable result after confirmed boundary Stop

**Files:**
- Modify: `kinova_teleop/backend.py`
- Modify: `kinova_teleop/kortex_backend.py`
- Test: `tests/test_kortex_backend.py`
- Test: `tests/test_kortex_features.py`

**Interfaces:**
- Produces: `BackendResult(..., reanchor_required: bool = False)`.
- Produces: Kortex boundary rejection that returns `reanchor_required=True` only after `_attempt_stop(..., raise_on_failure=True)` succeeds.
- Preserves: all non-boundary faults use the existing latched-fault path.

- [ ] **Step 1: Write failing backend tests**

Add tests using the existing fake Kortex connection that activate control, establish a feedback anchor, and command targets just outside the anchor and absolute workspace. Assert literal behavior:

```python
result = backend.command_pose(outside_target)
assert result.accepted is False
assert result.reanchor_required is True
assert result.reason == "target outside anchor translation envelope"
assert connection.base.stop_calls == 1
assert connection.base.twist_commands == []
assert backend.stop_confirmed is True
assert backend.fault_reason is None
```

For absolute rejection, require reason `target outside workspace`. Add a Stop-failure case that raises `KortexSafetyError("Stop attempted but unconfirmed")`, leaves `reanchor_required` unavailable, and records no Twist. Keep an existing non-boundary RPC-fault assertion proving it remains latched.

- [ ] **Step 2: Run RED**

Run:

```powershell
$env:PYTHONPATH = (Get-Location).Path
python -m pytest tests/test_kortex_backend.py tests/test_kortex_features.py -q -p no:cacheprovider
```

Expected: the new boundary tests fail because `BackendResult` has no `reanchor_required` field and boundary validation still latches a fatal fault.

- [ ] **Step 3: Implement the minimal backend contract**

Extend the immutable result with a default to preserve existing constructors:

```python
@dataclass(frozen=True)
class BackendResult:
    accepted: bool
    converged: bool
    position_error: float
    rotation_error: float
    reason: str
    reanchor_required: bool = False
```

In `KortexBackend`, add one private helper that validates the generation, requests Stop without setting `_fault_reason`, emits `workspace_rejected` or `anchor_rejected`, calls `_attempt_stop(token=token, raise_on_failure=True)`, and returns:

```python
BackendResult(False, False, 0.0, 0.0, reason, reanchor_required=True)
```

Use it only for the two boundary decisions before feedback/Twist construction. Do not catch or downgrade a Stop failure.

- [ ] **Step 4: Run GREEN and adjacent backend tests**

Run the Step 2 command. Expected: PASS, rejected targets produce zero Twist, exactly one confirmed Stop, and no safety fault latch.

- [ ] **Step 5: Review and commit Task 1**

Review generation checks, lock ordering, Stop token ownership, evidence ordering, and failure propagation. Then:

```powershell
git add kinova_teleop/backend.py kinova_teleop/kortex_backend.py tests/test_kortex_backend.py tests/test_kortex_features.py
git commit -m "feat: stop recoverably at Kortex boundaries"
```

---

### Task 2: Require release and feedback re-anchor without exiting the controller

**Files:**
- Modify: `kinova_teleop/pose_mapping.py`
- Modify: `kinova_teleop/teleop_controller.py`
- Test: `tests/test_pose_mapping.py`
- Test: `tests/test_teleop_controller.py`
- Test: `tests/test_backend_controller.py`

**Interfaces:**
- Consumes: `BackendResult.reanchor_required` from Task 1.
- Produces: `RelativePoseMapper.require_release() -> MappingOutput`.
- Produces: controller diagnostics `active=False`, `clutch_state=WAITING_FOR_RELEASE`, with the backend reason retained.

- [ ] **Step 1: Write failing mapper/controller tests**

Add a mapper test that activates, calls `require_release()`, and asserts:

```python
output = mapper.require_release()
assert output.clutch_state is ClutchState.WAITING_FOR_RELEASE
assert output.active is False
assert output.deactivated is True
```

Feed pressed samples afterward and prove the mapper stays waiting. Feed three fresh released timestamps and prove it becomes READY. Press again with a backend pose changed to a literal new pose and prove the activation cycle calls `begin_control/current_pose` but sends no command; the following small motion is relative to the new feedback pose.

Add a controller/backend integration fake whose first command returns `BackendResult(..., reanchor_required=True)`. Assert no exception, no second `hold()`, no command while pressed, and later re-anchor. Add a fake that raises on Stop failure and assert `TeleopSafetyError`/Kortex safety error remains terminal.

- [ ] **Step 2: Run RED**

```powershell
$env:PYTHONPATH = (Get-Location).Path
python -m pytest tests/test_pose_mapping.py tests/test_teleop_controller.py tests/test_backend_controller.py -q -p no:cacheprovider
```

Expected: FAIL because `require_release()` and `reanchor_required` handling do not exist.

- [ ] **Step 3: Implement the mapper transition and controller branch**

Implement `RelativePoseMapper.require_release()` by reusing its existing fail-closed deactivation transition with `stale=False` and `InputFault.NONE`; it must clear controller/EE references, timestamps, update time, and release streak while preserving the last finite target.

After `command_pose`, when `result.reanchor_required` is true, replace the current mapping output with `self.mapper.require_release()`. Do not call `backend.hold()` because Task 1 already confirmed Stop. Continue through `backend.step()` and return diagnostics containing the rejection reason.

- [ ] **Step 4: Run GREEN and state-machine regressions**

Run the Step 2 command. Expected: PASS. Also run:

```powershell
python -m pytest tests/test_hardware_admission.py tests/test_pico_udp_input.py tests/test_xr_input.py -q -p no:cacheprovider
```

Expected: stale recovery, source locking, release admission, and buffer lifecycle remain PASS.

- [ ] **Step 5: Review and commit Task 2**

Review that boundary recovery cannot bypass release stability or first-press anchor-only behavior, and that a source change cannot re-enter motion. Then:

```powershell
git add kinova_teleop/pose_mapping.py kinova_teleop/teleop_controller.py tests/test_pose_mapping.py tests/test_teleop_controller.py tests/test_backend_controller.py
git commit -m "feat: re-clutch after confirmed boundary Stop"
```

---

### Task 3: Admit the asymmetric workspace and doubled responsive speed

**Files:**
- Modify: `kinova_teleop/hardware_profile.py`
- Modify: `kinova_teleop/main.py` only if a resolver name or message must change
- Test: `tests/test_hardware_profile.py`
- Test: `tests/test_cli_kortex.py`
- Create local/unstaged: `results/gen3-pico-asymmetric020-reviewed-preflight-20260813.json`
- Create local/unstaged: `results/start-gen3-pico-asymmetric020-20260813.ps1`

**Interfaces:**
- Produces: responsive maximum workspace half-span tuple `(0.6, 0.6, 0.32)` m, yielding total spans `(1.2, 1.2, 0.64)` m.
- Produces: responsive maximum linear speed `0.02` m/s.
- Preserves: default `0.005` and non-responsive translation-only `0.005` speed caps; other workspace modes retain prior caps.

- [ ] **Step 1: Write failing CLI/profile tests**

Use exact literal approved bounds and assert `_validate_kortex_args(..., check_password=False)` accepts them only with responsive calibrated translation-only flags. Assert X or Y span `1.200001`, Z span `0.640001`, or speed `0.020001` rejects before password, SDK, input, `MOVE`, or connection. Assert non-responsive `0.0051` remains rejected. Assert the constructed backend still receives anchor `(0.05, 0.05, 0.05)` and exact absolute min/max.

- [ ] **Step 2: Run RED**

```powershell
$env:PYTHONPATH = (Get-Location).Path
python -m pytest tests/test_hardware_profile.py tests/test_cli_kortex.py -q -p no:cacheprovider
```

Expected: approved spans and `0.02` fail under current responsive limits.

- [ ] **Step 3: Implement exact responsive caps**

Change only:

```python
RESPONSIVE_TRANSLATION_MAX_LINEAR_SPEED_MPS = 0.02
RESPONSIVE_TRANSLATION_WORKSPACE_HALF_WIDTH_AXIS_M = (0.6, 0.6, 0.32)
```

Retain the existing floating-point spacing tolerance and all flag gating.

- [ ] **Step 4: Run GREEN and build the local guarded package**

Run Step 2. Create the new local report and launcher by copying the current strict schemas and credential lifecycle. Use exact bounds, scale `0.8`, speed `0.02`, dynamic UTC+GUID evidence path, calibration, responsive recovery, motion lease, and exact in-main `MOVE`. Bind the report to the clean implementation HEAD and state that the expanded workspace, doubled speed, and boundary recovery are hardware-unverified.

- [ ] **Step 5: Static-review and commit tracked Task 3 code**

Parse the launcher with the PowerShell AST; strictly load report/calibration/lease; verify bounds using Decimal and NumPy; confirm no parent password environment assignment, no fixed evidence reuse, and no forbidden flags. Commit tracked code/tests only:

```powershell
git add kinova_teleop/hardware_profile.py kinova_teleop/main.py tests/test_hardware_profile.py tests/test_cli_kortex.py
git commit -m "feat: admit asymmetric responsive workspace"
```

---

### Task 4: Full verification, independent reviews, and guarded handoff

**Files:**
- Verify: all tracked changes and both local Task 3 artifacts.
- Report: `.superpowers/sdd/2026-08-13-gen3-recoverable-boundary/task-4-report.md`

**Interfaces:**
- Consumes: Tasks 1–3.
- Produces: offline PASS/FAIL and one onsite launcher command; never claims hardware validation.

- [ ] **Step 1: Run full offline verification**

```powershell
$env:PYTHONPATH = (Get-Location).Path
python -m pytest -q -p no:cacheprovider
python -m coverage run -m pytest -q -p no:cacheprovider
python -m coverage report --fail-under=80
python -m compileall -q kinova_teleop tests
python -m kinova_teleop.main --dry-run --headless --steps 2000
git diff --check
```

Require zero failures, only the known Windows symlink-capability skip if still applicable, coverage at least 80%, and `finite_state=true`.

- [ ] **Step 2: Run correctness and hardware-safety reviews**

Require no Critical/High/Medium findings. Review recoverable-vs-fatal classification, Stop confirmation, zero Twist on rejection, release stability, fresh feedback re-anchor, endpoint/source lock, exact workspace/speed gating, watchdog, credentials, and unique evidence.

- [ ] **Step 3: Rebind and revalidate the local package**

After the last tracked commit, set the reviewed report `code_revision` to exact `git rev-parse HEAD`, require tracked-clean status, strict-load all artifacts, parse PowerShell AST, and confirm the next evidence path is absent. Do not execute the launcher or connect hardware.

- [ ] **Step 4: Present the guarded onsite command**

Only after the operator reconfirms the complete asymmetric sweep volume, fixture/payload/cables, reachable E-stop/Web Stop, second observer, foreground PICO, and released Grip, present:

```powershell
cd D:\yezq\26.7.24_Kinova_controll\.worktrees\kortex-merge
& .\results\start-gen3-pico-asymmetric020-20260813.ps1
```

At exact `MOVE`, test one axis at a time with millimetre-scale hand motion. Verify Grip release Stop first, then deliberately approach a per-Grip boundary and observe `waiting_for_release` without process exit. Release immediately on any mismatch.
