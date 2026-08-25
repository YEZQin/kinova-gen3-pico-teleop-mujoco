# Gen3 Recentered 100 mm Workspace Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver a new guarded Gen3 calibrated launcher centered on the current read-only tool pose with a 100 mm per-axis half-width and collision-free evidence filenames on every invocation.

**Architecture:** Production mapping, Kortex transport, and safety code remain unchanged at implementation commit `8122b486c8eae87c6ef56006b9fee388c86af701`. Two new local controlled artifacts provide the enlarged reviewed preflight and launcher; the launcher creates one immutable timestamp-plus-GUID evidence path before any PICO/password/hardware admission and never reuses or deletes prior evidence.

**Tech Stack:** PowerShell 7/Windows PowerShell AST, Python 3.11.15 Kortex environment, strict project JSON loaders, pytest, SHA-256.

## Global Constraints

- New center is exactly `[0.746914864, -0.082188472, 0.064038418]` m.
- New minimum is exactly `[0.646914864, -0.182188472, -0.035961582]` m.
- New maximum is exactly `[0.846914864, 0.017811528, 0.164038418]` m.
- Each center-to-bound distance is exactly `0.100000000` m; the onsite operator confirmed the complete cube, links, gripper, cables, payload, and lowest Z plane are clear.
- Per-Grip anchor translation remains `0.050 m` per axis; anchor rotation remains `5 deg`.
- Keep calibration SHA-256 `682ff4dbe5ad572d9e2a66828fd823edcbe6290ee3e860db322821ab65028c02`, scale `0.8`, maximum linear speed `0.01 m/s`, control rate `40 Hz`, stale timeout `0.2 s`, maximum angular speed `2 deg/s`, and translation-only responsive recovery.
- Keep exact `MOVE`, release-first PICO admission, source lock, post-MOVE recheck, motion lease, reviewed preflight, watchdog, Stop confirmation, and cleanup unchanged.
- Do not add `--invert-translation`, gripper control, Home, stored actions, low-level cyclic control, timeout increases, or watchdog bypass.
- Preserve `results/gen3-pico-calibrated080-events-20260813.jsonl` byte-for-byte; never delete, truncate, rename, append, or reuse it.
- New event names must match `gen3-pico-calibrated080-workspace100-events-<UTC>-<GUID>.jsonl`, be absent before use, remain under the existing `results` directory, and be printed after the child exits.
- Do not connect PICO or Kortex, probe a network port, or execute the new launcher during implementation/verification.
- The new `results` artifacts are local, untracked, and unstaged; do not add them to Git. Only the plan document is tracked.

---

### Task 1: Freeze prior evidence and create the enlarged reviewed preflight

**Files:**
- Read only: `results/gen3-pico-calibrated080-events-20260813.jsonl`
- Read only: `results/pico-operator-axes-20260813-051504.json`
- Create, local/untracked: `results/gen3-pico-calibrated080-workspace100-reviewed-preflight-20260813.json`
- Report: `.superpowers/sdd/2026-08-13-gen3-recentered-workspace100/task-1-report.md`

**Interfaces:**
- Consumes: prior event bytes, exact implementation/calibration/source hashes, onsite clearance confirmation, and exact center/min/max.
- Produces: a strict passing preflight report whose safety limits describe the new workspace and dynamic evidence naming contract without claiming the enlarged workspace or directions have been hardware-validated.

- [ ] **Step 1: Record the prior evidence identity and Stop sequence**

Use `Get-FileHash -Algorithm SHA256` and parse the existing JSONL read-only. Record its SHA-256, length, and last-write time. Require one `workspace_rejected`, at least one `host_stop_requested`, one `stop_rpc_returned`, and one `cleanup_completed`; require no `stop_unconfirmed`. Save these facts only in the task report.

- [ ] **Step 2: Run the preflight contract RED gate**

Run a one-off PowerShell assertion that requires the new report path to exist, strictly loads it, and asserts exact center/min/max, half-width arithmetic, anchor `0.05`, scale `0.8`, calibrated mode, Python `3.11.15`, implementation revision, and dynamic evidence pattern.

Expected: FAIL because `results/gen3-pico-calibrated080-workspace100-reviewed-preflight-20260813.json` does not exist.

- [ ] **Step 3: Create the strict reviewed preflight artifact**

Copy the accepted calibrated report schema and change only the values required by this design. Use:

```json
{
  "code_revision": "8122b486c8eae87c6ef56006b9fee388c86af701",
  "runtime": {"python": "3.11.15", "protobuf": "3.20.0"},
  "safety_limits": {
    "workspace_center_m": [0.746914864, -0.082188472, 0.064038418],
    "workspace_min_m": [0.646914864, -0.182188472, -0.035961582],
    "workspace_max_m": [0.846914864, 0.017811528, 0.164038418],
    "anchor_translation_axis_m": 0.05,
    "evidence_jsonl_pattern": "results/gen3-pico-calibrated080-workspace100-events-<UTC>-<GUID>.jsonl"
  }
}
```

Retain every strict required top-level field and required Kortex check. Add a passing `evidence_boundary` check whose detail says the previous workspace Stop is hardware-observed, while the enlarged workspace and calibrated physical directions remain pending/unobserved. Do not name an unobserved hardware trial as a passing check.

- [ ] **Step 4: Run the report GREEN gate**

Use `load_passing_preflight_report`, recompute calibration/mapping/loader SHA-256 values, verify the exact 0.1 m arithmetic with Python/NumPy, confirm the prior evidence hash is unchanged, and run `git status --short --untracked-files=no` plus `git diff --check`.

Expected: strict load accepted, exact bounds true, prior evidence unchanged, tracked worktree clean.

- [ ] **Step 5: Obtain a task-scoped read-only review**

Review the report for schema accuracy, runtime/revision/hash binding, physical-evidence wording, exact bounds, and unchanged speed/anchor/freshness limits. Resolve every Critical/Important finding by another RED/GREEN cycle. Do not stage or commit the `results` artifact.

---

### Task 2: Create the repeatable guarded launcher

**Files:**
- Read only: `results/start-gen3-pico-calibrated080-20260813.ps1`
- Create, local/untracked: `results/start-gen3-pico-calibrated080-workspace100-20260813.ps1`
- Use: `results/gen3-pico-calibrated080-workspace100-reviewed-preflight-20260813.json`
- Report: `.superpowers/sdd/2026-08-13-gen3-recentered-workspace100/task-2-report.md`

**Interfaces:**
- Consumes: strict Task 1 report, existing calibration and lease, exact workspace parameters.
- Produces: one launcher that generates a new evidence path per invocation, keeps it fixed for that child, preserves all prior logs, and propagates child exit status.

- [ ] **Step 1: Run the launcher contract RED gate**

Run a static PowerShell assertion requiring the new launcher and checking AST syntax plus these tokens/relationships:

```text
--operator-calibration
--responsive-translation-profile
--recover-stale-input
--translation-only
--expanded-translation-envelope
--scale 0.8
--max-linear-speed 0.01
--workspace-min 0.646914864 -0.182188472 -0.035961582
--workspace-max 0.846914864 0.017811528 0.164038418
[DateTime]::UtcNow
[Guid]::NewGuid
-AsSecureString
finally
```

Require `--invert-translation`, `--gripper`, fixed trajectory, Home/action, parent-process `$env:KINOVA_PASSWORD`, and the old fixed event filename to be absent from the motion arguments.

Expected: FAIL because the new launcher does not exist.

- [ ] **Step 2: Implement dynamic evidence path creation before admission**

Create the new launcher from the accepted calibrated launcher. Before the `try` block, compute:

```powershell
$resultsDirectory = Join-Path $project 'results'
$utcStamp = [DateTime]::UtcNow.ToString(
    'yyyyMMddTHHmmssfffZ',
    [Globalization.CultureInfo]::InvariantCulture
)
$runNonce = [Guid]::NewGuid().ToString('N')
$events = Join-Path $resultsDirectory (
    "gen3-pico-calibrated080-workspace100-events-$utcStamp-$runNonce.jsonl"
)
```

Resolve `$resultsDirectory` and the parent of `$events` with `[IO.Path]::GetFullPath`; require exact equality. Require the event path to be absent. Never invoke deletion or truncation. Pass this same `$events` value in `--evidence-jsonl` and print it once before motion admission and once after the child exits.

- [ ] **Step 3: Apply only the approved workspace/report changes**

Point `$report` to the Task 1 report and replace only the six workspace CLI numbers. Preserve PICO 20-sample gate, `CALIBRATED-READY`, masked SecureString input, child-only `ProcessStartInfo.EnvironmentVariables['KINOVA_PASSWORD']`, removal immediately after child start and in `finally`, PYTHONPATH restoration, SecureString/process disposal, exact in-main `MOVE`, and final child exit-code propagation.

- [ ] **Step 4: Run the launcher GREEN gate without executing it**

Parse with the PowerShell AST and run the static token/ordering assertions. Separately evaluate the timestamp-plus-GUID expression twice in a harmless PowerShell probe and require two distinct, filesystem-safe leaf names under the exact `results` directory. Confirm neither path is created by the probe. Confirm the old event file SHA-256/metadata are unchanged and the launcher is unstaged.

- [ ] **Step 5: Obtain a task-scoped correctness/security review**

Review dynamic path confinement, collision resistance, old-log preservation, secret lifecycle, admission order, exact workspace tokens, forbidden flags, exit propagation, and evidence-path printing. Resolve every Critical/Important finding via RED/GREEN. Do not run or commit the launcher.

---

### Task 3: Complete offline verification and hand off the onsite command

**Files:**
- Verify: `results/gen3-pico-calibrated080-workspace100-reviewed-preflight-20260813.json`
- Verify: `results/start-gen3-pico-calibrated080-workspace100-20260813.ps1`
- Report: `.superpowers/sdd/2026-08-13-gen3-recentered-workspace100/task-3-report.md`

**Interfaces:**
- Consumes: Task 1 and Task 2 approved artifacts.
- Produces: a static PASS/FAIL package verdict and a single guarded onsite command; it does not produce hardware validation.

- [ ] **Step 1: Run focused adjacent regression tests**

Run:

```powershell
python -m pytest tests/test_cli.py tests/test_cli_kortex.py tests/test_operator_calibration.py tests/test_hardware_admission.py tests/test_kortex_backend.py tests/test_preflight.py tests/test_motion_lease.py -q -p no:cacheprovider
python -m compileall -q kinova_teleop tests
python -m kinova_teleop.main --dry-run --headless --steps 2000
```

Require zero failures and `finite_state=true`. A Windows symlink-capability skip may remain only if the independent reparse test runs and passes.

- [ ] **Step 2: Run the complete package static gate**

Using the field Python 3.11.15 environment, strictly load the new preflight, calibration, and motion lease. Verify all hashes, exact workspace arithmetic, PowerShell AST, required/forbidden tokens, dynamic event path confinement, old evidence hash/metadata, tracked-clean status, unstaged artifacts, and `git diff --check`. Scan source/launcher for real secret-like values without printing environment secret values.

- [ ] **Step 3: Obtain final independent correctness/security review**

Review both artifacts and all task reports. Require no Critical/High/Medium findings. Correct package defects through one scoped fix/re-review cycle; do not change production code or broaden scope.

- [ ] **Step 4: Preserve the workspace and present the guarded run command**

Do not merge, push, delete reports, clean `results`, or remove the worktree. Tell the onsite operator that the new bounds are hardware-unverified. Require current physical checklist confirmation and released Grip before presenting:

```powershell
Set-Location (git rev-parse --show-toplevel)
& .\results\start-gen3-pico-calibrated080-workspace100-20260813.ps1
```

At exact `MOVE`, test one direction at a time with only a few millimetres of hand motion and release Grip immediately on any mismatch. After exit, use the printed unique evidence path for the next diagnosis; rerunning the same launcher must generate a different path automatically.
