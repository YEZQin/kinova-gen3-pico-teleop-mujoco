# Gen3 Translation-Only Scale 0.5 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Permit an explicitly selected Kortex translation-only scale of 0.5 while retaining the 0.25 default/full-pose limit and every existing motion safety gate.

**Architecture:** Add one immutable translation-only scale ceiling beside the existing first-hardware profile. Select the ceiling during pre-connect CLI validation based only on `args.translation_only`; no mapping, backend, watchdog, workspace, or Stop code changes. Generate a separate onsite launcher and reviewed report after the code commit so the report revision is exact.

**Tech Stack:** Python 3.11, pytest, NumPy, PowerShell, Kortex API 2.8.0.post5.

## Global Constraints

- Kortex default and full-pose maximum translation scale remain `0.25`.
- Explicit translation-only maximum scale is `0.5`; larger values reject before password lookup, confirmation, SDK import, or robot connection.
- Translation-only orientation stays fixed to the Grip anchor.
- Control rate remains 40 Hz, stale timeout 0.2 seconds, linear speed at most `0.005 m/s`, and angular speed at most `2 deg/s`.
- Absolute onsite workspace is the latest measured tool pose plus or minus 10 mm on X, Y, and Z.
- Exact `MOVE`, Grip release/admission, source continuity, freshness, watchdog, workspace/anchor envelopes, Stop, and cleanup remain unchanged.

---

### Task 1: Scope scale 0.5 to translation-only Kortex control

**Files:**
- Modify: `kinova_teleop/hardware_profile.py`
- Modify: `kinova_teleop/main.py`
- Test: `tests/test_hardware_profile.py`
- Test: `tests/test_cli_kortex.py`

**Interfaces:**
- Consumes: `argparse.Namespace.translation_only`, `resolve_translation_scale(args)`.
- Produces: `MAX_TRANSLATION_ONLY_SCALE: float = 0.5` and pre-connect validation that chooses `0.5` only for translation-only mode.

- [ ] **Step 1: Write the failing boundary tests**

Add a profile assertion:

```python
assert MAX_TRANSLATION_ONLY_SCALE == 0.5
```

Add a valid full CLI test using existing Kortex fakes:

```python
assert main(_motion_gate_args([
    "--backend", "kortex", "--enable-hardware",
    "--input", "xrobotoolkit", "--translation-only", "--scale", "0.5",
])) == 0
assert created["controller_config"].translation_scale == 0.5
assert created["controller_config"].orientation_enabled is False
```

Retain the existing full-pose `0.26` rejection and add translation-only
`0.500001` to the pre-connect rejection parameter table.

- [ ] **Step 2: Run the tests to verify RED**

Run:

```powershell
python -m pytest -q tests/test_hardware_profile.py tests/test_cli_kortex.py -k "scale or profile"
```

Expected: the new 0.5 translation-only acceptance test fails with the existing
`--scale must not exceed 0.25` error, and the new constant import initially
fails until the constant is added.

- [ ] **Step 3: Implement the minimal scoped ceiling**

In `hardware_profile.py` add:

```python
MAX_TRANSLATION_ONLY_SCALE = 0.5
```

Import it in `main.py` and replace the single ceiling check with:

```python
maximum_scale = (
    MAX_TRANSLATION_ONLY_SCALE
    if args.translation_only
    else FIRST_HARDWARE_PROFILE.translation_scale
)
if resolve_translation_scale(args) > maximum_scale:
    return f"--scale must not exceed {maximum_scale:g} for --backend kortex"
```

- [ ] **Step 4: Run focused and full offline verification**

Run:

```powershell
python -m pytest -q tests/test_hardware_profile.py tests/test_cli_kortex.py tests/test_pose_mapping.py tests/test_teleop_controller.py tests/test_kortex_backend.py
python -m pytest -q
python -m kinova_teleop.main --dry-run --headless --steps 2000 --translation-only --scale 0.5
git diff --check
```

Expected: all tests pass and the dry run prints `finite_state=true`.

- [ ] **Step 5: Review and commit**

Review that no Kortex backend write path or safety gate changed. Stage only the
four code/test files and commit:

```powershell
git commit -m "feat: allow scale-half translation teleop"
```

### Task 2: Build the pose-centered onsite launch package

**Files:**
- Create: `results/gen3-pico-scale050-reviewed-preflight-20260812.json`
- Create: `results/start-gen3-pico-scale050-20260812.ps1`
- Runtime output: `results/gen3-pico-scale050-events-20260812.jsonl`

**Interfaces:**
- Consumes: the Task 1 commit hash, latest read-only pose, existing motion lease, and strict preflight schema.
- Produces: one guarded translation-only scale-0.5 launcher with a unique evidence path.

- [ ] **Step 1: Generate exact bounds and report**

Use center `[0.711899340, -0.052683271, 0.032770045]`, minimum
`[0.701899340, -0.062683271, 0.022770045]`, and maximum
`[0.721899340, -0.042683271, 0.042770045]`. Record scale `0.5`,
translation-only mode, 40 Hz, 0.2-second stale timeout, `0.005 m/s`, and
`2 deg/s`. Bind `code_revision` to the exact clean Task 1 commit.

- [ ] **Step 2: Generate the guarded PowerShell launcher**

The launcher must run a 20-sample PICO-only gate, prompt with
`Read-Host -AsSecureString`, set `KINOVA_PASSWORD` only around the Python child,
invoke Kortex with `--translation-only --scale 0.5`, the exact bounds, reviewed
report, lease, and unused evidence path, then remove the environment variable
and dispose the secure string in `finally`.

- [ ] **Step 3: Validate without robot access**

Strict-load the report, prove its revision equals `git rev-parse HEAD`, prove
the tracked worktree is clean, parse the PowerShell AST, verify the evidence
path does not exist, and call `_validate_args` plus `_validate_kortex_args`
with password checking disabled.

- [ ] **Step 4: Perform the physical gate and guarded trial**

Before presenting the launcher, require explicit onsite confirmation that the
arm is stationary, the entire XYZ plus-or-minus 10 mm sweep is clear, E-stop is
reachable, the second observer is present, and Grip is released. The operator
then types exact `MOVE`, tests one axis with approximately 1 mm of controller
motion, releases Grip, and reports physical stopping plus terminal output.
