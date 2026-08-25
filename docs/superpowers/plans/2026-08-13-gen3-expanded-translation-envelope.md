# Gen3 Expanded Translation Envelope Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an explicit, fail-closed Kortex translation-only mode that permits a 50 mm per-axis Grip anchor envelope and a 100 mm per-axis absolute workspace span without changing existing defaults.

**Architecture:** Keep `FIRST_HARDWARE_PROFILE` unchanged and add one immutable expanded translation-only axis limit. Resolve that limit from an explicit CLI flag, use the same resolved tuple for pre-connect workspace-span validation and runtime `AnchorEnvelope`, and reject every invalid flag combination before credentials, SDK import, `MOVE`, or connection.

**Tech Stack:** Python 3.11, argparse, NumPy, pytest, Coverage.py, PowerShell, Kortex API 2.8.0.post5, protobuf 3.20.0.

## Global Constraints

- The expanded limit is exactly `(0.05, 0.05, 0.05)` metres per Grip anchor axis.
- The expanded absolute workspace span is at most `(0.10, 0.10, 0.10)` metres.
- The option is valid only for explicit Kortex hardware translation-only teleoperation.
- Existing full-pose and ordinary translation-only modes retain `(0.02, 0.02, 0.02)` metres and a 40 mm maximum span.
- Translation scale remains at most `0.5` only in translation-only mode.
- Maximum linear speed remains `0.005 m/s`; maximum angular speed remains `2 deg/s`; control rate remains at most 40 Hz; stale timeout remains at most 0.2 seconds.
- Exact `MOVE`, Grip release admission, source continuity, watchdog, motion lease, preflight report, latched faults, Stop, and cleanup remain mandatory.
- Gripper writes, Home, action playback, low-level cyclic control, clamping, automatic recovery, and watchdog/freshness bypass remain prohibited.
- The approved workspace center is `[0.710730433, -0.054256245, 0.029749740]` metres.
- The approved bounds are minimum `[0.660730433, -0.104256245, -0.020250260]` and maximum `[0.760730433, -0.004256245, 0.079749740]` metres.

---

### Task 1: Define and resolve the expanded envelope profile

**Files:**
- Modify: `kinova_teleop/hardware_profile.py`
- Modify: `kinova_teleop/main.py`
- Test: `tests/test_hardware_profile.py`

**Interfaces:**
- Produces: `EXPANDED_TRANSLATION_ONLY_ANCHOR_AXIS_M: tuple[float, float, float]`.
- Produces: `resolve_anchor_translation_axis(args: argparse.Namespace) -> tuple[float, float, float]`.
- Changes: `validate_workspace_span(limits: WorkspaceLimits, maximum_translation_axis_m: tuple[float, float, float] = FIRST_HARDWARE_PROFILE.anchor_translation_axis_m) -> None`.
- Later tasks consume the resolver in CLI validation and backend construction.

- [ ] **Step 1: Write failing profile and resolver tests**

Add imports and tests equivalent to:

```python
from kinova_teleop.hardware_profile import (
    EXPANDED_TRANSLATION_ONLY_ANCHOR_AXIS_M,
)
from kinova_teleop.main import resolve_anchor_translation_axis


def test_expanded_translation_envelope_is_explicit_and_defaults_stay_small() -> None:
    parser = build_parser()
    default = parser.parse_args(["--backend", "kortex"])
    expanded = parser.parse_args([
        "--backend", "kortex",
        "--enable-hardware",
        "--translation-only",
        "--expanded-translation-envelope",
    ])
    assert EXPANDED_TRANSLATION_ONLY_ANCHOR_AXIS_M == (0.05, 0.05, 0.05)
    assert resolve_anchor_translation_axis(default) == (0.02, 0.02, 0.02)
    assert resolve_anchor_translation_axis(expanded) == (0.05, 0.05, 0.05)


def test_expanded_workspace_accepts_exact_ten_centimetres_only() -> None:
    exact = WorkspaceLimits((0.0, 0.0, 0.0), (0.10, 0.10, 0.10))
    validate_workspace_span(exact, EXPANDED_TRANSLATION_ONLY_ANCHOR_AXIS_M)
    over = WorkspaceLimits((0.0, 0.0, 0.0), (0.100001, 0.10, 0.10))
    with pytest.raises(ValueError, match="span must not exceed 0.1 m"):
        validate_workspace_span(over, EXPANDED_TRANSLATION_ONLY_ANCHOR_AXIS_M)
```

Retain the existing assertion that the first-hardware anchor tuple is `(0.02, 0.02, 0.02)`.
Update the existing default-span test to call `validate_workspace_span(limits)`
without passing the former `HardwareProfile` positional argument; the default
argument is now the unchanged first-hardware anchor tuple.

- [ ] **Step 2: Run the tests and witness RED**

Run:

```powershell
python -m pytest tests/test_hardware_profile.py -q -p no:cacheprovider
```

Expected: collection or parsing fails because the constant, flag, resolver, and new validator signature do not exist.

- [ ] **Step 3: Implement the immutable limit, parser flag, resolver, and pure span validator**

In `hardware_profile.py`, add:

```python
EXPANDED_TRANSLATION_ONLY_ANCHOR_AXIS_M = (0.05, 0.05, 0.05)
```

Change `validate_workspace_span` to accept the axis tuple, validate that it contains three positive finite values, compare the workspace span with twice that tuple, and report `0.04 m` for the default tuple or `0.1 m` for the expanded tuple. Do not mutate the supplied tuple or `WorkspaceLimits`.

In `build_parser()`, add:

```python
parser.add_argument(
    "--expanded-translation-envelope",
    action="store_true",
    help=(
        "permit a 50 mm per-axis Grip anchor envelope only for explicit "
        "Kortex translation-only hardware teleoperation"
    ),
)
```

Add the resolver:

```python
def resolve_anchor_translation_axis(
    args: argparse.Namespace,
) -> tuple[float, float, float]:
    if args.expanded_translation_envelope:
        return EXPANDED_TRANSLATION_ONLY_ANCHOR_AXIS_M
    return FIRST_HARDWARE_PROFILE.anchor_translation_axis_m
```

- [ ] **Step 4: Run profile tests and witness GREEN**

Run:

```powershell
python -m pytest tests/test_hardware_profile.py -q -p no:cacheprovider
```

Expected: all profile tests pass, including exact 100 mm acceptance and overrun rejection.

- [ ] **Step 5: Review and commit Task 1**

Run `git diff --check`, inspect only the three owned files, then commit:

```powershell
git add kinova_teleop/hardware_profile.py kinova_teleop/main.py tests/test_hardware_profile.py
git commit -m "feat: define expanded translation envelope"
```

---

### Task 2: Gate invalid combinations and wire one resolved limit end to end

**Files:**
- Modify: `kinova_teleop/main.py`
- Modify: `tests/test_cli_kortex.py`
- Test: `tests/test_workspace.py`

**Interfaces:**
- Consumes: `resolve_anchor_translation_axis(args)` from Task 1.
- Produces: pre-connect admission for `--expanded-translation-envelope` only in Kortex translation-only hardware teleoperation.
- Produces: backend construction with `AnchorEnvelope(resolve_anchor_translation_axis(args), math.radians(FIRST_HARDWARE_PROFILE.anchor_rotation_deg))`.

- [ ] **Step 1: Write failing pre-connect and wiring tests**

Add a dedicated exact-span argument helper so `_motion_gate_args`, which uses the default 40 mm span, remains unchanged:

```python
def _expanded_motion_gate_args(arguments: list[str]) -> list[str]:
    return arguments + [
        "--workspace-min", "0", "0", "0",
        "--workspace-max", "0.10", "0.10", "0.10",
        "--motion-lease", "fixture-motion.lock",
        "--preflight-report", "fixture-preflight.json",
    ]
```

Add tests that use `_install_unreachable_connection_factory` to prove that each invalid combination returns exit code 2 with zero connection attempts:

```python
@pytest.mark.parametrize("arguments", [
    ["--expanded-translation-envelope"],
    ["--backend", "kortex", "--enable-hardware", "--expanded-translation-envelope"],
    ["--backend", "kortex", "--enable-hardware", "--translation-only",
     "--check-kortex", "--expanded-translation-envelope"],
    ["--backend", "kortex", "--enable-hardware", "--translation-only",
     "--input", "none", "--fixed-trajectory", "fixture-trajectory.json",
     "--expanded-translation-envelope"],
])
def test_expanded_envelope_invalid_modes_reject_before_connection(
    monkeypatch, capsys, arguments: list[str]
) -> None:
    calls = _install_unreachable_connection_factory(monkeypatch)
    monkeypatch.setenv("KINOVA_PASSWORD", "secret")
    assert main(_motion_gate_args(arguments)) == 2
    assert calls == []
    assert "--expanded-translation-envelope requires" in capsys.readouterr().err
```

Add an overrun test using `0.100001` on one axis and the full valid Kortex translation-only flag set. It must return 2 before the connection factory, password prompt, SDK runtime gate, or `MOVE` callback can run.

Add a positive fake-backend test:

```python
def test_expanded_translation_only_wires_five_centimetre_anchor(monkeypatch) -> None:
    created: dict[str, object] = {}
    _install_valid_kortex_fakes(monkeypatch, created)
    monkeypatch.setattr("kinova_teleop.main.SdkXrInput", _FakeSource)
    argv = _expanded_motion_gate_args([
        "--backend", "kortex",
        "--enable-hardware",
        "--input", "xrobotoolkit",
        "--translation-only",
        "--expanded-translation-envelope",
        "--scale", "0.5",
    ])
    assert main(argv) == 0
    envelope = created["backend"].kwargs["anchor_envelope"]
    assert envelope.maximum_translation_axis_m == (0.05, 0.05, 0.05)
    assert created["controller_config"].orientation_enabled is False
    assert created["controller_config"].translation_scale == 0.5
```

Retain or add a normal translation-only test asserting its backend envelope remains `(0.02, 0.02, 0.02)`.

In `tests/test_workspace.py`, add a boundary test establishing that a 50 mm target is inclusive and a representable overrun is rejected.

- [ ] **Step 2: Run focused tests and witness RED**

Run:

```powershell
python -m pytest tests/test_cli_kortex.py tests/test_workspace.py -q -p no:cacheprovider
```

Expected: expanded combinations are not yet rejected, exact 100 mm is rejected by the old validator call, and the backend still receives `(0.02, 0.02, 0.02)`.

- [ ] **Step 3: Implement fail-closed mode validation and shared wiring**

In `_validate_args`, before any runtime or hardware setup, reject the expanded flag unless all of these hold:

```python
expanded_mode_valid = (
    args.backend == "kortex"
    and args.enable_hardware
    and args.translation_only
    and not args.check_kortex
    and args.fixed_trajectory is None
)
if args.expanded_translation_envelope and not expanded_mode_valid:
    return (
        "--expanded-translation-envelope requires Kortex "
        "translation-only hardware teleoperation"
    )
```

In `_validate_kortex_args`, change the span gate to:

```python
validate_workspace_span(limits, resolve_anchor_translation_axis(args))
```

In Kortex backend construction, change the anchor envelope to:

```python
anchor_envelope=AnchorEnvelope(
    resolve_anchor_translation_axis(args),
    math.radians(FIRST_HARDWARE_PROFILE.anchor_rotation_deg),
)
```

Do not change scale, speed, stale, admission, connection ordering, watchdog, workspace evaluation, Stop, or cleanup code.

- [ ] **Step 4: Run focused and adjacent tests and witness GREEN**

Run:

```powershell
python -m pytest `
  tests/test_hardware_profile.py `
  tests/test_cli.py `
  tests/test_cli_kortex.py `
  tests/test_workspace.py `
  tests/test_kortex_backend.py `
  tests/test_hardware_admission.py `
  tests/test_pico_udp_input.py `
  -q -p no:cacheprovider
```

Expected: all tests pass; invalid expanded modes and spans create no Kortex connection; the positive path passes the 50 mm tuple to the backend.

- [ ] **Step 5: Review and commit Task 2**

Inspect the diff for any changed speed, stale, watchdog, Stop, gripper, Home, or low-level RPC behavior. Commit only the Task 2 files:

```powershell
git add kinova_teleop/main.py tests/test_cli_kortex.py tests/test_workspace.py
git commit -m "feat: gate expanded translation workspace"
```

---

### Task 3: Run complete offline verification and independent reviews

**Files:**
- Review: all files changed after commit `a1abde9`
- Do not modify unrelated tracked or untracked files.

**Interfaces:**
- Consumes: completed Task 1 and Task 2 commits.
- Produces: evidence that the implementation is ready for a guarded onsite trial, not evidence of successful hardware motion.

- [ ] **Step 1: Run the complete offline suite**

```powershell
python -m pytest -q -p no:cacheprovider
```

Expected: all tests pass with no skips introduced by this feature.

- [ ] **Step 2: Run coverage and enforce at least 80 percent**

Use pytest-cov when installed:

```powershell
python -m pytest --cov=kinova_teleop --cov-report=term-missing --cov-fail-under=80 -q -p no:cacheprovider
```

If the offline environment lacks pytest-cov, use the already installed Coverage.py without installing from the network:

```powershell
python -m coverage erase
python -m coverage run -m pytest -q -p no:cacheprovider
python -m coverage report --fail-under=80
```

Expected: total coverage is at least 80 percent.

- [ ] **Step 3: Run the finite simulator and static checks**

```powershell
python -m kinova_teleop.main --dry-run --headless --steps 2000
python -m compileall -q kinova_teleop tests
git diff --check a1abde9..HEAD
```

Expected: `completed steps=2000 finite_state=true`; compile and diff checks pass.

- [ ] **Step 4: Run correctness and security reviews**

Give independent reviewers the committed diff `a1abde9..HEAD`. Require them to verify:

- the expanded mode is explicit and translation-only;
- all invalid modes and spans reject before credential lookup, SDK import, `MOVE`, and connection;
- pre-connect and runtime limits come from the same resolver;
- defaults, scale, speed, stale timeout, watchdog, Stop, cleanup, and no-gripper rules remain unchanged;
- no secrets, device identifiers, credentials, or broad firewall changes appear in tracked files.

Resolve every Critical, High, or Medium finding through a new RED/GREEN cycle and conventional commit. Re-run Steps 1 through 3 after any production change.

---

### Task 4: Build and validate the current-session onsite package

**Files:**
- Create, ignored/untracked: `results/gen3-pico-expanded050-reviewed-preflight-20260813.json`
- Create, ignored/untracked: `results/gen3-expanded050-motion-20260813.lock`
- Create, ignored/untracked: `results/start-gen3-pico-expanded050-20260813.ps1`
- Reserve, do not pre-create: `results/gen3-pico-expanded050-events-20260813.jsonl`

**Interfaces:**
- Consumes: clean final implementation `HEAD`, current mapping SHA256, the approved current pose and bounds, run ID `gen3-first-hardware`, and owner `kinova-teleop`.
- Produces: a strict passing report, a fresh current-session motion lease, a guarded launcher, and a unique unused evidence path.

- [ ] **Step 1: Capture clean immutable inputs**

Run read-only checks:

```powershell
git status --short --untracked-files=no
git rev-parse HEAD
Get-FileHash -Algorithm SHA256 .\kinova_teleop\pose_mapping.py
```

Expected: tracked status is empty. Record the exact full commit hash and lowercase mapping SHA256 in the report. Do not use `a1abde9`, because the report must bind the final implementation revision.

- [ ] **Step 2: Create and validate a fresh motion lease**

Use a newly generated 32-character lowercase hexadecimal UUID with hyphens removed and the current UTC timestamp ending in `Z`. Create the lease through `apply_patch` with exactly:

```json
{
  "lease_id": "the newly generated 32-character lowercase hexadecimal value",
  "device": "gen3",
  "run_id": "gen3-first-hardware",
  "owner": "kinova-teleop",
  "acquired_utc": "the current UTC ISO 8601 timestamp ending in Z"
}
```

Immediately validate it with:

```python
validate_motion_lease(
    "results/gen3-expanded050-motion-20260813.lock",
    "gen3-first-hardware",
    "kinova-teleop",
)
```

The quoted descriptions above are generation instructions, not literal file values.

- [ ] **Step 3: Create the strict report with exact current-session facts**

Use `apply_patch`. Copy the existing strict schema shape and required passing checks, but bind it to final clean `HEAD` and include these exact safety values:

```json
{
  "workspace_center_m": [0.710730433, -0.054256245, 0.029749740],
  "workspace_min_m": [0.660730433, -0.104256245, -0.020250260],
  "workspace_max_m": [0.760730433, -0.004256245, 0.079749740],
  "control_hz": 40.0,
  "translation_scale": 0.5,
  "translation_only": true,
  "expanded_translation_envelope": true,
  "stale_timeout_s": 0.2,
  "max_linear_speed_m_s": 0.005,
  "max_angular_speed_deg_s": 2.0,
  "anchor_translation_axis_m": 0.05,
  "anchor_rotation_deg": 5.0
}
```

State that the expanded implementation has only offline validation. Record the operator's explicit physical confirmation for the complete 100 mm cube, including the negative Z lower bound, links, tool, gripper, payload, cables, fixture, E-stop, Web Stop, second observer, and released Grip. Do not claim expanded hardware motion has succeeded.

- [ ] **Step 4: Create the guarded PowerShell launcher**

Use `apply_patch` and preserve the established launcher sequence:

```powershell
& $python -m kinova_teleop.main `
    --input pico-udp --check-input --samples 20 --check-timeout 15 `
    --control-hz 40 --stale-timeout 0.2
```

Only after that gate succeeds, prompt with `Read-Host -AsSecureString`, set `KINOVA_PASSWORD` only for the child process, and run:

```powershell
& $python -m kinova_teleop.main `
    --backend kortex --enable-hardware `
    --input pico-udp `
    --robot-ip 192.168.1.10 --robot-user admin `
    --control-hz 40 --scale 0.5 --translation-only `
    --expanded-translation-envelope --stale-timeout 0.2 `
    --max-linear-speed 0.005 --max-angular-speed-deg 2 `
    --workspace-min 0.660730433 -0.104256245 -0.020250260 `
    --workspace-max 0.760730433 -0.004256245 0.079749740 `
    --motion-lease $lease `
    --preflight-report $report `
    --evidence-jsonl $events
```

In `finally`, remove `KINOVA_PASSWORD`, restore `PYTHONPATH`, and dispose the `SecureString`. Preserve the child exit code.

- [ ] **Step 5: Validate the package without connecting hardware**

Verify all of the following with file reads, the PowerShell AST parser, strict report loader, motion lease validator, and CLI validation with password checking disabled:

- report revision equals clean `HEAD`;
- calibration hash equals the current mapping hash;
- center-to-minimum and maximum-to-center equal exactly `0.05` metres on every axis;
- launcher and report both specify scale `0.5`, translation-only, and the explicit expanded flag;
- launcher specifies 40 Hz, 0.2 seconds, `0.005 m/s`, and `2 deg/s`;
- launcher references the fresh lease and a unique evidence path that does not exist;
- PowerShell has zero parse errors;
- password cleanup is in `finally` and no password value is stored or printed;
- strict report and lease loaders accept the files;
- `_validate_args` and `_validate_kortex_args(..., check_password=False)` accept the exact launcher arguments.

Do not run the launcher, probe Kortex motion transport, construct a Kortex connection, or create the evidence file during this step.

- [ ] **Step 6: Present the guarded onsite handoff**

Before giving the launch command, require the onsite operator to reconfirm that the physical conditions have not changed since the report, Grip is released, and PICO Bridge is foreground. During the first expanded run, move one axis slowly and release Grip after a clearly visible 3 to 5 mm robot displacement. Treat wrong direction, stale input, workspace fault, Stop-unconfirmed, or cleanup failure as a stop condition; do not automatically relaunch.

Keep `results/` artifacts untracked unless the repository's existing ignore policy changes through a separately approved design.
