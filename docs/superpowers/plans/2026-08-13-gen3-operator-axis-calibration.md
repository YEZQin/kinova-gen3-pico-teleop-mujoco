# Gen3 Operator-Axis Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the failed all-axis inversion guess with a strict, onsite-measured PICO operator-axis calibration while preserving every existing Gen3 safety gate and default mapping.

**Architecture:** A focused loader validates the PICO-only capture and derives one immutable proper rotation. The CLI loads that rotation before any password, SDK, connection, or `MOVE` side effect and passes it through `TeleopConfig` into relative translation mapping. Existing default and explicit inversion paths remain unchanged; calibration and inversion are mutually exclusive.

**Tech Stack:** Python 3.10+, NumPy SVD/polar decomposition, frozen dataclasses, argparse, pytest, Kortex fake boundary tests.

## Global Constraints

- Use capture `results/pico-operator-axes-20260813-051504.json`; do not collect another calibration unless strict loading rejects it.
- Map operator right/up/forward to Kinova base `-Y/+Z/-X`.
- Keep scale `0.8`, maximum linear speed `0.01 m/s`, control `40 Hz`, stale timeout `0.2 s`, and translation-only orientation hold.
- Keep the absolute workspace and per-Grip anchor envelope; do not enlarge, clamp, or bypass them.
- Load and validate calibration before password lookup, SDK import, `MOVE`, connection, or robot RPC.
- Keep exact `MOVE`, Grip release-first admission, source lock, stale Stop/re-arm, watchdog, motion lease, reviewed preflight, Stop confirmation, and cleanup unchanged.
- Do not add gripper, Home, action playback, low-level cyclic control, dependencies, or network access.

---

### Task 1: Strict calibration artifact loader

**Files:**
- Create: `kinova_teleop/operator_calibration.py`
- Create: `tests/test_operator_calibration.py`

**Interfaces:**
- Consumes: capture JSON with `schema_version`, `controller`, `grip_released`, `gestures`, and `raw_operator_basis_columns_right_up_forward`.
- Produces: `OperatorAxisCalibration` and `load_operator_axis_calibration(path: str | Path) -> OperatorAxisCalibration`; `translation_rotation` is an immutable 3-by-3 tuple in the already transformed controller basis.

- [ ] **Step 1: Write the valid-capture RED test**

```python
def test_loader_derives_expected_proper_rotation(tmp_path: Path) -> None:
    path = write_capture(tmp_path, RIGHT_UP_FORWARD_CAPTURE)
    loaded = load_operator_axis_calibration(path)
    np.testing.assert_allclose(
        loaded.translation_rotation,
        [[0.861288411785, 0.469277688168, -0.194835117758],
         [-0.470679958298, 0.881303368425, 0.042008923554],
         [0.191422696095, 0.055523186053, 0.979935981190]],
        atol=1e-9,
    )
    np.testing.assert_allclose(
        np.asarray(loaded.translation_rotation).T
        @ np.asarray(loaded.translation_rotation),
        np.eye(3), atol=1e-12,
    )
```

- [ ] **Step 2: Run the test and witness RED**

Run: `python -m pytest tests/test_operator_calibration.py::test_loader_derives_expected_proper_rotation -q -p no:cacheprovider`

Expected: FAIL because `kinova_teleop.operator_calibration` does not exist.

- [ ] **Step 3: Add strict malformed/unsafe artifact RED cases**

Parameterize rejection for duplicate JSON keys, non-finite constants, extra or missing top-level fields, wrong controller, pressed Grip, labels not exactly `right/up/forward`, any gesture below `0.015 m`, matrix shape other than 3-by-3, max absolute pair dot above `0.35`, condition number above `2.0`, determinant at or below zero, symlink, and Windows reparse file.

```python
with pytest.raises(ValueError, match="operator calibration"):
    load_operator_axis_calibration(path)
```

- [ ] **Step 4: Implement the minimal immutable loader**

```python
@dataclass(frozen=True, slots=True)
class OperatorAxisCalibration:
    translation_rotation: tuple[tuple[float, float, float], ...]
    source_sha256: str

def load_operator_axis_calibration(path: str | Path) -> OperatorAxisCalibration:
    payload, raw = _load_strict_regular_json(Path(path))
    basis = _validate_capture(payload)
    u, singular_values, vt = np.linalg.svd(basis)
    orthogonal_basis = u @ vt
    rotation = PICO_TO_WORLD @ orthogonal_basis.T @ PICO_TO_WORLD.T
    return OperatorAxisCalibration(
        tuple(tuple(float(value) for value in row) for row in rotation),
        hashlib.sha256(raw).hexdigest(),
    )
```

Recompute magnitudes, pair dots, determinant, and condition from vectors in the file; do not trust the stored summary metrics. Reject symbolic links/reparse points before reading and decode only strict UTF-8 JSON with duplicate-key and non-finite-constant rejection.

- [ ] **Step 5: Run focused tests GREEN and commit**

Run: `python -m pytest tests/test_operator_calibration.py -q -p no:cacheprovider`

Commit: `feat: load operator-axis calibration`

---

### Task 2: Apply calibrated translation without changing defaults

**Files:**
- Modify: `kinova_teleop/pose_mapping.py`
- Modify: `kinova_teleop/teleop_controller.py`
- Modify: `tests/test_pose_mapping.py`
- Modify: `tests/test_teleop_controller.py`

**Interfaces:**
- Consumes: `translation_rotation: tuple[tuple[float, float, float], ...] | None` from `TeleopConfig`.
- Produces: calibrated relative translation in `RelativePoseMapper`; default `None` and existing `invert_translation=True` semantics are unchanged.

- [ ] **Step 1: Write mapper RED tests for the three measured gestures**

```python
@pytest.mark.parametrize(
    ("raw_delta", "expected_base"),
    [(RIGHT_RAW, [0.0, -1.0, 0.0]),
     (UP_RAW, [0.0, 0.0, 1.0]),
     (FORWARD_RAW, [-1.0, 0.0, 0.0])],
)
def test_operator_calibration_maps_measured_directions(raw_delta, expected_base):
    # The mapper receives PICO_TO_WORLD @ raw_delta from transform_controller_pose.
    output = move_after_anchor(config_with_calibration, raw_delta)
    assert normalized(output.target.position - EE.position) == approx(expected_base, abs=0.11)
```

Also assert translation-only retains the non-identity anchor orientation, the first Grip press is anchor-only, default mapping still equals `PICO_TO_WORLD @ raw_delta`, and inversion still equals its negative.

- [ ] **Step 2: Run mapper tests and witness RED**

Run: `python -m pytest tests/test_pose_mapping.py -q -p no:cacheprovider`

Expected: the calibrated configuration/API is absent.

- [ ] **Step 3: Implement immutable validated config wiring**

Add the field to the end of both frozen dataclasses so positional compatibility remains intact:

```python
translation_rotation: tuple[tuple[float, float, float], ...] | None = None
```

In `MappingConfig.__post_init__`, require shape `(3, 3)`, finite values, `R.T @ R == I` within `1e-9`, and determinant `+1` within `1e-9`. Reject simultaneous `invert_translation` and `translation_rotation`.

Apply only to the relative translation delta:

```python
delta = controller_pose.position - self._controller_reference.position
if self.config.translation_rotation is not None:
    delta = np.asarray(self.config.translation_rotation) @ delta
desired_position = self._ee_reference.position + sign * scale * delta
```

- [ ] **Step 4: Pass the rotation through `TeleopController` and run GREEN**

Run: `python -m pytest tests/test_pose_mapping.py tests/test_teleop_controller.py -q -p no:cacheprovider`

Commit: `feat: apply operator-axis translation rotation`

---

### Task 3: Fail-closed CLI integration

**Files:**
- Modify: `kinova_teleop/main.py`
- Modify: `tests/test_cli.py`
- Modify: `tests/test_cli_kortex.py`

**Interfaces:**
- Consumes: `--operator-calibration PATH`.
- Produces: one loaded `OperatorAxisCalibration` retained in local runtime state and passed to `TeleopConfig.translation_rotation` without re-reading the file.

- [ ] **Step 1: Write CLI RED tests for scope and ordering**

Test exact calibrated responsive arguments and assert scale `0.8`, orientation disabled, recovery enabled, inversion false, and the expected rotation reaches the controller. Parameterize invalid cases: calibration without responsive profile; without expanded/translation-only/recovery; with `--invert-translation`; malformed or missing file.

For every invalid case, monkeypatch password lookup, SDK validation, input factory, `input()`, connection factory, and backend factory to raise if called. Assert exit code `2` and zero side effects.

- [ ] **Step 2: Run CLI tests and witness RED**

Run: `python -m pytest tests/test_cli_kortex.py -k 'operator_calibration or responsive' -q -p no:cacheprovider`

- [ ] **Step 3: Add parser and responsive-scope rules**

```python
parser.add_argument(
    "--operator-calibration", type=Path,
    help="validated PICO right/up/forward calibration for responsive translation",
)
```

Responsive Kortex motion requires `--responsive-translation-profile`,
`--recover-stale-input`, expanded translation-only mode, and exactly one of
`--invert-translation` or `--operator-calibration`.

- [ ] **Step 4: Load once before password lookup and wire the immutable value**

```python
calibration = (
    load_operator_axis_calibration(args.operator_calibration)
    if args.operator_calibration is not None else None
)
# Later, after all existing admissions:
TeleopConfig(
    ...,
    invert_translation=args.invert_translation,
    translation_rotation=(calibration.translation_rotation if calibration else None),
)
```

Convert loader errors to `error: ...` and exit `2`; never fall through to password or hardware setup.

- [ ] **Step 5: Run focused/adjacent tests GREEN and commit**

Run: `python -m pytest tests/test_operator_calibration.py tests/test_pose_mapping.py tests/test_teleop_controller.py tests/test_cli.py tests/test_cli_kortex.py tests/test_hardware_admission.py tests/test_pico_udp_input.py -q -p no:cacheprovider`

Commit: `feat: admit calibrated responsive teleoperation`

---

### Task 4: Verification, independent review, and guarded onsite package

**Files:**
- Create after implementation commit: `results/gen3-pico-calibrated080-reviewed-preflight-20260813.json`
- Create after implementation commit: `results/start-gen3-pico-calibrated080-20260813.ps1`
- Use without modifying: `results/pico-operator-axes-20260813-051504.json`

**Interfaces:**
- Consumes: clean implementation `HEAD`, strict calibration hash, existing motion lease, unchanged ±50 mm workspace, and a new unused evidence path.
- Produces: one PowerShell launcher that retains PICO gate, current physical reconfirmation, child-only password environment, exact in-main `MOVE`, and `finally` cleanup.

- [ ] **Step 1: Run complete offline verification**

Run:

```powershell
python -m pytest -q -p no:cacheprovider
coverage run -m pytest -q -p no:cacheprovider
coverage report --fail-under=80
python -m compileall -q kinova_teleop tests
python -m kinova_teleop.main --dry-run --headless --steps 2000
git diff --check
```

Require all tests green, coverage at least 80%, and `finite_state=true`.

- [ ] **Step 2: Review correctness and security**

Inspect the complete diff for changed defaults, extra Kortex RPCs, calibration double-application, reparse/JSON bypass, password or identifier leakage, TOCTOU re-read, source-handoff, freshness/watchdog bypass, workspace/anchor removal, and missing Stop/cleanup. Resolve every Critical/High/Medium finding with another RED/GREEN cycle.

- [ ] **Step 3: Commit clean implementation before creating evidence**

Stage only intended source/tests/docs and commit with a conventional message. Confirm `git status --short --untracked-files=no` is empty.

- [ ] **Step 4: Build and statically validate the onsite package**

Bind the reviewed report to exact clean `HEAD`, exact SHA-256 of the calibration JSON and mapping source, scale `0.8`, translation-only, calibrated mode, 40 Hz, 0.2 s, 0.01 m/s, 2 deg/s, and unchanged workspace/anchor limits. The launcher must use `--operator-calibration` and must not use `--invert-translation`. Require a unique absent evidence path and PowerShell AST parse with zero errors. Do not run the launcher during offline validation.

- [ ] **Step 5: Hardware direction trial**

After the onsite operator reconfirms the full physical checklist, start the reviewed launcher, type exact `MOVE`, and test right, left, up, down, forward, and backward separately with only a few millimetres of hand motion. Release Grip immediately on mismatch. Record Grip-release Stop and transient-stale Stop/re-arm. Hardware observation, not offline math, determines completion.
