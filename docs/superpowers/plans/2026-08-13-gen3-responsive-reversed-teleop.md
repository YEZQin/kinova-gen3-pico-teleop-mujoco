# Gen3 Responsive Reversed Translation Teleoperation Implementation Plan

**Goal:** Deliver an explicit Kortex translation-only profile with reversed
XYZ hand motion, scale `0.8`, `0.01 m/s` maximum linear speed, and safe
stale-input re-arming while preserving every existing default and guardrail.

## Task 1: Translation signs and release-stability mapping

**Files:** `kinova_teleop/pose_mapping.py`,
`kinova_teleop/teleop_controller.py`, `tests/test_pose_mapping.py`.

- [ ] Write RED tests using hand-derived axis expectations. Prove default
  mapping is unchanged, `(-1,-1,-1)` reverses each relative mapped base axis,
  malformed signs reject, and three distinct released timestamps are needed
  after reset/stale before `READY`.
- [ ] Add immutable mapping/controller configuration for translation axis
  signs and release sample count. Validate shape, finite values, exact signs,
  and positive integer count.
- [ ] Apply signs only to the relative translation delta. Keep quaternion and
  default `PICO_TO_WORLD` behavior unchanged.
- [ ] Run focused mapping/controller tests GREEN.

## Task 2: Recoverable stale Stop and explicit responsive CLI

**Files:** `kinova_teleop/teleop_controller.py`,
`kinova_teleop/hardware_profile.py`, `kinova_teleop/main.py`,
`tests/test_teleop_controller.py`, `tests/test_hardware_profile.py`,
`tests/test_cli_kortex.py`.

- [ ] Write RED controller tests proving stale calls `hold()` but does not
  raise in explicit recovery mode, repeated/pressed frames cannot re-arm,
  and a new Grip press obtains a fresh anchor. Retain fatal invalid,
  source-change, and unconfirmed-Stop tests.
- [ ] Write RED CLI tests proving the responsive/inverted flags are valid only
  for expanded live Kortex translation-only hardware mode, accept exact scale
  `0.8` and speed `0.01`, reject overruns before side effects, and wire the
  approved mapping/recovery values into `TeleopConfig`.
- [ ] Implement the minimal explicit profile. Keep old scale/speed caps when
  the responsive flag is absent; keep stale timeout at 0.2 seconds.
- [ ] Add supported stale evidence handling without allowing evidence errors
  to alter Stop order.
- [ ] Run focused and adjacent suites GREEN.

## Task 3: Complete offline verification and review

- [ ] Run full pytest suite with no new skips.
- [ ] Run Coverage.py/pytest-cov and require at least 80 percent.
- [ ] Run `compileall`, `git diff --check`, and the 2000-step finite MuJoCo
  dry run.
- [ ] Inspect the diff for secrets, credentials, identifiers, new Kortex RPCs,
  watchdog/freshness bypass, workspace removal, and changed defaults.
- [ ] Obtain independent correctness and security reviews. Resolve every
  Critical/High/Medium finding through another RED/GREEN cycle.
- [ ] Commit the clean implementation with conventional commits.

## Task 4: Build the guarded onsite package

- [ ] Create a strict reviewed preflight report bound to clean final `HEAD`,
  current mapping hash, exact 100 mm workspace, scale `0.8`, translation-only,
  reversed XYZ, responsive mode, 40 Hz, 0.2 second freshness, `0.01 m/s`, and
  2 deg/s.
- [ ] Create a fresh lease and a PowerShell launcher with the existing PICO
  gate, physical reconfirmation, child-only password environment, exact
  in-main `MOVE`, new unused evidence path, and `finally` cleanup.
- [ ] Validate report, lease, CLI tokens, PowerShell AST, clean revision, hash,
  bounds, and absent evidence path without connecting to PICO or Kortex.
- [ ] Present the launcher only after the onsite operator reconfirms unchanged
  physical conditions. First validate all three directions separately at a
  few millimetres, then validate Grip-release and transient-stale recovery.
