# Gen3 PICO Kortex final fix report

Date: 2026-08-10

Branch: `feat/kortex-hardware-merge`

Fix commit: `154c5ba` (`fix: close Gen3 final safety gaps`)

Evidence level: offline only

## Outcome

The four load-bearing final-review findings are fixed and covered by witnessed
RED/GREEN regressions. The narrowly related direct-Python Windows reparse-point
checks for the reviewed preflight report and motion lease were also hardened.
No hardware, robot network, or Kortex SDK client was constructed or used.

The implementation preserves the reviewed architecture, mapper generation
rules, PICO session continuity, absolute and anchor-relative workspace gates,
watchdog behavior, and existing Kortex admission ordering.

## Findings and fixes

### 1. First Grip press is an anchor-only cycle

Root cause: `RelativePoseMapper` already exposed `MappingOutput.activated`, but
`TeleopController` treated every active mapping alike. It therefore called
`KortexBackend.command_pose()` immediately after the anchor feedback read. A
different feedback pose on that second read could be converted into a nonzero
compensating Twist.

Fix: `TeleopController.step_once()` now handles `mapping.activated` explicitly.
The activation cycle records the controller/end-effector anchor and performs
the ordinary backend step, but it does not call `command_pose()`. The first
possible pose command belongs to a later controller sample.

Regression: the Kortex integration fake changes end-effector feedback by 1 mm
between the anchor and the formerly vulnerable command-time read. Before the
fix, the first press emitted a Twist; after the fix, the activation cycle emits
no Twist and the later moved sample remains speed-capped.

### 2. Every Kortex cleanup stage contributes to success

Root cause: `KortexConnection._cleanup()` returned only the Stop result and
silently swallowed `CloseSession`, router deactivation/close, and TCP disconnect
failures. Consequently, `close(send_stop=False)` could return `True` for a
read-only T0 session even when all teardown stages failed.

Fix: cleanup now accumulates stable, secret-free stage labels in
`cleanup_failures`, continues attempting every later safe teardown stage, and
returns `False` if Stop is unconfirmed or any cleanup stage fails. Repeated
`close()` calls retain the aggregate result. Connection-establishment errors
also report the failed stage labels after password redaction. Existing CLI and
backend cleanup callers already treat an explicit `False` as fatal, so both T0
and motion cleanup paths fail closed.

Regression: the read-only fake raises from session close, router deactivation,
and transport disconnect in one run. The test proves all later stages were
attempted, the ordered stage tuple was retained, and close returned `False`.

### 3. Foreign PICO packets become a fatal source-change boundary

Root cause: while the locked PICO source was healthy, a packet from another IP
incremented the `foreign` counter but returned success from `_accept()`. The
adapter then emitted the cached valid sample, so the Kortex controller never
received the already-supported fatal `source_changed` input fault.

Fix: a healthy foreign packet still increments `foreign` and does not replace
or clear the locked source/session generation, but the current read now emits
an invalid `ControllerSample` with the stable reason `source changed`. Existing
mapper classification converts that reason to `InputFault.SOURCE_CHANGED`, and
the Kortex `fatal_input_faults` policy requests Stop and exits the loop.

Regression: a foreign packet arriving after a valid admitted sample produces
the invalid boundary, preserves the original `active_source`, and leaves the
old sample available only for diagnostics/release-gated non-hardware recovery.
The existing controller tests continue to prove `source_changed` is fatal for
Kortex and recoverable only under the MuJoCo-default policy.

### 4. Robot IPv4 validation is an explicit RFC1918 allowlist

Root cause: `ipaddress.is_private` includes several special-use ranges on the
tested Python runtime, including link-local, documentation, benchmarking, and
reserved addresses.

Fix: robot endpoints are accepted only when the parsed address is IPv4 and is a
member of exactly one of `10.0.0.0/8`, `172.16.0.0/12`, or `192.168.0.0/16`.
Tests cover both accepted RFC1918 boundaries and rejected public, loopback,
link-local, documentation, benchmarking, reserved/broadcast, IPv6, and hostname
inputs.

### 5. Direct Python report/lease reparse-point hardening

Assessment: this was narrowly safe in the already touched admission paths, so
it was implemented rather than deferred. Both the motion-lease and passing
preflight-report loaders now inspect `lstat().st_file_attributes` and reject
`FILE_ATTRIBUTE_REPARSE_POINT` in addition to ordinary symlinks. Tests inject
Windows regular-file metadata carrying the reparse flag and prove rejection.
The approved PowerShell launcher retains its independent reparse check.

## TDD evidence

No production file was edited until the regressions had been run and failed for
the expected missing behavior.

Initial focused RED run:

- Result: `12 failed, 10 passed`.
- Activation: `test_first_press_anchors_without_twist_when_feedback_changes`.
- Cleanup: `test_readonly_cleanup_aggregates_failures_and_attempts_every_stage`.
- Foreign source: `test_foreign_packet_after_admission_emits_source_changed_without_handoff`.
- IPv4: seven special-use address cases failed because they were incorrectly
  accepted.
- Reparse paths: both motion-lease and preflight-report cases failed because
  non-symlink Windows reparse attributes were not checked.

Targeted GREEN run after the minimal production changes:

- Result: `26 passed`.

Broader controller/backend/transport/input/admission/CLI regression run:

- Result: `233 passed`.
- Seven old assertions initially failed because they encoded the replaced
  behavior (commanding on activation or ignoring a foreign sender). Their
  expected event/command counts were updated to the approved behavior; the
  complete focused run then passed.

## Verification evidence

- Full offline suite: `353 passed in 26.44s`.
- Coverage fallback: `353 passed`; `kinova_teleop` total line coverage `89%`
  (`3003` statements, `321` missed), satisfying the required 80% minimum.
  The project `.venv` did not contain `pytest-cov` or Coverage.py, so the
  already installed system Python Coverage.py runner was used without network
  installation; `pytest-cov>=5.0` remains declared in development extras.
- Deterministic MuJoCo: `completed steps=2000 finite_state=true`.
- Python compilation: `python -m compileall -q kinova_teleop tests` passed with
  bytecode redirected to a temporary directory.
- Diff formatting: `git diff --check` passed; Git emitted only existing Windows
  LF-to-CRLF conversion notices.
- Security scan: no added hardcoded password/API-key/token, unique device
  identifier, gripper write, Action, or `ClearFaults` pattern was found.
- Kortex production write surface remains limited to `SetServoingMode`,
  `SendTwistCommand`, and `Stop`.
- Independent correctness review: no findings; reviewer also ran `353 passed`.
- Independent security review: no Critical, High, or Medium findings; reviewer
  ran 110 focused tests plus 41 Kortex CLI tests.

## Scope and remaining concerns

The deferred usability minors were deliberately not changed:

- optional `EvidenceJsonl` still requires pre-creation in the launcher;
- initially missing PICO input still fails closed instead of polling through
  the whole admission timeout;
- the first-press integration test does not add the deferred angular-cap
  exercise.

Pre-existing untracked pytest/coverage artifacts were preserved and not staged.

This report establishes offline implementation readiness only. It does not
establish T0 read-only observation or T1-T3 hardware-observed behavior. Physical
E-stop readiness, current robot identity/state, exact onsite `MOVE` confirmation,
actual no-jump behavior, and measured stop response still require the guarded
onsite procedure for this exact revision.
