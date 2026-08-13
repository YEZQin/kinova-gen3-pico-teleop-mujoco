# Public Zero-to-Kinova PICO Teleoperation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish a GitHub branch, Draft PR, and `v0.2.0-rc.1` pre-release that let a new Windows user install the supplied PICO APK and Kortex wheel, validate PICO/MuJoCo/Kortex, generate robot-specific guarded artifacts, and reach translation-only Gen3 teleoperation without developer-local files.

**Architecture:** Keep binary dependencies out of Git history and distribute them as hash-pinned GitHub pre-release assets. Add one PowerShell bootstrap boundary for clean-host installation, one Python calibration command for PICO-only operator-axis capture, and one Python hardware-package command that converts current T0 evidence plus explicit local bounds/checklist into strict lease/report/launcher artifacts. The existing `kinova_teleop.main` Kortex admission remains the only motion engine and continues to bind clean HEAD, calibration SHA, safety limits, evidence uniqueness, exact `MOVE`, watchdog, Stop, and source identity.

**Tech Stack:** Windows PowerShell 5.1+, CPython 3.11, pytest/coverage, NumPy, MuJoCo, Kinova Kortex API 2.8.0.post5/protobuf 3.20.0, Unity 2022.3.62f3c1/Android IL2CPP/OpenXR, GitHub CLI and GitHub Releases.

## Global Constraints

- Work only in `D:\yezq\26.7.24_Kinova_controll\.worktrees\kortex-merge`; preserve unrelated ignored/untracked artifacts.
- Publish source on `codex/public-gen3-pico-hardware-teleop`; keep `main` unchanged and open a Draft PR.
- English `README.md` and complete Chinese `README_CN.md` must have command parity and no developer-machine absolute path.
- Public first motion is translation-only, no gripper writes, 40 Hz, stale timeout at most 0.2 s, angular cap at most 2 deg/s, and operator-selected linear cap no greater than 0.02 m/s.
- Each robot must create its own calibration, T0 report, bounds, motion lease, reviewed report, evidence path, and generated launcher; never publish the laboratory's local report/lease/bounds as defaults.
- Never hardcode or persist the robot password. Non-secret package checks precede the masked prompt; exact `MOVE` remains inside the Python motion process.
- APK and Kortex wheel are release assets, not Git blobs. Pin exact SHA-256 and include redistribution notices.
- No implementation or documentation step may connect to PICO or Kortex or run a motion launcher. Hardware/network work is reserved for a human following the published instructions.
- Full regression coverage must remain at least 80%; final publication evidence must state the actual stronger result.
- Never describe the project as safety-rated, unattended, production-safe, or universally hardware-validated.

---

### Task 1: Release manifest and binary integrity contract

**Files:**
- Create: `release/public-release-assets.json`
- Create: `release/THIRD_PARTY_NOTICES.txt`
- Create: `kinova_teleop/release_assets.py`
- Test: `tests/test_release_assets.py`
- Modify: `.gitignore`

**Interfaces:**
- Produces: `ReleaseAsset(name: str, sha256: str, size_bytes: int, download_url: str)`, `load_release_manifest(path: Path) -> ReleaseManifest`, and `verify_release_asset(path: Path, asset: ReleaseAsset) -> None`.
- Consumes later: Task 2 uses the manifest for downloads; Task 6 replaces the release URLs only after the tag exists and revalidates the exact assets.

- [ ] **Step 1: Write strict manifest RED tests**

Add tests that construct a valid manifest and reject duplicate JSON keys, unexpected fields, non-HTTPS/GitHub URLs, traversal names, uppercase or malformed hashes, non-positive sizes, symlinks/reparse points, changed file identity, wrong size, and wrong hash. The valid payload is:

```python
{
    "schema_version": "1.0",
    "release": "v0.2.0-rc.1",
    "assets": [
        {
            "name": "kinova-pico-udp-bridge.apk",
            "sha256": "a" * 64,
            "size_bytes": 3,
            "download_url": "https://github.com/YEZQin/kinova-gen3-pico-teleop-mujoco/releases/download/v0.2.0-rc.1/kinova-pico-udp-bridge.apk",
        },
        {
            "name": "kortex_api-2.8.0.post5-py3-none-any.whl",
            "sha256": "5796425d48f0ab70c56ce9ecf06b8ab91f0b951319266e5d4c0b179cdb9e99cf",
            "size_bytes": 161265,
            "download_url": "https://github.com/YEZQin/kinova-gen3-pico-teleop-mujoco/releases/download/v0.2.0-rc.1/kortex_api-2.8.0.post5-py3-none-any.whl",
        },
    ],
}
```

- [ ] **Step 2: Run the release-asset tests and observe RED**

Run: `python -m pytest tests/test_release_assets.py -q -p no:cacheprovider`

Expected: import failure because `kinova_teleop.release_assets` does not exist.

- [ ] **Step 3: Implement the immutable manifest loader**

Use frozen/slotted dataclasses, strict UTF-8 JSON with duplicate-key and non-finite rejection, a fixed top-level/asset field set, `Path.name == name`, exact lowercase `[0-9a-f]{64}`, and GitHub release URL prefix validation. Implement `verify_release_asset` with `lstat`/`os.open`/`fstat`, symlink/reparse rejection, identity-before/after checks, streaming SHA-256, and exact size comparison.

- [ ] **Step 4: Add notices and ignore boundaries**

Add the complete Kinova BSD-3-Clause/Protocol Buffer notice and identify Kortex `2.8.0.post5`, the PICO OpenXR SDK pin `3aa3e62bff41df618529eeb60ff02c29a515dafe`, Unity runtime licensing, and project MIT license. Add `/downloads/`, `/local-config/`, `.venv-kortex/`, wheel files, and APK files to `.gitignore` while keeping `release/public-release-assets.json` and notices tracked.

- [ ] **Step 5: Run GREEN and commit**

Run:

```powershell
python -m pytest tests/test_release_assets.py -q -p no:cacheprovider
git diff --check
git add .gitignore release/public-release-assets.json release/THIRD_PARTY_NOTICES.txt kinova_teleop/release_assets.py tests/test_release_assets.py
git commit -m "feat: define public release asset integrity"
```

### Task 2: Clean-Windows bootstrap installer

**Files:**
- Create: `scripts/bootstrap_public_teleop.ps1`
- Create: `tests/test_public_bootstrap.py`
- Modify: `scripts/assert_no_untracked_python_startup_hooks.ps1`

**Interfaces:**
- CLI: `bootstrap_public_teleop.ps1 [-PythonExe <python.exe>] [-ReleaseManifest <json>] [-DownloadDirectory <dir>] [-VenvDirectory <dir>] [-ApkPath <apk>] [-KortexWheel <whl>] [-InstallApk] [-DeviceSerial <serial>] [-SkipOfflineTests]`.
- Produces: verified `downloads/` binaries, `.venv-kortex/`, `local-config/install-receipt.json`, and optionally an APK installed on exactly one authorized PICO.

- [ ] **Step 1: Write bootstrap RED contract tests**

Use temporary repositories plus fake `python`, `git`, `Invoke-WebRequest`, and `adb` shims. Assert that the script rejects non-Windows/Python-not-3.11, a dirty/untracked startup hook, manifest/hash/size mismatch, an existing non-venv target, multiple/unauthorized ADB devices, and download redirects outside HTTPS GitHub release assets. Assert that it never invokes ADB unless `-InstallApk`, never creates firewall rules, never reads `KINOVA_PASSWORD`, and orders:

```text
startup hook gate -> manifest load -> downloads -> hash verification -> venv -> pip installs -> import-only check -> pytest/dry-run -> optional ADB install -> receipt
```

- [ ] **Step 2: Run RED**

Run: `python -m pytest tests/test_public_bootstrap.py -q -p no:cacheprovider`

Expected: missing bootstrap script failures.

- [ ] **Step 3: Implement bootstrap with argument arrays**

The script must default to the tracked manifest, `downloads`, and `.venv-kortex`; select a Python 3.11 executable; invoke the existing startup-hook gate; download only missing assets; call a small import-only Python snippet using `release_assets.load_release_manifest` and `verify_release_asset`; create the venv; install the Kortex wheel before `${projectRoot}[dev]`; pin `protobuf==3.20.0`; run `import kortex_api`, focused pytest, and `--dry-run --headless --steps 2000`; and write a receipt containing versions, hashes, Git HEAD, and no secret.

- [ ] **Step 4: Implement safe APK installation**

When `-InstallApk`, require ADB, parse `adb devices` to exactly one `device` entry or an exact authorized `-DeviceSerial`, run `adb -s SERIAL install -r APK`, and never run `adb tcpip`, `adb connect`, or firewall commands.

- [ ] **Step 5: Run GREEN and commit**

Run:

```powershell
python -m pytest tests/test_public_bootstrap.py tests/test_kortex_launch_script.py tests/test_offline_env_script.py -q -p no:cacheprovider
git diff --check
git add scripts/bootstrap_public_teleop.ps1 scripts/assert_no_untracked_python_startup_hooks.ps1 tests/test_public_bootstrap.py
git commit -m "feat: bootstrap public PICO Kortex environment"
```

### Task 3: Public PICO calibration capture

**Files:**
- Create: `kinova_teleop/calibration_capture.py`
- Create: `scripts/capture_pico_operator_calibration.ps1`
- Create: `tests/test_calibration_capture.py`
- Modify: `kinova_teleop/main.py`

**Interfaces:**
- Python API: `capture_operator_calibration(source: XrInputSource, prompts: CalibrationPrompts, *, samples_per_pose: int = 20) -> dict[str, object]` and `write_new_calibration(path: Path, payload: Mapping[str, object]) -> None`.
- CLI: `python -m kinova_teleop.calibration_capture --output local-config/operator-axes.json [--samples-per-pose 20]`.
- Output must load successfully through the existing `load_operator_axis_calibration` strict loader.

- [ ] **Step 1: Write capture RED tests**

Drive a fake PICO source through neutral/right/neutral/up/neutral/forward poses. Test median aggregation, distinct increasing timestamps, Grip released throughout, at least 15 mm displacement, return-to-neutral at most 15 mm, positive determinant, condition number at most 2.0, pairwise absolute dot at most 0.35, exact field schema, atomic exclusive output, and refusal to overwrite/symlink/reparse paths. Include stale, source-change, pressed Grip, mixed axes, duplicate timestamp, and early EOF cases.

- [ ] **Step 2: Run RED**

Run: `python -m pytest tests/test_calibration_capture.py -q -p no:cacheprovider`

Expected: missing module failure.

- [ ] **Step 3: Implement capture and prompts**

Use `PicoUdpInput` through `ContinuousInputBuffer`, require the observer to press Enter at each pose, aggregate only finite valid distinct released samples, compute shared-neutral gesture schema exactly as expected by `operator_calibration.py`, recompute all summaries, and validate the completed temporary payload through the same pure validation path before an exclusive atomic write.

- [ ] **Step 4: Add a thin PowerShell entry point**

The script must locate `.venv-kortex\Scripts\python.exe` by default, write only beneath `local-config`, and print that it never connects to Kortex. It must not accept a robot IP or password.

- [ ] **Step 5: Run GREEN and commit**

Run:

```powershell
python -m pytest tests/test_calibration_capture.py tests/test_operator_calibration.py tests/test_pico_udp_input.py -q -p no:cacheprovider
git diff --check
git add kinova_teleop/calibration_capture.py scripts/capture_pico_operator_calibration.ps1 tests/test_calibration_capture.py kinova_teleop/main.py
git commit -m "feat: capture public PICO axis calibration"
```

### Task 4: Robot-specific guarded package generator

**Files:**
- Create: `kinova_teleop/public_hardware_setup.py`
- Create: `scripts/prepare_gen3_hardware.ps1`
- Create: `scripts/start_generated_gen3_teleop.ps1`
- Create: `tests/test_public_hardware_setup.py`
- Create: `tests/test_generated_hardware_launcher.py`
- Modify: `kinova_teleop/preflight.py`
- Modify: `kinova_teleop/motion_lease.py`

**Interfaces:**
- CLI phase 1: `python -m kinova_teleop.public_hardware_setup t0 --robot-ip <private-ip> --output local-config/t0.json` performs only existing read-only Kortex preflight after masked credential handoff by PowerShell.
- CLI phase 2: `python -m kinova_teleop.public_hardware_setup package --t0 <json> --calibration <json> --workspace-min X Y Z --workspace-max X Y Z --linear-speed <0..0.02> --output-dir local-config` is offline and writes a lease, reviewed report, and `teleop-profile.json` only after explicit physical checklist flags.
- Launcher: `start_generated_gen3_teleop.ps1 -Profile local-config/teleop-profile.json` performs offline validation, PICO gate, current checklist confirmation, masked password, current live read-only preflight, exact in-main `MOVE`, and unique evidence.

- [ ] **Step 1: Write generator RED tests**

Use a schema-valid T0 fixture and calibration fixture. Require T0 device `gen3`, model L53, 7 DoF, firmware `2.8.0-5`, private IP match, finite feedback pose, Running/Single Level/manual-control-ready state, clean current revision, and all checklist flags. Reject stale/mismatched T0, missing confirmations, bounds not containing the T0 pose, non-finite/reversed bounds, spans over calibrated caps `(1.2, 1.2, 0.64)`, lower Z below zero, speed over `0.02`, gripper/orientation requests, output overwrite, symlink/reparse output, and copied code/calibration hashes.

- [ ] **Step 2: Run generator RED**

Run: `python -m pytest tests/test_public_hardware_setup.py tests/test_generated_hardware_launcher.py -q -p no:cacheprovider`

Expected: missing module/scripts.

- [ ] **Step 3: Add public immutable artifact builders**

Add `create_motion_lease(run_id, owner, now, nonce) -> MotionLease` while retaining read-only validation. Add a preflight builder that accepts an already validated T0 structure, current code/calibration/driver hashes, exact motion contract, and nine explicit physical checks; it must serialize the same strict schema consumed by `load_passing_preflight_report` and self-load it before output.

- [ ] **Step 4: Implement the two-phase setup CLI**

`t0` prints the same evidence boundary as the current quickstart and exclusively writes read-only JSON. `package` performs no password lookup, SDK import, PICO read, network call, or Kortex construction. It generates random run/lease identifiers, binds the current Git revision and calibration SHA, records the measured reference pose separately from the asymmetric geometric midpoint, and writes all artifacts with exclusive creation under `local-config`.

- [ ] **Step 5: Implement the generated-profile launcher**

The profile contains only robot IP/user, paths relative to its directory, exact bounds/speed, run/owner IDs, and non-secret hashes. The PowerShell launcher validates the startup-hook gate, regular files, profile schema, and `--validate-motion-package` before PICO or password; performs the input gate; prompts for a fixed phrase `HARDWARE-READY`; reads the password as `SecureString`; supplies it only to the child process; and cleans it in `finally`. It invokes the existing Python motion CLI with translation-only, expanded calibrated responsive profile, stale recovery, 40 Hz, scale 0.8, operator calibration, no gripper, and unique UTC+GUID evidence.

- [ ] **Step 6: Run GREEN and commit**

Run:

```powershell
python -m pytest tests/test_public_hardware_setup.py tests/test_generated_hardware_launcher.py tests/test_preflight.py tests/test_motion_lease.py tests/test_cli_kortex.py -q -p no:cacheprovider
git diff --check
git add kinova_teleop/public_hardware_setup.py kinova_teleop/preflight.py kinova_teleop/motion_lease.py scripts/prepare_gen3_hardware.ps1 scripts/start_generated_gen3_teleop.ps1 tests/test_public_hardware_setup.py tests/test_generated_hardware_launcher.py
git commit -m "feat: generate robot-specific guarded teleop package"
```

### Task 5: Complete bilingual zero-to-run documentation

**Files:**
- Rewrite: `README.md`
- Create: `README_CN.md`
- Rewrite: `docs/kortex-hardware-quickstart.md`
- Modify: `docs/pico-udp-quickstart.md`
- Create: `docs/evidence-levels.md`
- Create: `tests/test_public_docs.py`
- Modify: `pyproject.toml`

**Interfaces:**
- Produces the public human interface; every command must reference a tracked file or an actual `v0.2.0-rc.1` release asset.
- English and Chinese heading/command manifests must match.

- [ ] **Step 1: Write documentation RED contracts**

Parse fenced PowerShell commands and require both READMEs to contain the same ordered lifecycle markers:

```text
clone -> bootstrap -> APK install -> PICO gate -> MuJoCo finite -> calibration -> T0 -> local bounds/package -> offline validation -> physical checklist -> hardware launcher -> Grip release -> MOVE -> stop/troubleshooting
```

Reject any `D:\yezq`, local laboratory report/lease/event filename, literal password, broad copied workspace, `admin` presented as a password, untracked script path, missing evidence table, missing SDK/APK licenses, or hardware-validated claim for the final expanded profile.

- [ ] **Step 2: Run docs RED**

Run: `python -m pytest tests/test_public_docs.py -q -p no:cacheprovider`

Expected: missing Chinese README and lifecycle failures.

- [ ] **Step 3: Write English README from zero**

Cover supported scope, architecture, safety warning, clean Windows prerequisites, exact branch clone/ZIP, bootstrap, APK build alternative, network/firewall, PICO-only check, MuJoCo check, calibration, Kortex wheel/firmware compatibility, T0, robot-specific bounds, artifact generation, package validation, first motion, Stop/recovery, gripper/vision limits, troubleshooting, development checks, evidence matrix, licensing, citations, and an exact quick-start recap.

- [ ] **Step 4: Write Chinese parity documentation**

Translate meaning rather than merely headings, preserving every command and numerical constraint. Link `README.md` and `README_CN.md` to each other at the top. Refactor the two quickstarts so they add detail without contradicting the root READMEs.

- [ ] **Step 5: Update packaging metadata and run GREEN**

Set the project description to include guarded Kortex hardware teleoperation, include relevant non-code package data if needed, and add a `project.urls` block pointing at repository, documentation, issues, and release pages.

Run:

```powershell
python -m pytest tests/test_public_docs.py tests/test_pico_udp_scripts.py tests/test_kortex_launch_script.py -q -p no:cacheprovider
git diff --check
git add README.md README_CN.md docs/kortex-hardware-quickstart.md docs/pico-udp-quickstart.md docs/evidence-levels.md tests/test_public_docs.py pyproject.toml
git commit -m "docs: add bilingual zero-to-Kinova guide"
```

### Task 6: Publication branch, exact binaries, full validation, Draft PR, and pre-release

**Files:**
- Modify: `release/public-release-assets.json` with exact APK hash/size and confirmed URLs
- Create locally only: `artifacts/kinova-pico-udp-bridge.apk`
- Create locally only: `artifacts/kortex_api-2.8.0.post5-py3-none-any.whl`
- Create locally only: `artifacts/SHA256SUMS.txt`
- Create locally only: `artifacts/THIRD_PARTY_NOTICES.txt`
- Report: `.superpowers/sdd/2026-08-13-public-zero-to-kinova-teleop/final-report.md`

**Interfaces:**
- Produces remote branch `codex/public-gen3-pico-hardware-teleop`, Draft PR to `main`, and GitHub pre-release `v0.2.0-rc.1` with four verified assets.

- [ ] **Step 1: Reconcile with remote main on a publication branch**

Fetch `origin`, create `codex/public-gen3-pico-hardware-teleop` from the completed feature tip, and merge `origin/main` with a merge commit. Resolve conflicts by preserving the new bilingual/public workflow and the latest stable-main changes. Do not modify local or remote `main`.

- [ ] **Step 2: Build the exact APK**

Use Unity `2022.3.62f3c1` and the tracked `build_pico_udp_bridge.ps1` without installation to run EditMode tests and build ARM64/IL2CPP from the exact publication commit. Copy only the resulting APK to `artifacts/kinova-pico-udp-bridge.apk`, compute SHA-256/size, update the tracked manifest, and commit:

```powershell
git add release/public-release-assets.json
git commit -m "chore: pin public release binaries"
```

- [ ] **Step 3: Assemble and scan release assets**

Copy the exact Kortex wheel and notices to `artifacts`, create sorted `SHA256SUMS.txt`, inspect archive/APK contents for secrets and developer paths, verify Kortex metadata/license, and confirm no firmware, virtualenv, Unity installation, logs, serial numbers, credentials, or private report is present.

- [ ] **Step 4: Test from a clean temporary checkout**

Clone the local publication commit into a temporary directory with no reused venv. Run the bootstrap against local asset overrides, without APK installation or network/robot access; run calibration and package CLIs against deterministic fixtures; run documentation command/path checks. Verify the temporary tree creates only ignored `downloads`, `.venv-kortex`, and `local-config` outputs.

- [ ] **Step 5: Run final verification**

Run:

```powershell
$env:PYTHONPATH = (Get-Location).Path
python -m coverage run -m pytest -q -p no:cacheprovider
python -m coverage report --fail-under=80
python -m compileall -q kinova_teleop tests
python -m kinova_teleop.main --dry-run --headless --steps 2000
git diff --check
git status --short --untracked-files=no
```

Require zero failures, at most the known Windows symlink-capability skip, coverage at least 80%, `finite_state=true`, and clean tracked status. Run independent correctness/security/license reviews and fix every Critical/High/Medium finding with RED-first tests and scoped re-review.

- [ ] **Step 6: Push and create Draft PR**

Push the named branch with upstream tracking; verify `git rev-parse HEAD` equals `git ls-remote origin refs/heads/codex/public-gen3-pico-hardware-teleop`. Create a Draft PR to `main` whose body lists architecture, exact test evidence, real-hardware evidence levels, first-motion checklist, limitations, license treatment, and unverified final profile.

- [ ] **Step 7: Create and verify pre-release**

Create annotated tag `v0.2.0-rc.1` at the exact remote branch head and a GitHub pre-release with the APK, wheel, checksums, and notices. Download all four assets into a fresh directory and verify their SHA-256 against `SHA256SUMS.txt`. Verify the GitHub tag target equals the validated commit.

- [ ] **Step 8: Write final report and handoff links**

Record local/remote/tag hashes, tests, coverage, Unity build evidence, asset hashes/sizes, secret/license scans, Draft PR URL, release URL, branch README/Chinese README/ZIP URLs, asset URLs, exact branch clone command, and the explicit hardware-evidence boundary. Do not merge the Draft PR.

---

## Plan self-review

- Spec coverage: all publication topology, binary, bilingual docs, bootstrap, calibration, robot-local package, evidence, licensing, validation, push, PR, release, and link requirements map to Tasks 1–6.
- Placeholder scan: no TBD/TODO/deferred implementation instruction remains. Release hashes that cannot exist before the exact APK build are updated and committed explicitly in Task 6 before publication.
- Interface consistency: Task 1 manifest is consumed by Task 2; Task 3 output is accepted by the existing strict calibration loader and consumed by Task 4; Task 4 artifacts are consumed by the existing `kinova_teleop.main`; Task 5 documents only these tracked interfaces; Task 6 validates and publishes their exact commit.
