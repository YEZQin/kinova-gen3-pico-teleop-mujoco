# Public zero-to-Kinova PICO teleoperation release design

## Objective

Publish a reproducible public handoff that lets a new operator start with a
clean Windows 10/11 computer and reach guarded PICO left-controller
teleoperation of a Kinova Gen3 without relying on files, paths, credentials,
or undocumented knowledge from the development computer.

The handoff must be useful before real-arm motion: a new user must be able to
install the software, install and validate the PICO application, run the
MuJoCo check, observe fresh controller data, prepare robot-specific safety
artifacts, perform a read-only Kortex check, and only then enter the explicit
motion workflow.

## Supported public scope

- Host: native Windows 10/11 PowerShell, not WSL.
- Python: CPython 3.11.x for the Kortex 2.8 environment.
- XR device: PICO 4 / PICO 4 Ultra using the included Unity OpenXR bridge and
  the left controller.
- Robot: Kinova Gen3 7DoF on the Kortex 2.8 family. The tested development
  device was an L53 Gen3 Ultra with vision module and gripper.
- Default safe path: MuJoCo simulation.
- Hardware path: guarded Kortex Cartesian Twist control, explicit enablement,
  local credentials, current read-only checks, exact `MOVE`, Grip clutch,
  watchdog, Stop, workspace limits, and evidence logging.
- The initial public hardware walkthrough uses translation-only motion and no
  gripper command. Full orientation and gripper control remain advanced,
  separately reviewed options rather than first-run defaults.

## Publication topology

The source will be published on a new branch named
`codex/public-gen3-pico-hardware-teleop`. The current stable `main` will not be
replaced while the final expanded-speed hardware profile remains unobserved.
A Draft PR will target `main`.

A GitHub pre-release named `v0.2.0-rc.1` will point at the exact validated
publication commit. It will contain:

- `kinova-pico-udp-bridge.apk`, rebuilt from the publication commit;
- `kortex_api-2.8.0.post5-py3-none-any.whl`;
- `SHA256SUMS.txt` covering every binary release asset;
- `THIRD_PARTY_NOTICES.txt` containing the Kinova and relevant binary
  redistribution notices.

The APK and Kortex wheel are release assets, not normal Git blobs. The source
tree will contain their expected names, versions, SHA-256 values, download
locations, and reproducible build/install instructions. No release or tag may
be advertised until GitHub confirms that it exists and points at the validated
commit.

## Documentation structure

`README.md` will be the complete English entry point. `README_CN.md` will be a
complete Chinese counterpart with the same command sequence and safety gates.
Neither document may require the reader to infer a step from the other.

Both documents must contain this zero-to-run sequence:

1. supported hardware, evidence level, architecture, and limitations;
2. install Git, Python 3.11, ADB/Android tooling, and required Windows tools;
3. clone the exact publication branch or download its ZIP;
4. run the bootstrap installer, which downloads or accepts the release assets,
   verifies SHA-256, creates a local venv, and installs the project plus Kortex
   wheel without storing credentials;
5. install the supplied APK with ADB or use the documented headset-side path;
6. start the PICO Bridge and pass a PICO-only fresh-sample/Grip check;
7. run the deterministic MuJoCo 2000-step finite-state check;
8. capture or load an operator-axis calibration and verify right/up/forward;
9. configure the robot's private IPv4 address and run current-session read-only
   Kortex T0;
10. choose robot-specific absolute XYZ bounds from the measured TCP pose and
    physically inspect the entire swept volume;
11. generate local motion-lease and reviewed-preflight artifacts through a
    guided tool, never by copying the development computer's JSON;
12. run a package-only validation that reads no password and opens no Kortex
    connection;
13. reconfirm the physical checklist, enter the password through a masked
    prompt, type exact `MOVE`, verify Grip-release Stop, and test one axis at a
    time with millimetre-scale motion;
14. stop, recover, collect evidence, and troubleshoot common PICO, UDP, Kortex,
    workspace, stale-input, and startup-hook errors.

The quick-start block at the end must use only commands that exist in the
published commit. Machine-specific paths will be variables or discovered
paths. Developer-machine absolute paths, passwords, calibration
report, lease, evidence JSONL, and wide workspace values must not appear as
copyable defaults.

## Public bootstrap and local configuration tools

The publication will add a focused bootstrap script that:

- validates Windows PowerShell and Python 3.11;
- obtains the APK and Kortex wheel from the exact GitHub pre-release, or accepts
  explicitly supplied local files;
- verifies the pinned SHA-256 values before install;
- creates `.venv-kortex` and installs the wheel, project, and locked compatible
  dependencies;
- optionally installs the APK only after confirming exactly one authorized
  PICO ADB device;
- performs import-only and offline checks;
- never changes the firewall automatically and never contacts the robot.

A separate local setup tool will guide the operator through calibration and
hardware-package creation. Its responsibilities are:

- write artifacts only under a Git-ignored local directory;
- validate all paths, numbers, finite poses, bounds, hashes, model, firmware,
  user confirmations, and evidence level;
- derive no workspace from another robot or the development machine;
- preserve the current admission contract: calibration SHA, clean code
  revision, exact CLI safety limits, unique evidence path, and motion lease;
- stop after read-only T0 and require a separate explicit operator step before
  any motion launcher is presented.

The existing generic Kortex launcher remains the execution boundary. If its
interface is insufficient for a fresh-computer workflow, it may be wrapped by
a local generated launcher, but no password, absolute developer path, or broad
reusable workspace may be written into tracked files.

## Safety and evidence claims

The public documentation must distinguish:

| Capability | Publication evidence |
|---|---|
| Python mapping, admission, Stop, and watchdog behavior | automated unit/integration tests |
| Model and loop finite | deterministic headless MuJoCo run |
| PICO pose/Grip stream | previously observed onsite; each new installation must repeat the input gate |
| Read-only Kortex compatibility | previously observed on the development device; each robot must repeat T0 |
| Fixed small Kortex motion | previously observed on the development device |
| PICO translation-only arm motion | previously observed on the development device |
| Final `0.02 m/s`, asymmetric workspace, and recoverable-boundary profile | offline verified, still requiring onsite observation |

No README, release, PR, badge, or title may describe the project as
safety-rated, production-safe, universally hardware-validated, or suitable
for unattended operation. Stop RPC completion is not proof of physical
stationarity. A reachable physical E-stop/Web Stop and a second observer are
required for first motion.

## Licensing and redistribution

The repository remains MIT licensed for project-owned source. The Kinova
Kortex wheel declares BSD-3-Clause metadata and will be redistributed only
with the Kinova copyright, conditions, disclaimer, exact upstream identity,
version, and SHA-256. The PICO APK is a compiled artifact of this repository's
Unity project; third-party Unity/PICO components and their source pins will be
identified in the notices and README.

The release must not contain Unity installation files, Android SDK/NDK files,
virtual environments, firmware packages, private SDK directories, generated
logs, robot credentials, PICO serial numbers, private device data, or any
dependency without a documented redistribution basis.

## Validation and publication gates

Before any push or GitHub release:

- merge or otherwise reconcile the current feature history with latest
  `origin/main` on the publication branch without rewriting the stable branch;
- make the tracked worktree clean and review every explicitly staged path;
- run the full pytest suite with at least 80% coverage;
- run compileall and the 2000-step finite MuJoCo check;
- validate English/Chinese command parity and all referenced paths;
- test bootstrap in a temporary clean directory without using the existing
  venv;
- build the APK from the exact publication commit and run its Unity test gate;
- verify binary names, sizes, SHA-256 values, notices, and absence of secrets;
- run correctness and security review with no open Critical/High/Medium issue;
- push the publication branch and verify remote HEAD equals local HEAD;
- create the Draft PR and pre-release, upload all assets, download them back,
  and re-verify SHA-256.

The final handoff must provide actual URLs for the branch README, Chinese
README, branch ZIP, Draft PR, pre-release, APK, Kortex wheel, checksums, and
clone command.

## Acceptance criteria

The publication is complete only when a technically competent user on a clean
Windows computer can follow one language version in order without unpublished
files or developer-specific paths, reach all offline/PICO/read-only gates, and
arrive at a guarded motion command whose remaining inputs are exclusively
their own robot pose, inspected workspace, local calibration, local lease,
local reviewed report, masked password, and exact `MOVE` confirmation.
