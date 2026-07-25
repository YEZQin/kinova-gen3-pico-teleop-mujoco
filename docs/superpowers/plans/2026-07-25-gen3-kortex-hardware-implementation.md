# Gen3 Kortex Hardware Teleoperation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an explicitly gated Kinova Kortex TCP backend while preserving the existing MuJoCo default and allowing complete offline verification.

**Architecture:** Extract the current MuJoCo-specific behavior behind a pose-target backend, then add a lazily imported Kortex transport and a base-frame pose-to-twist backend with an independent watchdog. Keep XR mapping and loop orchestration backend-neutral; construct the selected backend only after CLI validation and hardware confirmation.

**Tech Stack:** Python 3.10+, NumPy, MuJoCo, optional Kinova Kortex Python wheel, pytest.

## Global Constraints

- Default backend is MuJoCo and its existing behavior must remain unchanged.
- Hardware requires `--backend kortex --enable-hardware`, `KINOVA_PASSWORD`, and exact interactive confirmation `MOVE` before importing Kortex or connecting.
- Fixed upper limits are `0.03 m/s` linear and `5 deg/s` angular; CLI values may be lower only.
- XR stale timeout and the independent command watchdog are both `0.2 s`.
- Grip release, invalid/stale XR, backend exception, watchdog timeout, and exit must Stop.
- No automatic Home, stored action, gripper command, UDP low-level servo, SDK binary, or secret is added.
- Kortex imports remain lazy so MuJoCo and tests work without the SDK.
- All production behavior is introduced test-first and verified using fake clients; no real robot connection occurs.

---

### Task 1: Backend protocol and MuJoCo extraction

**Files:**
- Create: `kinova_teleop/backend.py`
- Create: `kinova_teleop/mujoco_backend.py`
- Modify: `kinova_teleop/teleop_controller.py`
- Modify: `tests/test_teleop_controller.py`
- Create: `tests/test_backend_controller.py`

**Interfaces:**
- Produces `BackendResult`, `EndEffectorTargetBackend`, `MuJoCoBackend`, and backend-neutral `TeleopController`.
- `TeleopController(config, source, backend)` owns the mapper; `backend` owns simulator state and stepping.

- [ ] **Step 1: Write failing orchestration tests**

Use a real `RelativePoseMapper` plus a recording backend. Verify first Grip calls
`begin_control`, active input calls `command_pose`, release calls `hold` once,
every iteration calls `step`, and close calls backend/source close idempotently.

- [ ] **Step 2: Verify RED**

Run `python -m pytest tests/test_backend_controller.py -q`; it must fail because
the backend protocol and constructor do not exist.

- [ ] **Step 3: Implement the minimal protocol and orchestration**

```python
@dataclass(frozen=True)
class BackendResult:
    converged: bool = True
    position_error: float = 0.0
    rotation_error: float = 0.0

class EndEffectorTargetBackend(Protocol):
    def current_pose(self) -> Pose: ...
    def begin_control(self) -> None: ...
    def command_pose(self, target: Pose) -> BackendResult: ...
    def hold(self) -> None: ...
    def step(self) -> None: ...
    def close(self) -> None: ...
```

Move model loading, data, IK, controls, marker and MuJoCo stepping into
`MuJoCoBackend`. Preserve public compatibility properties on `TeleopController`
(`model`, `data`, `contract`, `ik`, `steps`) where existing tests/CLI consume
them.

- [ ] **Step 4: Verify GREEN and regression**

Run:

```text
python -m pytest tests/test_backend_controller.py tests/test_teleop_controller.py -q
python -m kinova_teleop.main --dry-run --headless --steps 500
```

- [ ] **Step 5: Commit**

Commit as `refactor: extract teleoperation backend`.

### Task 2: Kortex transport, pose-to-twist and watchdog

**Files:**
- Create: `kinova_teleop/kortex_transport.py`
- Create: `kinova_teleop/kortex_backend.py`
- Create: `tests/test_kortex_transport.py`
- Create: `tests/test_kortex_backend.py`

**Interfaces:**
- Consumes `Pose`, quaternion helpers, `BackendResult`.
- Produces `KortexConnection`, `KortexConfig`, `KortexBackend`.
- Transport factories and monotonic time are injectable; tests never import the real SDK.

- [ ] **Step 1: Write failing math and lifecycle tests**

Cover literal hand-derived cases:

- feedback `(x,y,z, theta_x,theta_y,theta_z)=(1,2,3,0,0,90)` produces position
  `[1,2,3]` and wxyz `[sqrt(0.5),0,0,sqrt(0.5)]`;
- `0.1 m` X error at gain `1/s` clips to `[0.03,0,0] m/s`;
- `90 deg` Z error clips to `[0,0,5] deg/s`;
- release/hold sends Stop once; close is idempotent and stops before disconnect;
- a fake-clock watchdog crossing `0.2 s` stops once and latches;
- a fresh command resets the deadline; a stopped generation cannot send afterward.

- [ ] **Step 2: Verify RED**

Run `python -m pytest tests/test_kortex_transport.py tests/test_kortex_backend.py -q`;
it must fail because both modules are absent.

- [ ] **Step 3: Implement lazy transport**

`KortexConnection.connect()` imports `kortex_api` internally, builds TCP transport
on port `10000`, Router, SessionManager, BaseClient and BaseCyclicClient, and
creates the session. `close()` calls Stop then closes session/router/transport in
reverse order and never includes the password in raised messages.

- [ ] **Step 4: Implement Kortex backend**

Use extrinsic fixed-axis XYZ:

```python
R = Rz(theta_z) @ Ry(theta_y) @ Rx(theta_x)
q_error = target_q * conjugate(measured_q)
rotvec = quat_to_rotvec(q_error)
linear = clip_norm(kp_linear * position_error, max_linear_speed)
angular_deg = degrees(clip_norm(kp_angular * rotvec, radians(max_angular_deg)))
```

Send `TwistCommand` in BASE frame with `duration=0`. Set
`SINGLE_LEVEL_SERVOING`; never execute an Action/Home. Synchronize send, heartbeat
and Stop with one lock. Arm a daemon watchdog only after a successful nonzero
Twist; timeout is `0.2 s`.

- [ ] **Step 5: Verify GREEN**

Run `python -m pytest tests/test_kortex_transport.py tests/test_kortex_backend.py -q`.

- [ ] **Step 6: Commit**

Commit as `feat: add guarded Kortex pose backend`.

### Task 3: CLI hardware safety gate and integration

**Files:**
- Modify: `kinova_teleop/main.py`
- Modify: `tests/test_cli.py`

**Interfaces:**
- Consumes `MuJoCoBackend`, `KortexConnection`, `KortexBackend`.
- Produces parser options and selection while retaining `main(argv) -> int`.

- [ ] **Step 1: Write failing CLI tests**

Verify all rejection paths return `2` before a supplied connection factory is
called: missing `--enable-hardware`, missing `KINOVA_PASSWORD`, wrong confirmation,
linear limit above `0.03`, angular limit above `5`, and Kortex plus `--dry-run`.
Verify MuJoCo default never imports/connects Kortex. Verify valid Kortex arguments
and `MOVE` create the connection and run the controller with scale default `0.5`.

- [ ] **Step 2: Verify RED**

Run `python -m pytest tests/test_cli.py -q`; new tests must fail because the
options/gates are absent.

- [ ] **Step 3: Implement parser and pre-connect gate**

Add `--backend {mujoco,kortex}`, `--enable-hardware`, `--robot-ip`,
`--robot-user`, `--max-linear-speed`, and `--max-angular-speed-deg`.
Validate all arguments and environment first, then call `input()` and require the
exact string `MOVE`; only afterward import/construct Kortex classes. Do not echo
the password. Keep MuJoCo default construction and finite-state report.

- [ ] **Step 4: Verify GREEN and regression**

Run:

```text
python -m pytest tests/test_cli.py tests/test_teleop_controller.py -q
python -m kinova_teleop.main --dry-run --headless --steps 500
```

- [ ] **Step 5: Commit**

Commit as `feat: gate Kortex hardware backend`.

### Task 4: Operator documentation and full offline verification

**Files:**
- Modify: `README.md`
- Modify: `pyproject.toml` only if optional metadata is needed; do not add a
  downloadable Kortex dependency.

**Interfaces:**
- Documents the existing MuJoCo path and the new explicitly gated hardware path.

- [ ] **Step 1: Update README**

Add a clearly marked “designed/offline-tested, not yet real-robot validated”
section. Document matching the Kortex wheel to robot firmware/Python, wired
network/TCP `10000`, read-only first check, environment-only password, exact
launch command, `MOVE`, no auto-Home, first-Grip anchoring, and the future
low-speed acceptance checklist (`0.01 m/s`, `2 deg/s` initially).

- [ ] **Step 2: Run full verification**

Run:

```text
python -m pytest -q
python -m kinova_teleop.main --dry-run --headless --steps 2000
python -m kinova_teleop.main --help
git diff --check
```

- [ ] **Step 3: Commit**

Commit as `docs: explain Kortex hardware operation`.

- [ ] **Step 4: Whole-branch review**

Review the complete diff from the design commit through HEAD for safety,
specification compliance, lazy SDK behavior, thread races, secret handling and
MuJoCo regression. Fix Critical/Important findings and rerun the full verification.
