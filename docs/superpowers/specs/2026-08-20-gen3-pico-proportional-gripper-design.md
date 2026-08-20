# Gen3 PICO Proportional Gripper Teleoperation Design

## Goal

Extend the existing PICO-to-Kinova translation-only teleoperation path with
proportional Gen3 gripper control while simplifying the onsite launch flow and
making motion scale, linear-speed limit, and absolute workspace bounds runtime
parameters.

The approved operator-facing behavior is:

- hold the left-controller Grip button to clutch arm translation;
- while Grip remains held, map the left index Trigger proportionally to the
  gripper, with `0.0` fully open and `1.0` fully closed;
- when Grip is released, stop arm following and hold the gripper at its last
  commanded position;
- use translation-only arm mapping with a default scale of `1.0` and a default
  maximum linear speed of `0.05 m/s`;
- accept operator-supplied finite absolute XYZ bounds without the project's
  previous fixed workspace-span cap; and
- request exact `MOVE` once, without a preceding duplicate
  `HARDWARE-READY` confirmation.

This is an advanced hardware profile built on the already observed PICO
translation-only hardware path. It is not the public first-motion default and
does not claim safety-rated or unattended operation.

## Selected Approach

Use one integrated controller and one Kortex backend. Port the mature
`GRIPPER_POSITION` command implementation from the existing
`kinova-only-joint-speed` worktree into the current hardened backend rather
than introducing a second control process or a second Kortex connection.

The arm and gripper share the backend's RPC serialization, generation state,
fault handling, and cleanup lifecycle. This preserves Stop priority and avoids
concurrent vendor RPCs from independent loops. The current input, admission,
watchdog, stale-recovery, source-locking, and feedback-based re-anchoring paths
remain the foundation; the older controller implementation is not copied.

Rejected alternatives are:

- a separate gripper thread or process, because it creates RPC ordering and
  shutdown races; and
- Trigger-as-gripper-velocity control, because the approved contract is
  proportional gripper position.

## Architecture and Data Flow

```text
PICO left controller
  |-- pose --------------------------> translation mapping
  |-- Grip --------------------------> arm/gripper clutch
  `-- Trigger [0, 1] ----------------> gripper target position
                                                |
                                                v
TeleopController
  |-- feedback-based arm anchor on Grip press
  |-- scale=1.0 relative translation target
  |-- max-linear-speed=0.05 m/s rate limit
  `-- proportional gripper target while active
                                                |
                                                v
KortexBackend (one connection and one RPC lock)
  |-- SendTwistCommand
  `-- SendGripperCommand(GRIPPER_POSITION)
```

The Trigger value already transported by the SDK and PICO UDP input adapters
is the only new controller signal consumed by the teleoperation loop. Legacy
PICO payloads that do not carry Trigger retain their decoded value of zero and
therefore cannot unexpectedly close the gripper.

## Clutch and Gripper State Semantics

The controller keeps one coherent Grip-clutch state machine:

1. With Grip released, arm following is inactive and no new gripper target is
   sent. The gripper holds its last commanded position.
2. The first accepted Grip-pressed sample establishes the arm's current
   controller and end-effector references. The activation cycle remains
   anchor-only and sends no Twist. Trigger control may begin after the backend
   has successfully entered the active generation.
3. On subsequent fresh Grip-pressed samples, pose controls translation and
   Trigger controls proportional gripper position concurrently.
4. Releasing Grip requests the existing arm hold/Stop transition, disables new
   gripper commands, and preserves the last gripper position.
5. A later Grip press obtains fresh robot feedback and starts a new generation
   with a new arm anchor. It does not reset or open the gripper.

Gripper commands clamp the finite proportional input to `[0.0, 1.0]`, use
`GRIPPER_POSITION`, set finger identifier `1`, and are suppressed by a small
deadband and minimum send interval to avoid unnecessary RPC load. They share
the backend's generation and RPC lock. Gripper commands do not arm or refresh
the arm Twist watchdog.

## Motion Mapping and Runtime Limits

The advanced profile defaults are:

```text
translation_only = true
translation_scale = 1.0
max_linear_speed_m_s = 0.05
gripper_enabled = true when explicitly requested
gripper_open_position = 0.0
gripper_closed_position = 1.0
```

Scale and maximum linear speed are independent. Scale maps controller
displacement to target displacement; the speed limit bounds how quickly the
robot may approach that target.

The launcher and Python CLI expose robot host, controller-axis calibration,
scale, maximum linear speed, workspace minimum, workspace maximum, and gripper
enablement as runtime parameters. Values must be finite, ordered, and within
the command-specific domain. The current read-only end-effector position must
lie inside the supplied workspace before the motion connection is constructed.

The project's previous global workspace-span cap is removed only for this
explicit advanced runtime profile. Absolute workspace bounds remain mandatory.
The program does not create an unbounded mode and does not override Kinova Web
App limits, robot firmware limits, or the physical E-stop.

The separate fixed per-Grip translation envelope is removed for this profile.
The supplied absolute workspace is the sole project-level translational
envelope.

## Boundary and Fault Behavior

An arm target beyond the supplied absolute workspace is projected to the
nearest point inside the box. The program remains active:

- continued outward controller motion holds the robot at the boundary;
- inward controller motion immediately resumes following; and
- releasing and pressing Grip re-anchors at the current robot pose.

No out-of-box target reaches `SendTwistCommand`, and boundary projection is not
reported as a Kortex safety fault or process exit.

Other lifecycle behavior remains explicit:

- a short PICO stale interval requests arm hold/Stop, freezes the gripper, and
  waits for fresh released samples before returning to ready;
- Grip release requests arm hold/Stop and freezes the gripper without exiting;
- changed input source, malformed or non-finite input, Kortex fault, watchdog
  failure, unconfirmed Stop, or arm/gripper RPC failure requests Stop and
  terminates the session;
- `Ctrl+C` requests Stop, closes the backend and input source, and leaves the
  gripper at its current position; and
- no stale, error, shutdown, or cleanup path automatically opens the gripper.

A gripper RPC failure is treated as a session-level hardware fault because arm
and gripper share one transport and RPC lifecycle. The backend must attempt the
same confirmed Stop transition used for motion-command failures.

## Launch Flow

The advanced PowerShell launcher accepts named parameters equivalent to:

```powershell
.\scripts\start_gen3_pico_teleop.ps1 `
  -RobotHost 192.168.1.10 `
  -Scale 1.0 `
  -MaxLinearSpeed 0.05 `
  -WorkspaceMin "XMIN,YMIN,ZMIN" `
  -WorkspaceMax "XMAX,YMAX,ZMAX" `
  -EnableGripper
```

The operational sequence is reduced to:

1. verify fresh PICO input with released Grip;
2. obtain a current read-only Kortex state and end-effector pose;
3. collect the password through a masked prompt scoped to the child process;
4. request exact `MOVE` once inside the Python motion boundary; and
5. enter Grip-clutched translation and Trigger-controlled gripper operation.

The launcher removes the duplicate `HARDWARE-READY` prompt. It retains input
freshness, source continuity, live read-only Kortex checks, exact `MOVE`,
watchdog, Stop, workspace validation, credential cleanup, and unique evidence
logging. Runtime changes to scale, speed, or workspace do not require copying a
new tracked launcher; the command-line contract itself performs pre-motion
validation against the current robot pose.

The existing generated-package path remains supported for the public guarded
walkthrough. The new advanced launcher must not silently weaken or reinterpret
an existing reviewed package. Where a reviewed report is supplied, its robot
and calibration identity must still match; runtime motion values are validated
directly by the advanced CLI contract.

## Compatibility

- MuJoCo behavior and its default mapping remain unchanged.
- Full-orientation Kortex mapping remains subject to its existing lower
  profile limits and is not enabled by this launcher.
- Kortex gripper writes require explicit gripper enablement; existing
  translation-only commands without that flag send no gripper RPCs.
- Fixed trajectories, read-only T0, calibration capture, PICO diagnostics, and
  source-only public release/bootstrap behavior remain unchanged.
- The public first-motion guide continues to keep gripper commands disabled;
  this advanced feature is documented separately with its own parameters and
  evidence boundary.

## TDD and Verification

Implementation must begin with failing tests that prove the absence of the
approved behavior, then add the minimum production changes. Tests must cover:

- Trigger `0.0`, intermediate values, and `1.0` mapping to proportional
  gripper positions;
- no gripper command before backend activation or while Grip is released;
- Grip release, stale recovery, boundary contact, shutdown, and re-clutch all
  preserve the last gripper position;
- deadband, minimum interval, clamping, generation checks, and RPC
  serialization;
- Stop priority over queued arm and gripper writes;
- non-finite Trigger and gripper RPC failure follow the fatal confirmed-Stop
  path;
- gripper writes never arm the Twist watchdog;
- activation remains anchor-only for arm motion;
- target projection covers all six workspace faces and never sends an
  out-of-box target;
- movement resumes from the boundary without process exit;
- scale `1.0`, maximum linear speed `0.05 m/s`, arbitrary finite ordered
  absolute bounds, and connection-before-validation rejection behavior;
- launcher argument propagation, one exact `MOVE`, masked credential cleanup,
  and absence of the duplicate readiness prompt;
- unchanged MuJoCo, fixed-trajectory, read-only, calibration, and non-gripper
  Kortex behavior.

Completion requires the focused tests, full offline suite, at least 80%
coverage, compile checks, a finite 2000-step MuJoCo run, PowerShell parsing,
diff and secret scans, and independent correctness and hardware-safety review.
No offline result is described as hardware validation. The first real gripper
trial begins with no payload and a small Trigger value while physical Stop
controls remain reachable.

