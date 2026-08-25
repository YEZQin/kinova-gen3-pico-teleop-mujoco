# Gen3 Recoverable Grip-Anchor Boundary Design

## Goal

Keep calibrated PICO teleoperation running when a target reaches the
per-Grip translation envelope. Instead of terminating with exit code 2 after
a confirmed Stop, the controller shall require the operator to release Grip
and then permit a fresh, feedback-based re-anchor on the next Grip press.

This change addresses hardware evidence from three trials in which motion was
successfully commanded for 434, 96, and 24 control cycles before the same
`target outside anchor translation envelope` condition. Every trial recorded
`stop_rpc_returned` and `cleanup_completed`, with no unconfirmed Stop.

## Approved Asymmetric Workspace and Limits

The latest read-only Kortex feedback pose is
`[0.743218601, -0.070251882, 0.057065614]` m. The onsite operator explicitly
approved 600 mm in both directions on X and Y, 40 mm downward on Z, and
600 mm upward on Z. The resulting absolute bounds are:

```text
minimum_m = [ 0.143218601, -0.670251882, 0.017065614]
maximum_m = [ 1.343218601,  0.529748118, 0.657065614]
```

This is a 1.2 m total X span, 1.2 m total Y span, and asymmetric 0.64 m Z
span. The limits are an operator-confirmed collision-clear software box; they
do not claim that every point is kinematically reachable by the arm.

- The per-Grip translation envelope remains ±50 mm per robot axis.
- Translation scale remains 0.8, maximum linear speed is raised to 0.02 m/s,
  control rate remains 40 Hz, and stale timeout remains 0.2 s.
- Translation-only mapping and the accepted operator-axis calibration remain
  unchanged.
- Exact `MOVE`, source locking, post-`MOVE` release recheck, watchdog, motion
  lease, reviewed preflight, and cleanup remain mandatory.
- No unbounded workspace is introduced. The approved asymmetric bounds above
  remain mandatory on every target.

## Boundary Behavior

For a target outside the per-Grip translation envelope or the approved
absolute workspace:

1. Reject the target before feedback or `SendTwistCommand`.
2. Request Kortex Stop and require the Stop RPC to return successfully.
3. Record a stable boundary-rejection evidence event identifying whether the
   per-Grip or absolute workspace boundary rejected the target.
4. Put the controller clutch into `waiting_for_release` without terminating
   the process.
5. While Grip remains pressed, transmit no target commands.
6. Require the configured stable run of fresh released samples before
   returning to `ready`.
7. On a later Grip press, execute the existing `begin_control` transaction,
   read fresh robot feedback, and use that pose as the new anchor. The
   activation cycle remains anchor-only and sends no Twist.

If Stop is not confirmed, the condition remains fatal and the program exits.
An absolute workspace rejection may recover only after a confirmed Stop and
fresh release/re-clutch sequence; it never clamps or transmits an out-of-box
target. Invalid or changed input source, Kortex RPC failure, watchdog failure,
robot fault, and cleanup failure retain their current fatal behavior.
Rotation-envelope rejection is unchanged because the onsite mode is
translation-only.

## Interface and Ownership

The backend result gains an explicit immutable signal indicating that motion
was safely stopped and a new Grip release is required. The Kortex backend
owns target validation and Stop confirmation. The teleoperation controller
owns the clutch transition. The pose mapper exposes a small transition that
clears its active references and enters `waiting_for_release` while preserving
its last finite target until a new feedback anchor is obtained.

This keeps vendor RPC and safety-state ownership out of the pose mapper and
avoids matching human-readable error strings in the controller.

## Verification

TDD must demonstrate:

- the old implementation fails a test expecting a confirmed anchor-boundary
  Stop to remain recoverable;
- the rejected target never reaches `SendTwistCommand`;
- exactly one Stop is requested for the boundary event;
- pressed samples after rejection cannot resume motion;
- fresh stable Grip release returns the clutch to `ready`;
- the next press re-arms feedback and anchors without sending Twist;
- an unconfirmed Stop remains fatal;
- absolute workspace rejection sends no Twist and follows the same confirmed
  Stop/release/re-anchor recovery;
- the exact X/Y/Z bounds above are accepted while any target outside them is
  rejected;
- the new 0.02 m/s limit is accepted only by the approved calibrated
  responsive hardware profile, while larger values and other profiles retain
  their existing lower caps;
- default simulation behavior and all existing Kortex safety tests remain
  unchanged.

Completion requires the full offline suite, at least 80% coverage, finite
2000-step MuJoCo execution, compile checks, secret/diff scans, and independent
correctness plus hardware-safety review. These are offline claims only; the
new recovery behavior remains hardware-unverified until an onsite trial.
