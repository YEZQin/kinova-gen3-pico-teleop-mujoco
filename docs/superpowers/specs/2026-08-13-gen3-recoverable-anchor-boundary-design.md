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

## Retained Limits

- The absolute Cartesian workspace remains the already approved XYZ ±100 mm
  box centered at `[0.746914864, -0.082188472, 0.064038418]` m.
- The per-Grip translation envelope remains ±50 mm per robot axis.
- Translation scale remains 0.8, maximum linear speed remains 0.01 m/s,
  control rate remains 40 Hz, and stale timeout remains 0.2 s.
- Translation-only mapping and the accepted operator-axis calibration remain
  unchanged.
- Exact `MOVE`, source locking, post-`MOVE` release recheck, watchdog, motion
  lease, reviewed preflight, and cleanup remain mandatory.
- No unbounded workspace or ±600 mm workspace is introduced.

## Boundary Behavior

For a target outside the per-Grip translation envelope only:

1. Reject the target before feedback or `SendTwistCommand`.
2. Request Kortex Stop and require the Stop RPC to return successfully.
3. Record an `anchor_rejected` evidence event with the stable reason.
4. Put the controller clutch into `waiting_for_release` without terminating
   the process.
5. While Grip remains pressed, transmit no target commands.
6. Require the configured stable run of fresh released samples before
   returning to `ready`.
7. On a later Grip press, execute the existing `begin_control` transaction,
   read fresh robot feedback, and use that pose as the new anchor. The
   activation cycle remains anchor-only and sends no Twist.

If Stop is not confirmed, the condition remains fatal and the program exits.
Absolute workspace rejection, invalid or changed input source, Kortex RPC
failure, watchdog failure, robot fault, and cleanup failure retain their
current fatal behavior. Rotation-envelope rejection is unchanged because the
onsite mode is translation-only.

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
- absolute workspace rejection remains fatal;
- default simulation behavior and all existing Kortex safety tests remain
  unchanged.

Completion requires the full offline suite, at least 80% coverage, finite
2000-step MuJoCo execution, compile checks, secret/diff scans, and independent
correctness plus hardware-safety review. These are offline claims only; the
new recovery behavior remains hardware-unverified until an onsite trial.
