# Gen3 Responsive Reversed Translation Teleoperation Design

## Context

The expanded scale-`0.5` translation-only hardware run achieved sustained
PICO-to-Gen3 motion. The onsite operator observed that controller right/left,
up/down, and forward/back translations all moved the robot in the opposite
direction. The same run felt slow and ended after a transient PICO input gap
crossed the existing 0.2 second freshness boundary.

Offline diagnosis traces all translation signs to the relative mapping before
the Kortex backend; the backend sends the resulting velocity in the Kortex
base frame without another basis conversion. A separate 30 second PICO-only
observation recorded two recoverable stale boundaries with no rejected or
foreign-source packets. Hardware evidence records that Stop was requested and
cleanup completed. These observations do not establish safety-rated stopping
performance.

The onsite operator approved an explicit responsive translation-only profile:
translation scale `0.8`, maximum linear speed `0.01 m/s`, and reversal of all
three mapped translation axes. The existing axis-aligned absolute workspace
and per-Grip anchor envelope remain 50 mm in each direction.

## Chosen approach

Add an explicit responsive Kortex mode and an explicit all-axis translation
inversion. They are valid only for live-input, expanded-envelope, Kortex
translation-only hardware teleoperation. Existing MuJoCo, full-pose, ordinary
translation-only, fixed-trajectory, and read-only paths retain their current
defaults and limits.

The relative translation becomes:

```text
target = tool_anchor
       + scale * axis_sign * (mapped_controller - controller_anchor)
```

where the approved hardware launcher uses `axis_sign = [-1, -1, -1]`.
Orientation remains fixed to the current Grip anchor. The sign operation is
applied only to the translation delta; it does not create an improper
quaternion basis or alter global `PICO_TO_WORLD` behavior.

Responsive mode permits scale at most `0.8` and maximum linear speed at most
`0.01 m/s`. Without that explicit mode, the current translation-only scale cap
of `0.5` and Kortex speed cap of `0.005 m/s` remain unchanged. Control remains
40 Hz, angular speed remains at most 2 deg/s, and stale timeout remains at
most 0.2 seconds.

## Recoverable stale boundary

The 0.2 second freshness boundary is not raised or bypassed. When stale input
is observed during active control, the host immediately calls the existing
backend `hold()`/Kortex Stop path and resets the clutch. The process stays
alive only if Stop returns successfully.

Recovery requires three distinct, increasing-timestamp, finite valid samples
with Grip below the release threshold. Pressed or repeated-timestamp samples
do not advance recovery. The operator must then press Grip again; that press
runs a new `begin_control()` transaction and captures new controller and tool
anchors. The previous target is never resumed automatically.

Malformed/untracked input, source change, timestamp regression, an
unconfirmed Stop, Kortex fault, workspace rejection, or anchor-envelope
rejection remains fatal and closes the process. Stale and recovery events are
recorded in the existing evidence schema.

## Safety boundaries

The approved absolute workspace remains centered on the previously inspected
tool pose and spans 100 mm per axis:

```text
minimum_m = [0.660730433, -0.104256245, -0.020250260]
maximum_m = [0.760730433, -0.004256245,  0.079749740]
```

Each Grip anchor remains limited to 50 mm per mapped base axis. Targets remain
rejected rather than clamped. Exact `MOVE`, PICO release/freshness admission,
post-`MOVE` recheck, motion lease, reviewed preflight report, command watchdog,
Stop confirmation, and cleanup all remain mandatory. Gripper commands, Home,
stored actions, and low-level cyclic control remain disabled.

Human observation, the Web Stop, and the physical E-stop are independent
controls; they are not substitutes for the software workspace or freshness
checks.

## Validation and onsite handoff

Tests must first fail against the current implementation and then prove:

1. all three relative translation axes reverse only when explicitly selected;
2. default mapping and MuJoCo behavior are unchanged;
3. responsive mode accepts exact scale `0.8` and `0.01 m/s`, while overruns or
   invalid mode combinations reject before password, SDK import, `MOVE`, or
   connection;
4. a stale active sample calls Stop and leaves the controller running in a
   release-first recovery state;
5. three distinct released samples are required before `READY`, then a new
   Grip press is anchor-only and does not resume the old target;
6. invalid/source-changed input and Stop failure remain fatal;
7. workspace, anchor, watchdog, admission, and cleanup regressions stay green.

Before onsite use, the complete offline suite, at least 80 percent coverage,
compile checks, a 2000-step finite MuJoCo run, and independent correctness and
security reviews must pass. A new launcher and strict preflight report must
bind a clean implementation commit, the exact flags and bounds, and a new
unused evidence path. Offline readiness is not hardware validation. The first
run must verify each direction separately using a few millimetres of travel
and record Grip-release, stale-recovery, and process-exit stop observations.
