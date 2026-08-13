# Gen3 Expanded Translation Envelope Design

## Context

The first scale-`0.5`, translation-only PICO-to-Gen3 hardware run reached the
guarded motion path and then stopped when its mapped target exceeded the
approved absolute XYZ workspace. The evidence records approximately 1.11
seconds between the first moving event and the workspace fault, a returned
Stop RPC approximately 16 ms after the host Stop request, and completed
cleanup. This proves the software fault and Stop-request path for that run; it
does not establish physical stopping performance beyond the operator's onsite
observation.

After that stop, the latest read-only tool pose was:

```text
position_m = [0.710730433, -0.054256245, 0.029749740]
angles_deg = [96.850051880, -9.495429993, 78.643257141]
```

The onsite operator confirmed that the arm is fixed and that the complete
tool, gripper, payload, cable, and robot-link sweep volume is clear throughout
an axis-aligned workspace extending 50 mm in both directions on X, Y, and Z
from this pose. The operator also confirmed that the E-stop and Web Stop are
reachable, a second observer is present, and Grip is released.

## Chosen approach

Add an explicit `--expanded-translation-envelope` hardware option. It is valid
only when all of the following are true:

- the backend is `kortex`;
- hardware is explicitly enabled;
- `--translation-only` is active; and
- explicit workspace minimum and maximum values are supplied.

The option selects an anchor-relative translation limit of 50 mm per axis and
allows an absolute workspace span of at most 100 mm per axis. Without the
option, the existing 20 mm-per-axis anchor envelope and 40 mm maximum absolute
span remain unchanged. Full-pose operation can never select the expanded
envelope.

This explicit option is preferred over automatically expanding every
translation-only session or globally changing the first-hardware profile. It
keeps existing launchers and defaults unchanged and makes the larger physical
authorization visible in the command line and preflight evidence.

## Components and data flow

The hardware profile will expose a separate immutable translation-only
expanded limit of `(0.05, 0.05, 0.05)` metres. Argument validation will choose
the permitted workspace span from the explicit option before password lookup,
SDK import, `MOVE`, or any robot connection. The same resolved limit will be
passed to `AnchorEnvelope`, ensuring that pre-connect workspace validation and
runtime anchor validation use one value rather than duplicated constants.

The approved absolute workspace is centered on the latest read-only pose:

```text
minimum_m = [0.660730433, -0.104256245, -0.020250260]
maximum_m = [0.760730433, -0.004256245,  0.079749740]
```

The control path remains:

```text
PICO freshness/release gate
  -> read-only Kortex readiness
  -> exact MOVE confirmation
  -> post-MOVE PICO release/source/freshness recheck
  -> Grip anchor
  -> translation-only target mapping
  -> absolute workspace check
  -> anchor-envelope check
  -> bounded Twist command
```

Translation scale remains `0.5`, maximum linear speed remains `0.005 m/s`,
maximum angular speed remains `2 deg/s`, control rate remains 40 Hz, and stale
timeout remains 0.2 seconds. Translation-only mode continues to hold the tool
orientation captured at each Grip anchor. Gripper commands remain disabled.

## Failure behavior

Invalid combinations of the expanded option fail before credential access or
any network connection. A workspace span above 100 mm on any axis also fails
at that boundary. At runtime, a target beyond either the explicit absolute
workspace or the 50 mm anchor-relative envelope remains a fatal latched safety
fault: the backend requests Stop and the process exits. Targets are not
silently clamped, and the process does not automatically recover or reconnect.

All existing release-first, finite-value, source-continuity, stale-input,
watchdog, motion-lease, preflight-report, exact-`MOVE`, Stop-confirmation, and
cleanup behavior remains unchanged. The expanded option does not enable Home,
action playback, gripper writes, low-level cyclic control, or any watchdog or
freshness bypass.

## Validation

Tests must be written before implementation and must prove:

1. default and full-pose Kortex operation retain the 20 mm anchor envelope and
   40 mm maximum workspace span;
2. `--expanded-translation-envelope` is rejected without Kortex
   translation-only hardware mode, before password lookup or connection;
3. expanded translation-only mode accepts an exact 100 mm span and passes a
   50 mm-per-axis `AnchorEnvelope` to the backend;
4. a span above 100 mm is rejected before password lookup or connection;
5. the exact 50 mm anchor boundary is inclusive and an overrun is rejected;
6. existing Grip, PICO fault, workspace, watchdog, Stop, and cleanup tests stay
   green.

Before onsite use, the focused suites, full offline suite, coverage threshold,
and 2000-step finite MuJoCo run must pass. A new strict preflight report and a
new launcher must bind the clean implementation revision, the current pose,
the exact bounds above, scale `0.5`, translation-only mode, the explicit
expanded-envelope option, and a previously unused evidence path. The launcher
must retain the masked password prompt, exact `MOVE`, and `finally` cleanup.

Scale-`0.5` operation with the expanded workspace remains unvalidated on real
hardware until a new guarded run records the physical motion direction and the
observed Grip-release stop.
