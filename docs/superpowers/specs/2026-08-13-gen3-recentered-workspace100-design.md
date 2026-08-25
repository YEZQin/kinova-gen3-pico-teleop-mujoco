# Gen3 Recentered 100 mm Workspace Design

## Observed failure and retained evidence

The calibrated scale-0.8 translation-only hardware trial entered motion and
recorded 24 `MOVING` events before the host rejected a target outside the
absolute workspace. The host requested Stop, the Kortex Stop RPC returned,
and cleanup completed. This is hardware-observed control and Stop evidence;
it does not validate all physical motion directions.

The prior absolute workspace was centered at
`[0.710730433, -0.054256245, 0.029749740]` m with 50 mm per-axis half-width.
The post-trial read-only tool pose was
`[0.746914864, -0.082188472, 0.064038418]` m. It was only 13.8 mm from the
old positive-X boundary, 22.1 mm from the old negative-Y boundary, and
15.7 mm from the old positive-Z boundary. The failure therefore reflects an
obsolete absolute center, not a failed Stop path or a persistent robot fault.

The original evidence file
`results/gen3-pico-calibrated080-events-20260813.jsonl` remains unchanged.
The old launcher intentionally refuses to append to it and will remain
unchanged as an audit artifact.

## Approved workspace

The onsite operator explicitly confirmed that the complete new 200 mm XYZ
cube is clear, including the arm links, gripper, cables, payload, and the
lowest Z plane. The new absolute workspace is centered on the read-only pose:

```text
center_m = [0.746914864, -0.082188472,  0.064038418]
minimum_m = [0.646914864, -0.182188472, -0.035961582]
maximum_m = [0.846914864,  0.017811528,  0.164038418]
```

Each center-to-bound distance is exactly 0.100000000 m. This enlarges only
the absolute Cartesian workspace. The per-Grip anchor translation envelope
remains 0.050 m per axis and the anchor rotation envelope remains 5 degrees.
Targets outside either boundary continue to latch a fault, request Stop, and
exit rather than clamp or continue.

## Repeatable evidence paths

Create a new launcher instead of overwriting the prior package. At each
invocation, before PICO or password admission, it constructs one evidence
filename under the existing `results` directory from a UTC timestamp and a
random GUID:

```text
gen3-pico-calibrated080-workspace100-events-<UTC>-<GUID>.jsonl
```

The timestamp is filesystem-safe and the GUID prevents same-tick collisions.
The launcher requires the generated path to be absent and never deletes,
truncates, renames, or reuses any evidence file. The same immutable path is
passed to the child process for that invocation. A later run generates a new
path automatically, so a completed or faulted run cannot block relaunch.

The reviewed preflight report records this evidence naming contract rather
than claiming a particular future filename already exists.

## Unchanged control and admission behavior

The new launcher retains the accepted operator-axis calibration artifact and
all current control values:

```text
translation scale = 0.8
maximum linear speed = 0.01 m/s
control rate = 40 Hz
stale timeout = 0.2 s
maximum angular speed = 2 deg/s
translation-only = true
responsive stale recovery = true
```

It continues to require fresh released PICO input, current physical checklist
confirmation, a masked child-only password, reviewed preflight, motion lease,
live read-only Kortex readiness, exact `MOVE`, and a post-`MOVE` Grip/source
recheck before motion construction. Grip release, stale input, source change,
workspace rejection, anchor rejection, watchdog expiry, cancellation, and
cleanup retain their existing Stop behavior. No gripper command, Home, stored
action, low-level cyclic control, timeout increase, or watchdog bypass is
introduced.

## Validation and onsite procedure

Offline validation must prove the new workspace arithmetic, new launcher
tokens, unique evidence generation, preservation of old evidence, child-only
credential cleanup, PowerShell syntax, strict preflight/lease/calibration
loading, and failure propagation. Existing full tests, coverage, compilation,
and 2000-step finite MuJoCo evidence remain applicable because production code
does not change; focused launcher/package checks must be run freshly.

The new report must bind clean implementation HEAD `8122b486c8eae87c6ef56006b9fee388c86af701`,
Python 3.11.15, and calibration SHA-256
`682ff4dbe5ad572d9e2a66828fd823edcbe6290ee3e860db322821ab65028c02`.
It must distinguish the prior hardware-observed workspace Stop from the still
pending validation of the enlarged workspace and calibrated directions.

At the next trial, the operator reconfirms the current physical checklist,
keeps Grip released through admission, types exact `MOVE`, and tests one
direction at a time with only a few millimetres of hand motion. Grip is
released immediately on any mismatch. The generated evidence path is printed
after the child exits so it can be inspected without searching the directory.
