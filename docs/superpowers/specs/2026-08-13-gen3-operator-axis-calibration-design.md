# Gen3 Operator-Axis Calibration Design

## Context and evidence

The responsive translation-only trial proved that PICO input, Grip clutching,
Kortex motion, confirmed Stop, and stale-input recovery all work on hardware.
The remaining failure is coordinate alignment: the fixed `PICO_TO_WORLD` basis
assumes a headset/operator heading that does not match the onsite setup. An
all-axis sign flip cannot correct a rotated basis and produced mixed targets
that eventually reached the existing workspace boundary.

The PICO-only capture
`results/pico-operator-axes-20260813-051504.json` records one shared neutral and
three deliberate gestures. It never connected to Kortex. The gesture lengths
are 83.4 mm right, 110.4 mm up, and 126.2 mm forward. The maximum absolute
pairwise unit-vector dot product is 0.190, the basis condition number is 1.258,
and the determinant is 0.973. These pass the capture's quality gates and are
sufficient to derive a stable operator basis. This is PICO-only evidence, not
hardware validation of the corrected directions.

## Calibrated coordinate mapping

Let `A` contain the captured unit raw-PICO directions as columns ordered
`right`, `up`, `forward`. Compute its nearest proper orthogonal basis with the
polar decomposition:

```text
U, _, Vt = svd(A)
Q = U @ Vt
```

The loader rejects malformed JSON, non-finite or wrongly shaped matrices,
unexpected gesture labels, insufficient gesture displacement, left-handed or
ill-conditioned bases, and poor axis separation. It recomputes every quality
metric instead of trusting reported values.

The operator's directions map into the Kinova base frame as follows:

```text
operator right   -> robot -Y
operator up      -> robot +Z
operator forward -> robot -X
```

This is the same right-handed convention encoded by the existing
`PICO_TO_WORLD` matrix `P`. Because controller positions are already converted
to the default world basis before relative mapping, the additional proper
rotation applied to the controller delta is:

```text
C = P @ Q.T @ P.T
```

For the accepted capture, `C` is approximately:

```text
[[ 0.861288,  0.469278, -0.194835],
 [-0.470680,  0.881303,  0.042009],
 [ 0.191423,  0.055523,  0.979936]]
```

The mapped measured gestures are close to the intended base axes: right has
`-Y=0.998`, up has `+Z=0.994`, and forward has `-X=0.994`. Calibration changes
only the clutch-relative translation delta. Translation scale remains 0.8;
translation-only mode continues holding the Grip anchor orientation.

## Interfaces and scope

Add a strict calibration loader in a focused module and expose it through an
explicit `--operator-calibration PATH` CLI option. The immutable output is a
3-by-3 tuple rotation suitable for `TeleopConfig` and `MappingConfig`.

The option is allowed only for live-input, expanded-envelope, responsive,
translation-only Kortex hardware teleoperation with stale recovery enabled.
It is mutually exclusive with `--invert-translation`; exactly one of the old
inversion mode or the new calibration mode selects the responsive mapping.
Existing launchers therefore remain parse-compatible, while the calibrated
launcher cannot accidentally apply both transformations.

Default MuJoCo, full-pose, ordinary translation-only, read-only Kortex, fixed
trajectory, and the previous explicit inversion behavior remain unchanged.
Calibration is loaded and validated before password lookup, SDK import,
`MOVE`, network connection, or any robot RPC.

## Failure and safety behavior

Invalid calibration fails closed with a concise message and no hardware side
effect. The runtime does not reload the file after motion admission, so the
matrix used by control is the same immutable value that passed validation.

This change does not remove or enlarge the absolute workspace or the per-Grip
50 mm anchor envelope. It does not change the 0.01 m/s linear speed cap, 40 Hz
control rate, 0.2 second freshness boundary, source lock, Grip release-first
clutch, watchdog, exact `MOVE`, motion lease, reviewed preflight, Stop, or
cleanup behavior. The gripper remains out of scope.

## Verification and onsite handoff

Tests must first fail and then prove strict loader rejection, the derived
rotation's numerical axis expectations, default/inversion compatibility,
translation-only orientation retention, CLI side-effect ordering, and exact
config wiring. Focused controller/backend/admission tests, the complete offline
suite, coverage of at least 80 percent, compile checks, diff checks, and a
2000-step finite MuJoCo run must remain green.

After a clean implementation commit and independent correctness/security
review, generate a new strict preflight report and launcher bound to the exact
calibration file hash and code revision. The first hardware validation moves
only one direction at a time by a few millimetres and releases Grip immediately
on mismatch. Only observed motion can validate the final physical direction.
