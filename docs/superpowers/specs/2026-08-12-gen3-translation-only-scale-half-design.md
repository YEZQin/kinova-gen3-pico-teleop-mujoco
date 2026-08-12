# Gen3 Translation-Only Scale 0.5 Design

## Context

The first PICO-to-Gen3 translation-only hardware trial completed four Grip
activation/release cycles at scale `0.003`. Its evidence contains 626 guarded
motion updates, four returned Stop RPCs, no workspace or fault events, and a
completed cleanup. The operator reported that motion was too small to observe
comfortably and selected scale `0.5` with an absolute workspace centered on the
latest measured tool pose and extending 10 mm in each direction on X, Y, and Z.

The latest read-only tool pose is:

```text
position_m = [0.711899340, -0.052683271, 0.032770045]
angles_deg = [97.018997192, -9.022003174, 78.733444214]
```

## Approved behavior

- Keep the Kortex default translation scale at `0.25`.
- Keep the full 6DoF mapping limit at `0.25`.
- Permit an explicit scale up to and including `0.5` only when
  `--translation-only` is active.
- Reject scale values above `0.5` before password lookup, SDK import, motion
  confirmation, or robot connection.
- In translation-only mode, retain the tool orientation captured on each Grip
  activation. PICO controller rotation never becomes a Kortex target rotation.
- Keep maximum linear speed at `0.005 m/s`, maximum angular speed at `2 deg/s`,
  control rate at 40 Hz, and stale timeout at 0.2 seconds.
- Use absolute XYZ bounds centered on the latest measured pose with 10 mm in
  each direction:

  ```text
  minimum_m = [0.701899340, -0.062683271, 0.022770045]
  maximum_m = [0.721899340, -0.042683271, 0.042770045]
  ```

- Keep the existing 20 mm-per-axis anchor-relative translation envelope. The
  tighter 10 mm absolute half-width remains authoritative for this trial.
- Keep Grip admission, exact `MOVE`, source continuity, finite/fresh input,
  watchdog, workspace validation, latched faults, Stop, and cleanup behavior
  unchanged.
- Do not enable gripper commands, automatic actions, low-level cyclic control,
  or any watchdog/freshness bypass.

## Validation and failure behavior

Argument tests must establish all three scale boundaries:

1. full-pose Kortex at `0.5` is rejected;
2. translation-only Kortex at `0.5` is accepted;
3. translation-only Kortex above `0.5` is rejected.

Existing tests must continue to prove that the Kortex default is `0.25` and
that translation-only preserves mapped position while holding the Grip anchor
orientation. The focused Kortex/PICO suites, full offline suite, and 2000-step
MuJoCo dry run must remain green.

The onsite launcher must use a new evidence path, run the PICO-only freshness
gate first, obtain the password through a masked prompt, clear it in `finally`,
and require exact `MOVE`. Any invalid/stale/source-changed input, workspace
violation, RPC failure, watchdog failure, or unconfirmed Stop remains fatal.

Before the launcher is presented, the onsite operator must explicitly confirm
that the arm is stationary and the full XYZ plus-or-minus 10 mm workspace and
sweep volume are physically clear, with the E-stop reachable, the second
observer present, and Grip released.
