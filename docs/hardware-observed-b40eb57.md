# Hardware-observed tuned PICO gripper teleoperation baseline

This note preserves the onsite baseline that the operator reported as “basically meets requirements” on 2026-08-24. It is installation-local evidence, not a universal safety claim.

## Baseline identity

- Core control commit observed onsite: `b40eb57f382068153299041ffefe30542778b541` (`feat: add binary burst gripper control`).
- Publication branch: `codex/public-gen3-pico-hardware-teleop`.
- Target observed installation: vision-equipped Kinova Gen3 L53, 7 DoF, firmware `2.8.0-5`, Kortex API `2.8.0.post5`, protobuf `3.20.0`, Python `3.11.15`, PICO 4 Ultra left controller, and a gripper exposed by Kortex as one logical finger actuator. Vision was not commanded.
- Publication follow-up commits may add documentation, tests, and reusable launch wrappers; they must not be described as separately hardware-observed unless another onsite run records that fact.

## Tuned operator semantics

- Grip is the arm-motion clutch.
- Translation is operator-axis calibrated, translation-only, responsive, and stale-input recoverable.
- The observed tuned preset uses `Scale 1.0`, `MaxLinearSpeed 0.05`, `LinearGain 1.5`, and `TranslationAxisGain @(-2.0, 1.0, 1.0)`.
- Negative X corrects the observed forward/back direction for that calibration; Y and Z remain positive after operator-axis calibration.
- Trigger is intentionally binary in the observed profile: `Trigger > 0.9` commands gripper `0.99` closed; `Trigger <= 0.9` commands gripper `0.01` open.
- A changed binary gripper target starts a 1.25 s Kortex position-command burst, reissued every active control cycle, then deduplicated.
- Releasing Grip stops arm following and leaves the current gripper command unchanged.

## Observed outcome

- End-effector following was reported usable after the negative-X, linear-gain, and speed tuning.
- Binary Trigger gripper control was reported usable after switching away from slow proportional/velocity-style control.
- Earlier direct gripper measurement showed position command `0.99` reached approximately `0.9869`; this supported the final position-command burst design.
- The preserved 155.172 s evidence stream contains 1,216 `moving` records, 414 successful `gripper_commanded` records with targets only `0.01` and `0.99`, 15 `control_started` / `input_release` clutch cycles, 15 returned Stop RPC records, six workspace-clamp records, and one completed cleanup record.
- The raw evidence stream, robot-local package, IP-bound profile, and onsite session directories are intentionally excluded from Git; this note retains only the non-secret aggregate.
- The operator closed the session by approving this version for preservation and GitHub publication.

## Boundary

- This is not safety-rated, production-safe, unattended, or generalized hardware validation.
- A returned Stop RPC record is not proof of physical stationarity, and a workspace clamp is not a safety-rated workspace controller.
- New robots, firmware, tools, payloads, workspaces, calibrations, or PICO tracking setups need fresh calibration, read-only T0, package generation, and onsite observation.
- Do not publish local session logs, credentials, private lab paths, APK binaries, SDK binaries without redistribution rights, or robot-specific generated `local-config` artifacts.
