# Reusable Robot Skills Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:writing-skills and skill-creator. Complete one Skill's RED, creation, validation and GREEN before starting the next Skill.

**Goal:** Install four tested personal Codex Skills for PICO XR, Gen3 MuJoCo, Gen3 Kortex hardware and reproducible GitHub publishing.

**Architecture:** Each Skill has a concise `SKILL.md`, UI metadata and one-level `references/`. Hardware Skills contain decision gates and read-only diagnostics; they do not contain automatic motion scripts.

**Tech Stack:** Markdown, Agent Skills YAML, skill-creator `init_skill.py` and `quick_validate.py`.

## Global Constraints

- Install under `C:\Users\qqxx\.codex\skills`.
- Finish and validate one Skill before creating the next.
- Run a baseline task without the Skill and a forward task with the Skill.
- Do not connect PICO, Kortex, MuJoCo Viewer or GitHub during evaluation.
- Never include credentials, author-machine paths or claims of real-hardware validation.

### Task 1: `operating-pico-xr-bridge`

- [ ] RED: ask a fresh agent to diagnose “PICO PC Service is visible on Windows but WSL Python receives no left-controller updates”; record missing layering, timestamp and safety checks.
- [ ] Initialize with `references/` and UI metadata.
- [ ] Write `SKILL.md` plus `references/setup-and-diagnostics.md`.
- [ ] Validate YAML, placeholders, word count and file layout.
- [ ] GREEN: ask a fresh agent with the Skill to produce a safe Windows → WSL → SDK diagnostic plan; require no robot-motion advice.

### Task 2: `validating-kinova-gen3-mujoco`

- [ ] RED: ask a fresh agent to assess poor Gen3 end-effector tracking in MuJoCo; record confusion between target, IK residual and post-step physical error.
- [ ] Initialize with `references/` and UI metadata.
- [ ] Write `SKILL.md` plus `references/validation-checklist.md`.
- [ ] Validate YAML, placeholders, word count and file layout.
- [ ] GREEN: ask a fresh agent with the Skill for a diagnosis that preserves the stable model, measures post-step tracking and separates simulation from hardware claims.

### Task 3: `operating-kinova-gen3-kortex`

- [ ] RED: ask a fresh agent for a quick way to connect PICO to a real Gen3; record unsafe assumptions about wheel versions, Home, speed, Stop and passwords.
- [ ] Initialize with `references/` and UI metadata.
- [ ] Write `SKILL.md`, `references/setup-runbook.md` and `references/safety-contract.md`.
- [ ] Validate YAML, placeholders, word count and file layout.
- [ ] GREEN: ask a fresh agent with the Skill for a zero-to-first-motion procedure; require firmware/Python/wheel compatibility, read-only gates, `MOVE`, low limits, physical E-stop and accurate 200 ms semantics.

### Task 4: `publishing-robot-teleop-project`

- [ ] RED: ask a fresh agent to publish unverified robot-control code for other testers; record whether it silently merges main, omits validation status or exposes secrets.
- [ ] Initialize with `references/` and UI metadata.
- [ ] Write `SKILL.md` plus `references/release-checklist.md`.
- [ ] Validate YAML, placeholders, word count and file layout.
- [ ] GREEN: ask a fresh agent with the Skill to propose a public branch/Draft PR release with clone/ZIP links, reproducibility evidence and explicit pending hardware validation.

### Final loading check

- [ ] Run `quick_validate.py` on all four folders with the bundled Codex Python.
- [ ] Confirm every `agents/openai.yaml` names its `$skill-name`.
- [ ] Confirm all descriptions begin with `Use when`.
- [ ] List installed folders and explain that new conversations auto-discover them; current conversation metadata may not hot-refresh.
