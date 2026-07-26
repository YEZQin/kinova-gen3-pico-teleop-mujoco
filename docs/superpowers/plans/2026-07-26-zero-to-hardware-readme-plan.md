# Zero-to-Hardware README Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make README a complete WSL2 Ubuntu 22.04 runbook from public clone to the first low-speed Kinova Gen3 hardware trial.

**Architecture:** Keep the existing MuJoCo/PICO guide intact and expand the hardware section with one ordered deployment path. Link to Kinova's official wheel installation documentation while requiring firmware/Python compatibility instead of pinning an unsafe universal wheel.

**Tech Stack:** Markdown, WSL2 Ubuntu 22.04, Python 3.10 venv, XRoboToolkit, optional Kortex Python wheel.

## Global Constraints

- Documentation only; no production behavior or dependency changes.
- WSL2 Ubuntu 22.04 is the supported primary flow.
- Never include a real password or claim hardware validation.
- Kortex wheel version must match the target firmware and supported Python.
- First hardware limits remain `0.01 m/s` and `2 deg/s`.

### Task 1: Expand README and publish

**Files:**
- Modify: `README.md`

- [ ] Add public branch clone and WSL/Python/project installation commands.
- [ ] Add firmware discovery, official Kortex wheel download/install/import checks.
- [ ] Add Windows Ethernet/WSL TCP 10000 checks without an automatic network mutation command.
- [ ] Add the ordered MuJoCo → XR → Kortex import → TCP → safety → MOVE checklist.
- [ ] Add complete low-speed command, Grip operation, stop/restart and troubleshooting.
- [ ] Verify README arguments against CLI help.
- [ ] Run `python -m pytest -q`, 2000-step dry-run, compileall and `git diff --check`.
- [ ] Commit and push the documentation update to PR #2.
