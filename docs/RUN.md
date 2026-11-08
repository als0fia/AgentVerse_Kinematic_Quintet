# Run Antenna — reproducible release guide

**Current submission candidate:** release/agentverse-2026-submission, based on green Presentation Mode commit **60c7283**. The single runtime is **src/ricebot/** with MuJoCo 4.0.0; the historical **src/riceswarm_ai/** calendar experiment is not the Antenna demo.

## 1. Install and verify on Windows

Use Python **3.11 or newer** from the machine where the video will be recorded. The independent release checks used Python **3.13.14** on Windows. Run from the cloned repository's root:

~~~powershell
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r requirements-mujoco.txt
.\.venv\Scripts\python.exe scripts\check_repo.py
.\.venv\Scripts\python.exe scripts\sync_skills.py --check
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
~~~

If the repository already has a compatible virtual environment, reuse it only in your **own isolated worktree**; never modify a teammate's environment. Python's Tkinter installation and a functioning display/OpenGL context are needed for the native GUI. Headless CI does not validate laptop graphics.

**Expected reference evidence:** the Windows release verification at **32ce0a3** reported **142 passed, none skipped**; the later Presentation Mode branch at **60c7283** passed **149 Linux CI tests with 2 headless Tk skips**. Re-check results on the actual pinned submission SHA instead of treating those earlier runs as proof of an untested later commit.

## 2. Launch the judge-facing live simulation

~~~powershell
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --presentation --scenario golden_rescue --policy cooperative --rescue-policy react_orchestrator --seed 7
~~~

This uses the actual continuous MuJoCo episode and the controlled cooperative rescue fixture. Presentation Mode gives camera presets, scene events, measured trails and live overlays without changing the control policy. Use **Engineering / Advanced** to reveal instrumentation.

For a high-risk refusal:

~~~powershell
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --presentation --scenario high_risk_abstain --policy cooperative --seed 7
~~~

For randomized cooperative or decentralized missions:

~~~powershell
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --dashboard --scenario randomized --policy mycelial --seed 8
~~~

Use the engineering controls to select **2 / 4 / 6 rovers**, **1.0× / 1.6×** field scale and normal / intermittent / isolated communications. Apply to reconstruct the world. Expanded-field robots have common sampling/planting capabilities. The original small rescue benchmark preserves its reproducible historical initial inventories.

**The selected coordination policy is authoritative:** cooperative uses the shared-map controller; Mycelial uses local beliefs and simulated peer gossip. Do not call a shadow observer the acting policy.

## 3. Run episodes without a graphical window

~~~powershell
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --scenario golden_rescue --policy cooperative --seed 7 --steps 220 --headless
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --scenario high_risk_abstain --policy cooperative --seed 7 --steps 220 --headless
~~~

These print JSON episode evidence rather than a GUI. Do not confuse a scripted benchmark fixture with an arbitrary randomized success guarantee.

## 4. Compare measured fleet/scaled-field outcomes

Short smoke experiment:

~~~powershell
.\.venv\Scripts\python.exe scripts\run_scaled_field_study.py --seeds 8 --fleets 4 --scales 1.6 --policies mycelial --comms normal isolated --rescue-policies react_orchestrator --steps 40 --out artifacts\quick-study.json
~~~

Full 18-episode matched study used for the supplied [recorded evidence](../experiments/release_verification_20261108.json):

~~~powershell
.\.venv\Scripts\python.exe scripts\run_scaled_field_study.py --seeds 3 8 --fleets 2 4 6 --scales 1.6 --policies mycelial cooperative --comms normal isolated --rescue-policies react_orchestrator --steps 240 --out artifacts\release-study.json
~~~

The study records real simulated physical motions, site contacts, rescue outcomes and peer traffic. Its dataset has two seeds and is **not** a statistically powered performance trial.

A separate model-comparison runner exists at **scripts/compare_orchestrator_models.py**. It requires an explicit paid-call flag and provider key, and enforces a maximum call count. **Do not execute live-model calls or share keys without team authorisation.** The 18-run benchmark made zero actual LLM requests.

## 5. Check graphical behaviour before filming

Open the GUI on the same Windows machine and resolution intended for capture. Verify:

- The 3D MuJoCo bodies actually move, and the 2D view tracks the same robot poses.
- Golden rescue shows incident evidence, helper assignment, latch/extraction and real post-rescue fieldwork.
- High-risk refusal dispatches no unsafe helper.
- Present/Advanced toggle and Field/Rescue/Auto camera controls respond without changing physical state.
- Pause, play, restart and changed seed leave no stale map, misplaced rover, frozen rendering, or incorrect labels.
- A six-rover 1.6× randomized world shows real physical bounds, distributed sites and active work rather than a six-icon overlay.
- No incorrect outcome statement is shown before a physical event occurs; disabled/stuck sensor faults do not masquerade as successful rescue.

For the submission video, record a **single continuous 60-second-or-shorter take** with legible perception/commands and a mid-episode change. Follow [SUBMISSION](SUBMISSION.md). Neither source nor CI can certify the final screen recording.

## 6. Troubleshooting and safety

- **Tkinter / TclError / no display:** run the native GUI on a graphical Windows host. Linux CI intentionally skips GUI-only tests; use headless commands in CI.
- **MuJoCo/OpenGL renderer unavailable:** check local graphics drivers/OpenGL support; verify a tiny physics episode first with **scripts/smoke_mujoco.py**. Do not replace actual MuJoCo output with a fabricated video.
- **Incorrect branch or mixed changes:** print your commit SHA and run in a fresh isolated worktree. Do not reset, stash or clean another contributor's checkout.
- **Different scenario seed:** reconstruct/rebind the session and renderer when changing seed. Record the seed, fleet size, map scale and controller for a reproducible run.
- **Unsafe physical decision:** the safety gate bounds rescue commands, but the simulations still record collisions and secondary casualties. Do not claim zero-risk autonomy.
- **No LLM response:** the offline deterministic fallback is expected. No network call is evidence of a successful LLM inference.

## 7. What is deliberately not part of the demo

The legacy **src/riceswarm_ai/** seasonal farming UI, Isaac Sim translation proposals, real UWB hardware, Jetson deployments, full RL training, and numerical market forecasts are not required to launch this Antenna simulation. The complete source archive remains in Git history and other documents for tracing contributor work.

See [Architecture](ARCHITECTURE.md), [Testing](TESTING.md), [Provenance](PROVENANCE.md) and the [Submission checklist](SUBMISSION.md).

## Historical anchors retained for older development notes

### Current minimum working rescue integration assignment

The historical "integrate the first rescue" assignment is completed for the **controlled cooperative golden fixture**. Use Sections 2, 3 and 5 above for its current reproducible launch and live proof.

### Recovery integration gate (separate from existing MuJoCo mechanical test)

Current simulated rescue, physical outcomes and limitations are described in [Testing](TESTING.md); the old M1 gate is superseded.

### MuJoCo rapid prototyping SimulationPort & Isaac Sim translation protocol

MuJoCo is the release runtime. Isaac Sim migration has **not** been implemented or verified.

### Next agent first integration task — Isaac Sim planting first skeleton not yet implemented

Historical link compatibility only. It is superseded by the release candidate above; do not start Isaac development for this submission.
