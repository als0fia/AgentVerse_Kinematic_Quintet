# Reproducible setup, run and recovery

**Selected project is PaddyPatch cooperative rice-field stand-gap mapping, targeted replanting and physical seed placement.** **Isaac Sim is the user-selected simulator for now**, but exact version, machine/container, scene performance and hardware are **not yet verified**. Do not claim a running robot, working seed drop, GPU credits, model or deployment. Do not falsely claim installation, simulator/robot access, models, cloud credits, supported operating systems or a live demo.

## New teammate or agent onboarding

For **any fresh Windows/macOS/Linux machine**, start with [WORKSTATION-SETUP](WORKSTATION-SETUP.md): inspect/clone safely, create local `.venv`, run actual kernel checks, optionally verify a **toy MuJoCo physics joint/sensor** via `scripts/smoke_mujoco.py`, verify Antigravity/Gemini tool access, and gate Isaac Sim on real compatible GPU/RAM and permissions. Do not copy another teammate's private keys or assume their local prototype exists on this machine. Selected production target remains Isaac Sim; MuJoCo is optional local preflight/fallback by human decision. [COMPONENTS](COMPONENTS.md) owns world vs robot vs single Isaac integration writer and the module README standard.

## Implemented core check (no Isaac Sim required)

From repo root with Python 3.10+:

```bash
python -m unittest discover -s tests -p 'test_ricebot_foundation.py' -v
python -m unittest discover -s tests -v
```

These checks exercise **scripted sensor frames**, measured local soil fusion, simple tasks, bounded commands and an injected fake simulation-port response. **They do not verify Isaac Sim wheel motion, a physical hopper, landing contact or a working demo.**

**Next coding agent:** implement the actual Isaac Sim `SimulationPort` against `src/ricebot/contracts.py`; first a real wheel-driven rover and range sensor, then an **actual rigid-body seed drop and landing/contact feedback**. Use the installed Isaac version; report exact host/VRAM/driver, commands, physics traces and failures. The physical bridge must refuse expired/invalid drive and drop commands independently. Keep the hidden true soil field out of the observations.

**Scope cap:** about 3–6 planting sites, two simple rovers, a few synthetic local soil/stand readings, one physical drop, and one mid-run changed obstacle/site. No full farm, crop growth, flood mitigation or robot rescue.

## Foundation checks — actually available

```bash
python scripts/check_repo.py
python scripts/sync_skills.py --check
python -m unittest discover -s tests -v
python scripts/evaluate.py --candidate tests/fixtures/candidate.jsonl --baseline tests/fixtures/baseline.jsonl
```

Run `python scripts/sync_skills.py --write` after editing any canonical `.agents/skills/` file. The check command fails if the complete Claude mirrors diverge or skills are missing/extra. Python 3.10+ standard library only. These checks validate **repo structure and evaluation tooling**, not any robotics entry.

## Optional Gemini CLI development-agent preflight

The repository does **not** require Gemini/Argon to run its foundation checks. When a teammate elects to use Gemini CLI, verify in that local installation: `/memory show` (imports), `/skills list` (canonical skills) and `/agents list` (built-in and project specialists). The read-only `.gemini/agents/independent-reviewer.md` is a **definition**, not proof it has launched or that `gemini-3.8-flash` is accessible.

Google's current API documentation confirms the `gemini-4-argon` and `gemini-3.8-flash` IDs for API use. No workspace API keys, model quotas, AI Studio entitlements, Antigravity installation, cloud spend or extra agent runtimes are assumed here. Do not run presenter-suggested remote installer commands without reviewing source/version and required permissions. If an actual robotics app uses Gemini, record pinned dependencies, configuration names (never secret values), errors, cost, latency and a no-network or safe-failure path below.

## Isaac Sim preflight — chosen target, not yet proven

**Use NVIDIA Isaac Sim** for the actual USD scene, physically actuated rover wheels and hopper, rigid-body seed proxies, raycast/range/contact/odometry and second scout. Start from [official Isaac Sim requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html), [current sensor docs](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_physics_raycast.html), [Python API](https://docs.isaacsim.omniverse.nvidia.com/latest/py/index.html). **Do not pin a version from these links**: confirm and pin the *actually installed* build and its supported API; versions differ.

**Hardware reality:** official Isaac Sim guidance specifies 32 GB system RAM and 16 GB GPU VRAM minimum for supported workstation operation; some teammates' reported laptops with 8 GB VRAM are below that. Check each actual machine's CPU/system RAM, GPU model + **VRAM**, driver, OS, free disk and run the Isaac Sim Compatibility Checker. If one machine already runs it reliably, use that as the physics host and connect other PCs through a bounded, logged network layer **only if their permissions/network work**. A GPU cloud host is possible only after verifying allocation, cost/credits, security, streaming and explicit spending/deployment approval. The Linux-only container build, assets network requirements and RTX constraints may affect the choice. Keep a headless/low-sensor-count scene to reduce rendering load. **Do not silently switch back to MuJoCo**; if Isaac Sim genuinely blocks progress, report logs and obtain human choice for fallback.

**Preflight must show actual evidence:** installed version, driver/VRAM/RAM, engine executable, physics tick rate, one moving wheel-driven articulation, one measured range sensor value, ability to generate rigid-body seed proxy with ground contact, deterministic reset, a screenshot/trace, CPU/GPU usage and any failures. No source or cloud credentials in Git. If physics is remote, distinguish onsite robot controller/agent process from remote simulation; a disconnected agent cannot be called locally autonomous merely because a remote renderer is still running.

## Next agent first integration task — Isaac Sim planting-first skeleton (not yet implemented)

**Read:** [PRODUCT](PRODUCT.md), [ARCHITECTURE](ARCHITECTURE.md), proposed [rice.v1 seam](CONTRACTS.md), [TEAM five-role ownership](TEAM.md#selected-rice-project-five-human-work-split-proposed-not-yet-assigned-by-name) and the smallest matching `embodied-loop` / `contracts-first` agent skills. One human integrator owns the first branch and contracts; human robotics owner approves rover body, wheeled drivetrain, probe position, seed dropper geometry and viable traversable field lanes.

**Copy-paste first coding assignment:**
```text
Implement the FIRST EXECUTABLE ISAAC SIM PADDYPATCH SKELETON.
Use the latest main and read AGENTS, PRODUCT, ARCHITECTURE, CONTRACTS,
RUN, and matching skills. Own one isolated branch/worktree.
Inspect installed Isaac Sim version, compatibility and existing robot assets
before writing version-specific imports; never invent a successful launch.
Create a tiny OpenUSD/PhysX field, one wheel-driven rover articulation,
an actual physics range sensor and bounded local drive commands.
Provide deterministic reset/step and an independently sampled observation;
insert an obstacle after motion starts and react from range/odometry only.
Then implement the earliest feasible physical planting proof: one simple
hopper/release action spawns/releases a rigid-body seed proxy from the rover
at a marked site, and the contact/landing is measured by the simulator.
A PLANT text event or placing a seed directly at the goal is NOT enough.
Do not build an LLM, a shared swarm cloud, flood mitigation, rescue/towing,
or a polished dashboard. Keep UI read-only and expose typed rice.v1 fixture
and runner for later map/scout/planner owners. Record exact startup/install
commands, asset licence, tested SHA, actual sensor/action/seed-contact trace,
hardware usage, failures and what remains undone. Do not merge automatically,
expose secrets, spend credits or submit.
```

**First PR acceptance (P0a plus earliest P0b physical action):** physically driven rover; real sampled sensor changes as the scene changes; mid-run obstacle avoidance or safe hold; real rigid-body seed release/contact/landing; no phantom inventory decrement; trace and actual launch/test commands. If Isaac cannot launch, publish its real diagnostics and *do not fabricate a simulated result*. The integrator may split the motion/sensor fixture and seed-drop fixture into **two small sequential PRs**, but physical planting is the **next priority**, not long-term framework abstraction.

**After first seam:** other owners independently build (a) second rover and locally sampled stand/soil proxy, (b) shared map estimation, (c) site selection and two-robot routing, (d) grader/baselines/video, all consuming the same tested contract. P0 still requires a complete physical rice planting mission and surprise-driven replanning. Separate time-permitting rescue and flood-gate work **only after** end-to-end physical placement and scoring are safe.

**Target future command shape only (NOT implemented):** `./python.sh scripts/run_paddypatch.py --scenario patchy-stand --seed 7 --headless` if using an Isaac Sim workstation install; exact startup depends on verified installation, OS and pinned version. Replace with actually tested command in this file.

## Fill immediately after selecting the product stack

| Action | Exact command / requirement |
| --- | --- |
| Supported OS/platform | TBD |
| Minimum CPU/GPU and optional hardware | TBD; do not assume NVIDIA or a robot |
| Clean install / pinned dependency versions | TBD |
| Offline or no-secrets mode | TBD |
| Secrets configuration **names only** | TBD; never record values |
| Start the actual system | TBD |
| One deterministic smoke-demo with seed | TBD |
| Unit and integration tests | TBD |
| Independent baseline run and evaluator export | TBD |
| Reset simulator/robot and recover from errors | TBD |
| Troubleshooting/log location | TBD |

A team member who did **not** develop the project should be able to reproduce the minimum demo from this file. Record commands actually validated on a clean environment and link the tested commit. For physical hardware, add an operator stop/abort procedure and a simulator-only or replay fallback.
