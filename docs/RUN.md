# Reproducible setup, run and recovery

**Post-PR #48 status (7 Nov 2026, `main` `16768dc`):** The single `scripts/run_mujoco_paddypatch.py --scenario golden_rescue` MuJoCo path now includes a typed `TaskMode.RESCUE`, a bounded docking/extraction command, `RecoveryFeedback` and post-rescue planting. [PR #48](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/48) merged; [combined GitHub CI](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/actions/runs/42418413738) ran **78 tests successfully**. The integrated test now checks two seed drops, maximum recorded equality-constraint force below 250 N and pitch below 0.12 rad in its fixture. **This proves the checked-in simulation/test path only:** helper survivability across hazardous traction, policy-only sensing without privileged simulator fault flags, independent multi-seed baselines, live model benefit, physical hardware and a finished submission video/PDF are *not* established by that CI.

**Canonical verification commands (already exercised in the linked CI, not rerun in this documentation pass):** `python scripts/check_repo.py`, `python scripts/sync_skills.py --check`, `python -m unittest discover -s tests -v`. For the rescue scenario use `python scripts/run_mujoco_paddypatch.py --scenario golden_rescue --headless`, and run the continuous viewer on a local MuJoCo machine before recording. Keep visual footage and independently measured success separate from automated CI.

**Current user-directed product scope — Antenna, rescue-first (7 Nov 2026):** The point of the system is **risk-aware robot-to-robot physical recovery** while robots carry out useful rice-field sensing, mapping and planting. The earlier planting-first/rescue-optional instructions below record existing work, not the current primary research question. [D-0008](DECISIONS.md) remains the canonical **naming** decision; new [D-0009](DECISIONS.md) records the **scope** correction. The merged rigid-docking MJCF and `test_mujoco_rigid_docking_cam_latch_and_tandem_extraction` show a **scripted physics extraction test** (victim moves >0.08 m), but not an autonomous sensed-fault→rescue→mission-continuation episode, learned policy or live Jetson inference. [SWARM_EDGE_PLAN](SWARM_EDGE_PLAN.md) / [RESCUE_EDGE_RESEARCH](RESCUE_EDGE_RESEARCH.md).

**Current project direction:** Antenna rescue-first MuJoCo integration is merged and covered by an automated golden-episode physics test. The next gate is **independent physical credibility and broader scenario verification**; learned model policy, distributed Jetson inference, actual hardware and Isaac integration remain unverified.

## Recovery integration gate (separate from existing MuJoCo mechanical test)

## Current minimum working rescue integration assignment

**Owner:** human integrator + existing robot/mechanics owner; keep other authors' source unchanged. Verify the branch and inspect the real actuator IDs and model before implementation. The following is a **test requirement, not a claim it works**:

1. In the **existing** MuJoCo field, issue bounded drive/turn commands to a rover doing a real rice task. After measurable motion, inject a reproducible terrain/traction stall or physically immobilise it; record each issued command, pose, yaw and valid sensor status. Separately demonstrate a missing range sensor and a legitimate HOLD **do not** cause rescue.
2. From that evidence, compare the three actions **dispatch, defer, abstain** using a numerical risk baseline (approach safety, remaining task value and secondary-casualty risk). A model may interpret qualitative context but must not create raw wheel torques or override deterministic force limits.
3. Couple the approved rigid docking/cam-latch/pitch-lift mechanism through a **bounded typed recovery skill**, not a string masquerading as an action or a teleport of the victim. Verify physical contact and **independent** target displacement and helper safety; test failure/abort.
4. Continue at least one local mapping/seed task after successful extraction, or show a reasoned safe-stop and task reallocation if rescue is unsafe. Record the same continuous episode with seeded logs and distinguish *full mission* evidence from the separate already working scripted latch test.
5. Run `python scripts/check_repo.py`, `python scripts/sync_skills.py --check` and `python -m unittest discover -s tests -v` after installing `requirements-mujoco.txt`. For GUI separately launch `python scripts/run_mujoco_paddypatch.py --dashboard` on an actual graphical host; GitHub Actions is **headless** and does not validate a desktop rendering session.

**Current limitation:** PR #48 connects the rescue skill and triage into one automated MuJoCo golden episode. A single passing fixture is not independent evidence of generalizable mechanical realism, safe refusal under varied mud loads, perceptual no-oracle integrity or a finished continuous demo. Prefer real falsifiable results over a more complex GUI or a second physics engine.


To claim the full product, demonstrate a **real detected fault**, contextual rescue/abstain decision, bounded command driving the merged `paddy_ant_rigid_rescue.xml` / `update_rigid_dock_latch` mechanism, independently checked target displacement/helper safety and continued actual fieldwork. The current `test_mujoco_rigid_docking_cam_latch_and_tandem_extraction` is scripted contact/extraction, not end-to-end. Research pooled offline inference only on verified hardware and trusted network; no public llama.cpp RPC or cloud charges without approval.

## New teammate or agent onboarding

For **any fresh Windows/macOS/Linux machine**, start with [WORKSTATION-SETUP](WORKSTATION-SETUP.md): inspect/clone safely, create local `.venv`, run actual kernel checks, optionally verify a toy MuJoCo joint with `scripts/smoke_mujoco.py` **or run the now-merged PaddyPatch MuJoCo prototype**, verify Antigravity/Gemini access, and gate Isaac Sim on compatible hardware and permissions. Do not copy another teammate's private keys or assume their local prototype exists on this machine. MuJoCo is the approved, actually exercised current rescue backend; **Isaac Sim is an unverified optional translation target**, not the first development task. Porting to Isaac still requires tests. [COMPONENTS](COMPONENTS.md) owns world vs robot vs single Isaac integration writer and the module README standard.

## Implemented core check (no Isaac Sim required)

For **core-only** checks (Python 3.10+, no simulation dependency):

```bash
python -m unittest discover -s tests -p 'test_ricebot_foundation.py' -v
```

These use **scripted observations only**. Full discovery additionally runs MuJoCo physics tests and requires Python 3.11+ and `requirements-mujoco.txt`. Neither level proves Isaac Sim or farm performance.

**Next coding agent:** use existing **MuJoCo** and the merged rigid rescue chassis. Connect a genuine observed immobilisation event to the supervisor's risk/abstain decision, then a typed bounded physical docking/extraction skill and independent recovery/outcome check. Avoid creating a second runtime or changing the robot lead's docking geometry without coordination. The conservative central planner now rejects missing-range-only rescue claims; its sensor/command tests passing is **not** proof of physical rescue. Keep both nominal rice-field work and an incident in the **same** episode.

**Scope cap:** two existing MuJoCo rovers, one real simulated seed/sensor mission, one induced fault, one risk-informed rescue or abstention and independently measured mission continuation. No extra mud/planting simulator, full crop lifecycle, new RF hardware, distributed LLM or unreviewed winch.

## MuJoCo Rapid-Prototyping SimulationPort & Isaac Sim Translation Protocol

The checked-in MuJoCo prototype is the **active local development backend**. See [translation notes](MUJOCO-PROTOTYPE-AND-ISAAC-TRANSLATION.md) and [component README](../src/ricebot/mujoco_sim/README.md). With Python 3.11+, in Windows PowerShell:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements-mujoco.txt
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --headless
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --viewer-seconds 12
```

Use `.venv/bin/python` on Linux/macOS. CI runs headlessly; a MuJoCo success does not prove Isaac translation or real crop outcomes.

## Foundation checks — actually available

```bash
python scripts/check_repo.py
python scripts/sync_skills.py --check
python -m unittest discover -s tests -v
python scripts/evaluate.py --candidate tests/fixtures/candidate.jsonl --baseline tests/fixtures/baseline.jsonl
```

Run `python scripts/sync_skills.py --write` after editing a canonical `.agents/skills/` file. Repo/skill/evaluator tools remain Python 3.10+ standard-library-only; **full unittest discovery also executes MuJoCo integration tests and requires `requirements-mujoco.txt` on Python 3.11+**. CI installs it. Physics success does not establish Isaac compatibility.

## Optional Gemini CLI development-agent preflight

The repository does **not** require Gemini/Argon to run its foundation checks. When a teammate elects to use Gemini CLI, verify in that local installation: `/memory show` (imports), `/skills list` (canonical skills) and `/agents list` (built-in and project specialists). The read-only `.gemini/agents/independent-reviewer.md` is a **definition**, not proof it has launched or that `gemini-3.8-flash` is accessible.

Google's current API documentation confirms the `gemini-4-argon` and `gemini-3.8-flash` IDs for API use. No workspace API keys, model quotas, AI Studio entitlements, Antigravity installation, cloud spend or extra agent runtimes are assumed here. Do not run presenter-suggested remote installer commands without reviewing source/version and required permissions. If an actual robotics app uses Gemini, record pinned dependencies, configuration names (never secret values), errors, cost, latency and a no-network or safe-failure path below.

## Isaac Sim preflight — chosen target, not yet proven

**Optional later Isaac Sim translation only (not current rescue P0):** if selected by the human team on suitable hardware, use NVIDIA Isaac Sim for an additional USD scene, physically actuated rover wheels and hopper, rigid-body seed proxies, raycast/range/contact/odometry and second scout. Start from [official Isaac Sim requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html), [current sensor docs](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_physics_raycast.html), [Python API](https://docs.isaacsim.omniverse.nvidia.com/latest/py/index.html). **Do not pin a version from these links**: confirm and pin the *actually installed* build and its supported API; versions differ.

**Hardware reality:** official Isaac Sim guidance specifies 32 GB system RAM and 16 GB GPU VRAM minimum for supported workstation operation; some teammates' reported laptops with 8 GB VRAM are below that. Check each actual machine's CPU/system RAM, GPU model + **VRAM**, driver, OS, free disk and run the Isaac Sim Compatibility Checker. If one machine already runs it reliably, use that as the physics host and connect other PCs through a bounded, logged network layer **only if their permissions/network work**. A GPU cloud host is possible only after verifying allocation, cost/credits, security, streaming and explicit spending/deployment approval. The Linux-only container build, assets network requirements and RTX constraints may affect the choice. Keep a headless/low-sensor-count scene to reduce rendering load. **The already approved MuJoCo prototype can continue independently**; do not call it Isaac. If Isaac preflight fails, report logs and ask the human team whether to retain MuJoCo for the final demo.

**Preflight must show actual evidence:** installed version, driver/VRAM/RAM, engine executable, physics tick rate, one moving wheel-driven articulation, one measured range sensor value, ability to generate rigid-body seed proxy with ground contact, deterministic reset, a screenshot/trace, CPU/GPU usage and any failures. No source or cloud credentials in Git. If physics is remote, distinguish onsite robot controller/agent process from remote simulation; a disconnected agent cannot be called locally autonomous merely because a remote renderer is still running.

## Next agent first integration task — Isaac Sim planting-first skeleton (not yet implemented)

**Historical anchor only; superseded by D-0009 rescue-first.** Older contributor links may point here. Do **not** execute the prior planting-first task or ask another teammate to install Isaac Sim merely to meet current P0. Its complete original specification is retained in Git history. The active demo backend is the already merged MuJoCo rover/seed/rigid-dock physics stack; optional Isaac translation is a separate future preflight.

## Verified Integrated Baseline Startup & Demo Commands (`integration/abd-canonical-rescue`)

| Action | Exact command / requirement |
| --- | --- |
| Supported OS/platform | Windows 11 / Linux / macOS with Python 3.11+ (tested on Windows 11 with Python 3.13 in `.venv` and MuJoCo 4.0.0) |
| Minimum CPU/GPU and optional hardware | Standard CPU + OpenGL for MuJoCo 3D viewer and Tkinter GUI (no NVIDIA GPU or physical robot required) |
| Clean install / pinned dependency versions | `.\.venv\Scripts\python.exe -m pip install -r requirements-mujoco.txt` |
| Offline or no-secrets mode | Works 100% offline by default via `DeterministicReasoner` fallback (`SwarmOrchestratorAgent` requires no API key) |
| Secrets configuration **names only** | Optional `GEMINI_API_KEY` (only if live Gemini API calls are explicitly enabled; never required for baseline) |
| Start the actual GUI system | `.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --dashboard --scenario golden_rescue --seed 7` |
| Start 3D MuJoCo passive viewer | `.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --scenario golden_rescue --policy cooperative --rescue-policy react_orchestrator --seed 7 --viewer-seconds 12` |
| Preset 1 — Successful Rescue (headless JSON) | `.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --scenario golden_rescue --policy cooperative --rescue-policy react_orchestrator --seed 7 --headless` |
| Preset 2 — Safe Refusal / Abstain (headless JSON) | `.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --scenario high_risk_abstain --policy cooperative --rescue-policy react_orchestrator --seed 7 --headless` |
| Preset 3 — Sensor / Comms Failure (headless JSON) | `.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --scenario sensor_degraded --policy mycelial --comms intermittent --seed 7 --headless` |
| Matched-scenario baseline comparison | `.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --compare-baselines --seed 7` |
| Unit and integration tests (102 tests) | `.\.venv\Scripts\python.exe -m unittest discover -s tests -v` |
| Repository & skill integrity checks | `.\.venv\Scripts\python.exe scripts\check_repo.py` and `.\.venv\Scripts\python.exe scripts\sync_skills.py --check` |
| Reset simulator/robot and recover from errors | Click `↺ Reset` or `🎲 Random Scenario` in the GUI toolbar, or call `session.reset(seed)` |

### Known Limitations of the Integrated Baseline
1. **Analytical Bekker–Wong Terramechanics + MuJoCo Rigid Contact:** Mud sinkage and suction drag are modelled via analytical Bekker–Wong terramechanics coupled to MuJoCo rigid-body wheel friction and viscous drag, not full SPH/MPM continuum fluid-soil simulation.
2. **Offline Deterministic Reasoner Default:** When no external LLM backend is configured or reachable, `SwarmOrchestratorAgent` executes the 4-tool ReAct loop (`inspect_incident`, `probe_perimeter`, `predict_recovery_displacement`, `evaluate_farmer_risk`) using the calibrated offline `DeterministicReasoner` (`fallback_used=True`, explicitly labelled `[FALLBACK:DeterministicReasoner]` in the GUI).
3. **Single-Tier Paddy Plot Canonical Scene:** Stepped multi-terrace ramp navigation from `feat/agent-c-sensing-world` (`63c72af`) is parked outside the canonical rescue baseline so the 2-rover rigid-docking recovery loop remains deterministic and reliable.
4. **Local Workstation Communication:** Peer-to-peer UWB/HaLow gossip and blackout windows are simulated in-process within `DecentralizedMycelialRuntime`; physical radio hardware and Isaac Sim 5.1/6.2 migration remain separate follow-up issues.
