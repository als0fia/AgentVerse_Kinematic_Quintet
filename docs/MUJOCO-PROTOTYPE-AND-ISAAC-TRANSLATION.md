# Antenna: MuJoCo → Isaac Sim 6.2 / Isaac Lab 3.0 migration and learning plan

> **Authoritative migration plan — investigated 7 November 2026, not an implemented Isaac backend.**
> **Audited main:** [38c1b22](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/commit/38c1b2222e6ce4ab31cd773ea2662dda8eb0113b); independent source check and vendor documentation. **No local Isaac installation, graphics host, MuJoCo replay or robot-training run performed during this investigation.**
> **Current unmerged overlap:** [PR #54](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/54) changes the same robot, controller and simulator modules and adds terrace-ramp vision work. Reconcile per feature at its actual HEAD; never overwrite a live teammate's branch.
> **Product:** Antenna's rescue-first autonomous recovery (rice field is the use-case and useful mission). **Do not replace the time-critical working MuJoCo demo with an untested Isaac scene.** Keep Isaac work isolated; no cloud provisioning, paid APIs, new hardware or public deployment without explicit approval.

## Executive decision: translate the *embodiment and environment*, reuse the *decision architecture*

- **Baseline, keep:** MuJoCo 4.0.0, the working headless golden episode, physical rescue assets, original tests, and the existing rice.v1 Python MissionCore/SimulationPort contracts. This is the regression oracle and a fallback, not an engine to delete.
- **Primary Isaac target:** [Isaac Sim 6.2.0](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/download.html) with PhysX, physically collidable USD terrain/robots and native camera/physics sensors. The presentation RTX camera is **not** the robot's perception unless those images feed the policy.
- **Learning target:** [Isaac Lab 3.0](https://docs.nvidia.com/isaaclab/3.0/isaac-lab/installation) for parallel rescue-skill experiments, domain randomisation, imitation/RL, and held-out evaluation. Its [current compatibility matrix](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/compatibility.html) pairs Isaac Sim 6.2 with Isaac Lab 3.0 GA. Use Python **3.12 in a separate, pinned environment** (do not upgrade the existing MuJoCo environment in-place); vendor dependency overrides matter.
- **Advanced physics, optional research:** Newton and coupled MPM granular material in Isaac Lab 3.0 are **experimental**, not a turnkey accurate flooded paddy. Test one small patch with calibrated evidence before adding rigid–MPM coupling to full rescue. See [MPM](https://docs.nvidia.com/isaaclab/3.0/concepts/implicit-mpm) and [coupled solvers](https://docs.nvidia.com/isaaclab/3.0/concepts/coupled-solvers).
- **Anti-goal:** don't migrate all Python logic into OmniGraph/Kit, duplicate a farm simulator, retrain the strategic LLM without an evaluation hypothesis, or assume imported MJCF has equivalent contacts/docking in PhysX.

**Hardware gate:** [Isaac Sim 6.2 requirements](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/requirements.html) list **32 GB RAM and 16 GB VRAM minimum**, recommended ≥64 GB RAM for more complex work, a supported RTX GPU and appropriate drivers. Training and heavy sensors require more. An 8 GB VRAM/16 GB RAM laptop is *below* supported full Isaac Sim requirements. No host, VM, credit or quota has been approved/validated; a cloud GPU is only an option after permission, costs and streaming/security are verified. Isaac Lab's headless Newton path avoids the full Isaac Sim GUI/RTX requirement, but must still be profiled on a real GPU before promising training scale.

## Gate A — verify repair boundaries *before porting robotics behaviour*

Source-checked on main 38c1b22 using the [15-finding boundary audit](audits/2026-11-07-simulation-perception-viewer.md). The following are **PRESENT at the audited SHA**, not newly runtime-reproduced. Re-audit against live origin/main immediately before implementation.

| Gate | Source fingerprint / defect | Why migration must not reproduce it | Exit evidence |
| --- | --- | --- | --- |
| **A0 functional** | agent/mycelial_swarm.py builds RescueAssessment using rescuer_robot_id / net_rescue_utility while planner.py defines helper_robot_id / net_rescue_value; viewer reads wrong names; Mycelial calls advisor.recommend() although interface defines advise() | Broken incident/UI/AI path may be hidden by central golden test | Mycelial incident + central incident + inspector and mock advisor tests run with no exceptions; one dataclass and one observable advisor contract |
| **A1 perception** | mujoco_sim/adapter.py exposes simulator _bogged_origins as RobotObservation.is_bogged; planner and Mycelial consume it as fault truth | Agent appears to sense a fault it was actually told | Separate private event and timestamped observed slip/odometry/IMU/command fault evidence; HOLD/dropout/obstacle false-positive tests |
| **A2 rescue skill** | adapter.py uses perfect MuJoCo front/rear dock site 3D positions/quat to steer the rescuer | Engine secretly implements precise control that robot is credited for | Robot-side closed-loop controller uses permitted relative sensor estimates; engine owns only actuation and physical latch/contact; fail-safe abort cases |
| **A3 terrain/soil** | relief hfield collision disabled; flat plane collidable; height changes via raw qpos and speed via qvel damping | Beautiful scene could depict hills/sinkage not experienced by wheels | One authored collidable surface, consistent materials and forces, no arbitrary per-tick chassis teleport; separate labelled analytical soil proxy |
| **A4 runtime/viewer** | viewer/live_app.py changes sim fields/private methods and executes separate sim.step and agent.decide loop | UI toggles/control can change physics and diverge from CLI run | One scenario/session owner drives both headless and GUI; passive renderer/evaluator; same-seed GUI/view-off trace parity |
| **A5 local information** | mycelial_swarm.py gives every node full peer_poses regardless of comm isolation | Fake decentralisation and fake range/pose sensing | Node receives local observations and actual simulated received packets/UWB measurements only; blackout leakage test |
| **A6 sensor and claims** | exact site soil truth returned by location gate; all-invalid range maps to “clear” 4m; hardcoded benchmark popup; camera viewport not a perception input; sensor_degraded has no dropout | Realistic RTX visuals don't repair ungrounded sensing or results | Probe validity/noise/contact contract, invalid/stale range, camera-channel provenance, measured metrics and a real dropout fixture |

**Go/no-go:** A0 must be fixed before any substantial Isaac integration; A1/A2/A4/A5 must be fixed or **explicitly isolated as simulator-assisted legacy baselines** before claiming autonomous Isaac rescue. A3 must be designed afresh for PhysX; do not translate its raw qpos/speed hacks. Cosmetic USD scene authoring may proceed in parallel as a non-acting visual probe. An open PR (currently #54) is not a fix until its tests and integration actually pass.

## Architecture: one product core, two interchangeable simulator adapters

~~~text
                 Config + scenario seed + mission/operator priorities
                                      |
                         ONE episode/session owner
                        (reset/step/events/trace)
                                      |
                   +------------------+------------------+
                   |                                     |
     SimulationPort: MuJoCoPaddySimulation      SimulationPort: IsaacPaddySimulation
     established baseline (MJCF)              proposed: USD / PhysX assets
                   |                                     |
       model/data private truth                 stage/physics private truth
                   +------------------+------------------+
                                      |
                        Sensor + fault-event bridge
                  (timestamps, frame IDs, validity, noise)
                                      |
                 per-robot Observations + received packets
                                      |
          unchanged System 0/1/2 core: local controller,
            mission, numeric triage, optional AI supervisor
                                      |
                    bounded RobotCommand / skill request
                                      |
                     engine actuation/contact physics
                                      |
                   evaluator truth + structured trace
                                      |
         independent grading and passive UI/RTX display
~~~

**Never conflate:**
1. Simulator truth (private rigid-body state, hidden soil map, scheduled fault and known other-robot poses).
2. Measured/synthetic *sensor channel* (timestamps, validity, noise, occlusion, delay and mounting).
3. Estimated local robot belief and actual communicated peer knowledge.
4. Independent grader / spectator graphics.

Shared core stays in src/ricebot/contracts.py, runtime.py, mission.py, control.py and agent/. A new isolated src/ricebot/isaac/ module owns the single Isaac simulator composition. Reuse world/terrace.py site data and robot/rovers.py physical **specifications**, not its MJCF XML constructor as a second canonical scene language. GUI is a subscriber, not a third simulator. The existing separate seasonal src/riceswarm_ai/ is NOT the rescue environment.

**Contract compatibility:** preserve SimulationPort.reset(seed) -> ObservationFrame and step(commands) -> ObservationFrame initially; add an owned side channel for private truth/evaluation, recorded disturbances, skill diagnostics and sensor capabilities if needed. A human integrator approves any shared schema delta. Use adapter-specific extended telemetry as optional namespaced data, not undocumented changes to RobotObservation fields.

## Migration map — what transfers and what must be rebuilt

| Existing feature | Portable application/core logic | Isaac-specific rebuild + validation |
| --- | --- | --- |
| Rice field: site IDs, permitted/no-go, hidden synthetic soil/stand maps, patch tasks | World **data definitions** and seed selection, site-level mission logic | Single USD stage with real collider meshes, bunds, crop/obstacles; authored physics material distinct from render material; meter/z-up frame parity |
| Terrain + bogging | Documented continuous terrain and measured numerical soil proxy if retained | Proper contact ground, friction/compliance, wheel slip and resistance forces; no direct chassis z-qpos or qvel hacks; incident injection is environmental not robot oracle |
| Dual 4-wheel homogeneous rover + tools | Named chassis specs, masses, joint intent, wheel velocity conversion and task capabilities | USD articulation(s), collision hull/wheels, limits, mass/inertia, motor speed/effort, seed chute, dibber/probe, camera mounts, coordinate conversion; validate each DOF |
| Existing MJCF robot assets | Initial reference geometry/joint naming and asset provenance | Try supported [MJCF importer](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/importer_exporter/import_mjcf.html) **only as a starting mesh/articulation candidate**. MuJoCo equality/docking, multi-DOF joint representations, inertias/collision filters and PhysX drive tuning are not automatically equivalent |
| Rigid rescue docking, cam latch, pitch lift and tow | Recovery phases, safety envelopes, target identity and RecoveryFeedback semantics | PhysX-compatible temporary mechanical constraint under verified physical capture/latch conditions; enforce contact/relative pose, force limits, safe detach/abort; test no snap/teleport/unstable articulation pairs |
| Seed proxies and crop work | Target assignment, seed inventory, plant feedback | Physical rigid sphere release through actual nozzle into correct collider; measure contact and location by independent evaluator, not position-spawn-at-target shortcut |
| Range, odometry, GNSS, IMU, joint encoders, probe | Sensor definitions/protocol and imperfect inference | [Isaac 6.2 physics raycast](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/sensors/isaacsim_sensors_physics_raycast.html), [physics contact/IMU/joint sensors](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/sensors/isaacsim_sensors_physics.html), optional RTX camera/depth/LiDAR; explicit per-sensor invalid/unknown/age/noise and sample-contact event |
| Peer UWB, simulated gossip, isolated nodes | Local maps, packet schema, stale/duplicate handling, task coordination | Engine/sensor bridge emits noisy range/bearing when physically plausible; comm transport independently models delay/loss/range and does not expose privileged peer poses |
| AI rescue supervisor and numerical planner | Agent tool calls, mission values, incident detection, approved actions and safety | Keep in ordinary Python. It consumes the *same typed observations* regardless of engine; never put agent policy inside Kit scene scripts |
| Spectator 3D and 2D maps | Evaluator trace format, provenance labels, visual interpretation | Isaac viewport/RTX cameras, labelled robot-local camera view vs spectator, optional streamed frontend; no independent action/world mutation from UI |

**Timing and frame traps:** canonical metres, seconds, rad, z-up, named body/sensor frames. Document MuJoCo quaternion WXYZ ↔ Isaac Lab 3.0 **XYZW** conversions ([3.0 release](https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0)); do NOT assume every USD/PhysX helper uses the same external quaternion order. Define one adapter-local conversion library and test with 90° yaw/pitch. Separate high-frequency physics dt from 0.1 s SimulationPort control/mission tick; hold/saturate command between ticks; reject expired commands; record exact clock and sensor sample time. Compare dimensional units and wheel drive target conventions.

## Staged implementation — smallest useful milestones with exit tests

### M0 — Freeze MuJoCo evidence and repair critical interfaces (first)
- Pin actual working main SHA, golden-rescue seed/config and a **machine-generated** replay log: applied commands, timestamped permitted robot observations, truth-only evaluator state, contacts/forces, mission decision, AI calls and result. Log reproducibility artifacts, not unverified screenshots.
- Fix A0 and show an actual exercised Mycelial incident and UI inspector. Work through A1/A2/A4/A5 via separately owned PRs; choose one accepted sensor/docking architecture. Re-run full tests, headless golden and actual GUI on a capable local host.
- **Exit:** no hidden fault/pose dependence in policy tests, consistent viewer/controller outcome, explicitly labelled remaining mud/sensor approximations. If blocked, the Isaac work is research and non-acting USD geometry only.

### M1 — Isaac hardware and physics smoke (one developer/host, no product refactor)
- Run NVIDIA Compatibility Checker and record OS, CPU, physical RAM, GPU/VRAM, driver, free disk, exact installed versions, permissions, assets availability and actual elapsed simulation step rate. Prefer local compatible host; alternative remotely requires explicit cost/security approval.
- Create one physics world, one **wheel-driven articulated rover** and a ground collider; observe physical translation, wheel contacts, height/slope, real sensor ray and IMU/encoder. Confirm reset reproducibility. Save screenshot and raw trace with exact commit.
- **Exit:** continuous physically actuated single-rover trace, no artificial direct position writes, stable reset; no need for beautiful field, second rover or AI yet.

### M2 — USD embodiment + terrain + physical recovery mechanics
- Import/adapt chassis reference, inspect all DOFs/colliders/masses/wheel joints and names, then build two rovers, seed proxy and bounded real docking/latch/capture. Use one authored collision world and separate pretty rice/soil visual layer.
- Start dry/rigid/soft-material zones with measured resistance; compare known wheel speeds, ground clearance and slopes. Add mud traction **as a documented force/material proxy** only if PhysX cannot represent relevant behaviour directly.
- Define the latch activation rule by physical proximity/contact and actual jaw state; validate helper force, peak impulses at physics step resolution, stability through lift/pull, safe release. Test disturbances and abort.
- **Exit:** *scripted mechanical* Isaac physics rescue and seed drop through the real assembled models. Do not call it autonomous yet.

### M3 — IsaacPaddySimulation + real robot observations + one session owner
- Add the src/ricebot/isaac/ adapter behind the existing SimulationPort and one authoritative session (config, reset, step, events, trace). Keep policy/planner/controller as a shared pure-Python dependency.
- Build a sensor adapter for per-robot timestamped range/odometry/encoder/IMU, contact, optional relative peer and local probe; RGB/D images are robot inputs **only if processed into actual detections**. Surface missing and occluded measurements as unknown, never safe clearance or actual fault truth.
- Inject one unannounced traction/stall disturbance into environment dynamics without sending its hidden cause to the robot.
- **Exit:** same code policy consumes Isaac observation frames and issues bounded skill commands; no policy accesses stage or private physics; comparable command/fault/abstain traces exist.

### M4 — Continuous rescue scenario + trustworthy RTX presentation
- Run two agents and fieldwork in one Isaac episode: meaningful sampling/seed action → physically induced stall → independently sensed evidence → risk-sensitive RESCUE/DEFER/REFUSE → actual coupling/tow or safe abstain → verified continuation.
- Add RTX scene materials, terrain texture, vegetation, light/weather and camera framing **without changing collision/hazard truth**. Show a robot egocamera and sensor data adjacent to the spectator view; display quantitative physical contact and risk on a separate evaluator panel.
- Use camera/viewport settings as *view-only*. Renderer on/off, resolution, spectator camera, map overlays must not alter missions or physics; interactive scenario changes pass through logged experiment/session API.
- **Exit:** live run and optional recording with provenance, crash-free inspector, one verified sensor→decision→action causal chain and explicit assumptions. An RTX video alone is not an autonomy demonstration.

### M5 — Isaac Lab learning *inside the same embodiment* (optional separate task package)
- Create one small, **manager-based or direct Isaac Lab 3.0 task** using the shared USD rover and sensor schema; do not introduce a second crop/world simulator or refactor the strategic agent to fit an RL interface.
- **First skill:** camera/range/odometry-guided helper approach, alignment and safe docking (fixed target initially). **Next skill:** latched lift/pull with learned traction-aware speed/torque and abort. The existing deterministic controller is the teacher and essential baseline.
- Training **policy observation** = only relative estimated target pose, own wheel/IMU/odometry, proximity/contact, valid terrain observations, command history and sensor confidence; never global true pose, trap flag, hidden soil truth, future incident or reward terms.
- Training **action** = bounded wheel velocities, discrete latch/lift/abort requests, optional safe local control deltas. The deterministic safety shield enforces physical force/angle/velocity/no-go constraints; AI does not command arbitrary solver state.
- **Reward/termination** is evaluator-side only: successful physical verified latch/extraction, time/energy, track error, excessive wheel spin, collision with crop, helper bogging, joint force, tip-over, premature latch, damage. Penalise prohibited contacts and second casualty hard; success cannot be earned through reward-only proximity or ghost latch.
- **Curriculum:** (i) alignment on dry level ground, (ii) fixed docking and short tow, (iii) randomized angle/relative pose, (iv) uncertain/noisy/delayed sensing, (v) plausible friction/mass/compaction, (vi) rescue-vs-abort situations; then optionally camera-pixel policy. Use expert rollouts/imitation before expensive RL if reliable scripted control exists.
- **Randomize** geometry tolerances, rover masses, wheel friction/torque/soil compliance, visual lighting/occlusion, sensor noise/dropout, networking, fault time and crop obstacles within documented plausible ranges. Hold out combinations and never tune on the final test seeds.
- **Exit:** versioned policy weights with training seed/config/code, actual curves, held-out success/abort/helper safety/force/latency vs the same deterministic teacher, and cross-engine rollout where meaningful. Never call a training job “learning” without a reproducible trained checkpoint.

### M6 — Higher-fidelity soil physics and autonomous supervisor learning (research, not MVP)
- PhysX material and compliant-contact calibration first. Only if contact/traction limits dominate results, test an isolated Newton **rigid–MPM** patch with experimentally constrained material parameters. Benchmark timestep, real-time factor and contact-energy artifacts; its coupled solver is explicitly experimental and may not interoperate with every sensor/backend.
- A separate *strategic* study can evaluate LLM tool-using supervisor vs calibrated deterministic risk optimiser and never/always rescue on matched high-uncertainty incidents. Test investigation, refusal and revise/abort; use outcome evidence rather than language quality as the score.
- Do **not** conflate learned docking control with a smarter contextual rescue agent. Both require separate baselines and results. Never train an LLM merely because Isaac Lab is present.

## Cross-simulator verification and promotion rules

**Same config and action sequence, not identical floating-point trajectories:** two physics solvers will not yield bit-for-bit same contacts. Require matching contracts, plausible physics and independent measured outcomes after calibrating acceptable tolerances.

| Gate | Recorded proof; suggested test design (unexecuted) |
| --- | --- |
| C0 | CLI vs GUI/view-off in **each** backend, same seed/policy: identical simulator time, issued commands, mission transitions and outcome; camera toggle cannot affect physics |
| C1 | 100% of emitted observations schema-validate; units, frames, quaternions and timestamps checked, including 90° transforms and invalid/no-data cases |
| C2 | Blackout/occlusion/hidden fault changes do not let policy observe disallowed remote pose or bogged truth; logged producer→consumer provenance |
| C3 | Nominal no-rescue tasks: safe wheel motions, genuine local soil sampling, rigid-body seed ground contact at reasonable locations, zero ghost inventory |
| C4 | Intentional HOLD, missing front-range and obstacle braking do not trigger bogus rescue; real observed stall does, within a measured response time |
| C5 | Rescue coupling, lift/tow, helper survival and safe abort graded with **physics-step** forces and contacts; no teleport, invisible tether or equality/fixed-joint snap |
| C6 | Matched scenario/seed bank including nominal, golden stall, risky rescue, justified abstain, off-axis dock, helper fault, blocked path, sensory dropout; report numerator/denominator and failure categories (e.g., at least 20 prespecified seeds per category before statistical claims) |
| C7 | Recorded video comes from that executable scene and identifies SHA, model/physics backend, seed and whether footage is real-time or replay |
| C8 | Isaac Lab trained-vs-untrained controller, vs deterministic teacher: held-out success, force/impulse, time/energy, second-casualty and rollback; no inference from training reward alone |

Promotion to the primary demo backend requires **all core functional gates (C0–C7)** and a reliable graphical run on an actually supported host. If the evidence is incomplete, use MuJoCo for the submission and Isaac only as exploratory/presentation work explicitly labelled as such. Isaac Lab training is optional and must not block robust physical rescue.

## Safe feature integration and roles

- **Integrator:** only writer of SimulationPort/shared typed contracts, common episode/config/trace and release gate. 
- **Physics/robot owner:** one writer for USD articulation, physical terrain, latch and force tests; reconcile human-authored geometry and PR #54 mechanically.
- **Sensors and intelligence owner:** sensor adapter/validity, peer packets, policy-only observations, event-driven supervisor; never use engine truth for action.
- **Learning owner (later):** Isaac Lab task/data/reward, domain randomisation, deterministic teacher, checkpoint+held-out metrics; no simultaneous edits to main runtime.
- **Evaluator/demo owner:** truth-only grader, camera/provenance inspector, hardware measurements, continuous live run, renderer invariance.

**Suggested PR split:** (1) A0 crash/advisor tests; (2) fault evidence + permitted relative sensor contracts; (3) session/viewer parity and truth separation; (4) isolated Isaac smoke/articulation (no production flag); (5) physical USD world+rigid latch+seed; (6) Isaac adapter and consistent scenarios; (7) RTX dashboard/demo verification; (8) Isaac Lab skill training; (9) experimental granular soil. Every PR states base SHA, owner, touched files, installed software/host, exact tests/seed, observed logs and fallback. Merge **per feature, not per old branch** per [SIMULATOR-VARIANTS](SIMULATOR-VARIANTS.md). Avoid overlap with active work (notably PR #54).

## First engineering assignment after review (one bounded task)

> On a **verified** Isaac Sim 6.2 capable workstation, and only after an integrator selects an owned branch, create **src/ricebot/isaac/** with one USD-collidable flat ground, one imported/reconstructed four-wheel Antenna rover articulation, wheel-speed actuation, timestamps and one physics raycast sensor. Show physical locomotion and sensor changes in a trace; compare geometry, units, joint count and wheel displacement against a pinned MuJoCo single-rover fixture. Do not implement AI/LLM, rice lifecycle, docking, MPM or RTX presentation yet. Do not change existing MuJoCo/default runner, shared contracts or other people's live branches. Report the exact Isaac/Isaac Lab version, machine specs, commands run, screenshot/trace, performance and obstacles. If the hardware gate fails, stop at a documented preflight; never claim an Isaac run.

## Vendor and repo references (verified at research time)

- [Isaac Sim 6.2 download / release](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/download.html), [system requirements](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/requirements.html), [compatibility](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/compatibility.html).
- [Isaac Lab 3.0 install, version pins and extras](https://docs.nvidia.com/isaaclab/3.0/isaac-lab/installation), [task architecture](https://docs.nvidia.com/isaaclab/3.0/concepts/direct-and-manager-mdp), [release/migration notes](https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0).
- [Isaac Sim 6.2 MJCF importer and limitations](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/importer_exporter/import_mjcf.html), [new physics sensors](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/sensors/isaacsim_sensors_physics.html), [robot assembly joint cautions](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/robot_setup/assemble_robots.html).
- [Isaac Lab granular MPM](https://docs.nvidia.com/isaaclab/3.0/concepts/implicit-mpm) and [experimental solver coupling](https://docs.nvidia.com/isaaclab/3.0/concepts/coupled-solvers).
- Internal: [boundary audit](audits/2026-11-07-simulation-perception-viewer.md), [product scope](PRODUCT.md), [architecture](ARCHITECTURE.md), [existing contracts](CONTRACTS.md), [tests](TESTING.md), [feature provenance](SIMULATOR-VARIANTS.md).

---

## Earlier MuJoCo prototype notes and pre-6.2 ideas (retained as historical reference)

The original sections below were written around the Isaac Sim **5.1** era and a planting-first P0. Keep their geometry/seed/physics research as a historical backlog, **not** as the live Isaac version pin, current architecture task or proof that sensor_degraded and all associated ideas have been implemented. The current plan above supersedes any incompatible wording.

## 1. Implemented MuJoCo prototype → unverified Isaac target
- **Hardware Constraint on 8 GB VRAM Laptops:** NVIDIA Isaac Sim 5.1 specifies a minimum of **32 GB System RAM** and **16 GB GPU VRAM** for supported workstation operation. The team has committed a **MuJoCo 4.0.0 prototype**, which can be run headlessly without an RTX GPU. Performance on each laptop must be measured, not assumed. MuJoCo 4.0.0 requires Python 3.11+.
- **Three-Way Architectural Split ([docs/COMPONENTS.md](COMPONENTS.md)):**
  1. **World & Sensor Host ([src/ricebot/world/README.md](../src/ricebot/world/README.md)):** Owns the 6-patch bunded rice terrace (`DEFAULT_SITES`), hidden synthetic spatial soil/stand-gap truth (`DEFAULT_HIDDEN_SOIL_TRUTH`), location-gated soil probe sampler (`PaddyTerraceWorld.sample_local_soil`), and world terrain geometry (`build_world_terrain_xml`).
  2. **Robot Embodiment ([src/ricebot/robot/README.md](../src/ricebot/robot/README.md)):** Owns the 4-wheel differential-drive rover specs (`PLANTER_SPEC`, `SCOUT_SPEC`), skid-steer wheel velocity kinematics (`compute_4wheel_skid_steer`), 3-ray forward LiDAR fan, odometry mount frames, and 6-DOF seed-hopper release embodiment (`build_rovers_and_seeds_xml`).
  3. **Simulator Composition Adapter ([src/ricebot/mujoco_sim/README.md](../src/ricebot/mujoco_sim/README.md) now, `src/ricebot/isaac/` next):** Composes `src/ricebot/world/` + `src/ricebot/robot/` behind `SimulationPort` (`reset(seed) -> ObservationFrame`, `step(commands) -> ObservationFrame`).
- **Reuse objective, not guarantee:** `SimulationPort` and the world/robot split are intended to minimise rewrites. **No Isaac adapter or equivalent cross-engine physics/contact test has been verified.** Reuse shared mission/field/kinematics modules only after measured API/frame/sensor parity; any necessary change goes through the human integrator.

---

## 2. Running the 3D MuJoCo PaddyPatch Simulator Locally

From the cloned repository root using Python **3.11+**, install the pinned [optional MuJoCo dependency](../requirements-mujoco.txt) first (`.\.venv\Scripts\python.exe -m pip install -r requirements-mujoco.txt` on Windows):

```powershell
# 1. Launch interactive 3D MuJoCo viewer (bunded terrace, 2 wheeled rovers, live laser rays & belief-map rings)
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --viewer-seconds 12

# 2. Run headless episode (dual-rover soil sampling + physical seed drop + ground-contact JSON trace)
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --headless

# 3. Run mid-episode obstacle surprise scenario (obstacle injected at t=1.5s, LiDAR fan triggers safe HOLD)
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --scenario obstacle --viewer-seconds 10

# 4. Run all current unit + world/robot + MuJoCo physics tests
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

---

## 3. Proposed Isaac parity checklist (translation not yet implemented)

When building a future Isaac adapter, **aim to reuse** `PaddyTerraceWorld` (`src/ricebot/world/`), `compute_4wheel_skid_steer` and `RoverChassisSpec` (`src/ricebot/robot/`). Validate frames, joints, sensor readings, timing and actual seed contacts in both engines; the table is a **target parity checklist, not proven automatic compatibility**:

| Boundary / Component | MuJoCo Prototype (`src/ricebot/mujoco_sim/`) | Isaac Sim Target (`src/ricebot/isaac/` on ≥16 GB VRAM host) | Shared Module / Contract Invariant |
| :--- | :--- | :--- | :--- |
| **World Coordinate Frame & Units** | Right-handed metric $(x, y, z)$ in metres, yaw in radians $(-\pi, \pi]$, monotonic `sim_time_s` (`dt = 0.004 s`, 25 substeps = `0.10 s` per tick). | OpenUSD stage with `UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)` and `metersPerUnit = 1.0`; physics step `0.10 s` per `SimulationPort.step()`. | All `Pose2D(x_m, y_m, yaw_rad)` and timestamps must be finite and strictly increasing (`frame.time_s > last_time`). |
| **6-Patch Rice Terrace (`DEFAULT_SITES`)** | `patch_a` $(0.85, -0.50)$ & `patch_d` $(1.60, 0.45)$ (*Lannitian* mud hazards); `patch_b` $(1.60, -0.50)$ & `patch_c` $(2.35, -0.50)$ (viable stand gaps); `crop_nw` $(0.85, 0.45)$ & `crop_ne` $(2.35, 0.45)$ (`permitted=False` rice stands). | Import `DEFAULT_SITES` from `src/ricebot/world/` and spawn matching USD Xform beds, perimeter bund colliders, and seedling meshes. | Protected sites (`permitted=False`) must never be sampled or crossed by `LocalController`. |
| **Dual 4-Wheel Rovers (`planter`, `scout`)** | 6-DOF `freejoint` chassis + 4 revolute wheels (`_fl`, `_fr`, `_rl`, `_rr`) driven by `compute_4wheel_skid_steer(spec, lin, ang)`. | Two USD `Articulation` prims driven by `ArticulationAction(joint_velocities=compute_4wheel_skid_steer(spec, lin, ang))`. | Expired commands (`not (issued_at_s <= now_s < expires_at_s)`) are rejected at the simulator bridge (zero wheel velocity). |
| **3-Ray Forward LiDAR Fan (`front_range_m`)** | 3 `mjSENS_RANGEFINDER` rays (`_range_c`, `_range_l` at $+18^\circ$, `_range_r` at $-18^\circ$), returning `min(c, l, r)` capped at `4.0 m`. | 3 PhysX raycasts (`raycast_closest`) at $0^\circ, +18^\circ, -18^\circ$ in the rover nose frame, returning `min(c, l, r)`. | Returns `float` in metres (`>= 0.0`) or `None` if offline; `< 0.35 m` triggers `LocalController` safe stop. |
| **Location-Gated Soil Probes (`SoilSample`)** | Calls `self.world.sample_local_soil(robot_id, pose, sim_t)` (`PaddyTerraceWorld` in `src/ricebot/world/terrace.py`). | Calls the **exact same** `self.world.sample_local_soil(robot_id, pose, sim_t)` instance. | Hidden field truth is **never** exposed outside `PaddyTerraceWorld`/evaluator; `sample.sampled_at_s == frame.time_s`. |
| **Physical Seed Release & Contact (`PlantFeedback`)** | Releases 6-DOF sphere `seed_proxy_X` from hopper chute ($z = 0.032\text{ m}$) on valid stationary `release_seed=True` and checks `mjData.contact` against `field_ground`. | Releases USD rigid-body sphere (`UsdPhysics.RigidBodyAPI` + `CollisionAPI`) from hopper chute and reads PhysX contact report with `field_ground`. | Inventory decrements **only** on physical release; `PlantFeedback(released=True, landing=Pose2D(...))` emitted **only** upon ground contact; `scout` can never release seeds. |

---

## 4. Simulation Design Backlog & Next Ideas (Pre-Code Proposals for Team Review)

Below are structured simulation ideas designed to strengthen physical realism, multi-agent coordination value, and evaluation rigor while strictly respecting the [docs/COMPONENTS.md](COMPONENTS.md) module boundaries. **These are documented as design specifications before writing code so the world, robot, planner, and evaluation owners can align first.**

### Idea 1 — Spatially Continuous Paddy Terramechanics & Mud Drag Zones (`src/ricebot/world/`)
- **Problem:** Currently, *Lannitian* deep mud patches (`patch_a`, `patch_d`) are unsafe to plant in (high moisture, low firmness), but driving across them has the same wheel friction as firm drained loam.
- **Proposed Simulation Enhancement:**
  - Add a position-dependent traction/viscous-drag modifier inside the simulator step when a rover's wheels overlap a high-moisture mud pan ($M(x, y) > 0.80, F(x, y) < 0.25$): reduce effective wheel-ground friction and apply a proportional velocity damping drag.
  - **Why it helps the product:** Makes `scout`'s lighter chassis (`2.4 kg`) safer for probing suspect mud pans while giving the heavier `planter` (`3.2 kg` + seed hopper) a physical incentive to avoid routing through soft mud pans once `scout` maps them on the shared `MeasuredFieldMap`.

### Idea 2 — Seed-Controlled Probe Noise & Active Multi-Sample Fusion (`src/ricebot/world/` + `src/ricebot/agent/`)
- **Problem:** Deterministic noise-free soil probes mean a single sample immediately yields zero physical variance, under-utilizing `MeasuredFieldMap`'s sample-count uncertainty reduction (`uncertainty = 1 / sqrt(count)`).
- **Proposed Simulation Enhancement:**
  - Allow optional deterministic, `seed`-controlled Gaussian probe noise $\epsilon \sim \mathcal{N}(0, \sigma_{\text{sensor}}^2)$ in `PaddyTerraceWorld.sample_local_soil` for borderline patches (e.g., firmness near the `0.35` threshold or moisture near `0.80`).
  - Allow periodic re-sampling after a minimum cooldown $\Delta t_{\text{resample}}$ (with distinct timestamped `sample_id = f"{robot_id}:{site_id}:{kind.value}:{tick}"`) so a borderline patch can be confirmed by a second reading before committing scarce seed inventory.

### Idea 3 — Hopper Drop Dispersion & Motion-Induced Landing Error (`src/ricebot/robot/`)
- **Problem:** When `planter` stops at the outer edge of `probe_radius_m` (`0.22 m` from site center) and drops a seed vertically, the seed lands ~`0.21 m` from the exact patch center.
- **Proposed Simulation Enhancement:**
  - Model the physical offset of the rear/center seed-drop nozzle in the rover body frame $(x_{\text{nozzle}}, y_{\text{nozzle}})$ plus small soil-impact restitution on uneven loam so `PlantFeedback.landing` reflects nozzle alignment relative to `site.center`.
  - **Why it helps evaluation:** Enables the independent evaluator (`src/ricebot/evaluation/`) to score both **patch eligibility** (did the seed land on a viable stand gap rather than a mud hazard?) and **placement precision** ($\|\text{landing} - \text{site.center}\|_2 \le r_{\text{target}}$).

### Idea 4 — Four Standardized Benchmark Scenarios for the ≤60s Demo & Grader (`src/ricebot/world/` + `src/ricebot/evaluation/`)
To make the final evaluation and video demo falsifiable across both MuJoCo and Isaac Sim, standardize 4 named scenarios selectable via `--scenario`:
1. **`nominal` (Cooperative Scout + Planter Division of Labor):**
   - `scout` sweeps nearby unknown patches (`patch_a`, `patch_d`) and flags *Lannitian* mud hazards into `MeasuredFieldMap`; `planter` skips rejected mud pans and plants its 2 seeds into `patch_b` and `patch_c`.
2. **`obstacle` (Unannounced Mid-Episode Bund Collapse / Crate Obstruction):**
   - At $t = 1.5\text{ s}$, an unannounced obstacle enters the active transit lane. Forward 3-ray LiDAR detects `front_range_m < 0.35 m`, triggering immediate safe `HOLD` or waypoint bypass around `no_go_radius_m`.
3. **`shifted_moisture` (Hidden Spatial Regime Shift Across Seeds):**
   - Across randomized episode seeds (`seed=1..20`), which of `patch_a..patch_d` are waterlogged mud hazards vs. viable stand gaps permutes deterministically from `seed`. **Hypothesis to test:** shared mapping may outperform blind/static planting on changing synthetic soil maps; measure actual success and failure distributions rather than declaring 50%/100% results in advance.
4. **`sensor_degraded` (Rangefinder Dropout / Stale Command Fault Injection):**
   - Injects a 1.0-second rangefinder dropout (`front_range_m = None`) or expired command window to prove `LocalController` enforces fail-safe zero-velocity hold with zero collisions and zero ghost seed drops.

### Idea 5 — Matched-Seed Baseline Comparison Table (`src/ricebot/evaluation/`)
When implementing Issue #13 (`src/ricebot/evaluation/`), evaluate every scenario seed across three matched policies using the exact same `MujocoPaddySimulation` / `IsaacPaddySimulation` physics:
- **Baseline 0 — Uniform / Blind Grid Planter:** Visits permitted sites in fixed index order without soil probing; wastes seeds on waterlogged *Lannitian* mud pans (`patch_a`, `patch_d`).
- **Baseline 1 — Single-Rover Planter Alone (`scout` disabled):** `planter` must physically drive to every patch itself to sample before planting; succeeds eventually but incurs higher travel distance and episode completion time.
- **Proposed Method — Cooperative Dual-Rover Shared Belief Map (`scout` + `planter`):** `scout` screens candidate patches in parallel while `planter` services verified stand gaps, maximizing **Viable Seed Establishment Score** per unit time with zero mud-hazard drops and zero protected-crop intrusions.
