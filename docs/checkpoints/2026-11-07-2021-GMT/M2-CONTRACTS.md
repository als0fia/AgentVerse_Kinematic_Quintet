# M2 — Antenna contracts, duplication and integration truth

**Mission:** M2 — skeptical independent systems integrator · **Report status:** **READ-ONLY / PARTIAL (source and CI verified; local physics NOT RUN)**  
**Recorded:** 2026-11-07 20:15 GMT (Europe/London) · **Canonical team repository:** [`als0fia/AgentVerse_Kinematic_Quintet`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet)  
**Pinned `main`:** [`130e881a54b932dcbe66971674e266b344e104f6`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/commit/130e881a54b932dcbe66971674e266b344e104f6) (merged PR #34 at 19:57 GMT).  
**Latest observed `main` (snapshot delta; separate from pinned-source analysis):** [`6c3a5752d73c1a1a2ed42479aa919e5f82415f7d`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/commit/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d), one descendant commit after pinned `130e881`; new UI/analytical mobility attenuation. See the dated **post-snapshot addendum** for superseded findings and new P0 risks. Its separate [CI run 42410512960](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/actions/runs/42410512960) passed 72 tests.
**Open proposal separately inspected:** [PR #32](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/32), head `6175e09e7c3a28cfe17c40b2988cc772153d071d`; **not merged**, documentation-only, branched before recent `main` changes.  
**Local review environment:** Python 3.13.5; MuJoCo not installed; `git ls-remote` failed because the container cannot resolve `github.com`. Source fetched through the authorized GitHub connection. **No local repository checkout, unit test, simulator, worktree or replay was run.** CI evidence below is from GitHub's separate Ubuntu job, not this environment.  
**Team checkpoint destination:** `docs/checkpoints/2026-11-07-2021-GMT/M2-CONTRACTS.md` in the public Antenna team repository. This independent report pins its own audited source snapshot; a coordinator index was not available for this audit.
**Audit scope:** The source/CI investigation was read-only. This report-only PR does not modify application code, tests, simulator behaviour or final submission.

## Decision summary (≤250 words)

**Antenna already has meaningful physical-simulation building blocks, but there is no implemented autonomous rescue call chain.** The canonical headless runner instantiates `DecentralizedMycelialRuntime` (Policy B) or `SimpleMissionPlanner` via `MissionCore` (Policy A). Neither calls `CooperativeSwarmPlanner`, which owns the existing heuristic rescue triage. Even that planner's `RESCUE_NOW` becomes a `SAMPLE` task aimed at the nearest planting site, not a contact/docking skill. The MuJoCo scene includes rigid latch equality constraints and actuators, but the normal `MujocoPaddySimulation.step()` never drives them or invokes `update_rigid_dock_latch`. The distinct test fixture *does*: CI verified that the isolated physics test passed, not that rescue follows a sensed incident.

Immediate integration work should **extend the existing `rice.v1`/`SimulationPort` path** to (1) observe a reproducible physical movement failure without misclassifying range-sensor loss, (2) request and validate a bounded recovery skill, (3) drive the existing docking/tow mechanism, and (4) independently measure victim displacement, helper safety and resumed fieldwork. Preserve planting and Policy B gossip, but keep the parallel six-rover `riceswarm_ai` lifecycle simulator outside the judge-facing runtime.

**Most urgent owner:** named human integration owner (currently **NOT VERIFIED**) for `contracts.py`, `runtime.py`, `mujoco_sim/adapter.py` and `scripts/run_mujoco_paddypatch.py`; human mechanics owner must approve robot recovery changes. No model should claim complete rescue before the same-seed executable episode exists.

## One-screen actual architecture

```text
Canonical team main / actual run_mujoco_paddypatch.py
   CLI (--policy mycelial | centralized) [line 503 onward]
          |
          +--> MujocoPaddySimulation (ricebot/mujoco_sim/adapter.py)
          |       |  ONE MuJoCo scene: world.build_world_terrain_xml
          |       |  + robot.build_rovers_and_seeds_xml
          |       |  + INACTIVE latch equality constraints (no live latch driver)
          |       |  reset(seed) -> ObservationFrame; step(RobotCommand[]) -> ObservationFrame
          |       |  read rangefinder, simulator frame-position pose, local synthetic soil
          |       +--> independent seed-contact log (not recovery outcome)
          |
          +--> EpisodeRunner.run  [same application clock/step port]
                  |
                  +-- DEFAULT Policy B: DecentralizedMycelialRuntime.decide
                  |       per-rover LocalMycelialMap/LocalSlamGrid + simulated gossip
                  |       auction -> SAMPLE/PLANT/HOLD -> own local wheel/drop controller
                  |       [NO RESCUE TRIAGE OR RECOVERY ACTUATION]
                  |
                  +-- Policy A: MissionCore.decide
                          MeasuredFieldMap -> SimpleMissionPlanner
                          -> LocalController -> wheel/drop RobotCommand
                          [NO RESCUE TRIAGE]

Separate code available, but NOT in above runner:
  CooperativeSwarmPlanner + optional GeminiSwarmAdvisor (agent/planner.py)
      heuristic RESCUE_NOW/DEFER/ABANDON -> SAMPLE rendezvous;
      tests exercise planner in synthetic scripted frames only.
  robot.update_rigid_dock_latch + standalone rescue MJCF test
      manually drives latch and pitch + reverse wheels; CI isolated success.
  riceswarm_ai/core.py + CLI + 6-robot discrete lifecycle simulator
      incompatible Field/Robot/Decision model; MuJoCo bridge moves mocap visuals.
  viewer/dashboard.py / scripts/evaluate.py
      read-only heatmaps and generic matched JSONL metrics, not rescue scoring.
```

**Important nuance:** The `viewer_seconds` path in the canonical MuJoCo script manually drives the *same* simulation/runtime loop for rendering, whereas headless uses `EpisodeRunner`; this is duplicate loop logic and needs headless/viewer parity testing, but is **not** evidence of a second physics world. The separate `riceswarm_ai` package **is** a separate world.

## Capability ledger (implemented source vs actual evidence)

| System / actual entry point | `main` or other | Actual caller / schema | Evidence observed | Status for Antenna rescue |
|---|---|---|---|---|
| `rice.v1` types (`contracts.py`) | Main | `ObservationFrame`, `TaskIntent`, `RobotCommand`, `PlantFeedback` | Foundation tests + CI | **Working base**, no recovery action/result types |
| Mission loop (`runtime.py`) | Main | `EpisodeRunner -> sim.reset/step`, `MissionCore.decide` | CI unit tests and physical planting tests | **Working for planting**, not recovery |
| Main MuJoCo physical world (`mujoco_sim/adapter.py`) | Main | one `MujocoPaddySimulation` with world/robot builders | CI planting/contact/range tests | **Working physical seed and obstacle slice**, not immobilisation |
| Physical rover assets / equality constraints | Main | `rovers.py` MJCF/`update_rigid_dock_latch` | CI isolated extraction test passed | **Tested in isolation**, not commanded by main adapter |
| Synthetic soil and continuous terrain (`world/terrace.py`) | Main | location-gated `SoilSample`, hidden site truth, continuous terrain | Unit/CI | **Working deterministic synthetic sensor**, not physical mud fault |
| Policy A simple map/plan (`mission.py`,`control.py`) | Main | `MissionCore` baseline in runner | Unit/CI and MuJoCo planting | **Working planting baseline** |
| Rescue triage (`agent/planner.py`) | Main | `CooperativeSwarmPlanner`, selected in **no current CLI mode** | `test_agentic_swarm` scripted tests | **Code tested in isolation; not live runner** |
| Optional Gemini advisor (`agent/planner.py`) | Main | API `generateContent` one-shot assignment override; no CLI instantiation | mocked advisor tests; no live model proof | **Code only / optional API**; not tool-using rescue supervisor |
| Policy B (`agent/mycelial_swarm.py`) | Main, merged PR #28 | `DecentralizedMycelialRuntime.decide` in canonical runner | CI simulated local-map/gossip + physical seed tests | **Working simulated coordination**, no physical rescue |
| Policy B simulated SLAM/peer ranges | Main | `process_local_sensors(frame, peer_poses)` | unit/CI; derived from global frame poses | **Synthetic local logic**; no deployed radio/localization |
| Telemetry + dashboard (`viewer/dashboard.py`) | Main | `ReadOnlyView.consume`, optional HTML export | CI dashboard smoke | **Supporting/optional**; not independent rescue evaluator |
| Generic JSONL comparison (`scripts/evaluate.py`) | Main | independent CLI, `case_id/seed/success/elapsed_ms` | unit/CI | **Working generic harness**; no rescue episode producer/scorer |
| Six-rover RiceSwarm simulation (`src/riceswarm_ai`) | Main, historical package | own CLI, `RiceSwarmSimulation`, `RecoveryPolicy`, `MujocoBridge` | CI lifecycle and discrete rescue tests | **Separate alternative baseline**, not integrated Antenna physics |
| Isaac Sim adapter | No implemented `src/ricebot/isaac/` tree | docs only | No verified Isaac run | **Not implemented in this snapshot** |
| Learned recovery controller / offline Jetson inference / real P2P | Not found in current executable source | research docs only | No checkpoint, hardware trace or deployment proof | **Missing/unverified** |
| Full sensor→decision→physical rescue→resumed task | Not found | no executable call chain | **No integrated test found** | **BLOCKER** |

### Canonical vs duplicate truth/control inventory

| Domain | Canonical for 60-second rescue | Other implementation | Recommendation |
|---|---|---|---|
| Scene/physics | `compose_paddypatch_mjcf` in `mujoco_sim/adapter.py` | standalone `build_rigid_docking_rescue_mjcf` test scene; separate `riceswarm_ai/mujoco_bridge.py` | Reuse normal scene's **existing** docking constraints; retain standalone physics fixture; park mocap scene |
| Clock and runner | `EpisodeRunner.run` + `SimulationPort` | canonical script's manually duplicated viewer loop; six-rover `RiceSwarmSimulation.run` | Keep one clock. Exercise headless/viewer parity, don't create another launcher |
| Mission planner | current selectable Policy B for fieldwork, Policy A as a baseline | `CooperativeSwarmPlanner` not selectable; six-rover recovery coordinator unrelated types | Add one incident supervisor over the selected fieldwork runner; preserve policies as controls/baselines |
| Map and beliefs | Policy B per-rover maps, or `MeasuredFieldMap` in Policy A | six-rover `Field.belief`; `ContinuousPaddyTerrain` hidden truth and read-only dashboard truth | Keep simulator truth out of agents; never bridge two incompatible field models |
| Fault/risk | existing heuristic in planner (not live) | `evaluate_bekker_terramechanics`, `world.mud_traction_scale`, six-rover grid risk, HUD metrics | One physically observable fault definition; analytics can be auxiliary, never assumed to apply traction |
| Model APIs | optional `GeminiSwarmAdvisor` (not running) | `riceswarm_ai/GeminiRecoveryAdviser` (other world); proposed PR #32 tool loop | Choose at most one optional mission-level model interface after deterministic path passes |
| Evaluators | `completed_drops` and `scripts/evaluate.py` | six-rover discrete success flags; dashboard and plant-summary output | New *independent* recovery measurement in the same MuJoCo episode; keep other comparisons explicit |

**Minimum active system count:** one canonical MuJoCo world/scene + physical rover/controller asset + episode contract/runner + fieldwork policy/map + incident detection/supervision + bounded recovery controller + independent evaluator = **seven functional responsibilities, one simulation backend and one top-level launcher**. Policy A/B selection and dashboard can remain as *contained modules*, not separate apps. No additional AI client or Jetson service on the safety-critical path.

## Findings with exact inspected evidence

### F1 — BLOCKER: existing rescue triage is not called by any canonical runner mode

- [`scripts/run_mujoco_paddypatch.py#L277-L316`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/scripts/run_mujoco_paddypatch.py#L277-L316): `policy == 'mycelial'` instantiates `DecentralizedMycelialRuntime`; else instantiates `SimpleMissionPlanner` inside `MissionCore`. CLI options are only `mycelial` and `centralized` ([L503-L513](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/scripts/run_mujoco_paddypatch.py#L503-L513)).
- [`agent/planner.py#L620-L699`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/agent/planner.py#L620-L699) implements a separate rescue heuristic and makes a helper's `RESCUE_NOW` task **`TaskMode.SAMPLE` at a field site**. [`contracts.py#L145-L167`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/contracts.py#L145-L167) has only SAMPLE/PLANT/HOLD.
- **Consequence:** nothing in the judge-facing default CLI responds to a stalled rover by dispatching a docking action; a synthetic triage result cannot independently prove recovery. **Rubric:** perception, adaptation, ambition.
- **Cheapest fix:** human integrator selects **one** existing fieldwork policy and inserts a narrow incident supervisor with real command/status seam, rather than replacing the entire planner. **Test:** same-seed source-frame anomaly leads to distinct `RecoveryCommand` (not a `SAMPLE` task), then a measured physical state change.

### F2 — BLOCKER: normal MuJoCo adapter never drives the physical docking mechanism

- [`mujoco_sim/adapter.py#L48-L94`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/mujoco_sim/adapter.py#L48-L94) composes the normal scene with two `active="false"` latch equality constraints.
- [`robot/rovers.py#L375-L389`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/robot/rovers.py#L375-L389) defines pitch/latch/spade actuators at indices 8–13. [`adapter.py#L263-L331`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/mujoco_sim/adapter.py#L263-L331) zeros controls, writes wheel offsets 0–7 and optional farming tools but **never** drives those rescue actuators or invokes `update_rigid_dock_latch`.
- In contrast, [`tests/test_world_and_robot.py#L135-L172`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/tests/test_world_and_robot.py#L135-L172) manually drives latch, pitch and reverse wheels in a **different** fixture, and calls [`update_rigid_dock_latch`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/robot/rovers.py#L479-L520) every step. The test asserts >0.08 m victim movement, not fault diagnosis, secondary casualty or task resumption.
- **Consequence:** assets exist, but physical recovery is not reachable from typed application commands. **Rubric:** perception→action, adaptation, ambition.
- **Cheapest fix:** only the owner of `adapter.py` and existing robot mechanic connects a bounded skill phase to actuator controls and latch monitor; do **not** construct a new physics scene. **Test:** from regular `MujocoPaddySimulation`, observe pre/post latch state and victim displacement with no manually injected `eq_active` or direct external torque.

### F3 — BLOCKER/HIGH: immobilisation observation and fault semantics cannot support honest dispatch

- [`contracts.py#L77-L109`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/contracts.py#L77-L109) permits fields `true_speed_m_s`, `pitch_rad`, `roll_rad`, `peer_distances`, `is_bogged`, but the actual [`adapter.py#L400-L419`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/mujoco_sim/adapter.py#L400-L419) omits them, leaving contract defaults (`None`, zero, empty, `False`). It also hard-codes `link_ok=True`. Thus the `LocalController` bogged/pitch/peer safety tests ([`control.py#L56-L70`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/control.py#L56-L70)) do not demonstrate those protections from the actual adapter.
- [`adapter.py#L362-L369`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/mujoco_sim/adapter.py#L362-L369) converts **all missing/invalid range rays to `4.0`** (apparently clear), contradicting contract's `front_range_m=None` meaning invalid/missing ([`contracts.py#L82`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/contracts.py#L82)).
- [`agent/planner.py#L364-L390`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/agent/planner.py#L364-L390) labels a robot stranded **immediately when range is `None`** even though that indicates sensor loss, or after blocked/immobile ticks. It uses previous task target rather than measured nonzero wheel command; HOLD/intent can be confused with immobilisation.
- [`mycelial_swarm.py#L373-L435`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/agent/mycelial_swarm.py#L373-L435) estimates movement/slip from pose differences and *last requested* speed, but only as Policy B map/telemetry logic and uses fabricated pitch/roll degrees, **not as integrated health fields or rescue supervisor input**.
- **Consequence:** sensor dropout might be called stranded, while an actual stuck rover may produce no fault event in the default runtime. **Rubric:** perception, safety, adaptation.
- **Cheapest fix:** add timestamped commanded-vs-achieved movement history, time window and independent sensor-health category in the **same** MuJoCo adapter/incident supervisor. Never convert missing measurements to "clear". **Test:** compare intentional HOLD, missing range, transient obstacle, persistent stuck wheel and true rescue-relevant immobilisation.

### F4 — HIGH: no physically modeled rice-mud immobilisation is demonstrated in the normal scene

- [`world/terrace.py#L373-L380`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/world/terrace.py#L373-L380) defines `mud_traction_scale` and [`robot/rovers.py#L612-L704`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/robot/rovers.py#L612-L704) computes analytical immobilisation/sinkage, but the normal `adapter.step` writes wheel targets without referencing either ([`adapter.py#L303-L331`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/mujoco_sim/adapter.py#L303-L331)).
- The standalone rigid rescue fixture uses a `mud_trap_pan` geom with collision disabled (`contype="0" conaffinity="0"`) and physically tows a peer placed in a favorable initial geometry, not a validated traction failure ([`rovers.py#L421-L460`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/robot/rovers.py#L421-L460)).
- Current `--scenario` offers nominal/obstacle/bad_soil only ([`run_mujoco_paddypatch.py#L503-L513`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/scripts/run_mujoco_paddypatch.py#L503-L513)); no true immobilisation scenario is selectable.
- **Consequence:** synthetic soil readings and the word "mud" do not prove a dynamic stuck/recovered event. **Rubric:** perception, relevance, technical quality.
- **Cheapest fix:** introduce **one bounded, clearly labeled synthetic physical fault** in the existing scene that reduces actual achieved motion/traction or imposes an observable obstruction; verify observed-vs-commanded status and independent extraction. Realistic mud CFD is unnecessary. **Test:** failed movement while nonzero drive is commanded; physical extraction restores mobility without teleporting/clearing a success flag.

### F5 — HIGH: success grading can overcount a planted site, and there is no rescue evaluator

- [`adapter.py#L239-L248`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/mujoco_sim/adapter.py#L239-L248) independently verifies seed-ground contact, and the entrypoint [`run_mujoco_paddypatch.py#L344-L355`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/scripts/run_mujoco_paddypatch.py#L344-L355) counts "viable seed placements" if the *intended target site* has favorable hidden soil and contact exists, **without checking the seed's landing pose lies within that site**. As written, it can potentially count a misplaced seed as viable.
- `PlantFeedback` carries a landing pose, but no rescue result, latch state, actual victim change, secondary-casualty result or return-to-fieldwork event exists in [`contracts.py#L111-L142`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/contracts.py#L111-L142).
- [`scripts/evaluate.py`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/scripts/evaluate.py) compares generic JSONL success/latency only; it does not itself generate independent physics outcomes.
- **Consequence:** even a green summary cannot prove correct planting *or* autonomous rescue. **Rubric:** demonstrated action, quality, impact.
- **Cheapest fix:** independent scorer reads actual simulator positions, constraints and contact (not the model's verdict); check target-radius landing and recovery displacement/helper state. **Test:** displaced seed is not counted; "rescued" string without physical movement fails.

### F6 — HIGH: default Policy B lacks the existing local controller's health/safety gates

- Policy B local drive issues wheel/drop `RobotCommand` directly ([`mycelial_swarm.py#L562-L705`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/agent/mycelial_swarm.py#L562-L705)), never testing `obs.is_bogged`, `obs.pitch_rad`, `obs.roll_rad`, or using `LocalController.command`. It does model peer repulsion when distances are under 0.44 m, whereas `LocalController` has a separate 1.0 m proximity brake triggered by populated `peer_distances` ([`control.py#L20-L22,L66-L70`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/control.py#L20-L22)). Neither path has a controlled narrow docking-approach exception.
- `MujocoPaddySimulation` enforces TTL and physical seed capability ([`adapter.py#L250-L309`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/mujoco_sim/adapter.py#L250-L309)), and wheel velocity targets are mechanically clipped in `compute_4wheel_skid_steer`, but **health and collision gating are inconsistent by selected policy**.
- **Consequence:** simply setting a new `is_bogged=True` field will not stop the current default Policy B driver. A blanket anti-collision brake would also make 6.5 cm capture impossible. **Rubric:** autonomy/safety.
- **Cheapest fix:** preserve both policies; put a common **adapter-level fail-closed recovery/safety authority** under both that explicitly manages normal collision separation versus authorized approach-to-dock and timeouts. **Test:** Policy A/B cannot drive an unsafe flagged robot and a recognized recovery skill can approach only its designated victim under approved bounds.

### F7 — MEDIUM/HIGH: hidden simulator coordinates and synthetic radio are presented as robot-local sensing

- The `RobotObservation.pose` contract says **estimated/odometric, never simulator truth** ([`contracts.py#L77-L82`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/contracts.py#L77-L82)), but [`adapter.py#L371-L388`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/mujoco_sim/adapter.py#L371-L388) reads MuJoCo `framepos` and `framequat` straight into the supplied pose. The `*_odom_site` name does not turn a global `framepos` into integrated wheel odometry.
- Policy B creates `peer_poses` from **all robots' frame poses** before any blackout/link checks ([`mycelial_swarm.py#L759-L789`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/agent/mycelial_swarm.py#L759-L789)) and every node computes exact inter-peer range/bearing from those poses ([L403-L414](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/agent/mycelial_swarm.py#L403-L414)). Network blackout is simulated for gossip ([L739-L748](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/agent/mycelial_swarm.py#L739-L748)), **not** a loss of all peer position observability or actual deployed UWB.
- Local soil readings are deterministic direct values of hidden per-site truth when entering radius ([`world/terrace.py#L344-L371`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/ricebot/world/terrace.py#L344-L371)). This is acceptable if explicitly described as a synthetic proximity-gated probe, **not** noisy or independently calibrated crop science.
- **Consequence:** partial-map code is real, but it is not a physically independent local radio/SLAM stack. **Cheapest fix:** label the limitations, ensure incident policy sees only an explicitly modeled permitted observation subset, and avoid demo claims of fully private per-rover localization/communications.

### F8 — MEDIUM: dual CLI/simulation stacks and stale commands invite accidental false integration

- Historical [`src/riceswarm_ai/core.py`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/riceswarm_ai/core.py), [`cli.py`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/riceswarm_ai/cli.py#L49-L105) and [`ai_agent.py`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/riceswarm_ai/ai_agent.py) implement a separate discrete six-rover farming/recovery system with different Field/Robot/Decision types. Its [`mujoco_bridge.py#L38-L79`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/riceswarm_ai/mujoco_bridge.py#L38-L79) creates **mocap bodies** and writes discrete robot positions into `data.mocap_pos`; this is visualization, not dynamics-driven rover extraction. Its passing `test_full_mud_rescue_and_resumption` is **not evidence** for the `ricebot` physical scene.
- Its package README still says `python sample/riceswarm_ai/run_demo.py` and `PYTHONPATH=sample` ([`src/riceswarm_ai/README.md#L43-L109`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/src/riceswarm_ai/README.md#L43-L109)), while the actual source tree is `src/riceswarm_ai/`: likely stale launch instructions for that **alternative** stack.
- **Consequence:** an AI may demonstrate a discrete `successful_recoveries` counter and mistakenly claim main MuJoCo rescue. **Cheapest fix:** no deletion; label alternative / historical in instructions, point demo commands only at `scripts/run_mujoco_paddypatch.py`.

### F9 — MEDIUM: test/benchmark coverage is strong for components, weak at application seams

- GitHub [CI job `128511786182`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/actions/runs/42408654176/job/128511786182) checked out **exact `130e881...`**, installed Python 3.12 + `mujoco==4.0.0`, and completed **72 unit/integration tests in 5.270s, OK**. Test names in logs include physical planting, obstacle safe HOLD, standalone docking extraction, gossip, and discrete six-robot mud rescue. This is *real CI evidence*; this M2 session did not execute those tests.
- [Workflow YAML](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/.github/workflows/foundation-checks.yml#L15-L26) runs repo refs/skills and unittest discovery, **not** the headless `scripts/run_mujoco_paddypatch.py` CLI nor a combined rescue episode.
- [`tests/test_mujoco_sim.py#L26-L55`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/tests/test_mujoco_sim.py#L26-L55) verifies fieldwork/seed contact, [L57-L88](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/tests/test_mujoco_sim.py#L57-L88) checks obstacle-triggered stop, and [`tests/test_agentic_swarm.py#L132-L179`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/tests/test_agentic_swarm.py#L132-L179) exercises rescue verdicts with **handcrafted** `front_range=None` frames. No test found for *real fault in regular MuJoCo scene → rescue plan → latch → displaced victim → resumed fieldwork*.
- `run_policy_comparison_matrix` ([`run_mujoco_paddypatch.py#L421-L455`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/scripts/run_mujoco_paddypatch.py#L421-L455)) compares differing case steps and artificially assumes centralised comms consumes `steps * 2 * 128` bytes ([L381-L391](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/scripts/run_mujoco_paddypatch.py#L381-L391)). Do **not** claim measured physical network efficiency or matched rescue superiority from it.

### F10 — MEDIUM: current docs/branches are not fully reconciled at latest main SHA

- [`docs/RUN.md#L23-L27`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/docs/RUN.md#L23-L27) still tells the next agent to implement Isaac and explicitly says "No ... robot rescue," contradicting the newer Antenna rescue-first heading. [`docs/TEAM.md#L25-L27`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/130e881a54b932dcbe66971674e266b344e104f6/docs/TEAM.md#L25-L27) likewise retains planting-first priority prose. `src/riceswarm_ai/README.md` has stale path commands. **These are current-line contradictions at `130e881`, not merely historical dates.**
- [PR #32](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/32) proposes to reconcile eight docs toward one MuJoCo rescue-first focus, but is **open** and based on `da27c8c...`; main has moved. Its text expressly says it **implements no new runtime**, so cannot close F1–F5 by merging.
- Compared using GitHub commits endpoint against `main`: `feature/agentic-swarm-intelligence` was **0 ahead / 18 behind**; `feature/homogeneous-robot-rigid-rescue` **0 ahead / 40 behind**; `feature/mujoco-simulation-port` **0 ahead / 55 behind**; `feature/riceswarm-mvp` **0 ahead / 39 behind**. Those named branches are **not unmerged newer functionality**. `feature/ui` branch comparison returned 404; **not evaluated**, do not claim clean/stale from the name alone.
- Actual local-only worktrees and uncommitted team features **cannot be inspected from GitHub**. The human integrator must independently verify any local work before assigning shared-file integration.
- **Consequence:** agents following old RUN/TEAM sections may spend limited time on Isaac or planting-only features; PR #32 may conflict with unrelated newer prose. **Cheapest fix:** integration owner reconciles documentation after choosing runnable path; no speculative cleanup or branch deletion.

## Precise seam matrix

| Producer → consumer | Present schema / units/frame/time | Actual state and error semantics | Readiness / minimal repair |
|---|---|---|---|
| MuJoCo rangefinder → `RobotObservation.front_range_m` → mission/controller | `float|None`, metres, frame `time_s` | Adapter returns `4.0` when all rays invalid; consumer needs `None` for missing, and planner treats `None` as stranded | **MISMATCH**: return `None` on invalid and separate sensor-loss from motion fault; fixture both cases |
| MuJoCo rover pose → planner/Policy B | `Pose2D(x,y,yaw)` metres/rad, frame-global; `time_s` sim secs | `framepos/framequat` ground-truth-based, not integrated odometry; no uncertainty | **LIMITATION**: label estimate/source or add bounded observation model; never claim full real SLAM |
| MuJoCo body dynamics → incident detector | No command-achieved displacement telemetry in regular observation; fields optional/default | No confirmed physical stuck event or time-windowed evidence | **MISSING**: producer records command + measured speed/displacement with age, health state and confidence |
| `world.sample_local_soil` → Policy A/Policy B map | `SoilSample` 0–1 dimensionless, site/local pose, sample at exact frame time, unique IDs | Location-gated but exact hidden synthetic site values; no noise | **READY for synthetic work** with explicit caveat; preserve |
| Policy A `SimpleMissionPlanner` → `LocalController` → adapter | `TaskIntent` SAMPLE/PLANT/HOLD; `RobotCommand` m/s, rad/s, TTL +0.2 s | Actor matching in controller, TTL in adapter, seed capability check; no rescue | **READY for fieldwork / MISSING recovery** |
| `CooperativeSwarmPlanner` rescue verdict → helper task | `RescueAssessment` values unitless hand-weights; `TaskIntent` SAMPLE target site, task reason `rescue_assist(...)` | Dispatch reaches a field site rather than victim latch geometry | **MISMATCH**: typed target robot/skill, safety preconditions, timeout and ack |
| Policy B node→wheel adapter | `RobotCommand` +0.2s TTL; peer repulsion from all poses | Does not honor optional bogged/IMU safety checks; can create false movement when stuck | **MISMATCH**: common fail-closed health/safety gate with authorized dock exception |
| Recovery command → dock actuator / equality latch | MJCF controls 8–13; physical dock sites in metres, latch angle rad | Normal adapter zeros actuators; no typed command/ack; fixture directly calls latch function | **MISSING live seam**: bounded recovery phase, latch monitor, abort/detach, independent feedback |
| Simulated comms → per-rover belief | gossip packets with `sent_at_s`, robot IDs, byte_size, range; blackout schedules | Single-process memory exchange; all peer poses available outside blackout; no actual radio | **WORKING simulated comparison**; keep optional; no hardware claims |
| Plant physics feedback → core/evaluator | `PlantFeedback`, release bool, contact time/pose, target task | Contact measured; summary scorer not checking landing in intended site | **PARTIAL**: enforce landing within site radius and ground contact |
| Recovery physics → evaluator/video | No `RecoveryResult`, no extracted-displacement/secondary-risk ledger | No independent rescue scorer or continuous replay in repo | **BLOCKER**: evaluator reads hidden physical truth, model sees bounded status only |
| Generic log → `scripts/evaluate.py` | JSONL (`case_id`, `seed`, `success`, `elapsed_ms`) | Strict matched keys, but no actual rescue logger yields these records | **TOOLS READY / SOURCE MISSING**: add real per-case producer; disallow fabricated rows |

**Clock/ownership:** `MujocoPaddySimulation` advances `data.time` by 25 × 0.004 = **0.1 simulated seconds per call** (source-level calculation; not locally measured); `EpisodeRunner` ticks, Policy A/B checks strictly increasing frame time, and commands expire in roughly 0.2 sim seconds. New rescue skill may persist across ticks but **must not introduce an independent clock**. One `episode_id`, one frame order, one actor ID registry and acknowledgment provenance should be preserved.

**Security/invalid outputs:** `RobotCommand` checks finite values and positive TTL, and `compute_4wheel_skid_steer` clips speeds; the adapter ignores expired commands. It does not yet expose a typed recovery rejection/result, all commands keyed by robot ID are last-writer-wins in a dictionary, and unrecognized robot IDs are ignored by the adapter rather than independently acknowledged as invalid. The human integrator should define deterministic safe HOLD on rejected recovery phases, not create a bypass for model-generated actuator values.

## Five minimal change packets — ordered for the integration owner

| # / priority | Human owner / minimal paths | Dependency & precise acceptance | Rollback / stop condition | Rough effort (estimate, not promise) |
|---|---|---|---|---|
| **1 / P0 — One fault with trustworthy sensing** | World+adapter owner, coordinated by integrator: `mujoco_sim/adapter.py`, small existing-scene fault fixture, `tests/test_mujoco_sim.py` | For same seed, nonzero commanded wheel motion yields statistically meaningful low observed displacement over several ticks; sensor-dropout alone **does not** declare stuck; `None` preserved | Disable fixture injection; stop if only a static label changes without dynamics | 45–90 min for simple physically observable synthetic fault if local MuJoCo available |
| **2 / P0 — One typed recovery request/result** | **Single shared-contract writer M1**, with M4/M6 proposals: `contracts.py`, `runtime.py`, matching tests | Data contract includes victim/helper IDs, skill/phase, source frame age, TTL, capability, geometry preconditions, rejection/ack and observed result; SAMPLE remains SAMPLE | Revert schema delta if it breaks existing planting tests; fallback HOLD | 30–75 min |
| **3 / P0 — Use existing docking actuator in regular scene** | M1 adapter integrator + human robot-mechanics owner: `mujoco_sim/adapter.py`, `robot/rovers.py` only if approved, **new seam test** | Regular scene can safely align, latch, lift, move victim > fixed tolerance (initial prototype >0.08 m), detach/abort; no direct scripted setting of success or teleport | Disable recovery adapter path, retain isolated test/planting; stop if latch never couples or helper destabilizes | 60–120 min, highly uncertain physics work |
| **4 / P0 — Independent measured end-to-end episode** | Independent evaluator owner + M1, test/report path agreed: `tests/test_mujoco_sim.py` or dedicated `tests/test_integrated_rescue.py`; same runner/scene | Distinct tested cases: rescue, justified refusal, bad sensor, failed latch, secondary helper hazard, mid-episode disturbance, fieldwork resumption; seeded log with observed displacement and actual tool/command chain | If rescue fails, video/report an honest bounded refusal or isolated demonstration—not synthetic success | 45–90 min once seam works |
| **5 / P1 — Canonical instructions/CI smoke and baselines** | Human integrator/docs owner: `docs/RUN.md`, `docs/TEAM.md`, `docs/DEMO.md`, `.github/workflows/foundation-checks.yml` only if stable | Clean checkout can run one canonical command; CI invokes headless smoke or well-bounded integrated regression; PR #32 merged/reconciled only after conflict review; retained `riceswarm_ai` labeled alternative | Leave historical docs on other branches; no mass deletion; stop before destabilizing runtime hours | 20–45 min |

**Race prevention:** M1 is the only proposed shared `contracts.py`/`runtime.py`/entrypoint writer. M4 may investigate robot/MJCF geometry **read-only**; human robot lead approves any changes. M5 owns fault-world fixture proposals, but cannot independently change the scene while M1 integrates. M6 designs tool schema but cannot separately update live contracts. M7 owns independent evaluation and must not alter tests to make a failing model pass. Existing issues [#7](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/issues/7), [#11](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/issues/11), [#12](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/issues/12), [#13](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/issues/13) are legacy planting-oriented work definitions; human owner should reuse carefully, not auto-create competing tickets.

## Reproducible verification packet — **NOT RUN here**

The following commands are **proposed for the human's physics-capable machine** at the pinned commit, with `python` referring to the active environment. The actual CI did run the first three checks, except the final CLI smoke, on Ubuntu Python 3.12 with MuJoCo 4.0.0. This M2 session ran none of these commands.

```bash
python -m pip install -r requirements-mujoco.txt
python scripts/check_repo.py
python scripts/sync_skills.py --check
python -m unittest discover -s tests -v
python scripts/run_mujoco_paddypatch.py --headless --seed 7 --steps 140 --policy mycelial
python scripts/run_mujoco_paddypatch.py --headless --seed 7 --steps 140 --policy centralized
python -m unittest discover -s tests -p 'test_world_and_robot.py' -v
```

**Observed external evidence:** CI `Foundation checks`, [run `42408654176`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/actions/runs/42408654176), on exact `main` SHA, showed `MuJoCo 4.0.0`, `Ran 72 tests in 5.270s` and `OK`, including `test_mujoco_rigid_docking_cam_latch_and_tandem_extraction`. Full runner smoke **not in CI**. 

**Needed new integration test packet (no code yet):**

1. `seed=7`, two physical rovers, 3–6 real sampling/planting sites; collect robot poses, command TTLs, range values, physical wheel/body movement and sampled site IDs at each tick.
2. At known **injection** time create *physical* failure (not model label), with observed drive-motion discrepancy across time and no privileged failure ID in agent input; keep evaluator-only truth.
3. Run one permitted incident supervisor, verify actor ID, type, valid TTL, reason, source frame, target; forbid raw model torque.
4. Approach/capture/lift/tow through **normal** `MujocoPaddySimulation.step`, log constraint state and actual transforms; verify victim moved, helper did not become a second casualty, and successful/failed latch is distinguishable.
5. Continue existing fieldwork and compare a matched same-seed deterministic baseline. Test a false-range-fault and helper-danger refusal; log wall-clock/model latency separately from sim seconds.

**Failure trace to isolate first:** on current `main`, a genuine high-level `RecoveryCommand` cannot even be constructed because the shared schema has no recovery mode, and the normal adapter has no latch command branch. This is a **source-proven missing path**, not an exception captured from running a failing test.

## Current readiness per seam

| Gate | Verdict | Evidence |
|---|---|---|
| Clone/import core + existing tests | **CI verified** | 72 passing tests at pinned SHA, Ubuntu |
| Headless normal mission CLI | **Source exists; NOT RUN in this session and not CI-smoked** | runner script |
| Two physically simulated moving rovers + seed contact | **Isolated/integrated planting test passed in CI** | `test_mujoco_sim` |
| Surprise obstacle → range → safe HOLD | **Physics test passed in CI** | `test_mid_episode_obstacle...` |
| Per-rover gossip and changing plant target | **CI tests passed**; single-process simulated radios | `test_mycelial_swarm` |
| Actual physical rover immobilisation from modeled fault | **NOT VERIFIED** | no executable normal-scene incident fixture |
| Sensor fault vs immobilisation distinction | **CONTRADICTED by code** | F3 |
| Incident triage → typed recovery command | **MISSING** | F1 |
| Recovery command → normal scene latch/tow | **MISSING** | F2 |
| Independent helper safety / displacement / resumed work | **MISSING** | F5 |
| Optional contextual tool-using model supervisor | **PROPOSED only** | PR #32; current advisor is one-shot |
| Isaac Sim hardware/cloud/Jetson/real radio | **NOT VERIFIED** | no implementation/proof |
| ≤60-second continuous verified rescue recording | **NOT VERIFIED / no finished evidence found** | existing docs say proposed/not recorded |

## Conflicts, M1 comparison, and final recommendation

- **M1 report/plan:** **not provided / not available**. This analysis was recorded without consulting any M1 report. When M1 becomes available, compare its actor/controller/seam choices against F1–F6; demand an actual latest-SHA trace before calling integration done. If M1's local WIP is not on GitHub, label it **unverified**, not missing globally.
- **Open PR #32** accurately scopes a tool-using rescue supervisor, but does not implement one. Its scope advice is broadly consistent with the independently derived minimal path. The still-open PR's changed files need reconciling with later `main` before any merge; docs do not replace code/tests.
- **Strongest legitimate current claim:** two simulated MuJoCo rovers can perform local synthetic soil sampling/physical seed work in existing tests; rigid dock/lift/tow mechanics can move a peer >0.08 m in a **scripted separate** MuJoCo test; Policy B simulates per-rover belief gossip.
- **Claims that are presently unsupported:** automated sensed physical peer rescue, an adaptive context/tool-using LLM controller, genuinely learned rescue policy, actual mud physics recoverability, independent deployed P2P radios, pooled Jetson inference, robust production farm economics and a verified end-to-end 60-second rescue movie.
- **Top recommendation to human integrator:** integrate **one actual physics failure→bounded recovery action→measured result** through current MuJoCo runner and existing robot asset before broadening modelling or AI. If that cannot be achieved, retain the functioning planting and isolated extraction demonstrations, explain that the connecting supervisor is incomplete, and demonstrate justified safe abstention honestly. Do not build a second program, silently merge old branches, or make the test's `>0.08m` displacement synonymous with autonomously recovered mission.

**Team publication target (7 Nov 2026 correction):** `docs/checkpoints/2026-11-07-2021-GMT/M2-CONTRACTS.md` in the actual public team repository. Human integrator reviews and merges the dedicated PR; preserve the original inspected snapshot and append dated deltas rather than silently rewriting the evidence.


---

## Post-snapshot delta — latest `main` `6c3a5752d73c` (7 November 2026, after 20:19 GMT)

**Evidence boundary:** The preceding findings, file/line links and initial capability ledger are explicitly pinned to `130e881a54b9`. GitHub then advanced `main` **one commit** ([#36 merge / commit `6c3a5752d73c`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/commit/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d)), changing `src/ricebot/mujoco_sim/adapter.py`, `scripts/run_mujoco_paddypatch.py`, and `src/ricebot/viewer/` (including a new 1,400-line `live_app.py`). There is **no proof from the diff of a new supervisor or physically completed autonomous rescue**. Open PR #32 remains documentation-only and open. The report's pinned findings remain as historical evidence; the following qualifications control **current `main`**.

### Delta D1 — F3 partially superseded: new health flags, still no trustworthy fault-diagnosis seam (**HIGH**)

- [New adapter `#L444-L499`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/mujoco_sim/adapter.py#L444-L499) **now populates** `pitch_rad`, `roll_rad`, and `is_bogged` in `RobotObservation`; the old F3 statement that *none* of those fields are populated is **stale for new main**. `true_speed_m_s` and `peer_distances` remain absent, `link_ok=True` remains hard-coded, and [`_read_min_front_range_m`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/mujoco_sim/adapter.py#L431-L439) still reports `4.0` if every range ray is invalid. The existing [`CooperativeSwarmPlanner` fault classifier](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/agent/planner.py#L364-L390) still conflates `range=None` with being physically stranded and still does not run in the canonical CLI.
- `is_bogged` is calculated from `evaluate_bekker_terramechanics`'s hidden terrain/analytical `bellied_out` state, not independently inferred from time-windowed measurements. It is currently effectively an **oracle-labeled synthetic health flag**, unless demonstrated as a calibrated or otherwise allowed on-robot measurement. It may be used as transparent synthetic instrumentation in a test, but not represented as independently perceived soil failure. M5/M6 should determine the permissible agent-facing evidence.
- The default `DecentralizedMycelialRuntime` still does **not** gate its local velocity commands on `is_bogged`, pitch or roll: [Policy B `compute_local_command`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/agent/mycelial_swarm.py#L562-L705). F6 remains open. The controller's guards are not automatically applied to the current default runner.

### Delta D2 — F4 partially superseded: modeled attenuation now changes drive command, not demonstrably recoverable mud contact (**HIGH**)

- The new adapter [evaluates terrain and the analytical sinkage model each tick](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/mujoco_sim/adapter.py#L229-L249). Its [drive branch](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/mujoco_sim/adapter.py#L354-L378) scales requested `linear_m_s` and `angular_rad_s` **before** wheel velocity actuation and sets that factor to zero when `bellied_out`. Therefore old F4's blanket claim of *no applied mud-based mobility effect* is stale on latest main. The model now can create a synthetic immobilisation by suppressing commanded wheel targets.
- This is **not yet physically modeled contact/friction resistance or a demonstrated recoverable suction trap**: the code does not apply mud drag/contact forces or a terrain state that visibly changes because a second rover lifts/extracts the first. Critically, if `bellied_out` stays true after towing, the drive targets remain suppressed. A recovery test must measure both physical displacement and whether the victim can drive again under defined release conditions. Keep the distinction *simulated attenuation* versus *physics-based extraction* explicit.
- For fault detection, compare the high-level nonzero **requested** velocity against achieved body motion and the emitted wheel command; never infer an intentional STOP merely from the attenuated zero wheel drive. M1/M4 need the same actor/episode clock and source timestamps.

### Delta D3 — New F11: the so-called read-only dashboard is also a world/actuation input (**HIGH / ownership violation**)

- [`src/ricebot/viewer/live_app.py#L192-L206`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/viewer/live_app.py#L192-L206) writes `sim.robot_mass_kg`, `sim.robot_wheel_radius_m` and `sim.robot_wheel_width_m` then invokes the simulator's `_update_rover_terramechanics()`. [`trigger_mud_belly_out_demo` at L740–746](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/viewer/live_app.py#L740-L746) sets the values to force a synthetic belly-out. The dashboard is **not read-only**, notwithstanding its README description and the `ReadOnlyView` principle. This is a *demo control panel* with simulation mutations. It can be legitimate as a clearly labeled operator-induced test injection, but cannot be presented as an independent visualization or spontaneous, unannounced incident.
- **Smallest fix:** route any permitted scenario/robot-size change through a separately labeled **test injector/operator control**, log timestamp, actor, parameter and prior/new value in episode trace, and keep the inspector/viewer consumer truly read-only. Freeze incident configuration and hidden scenario events during scored episodes unless the rules explicitly allow controlled disturbance injection. M1 owns simulator API; UI owner edits only UI, with a mutually agreed injection seam.

### Delta D4 — New F12: third manually driven runtime loop and incidental GUI dependencies (**MEDIUM, demo fragility**)

- New runner [`--dashboard` / `--gui`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/scripts/run_mujoco_paddypatch.py#L511-L533) calls [`launch_live_dashboard`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/viewer/live_app.py#L1383-L1400); the new UI's [own loop at L765–803](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/viewer/live_app.py#L765-L803) calls `sim.step` and `runtime.decide` directly, and resets by constructing a new sim/runtime. Combined with the previous headless `EpisodeRunner` and passive MuJoCo viewer loop, there are now **three paths that drive ticks**, though all currently use the same physical adapter family. That is duplicated *episode composition*, not necessarily three physics backends. No headless/UI parity test is present in current CI.
- [`ricebot.viewer.__init__`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/viewer/__init__.py#L1-L13) unconditionally imports `live_app`, which unconditionally imports `tkinter` and `mujoco` ([`live_app.py#L24-L42`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/viewer/live_app.py#L24-L42)). This may break a headless/minimal-machine *viewer import*, despite the older HTML recorder being conceptually read-only. Avoid putting GUI dependencies in the critical headless execution path; use a lazy GUI export/import when practical and test in an actual display-capable workstation.
- **Acceptance:** same seed/policy/episode configuration produces matched core decisions and independent physics outcomes in headless and GUI; GUI absent should not prevent headless runner; hidden grader truth remains outside the policy. If parity fails, demo the tested headless path and record an honest replay; do not delay autonomous rescue for UI cosmetics.

### Latest CI evidence and current recommendation

GitHub's [newest CI run `42410512960`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/actions/runs/42410512960), triggered for **exact** `6c3a5752d73c`, completed successfully on Ubuntu; retrieved job logs show `MuJoCo 4.0.0`, `Ran 72 tests in 5.340s`, `OK`. This verifies existing tests remain passing after the UI/adapter change. **No new rescue test, GUI run, slider/engine parity check, sensor-health separation or end-to-end autonomous recovery trace was identified.** The headless CLI itself was still not executed by this M2 session.

**Updated highest-impact order at newest main:** (1) keep M1's **one shared runtime/contract writer** and get a physical failure with defensible sensor evidence; (2) connect bounded rescue phases to the *existing scene's* latch, pitch and joint commands; (3) independently verify actual extraction, helper safety, recoverability and mission continuation; (4) neutralize dashboard-controlled outcome leakage for the scored run; (5) verify a single canonical headless episode and parity with whichever viewer is used. No part of `6c3a...` establishes finished autonomous recovery. The initial F1, F2, F5, F6, F7, F8, F9, F10 conclusions otherwise remain valid subject to the recorded delta.

**Publication boundary:** Findings remain tied to the pinned `130e881...` source and explicitly dated `6c3a...` addendum. Later commits and local-only WIP have not been re-audited in this report. No restricted operational materials or private checkpoint records are included.