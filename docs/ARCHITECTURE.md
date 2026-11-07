# Runtime architecture — Antenna rescue-first collective rice-field robotics

**Current canonical mission:** Antenna is **rescue-first**. Two robots do a small amount of useful fieldwork and must handle a genuine stranded teammate. The mission-level supervisor should reason across robot state, human priorities and calculated risk, use tools to dispatch/monitor a recovery, and adapt when its first plan fails; field mapping and planting remain context. The new Paddy Ant rigid-dock MJCF and scripted towing test prove one **test-level** physical mechanism, while the command-versus-motion incident classifier has separate passing tests; neither proves automatically actuated recovery or resumed field operations. MuJoCo is checked in; Isaac Sim remains unverified. See [PRODUCT](PRODUCT.md), [SWARM_EDGE_PLAN](SWARM_EDGE_PLAN.md) and D-0009.

**Update after merged PR #28 (7 Nov 2026):** `src/ricebot/agent/mycelial_swarm.py` now implements **simulated Policy B**: separate `LocalMycelialMap` and `LocalSlamGrid` instances for each rover, event-triggered `MycelialGossipPacket` exchanges with modelled distance/link blackout, provenance deduplication, and task bids. Tests include isolated vs connected maps, blackout recovery and changed planting target. It runs inside a shared Python/MuJoCo simulation; **no physical UWB or Wi-Fi HaLow radios, deployed distributed processes, Jetson inference or integrated autonomous physical rescue have been verified**. Policy A's `CooperativeSwarmPlanner` remains a centralised baseline. The merged PR #24 separate scripted MuJoCo rigid-docking/tandem extraction test is physical simulation mechanics, not the full sensed-rescue mission.

## Rescue supervisor and existing docking mechanism: integration gap

`CooperativeSwarmPlanner` still aggregates bids in one process from a single shared map. Existing `rescue_assist(...)` uses `SAMPLE` to route to a rendezvous. Separately, `src/ricebot/robot/rovers.py` models the homogeneous Paddy Ant rescue mechanism and `tests/test_world_and_robot.py::test_mujoco_rigid_docking_cam_latch_and_tandem_extraction` scripts approach, latch, pitch-lift and reverse drive; it asserts physical victim displacement greater than 0.08 m. **These two components are not an integrated autonomous rescue loop.**

Next ownership seam: physically observable fault → typed bounded evidence and incident → strategic rescue/investigate/abstain assessment → validator and a new **recovery skill command** driving the actual docking mechanism → independent contact/extraction result and safe resumption of rice tasks. One integrator updates shared schema; world owner supplies fault/soil; human robot designer validates clearance, latch, force and anchor; agent owner handles decision; evaluator owns hidden truth. A learned skill and P2P partial beliefs are separately testable extensions, not assumed.

## Current module ownership and mock/real replacement boundary

The [canonical component guide](COMPONENTS.md) splits the **existing** MuJoCo world/probe, homogeneous robot/mechanical rescue, one simulation composition writer, `rice.v1` kernel integrator, coordination/agent and independent evaluator. Keep the **implemented** `SimulationPort.reset(seed)/step(commands)` seam; add the smallest compatible fault/recovery action/result integration through its sole owner. Any future Isaac translation would reuse that seam and require separate physical validation.

For the weekend, **reuse the committed MuJoCo rover bodies and small field**, not a second mock world or a new Isaac scene. The human mechanical owner controls dock/traction limits; the integrator wires actual contact, result and stop feedback. A future asset/engine swap would require retesting geometry, force, collision and sensor/actuator mapping under the [existing swap checklist](COMPONENTS.md#mock-rover-first--approved-designed-rover-next). Never create a second episode kernel, hidden-truth oracle or animation-only recovery.

## Single runtime and one rescue episode (current design target)

```text
              ONE MuJoCo simulation / reset+step loop
 tiny rice-field mission + two homogeneous robots + one traction/fault event
                          | sensed state / physics feedback
                          v
                  rice.v1 ObservationFrame
                          |
              fault evidence + fleet/field context
                          v
       ONE mission supervisor / coordination seam
       - existing: numerical triage and Policy A task bids
       - alternative implemented: simulated Policy B per-rover gossip
       - proposed: LLM tool loop for inspect -> assess -> act -> monitor/revise
                          |
                     risk/safety gate
                          v
         typed recovery skill request (NOT YET IMPLEMENTED)
                          |
          bounded drive -> dock -> latch -> lift/tow
                          |
               MuJoCo physical outcome
                          |
            supervisor feedback + continued work
                          |
          independent evaluator / read-only viewer
```

**Integration gaps:** the `SimulationPort`, rover hardware model, rice world, planting contact and scripted rigid-dock extraction already exist in the shared MuJoCo codebase. **A corrected central fault detector now records actual issued drive commands and filters missing front-range or deliberate safe stops**; its regression tests pass, but the autonomous sensed-fault → rescue decision → typed physical command → measured outcome → task continuation episode **does not exist yet**. `front_range_m=None` is sensor invalidity, not immobilisation; do not use hidden simulator truth in agent decisions. The optional Gemini advisor can currently suggest task intents, not execute a persistent tool loop. Do not show an LLM-generated rescue sentence as an actuated rescue.

**One simulator, alternatives inside it:** use the checked-in MuJoCo composition for the hackathon. Isaac Sim remains a separate, unverified translation target and is **not** a parallel demo obligation. Policy A (central shared map) and Policy B (simulated independent maps/gossip) are alternative coordinators, not two worlds or two simultaneous products. Pick one physical rescue integration path and use the other as a baseline/optional comparison. Keep the same `rice.v1` seam and physics assets.

**Scope/order:** P0 integrate one physically credible recovery skill and safety stop; monitor and react to its feedback (a limited second strategy or abort if genuinely executable). P1, using the **same** supervisor and telemetry, is a bounded start/end-of-day analysis → parameter experiment → validate/deploy/rollback cycle. Do not implement a separate training platform, multi-technique RL suite, realistic suction physics, live radio network, model-parallel Jetsons or a second simulator to prove this product. The 12-case adaptive mud-extraction benchmark described in an external conversation has **not** been located as an executable, committed `main` evaluation and is not currently validated product evidence.

## Human-led robot and farm design

**Human robot lead chooses and approves** the body, wheel/contact setup, mass/inertia, service-lane width, sample probe placement, hopper release location/reach, motor bounds and collision/no-go limits. Two similar small wheeled rovers in a **prepared dry/direct-seeded or drained plot** are a plausible first approximation; the scout reuses body assets and need not carry a hopper. Flooded paddies, planting live seedlings with realistic roots, mud deformation and exact agronomic soil dynamics are excluded until separately modelled and validated.

**Field realism:** 4×4–6×6 candidate patches initially; existing stand gaps; synthetic ground moisture and firmness gradients; varying travel/access; limited planting inventory; known and newly revealed barriers. A soil measurement may be an explicit virtual probe with radius/noise and unit-specific sample, plus local vegetation/stand proxy; **a camera cannot directly know soil nutrients or compaction**. The hidden establishment-benefit model is an experiment scoring proxy, **not** an agronomic forecast of rice yield or survival. Record assumptions and compare to uniform/grid coverage as well as competent map-aware deterministic planning.

## Current MuJoCo prototype versus pending Isaac physics

`src/ricebot/contracts.py`, `mission.py`, `control.py` and `runtime.py` form the **pure-Python** observation → measured map → task → bounded command seam. **`src/ricebot/mujoco_sim/` now implements it in MuJoCo** with articulated rover models, synthetic local probes, range/obstacle scenarios and a physical seed-contact test. Headless CI runs its physics tests with the pinned optional dependency. **The Isaac Sim implementation and cross-engine parity tests remain unimplemented.** The baseline still uses site-local means and conservative straight-line HOLD rather than a complete optimal route planner. One human integration owner controls shared contracts.

## Current implementation seams and minimal rescue integration (one MuJoCo stack)

| Existing owner/module | Existing contribution | Only necessary rescue-first addition |
| --- | --- | --- |
| `src/ricebot/world/` | Small synthetic rice field, local probes and terrain context | One reproducible physical stall and valid observed fault indicators, no new farm simulator |
| `src/ricebot/robot/` | Reusable homogeneous bodies and **scripted** rigid docking/cam-latch/tow physics | Expose one bounded rescue sequence as a callable skill with progress/contact evidence |
| `src/ricebot/mujoco_sim/` | **Current** shared MuJoCo physics composition and `SimulationPort` | Wire fault, recovery command and result into the **same** step/reset loop |
| `src/ricebot/contracts.py`, `runtime.py`, `control.py` | Typed observations/commands, orchestration and safety-constrained drives | Integrator-owned minimal fault/recovery action/result seam; preserve existing callers |
| `src/ricebot/agent/` | Policy A numerical bids/triage/optional REST advisor; Policy B simulated per-rover maps/gossip | Select **one** coordination integration path, monitor the real rescue result, compare to baseline; optional bounded tool-using LLM supervisor |
| Existing `tests/` and evaluation scripts | Unit and MuJoCo physics checks, field/seed tests and policy comparisons | Independent end-to-end incident/rescue/resumption test on same seeds; true/false incident and safe-abstain cases |

**Dependency order:** (1) physically detectable immobilisation in the existing MuJoCo scene; (2) typed rescue skill invocation that uses the existing rigid dock; (3) observed extraction/second-helper safety; (4) mission continuation and a deterministic comparison; (5) optional LLM tool loop and one tested feedback-driven correction. No team member should build a second world engine, duplicate `SimulationPort`, another crop model or parallel recovery prototype to satisfy these steps.

## Deferred Isaac Sim translation notes — not part of the current rescue P0

- Verify the **actual installed version** and exposed API before importing version-specific extensions. NVIDIA offers OpenUSD scenes, PhysX articulations, raycast/contact/IMU sensors and standalone scripting; use **simpler physics sensors** before heavy multi-camera RTX perception. [Isaac Sim documentation](https://docs.isaacsim.omniverse.nvidia.com/latest/py/index.html).
- Official requirements list **32 GB RAM / 16 GB VRAM minimum** on an RTX GPU for supported full usage; several discussed 8 GB laptop GPUs may be below spec. Run the compatibility checker on the actual host, then measure an actual physics/render episode. Do not promise that an 8 GB machine or remote streaming is functional. Cloud host requires explicit provisioning/cost approval. [Official requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html).
- Use a fixed sim clock and timestamped per-robot sample provenance. Start with **inverse-distance/nearest-neighbour interpolation** for a small field, A*/Dijkstra or bounded graph search along actual traversable lanes. Gaussian processes/Kriging, RL or deliberative LLM reasoning only when baseline comparison supports it.
- If running simulation on a shared/remote workstation, explicitly record **where physics, renderer, each robot controller, map and supervisor execute**; communication loss can't be claimed as onboard autonomy unless a local controller actually keeps working.
- Never use `sim.get_true_soil`, future event flags or directly set seed landing site within policy. Planner may request `PLANT`; actuator releases a real proxy; physics determines landing; grader separately scores hidden synthetic benefit.
- **Superseded planting-only note:** physical robot rescue is now the core priority; flood-water controls, reforestation/space domain transfers remain optional.

## Integration falsification

The product fails its **core rescue-first claim** if a physically genuine incident is not identified from valid evidence, if only a rendezvous or text decision is made without a bounded physical skill, if simulated docking merely replays a standalone script, if helper safety and victim displacement are not independently measured, or if no meaningful fieldwork resumes after a claimed recovery. The existing seed-contact and separate per-rover soil observations are necessary *mission context* and must remain honest, not become another world to build. An abstention is success only when its risk reasoning and alternative task outcome can be demonstrated.

