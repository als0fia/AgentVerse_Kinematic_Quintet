# Runtime architecture — Antenna rescue-first collective rice-field robotics

**Current canonical mission:** Antenna is **rescue-first**. Physical rice-field soil/stand sensing, map, navigation and seed placement supply a real useful task; the principal agent decision is risk-aware physical recovery or safe abstention. The new Paddy Ant rigid-dock MJCF and scripted towing test prove one **test-level** physical mechanism, but not automatically detected recovery and resumed field operations. MuJoCo is checked in; Isaac Sim remains unverified. See [PRODUCT](PRODUCT.md), [SWARM_EDGE_PLAN](SWARM_EDGE_PLAN.md) and D-0009.

## Rescue supervisor and existing docking mechanism: integration gap

`CooperativeSwarmPlanner` still aggregates bids in one process from a single shared map. Existing `rescue_assist(...)` uses `SAMPLE` to route to a rendezvous. Separately, `src/ricebot/robot/rovers.py` models the homogeneous Paddy Ant rescue mechanism and `tests/test_world_and_robot.py::test_mujoco_rigid_docking_cam_latch_and_tandem_extraction` scripts approach, latch, pitch-lift and reverse drive; it asserts physical victim displacement greater than 0.08 m. **These two components are not an integrated autonomous rescue loop.**

Next ownership seam: physically observable fault → typed bounded evidence and incident → strategic rescue/investigate/abstain assessment → validator and a new **recovery skill command** driving the actual docking mechanism → independent contact/extraction result and safe resumption of rice tasks. One integrator updates shared schema; world owner supplies fault/soil; human robot designer validates clearance, latch, force and anchor; agent owner handles decision; evaluator owns hidden truth. A learned skill and P2P partial beliefs are separately testable extensions, not assumed.

## Current module ownership and mock/real replacement boundary

The [canonical component guide](COMPONENTS.md) splits the world/field/probe owner, human-led robot/seed-tool designer, one Isaac scene-composition/SimulationPort writer, kernel integrator, planner and independent evaluator. The human using ChatGPT and the human using Gemini can exchange the world/robot roles without changing this boundary. Keep `src/ricebot/runtime.py`'s **implemented** `SimulationPort.reset(seed)/step(commands)` interface as the only cross-engine mission seam; the actual Isaac implementation must drive real joints and detect real seed contact.

Develop the 3–6-site world against a *physically actuated* mock rover if the designed rover is not ready. Swap the asset and motor/sensor mapping in the Isaac composition layer after human approval; **retest** clearance, wheel radius, turning limits, mounting frames, reach, release and contact. No second episode kernel, independent planner, direct world-truth read or teleporting seed. The full [mock-to-real acceptance checklist](COMPONENTS.md#mock-rover-first--approved-designed-rover-next) is the reference.

## Physical simulation and task flow

```text
                 ISAAC SIM OpenUSD + PhysX world
    prepared/partly drained rice patch, gaps, synthetic soil,
    existing plants, lanes/no-go, surprise obstacles, seed proxies
       |                                        ^
       | local sampled sensor readings          | actual wheel/seed-drop actions
       v                                        |
  [rover-1: planter]  [rover-2: scouting robot] |
       |                   |                    |
       +---- measured samples & pose estimates--+
                           |
                 SHARED BELIEF MAP
          estimated moisture/firmness/stand gap
         mean, uncertainty, provenance, freshness
                           |
                   MISSION PLANNER
         choose SAMPLE vs PLANT vs HOLD,
         which robot, site and safe route
                           |
             BOUNDED LOCAL CONTROLLERS ---------+
                           |
                independently observed landing
                           |
         READ-ONLY VIEWER + SEPARATE TRUTH GRADER
```

**Separation rule:** Isaac Sim owns all physics and hidden true synthetic soil/stand map. Sensors must be sampled at physical robot positions; an imaginary soil score or object-location oracle is not an observation. Each robot contributes independently timestamped samples to the same map. The mission planner acts on the **estimated** map, not hidden truth. Local controllers own high-rate wheel/obstacle/no-go safety. The planter needs a **physically simulated hopper/dropper and rigid-body seed proxy with observed contact/landing**, not text saying “planted”. UI is a subscriber, not a second world engine. Hidden truth exists only for evaluation.

## Human-led robot and farm design

**Human robot lead chooses and approves** the body, wheel/contact setup, mass/inertia, service-lane width, sample probe placement, hopper release location/reach, motor bounds and collision/no-go limits. Two similar small wheeled rovers in a **prepared dry/direct-seeded or drained plot** are a plausible first approximation; the scout reuses body assets and need not carry a hopper. Flooded paddies, planting live seedlings with realistic roots, mud deformation and exact agronomic soil dynamics are excluded until separately modelled and validated.

**Field realism:** 4×4–6×6 candidate patches initially; existing stand gaps; synthetic ground moisture and firmness gradients; varying travel/access; limited planting inventory; known and newly revealed barriers. A soil measurement may be an explicit virtual probe with radius/noise and unit-specific sample, plus local vegetation/stand proxy; **a camera cannot directly know soil nutrients or compaction**. The hidden establishment-benefit model is an experiment scoring proxy, **not** an agronomic forecast of rice yield or survival. Record assumptions and compare to uniform/grid coverage as well as competent map-aware deterministic planning.

## Current MuJoCo prototype versus pending Isaac physics

`src/ricebot/contracts.py`, `mission.py`, `control.py` and `runtime.py` form the **pure-Python** observation → measured map → task → bounded command seam. **`src/ricebot/mujoco_sim/` now implements it in MuJoCo** with articulated rover models, synthetic local probes, range/obstacle scenarios and a physical seed-contact test. Headless CI runs its physics tests with the pinned optional dependency. **The Isaac Sim implementation and cross-engine parity tests remain unimplemented.** The baseline still uses site-local means and conservative straight-line HOLD rather than a complete optimal route planner. One human integration owner controls shared contracts.

## Parallel module seams (proposed paths, partially implemented)

| Module | What to implement | Owner |
| --- | --- | --- |
| `src/ricebot/contracts.py`, `runner.py` | tested `rice.v1` observation/sample/map/task/drive/drop/result contracts, Isaac launcher/step and logging; only one integration writer | 1 Integrator |
| `src/ricebot/robot/` | human-approved rover bodies, wheel articulated motion, local no-go/collision safe controls, **hopper/dropper physics** | 2 Mechanical + control |
| `src/ricebot/world/`, `sim/` | OpenUSD field, hidden synthetic soil distribution, sample-radius sensor bridge, stand gaps, second rover/scout, injected surprise | 3 World/physics |
| `src/ricebot/agent/` | shared map interpolation/uncertainty, site scoring, active sample-vs-plant decision, robot/site assignment and route planning | 4 Agent/planning |
| `src/ricebot/evaluation/`, `viewer/` | hidden-truth grader, uniform/greedy/map-aware baselines, seed contact outcomes, telemetry and ≤60s continuous demo | 5 Evaluation/UI |

**First freeze:** smallest running Isaac scene + one moving rover + range sensor and `rice.v1` fixture. **Second:** physically drop a seed proxy and prove landing before chasing complex intelligence. **Third:** second moving scout and distinct local samples, shared mean/uncertainty map and route allocation. **Fourth:** inject an unannounced obstruction or locally discovered unsuitable patch; alter actual plant location/route safely; run baselines and record. Merge components in these dependency stages and keep the latest `main` runnable.

## Isaac Sim implementation choices and limits

- Verify the **actual installed version** and exposed API before importing version-specific extensions. NVIDIA offers OpenUSD scenes, PhysX articulations, raycast/contact/IMU sensors and standalone scripting; use **simpler physics sensors** before heavy multi-camera RTX perception. [Isaac Sim documentation](https://docs.isaacsim.omniverse.nvidia.com/latest/py/index.html).
- Official requirements list **32 GB RAM / 16 GB VRAM minimum** on an RTX GPU for supported full usage; several discussed 8 GB laptop GPUs may be below spec. Run the compatibility checker on the actual host, then measure an actual physics/render episode. Do not promise that an 8 GB machine or remote streaming is functional. Cloud host requires explicit provisioning/cost approval. [Official requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html).
- Use a fixed sim clock and timestamped per-robot sample provenance. Start with **inverse-distance/nearest-neighbour interpolation** for a small field, A*/Dijkstra or bounded graph search along actual traversable lanes. Gaussian processes/Kriging, RL or deliberative LLM reasoning only when baseline comparison supports it.
- If running simulation on a shared/remote workstation, explicitly record **where physics, renderer, each robot controller, map and supervisor execute**; communication loss can't be claimed as onboard autonomy unless a local controller actually keeps working.
- Never use `sim.get_true_soil`, future event flags or directly set seed landing site within policy. Planner may request `PLANT`; actuator releases a real proxy; physics determines landing; grader separately scores hidden synthetic benefit.
- **Superseded planting-only note:** physical robot rescue is now the core priority; flood-water controls, reforestation/space domain transfers remain optional.

## Integration falsification

The product fails its core claim if no seed physically contacts a legitimate planting patch, if map values come from hidden truth instead of measured rover samples, if second rover has no distinct contribution, if a changed route triggers a scripted scene swap, or if apparent success is an unverified UI animation. One self-contained actual runnable episode with *physical seed placement and task-driven replanning* matters more than additional agents.

