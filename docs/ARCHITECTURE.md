# Runtime architecture — cooperative rice planting in Isaac Sim

**Canonical mission:** [PaddyPatch PRODUCT](PRODUCT.md), superseding RiceWatch alarm-only monitoring. **Isaac Sim is the user-selected simulator** (version/platform not yet verified). P0 requires **physical planting**, locally measured heterogeneous field conditions, shared evidence from **two embodied rovers**, site selection, safe route/task coordination and changed-world adaptation. Flood mitigation and robot-to-robot physical rescue are optional.

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

## Parallel module seams (proposed paths, not present yet)

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
- Low-priority options **only after P0 verified:** robot fault/rescue orchestration, flood-water monitoring/physical outlet intervention, reforestation or space domain adapters.

## Integration falsification

The product fails its core claim if no seed physically contacts a legitimate planting patch, if map values come from hidden truth instead of measured rover samples, if second rover has no distinct contribution, if a changed route triggers a scripted scene swap, or if apparent success is an unverified UI animation. One self-contained actual runnable episode with *physical seed placement and task-driven replanning* matters more than additional agents.

