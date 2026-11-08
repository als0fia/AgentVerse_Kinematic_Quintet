# Antenna architecture

**Release candidate: single authoritative MuJoCo application, 8 November 2026.**

The selected product is a **rescue-first collective robotics simulation**. The only authoritative implementation is **src/ricebot/** plus the run/evaluation scripts. The separate historical seasonal prototype at **src/riceswarm_ai/** is retained for provenance and its own regression tests, not used to drive the Antenna demo.

## One episode and two alternative controllers

~~~text
               Seed + scenario + fleet N + field scale
                               |
                  build_field_layout()
                 + world truth generation
                               |
                MujocoPaddySimulation
         physical boundaries / rover models / sensors
                               |
                      ObservationFrame
                               |
                     EpisodeSession
          reset / tick / telemetry / decisions
                               |
               selected acting controller
            +-------------------+--------------------+
            |                                        |
       MissionCore                          DecentralizedMycelialRuntime
   CooperativeSwarmPlanner               distinct per-rover belief/SLAM
     shared measured map                  discovered/probed sites and EIG
   deterministic bids/risks                 delayed peer gossip + bids
            |                                        |
            +-------------------+--------------------+
                               |
               incident / rescue reasoning
        bounded orchestrator tools and safety gate
                               |
                   typed TaskIntent
                  + RobotCommand
                               |
       motor, contact, dock, latch, extract, seed release
                               |
                   MuJoCo physics step
                               |
          measured recovery / planting / mobility feedback
                               |
             Independent evaluator + read-only GUI
~~~

**No separate animation/world-state machine determines outcomes.** Presentation Mode only reads observable data and moves cameras; the simulation produces the actual rover trajectories and physics.

## Module boundaries

| Module | Authoritative responsibilities |
| --- | --- |
| **src/ricebot/contracts.py** | Typed observations, sensed samples, robot capability, task intent, robot command, rescue feedback and units |
| **src/ricebot/world/terrace.py** | Deterministic field layout, site list, physical scale, soil properties, hazard seeds and truth grids |
| **src/ricebot/robot/rovers.py** | Articulated rover MJCF geometry, sensors, wheels, farming tool, docking/latch actuator definitions |
| **src/ricebot/mujoco_sim/adapter.py** | One compiled MuJoCo model, physical step/reset, seed-contact and recovery evidence |
| **src/ricebot/runtime.py** | EpisodeConfig and EpisodeSession, active controller choice, action/event trace and reset semantics |
| **src/ricebot/mission.py / control.py** | Baseline measured-site map and conservative bounded navigation/plant control |
| **src/ricebot/agent/planner.py** | Shared-map cooperative policy and mission-aware rescue triage |
| **src/ricebot/agent/mycelial_swarm.py** | Per-rover private mapping, visual discovery, local probes, gossip, consensus task bids and decentralized rescue |
| **src/ricebot/agent/orchestrator.py / rescue.py** | Tool-mediated incident reasoning, deterministic offline fallback, hard safety checks and bounded recovery techniques |
| **src/ricebot/diagnostics.py** | Non-authoritative monitoring and physical failure traces |
| **src/ricebot/evaluation.py** | Independent physical/safety/fieldwork scoring on completed runs |
| **src/ricebot/viewer/live_app.py / presentation.py** | Desktop engineering and judge-facing views; no ground-truth writeback to planner |

## Field scaling and rover identity

**1.0× controlled fixtures** preserve the original small, calibrated benchmark sites and scenario-specific seed inventories, so golden rescue and high-risk refusal comparisons remain reproducible.

**1.6× expanded fields** enlarge the physical MuJoCo boundaries, position more planting sites in the world, generate additional soil truth, use safe per-rover spawn placement, extend local belief-map extents and give each active rover the same sampling/planting hardware capabilities and default inventory. The simulator creates distinct physical models for **2, 4 or 6 rovers**. Those models are not simply copied markers on a zoomed-out dashboard.

Forwards-compatible internal identifiers retain some legacy **planter/scout** names for compatibility; these names are **not valid evidence of hardware specialization** in expanded homogeneous-fleet mode. Current work/task state is independent of the name. Users should assess actual allocated tasks and per-rover physical movement in the trace.

A configurable seed must reproduce the same initial physical world, robot spawns, soil truth and resulting deterministic controller actions. Changed-seed resets rebuild the scene and must rebind the viewer's model/renderer references.

## Two coordination strategies

**Cooperative / shared map:** a central measured map collects sampled data from multiple rovers. The cooperative mission planner bids and assigns work and implements the best-characterized golden rescue/refusal fixtures. This is a credible deterministic baseline, not an AI-less straw man.

**Mycelial / decentralized:** each rover owns its local map and spatial belief. Locally sensed sample, scene and hazard data can be exchanged through compact, simulated, delayed peer packets. Normal, isolated and intermittent communication modes permit matched ablations. The selected Mycelial runtime is the actual acting controller when requested; a separate passive observer may exist for diagnostics, but must not be confused with the chosen policy.

The network model is **software-simulated**. It is neither an RF packet capture nor proof of physical distributed inference; the robots still run within one MuJoCo process.

## Rescue and physical safety

1. Detect a persistent fault using commanded motion, observed position/yaw and available sensor validity. A deliberate HOLD or missing range reading alone should not provoke a rescue.
2. Construct an incident with known helper availability, opportunity cost, estimated physical risk, task value and probe results.
3. Choose rescue, defer or abandon/escalate under a specified policy. A tool-using orchestrator may query a live model **only when explicitly enabled**; otherwise the deterministic reasoner handles it.
4. Apply an independent **safety gate**, then typed speed/turn/docking/latch/recovery constraints. LLM text cannot directly write motor torques.
5. Observe the actual MuJoCo contact, latch, victim displacement, helper safety and subsequent rover autonomy/seed contact. Record failed rescue attempts, vetoes and secondary casualties.

The controlled cooperative golden scenario validates latch and extraction in a prepared recovery case. Severe rollover and nose-down modes have partial component-level coverage, not proven complete autonomous end-to-end recovery across arbitrary terrain.

## Evaluation and evidence boundaries

The independent evaluator can use hidden synthetic world truth to score viable planting and danger. The controller must use **only permitted observations**, never the evaluator's hidden truth. The [release verification evidence](../experiments/release_verification_20261108.json) contains 18 actual MuJoCo simulation runs at 1.6× field scale, including six matched gossip comparisons.

Experiment records retain the effective acting policy, fleet count, field scale, seed, per-rover motion, physical recovery events, seed placements, gossip packets and failures. A numerical success field inherited from the two-rover benchmark must not be used as a generic six-rover success rate.

**Known limitations:** terrain and soil parameters are synthetic and not calibrated against real crops; configured mud hazards are not independently measured natural failure statistics. Recovery and collision success is scenario-dependent. The visual heightfield is not evidence of fully physical deformable ground; validate specific collision and traction semantics before making a contact-model claim. No live-model gain or real-world farm hardware has been established.

## Run and test

Follow [RUN](RUN.md) for commands and [TESTING](TESTING.md) for exact test outcomes. The independently verified Windows candidate (32ce0a3) passed 142 tests without skips. The later Presentation Mode commit (60c7283) passed 149 tests in Linux CI with two headless GUI skips; a separate real Windows GUI recording remains a submission acceptance step.

For historical design discussion, ownership and source reconciliation use [DECISIONS](DECISIONS.md), [COMPONENTS](COMPONENTS.md), and the retained engineering history. Those records are **not** the current product status.
