# Runtime architecture — Rice flood response

**Status:** selected *working project direction*, architecture specification **proposed**, implementation and simulator runtime **not yet demonstrated**. The human integrator owns the first executable vertical slice and final interface freeze. The human robot-design owner decides geometry/sensors/actuators; these are not hardware claims. See [PRODUCT](PRODUCT.md) for the user need and minimal demo, [CONTRACTS](CONTRACTS.md) for versioned messages and [TEAM](TEAM.md) for work boundaries.

## Architectural principle

**The agent is not the simulator, controller or UI.** Keep a narrow testable boundary: **simulator/world → measured observation → estimator/supervisor → bounded task intent → local controller/actor adapter → physical simulation → next observation.** Independent evaluator sees privileged world truth; planner never does. Read-only visualisation receives a render/telemetry copy but **never directly drives the robot or owns the world state**.

```text
          INDEPENDENT SCENARIO / EVALUATOR (hidden truth, event injector)
                               |
                               v
           WORLD ENGINE + ROBOT PHYSICS + SIMPLE WATER PROXY
                  |                            ^
      sampled rover sensors                  | bounded actuation
                  v                            |
          ObservationV0 ────> MISSION SUPERVISOR (monitor, inspect, mitigate, hold)
                                      |
                             TaskIntentV0
                                      v
                     LOCAL NAVIGATION / SAFETY CONTROLLER
                                      |
                              DriveCommandV0
                                      |
                           SIM ACTUATOR ADAPTER
                                      
           read-only renderer / UI  <--- render snapshot and trace
           evaluator / JSONL       <--- hidden truth and actual results
```

**v0 policy:** one robot and one changed world. Do not confuse system-level planner (which task, why investigate) with local numerical controller (how to move). The first runnable loop may use a straightforward reactive supervisor until the real simulator seam is validated. No LLM/large RL training, network service or extra agent subprocess is a requirement.

## Proposed module ownership and seams

These are **planned paths** for the next coding agent to create, **not files that already exist**. Names may be adjusted once to fit a proved working engine, with one integrator signing off before parallel workers begin.

| Boundary / proposed path | Responsibility | Owner role | First acceptance |
| --- | --- | --- | --- |
| `src/ricebot/contracts.py` | Typed, validated `ObservationV0`, `TaskIntentV0`, `DriveCommandV0` and step result; version + units | **Integrator only** | contract unit tests, invalid/stale command rejection |
| `src/ricebot/sim/adapter.py`, `scene.xml` | MuJoCo (preferred if verified) world step/reset and embodied rover sensor/actuator bridge, sealed hidden state | **Integrator initially**, later simulation owner for `sim/world/` | wheel joint actually moves; range/encoder changes in next observation |
| `src/ricebot/robot/` | Robot geometry/MJCF, mobility and safe local controller; collision, speed/turn bounds | **Human robot/mechanics owner** + scoped coding agent | physically credible rover path + collision/timeout stop |
| `src/ricebot/agent/` | Observation-driven supervisor: anomaly triage, inspection viewpoint, planning, actor selection and optional bounded reasoning | **Agent/planning owner** | substitutes fixed, nearest and belief-aware decisions through identical API |
| `src/ricebot/world/` | Discrete irrigation geometry, sensor placement, water-level/flow **proxy**, outlet and scripted event injection | **World/simulation owner** | route block or water rise occurs inside engine, not delivered as an oracle |
| `src/ricebot/viewer/` | Read-only view of rover, measurements, chosen action, episode metrics | **Evaluation/UI owner** | can disable UI without changing control outcomes |
| `src/ricebot/evaluation/`, `tests/ricebot/` | Hidden-state grader, reproducible scenarios, competing baselines, traces and video runner | **Evaluation/UI owner**, integrator coordinates shared runner | same seeds, evidence + failing tests, independent scoring |
| `src/ricebot/runner.py` | Single composition root, CLI, timings, event ordering, stop/reset and recording | **Integrator only** | one command runs nominal and surprising episodes on a clean machine |

**Engine choice:** organiser email favours laptop MuJoCo; try it first on actual team machines. PyBullet is a permitted fallback if a real sensor/physics loop runs sooner. Do not split into independent UI physics and agent physics. No assumed GPU/Modal/Isaac access.

## First executable vertical slice — mandatory integration gate

1. A **human integrator names the first owner**; verify clean current `main`, system Python, MuJoCo availability, and chosen rover model. If MuJoCo cannot run, record the exact failure and pick one allowed fallback with human agreement.
2. In a **single owned integration branch**, define only the `v0` dataclasses and fixture needed for **one rover, one forward range sensor, wheel motion and a sudden obstacle**. The simulator adapter returns only measurable observations; its hidden state is evaluator-only.
3. Make one physical scene: service-bank track, rover at point A, inspect marker B; wheel actuation is bounded, the rover senses an unexpected obstruction, changes steering/route or stops safely. Record the observed sensor before/after the action.
4. Add a minimal station water-level sample to support real **flood investigation** decisions (with source/validity), then tie a task intent to inspect a channel checkpoint. Keep water-flow proxy intentionally simple and labelled.
5. Add a test for `reset(seed)`, bounded `step(action)`, monotonic observation sequencing, stale-action rejection, unexpected event reaction, and independence from UI or remote service.
6. Only **after a real local smoke run**: integrate modules from other humans through agreed `v0` contracts; commit an actually reproducible runner and update [RUN](RUN.md) with measured commands and dependencies. **An unrun suggested command is not a verified smoke test.**

**Suggested future CLI (not implemented or tested):** `python -m ricebot.runner --scenario blocked_route --seed 7 --headless`. The next agent must choose/create a correct installable package layout and replace this with the exact tested command; do not tell judges it already works.

## Timing, state and safety

- **Physics time:** fixed simulation step chosen by implementation owner and recorded; controller and planner rates are independent, with bounded, logged decision latency. Do not hard-code LLM calls inside motor loops.
- **Frames:** world X/Y in metres, yaw in radians; positive forward speed `m/s`, turn rate `rad/s`; origin/handedness and camera/range frames recorded in simulator adapter. Odometry/pose **estimate** is not hidden physics truth.
- **Inputs:** timestamped samples with source, freshness/validity, rover status and battery proxy. A station/fixed sensor is a distinct origin from rover sensors. Unknown/out-of-range water readings are `null` or explicitly invalid, **not zero**.
- **Safety:** cap velocity/turn; enforce no-go/crop boundary, collision/stop condition, battery reserve if modelled, stale command expiry, watchdog stop/hold; commands with unknown actors or malformed units fail closed.
- **Events:** the world/scenario controller injects obstacles, water rise, blocked outlet, drift or link loss at runtime; agent gets **only the physical sensor consequences**. Do not encode the event label as an agent feature.
- **Mitigation actor (v1 stretch):** a simulated hinge/valve must move visibly on an authorised request and update a **labelled simplified** water proxy; invalid gate actuation refuses/aborts. Its control becomes a separate actor interface, not a fake change of ground-truth numbers by the planner.
- **Rescue actor (v2 stretch):** additional rover shares the same observation/task/actor interfaces. Real contact/towing requires verified physics. If not, only demonstrate dispatch/reallocation with explicit limitations.

## Adaptation beyond rice — design for extension, not implementation scope

Use distinct **environment adapters, sensed capabilities, actor registries and mission objectives** rather than rice-specific fields inside all planner decisions. A `hazard` has observable evidence, uncertainty, urgency, available investigations, authorised interventions, constraints and independent outcome. A new application **must implement its own physical sensing/action adapter and prove its own results**:
- **Reforestation:** monitor a damaged planting patch, investigate uncertainty, decide protect/plant/defer.
- **Firewatch:** watch alert → dispatch safe scouting / authorised response, with human escalation.
- **Space:** inspect degraded route/equipment, allocate constrained recovery work.
- **Swarm rescue:** a robot fault triggers inspect/self-recover/wait/assist/escalate and perhaps physical helper action.

The demo is **rice only**. Do not create generic plugin registries, orchestration microservices, extra simulated worlds or dozens of actor classes before the rice rover works.

## Independent proof and ownership boundaries

The world/evaluator may read true blockage, water state, collision geometry and successful interventions. A policy receives `ObservationV0` only; an evaluation copy may annotate source truth **outside the policy API**. The UI must remain read-only; hiding it cannot change run outcome. Each module owner proves a fixture and hands it to the integrator; one integration owner resolves schema changes. Document actual control, sensing, latency, simulator engine and runtime topology in [RUN](RUN.md).

