# Runtime architecture

**Selected application:** rice-field abnormal-water monitoring, active investigation and targeted response. **The robot body, simulator, models and actuator are design proposals, not built or proven.** This is the canonical architecture; the mission is in [PRODUCT](PRODUCT.md), shared wire contracts in [CONTRACTS](CONTRACTS.md), observed tests in [TESTING](TESTING.md).

## End-to-end system: a monitoring decision with physical consequences

```text
Field gauges / rover sensors -> belief + incident triage -> pick next informative task
       ^                                                       |
       |                                                       v
Independent outcome verifier <- physical simulated environment <- safe local rover controller
       ^                                                       ^
       |                                                       |
Mid-run disturbance injector                      optional gate/contact mitigation skill
```

For P0, a **single real simulated rover** receives an abnormal-rise alert, drives to an inspection site using sampled sensors, encounters an unannounced route obstruction or sensor inconsistency, chooses a safe changed course and takes a follow-up observation. For P1, its **physical actuation** changes an explicit, simplified water/drainage model; an independent reading checks actual simulated effect. No scripted UI gate, magically cleared obstruction, oracle truth passed to the planner or mandatory cloud servo loop.

| Functional component | Input → output | Candidate method | Status |
| --- | --- | --- | --- |
| **Field / sensors** | Timed water/flow *measurements*, RGB/depth/range/contacts, time, noise → observations | Transparent simplified water-balance model and simulated sensors | Design only |
| **Rover perception** | Local measured ranges, image/depth, odometry → obstacles and pose *estimates* | Classical perception/state estimation, small classifier only if necessary | Design only |
| **Incident investigator** | Gauge/robot evidence → hypothesis and next useful observation | Bayesian belief + information gain; optional bounded LLM when it improves the baseline | Design only |
| **Task coordinator** | Candidate inspection/response, battery, risk → approved robot task | Constrained planner/allocator; single robot to start | Design only |
| **Robot control / safety** | Target task + fresh local sensing → bounded wheel/steering/contact forces | Deterministic navigation / motion planning / safety stop | Design only |
| **Mitigation interface (P1)** | Inspected gate + reachable tool → bounded manipulation, then new water proxy | Simple lever/contact skill and simulated water balance, **only if implemented** | Wish / stretch |
| **Independent evaluator** | Hidden simulator state + trace → measurements and failed attempts | Scripted world disturbance, automatic physical result checker, same-seed baseline | Design only |

## Robot embodiment is a real engineering decision

**A robot must have a physically plausible body and mechanisms; a moving map icon isn't enough.** For the first demo, the *recommended reference concept* is a compact wheeled inspection rover **travelling on a relatively firm rice-field bund/service edge**, looking into adjacent paddies and visiting an outlet. This is a deliberately restricted **simulated operating envelope**, not a claim a conventional rover can drive through flooded paddy crops.

**Human-led design review is required before implementation commitments**: a teammate should approve where it operates, whether wheels/tracks/flotation are appropriate, approximate chassis dimensions, mass/centre of mass, wheelbase/track footprint, traction and clearance assumptions, motor/steering layout, battery and return reserve, sensor position, water ingress/splash limits, crop/no-go clearance, reach to a gate and safe contact-force limits. The AI may help generate concept options, parametrised models and tests but **must not invent a fabricated robot design as finished hardware** or autonomously decide to procure/modify hardware.

| Design alternative | Benefit | Principal burden and suitability |
| --- | --- | --- |
| **Bund / field-edge wheeled rover (provisional P0)** | Simple reproducible physics, visual line of sight to drain, low crop disturbance, reachable outlet | Edges may be narrow, sloped or slippery; must confirm sufficient clear width and no-fall boundaries in the simulated scene |
| **Tracked muddy-ground rover** | Potential traction on softer soil | Higher turning resistance, disturbance to rice roots/soil, track physics; P2 only if field access demands it |
| **Floating / amphibious rice-row platform** | Can directly inspect submerged field and adapt to water level | Buoyancy, water propulsion, clearance around plants, contact realism and greater simulation complexity; P2 research path |
| **Fixed station + small mobile rover** | Grounded water monitor and mobile verification, with local/offline control | Distributed sensors and possibly radio model; start with a local message interface, not a required second physical robot |

**Mechanical simulation acceptance:** actuated joints/wheels; real collision geometries; reasonable mass and inertia; traversable/non-traversable surfaces, obstacle and crop-protection zones; locally sampled sensors with timestamps; a measurable stop/replan after surprising contact or obstacle; and one explicit fall/slip/battery/blocked-gate failure. A later manipulator must physically reach and operate the outlet with collision/force limits; if it does not, label the mitigation as a *recommendation* only.

**Relevant engineering precedents:** [2026 paddy chassis review](https://pmc.ncbi.nlm.nih.gov/articles/PMC12901939/) reports tracked versus wheeled terrain/soil-disturbance trade-offs; a [2026 floating two-wheel paddy weeding robot](https://www.fujipress.jp/jrm/rb/robot003800020404/?lang=ja) uses floats and a four-bar wheel mechanism. Neither design is automatically our hardware or a ready-made MuJoCo asset. Do not extrapolate demonstrated paddy-row capability from an unverified bund-rover model.

## Generalisable intervention framework, without claiming transfer

Domain-neutral core: **detect uncertain event → decide inspect / wait / intervene / escalate → assign constrained physical skill → verify consequence / replan**. Rice-specific water-level interpretation, bund geometry, crop protection and outlet mechanism are **adapters**; forest restoration and space/construction would need *different bodies, sensors, safety and effect models*. They are pitch-level potential applications until separately simulated and evaluated.

**Robot-to-robot rescue (P2 optional):** when the rover becomes trapped, disconnected or low on reserve, a future fleet agent weighs wait/self-recovery versus human contact versus safe helper dispatch and lost task productivity. A genuine rescue demo would require another physics-enabled robot physically assisting, not only a text assignment. It must not delay P0/P1.

## Build ownership

The human integrator should first verify that one candidate robot model can **move from sensor-conditioned commands in the actual local simulator**. Only then freeze a minimal observation/action schema, assign distinct perception/controller/mission/evaluator modules, and measure each. Core controller stays local; optional LLM reasoning may be offline/unavailable without causing unsafe motion. Record actual simulator/robot choices and tested parameters in [RUN](RUN.md) and versioned message contracts in [CONTRACTS](CONTRACTS.md).

## Required details to confirm as the build proceeds

- **Data and control flow:** observation → optional AI decision → validation → bounded execution → measured outcome; show retries, human override and failure exits.
- **Runtime topology:** executable entrypoints, model/server/process placement, data sources, simulator/robot if any, ports, external APIs and trust boundaries.
- **Units and frames:** metres vs centimetres, radians vs degrees, body/world/camera frames, timestamp origin, coordinate handedness, seed format. Never assume a convention.
- **Performance budgets:** intended latency for control loops vs deliberation, compute and monetary budgets; measure observed values separately.
- **Safety:** tool allowlist, action limits, invalid-state handling, timeouts, permission boundary, kill switch for physical motion.
- **Operational recovery:** expected failures, restart/reset path, deterministic fixtures, fallback seed, clean reproduction command.
- **Versioning:** any incompatible inter-module change updates `docs/CONTRACTS.md`, fixtures/tests, callers, and the integration owner before merge.

## Engineering principles

1. Start with a walking skeleton: input → at least one actual operation → independent observable result.
2. Hide optional LLMs, VLMs and simulation behind narrow replaceable interfaces; they may fail or be offline.
3. Keep hard realtime/safety control in a deterministic controller rather than an unbounded language-model loop.
4. Make state transitions and errors machine-inspectable (structured logs, IDs, timestamps and seeds), not just natural-language narration.
5. Test the seams between modules before polishing internal implementations.
