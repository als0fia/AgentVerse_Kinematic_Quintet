# Product mission and acceptance criteria

**Selected working project direction (7 November 2026, user instruction):** **Rice-field flood monitoring, active investigation and targeted mitigation**, built on a reusable **sense → diagnose → investigate → coordinate actor → verify/recover** architecture. This supersedes the earlier unselected Firewatch/RegenScout shortlist **for this submission**. Human integrator must confirm the precise demonstration scope and robot design with teammates. **No working robot, simulation, physical mitigation, model performance or hardware has been verified yet.**

## Job to be done

**Beneficiary:** rice growers and irrigation/drainage operators managing excess water, blocked outlets or failed irrigation infrastructure. Normal standing water in managed rice paddies is **not** automatically harmful; the problem is an *abnormal, damaging* rise or drainage failure relative to a crop- and site-specific management target. A practical system would reduce wasted inspections, response delay and unnecessary intervention while avoiding crop damage and unsafe dispatch.

**Product one-liner:** A robot-supported paddy monitoring station detects a suspicious water-management failure, dispatches a scout to acquire useful physical evidence, selects a safe targeted response, and verifies its effect—even when a sensor, route, connection or robot fails.

**We are not building:** a precision rice planter, an accurate crop-growth/flood simulator, a real autonomous flood-control product, a generalised wildfire system, or an all-purpose robot rescuer. Previous concepts remain research/future application examples, not concurrent deliverables. [EVENT](EVENT.md) owns official requirements.

## First embodied demonstration: inspect a suspected blocked drain

A small **simulated wheeled service rover** moves **along a service bank/track**, not through submerged rice plants. A simple irrigation channel has two monitored regions and an outlet/gate. A fixed station reports an ambiguous rising-water cue; the rover chooses an informative inspection point, travels under local control, and reads its own simulated range/odometry and position-dependent water-level/proximity measurement. A **mid-run surprise**—a newly blocked route or outlet, a false sensor cue, or a sudden rise—invalidates its prior plan. The robot must perceive the change, replan, continue within limits or safely stop. An independent evaluator knows the hidden cause but the agent does not.

**Minimum non-negotiable first pass:** one physics-actuated rover, one real simulated sensor that changes the chosen motion, an actual changed-world event injected mid-episode, visible physical movement and observed recovery/abstention. Do **not** stall this for a functioning fluid model, second rover, robot arm, cloud inference or impressive UI.

**Desired next increment — targeted mitigation (not yet proven):** after diagnosing a genuinely blocked outlet, a *separately modelled and physically actuated* canal gate/valve or a simple robot-operated gate interface can alter a **clearly labelled synthetic water-level/flow proxy**. The coordinator may choose inspect / wait / request gate actuation / escalate. The command is accepted only when actor capabilities and safety conditions permit it; opening a gate is not automatically the right agronomic choice. Show the visible gate motion, measured downstream proxy and possible failed intervention. This is an extra acceptance gate, not permission to say we prevent real crop losses.

**Stretch, conditional on a working baseline:** a second scout or helper for a stuck rover; task reallocation, local communication-loss fallback, and a small constrained high-level investigator. Do not let peer rescue displace the rice-flood loop or claim physical towing until contacts and force transfer actually work.

## Why the intelligence could be useful

- **Unknown causes and selective sensing:** water rise may be real, sensor drift, rain, obstruction or gate error. Choose *where and what to measure next* rather than dispatching blindly.
- **Consequence-aware interventions:** compare doing nothing, scouting, gate action and escalation given confidence, environmental constraints, remaining battery and mission disruption.
- **Failure handling:** distinguish failed route from bad sensing and failed mitigation; retry boundedly, reallocate only when help adds value, and prefer safe abstention over repeated damage.
- **Baseline to beat:** (A) fixed threshold / no scout, (B) nearest-inspection/shortest-path reactive rule, (C) competent Bayesian or rule-based active-information-gain planner with the same observations and resource budget. A reasoning model is **optional and must earn its role** against C; it never controls motor torques on every step.
- **Broader framework:** the same *interfaces* could support reforestation (uncertain site assessment and targeted protection/planting), Firewatch (uncertain fire alerts → safe scouting) or space/industrial inspection (fault → investigation → response). These are **transfer hypotheses**; no claim we built or validated those domains.

## Proposed acceptance and scoring

| Requirement | Independent demonstration / measure |
| --- | --- |
| Perception-driven action (25%) | timestamped simulator sensor observation causes different rover velocity/waypoint than when no hazard is sensed; robot actually moves |
| Adaptation (25%) | inject mid-episode obstacle or anomalous water reading unknown to the policy; record detect → revised action or safe stop in one continuous run |
| Relevance/impact (20%) | credible irrigation drainage problem, a visible correct response to a dangerous scenario, and honest comparator; cite agronomic evidence, not speculative farmer testimony |
| Autonomy (15%) | local bounded policy and safe-stop when remote planner/link unavailable; disclose where simulator and model run |
| Technical quality (15%) | traceable state/action design, reproducible seed, independent hidden-state evaluator, non-trivial counterfactual/action selection and actual measured trade-offs |
| Mitigation stretch | gate/valve joint physically moves and simplified water-flow proxy responds; intervention can fail; no remote magic scene reset |
| Optional recovery stretch | second actor reaches disabled rover and demonstrates *physically valid* assistance or justified non-rescue; don't count mere task reassignment as towing |

**Suggested measurements:** correct hazard confirmation vs false alarm; inspection delay; needless dispatches; response/mitigation success in the synthetic world; rover collisions/crop/no-go incursions; battery/energy proxy and safe reserve; number of failed retries; decision latency; local fallback; background work preserved. Report denominators/seeds and failures. No real-world rice yield, flood avoidance, hydrological accuracy or cross-domain transfer claims.

## Human robot-design decision (needed early, not to block contracts)

**Provisional geometry, not a final mechanical design:** simple differential-drive wheeled inspection rover on an accessible raised embankment, with range/odometry and a local water-level probe. Reuse a permitted MuJoCo model or minimal credible MJCF where possible; realistic muddy traction, flotation, paddies and a manipulator are **not part of the first pass**. A human robotics owner decides physical shape, wheelbase, sensors, actuator feasibility, mass, clearance and any gate interaction after simulator preflight. Agents may compare designs and test feasibility, but **do not silently decide hardware/actuators or generate a complex custom rover** without that human review.

**Scope freeze:** core loop first; then one visible mitigation actor only if the core works; then recovery/second rover if time. The coordinator must expose an interface for future extra actors rather than implementing them now.

## Project readiness checklist

- [x] User chose rice-based monitoring/mitigation as working project.
- [x] Narrow first scene and comparator defined in this document.
- [ ] Human integrator confirms exact physical robot/gate design and who owns integration.
- [ ] A team workstation actually runs a physics rover with a sensor and logs a closed-loop action.
- [ ] Contract schema/fixtures implemented, independently checked and accepted by all component owners.
- [ ] Mid-run surprise reproduced without leaking ground truth to the policy.
- [ ] Real same-seed baseline and candidate results measured.
- [ ] Mitigation actuator or rescue added only after demonstrably working and appropriately described.

## Change control

This is the **single canonical current product definition**. [ARCHITECTURE](ARCHITECTURE.md) owns runtime/module seams, [CONTRACTS](CONTRACTS.md) owns interface semantics, [TEAM](TEAM.md) owns assignments, [RUN](RUN.md) owns tested commands, [TESTING](TESTING.md) owns evidence and [DECISIONS](DECISIONS.md) preserves why the earlier shortlist changed. No new contradictory product-spec file is needed.

## Embodied Robotics judging gate — revealed 7 November 2026

**Challenge constraints are now known:** see [EVENT](EVENT.md) (organiser slides 35–38, full 100% rubric now confirmed from a clearer attendee photo), [independent evaluation](TESTING.md), and [fast/slow model / relevant prior-art research](RESEARCH.md#robotics-decision-model-research-fast-execution-slow-investigation-7-nov-2026). The track requirements preceded the selected rice project; use them as a continuing acceptance gate. All candidate ideas must be judged by the same objective, not by whether they use an LLM or a fashionable framework.

| Gate | Team must be able to show | Do not accept as proof |
| --- | --- | --- |
| **Perception-driven action** | An actual simulated robot consumes at least one simulated sensor and visibly selects an action based on the observation; new observations change later decisions | UI-only chat, privileged simulator coordinates disguised as sensor readings, pre-rendered animation |
| **Adaptation** | Within **one continuous <=60s episode**, an unexpected scene/sensor/goal change is detected and elicits bounded replanning, new measurement, or safe refusal; record the before/after trace | Operator switching prerecorded tracks or merely logging “obstacle detected” |
| **Operational autonomy** | Core step decisions and safety run with a local policy/controller; identify which reasoning/infrastructure can be remote and demonstrate bounded loss-of-network behavior | Remote hosted model on every servo tick, or asserting “offline” because the remote simulator streams video |
| **Relevance and impact — 20% of rubric** | A named human beneficiary with **specific supporting evidence** (firsthand need, actual operational failure or credible documented source), quantitative/observable benefit, and a **critical failure mode handled visibly in the demo** | Imaginary customer quotes, a generic “robots are useful” pitch, or just causing a failure without a safe response |
| **Ambition and technical quality — 15%** | Nontrivial technical barrier, original or meaningfully novel application, reliable engineering and measured trade-offs | Claiming a standard A*/RL/VLM/two-model loop is inherently innovative |
| **Submission proof** | One-page technical PDF, <=60s continuous simulation video with perception beside actions and a mid-run surprise; 5-minute pitch **if shortlisted** | Synthetic benchmark fixture, paper's reported score or a slide without actual robot action |

**Fast-versus-slow is not the idea:** A specialised/reactive controller, compact learned policy or bounded System One decision model can handle routine high-rate choices. A deliberative agent earns its role only when **active investigation, ambiguous diagnosis or mission-level adaptation** outperforms classical replanning, probabilistic inference or fixed experiment selection. [REA (2026)](https://arxiv.org/abs/2610.42053) already uses System 0/1/2 layers; [robojev bench](https://robo-jev.dev/) already compares typed decision models in robotic simulations. Do not sell either architecture as unprecedented.

**Minimum before extending implementation:** name one real person/organisation and mission with source-qualified evidence for why it matters, one feasible actuated robot scene + its real sensor channel, a mid-run disturbance not hardcoded into the controller, one strong classical baseline on the same seeds, independent non-LLM success/unsafe-action checks, run/recording feasibility on actual compute, and a known failure/abstention path. **Human integrator confirms exact demonstration scope and hardware design.** A remote Jev account, NVIDIA/Modal provision, correct physical sensor drivers or any model performance remain unverified.
