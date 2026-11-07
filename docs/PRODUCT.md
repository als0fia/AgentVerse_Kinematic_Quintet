# Product mission and acceptance criteria

**Selected and scope-frozen for AgentVerse 2026 (7 November): RiceWatch — autonomous rice-field flood-warning inspection and diagnosis.** We are building **one simulated mobile scout** that verifies a potentially misleading water-level alert, navigates under sensor feedback, handles an unexpected route obstruction and reports a defensible result to a field operator. **Opening a gate, pumping, draining, manipulating a valve, reducing floodwater, physically rescuing another robot and cross-domain transfer are NOT part of the required product.** No working rover/simulator or measured performance is currently verified.

## The exact real-world job

**Beneficiary:** rice farmers and irrigation/drainage operators who receive an unusual water-rise alert but cannot tell remotely whether it is a real drainage concern, a normal controlled ponding condition or a faulty gauge. They currently have gauges, routine inspections and human response; our experiment tests whether *selective mobile verification* can produce better evidence while avoiding unnecessary trips or unsafe robot movement.

**One-sentence promise:** when a rice-field water sensor reports an abnormal reading, an autonomous scout chooses where to inspect, physically gathers independent local evidence, adapts if access changes and returns **verified anomaly / likely false alarm / unresolved—needs human inspection** with uncertainty and sensor provenance.

Ordinary standing water in managed paddies is normal; a shallow/high reading is not inherently crop damage. The simulation must define a **case-specific synthetic safe range and what constitutes an abnormal rise**, not purport to determine real crop survival. Context: [IRRI drainage and flash-flood discussion](https://books.irri.org/TechnicalBulletin16_content.pdf); [rice submergence physiology review](https://academic.oup.com/plphys/article/197/2/kiaf029/7989433).

## Required P0 — what we will actually demonstrate

**One rover, one field, one concrete mission, two useful incident outcomes.** A fixed field gauge raises an ambiguous warning near an outlet. The rover travels on a physically modelled service bank/bund, not through submerged crop rows, using actual simulated range/odometry and a **location-dependent local water/depth measurement** available only when it reaches an observation point. It chooses a useful observation point, obtains the second independent reading, and classifies the incident using only permitted measurements.

**Mid-episode surprise:** once the rover has started the inspection, the simulator unexpectedly obstructs the planned route or otherwise invalidates its next motion. The rover must observe the change with its own simulated sensor, **replan to a safe reachable vantage** or, if none is safe, stop and return **unresolved—human needed**. The world injects the surprise; the policy must not receive an oracle event label. We show physical motion, changed sensor values, changed decisions and the final evidence report in one continuous run.

**Minimum required outcomes:** (A) a true abnormal water rise is independently checked and reported with the evidence source; (B) a faulty fixed reading/normal-water case is *not* confidently misreported as damaging flooding; (C) a changed-world route is handled by replanning or justified safe abstention. The comparison is against a stationary threshold alert and a competent conventional scout/route planner, on the **same seeds and observation budgets**. An LLM isn't required for the controller and must show added value if included.

**Build gates (P0 is not complete at gate 1 alone):**

| Stage | Required behaviour / acceptance |
| --- | --- |
| **P0a — Integration skeleton first** | One actual physically moving simulated rover, true sampled range sensor, bounded local steering/stopping, seeded reset and unexpected obstacle seen through sensor; typed observation/action seam and read-only UI. This is architecture-ready, **not yet a completed rice product**. |
| **P0b — Rice-specific investigation** | Simulated station reading can be wrong; rover must physically collect a **distinct local measurement at a reached observation location**, update its belief/assessment and report `verified abnormal`, `likely false alarm` or `unresolved`. Not a scripted map overlay or revealed ground truth. |
| **P0c — Demonstrable autonomy and evidence** | A single continuous sensor/action/surprise run; reproducible nominal/false-alert/route-blocked cases; same-seed credible baseline, independent grader, local safe stop and exact tested command/video. |

**Readiness:** P0a/P0b/P0c have **not** been executed. The selected application is a direction and implementation contract, not evidence of functioning software.

## Explicitly optional, time-permitting extensions — not acceptance criteria

**P1 — Targeted flood mitigation.** *Only if P0 works, tests are preserved and the judge video is secured*: use a physically actuated simple gate/lever or independently controlled field outlet and a clearly **synthetic water-flow proxy**. A separately validated capability must exist before the mission supervisor may send `REQUEST_MITIGATION`. Verify that the joint changes and a subsequent simulated reading changes; support jam/refusal. **If none of this is implemented, the completed P0 remains a valid inspection-and-diagnosis product.** An advisory recommendation is not physical mitigation.

**P2 — Fleet recovery and other applications.** Optional second rover to help a stranded scout, disconnected edge operation, resource-aware dispatch, or adapters for reforestation, Firewatch and space inspection. These demonstrate architectural potential only after separate physical tests; no multi-robot rescue, towing or cross-domain generalisation is promised for the hackathon.

**Never move an optional feature into P0 by accident.** No gate physics, manipulators, drainage control, flood suppression, multi-robot coordination, trained VLA/RL, accurate hydrodynamics, carbon/yield effect, real robot hardware or cloud infrastructure is required to declare P0 done.

## How the software and intelligence should be separated

- **Monitoring and investigator:** compare a suspicious fixed-gauge reading with uncertainty, select a *physical place to gather additional evidence*, integrate the new reading, triage/abstain and explain the assessment; do not invent hidden water states.
- **Local controller and safety:** execute bounded waypoint/velocity commands with real simulator sensors, speed/turn/collision/edge limits and local stop/replan; cannot be replaced with an LLM prompt per wheel tick.
- **Simulation + sensor adapter:** one actuated rover, collision geometry, two independent sensor origins, optional **simplified water-level ground truth** hidden from policy, seeded mid-run obstruction.
- **Independent evaluator/UI:** hidden-world truth for grading *only*, fixed-seed case outcomes and a **read-only** sensor-and-action visualisation. UI must not own physics or secretly alter results.
- **General-purpose interfaces:** incident observation → decide investigate/wait/escalate → bounded physical skill → observe result. Keep future actors possible without building them now.

See [ARCHITECTURE](ARCHITECTURE.md) for module ownership, [CONTRACTS](CONTRACTS.md) for proposed `rice.v0` seam, [RUN](RUN.md) for first coding task and [TEAM](TEAM.md) for five human owners.

## Human-led embodiment decision

**Provisional reference:** a simple differential/four-wheel rover travelling on a sufficiently wide firm paddy service-bank edge with range/odometry and position-dependent local water observation. A human teammate approves chassis, contact and safe operating envelope before detailed modelling. A tracked/amphibious rover or gate manipulator is **not** a P0 requirement. No real waterproofing, traction, agricultural usefulness, force rating or hardware deployment has been verified. Consider [paddy robot chassis research](https://pmc.ncbi.nlm.nih.gov/articles/PMC12901939/) and [floating paddy robot prototype](https://www.fujipress.jp/jrm/rb/robot003800020404/?lang=ja) as comparisons, not implemented designs.

## Baselines, metrics and claims

**Comparison on matched seeds:** stationary gauge threshold (no physical inspection), fixed/nearest-waypoint scout with a capable obstacle-reactive controller, and proposed active-sensing/uncertainty-aware inspector. If a slow model is used, ablate it against a strong non-LLM belief-based approach. Measure **correct vs false alarms**, unresolved incidents, time to independent observation, useful information gathered per movement/energy proxy, safe response after blockage, collisions/crop/no-go incursions, autonomy/control latency and episode completion. The agent never grades itself.

The strongest novel claim we can *test* is that **physical active verification combined with robust mission adaptation** outperforms a passive alarm or conventional scouting for this stylised case—not that the software has saved rice yields or solved real flood mitigation.

**Delivery:** one-page technical PDF, one ≤60-second **single continuous** simulated sensor/action video featuring a genuine mid-run surprise, and five-minute pitch **only if shortlisted**. Source/rubric/deadline are in [EVENT](EVENT.md). Final submission requires human permission.

## Embodied Robotics implementation gate — revealed 7 November 2026

**Challenge constraints are now known:** see [EVENT](EVENT.md) (organiser slides 35–38, full 100% rubric now confirmed from a clearer attendee photo), [independent evaluation](TESTING.md), and [fast/slow model / relevant prior-art research](RESEARCH.md#robotics-decision-model-research-fast-execution-slow-investigation-7-nov-2026). The rice-field mission above is the selected direction; the exact robot stack and testable scope must still be implemented, and it must be judged by the same objective as a competent non-LLM alternative.

| Gate | Team must be able to show | Do not accept as proof |
| --- | --- | --- |
| **Perception-driven action** | An actual simulated robot consumes at least one simulated sensor and visibly selects an action based on the observation; new observations change later decisions | UI-only chat, privileged simulator coordinates disguised as sensor readings, pre-rendered animation |
| **Adaptation** | Within **one continuous <=60s episode**, an unexpected scene/sensor/goal change is detected and elicits bounded replanning, new measurement, or safe refusal; record the before/after trace | Operator switching prerecorded tracks or merely logging “obstacle detected” |
| **Operational autonomy** | Core step decisions and safety run with a local policy/controller; identify which reasoning/infrastructure can be remote and demonstrate bounded loss-of-network behavior | Remote hosted model on every servo tick, or asserting “offline” because the remote simulator streams video |
| **Relevance and impact — 20% of rubric** | A named human beneficiary with **specific supporting evidence** (firsthand need, actual operational failure or credible documented source), quantitative/observable benefit, and a **critical failure mode handled visibly in the demo** | Imaginary customer quotes, a generic “robots are useful” pitch, or just causing a failure without a safe response |
| **Ambition and technical quality — 15%** | Nontrivial technical barrier, original or meaningfully novel application, reliable engineering and measured trade-offs | Claiming a standard A*/RL/VLM/two-model loop is inherently innovative |
| **Submission proof** | One-page technical PDF, <=60s continuous simulation video with perception beside actions and a mid-run surprise; 5-minute pitch **if shortlisted** | Synthetic benchmark fixture, paper's reported score or a slide without actual robot action |

**Fast-versus-slow is not the idea:** A specialised/reactive controller, compact learned policy or bounded System One decision model can handle routine high-rate choices. A deliberative agent earns its role only when **active investigation, ambiguous diagnosis or mission-level adaptation** outperforms classical replanning, probabilistic inference or fixed experiment selection. [REA (2026)](https://arxiv.org/abs/2610.42053) already uses System 0/1/2 layers; [robojev bench](https://robo-jev.dev/) already compares typed decision models in robotic simulations. Do not sell either architecture as unprecedented.

**Minimum before committing to an implementation stack:** name one real person/organisation and mission with source-qualified evidence for why it matters, one feasible actuated robot scene + its real sensor channel, a mid-run disturbance not hardcoded into the controller, one strong classical baseline on the same seeds, independent non-LLM success/unsafe-action checks, run/recording feasibility on actual compute, and a known failure/abstention path. **The direction is selected; coordinate actual interfaces and team ownership before parallel implementation.** A remote Jev account, NVIDIA/Modal provision, correct physical sensor drivers or any model performance remain unverified.
