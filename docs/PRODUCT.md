# Product mission and acceptance criteria

**Selected project direction — 7 November 2026:** Rice-field flood monitoring, investigation and adaptive response. The team's chosen application is **rice**, not Firewatch or precision tree/rice planting. The selected research architecture is reusable monitoring → active investigation → targeted intervention → outcome verification, with robot recovery as an **optional** extension. This is an authorised project direction, **not** evidence of a working robot, fixed simulator or final team-tested implementation.

## One-sentence mission

For rice growers and irrigation-field operators facing **unusually deep or prolonged water rise**, a sensor-driven mobile robot investigates the cause, selects a safe response, and checks whether the action helped while limiting battery use, crop damage and unnecessary deployments.

**Why it matters:** lowland rice normally grows in intentionally shallow standing water; not all water is an emergency. Uncontrolled flash flooding, deep/extended submergence and drainage failures can nevertheless harm crops. The farmer currently has water-level monitoring, irrigation/drainage management, periodic inspection and human-operated controls; our project must show an advantage over these or a capable reactive planner, not invent an absence of existing techniques. Sources: [IRRI on field water management](https://www.ricetoday.irri.org/water-management-for-food-security-how-does-irri-contribute/) and [2026 flood stress review](https://pmc.ncbi.nlm.nih.gov/articles/PMC13051865/).

**Not our product:** inspecting every rice planting position, precision rice transplantation, guaranteed yield improvement, or autonomous flood prevention under real farm conditions. Water depth alone does not prove flooding damage, and actual agronomic outcomes require field trials.

## Selected intelligence system

**Monitoring, Investigation & Targeted Mitigation** is the main system. It should support both stationary sensor warnings and mobile robot observations. It must distinguish *normal ponding* from a credible **abnormal rise**, decide whether another measurement is worth taking, deploy/route a capable robot, and select a bounded mitigation or safe abstention. The same domain-neutral functions could later be reused in reforestation (inspect → protect/seed), lunar/space construction (inspect → stabilise/repair), Firewatch (suspected hazard → verify/mitigate under human authority) or infrastructure inspection. **These are possible transfers, not working demos or independent products.**

**Swarm Recovery & Assistance** is a compatible *secondary capability*: when the dispatched rover itself becomes immobilised, the fleet can decide whether to self-recover, wait, send another robot, reallocate tasks or alert a human. It shares state estimation, allocation, safety and verification. **Do not make a rescue robot or multi-agent swarm a prerequisite for the first runnable rice demo.**

## Build priority: commit to one embodied result first

| Priority | What the robot demonstrably does | Success evidence / exit |
| --- | --- | --- |
| **P0 — Required working spine** | One simulated mobile rover reads at least one **simulated** environmental/robot sensor, navigates to inspect an abnormal water or drainage alert, and responds to an unexpected **mid-episode** obstruction, new observation or changed route. It must physically move and revisit its decision from new observations. | Timestamped sensor → decision → physical actuation → later observation, real collision/obstacle handling, safe stop, identical-seed baseline |
| **P1 — Preferred mitigation, only after P0 runs** | After diagnosing a plausible blocked/closed drainage outlet or abnormal inflow, the rover reaches an explicitly modelled safe control point and **physically actuates a small gate/lever or other feasible simulated mechanism**; a separate simplified water-balance model changes flow/depth, and the robot measures whether it helped. If that physical manipulation is too costly to implement, demonstrate **inspection and justified mitigation recommendation**, labelling fixed infrastructure actuation separately rather than claiming robot manipulation. | Verified gate/flow change not a scripted UI label; before/after water-level proxy, force/collision constraints, jam/no-effect detection and bounded escalation |
| **P2 — Optional transferable framework** | Add a second rover, dispatch/relay, swarm rescue for a stranded rover, and/or one alternative environment/configuration using the **same** planner/skill interfaces. | One independent working second scenario, never a re-skinned animation; only after core, baseline and video are secure |
| **Out of scope for the initial submission** | Accurate hydrodynamics, crop survival modelling, trained VLA/RL policy from scratch, full robot towing/swarm consensus, real farm intervention, cloud GPU dependency | These are future work, not demo claims |

**Concrete preferred episode:** a central field gauge reports **abnormal water rise**; the mobile rover investigates a drainage point via local depth/camera/range observations. An unexpected obstacle or changed water condition invalidates its planned route/assumption. It must avoid the hazard and replan, then either attempt **one safe modelled mitigation** (e.g. open a small bypass gate if accessible) and verify a lower water proxy, or explicitly refuse/escalate if the drain is jammed or the route is unsafe. The actual disturbance must be injected by the environment, not encoded as a canned controller sequence. Use an independent simulator evaluator, not the model's own narrative, for success.

## Where an agent could add value—and where not

- **System 0:** local deterministic navigation/contact bounds, battery reserve, obstacle stop, no-go crop zones and actuator limits. Never delegate safety-critical control to an unconstrained LLM.
- **System 1:** fast perception, incident classification, ordinary task reallocation, route planning and bounded skill choice. Conventional state machines, Bayesian belief filters and constrained planners are serious starting points.
- **System 2 (optional):** a slower investigator proposes useful **physical probes** or cause hypotheses when water-level changes have several plausible causes (blocked drain vs increased inflow vs faulty reading), or when a mitigation fails. Its suggested actions must pass local constraints and be measured against a strong non-LLM baseline.
- **Generalisation claim to test:** the architecture separates the *incident → belief → investigation → constrained physical skill → outcome* logic from a rice-specific sensor and actuator adapter. Reforestation or space scenarios are proposed adapters; common code alone does not demonstrate real transfer.

## Primary success measures and baselines

**Compare the same unseen seeds and observation/action budget:** (A) normal fixed sensor alarm + preset route/human-style response; (B) competent reactive or Bayesian investigation/replanner with safe actuator rule; (C) proposed uncertainty-aware investigation/mitigation policy, with an optional slow-model ablation. Measure mission success, time from abnormal event to verified cause/response, task interruption, energy/return reserve, unsafe contacts/crop-proxy disturbance, unnecessary intervention, and mitigation efficacy in the **simplified simulated water model**. Include no-alarm normal ponding, sensor-fault, obstruction and jammed-gate cases. Do not imply the LLM wins before running tests.

**Critical failure to handle on-screen:** after a real anomalous rise, the rover's expected inspection/mitigation path becomes unsafe or ineffective. It must detect the change through sensors and replan or stop/escalate safely, rather than silently opening a gate that cannot help.

## Ownership and acceptance

- **Direction:** rice-field flood investigation/response is chosen in this conversation (7 Nov). Named teammate integration owner, simulator, actuators, model, risk bounds and test claims remain to be confirmed by the actual team.
- **Required evidence:** physically simulated sensor-driven actuation and mid-episode adaptation; a source-supported end-user need; independent verifier; a serious baseline; one continuous ≤60 s perception/action video; one-page technical PDF; five-minute pitch only if shortlisted. See [EVENT](EVENT.md), [ARCHITECTURE](ARCHITECTURE.md), [CONTRACTS](CONTRACTS.md), [TESTING](TESTING.md), [DEMO](DEMO.md), [RUN](RUN.md).
- **Scope cutting rule:** never sacrifice the P0 sensor → physical action → surprise → recovery demonstration to add multi-robot rescue, cloud models, dashboards, fictional physics or unverified future applications.

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
