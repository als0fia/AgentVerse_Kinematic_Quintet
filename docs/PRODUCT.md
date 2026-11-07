# Product mission and acceptance criteria

**Selection status: NOT DECIDED.** This template is not a selected hackathon entry. Event brief and team decision take precedence over prior idea research.

## The one-sentence job

> For [specific human / organisation] who needs [concrete action] in [actual environment], the system [performs observable work], verified by [independent result].

**Beneficiary and firsthand evidence:** undecided. Record interviews or external references, not imagined pain points.

**Current alternative:** identify the real workflow, numerical optimiser, controller, VLM, off-the-shelf service or human process. State exactly how it performs and what it does not handle.

**Why AI is necessary:** specify the ambiguous perception, uncertainty, diagnosis, experiment design or language reasoning that deterministic tools cannot cheaply handle. A tool-calling LLM is not itself differentiation.

## Candidate system families (not selected)

**These are two reusable systems, not five separate products.** The team has not yet chosen either or verified a working simulation. Applications are examples to help select **one testable embodied task**, not promises of simultaneous implementation.

| System | Specific job / user | Minimum embodied demonstration | Difficult comparison |
| --- | --- | --- | --- |
| **Swarm Recovery & Assistance** | For field fleet operators, restore operation when one robot's ordinary self-recovery fails | A simulated disabled rover is assessed; one mobile helper investigates/reaches it and physically performs a bounded recovery or justified safe refusal after an unexpected scene change | Nearest-helper and deterministic rescue rules versus resource/risk-aware cooperative diagnosis and dispatch |
| **Monitoring, Investigation & Targeted Mitigation** | For environmental/infrastructure monitors, turn uncertain alerts into safe, justified physical investigation or action | A fixed lookout detects an ambiguous event; a simulated scout chooses a useful observation point, physically travels, verifies sensor evidence and replans after a surprise | Fixed-sensor alarm, nearest-scout rule and Bayesian information-gain / risk-aware planner versus proposed agent |

**Application focus:** Firewatch is the clearest current example of System 2: verify a suspected wildfire without unsafe or unnecessary deployments, and keep the scout safe during communications loss. Rice **drainage-failure inspection** and reforestation **restoration triage** are alternative uses of monitoring/mitigation, **not extra systems**. Simple precision rice planting has no established advantage over competent mechanised planting. A simulated forest-survival score cannot substantiate real tree establishment. Swarm Rescue is System 1 and may be a future optional fallback for a stranded scout, **not a required integration this weekend**.

**First slice for either:** one physically actuated simulated robot, one genuine simulated sensor, one observation-conditioned decision, one surprise *during the same episode*, a bounded local controller and independent grader. Add a second robot, environmental actuator or deliberative model only after this works. Capture the real user benefit and nearest published/current alternative in [RESEARCH](RESEARCH.md) when the team chooses. See [CONTRACTS](CONTRACTS.md) for interface ownership and [TESTING](TESTING.md) for falsification; do not use privileged simulator ground truth as agent input. The actual **selection, baseline, simulator, ownership and acceptance metric** must replace this shortlist in this file after human agreement.

## Minimum live scenario

| Field | Must be explicit after selection |
| --- | --- |
| Starting state | Scene/logs/data/environment, assumptions, allowed resets |
| User input | Real request, measured signal or physical situation |
| Agent judgment | The exact model choice, hypothesis or action selection |
| Deterministic subsystem | Controller, simulator, search/optimiser, safety gate, verifier |
| Observable effect | Physical/simulated action, repaired state, retrieved object or training improvement |
| Independent verifier | Non-LLM success condition and instrumentation |
| One difficult edge case | What change causes re-planning or a clear safe failure |
| Realistic failure | What can break and what the operator sees |

## Acceptance checks before product implementation

- [ ] Official track prompt and eligibility read
- [ ] **One real beneficiary, evidence/source for the need, meaningful outcome and most critical failure handled on screen (20% judge criterion)**
- [ ] Working small end-to-end demo designed, not just a component list
- [ ] One serious baseline and at least one near prior-art source
- [ ] Quantitative or observable success and failure criteria
- [ ] AI contribution has a testable hypothesis
- [ ] Named integrator and compatible stack available
- [ ] Team explicitly selects the scope; alternatives preserved in issue/decision history

## Change control

Use `docs/DECISIONS.md` for durable selection/architecture decisions, this file for *current* product intent, and GitHub issues for implementation work. If the demo is cut, update scope here before telling judges it is still supported.


## Embodied Robotics selection gate — revealed 7 November 2026

**Challenge constraints are now known:** see [EVENT](EVENT.md) (organiser slides 35–38, full 100% rubric now confirmed from a clearer attendee photo), [independent evaluation](TESTING.md), and [fast/slow model / relevant prior-art research](RESEARCH.md#robotics-decision-model-research-fast-execution-slow-investigation-7-nov-2026). This is **not** a product choice. All candidate ideas must be judged by the same objective, not by whether they use an LLM or a fashionable framework.

| Gate | Team must be able to show | Do not accept as proof |
| --- | --- | --- |
| **Perception-driven action** | An actual simulated robot consumes at least one simulated sensor and visibly selects an action based on the observation; new observations change later decisions | UI-only chat, privileged simulator coordinates disguised as sensor readings, pre-rendered animation |
| **Adaptation** | Within **one continuous <=60s episode**, an unexpected scene/sensor/goal change is detected and elicits bounded replanning, new measurement, or safe refusal; record the before/after trace | Operator switching prerecorded tracks or merely logging “obstacle detected” |
| **Operational autonomy** | Core step decisions and safety run with a local policy/controller; identify which reasoning/infrastructure can be remote and demonstrate bounded loss-of-network behavior | Remote hosted model on every servo tick, or asserting “offline” because the remote simulator streams video |
| **Relevance and impact — 20% of rubric** | A named human beneficiary with **specific supporting evidence** (firsthand need, actual operational failure or credible documented source), quantitative/observable benefit, and a **critical failure mode handled visibly in the demo** | Imaginary customer quotes, a generic “robots are useful” pitch, or just causing a failure without a safe response |
| **Ambition and technical quality — 15%** | Nontrivial technical barrier, original or meaningfully novel application, reliable engineering and measured trade-offs | Claiming a standard A*/RL/VLM/two-model loop is inherently innovative |
| **Submission proof** | One-page technical PDF, <=60s continuous simulation video with perception beside actions and a mid-run surprise; 5-minute pitch **if shortlisted** | Synthetic benchmark fixture, paper's reported score or a slide without actual robot action |

**Fast-versus-slow is not the idea:** A specialised/reactive controller, compact learned policy or bounded System One decision model can handle routine high-rate choices. A deliberative agent earns its role only when **active investigation, ambiguous diagnosis or mission-level adaptation** outperforms classical replanning, probabilistic inference or fixed experiment selection. [REA (2026)](https://arxiv.org/abs/2610.42053) already uses System 0/1/2 layers; [robojev bench](https://robo-jev.dev/) already compares typed decision models in robotic simulations. Do not sell either architecture as unprecedented.

**Minimum before selecting implementation:** name one real person/organisation and mission with source-qualified evidence for why it matters, one feasible actuated robot scene + its real sensor channel, a mid-run disturbance not hardcoded into the controller, one strong classical baseline on the same seeds, independent non-LLM success/unsafe-action checks, run/recording feasibility on actual compute, and a known failure/abstention path. **Select only with the team.** A remote Jev account, NVIDIA/Modal provision, correct physical sensor drivers or any model performance remain unverified.
