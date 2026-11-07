# Product mission and acceptance criteria

**Selection status: NOT DECIDED.** This template is not a selected hackathon entry. Event brief and team decision take precedence over prior idea research.

## The one-sentence job

> For [specific human / organisation] who needs [concrete action] in [actual environment], the system [performs observable work], verified by [independent result].

**Beneficiary and firsthand evidence:** undecided. Record interviews or external references, not imagined pain points.

**Current alternative:** identify the real workflow, numerical optimiser, controller, VLM, off-the-shelf service or human process. State exactly how it performs and what it does not handle.

**Why AI is necessary:** specify the ambiguous perception, uncertainty, diagnosis, experiment design or language reasoning that deterministic tools cannot cheaply handle. A tool-calling LLM is not itself differentiation.

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
- [ ] One real beneficiary and credible usefulness argument
- [ ] Working small end-to-end demo designed, not just a component list
- [ ] One serious baseline and at least one near prior-art source
- [ ] Quantitative or observable success and failure criteria
- [ ] AI contribution has a testable hypothesis
- [ ] Named integrator and compatible stack available
- [ ] Team explicitly selects the scope; alternatives preserved in issue/decision history

## Change control

Use `docs/DECISIONS.md` for durable selection/architecture decisions, this file for *current* product intent, and GitHub issues for implementation work. If the demo is cut, update scope here before telling judges it is still supported.


## Embodied Robotics selection gate — revealed 7 November 2026

**Challenge constraints are now known:** see [EVENT](EVENT.md) (onsite slides 35–38, partly illegible rubric), [independent evaluation](TESTING.md), and [fast/slow model / relevant prior-art research](RESEARCH.md#robotics-decision-model-research-fast-execution-slow-investigation-7-nov-2026). This is **not** a product choice. All candidate ideas must be judged by the same objective, not by whether they use an LLM or a fashionable framework.

| Gate | Team must be able to show | Do not accept as proof |
| --- | --- | --- |
| **Perception-driven action** | An actual simulated robot consumes at least one simulated sensor and visibly selects an action based on the observation; new observations change later decisions | UI-only chat, privileged simulator coordinates disguised as sensor readings, pre-rendered animation |
| **Adaptation** | Within **one continuous <=60s episode**, an unexpected scene/sensor/goal change is detected and elicits bounded replanning, new measurement, or safe refusal; record the before/after trace | Operator switching prerecorded tracks or merely logging “obstacle detected” |
| **Operational autonomy** | Core step decisions and safety run with a local policy/controller; identify which reasoning/infrastructure can be remote and demonstrate bounded loss-of-network behavior | Remote hosted model on every servo tick, or asserting “offline” because the remote simulator streams video |
| **Originality, quality and necessity** | Concrete beneficiary, useful objective, repeatable run, a real-world robotics barrier, strongest practical controller/planner competitor and independent success metric | Simply inserting “agent” into A*, RL, VLM, grasping or tutorial pipeline |
| **Submission proof** | One-page technical PDF, <=60s continuous simulation video with perception beside actions and a mid-run surprise; 5-minute pitch **if shortlisted** | Synthetic benchmark fixture, paper's reported score or a slide without actual robot action |

**Fast-versus-slow is not the idea:** A specialised/reactive controller, compact learned policy or bounded System One decision model can handle routine high-rate choices. A deliberative agent earns its role only when **active investigation, ambiguous diagnosis or mission-level adaptation** outperforms classical replanning, probabilistic inference or fixed experiment selection. [REA (2026)](https://arxiv.org/abs/2610.42053) already uses System 0/1/2 layers; [robojev bench](https://robo-jev.dev/) already compares typed decision models in robotic simulations. Do not sell either architecture as unprecedented.

**Minimum before selecting implementation:** name one real person/organisation and mission, one feasible actuated robot scene + its real sensor channel, a mid-run disturbance not hardcoded into the controller, one strong classical baseline on the same seeds, independent non-LLM success/unsafe-action checks, run/recording feasibility on actual compute, and a known failure/abstention path. **Select only with the team.** A remote Jev account, NVIDIA/Modal provision, correct physical sensor drivers or any model performance remain unverified.
