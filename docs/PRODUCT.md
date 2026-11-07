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
