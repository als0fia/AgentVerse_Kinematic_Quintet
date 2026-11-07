# The Kinematic Quintet — AgentVerse 2026

**State:** Repository engineering foundation only. The Robotics challenge, team solution, runtime stack, hardware, and working demo are **not yet selected or implemented**. Do not describe this as a functioning submission.

**Event:** UCL AI Society AgentVerse, 7–8 November 2026, Europe/London. The [official timetable](https://agentverse.uclai.co.uk/) lists the track reveal at **Saturday 12:30**, final submissions **Sunday 14:00**, and judging **Sunday 15:00–16:00**. Verify changes in the actual participant portal; official challenge details supersede these notes.

## Start here

1. **Humans and coding agents:** read [AGENTS.md](AGENTS.md) and [team coordination](docs/TEAM.md).
2. **Challenge revealed?** Update [event constraints](docs/EVENT.md) and choose the [product/mission](docs/PRODUCT.md) with the team. No proposal becomes the chosen project by implication.
3. **Before parallel coding:** agree [architecture](docs/ARCHITECTURE.md) and versioned [interfaces](docs/CONTRACTS.md); assign file ownership in an issue/PR.
4. **Build:** implement a smallest complete user-visible journey, then expand with tests and real [evaluation](docs/TESTING.md).
5. **Demonstrate:** follow [demo and submission](docs/DEMO.md), verify all claims and keep adequate buffer before the real portal deadline.

## Foundation commands

Requires only Python 3.10+ for the included tooling (not a commitment to Python for the robotics application):

```bash
python scripts/check_repo.py
python -m unittest discover -s tests -v
python scripts/evaluate.py --candidate tests/fixtures/candidate.jsonl --baseline tests/fixtures/baseline.jsonl
```

The fixture comparison is **synthetic tooling data, not product performance**. Once a real stack is chosen, write precise installation, execution and smoke-test commands in [RUN.md](docs/RUN.md).

## Documentation map

| Single source of truth | Contents |
| --- | --- |
| [EVENT](docs/EVENT.md) | Revealed challenge, official rules, resources and deadlines with source/time |
| [PRODUCT](docs/PRODUCT.md) | Real beneficiary, workflow, success criterion, alternative and selection rationale |
| [ARCHITECTURE](docs/ARCHITECTURE.md) | Modules, data/control flow, ownership and dependencies |
| [CONTRACTS](docs/CONTRACTS.md) | Versioned inter-module messages, units, errors and invariants |
| [TEAM](docs/TEAM.md) | Ownership protocol, parallel tasks, integration and handoff |
| [RUN](docs/RUN.md) | Reproducible setup, start, smoke and recovery commands |
| [TESTING](docs/TESTING.md) | Test matrix, baseline metrics, evaluation cases and failure taxonomy |
| [DEMO](docs/DEMO.md) | Live run, fallback, judge evidence and submission gate |
| [PROVENANCE](docs/PROVENANCE.md) | Attribution, pre-existing work, AI assistance, licensing and claims |
| [DECISIONS](docs/DECISIONS.md) | Architecture/engineering decisions and trade-offs |
| [RESEARCH](docs/RESEARCH.md) | Source-checked engineering/workflow and relevant prior-art references |

GitHub Issues track tasks, PRs track reviewed implementation, source/tests specify actual behaviour. Avoid shadow task lists or dated session logs. The private hackathon management workflow is separate from this public team code repository.

## Working agreement

- Deliver the **measurable job** a real person needs, not a generic LLM chat demo. Explain why a deterministic algorithm, optimiser, controller or ordinary VLM cannot already do it as well.
- Confirm interfaces and ownership before parallel agent edits. Keep shared schema changes explicit and reviewed.
- Each claim needs a runnable proof, reference or an honest label such as *planned / simulated / not tested*.
- No secrets, private organiser messages, personal data, unauthorised licensed assets or large unchecked model weights in Git.
- **No final submission, deployment, spending, messaging or repository visibility changes without approval from the relevant human/team owner.**

This repository provides a process, not a promise of winning; the event's actual judging rubric is authoritative.
