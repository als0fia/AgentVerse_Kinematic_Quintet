# The Kinematic Quintet — AgentVerse 2026

**State:** **Robotics challenge and deliverables now captured** in [event constraints](docs/EVENT.md): build an agent that perceives via simulated sensor(s), decides from observations, and acts in a changing simulated world. The team solution, simulator/runtime, hardware/credits and working demo are **not yet selected, validated or implemented**. Do not describe this as a functioning submission.

**Event:** UCL AI Society AgentVerse, 7–8 November 2026, Europe/London. The [official timetable](https://agentverse.uclai.co.uk/) lists the track reveal at **Saturday 12:30**, final submissions **Sunday 14:00**, and judging **Sunday 15:00–16:00**. Verify changes in the actual participant portal; official challenge details supersede these notes.

## Start here

1. **Humans and coding agents:** follow [AGENTS.md](AGENTS.md) and [team coordination/model routing](docs/TEAM.md). **Gemini CLI is the primary development route** through [GEMINI.md](GEMINI.md), with isolated optional project subagents. Codex remains optional; legacy Claude files are preserved without being the default.
2. **Challenge and resources:** Read [Robotics organiser slides 35–38 captured in EVENT](docs/EVENT.md). Before choosing the [product mission](docs/PRODUCT.md), verify access to a simulator and stream ([RUN](docs/RUN.md)); then build the smallest real **simulated sensor → decision → action → changed-scene recovery** episode ([TESTING](docs/TESTING.md)). Exact scoring text is partly obscured, and credits are not verified.
3. **Before parallel coding:** agree the smallest shared interface needed for a runnable end-to-end path in [architecture](docs/ARCHITECTURE.md) and [contracts](docs/CONTRACTS.md). Divide **independent** modules by branch/owner (an issue, PR or short message suffices).
4. **Build:** implement a smallest complete user-visible journey, then expand with tests and real [evaluation](docs/TESTING.md).
5. **Deliver:** follow [the actual Robotics submission brief](docs/DEMO.md): one-page technical PDF, **≤1-minute continuous** simulation video showing perception beside actions and one unexpected scene change, plus a **five-minute pitch if shortlisted**. Keep a buffer before the real portal deadline.

## Hard constraint from the revealed challenge

**The coding-agent workflow and the robot's autonomy are separate systems.** Gemini can help develop/test the entry, but judges want core simulated decisions to respond to real simulated sensors, re-plan when the scene changes, and avoid being dependent on cloud LLM calls at every step. NVIDIA Isaac Sim/Modal streaming is **optional** and separate from policy placement. The presenter explicitly allowed MuJoCo/PyBullet/Gazebo/Webots as alternatives. Simulator/stream readiness comes before fancy agent orchestration.

**Judging visibility:** perception-driven action, adaptation to an unscripted change, operational autonomy/local decision-making and originality/quality. The photographed weight bar is partly obscured; do not invent a fully weighted rubric.

## Foundation commands

Requires only Python 3.10+ for the included tooling (not a commitment to Python for the robotics application):

```bash
python scripts/check_repo.py
python scripts/sync_skills.py --check
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

Use GitHub Issues for substantial tasks, PRs for reviewed integration and source/tests as the proof. **No ticket is needed for a five-minute fix.** When multiple agents work at once, follow [the short GitHub and human-prompting guide](docs/TEAM.md#practical-parallel-work-without-ceremony), not shadow task lists or dated session logs. The private hackathon management workflow is separate from this public team code repository.

## Gemini-first agent development

**Avoid one massive context or one shared agent transcript.** Use Gemini 4 Argon, if accessible, for deep cross-component planning and synthesis; reserve 3.8 Flash for bounded research/review with only relevant files and limited tools. A read-only [independent reviewer](.gemini/agents/independent-reviewer.md) is configured for Gemini CLI but **has not been locally run or validated**. The [team protocol](docs/TEAM.md) defines escalation, context packets, branch ownership, issue/PR-based conversations and human integration. These are **development agent preferences**, not a mandate for the selected robotics product.

The six neutral workflow skills live in `.agents/skills/`. Gemini CLI consumes them on demand; `/memory show`, `/skills list` and `/agents list` should be checked on the actual machine. Existing Claude-native files and complete skill mirrors are **legacy compatibility retained for the moment**; the existing CI parity check still expects them. Do not delete them or rely on their maintenance for a new workflow. Hosted chats must read repo content explicitly; no instruction file automatically launches worker agents, remote A2A services or live coordination.

**Team communication:** GitHub issue comments for genuine cross-worker tasks and blockers, PRs for reviews, and one integrator coordinating merges. No additional message-server deployment or giant Markdown conversation folders. The repo is public; do not put private chats or credentials in its issue/PR threads. Actual model entitlement, API keys, limits, SDK invocation and grant credits remain to be verified.

## Working agreement

- Deliver the **measurable job** a real person needs, not a generic LLM chat demo. Explain why a deterministic algorithm, optimiser, controller or ordinary VLM cannot already do it as well.
- Confirm interfaces and ownership before parallel agent edits. Keep shared schema changes explicit and reviewed.
- Each claim needs a runnable proof, reference or an honest label such as *planned / simulated / not tested*.
- No secrets, private organiser messages, personal data, unauthorised licensed assets or large unchecked model weights in Git.
- **No final submission, deployment, spending, messaging or repository visibility changes without approval from the relevant human/team owner.**

This repository provides a process, not a promise of winning; the event's actual judging rubric is authoritative.
