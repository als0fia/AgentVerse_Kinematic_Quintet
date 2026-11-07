# The Kinematic Quintet — AgentVerse 2026

**State:** Repository engineering foundation only. The organiser's onsite Robotics theme is now captured in [event constraints](docs/EVENT.md), but the exact scored problem statement, team solution, runtime stack, hardware, and working demo are **not yet confirmed, selected or implemented**. Do not describe this as a functioning submission.

**Event:** UCL AI Society AgentVerse, 7–8 November 2026, Europe/London. The [official timetable](https://agentverse.uclai.co.uk/) lists the track reveal at **Saturday 12:30**, final submissions **Sunday 14:00**, and judging **Sunday 15:00–16:00**. Verify changes in the actual participant portal; official challenge details supersede these notes.

## Start here

1. **Humans and coding agents:** follow [AGENTS.md](AGENTS.md), with [team coordination](docs/TEAM.md) when collaborating. Claude Code imports it through [CLAUDE.md](CLAUDE.md), Gemini CLI through [GEMINI.md](GEMINI.md), and Codex reads it natively.
2. **Challenge and resources:** Read the photographed onsite [Robotics theme and model announcements](docs/EVENT.md), obtain any exact written challenge/eligibility requirements, then choose the [product/mission](docs/PRODUCT.md) with the team. Neither examples nor advertised models imply a selected project or granted access.
3. **Before parallel coding:** agree the smallest shared interface needed for a runnable end-to-end path in [architecture](docs/ARCHITECTURE.md) and [contracts](docs/CONTRACTS.md). Divide **independent** modules by branch/owner (an issue, PR or short message suffices).
4. **Build:** implement a smallest complete user-visible journey, then expand with tests and real [evaluation](docs/TESTING.md).
5. **Demonstrate:** follow [demo and submission](docs/DEMO.md), verify all claims and keep adequate buffer before the real portal deadline.

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

## Multi-model coding agents

The six full workflow skills have one canonical source in `.agents/skills/` for Codex and Gemini CLI; `.claude/skills/` contains **byte-identical complete** Claude Code mirrors (not fragile shortcut wrappers). A deterministic parity check in CI prevents drift. Edit the canonical skill then run `python scripts/sync_skills.py --write`. Claude-native `.claude/agents/` subagents remain untouched. Human role/branch/handoff rules are shared in [TEAM](docs/TEAM.md).

Codex: inspect the root `AGENTS.md` and use skills from `.agents/skills/`. Claude Code: use `CLAUDE.md`, its native `.claude/skills/` and optional `.claude/agents/`. Gemini CLI: use `GEMINI.md`, confirm `/memory show`, `/skills list`, and trust the repository if prompted. For web-only ChatGPT/Gemini/Claude, native discovery is not guaranteed: explicitly read the relevant repo files. None of these conventions automatically launches concurrent agents.

## Working agreement

- Deliver the **measurable job** a real person needs, not a generic LLM chat demo. Explain why a deterministic algorithm, optimiser, controller or ordinary VLM cannot already do it as well.
- Confirm interfaces and ownership before parallel agent edits. Keep shared schema changes explicit and reviewed.
- Each claim needs a runnable proof, reference or an honest label such as *planned / simulated / not tested*.
- No secrets, private organiser messages, personal data, unauthorised licensed assets or large unchecked model weights in Git.
- **No final submission, deployment, spending, messaging or repository visibility changes without approval from the relevant human/team owner.**

This repository provides a process, not a promise of winning; the event's actual judging rubric is authoritative.
