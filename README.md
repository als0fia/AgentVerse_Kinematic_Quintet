# The Kinematic Quintet — AgentVerse 2026

**State (7 Nov 2026):** Project direction **selected** — **rice-field flood monitoring, active investigation and targeted mitigation**. A rover investigates abnormal water/drainage events and adapts when conditions change. The underlying monitoring/decision framework may later support forest restoration, space logistics and swarm rescue, but only the rice mission is selected for the initial demo. Simulator, code, model, local hardware and successful episode are **not yet verified**; this is not a completed submission.

**Event:** UCL AI Society AgentVerse, 7–8 November 2026, Europe/London. The [official timetable](https://agentverse.uclai.co.uk/) lists the track reveal at **Saturday 12:30**, old-site final submissions **Sunday 14:00**, but latest authenticated BuilderBase dashboard explicitly says **Sunday 8 Nov 13:00 GMT**; **plan against 13:00** until clarified, and judging **Sunday 15:00–16:00**. Verify changes in the actual participant portal; official challenge details supersede these notes.

## Repository purpose and canonical branch

**`main` is the complete current instruction and integration base.** This public repo owns the actual team's **agentic coding instructions and eventually the submission code**, not the private idea archive or organiser credentials. Start from [AGENTS](AGENTS.md) and a narrowly relevant [skill](.agents/skills/); use [TEAM](docs/TEAM.md) for safe ChatGPT/Gemini/Codex handoffs and [PRODUCT](docs/PRODUCT.md) for the **selected rice-field response mission** and the reusable monitoring/mitigation framework. Swarm Recovery is a potential extension, not a competing selected product. [EVENT](docs/EVENT.md) owns official criteria, [RUN](docs/RUN.md) owns simulator preflight, and [TESTING](docs/TESTING.md) / [DEMO](docs/DEMO.md) own real evidence and judge deliverables.

Short-lived feature branches and PRs exist only for independent, reviewable changes. Merge approved prerequisites into `main` and branch new development from its current HEAD; do not treat a stacked PR or developer-agent branch as a second source of truth. Preserve real teammate contributions and don't force-update or delete branches without approval.

## Start here

1. **Humans and coding agents:** follow [AGENTS.md](AGENTS.md) and [team coordination/model routing](docs/TEAM.md). **ChatGPT can coordinate and code via connected GitHub** while separately started Gemini CLI sessions execute scoped worker tasks through [GEMINI.md](GEMINI.md). The human integrator owns merge and final decisions; no automatic ChatGPT→Gemini dispatch exists. Codex is optional; legacy Claude files remain compatible.
2. **Mission and resources:** The [rice-field flood response mission](docs/PRODUCT.md) is selected. Read [the organiser-derived Robotics constraints](docs/EVENT.md), verify a running local simulator ([RUN](docs/RUN.md)), then build one **simulated paddy sensor → rover decision → physical movement → unexpected change → recovery** episode ([TESTING](docs/TESTING.md)). A drainage-gate mitigation is the preferred next slice, not a substitute for the functioning robot loop.
3. **Before parallel coding:** agree the smallest shared interface needed for a runnable end-to-end path in [architecture](docs/ARCHITECTURE.md) and [contracts](docs/CONTRACTS.md). Divide **independent** modules by branch/owner (an issue, PR or short message suffices).
4. **Build:** implement a smallest complete user-visible journey, then expand with tests and real [evaluation](docs/TESTING.md).
5. **Deliver:** follow [the actual Robotics submission brief](docs/DEMO.md): one-page technical PDF, **≤1-minute continuous** simulation video showing perception beside actions and one unexpected scene change, plus a **five-minute pitch if shortlisted**. Keep a buffer before the real portal deadline.

## Selected rice project — immediate implementation focus

**Start the next coding agent here:** [first runnable integration assignment](docs/RUN.md#next-agent-first-integration-task--not-yet-implemented). One integrator first proves a physics-actuated rover + real simulated range sensor + obstacle response, and freezes the `rice.v0` seam; then [five human roles and small scoped agent tasks](docs/TEAM.md#selected-rice-project-five-human-work-split-proposed-not-yet-assigned-by-name) can proceed in separate branches. Robot shape, sensor placement and gate mechanism are **human-reviewed**, not automatically chosen by the planner.

**Human problem:** A rice-field operator needs to react when water rises abnormally and the cause is not immediately known. Ordinary controlled shallow flooding is not a failure. A mobile robot should investigate, adapt to an unexpected obstruction, and verify whether a **safe, physically modelled** drainage intervention helps.

**Priority:** 1 rover + 1 real simulated sensor + local control + changing-world recovery **first**; next, a minimal gate/flow intervention if feasible; then a second rover/robot-rescue or a reforestation/space adapter **only if the core demo, evaluator and continuous video are already working**. Architecture and testable pass/fail details live in [PRODUCT](docs/PRODUCT.md), [ARCHITECTURE](docs/ARCHITECTURE.md) and [TESTING](docs/TESTING.md). No verified robotics implementation is implied.

## Hard constraint from the revealed challenge

**The coding-agent workflow and the robot's autonomy are separate systems.** Gemini can help develop/test the entry, but judges want core simulated decisions to respond to real simulated sensors, re-plan when the scene changes, and avoid being dependent on cloud LLM calls at every step. The later organiser email **recommends MuJoCo first**, with Modal only for optional GPU work. NVIDIA Isaac Sim/Modal streaming is **optional** and separate from policy placement. The presenter explicitly allowed MuJoCo/PyBullet/Gazebo/Webots as alternatives. Simulator/stream readiness comes before fancy agent orchestration.

**Confirmed judging weights:** **25% perception-driven action; 25% adaptation; 15% operational autonomy; 20% relevance and impact; 15% ambition/technical quality.** The clearer organiser screenshot specifies **evidence of a real need and visible handling of the critical failure mode**. [Exact provenance and top-score descriptions](docs/EVENT.md#confirmed-robotics-judging-rubric--clear-organiser-photograph-7-november-2026).

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
| [RESEARCH](docs/RESEARCH.md) | Source-checked simulator requirements, workflow/agent-skill evaluation and relevant prior art |

Use GitHub Issues for substantial tasks, PRs for reviewed integration and source/tests as the proof. **No ticket is needed for a five-minute fix.** When multiple agents work at once, follow [the short GitHub and human-prompting guide](docs/TEAM.md#practical-parallel-work-without-ceremony), not shadow task lists or dated session logs. The private hackathon management workflow is separate from this public team code repository.

## Human-led, ChatGPT + Gemini developer-agent workflow

**Avoid one massive context or one shared agent transcript.** ChatGPT can actively edit/review code via GitHub and supply branch-bounded assignments to a human-launched Gemini CLI worker. Use Gemini 4 Argon, if accessible, for deep cross-component planning and synthesis; reserve 3.8 Flash for bounded research/review with only relevant files and limited tools. An optional [scoped coding worker](.gemini/agents/scoped-implementer.md) and [read-only independent reviewer](.gemini/agents/independent-reviewer.md) are configured for Gemini CLI but **have not been locally run or validated**. Start worker sessions explicitly in separate branches/worktrees, with an actual assignment. The [team protocol](docs/TEAM.md) defines escalation, context packets, branch ownership, issue/PR-based conversations and human integration. These are **development agent preferences**, not a mandate for the selected robotics product.

The seven neutral workflow skills (including `embodied-loop`) live in `.agents/skills/`. Gemini CLI consumes them on demand; `/memory show`, `/skills list` and `/agents list` should be checked on the actual machine. Existing Claude-native files and complete skill mirrors are **legacy compatibility retained for the moment**; the existing CI parity check still expects them. Do not delete them or rely on their maintenance for a new workflow. Hosted chats must read repo content explicitly; no instruction file automatically launches worker agents, remote A2A services or live coordination.

**Team communication:** [ASSIGN → BLOCKED / HANDOFF → REVIEW → INTEGRATE](docs/TEAM.md#communication-protocol-between-developer-agents-chatgpt--gemini--humans), with GitHub issue comments for cross-worker tasks/blockers, a [PR handoff template](.github/PULL_REQUEST_TEMPLATE.md) for reviewed changes, and a human integrator coordinating merges. No additional message-server deployment or giant Markdown conversation folders. The repo is public; do not put private chats or credentials in its issue/PR threads. Actual model entitlement, API keys, limits, SDK invocation and grant credits remain to be verified.

## Working agreement

- Deliver the **measurable job** a real person needs, not a generic LLM chat demo. Explain why a deterministic algorithm, optimiser, controller or ordinary VLM cannot already do it as well.
- Confirm interfaces and ownership before parallel agent edits. Keep shared schema changes explicit and reviewed.
- Each claim needs a runnable proof, reference or an honest label such as *planned / simulated / not tested*.
- No secrets, private organiser messages, personal data, unauthorised licensed assets or large unchecked model weights in Git.
- **No final submission, deployment, spending, messaging or repository visibility changes without approval from the relevant human/team owner.**

This repository provides a process, not a promise of winning; the event's actual judging rubric is authoritative.
