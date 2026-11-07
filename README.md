# The Kinematic Quintet — AgentVerse 2026

**Selected working direction (7 Nov 2026): Rice-field flood monitoring, active investigation and targeted mitigation**, on a reusable multi-actor decision/recovery framework. [PRODUCT](docs/PRODUCT.md) defines the rice mission and its smallest real simulated episode. The **human robot design, exact mitigation actuator, simulator installation, model and operating demo have not yet been confirmed or implemented**; this remains a build-ready proposal, not a functioning submission.

**Event:** UCL AI Society AgentVerse, 7–8 November 2026, Europe/London. The [official timetable](https://agentverse.uclai.co.uk/) lists the track reveal at **Saturday 12:30**, old-site final submissions **Sunday 14:00**, but latest authenticated BuilderBase dashboard explicitly says **Sunday 8 Nov 13:00 GMT**; **plan against 13:00** until clarified, and judging **Sunday 15:00–16:00**. Verify changes in the actual participant portal; official challenge details supersede these notes.

## Repository purpose and canonical branch

**`main` is the complete current instruction and integration base.** This public repo owns the actual team's **agentic coding instructions and eventually the submission code**, not the private idea archive or organiser credentials. Start from [AGENTS](AGENTS.md) and a narrowly relevant [skill](.agents/skills/); use [TEAM](docs/TEAM.md) for safe ChatGPT/Gemini/Codex handoffs and [PRODUCT](docs/PRODUCT.md) for the **selected rice-flood direction** (monitor/investigate/mitigate, with reusable rescue/other-domain interfaces as later extensions). [EVENT](docs/EVENT.md) owns official criteria, [RUN](docs/RUN.md) owns simulator preflight, and [TESTING](docs/TESTING.md) / [DEMO](docs/DEMO.md) own real evidence and judge deliverables.

Short-lived feature branches and PRs exist only for independent, reviewable changes. Merge approved prerequisites into `main` and branch new development from its current HEAD; do not treat a stacked PR or developer-agent branch as a second source of truth. Preserve real teammate contributions and don't force-update or delete branches without approval.

## Start here

1. **Humans and coding agents:** follow [AGENTS.md](AGENTS.md) and [team coordination/model routing](docs/TEAM.md). **ChatGPT can coordinate and code via connected GitHub** while separately started Gemini CLI sessions execute scoped worker tasks through [GEMINI.md](GEMINI.md). The human integrator owns merge and final decisions; no automatic ChatGPT→Gemini dispatch exists. Codex is optional; legacy Claude files remain compatible.
2. **Selected product and resources:** Read [rice flood mission](docs/PRODUCT.md), [simulation/agent/UI boundaries](docs/ARCHITECTURE.md), [v0 interfaces](docs/CONTRACTS.md) and the [first executable build gate](docs/RUN.md#next-agent-first-integration-task--not-yet-implemented). Verify local MuJoCo or an organiser-allowed fallback **on an actual machine**, then prove one physically actuated rover **sensor → decision → action → changed-scene recovery** episode before dividing further features. [EVENT](docs/EVENT.md) has the five-category rubric; cloud/GPU credits remain unverified.
3. **Before parallel coding:** the integrator proves and freezes the smallest `rice.v0` interface in [CONTRACTS](docs/CONTRACTS.md). Use [the five-role split and human-led robot-design gate](docs/TEAM.md#selected-rice-project-five-human-work-split-proposed-not-yet-assigned-by-name) to assign independent paths/branches and one testable result each. Do not have multiple agents independently redesign contracts, rover geometry or the physics engine.
4. **Build:** implement a smallest complete user-visible journey, then expand with tests and real [evaluation](docs/TESTING.md).
5. **Deliver:** follow [the actual Robotics submission brief](docs/DEMO.md): one-page technical PDF, **≤1-minute continuous** simulation video showing perception beside actions and one unexpected scene change, plus a **five-minute pitch if shortlisted**. Keep a buffer before the real portal deadline.

## First build: what must run before polishing anything

**Milestone 0:** one simplified wheeled rover travelling on a rice-field service track, reading a simulated range sensor, using locally bounded motion, and visibly replanning or safely stopping when an obstacle appears **mid-episode**. This establishes the actual embodied interface between simulator, controller, agent and read-only UI. **Milestone 1:** investigate an abnormal water-level reading and dispatch the scout to a useful inspection point. **Milestone 2 (desired stretch):** a physically actuated drainage gate changes a clearly simplified water proxy; the agent observes whether mitigation helped. **Milestone 3 (stretch):** add one stranded robot / second scout for meaningful recovery planning.

Do not sell ordinary paddy flooding as a defect, precision rice planting as a differentiator, or simulated water proxies as real crop-yield science. Human role 2 approves robot body/sensor/actuator design. The highest-value next task is in [RUN](docs/RUN.md#next-agent-first-integration-task--not-yet-implemented); the team decomposition and short agent commands are in [TEAM](docs/TEAM.md#selected-rice-project-five-human-work-split-proposed-not-yet-assigned-by-name). **There is not yet a working robot in this repository.**

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
