# The Kinematic Quintet — AgentVerse 2026

**Selected hackathon product:** **Antenna — cooperative rice-field soil mapping and targeted stand-gap replanting.** In a **prepared/direct-seeded or partially drained rice plot**, two physically simulated robots collect different local soil/stand evidence, fuse a **shared uncertainty-aware map**, coordinate traversable sample/plant routes, and **actually drop a seed proxy into a missing crop patch**. The system replans when a route or local soil reading changes unexpectedly. **Swarm rescue and flood monitoring/mitigation are optional future features, not the core.** **Isaac Sim is the user-selected simulator for now**, not yet verified running or compatible with a team machine.

**Event:** UCL AI Society AgentVerse, 7–8 November 2026, Europe/London. The [official timetable](https://agentverse.uclai.co.uk/) lists the track reveal at **Saturday 12:30**, old-site final submissions **Sunday 14:00**, but latest authenticated BuilderBase dashboard explicitly says **Sunday 8 Nov 13:00 GMT**; **plan against 13:00** until clarified, and judging **Sunday 15:00–16:00**. Verify changes in the actual participant portal; official challenge details supersede these notes.

## Repository purpose and canonical branch

**`main` is the complete current instruction and integration base.** This public repo owns the actual team's **agentic coding instructions and eventually the submission code**, not the private idea archive or organiser credentials. Start from [AGENTS](AGENTS.md) and a narrowly relevant [skill](.agents/skills/); use [TEAM](docs/TEAM.md) for safe ChatGPT/Gemini/Codex handoffs and [PRODUCT](docs/PRODUCT.md) for the **selected cooperative rice planting P0** and physically grounded map/site/route coordination (recovery and mitigation optional). Swarm Recovery is a potential extension, not a competing selected product. [EVENT](docs/EVENT.md) owns official criteria, [RUN](docs/RUN.md) owns simulator preflight, and [TESTING](docs/TESTING.md) / [DEMO](docs/DEMO.md) own real evidence and judge deliverables.

Short-lived feature branches and PRs exist only for independent, reviewable changes. Merge approved prerequisites into `main` and branch new development from its current HEAD; do not treat a stacked PR or developer-agent branch as a second source of truth. Preserve real teammate contributions and don't force-update or delete branches without approval.

## Start here

**New computer or new agent?** Use [Workstation onboarding](docs/WORKSTATION-SETUP.md) for a safe local clone, Python checks, optional MuJoCo toy-physics test, Isaac Sim hardware gate, Gemini/Antigravity instructions and actual evidence handoff. For any major feature or world-vs-robot handoff, use [Component ownership and audit](docs/COMPONENTS.md). These guides are reusable; don't create duplicate frameworks.

1. **Agent and human instructions:** read [AGENTS.md](AGENTS.md), [PRODUCT](docs/PRODUCT.md), [ARCHITECTURE](docs/ARCHITECTURE.md), [CONTRACTS](docs/CONTRACTS.md), and [TEAM](docs/TEAM.md). A human integrator owns the shared seam and actual decisions; Gemini CLI workers start **only** in human-owned worktrees.
2. **Confirm Isaac Sim:** follow [RUN](docs/RUN.md#isaac-sim-preflight--chosen-target-not-yet-proven); check actual GPU/VRAM/RAM/driver and physics startup on the intended workstation or approved compute. The organiser later suggested MuJoCo for laptop convenience, **but the user has explicitly chosen Isaac Sim for this project**; don't switch without human agreement.
3. **Build physical planting before a framework:** first actuated rover/range sensor and obstacle-replan; **immediately a physically released seed proxy with measured ground contact**. Then bring in a second scout, realistic local soil/stand proxies, shared estimated map, sample/site selection, safe routes, and new mid-episode evidence.
4. **Divide by single owners:** after the first tested `rice.v1` observation/action/seed-contact fixture, use [five-person split](docs/TEAM.md#selected-rice-project-five-human-work-split-proposed-not-yet-assigned-by-name), small focused coding agent tasks and integration PRs; don't let five agents create incompatible simulators or schemas.
5. **Deliver:** one continuous ≤60s scene showing actual sensors, both robots, fused map and changed-world **physical seed placement**, independently tested baseline and one-page PDF. Use [DEMO](docs/DEMO.md) and [EVENT](docs/EVENT.md); final submission needs explicit permission.

## Selected rice project — immediate implementation focus

**What we physically do:** rover scout explores/samples a patchy rice field, planter surveys or receives fused observations, planner selects viable planting sites where the stand is missing, robot follows a safe route and **releases seeds through a real simulated hopper**. The world tests moisture/firmness variability through documented local sensing proxies, not a hidden perfect map. A surprise obstruction or low-quality measured patch forces updated routes and planting decisions. [PRODUCT](docs/PRODUCT.md) defines what is mandatory and what is optional.

**Next builder:** [exact first Isaac Sim integration assignment](docs/RUN.md#next-agent-first-integration-task--isaac-sim-planting-first-skeleton-not-yet-implemented); first proof is actual wheel motion, a sampled ray sensor and **actual seed release/contact**. The second robot and soil map are the next integrated mission slice. **No working scene or planting is yet verified.** The mechanical lead approves simple wheeled body, reach and hopper geometry. A map without real planting and a clever rescue system with no task completion are not sufficient.

**Prior scope history:** earlier RiceWatch fixed water-gauge verification and optional drainage gate were documented in [DECISIONS](docs/DECISIONS.md) as D-0005/D-0006; they are **superseded** by D-0007. Firewatch/reforestation/space are future application hypotheses only.

## Hard constraint from the revealed challenge

**The coding-agent workflow and the robot's autonomy are separate systems.** Gemini can help develop/test the entry, but judges require core physically simulated decisions that respond to sensed conditions and unexpected changes, without a cloud LLM on every motor tick. A later organiser email **recommended MuJoCo for laptop convenience**, but the **team's current implementation choice is Isaac Sim**, subject to *actual GPU/system-memory compatibility and a tested run*; do not treat that recommendation as a mandated engine. A remote GPU/stream requires separate setup, budget/security approval and honest runtime disclosure. Simulation readiness and **real seed placement** come before fancy agent orchestration.

**Confirmed judging weights:** **25% perception-driven action; 25% adaptation; 15% operational autonomy; 20% relevance and impact; 15% ambition/technical quality.** The clearer organiser screenshot specifies **evidence of a real need and visible handling of the critical failure mode**. [Exact provenance and top-score descriptions](docs/EVENT.md#confirmed-robotics-judging-rubric--clear-organiser-photograph-7-november-2026).

## Code foundation now present (Isaac Sim scene still pending)

The `src/ricebot/` [Python core](src/README.md) now has typed observations, local sample aggregation, a deterministic sample/plant/hold baseline, bounded drive/drop intentions and a `SimulationPort`/read-only viewer seam. [Unit tests](tests/test_ricebot_foundation.py) use scripted observations only. This is **not an Isaac Sim rover or planted seed**. The next agent builds the *real Isaac physics/seed-drop adapter*; the human integrator alone owns shared `rice.v1` changes.

**Right-sized demo:** about 3–6 sample/plant sites, two reused simple rover bodies, one physical landed seed, one new obstacle/site reading, and a small shared map—**not** an entire rice-farming simulation.

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
| [COMPONENTS](docs/COMPONENTS.md) | World/robot/Isaac/kernel boundaries, mock swap, per-module documentation, independent audits |
| [WORKSTATION-SETUP](docs/WORKSTATION-SETUP.md) | Every teammate/agent: safe Windows setup, core checks, optional MuJoCo, gated Isaac, account hygiene |
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
