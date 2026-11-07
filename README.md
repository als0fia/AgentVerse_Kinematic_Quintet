# The Kinematic Quintet — AgentVerse 2026

**Selected product: Antenna — rescue-first resilient collective robotics in a rice-field mission.** The canonical MuJoCo `ricebot` application now includes an integrated sensed mud-stall → risk-triage → typed docking/extraction → return-to-planting golden episode after [PR #48](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/48). Its [combined `main` CI](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/actions/runs/42418413738) passed **78 tests** including the golden-episode fixture and force/pitch assertions. This is **simulated, test-scoped evidence**, not proof of deployed sensors/radios, learned rescue policy, live distributed Jetson inference, independent multi-seed physical validity or a finished judge-ready video. MuJoCo is the verified development engine; Isaac Sim remains unverified.

**Naming and branch authority:** **Antenna is canonical.** “PaddyPatch” is its earlier working name; “RiceWatch” is historical flood-inspection ideation. A separately developed “RiceSwarm AI” farm-lifecycle experiment is **not** the submission's authoritative runtime. Only the integrated `main` application under `src/ricebot/` defines the current submission; an unmerged feature branch or its README must not silently redefine the product or replace the physical planting evidence. Historical merged PR branches may remain for provenance but are not separate active versions.

**Event:** UCL AI Society AgentVerse, 7–8 November 2026, Europe/London. The [official timetable](https://agentverse.uclai.co.uk/) lists the track reveal at **Saturday 12:30**, old-site final submissions **Sunday 14:00**, but latest authenticated BuilderBase dashboard explicitly says **Sunday 8 Nov 13:00 GMT**; **plan against 13:00** until clarified, and judging **Sunday 15:00–16:00**. Verify changes in the actual participant portal; official challenge details supersede these notes.

## Repository purpose and canonical branch

**`main` is the complete current instruction and integration base.** This public repo owns the actual team's **agentic coding instructions and eventually the submission code**, not the private idea archive or organiser credentials. Start from [AGENTS](AGENTS.md) and a narrowly relevant [skill](.agents/skills/); use [TEAM](docs/TEAM.md) for safe ChatGPT/Gemini/Codex handoffs and [PRODUCT](docs/PRODUCT.md) for the **selected cooperative rice planting P0** and physically grounded map/site/route coordination (recovery and mitigation optional). Physical robot recovery is now the core user-directed product goal; the agricultural planting-first scope was an earlier phase. [EVENT](docs/EVENT.md) owns official criteria, [RUN](docs/RUN.md) owns simulator preflight, and [TESTING](docs/TESTING.md) / [DEMO](docs/DEMO.md) own real evidence and judge deliverables.

Short-lived feature branches and PRs exist only for independent, reviewable changes. Merge approved prerequisites into `main` and branch new development from its current HEAD; do not treat a stacked PR or developer-agent branch as a second source of truth. Preserve real teammate contributions and don't force-update or delete branches without approval.

## Start here

**New computer or new agent?** Use [Workstation onboarding](docs/WORKSTATION-SETUP.md) for a safe local clone, Python checks, the optional MuJoCo toy smoke **and current PaddyPatch MuJoCo prototype**, plus the separate Isaac Sim hardware gate, Gemini/Antigravity instructions and actual evidence handoff. For any major feature or world-vs-robot handoff, use [Component ownership and audit](docs/COMPONENTS.md). These guides are reusable; don't create duplicate frameworks.

1. **Agent and human instructions:** read [AGENTS.md](AGENTS.md), [PRODUCT](docs/PRODUCT.md), [ARCHITECTURE](docs/ARCHITECTURE.md), [CONTRACTS](docs/CONTRACTS.md), and [TEAM](docs/TEAM.md). A human integrator owns the shared seam and actual decisions; Gemini CLI workers start **only** in human-owned worktrees.
2. **Run the merged MuJoCo prototype first on a suitable local machine:** install [the pinned optional dependency](requirements-mujoco.txt) and follow [RUN](docs/RUN.md#mujoco-rapid-prototyping-simulationport--isaac-sim-translation-protocol). **Separately** perform [Isaac preflight](docs/RUN.md#isaac-sim-preflight--chosen-target-not-yet-proven) before any translation; don't claim interchangeability until both engines have passed equivalent tests.
3. **Build the rescue-first vertical slice in existing MuJoCo:** preserve the already tested rover, local sensors and seed drop. Inject one genuine mid-episode immobilisation, require measured commanded-motion evidence, choose rescue/defer/abstain under safety bounds, then couple the helper using the existing rigid latch and independently verify extraction and continuing fieldwork. Do not add an extra simulator, winch, full rice lifecycle or polished UI in place of this.
4. **Assign one integrator and non-overlapping owners:** use the [team handoff](docs/TEAM.md#current-rescue-first-owner-handoff) for the existing mechanical skill, field incident injection, planner and evaluator. Use one tested `rice.v1` seam, not parallel incompatible robot controllers.
5. **Deliver:** one continuous ≤60-second rescue-first scene showing sampled evidence, an intervention/abstention decision, actual bounded contact/extraction (if attempted), second-robot safety and real post-incident mapping/planting, with independent baseline and one-page technical PDF. Until a complete run is recorded, label it a target, not a completed demo. See [DEMO](docs/DEMO.md) and [EVENT](docs/EVENT.md); submission requires human permission.

**Historical status after merged PR #28 (superseded for rescue integration by PR #48):** `src/ricebot/agent/mycelial_swarm.py` now implements **simulated Policy B**: separate `LocalMycelialMap` and `LocalSlamGrid` instances for each rover, event-triggered `MycelialGossipPacket` exchanges with modelled distance/link blackout, provenance deduplication, and task bids. Tests include isolated vs connected maps, blackout recovery and changed planting target. It runs inside a shared Python/MuJoCo simulation; **no physical UWB or Wi-Fi HaLow radios, deployed distributed processes, Jetson inference or integrated autonomous physical rescue have been verified**. Policy A's `CooperativeSwarmPlanner` remains a centralised baseline. The merged PR #24 separate scripted MuJoCo rigid-docking/tandem extraction test is physical simulation mechanics, not the full sensed-rescue mission.

## Antenna rescue-first direction and preserved fieldwork baseline

**What we physically do:** rover scout explores/samples a patchy rice field, planter surveys or receives fused observations, planner selects viable planting sites where the stand is missing, robot follows a safe route and **releases seeds through a real simulated hopper**. The world tests moisture/firmness variability through documented local sensing proxies, not a hidden perfect map. A surprise obstruction or low-quality measured patch forces updated routes and planting decisions. [PRODUCT](docs/PRODUCT.md) defines what is mandatory and what is optional.

**Current rescue acceptance priority:** independently stress-test the newly integrated `golden_rescue` episode from PR #48 (no oracle/simulator-truth leakage, grounded docking without excessive solver force, safe refusal and helper survival under high drag, matched incident baselines and continued real simulated seed contact). Use the same MuJoCo runtime, retain proven seed/soil functionality and record one continuous judge-facing demonstration. Do not claim hardware or LLM gains from unit tests. [Current plan](docs/SWARM_EDGE_PLAN.md).

**Earlier planting-first/Isaac assignment — historical only:** [archived first integration task](docs/RUN.md#next-agent-first-integration-task--isaac-sim-planting-first-skeleton-not-yet-implemented) records prior scope, not today's marching orders. MuJoCo rover/seed physics and scripted rigid-latch extraction pass headless checks, but Isaac Sim, real agriculture and an autonomous recovery episode remain unverified. The mechanical lead still owns the docking geometry and force limits.

**Prior scope history:** earlier RiceWatch fixed water-gauge verification and optional drainage gate were documented in [DECISIONS](docs/DECISIONS.md) as D-0005/D-0006; they are **superseded** by D-0007. Firewatch/reforestation/space are future application hypotheses only.

## Hard constraint from the revealed challenge

**The coding-agent workflow and the robot's autonomy are separate systems.** Gemini can help develop/test the entry, but judges require core physically simulated decisions that respond to sensed conditions and unexpected changes, without a cloud LLM on every motor tick. A later organiser email **recommended MuJoCo for laptop convenience**, and the **team currently develops with committed MuJoCo**, while **Isaac Sim remains the user-selected intended integration engine** subject to compatibility and actual tests; do not treat that recommendation as a mandated engine. A remote GPU/stream requires separate setup, budget/security approval and honest runtime disclosure. Simulation readiness and **real seed placement** come before fancy agent orchestration.

**Confirmed judging weights:** **25% perception-driven action; 25% adaptation; 15% operational autonomy; 20% relevance and impact; 15% ambition/technical quality.** The clearer organiser screenshot specifies **evidence of a real need and visible handling of the critical failure mode**. [Exact provenance and top-score descriptions](docs/EVENT.md#confirmed-robotics-judging-rubric--clear-organiser-photograph-7-november-2026).

## Python core and MuJoCo physics prototype (Isaac Sim still pending)

The `src/ricebot/` [Python core](src/README.md) has typed observations, sample aggregation, baseline assignments and a `SimulationPort`. The [MuJoCo adapter](src/ricebot/mujoco_sim/README.md) now models two rovers, synthetic soil probes and a seed-contact test. **Scripted core tests and MuJoCo physics tests are different evidence levels**; full CI installs [pinned MuJoCo](requirements-mujoco.txt) before running both. The Isaac Sim bridge, validated agronomy and a complete judge-ready demo remain unfinished. The human integrator owns shared `rice.v1` changes.

**Right-sized demo:** about 3–6 sample/plant sites, two reused simple rover bodies, one physical landed seed, one new obstacle/site reading, and a small shared map—**not** an entire rice-farming simulation.

## Foundation commands

**Core-only** tooling works on Python 3.10+; the **full test suite** requires Python 3.11+ and `python -m pip install -r requirements-mujoco.txt` first (CI uses 3.12):

```bash
python scripts/check_repo.py
python scripts/sync_skills.py --check
python -m unittest discover -s tests -v
python scripts/evaluate.py --candidate tests/fixtures/candidate.jsonl --baseline tests/fixtures/baseline.jsonl
```

The fixture comparison is **synthetic tooling data, not product performance**. Current MuJoCo runner commands and the unverified Isaac translation plan are in [RUN.md](docs/RUN.md).

## Documentation map

| Single source of truth | Contents |
| --- | --- |
| [EVENT](docs/EVENT.md) | Revealed challenge, official rules, resources and deadlines with source/time |
| [PRODUCT](docs/PRODUCT.md) | Real beneficiary, workflow, success criterion, alternative and selection rationale |
| [ARCHITECTURE](docs/ARCHITECTURE.md) | Modules, data/control flow, ownership and dependencies |
| [CONTRACTS](docs/CONTRACTS.md) | Versioned inter-module messages, units, errors and invariants |
| [COMPONENTS](docs/COMPONENTS.md) | World/robot/Isaac/kernel boundaries, mock swap, per-module documentation, independent audits |
| [SIMULATOR-VARIANTS](docs/SIMULATOR-VARIANTS.md) | Known teammate simulator/UI/agent variants, source commits, feature-selection matrix and safe reconciliation procedure |
| [WORKSTATION-SETUP](docs/WORKSTATION-SETUP.md) | Every teammate/agent: safe Windows setup, core checks, optional MuJoCo, gated Isaac, account hygiene |
| [TEAM](docs/TEAM.md) | Ownership protocol, parallel tasks, integration and handoff |
| [RUN](docs/RUN.md) | Reproducible setup, start, smoke and recovery commands |
| [TESTING](docs/TESTING.md) | Test matrix, baseline metrics, evaluation cases and failure taxonomy |
| [DEMO](docs/DEMO.md) | Live run, fallback, judge evidence and submission gate |
| [PROVENANCE](docs/PROVENANCE.md) | Attribution, pre-existing work, AI assistance, licensing and claims |
| [DECISIONS](docs/DECISIONS.md) | Architecture/engineering decisions and trade-offs |
| [SWARM_EDGE_PLAN](docs/SWARM_EDGE_PLAN.md) | Rescue-first architecture and exact swarm/edge implementation status |
| [RESCUE_EDGE_RESEARCH](docs/RESCUE_EDGE_RESEARCH.md) | Physical rescue, local/partitioned model research, baselines and failure tests |
| [IMMOBILIZATION_SCENARIOS](docs/IMMOBILIZATION_SCENARIOS.md) | Eight distinct immobilisation and safe-rescue test hypotheses |
| [ANTENNA_RF_SUBSYSTEM](docs/ANTENNA_RF_SUBSYSTEM.md) | Corrected RF/sensor feasibility and real-versus-simulated boundaries |
| [COMMERCIALIZATION_CHINA](docs/COMMERCIALIZATION_CHINA.md) | Sourced Chinese rice mechanisation and commercial evidence gates |
| [RESEARCH](docs/RESEARCH.md) | Source-checked simulator requirements, workflow/agent-skill evaluation and relevant prior art |
| [SERVER_ECONOMICS_AND_RELIABILITY](docs/SERVER_ECONOMICS_AND_RELIABILITY.md) | GPU sizing, throughput, fleet scaling, CAPEX/OPEX, and multi-tier failure recovery |

Use GitHub Issues for substantial tasks, PRs for reviewed integration and source/tests as the proof. **No ticket is needed for a five-minute fix.** When multiple agents work at once, follow [the short GitHub and human-prompting guide](docs/TEAM.md#practical-parallel-work-without-ceremony), not shadow task lists or dated session logs. The private hackathon management workflow is separate from this public team code repository.

## Human-led, ChatGPT + Gemini developer-agent workflow

**Local-first workflow:** the five human owners use their locally running coding agents (including Gemini/Argon) to implement, actually run/test, commit and push focused features on separate branches; the human integrator reviews, merges and tests the combined `main`. ChatGPT's normal role is **technical advice, code/PR audits and requested documentation updates**, not remotely pushing application code unless explicitly asked. Every substantial PR links genuine local run evidence; GitHub CI alone does not prove an Isaac simulation. See [TEAM](docs/TEAM.md#local-first-development-review-and-integration) and the [PR template](.github/PULL_REQUEST_TEMPLATE.md). **Avoid one massive context or one shared agent transcript.** The human owner can give a branch-bounded assignment to a locally launched Gemini CLI worker. Use Gemini 4 Argon, if accessible, for deep cross-component planning and synthesis; reserve 3.8 Flash for bounded research/review with only relevant files and limited tools. An optional [scoped coding worker](.gemini/agents/scoped-implementer.md) and [read-only independent reviewer](.gemini/agents/independent-reviewer.md) are configured for Gemini CLI but **have not been locally run or validated**. Start worker sessions explicitly in separate branches/worktrees, with an actual assignment. The [team protocol](docs/TEAM.md) defines escalation, context packets, branch ownership, issue/PR-based conversations and human integration. These are **development agent preferences**, not a mandate for the selected robotics product.

The seven neutral workflow skills (including `embodied-loop`) live in `.agents/skills/`. Gemini CLI consumes them on demand; `/memory show`, `/skills list` and `/agents list` should be checked on the actual machine. Existing Claude-native files and complete skill mirrors are **legacy compatibility retained for the moment**; the existing CI parity check still expects them. Do not delete them or rely on their maintenance for a new workflow. Hosted chats must read repo content explicitly; no instruction file automatically launches worker agents, remote A2A services or live coordination.

**Team communication:** [ASSIGN → BLOCKED / HANDOFF → REVIEW → INTEGRATE](docs/TEAM.md#communication-protocol-between-developer-agents-chatgpt--gemini--humans), with GitHub issue comments for cross-worker tasks/blockers, a [PR handoff template](.github/PULL_REQUEST_TEMPLATE.md) for reviewed changes, and a human integrator coordinating merges. No additional message-server deployment or giant Markdown conversation folders. The repo is public; do not put private chats or credentials in its issue/PR threads. Actual model entitlement, API keys, limits, SDK invocation and grant credits remain to be verified.

## Working agreement

- Deliver the **measurable job** a real person needs, not a generic LLM chat demo. Explain why a deterministic algorithm, optimiser, controller or ordinary VLM cannot already do it as well.
- Confirm interfaces and ownership before parallel agent edits. Keep shared schema changes explicit and reviewed.
- Each claim needs a runnable proof, reference or an honest label such as *planned / simulated / not tested*.
- No secrets, private organiser messages, personal data, unauthorised licensed assets or large unchecked model weights in Git.
- **No final submission, deployment, spending, messaging or repository visibility changes without approval from the relevant human/team owner.**

This repository provides a process, not a promise of winning; the event's actual judging rubric is authoritative.
