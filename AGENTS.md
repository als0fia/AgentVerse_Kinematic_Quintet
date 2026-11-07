# Agent instructions — AgentVerse team submission repository

The repo is public. It contains **shared submission engineering**, not personal hackathon operations. Never read private personal-ops files as an input to team coding or paste restricted BuilderBase/Discord/team content here.

## Boot sequence (smallest relevant context)

1. Follow this file as the single repository-wide instruction contract. Use `README.md` for orientation; inspect the current working tree/branch and relevant diffs, issues and PRs before changing code.
2. For project selection or a changed brief, use the **complete, photo-verified five-category judging rubric** in `docs/EVENT.md` and the beneficiary/evidence gate in `docs/PRODUCT.md`; for parallel work use `docs/TEAM.md`. If the challenge or project is unresolved, work only on reusable infrastructure or explicitly requested research.
3. For interface/runtime changes, use `docs/ARCHITECTURE.md`, `docs/CONTRACTS.md`, `docs/RUN.md` and relevant tests. For evaluation/demo use `docs/TESTING.md` / `docs/DEMO.md`. For tiny fixes, do not read unrelated documents.
4. For a new developer computer use [WORKSTATION-SETUP](docs/WORKSTATION-SETUP.md); for ownership and implementation-specific documentation use [COMPONENTS](docs/COMPONENTS.md). Agent-onboarding must report real commands/results; the currently tested and approved P0 simulator is MuJoCo; Isaac Sim translation remains unverified and is not a precondition for the rescue demo.
5. Load only the **matching** `.agents/skills/<skill>/SKILL.md` when its workflow applies; do not inject all skills, histories and tool outputs into every prompt. Gemini CLI loads task skills on demand. Skills do not grant extra tools or authority.

## Current selected product: Antenna — rescue-first collective robotics

**Post-PR #48 status (7 Nov 2026, `main` `16768dc`):** The single `scripts/run_mujoco_paddypatch.py --scenario golden_rescue` MuJoCo path now includes a typed `TaskMode.RESCUE`, a bounded docking/extraction command, `RecoveryFeedback` and post-rescue planting. [PR #48](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/48) merged; [combined GitHub CI](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/actions/runs/42418413738) ran **78 tests successfully**. The integrated test now checks two seed drops, maximum recorded equality-constraint force below 250 N and pitch below 0.12 rad in its fixture. **This proves the checked-in simulation/test path only:** helper survivability across hazardous traction, policy-only sensing without privileged simulator fault flags, independent multi-seed baselines, live model benefit, physical hardware and a finished submission video/PDF are *not* established by that CI.

**User direction (7 Nov, after naming PR #25):** Antenna is canonical (PaddyPatch former name). The core intelligence is **detecting, assessing, coordinating and physically recovering a disabled robot when worthwhile**, with rice-field mapping, terrain evidence and planting as meaningful real physical work. This supersedes planting-first/rescue-optional scope; see naming [D-0008](docs/DECISIONS.md), scope [D-0009](docs/DECISIONS.md), [PRODUCT](docs/PRODUCT.md), [SWARM_EDGE_PLAN](docs/SWARM_EDGE_PLAN.md) and [RESCUE_EDGE_RESEARCH](docs/RESCUE_EDGE_RESEARCH.md).

**Historical pre-PR #48 snapshot (for provenance):** `rice.v1` and MuJoCo field/seed physics exist; PR #23 implemented **single-process** shared-map bids and heuristic recovery triage; PR #24 added homogeneous Paddy Ant **rigid docking/cam-latch/pitch-lift** MJCF assets with a scripted MuJoCo tandem extraction test asserting the victim moves >0.08 m. This is physical **test-level** recovery mechanics, **not** autonomously sensed fault → contextual decision → learned extraction → recovered mission. **Simulated Policy B now has per-rover partial maps and modelled P2P gossip (PR #28)**, but no deployed radios, local Jetson LLM or distributed inference is proven.

**Historical pre-PR #48 integration checklist (superseded by the status above):** (1) the conservative incident detector now uses explicitly issued commands, observed displacement/heading and sensor validity (or an explicit synthetic bogged flag), with tests for safe holds and missing range; (2) connect typed high-level rescue intent to tested robot mechanism under bounds; (3) independently verify contact/displacement, second-robot risk and resumed soil/planting task; (4) evaluate calculated deterministic rescue policy vs contextual event-driven AI on matched incidents, including justified **not rescuing**. Existing `rescue_assist(...)` creates a `SAMPLE` rendezvous, not a physical rescue. No integrated autonomous extraction is yet proven. Preserve all existing source/tests and the human robot-mechanics owner. No model outputs directly control torque.

**Update after merged PR #28 (7 Nov 2026):** `src/ricebot/agent/mycelial_swarm.py` now implements **simulated Policy B**: separate `LocalMycelialMap` and `LocalSlamGrid` instances for each rover, event-triggered `MycelialGossipPacket` exchanges with modelled distance/link blackout, provenance deduplication, and task bids. Tests include isolated vs connected maps, blackout recovery and changed planting target. It runs inside a shared Python/MuJoCo simulation; **no physical UWB or Wi-Fi HaLow radios, deployed distributed processes, Jetson inference or integrated autonomous physical rescue have been verified**. Policy A's `CooperativeSwarmPlanner` remains a centralised baseline. The merged PR #24 separate scripted MuJoCo rigid-docking/tandem extraction test is physical simulation mechanics, not the full sensed-rescue mission.

**Edge compute:** original Jetson Nano 4GB is not Orin Nano Super 8GB. One larger quantised LLM can in principle be partitioned across **real verified** edge nodes over local links, but it can fail when the disabled robot loses a necessary shard. Always keep safe offline fallback. Hardware count, model performance and expenses remain unverified; do not spend, provision, share credentials or deploy public RPC. An optional research study must not block the physical rescue demo.

## AgentVerse Robotics judge-alignment gate (event-specific)

**Before selecting a project, changing the embodied system, declaring a feature complete, or preparing a judge-facing claim**, read the organiser-derived [challenge and scoring evidence](docs/EVENT.md), then use [testing](docs/TESTING.md) and [demo requirements](docs/DEMO.md) for the applicable task. A reviewer/evaluator must challenge any unsupported claim; a developer must not substitute the appearance of an agent for behaviour in simulation.

Every candidate and its acceptance tests must address these five **onsite organiser judging dimensions**:

1. **Perception-driven action (25%):** at least one *real simulated sensor* informs the agent's decision, and that decision causes a meaningful action in the simulated world. No fixed-path animation, privileged-state shortcut disguised as perception, or static chat wrapper.
2. **Adaptation to change (25%):** introduce an unexpected **mid-episode** change, observe it through permitted sensors and demonstrate changed behaviour, recovery or justified safe stop—not a separate hand-scripted second trajectory.
3. **Operational autonomy (15%):** keep critical short-horizon decisions on-device/inside the simulator runtime where feasible, measure model calls and latency, and handle loss of any remote reasoning service honestly. A cloud-hosted *simulator* is distinct from a remotely dependent *control policy*.
4. **Relevance and impact (20%):** name an evidence-backed human need, a useful outcome, and the most critical failure handled visibly in the demo; compare one credible non-agent baseline and show reproducible evidence.
5. **Ambition and technical quality (15%):** deliver a genuinely demanding, well-engineered robot capability rather than relying on a model brand or a scripted tutorial.

**Submission gate:** prepare the organiser's **one-page technical PDF**, **≤60-second single continuous simulation recording** with perception and action visible plus a mid-run disturbance, and **five-minute pitch only if shortlisted**. Verify exact portal fields and actual deadline before submission. The newer clear organiser photograph verifies the **complete** Robotics weighting: perception 25%, adaptation 25%, autonomy 15%, **relevance/impact 20%**, ambition/technical quality 15%; [EVENT](docs/EVENT.md#confirmed-robotics-judging-rubric--clear-organiser-photograph-7-november-2026) is the canonical source. Require a real evidenced human need and on-screen handling of the critical failure.

When scope is cut, maintain a demonstrable sensor → decision → action loop and measurable disturbance response ahead of optional media, multi-agent orchestration and marketing polish. Do not claim that CI/unit checks alone demonstrate any of these judge criteria. These constraints apply **only to this AgentVerse Robotics entry** and do not override the team-selected solution or unverified sponsor rules.

## Execution ownership — local-first

**Current team instruction:** application development, simulator debugging, test execution and feature branch pushes are done by **human-directed local coding agents** (primarily the Isaac Sim workstation/Argon where available). Each worker pulls current `main`, operates on an owned local branch/worktree, runs tests and the actual Isaac scene when claiming physical behaviour, then supplies a PR with local commands, observed output, tested commit, sensor/action/seed-contact traces and failures. The human integrator approves merges and checks combined `main`. **This connected ChatGPT session normally provides advice, read-only auditing and requested small documentation updates; do not push application code unless explicitly asked again.** Never infer local physics success from CI or source review. [TEAM](docs/TEAM.md#local-first-development-review-and-integration) and the [PR template](.github/PULL_REQUEST_TEMPLATE.md) own the detailed handoff.

**Practical priority:** preserve tested seed/sensor physics, integrate the existing rigid docking test into a physically credible risk-aware rescue-or-abstain episode, then consider learned policies and optional pooled edge inference. Field/map/evaluation owners can work in parallel on narrow fixtures, but must not change shared contracts without the integrator. The full final demo still shows measured coordination and response to a mid-episode surprise.

## Order of work

1. Identify a real beneficiary, environment and observable outcome. Check closest non-LLM alternative/prior art; require actual AI-added judgement.
2. For a tiny independent fix, implement and test directly within the assigned branch/task; no new issue, planning document or approval ceremony is required. For concurrent or cross-module work, communicate owner, branch, intended paths, acceptance check and dependencies in an existing issue/PR or short human message.
3. For any shared interface change, specify fields, types, units, frame, semantics, version, errors and fixtures **before** consumers implement against it. Integrator owns shared contracts.
4. If others are working simultaneously, use a separate branch (and a separate local worktree/checkout if sharing a machine). Avoid two active writers to the same file or contract; coordinate shared changes through the integrator. Small non-overlapping solo work follows the current team branch convention.
5. Test in layers: pure logic → module contract → integration → real end-to-end episode; compare the nearest meaningful baseline. Keep deterministic random seeds and environment details.
6. For a substantial feature, maintain a concise **component-local README once the component exists** (purpose, owner, supported API/units, run/test commands actually executed, limitations, extension/replace steps) per [COMPONENTS](docs/COMPONENTS.md). Do not create placeholder docs or additional AGENTS files for tiny fixes; keep all collaborators using the same source of truth.
7. Debug by reproducing, collecting exact traces/logs, locating root cause, applying the minimal correction, adding a regression test and rerunning relevant checks. No speculative patch stacking.
8. Report changed paths, commands **actually** run, results, commit/PR, unknowns, blockers and rollback path. Never claim "tested" from code reading.
9. Keep one canonical current-truth document per subject; link to it instead of copying. Git history retains earlier decisions.

## Multi-agent cooperation

- **Interchangeable collaborators:** ChatGPT and Gemini may swap **world/simulation** or **robot embodiment** ownership as assigned by human owners. Keep the single writer for the `rice.v1` kernel and shared Isaac composition. Actual package boundaries, one mock-to-real robot seam and component README requirements are specified in [COMPONENTS](docs/COMPONENTS.md).
- On requests to "audit", "streamline" or "split" a program section, follow [module-audit](.agents/skills/module-audit/SKILL.md): assess actual code/tests and documentation **read-only by default**, with severity/evidence/owner and exact verification gaps. **Do not silently refactor** someone else's feature.


- Team human/integration lead owns task allocation, merge order, integration and final demo. ChatGPT can write/review on an isolated GitHub branch; Gemini CLI workers are separately launched, not directly controllable from ChatGPT chat. For cross-agent communication use `docs/TEAM.md` and the PR template; never claim automatic model dispatch.
- Implementation agents own isolated modules and corresponding tests; shared contracts change only through coordinated review.
- Independent evaluator probes claims and edge cases without changing the test to make a failing candidate appear successful.
- Demo lead owns the audience story and fallback but may not invent production capability.
- Follow the **minimal concurrency and GitHub safety protocol** in `docs/TEAM.md`: start from fresh head, work in an owned branch, never force-overwrite others' commits, and recheck remote state before final write/merge. This applies equally to GitHub API connector agents and local git agents.
- At handoff, report branch/commit/PR, touched paths and exact test evidence; include contract version and risks only if relevant. No always-growing session journal.
- GitHub Issues help track meaningful work; a quick fix does **not** require an issue. A short explicit human assignment is enough to separate two agents' work.

## Safety and claims

- Do not run opaque downloaded scripts, destructive commands, high-risk external actions or send secrets to a model/tool.
- No spending/cloud credit use, public deployment, submission, messaging outsiders, secrets, force pushes, deletion or visibility change without explicit human permission.
- Confirm challenge timing and prior work/AI disclosure rules with organisers. An absence of a restriction is not permission.
- The robot's high-rate safety-critical control should be deterministic or appropriate specialised control; use an LLM only where its reasoning adds value and bound its authority.
- Always distinguish actual hardware operation, simulation, replay, scripted fallback, and proposed future work.

## Task routing

| Task | Read |
| --- | --- |
| Assess direction | `.agents/skills/problem-first/SKILL.md` + `docs/PRODUCT.md` |
| Plan shared interface | `.agents/skills/contracts-first/SKILL.md` + `docs/CONTRACTS.md` |
| Build component | `.agents/skills/implement-and-verify/SKILL.md` + tests |
| Diagnose bug | `.agents/skills/root-cause-debug/SKILL.md` |
| Coordinate parallel agents | `.agents/skills/parallel-handoff/SKILL.md` + `docs/TEAM.md` |
| Compare evidence | `.agents/skills/evaluate-and-demo/SKILL.md` + `docs/TESTING.md` |
| Set up a teammate's machine, optional MuJoCo or gated Isaac | `.agents/skills/workstation-setup/SKILL.md` + `docs/WORKSTATION-SETUP.md` |
| Audit/split/streamline a subsystem | `.agents/skills/module-audit/SKILL.md` + `docs/COMPONENTS.md` |
| Get a real simulated sensor/action loop working | `.agents/skills/embodied-loop/SKILL.md` + `docs/EVENT.md` and `docs/RUN.md` |

## Gemini-first entrypoints and scoped delegation

- **Primary:** Gemini CLI imports `AGENTS.md` through `GEMINI.md`. The seven canonical on-demand skills (including `embodied-loop`) live in `.agents/skills/`; optional specialised Gemini subagents are under `.gemini/agents/`. Run `/memory show`, `/skills list`, `/agents list` to verify actual discovery when locally installed. No file automatically starts a worker.
- **Model/context routing:** follow [team coordination](docs/TEAM.md). Prefer Gemini 4 Argon for complex cross-module reasoning and Gemini 3.8 Flash for bounded specialist work, *if the team has access*. Supply the smallest relevant files, latest commit, interface and test fixture, not every chat or full repo. Use measured results, not marketing cache-hit guarantees.
- **Coordination:** GitHub issue/PR threads and owned branches/worktrees are the default shared interface. No automatic live agent messenger or global inbox. Human integrator approves priorities and merging, and each AI run must be invoked explicitly.
- **Other tooling:** Codex may follow the same neutral source `AGENTS.md` and skills. Claude-native entrypoints, agents and full skill mirrors are **retained as legacy compatibility only**, not the preferred route; existing CI currently checks their parity. Do not delete them or change shared CI without reconciling the team. Any assistant without workspace-native discovery must read relevant files explicitly.
- Do not impose multi-agent ceremony on tiny solo changes. Create independent subagents only when a bounded task actually benefits; Gemini CLI subagents do not recursively delegate.

The role/routing guidance above is for **developer agents**, not a commitment to use a particular model inside the eventual robotics product.

The existence of an agent file or automation does **not** mean any assistant has access to multiple runtimes, is executing in background, or has passed a test.
