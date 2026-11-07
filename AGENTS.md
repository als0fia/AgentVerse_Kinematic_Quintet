# Agent instructions — AgentVerse team submission repository

The repo is public. It contains **shared submission engineering**, not personal hackathon operations. Never read private personal-ops files as an input to team coding or paste restricted BuilderBase/Discord/team content here.

## Boot sequence (smallest relevant context)

1. Follow this file as the single repository-wide instruction contract. Use `README.md` for orientation; inspect the current working tree/branch and relevant diffs, issues and PRs before changing code.
2. For project selection or a changed brief, use the **complete, photo-verified five-category judging rubric** in `docs/EVENT.md` and the beneficiary/evidence gate in `docs/PRODUCT.md`; for parallel work use `docs/TEAM.md`. If the challenge or project is unresolved, work only on reusable infrastructure or explicitly requested research.
3. For interface/runtime changes, use `docs/ARCHITECTURE.md`, `docs/CONTRACTS.md`, `docs/RUN.md` and relevant tests. For evaluation/demo use `docs/TESTING.md` / `docs/DEMO.md`. For tiny fixes, do not read unrelated documents.
4. Load only the **matching** `.agents/skills/<skill>/SKILL.md` when its workflow applies; do not inject all skills, histories and tool outputs into every prompt. Gemini CLI loads task skills on demand. Skills do not grant extra tools or authority.

## Current selected project (7 November 2026)

**Superseding user decision:** **PaddyPatch — cooperative rice-field stand-gap mapping and targeted replanting.** The robots must **perform their field task**. P0 includes a physically simulated planter, a scout collecting independent local soil/stand proxy observations, shared uncertainty-aware field map, target selection, safe route allocation, a **real physically actuated seed-proxy drop/contact** and an unexpected mid-episode route/site change with adaptation. The earlier RiceWatch *flood-warning verification-only* scope is **historical**, not this submission's required mission. See [canonical PRODUCT](docs/PRODUCT.md), [Isaac Sim ARCHITECTURE](docs/ARCHITECTURE.md), [rice.v1 CONTRACTS](docs/CONTRACTS.md).

**Chosen simulation engine for now: NVIDIA Isaac Sim.** Do not silently substitute MuJoCo as first choice. Confirm installed version, full compatibility/RAM/VRAM, physics and sensor APIs on the actual machine; **do not claim it works until an actual scene runs**. [RUN](docs/RUN.md#isaac-sim-preflight--chosen-target-not-yet-proven) owns first-agent startup and fallback escalation. Isaac Sim 5.1 system guidance lists 32 GB system RAM and 16 GB GPU VRAM minimum; reported 8 GB laptops may fail. No unapproved cloud spending, billable instances or open streaming endpoints.

**Build priority:** P0a one genuine actuated rover/sensor + surprise/contract; **immediately P0b physically release a seed proxy and verify ground contact**; P0c add second locally sensing robot and **shared estimated soil-quality/stand-gap map + task/route coordination**; P0d independent baseline, continuous ≤60-second video and one-page PDF. A simulation that only recovers from faults or draws soil-quality heatmaps **fails the planting mission**. Physical rescue and field flood-control/mitigation are **optional later features**, not P0 gates. Cross-domain reforestation/space examples are future hypotheses, not current demos.

**Engineering routing:** [first coding assignment](docs/RUN.md#next-agent-first-integration-task--isaac-sim-planting-first-skeleton-not-yet-implemented), [five-human owner split](docs/TEAM.md#selected-rice-project-five-human-work-split-proposed-not-yet-assigned-by-name). Only integrator edits shared v1 schema/runner; human robotics lead approves body, sensor and hopper mechanics; agents do exclusive small feature/test passes and preserve teammate work. Read-only UI may not command physics; the policy never gets hidden soil/gap truth.

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

## Order of work

1. Identify a real beneficiary, environment and observable outcome. Check closest non-LLM alternative/prior art; require actual AI-added judgement.
2. For a tiny independent fix, implement and test directly within the assigned branch/task; no new issue, planning document or approval ceremony is required. For concurrent or cross-module work, communicate owner, branch, intended paths, acceptance check and dependencies in an existing issue/PR or short human message.
3. For any shared interface change, specify fields, types, units, frame, semantics, version, errors and fixtures **before** consumers implement against it. Integrator owns shared contracts.
4. If others are working simultaneously, use a separate branch (and a separate local worktree/checkout if sharing a machine). Avoid two active writers to the same file or contract; coordinate shared changes through the integrator. Small non-overlapping solo work follows the current team branch convention.
5. Test in layers: pure logic → module contract → integration → real end-to-end episode; compare the nearest meaningful baseline. Keep deterministic random seeds and environment details.
6. Debug by reproducing, collecting exact traces/logs, locating root cause, applying the minimal correction, adding a regression test and rerunning relevant checks. No speculative patch stacking.
7. Report changed paths, commands **actually** run, results, commit/PR, unknowns, blockers and rollback path. Never claim "tested" from code reading.
8. Keep one canonical current-truth document per subject; link to it instead of copying. Git history retains earlier decisions.

## Multi-agent cooperation

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
| Get a real simulated sensor/action loop working | `.agents/skills/embodied-loop/SKILL.md` + `docs/EVENT.md` and `docs/RUN.md` |

## Gemini-first entrypoints and scoped delegation

- **Primary:** Gemini CLI imports `AGENTS.md` through `GEMINI.md`. The seven canonical on-demand skills (including `embodied-loop`) live in `.agents/skills/`; optional specialised Gemini subagents are under `.gemini/agents/`. Run `/memory show`, `/skills list`, `/agents list` to verify actual discovery when locally installed. No file automatically starts a worker.
- **Model/context routing:** follow [team coordination](docs/TEAM.md). Prefer Gemini 4 Argon for complex cross-module reasoning and Gemini 3.8 Flash for bounded specialist work, *if the team has access*. Supply the smallest relevant files, latest commit, interface and test fixture, not every chat or full repo. Use measured results, not marketing cache-hit guarantees.
- **Coordination:** GitHub issue/PR threads and owned branches/worktrees are the default shared interface. No automatic live agent messenger or global inbox. Human integrator approves priorities and merging, and each AI run must be invoked explicitly.
- **Other tooling:** Codex may follow the same neutral source `AGENTS.md` and skills. Claude-native entrypoints, agents and full skill mirrors are **retained as legacy compatibility only**, not the preferred route; existing CI currently checks their parity. Do not delete them or change shared CI without reconciling the team. Any assistant without workspace-native discovery must read relevant files explicitly.
- Do not impose multi-agent ceremony on tiny solo changes. Create independent subagents only when a bounded task actually benefits; Gemini CLI subagents do not recursively delegate.

The role/routing guidance above is for **developer agents**, not a commitment to use a particular model inside the eventual robotics product.

The existence of an agent file or automation does **not** mean any assistant has access to multiple runtimes, is executing in background, or has passed a test.
