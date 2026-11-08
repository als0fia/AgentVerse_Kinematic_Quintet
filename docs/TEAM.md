> **Submission-branch note (8 Nov 2026):** This document preserves historical team/agent ownership and integration instructions. The prepared release candidate is [release/agentverse-2026-submission](../README.md), **not** a claim that GitHub main has been merged. Current executable architecture, test evidence, recording steps and disclosure are in [ARCHITECTURE](ARCHITECTURE.md), [TESTING](TESTING.md), [SUBMISSION](SUBMISSION.md) and [PROVENANCE](PROVENANCE.md). Do not restart deprecated branches or deploy code from old milestone instructions in this file.

# Team ownership, parallel agents and integration

**Current user-directed product scope — Antenna, rescue-first (7 Nov 2026):** The point of the system is **risk-aware robot-to-robot physical recovery** while robots carry out useful rice-field sensing, mapping and planting. The earlier planting-first/rescue-optional instructions below record existing work, not the current primary research question. [D-0008](DECISIONS.md) remains the canonical **naming** decision; new [D-0009](DECISIONS.md) records the **scope** correction. The merged rigid-docking MJCF and `test_mujoco_rigid_docking_cam_latch_and_tandem_extraction` show a **scripted physics extraction test** (victim moves >0.08 m), but not an autonomous sensed-fault→rescue→mission-continuation episode, learned policy or live Jetson inference. [SWARM_EDGE_PLAN](SWARM_EDGE_PLAN.md) / [RESCUE_EDGE_RESEARCH](RESCUE_EDGE_RESEARCH.md).

The human team decides roles. **No named teammate assignments have been verified.** An AI agent is a worker within human-approved scope, not a new registered team member. The organiser's 3–5 human team size does not specify an AI-agent quota; no separate AI-agent cap has been verified. Use **one human-led integration path, with as many bounded specialist tasks as can be independently tested**, not one monolithic shared AI session.

**Update after merged PR #28 (7 Nov 2026):** `src/ricebot/agent/mycelial_swarm.py` now implements **simulated Policy B**: separate `LocalMycelialMap` and `LocalSlamGrid` instances for each rover, event-triggered `MycelialGossipPacket` exchanges with modelled distance/link blackout, provenance deduplication, and task bids. Tests include isolated vs connected maps, blackout recovery and changed planting target. It runs inside a shared Python/MuJoCo simulation; **no physical UWB or Wi-Fi HaLow radios, deployed distributed processes, Jetson inference or integrated autonomous physical rescue have been verified**. Policy A's `CooperativeSwarmPlanner` remains a centralised baseline. The merged PR #24 separate scripted MuJoCo rigid-docking/tandem extraction test is physical simulation mechanics, not the full sensed-rescue mission.

## Current rescue-first owner handoff

Robot/mechanics owner has merged rigid docking MJCF and scripted towing test (#24); **do not rewrite that module** without their agreement. World owner supplies reproducible immobilisation and sensor health evidence; recovery/agent owner connects PR #23's **heuristic** verdicts to a typed bounded rescue action (sole contract integrator approval); evaluator validates physical latch/displacement/second-casualty and resumed rice work. ChatGPT/Gemini roles remain interchangeable. Optional Jetson model partitioning and actual P2P messaging are separate research, not blockers for a credible rescue-first physical demonstration. Earlier planting-first work breakdown below remains a useful baseline, not the current primary mission. [SWARM_EDGE_PLAN](SWARM_EDGE_PLAN.md) / [RESCUE_EDGE_RESEARCH](RESCUE_EDGE_RESEARCH.md).

## Simulation/robot co-development and recurring component audits

Two human collaborators may exchange who uses ChatGPT or Gemini for **world simulation** versus **robot embodiment**. Do not lock the division to an AI brand. The actual source ownership follows [COMPONENTS](COMPONENTS.md): world owns field/soil/obstacles/probe rules, robot owns chassis/joints/motors/sensors/hopper, **one Isaac composition writer** loads both and implements the existing `SimulationPort`, and the kernel integrator alone changes public `rice.v1` types/runner. A physically moving mock rover lets the world developer progress before the finished vehicle, but replaceability must be revalidated for actual dimensions, control bounds, sensor mounting and landing/contact.

For substantial new feature components require the small module README, actual test evidence and replacement notes in [COMPONENTS](COMPONENTS.md). On requests to audit/streamline/split program sections, activate [module-audit](../.agents/skills/module-audit/SKILL.md) and return **critical/major/minor findings, exact evidence, test status and owner**, no unapproved refactor. Agents setting up new machines follow [workstation setup](WORKSTATION-SETUP.md); neither a mock-only test nor a local MuJoCo demo proves real Isaac behaviour.

**Five-AI progressive integration (7 Nov 2026):** For the requested parallel reconciliation across programmers' distinct simulator/GUI/agent contributions, while **P0–P5 tasks continue landing**, follow [FIVE-AGENT-RECONCILIATION](FIVE-AGENT-RECONCILIATION.md). It assigns five domain-specific AI workstreams with one shared-kernel writer, versioned per-feature source receipts, rolling latest-main intake, independent cross-review, optional low-graphics physically checked Isaac experiment and one human-controlled final integration. The [SIMULATOR-VARIANTS](SIMULATOR-VARIANTS.md) registry remains the single feature-adoption ledger. Five agents are not independent owners of five full simulators; no existing teammate branch is frozen or rewritten merely by creating the plan.

## Alternate simulator sources and human reconciliation handoff

Use the [SIMULATOR-VARIANTS registry](SIMULATOR-VARIANTS.md) for **feature lineage across branches and independent UIs/experiments**, not as a second version of the application. When substantial alternate simulation/visualizer/agent work is pushed, the author updates its source SHA and capability row, or provides the minimum record in the PR for the integrator to update; no new document per worker or per version. **One human-led integrator** chooses features based on the current [product goal](PRODUCT.md), creates a feature-by-feature adopt/adapt/park/reject plan, checks active branch owners, ports compatible behaviour through the canonical `SimulationPort`, and demands combined physics/sensor/viewer tests. The integrator does not try to merge all branch trees, delete experiments or replace `main` with an alternative app. For technical boundaries read [COMPONENTS](COMPONENTS.md) and the [latest audited risks](audits/2026-11-07-simulation-perception-viewer.md).

## Local-first development, review and integration

**Agreed operating mode (7 Nov): local coding agents own application implementation and physical runtime tests.** Each human/agent pulls the latest integrated `main` into a **local** checkout, creates a short-lived task branch/worktree, edits only owned component paths, runs focused tests and—where relevant—**runs the actually installed physics backend** (MuJoCo now, Isaac only after its adapter exists) on an authorised workstation. The worker then commits, pushes its own branch and opens a PR. `main` remains the **canonical integrated source**, not a shared live working directory or a remote coding-agent scratchpad. The human integrator reviews and merges, then reruns affected integration checks against the combined `main`; passing a worker's unit tests alone is not physical proof.

**Every substantial PR states what ran locally versus what did not:** exact commands and observed pass/fail output, host/OS and Isaac Sim version/GPU where applicable, tested commit/seed, a trace and a screenshot/short clip when physical behaviour is claimed, plus known failures, limitations and dependency changes. Link the evidence rather than committing bulky video/assets or secrets. Documentation-only and small pure-logic PRs do **not** need an Isaac video; mark the real robot test **not run/not applicable**. A green CI job is additional evidence, **not** a substitute for physical movement, sampled sensors or observed seed contact.

**Priority for the weekend:** implement **one complete MuJoCo rescue-first episode**, not another planting subsystem: observed fault and sensor-health check → validated risk/abstain decision → typed bounded rigid dock/latch skill → independently checked victim displacement, rescuer safety and resumed local fieldwork. Retain tested seed/soil/planting and Policy B gossip as existing foundation. No new simulator, winch or distributed model shards are required for P0.

**ChatGPT's current role:** advice, research, read-only code/PR audits, identifying contract/test gaps and scoped documentation maintenance. **Do not have this connected ChatGPT session push application code or make integration edits by default**; local human-directed coding agents implement/test/push features. Application-code writes need a fresh explicit user request. Documentation updates are permitted when asked, should be narrow and coordinated with active branches, and must not claim local tests or authorise final submission. The human team—not any model—owns merge, deployment, spending and submission decisions.

## One-writer ownership protocol

1. **First make one executable path work.** The integrator and team define only the shared seam needed for that path (inputs, outputs, units, failures, one fixture). Avoid premature architecture and parallel dependencies.
2. **When two or more people/agents work concurrently**, give each an owned branch, non-overlapping files/modules and a checkable outcome. On one computer, use separate Git worktrees/working directories—not two sessions changing the same checkout. Record ownership in a short message, existing issue or PR; don't make a new ticket for every task.
3. Shared contracts, schemas, common configuration and integration files need one active writer. Discuss the interface before consumer branches depend on it. When it changes, notify dependent workers and test both sides.
4. Worker branches make normal commits and PRs. The integrator merges in dependency order and reruns affected integration/demo checks. No force pushes, silent overwrites or unrelated file resets.
5. **Fast lane:** for a tiny independent solo fix, no new planning document, issue, multi-agent delegation or formal handoff is necessary. Scope it, make the change, run a focused check and report where it landed. A green module unit test is not an end-to-end pass.

## Assignment record (optional; use only what prevents confusion)

For normal parallel work, **owner + branch + touched paths + expected outcome** is enough. The extended template below is useful for changes with shared dependencies, complex testing or hardware; don't require every field for a five-minute fix.

```text
Issue / goal:
Human owner / agent role:
Branch / base commit:
Owned paths (exclusive):
Shared contract ID + version:
Inputs supplied / output expected:
Definition of done (observable):
Tests and exact run command:
Upstream dependency / integration point:
Constraints / safety / spending:
```

## Agent-to-agent handoff (PR comment or concise message)

Minimum: branch/commit, files changed, test command + observed result. Add the remaining fields only when the integration really needs them.

```text
What I changed and why:
Files changed / commit:
Contract versions touched:
Setup and execution commands used:
Tests actually run + observed result:
One real sample input/output or trace:
Reproduction instructions for failures:
Unverified assumptions / remaining risks:
Merge and integration steps:
```

## Functional hats, not artificial bureaucracy

- **Integrator** owns global build, actual end-to-end execution, schema decisions, release/presentation freeze and merges.
- **Component builder(s)** each own isolated working component plus contract fixtures/regression tests.
- **Evaluator** owns independent test scenarios, simple baseline and quantified failure report; challenges optimistic claims.
- **Demo operator** owns user story, live run readiness, video/fallback and accurate submission inventory. Hats can be shared by a five-person-or-smaller team.

## Integration heartbeat

At each natural sync, show a runnable demo or explicit blocker, not a slide. Confirm: (a) latest integrated commit, (b) whether the main journey passes, (c) most important failing contract, (d) decision needed. Issues/PRs remain the task record; do not create a growing `STATUS-v7.md`.

**Canonical branch hygiene:** `main` is the integrated source of truth, not a separate product branch. Create short-lived isolated task branches only for actual concurrent work, avoid stacked PRs unless an explicit dependency requires them, and merge prerequisites first. Never substitute a worker branch for published main instructions. After integration, read `main` back, check workflows and record the known-good SHA; retaining old branch names is not evidence work is missing from main.

## Conflict and rollback

If branches conflict: integration lead preserves both contributors' intended behaviour and reruns affected interface and end-to-end tests. If new work breaks the demo, revert the **specific** bad change on a new safe commit or switch back to known-good commit; never force/reset others' history without human approval. Keep demo-safe branch/tag only when the team actively uses it.

## Historical planting-first implementation allocation (superseded)

**Archive guard:** the following detailed Isaac/planting-first work split and Gate A–D entries document the *original* integration sequence, not the active weekend task. The current owner directions are in [the rescue-first handoff](#current-rescue-first-owner-handoff), [RUN](RUN.md#current-minimum-working-rescue-integration-assignment) and [ARCHITECTURE](ARCHITECTURE.md). Preserve named original roles and historical context rather than rebuilding those milestones or changing robot geometry unilaterally.


The first pure-Python `rice.v1` core and MuJoCo rover/seed simulation are already merged. **One human integrator** owns the shared types/runner; the existing rigid docking mechanics owner retains robot geometry. The next agent assignment is a MuJoCo *rescue skill integration*, not a new Isaac/planting-first skeleton. Keep other workstreams on narrow fixtures.

**Five human-owned workstreams** remain: (1) integrator/shared code + merge; (2) robot mechanics and hopper/seed contact in Isaac; (3) field/soil sensor proxies and physically moving scout; (4) measured map, site scoring and route/task decisions; (5) independent evaluator, baseline, read-only UI and continuous video. Use **one narrowly scoped agent task per owner at a time**, never five agents independently rewriting the architecture. Human robot owner must approve physical mechanism.

**One additional ChatGPT Plus account** can be useful for an **independent reviewing/testing session**, checking leakage of hidden soil, bogus seed-contact claims, and edge cases, rather than creating a sixth owner of shared code. Other Plus sessions and Gemini runs are not automatically connected or launched by GitHub; each human supplies current `main` SHA, exclusive paths, test and observable outcome.

**Time/complexity budget:** two rovers, one tiny existing MuJoCo rice plot, one physically detected fault, a conservative rescue-or-abstain decision, one already modelled rigid extraction and one resumed sensing/planting event. Test and report limitations before extra worlds, comprehensive rice physics, new hardware, LLM sharding or further UI work.

## Selected rice project: five-human work split (proposed, not yet assigned by name)

**PaddyPatch P0:** two physically moving rice-field robots (planter + complementary scout), real local soil/stand sample proxies, **shared estimated soil/stand map**, site selection, safe coordinated routes, and **a physically dropped seed proxy in an eligible gap**, with a surprise change and measured results. **MuJoCo now provides a tested local prototype; Isaac Sim remains the selected but unverified integration target.** Human roles below are proposals, not named people or authority to spend. The first milestone is one rover then its physical planting tool; the second embodied scout is the final cooperative target. [PRODUCT](PRODUCT.md) owns scope; [CONTRACTS](CONTRACTS.md) owns the single interface and [RUN](RUN.md) owns actual launch evidence.

| Person / owner | Own code paths after first proven fixture | What they must deliver | Important exclusions |
| --- | --- | --- | --- |
| **1 — Integrator / Isaac Sim technical lead** | `src/ricebot/contracts.py`, `runner.py`, Isaac composition/setup and shared smoke tests | verified physics host, version, wheel/actuator/sensor reset/step, `rice.v1` interface fixture, integration and repeatable run, merges | only writer of shared contract/runner; avoid cloud or remote dependency without approval |
| **2 — Human-led robot mechanics and planting tool** | `src/ricebot/robot/`, physical rover/second-body reuse, hopper/dropper and low-level safe controllers | human-approved geometry and lanes; **real physically measured seed release/landing**, collision/velocity/seed inventory checks | no unauthorised massive model training, gripper dexterity, underwater rover or gate manipulator |
| **3 — Field physics, sensing and scouting** | `src/ricebot/world/`, sensors, second scout body and event injector | small plausible prepared/drained rice scene, hidden true soil/stand quality, only **local** moisture/firmness/coverage observations, second robot producing distinct samples, unexpected obstacle/unfit-patch injection | do not send hidden-quality truth to policy or invent laboratory soil measures |
| **4 — Shared map, planning and coordination** | `src/ricebot/agent/` including map estimator and site-route allocator | fuse **two robot sample streams** into mean+uncertainty map; rank stand gaps by synthetic incremental benefit, assign physical sample/plant routes, react to surprise | begin with IDW/nearest-neighbour and A*/Dijkstra, strong baseline; do not require LLM or rewrites of contract |
| **5 — Independent grader, baseline and read-only UI** | `src/ricebot/evaluation/`, `src/ricebot/viewer/`, `tests/ricebot/evaluation/` and evidence | actual landed seed and synthetic benefit, same-seed coverage/grid/greedy comparators, map confidence, sensor/action/seed display, single ≤60 s video | evaluator sees truth, agent/UI control never does; don't polish while planting fails |

**Human mechanical gate:** robot and hopper mechanism, chassis mass/joints/clearance, safe lane width, sensor/probe contact location and actual planting release must be reviewed by person 2. Person 3 owns environment assumptions; person 1 approves software interface, not farm hardware design. Seed pellets in a **prepared/dry direct-seeded or drained model** are more plausible than placing a wheeled rover into flooded rice rows. Soil properties must be explicitly synthetic and connected to a named local sensing mechanism. A second scout without independently meaningful observations is decorative, not a cooperative achievement.

### Dependency-aware parallel plan

1. **Gate A (one integrator first):** inspect actual Isaac Sim version/hardware, obtain real physics-moving rover and sampled range sensor; seed/reset/step, bounded wheel control and mid-run obstacle reaction. This is a useful integration skeleton but *not* the rice planting product.
2. **Gate B (planting capability next):** physically actuated seed-proxy release and actual landing/contact within an eligible stand gap. Do not delay this for distributed agent design, fancy soil maps or networking.
3. **Gate C (split into independent features from latest `main`):** roles 2/3 can refine chassis/field/second rover and local probes; role 4 writes **pure map/route policy** against fixed sample fixtures; role 5 builds grader/UI against captured traces. Each task owns unique paths, exact input fixture and pass condition. Shared v1 schema changes go through role 1.
4. **Gate D (end-to-end team behaviour):** two robots collect distinct measurements, a shared estimated map changes site/route choices and the planter physically deposits seeds while avoiding crops/obstacles. Inject one change *during* the episode and log the update.
5. **Historical planting-only Gate E (superseded by rescue-first D-0009):** preserve planting evidence, but the current core demonstration needs measured fault→risk-sensitive physical recovery or safe refusal. Flood mitigation remains optional.

### How humans should prompt coding agents

**Default: one small, vertical, verifiable feature per agent pass.** Ask for actual physical result and test in owned paths, not “implement all AI” or “redo simulation and planner.” Use a separate **design pass** only for human-reviewed mechanical layout/seed-drop feasibility or shared interface. Use a separate **review/debug pass** against real trace and failing test, not a competing parallel rewrite. When an interface change affects 2+ owners, the integrator publishes a fixture/schema update first, consumers rebase, and all repeat tests.

**A good human request:** “Implement an Isaac Sim hopper releasing one rigid-body seed from `planter-1` at `site-3`; touch only robot/dropper and its tests; prove actual release/contact in a logged physics run, refuse unreachable target, and document installed Isaac version.” Another: “Fuse 2 local soil-probe samples into a 4×4 grid with explicit confidence and source IDs; test missing data and two conflicting sensors; do not query simulator ground truth.”

**Historical Isaac planting-first coding prompt** remains in [RUN](RUN.md#next-agent-first-integration-task--isaac-sim-planting-first-skeleton-not-yet-implemented) for provenance only; **do not dispatch it as current work**. The current MuJoCo rescue task is in [RUN's recovery gate](RUN.md#recovery-integration-gate-separate-from-existing-mujoco-mechanical-test). A teammate's partial PR must report actual diagnostics and never imply a passed physical rescue when only the scripted latch test ran.

## Model and context routing for human-led coding (not robot app architecture)

**Goal:** use frontier reasoning for genuine synthesis, and smaller/faster isolated contexts for bounded work. **Do not** flood a lightweight worker with an entire repository or conversation. These are recommended *roles*, not a fixed model requirement, API access grant or one agent per person.

| Job | Preferred route if accessible | Context/tools / escalation |
| --- | --- | --- |
| **ChatGPT with connected GitHub** — advice, architecture/PR audits, research and scoped documentation maintenance; application code only on explicit user request | **ChatGPT**, with a human remaining implementation/merge owner | Inspect latest GitHub branch and selected files, assess diffs, tests and local evidence; edit documentation only when asked/coordinated. **It cannot validate teammates' local Isaac run from GitHub alone and does not automatically dispatch Gemini.** |
| Hard cross-component reasoning, difficult debugging or architecture counter-review | **Gemini 4 Argon**, if entitled (API model `gemini-4-argon`); higher reasoning setting only if task justifies it | Focused brief, relevant files/contracts and actual test traces; run in an explicitly started Gemini session, never assumed to be reachable by this chat |
| Bounded code comprehension, scoped implementation, documentation lookup, narrow test proposals, short code reviews | **Gemini 3.8 Flash** (documented `gemini-3.8-flash`); LOW for light tasks, MEDIUM as needed | Only named files, diff, exact interface/test fixture; read-only tools for research/review; return a short evidence report and source/line references |
| Optional private/offline narrow tasks | **Gemma 4** where hardware and licensed local weights **really exist** | Use for bounded classification/summarisation only after local capability test; not a substitute for complex review, and no assumed installation |
| Live video, speech or presentation media | **Gemini Live / Speech, Banana, Veo etc only if selected product needs them** | Kept separate from coding-agent coordination; exact availability and API IDs require preflight |
| Physics, control loops, evaluation, safety gating | **Deterministic controller/simulator/test harness** | LLM may propose, diagnose or choose experiments, but never be the unchecked high-rate actuator |

**Execution route:** Gemini CLI is preferred when a teammate has it installed; `GEMINI.md` loads common instructions and `.agents/skills/` loads procedures on demand. Its built-in `codebase_investigator` / `generalist` and local `.gemini/agents/` definitions can have separate contexts and limited tools. CLI subagents are *not* independent teammates; official CLI docs indicate subagents cannot recursively call other subagents. Verify `/agents list`, `/skills list`, `/memory show`, the selected model, and actual available tools locally before relying on delegation. Do not assume a Gemini CLI session, remote A2A service, API key, Antigravity environment or a paid quota exists just because a file or slide names it.

**API route, when implementing a real Gemini-backed product:** prefer Google's documented Interactions API if it fits the app and its privacy requirements. `previous_interaction_id` can resume the *same* conversation, **but tools, system instructions and generation settings must be supplied again on each new call**. Never share an interaction ID across unrelated worker roles to simulate a giant shared context. The provider stores state under its service/retention terms, and implicit caching can reduce reprocessing but does **not** guarantee a 95% hit rate. Validate actual SDK versions/model IDs and cost before coding; no API calls or cloud charges are authorised by this document.

**Per-worker context packet (bounded):** goal and expected observable output; latest branch/commit; relevant 1–3 docs/interfaces and file paths; constraints and permitted tools; one test fixture; owner/contact issue or PR; time/budget limit; known uncertainties. A worker replies with **result + evidence + exact changed files/tests + blocker**, not pages of unreviewed chain-of-thought or all intermediate terminal logs. Pull additional context via search/read tools *as needed*, not blanket embedding. Escalate when a change touches shared contracts, architecture, spending or human approvals.

## Communication protocol between developer agents (ChatGPT ↔ Gemini ↔ humans)

**Coordination authority:** the human integrator chooses scope, owns approvals and merges. ChatGPT can act as an *active architect/programmer/reviewer* in its connected GitHub session, not as an always-running boss. A Gemini CLI worker must be explicitly started by a teammate or a separately authorised automation. **No current connector here can directly invoke a teammate's Gemini CLI session.** GitHub issue/PR comments are asynchronous evidence exchanges, not synchronous remote procedure calls.

**CLI entrypoint when team member starts Gemini:** use the optional [scoped implementer](../.gemini/agents/scoped-implementer.md) only in that worker's own branch/worktree, e.g. `@scoped-implementer Implement the assigned sensor adapter using contract observation.v1; limit changes to src/sensors/ and tests/sensors/; run the specified fixture and return a handoff.` Confirm local availability with `/agents list`, `/skills list`, and `/tools`; if a pinned model is unavailable, the **human** must choose a supported option. The [independent reviewer](../.gemini/agents/independent-reviewer.md) stays read-only and should be invoked on a separate review pass. These definitions are not spawned automatically.

**Example per-worker Git worktree (local machine, from a clean repo; replace placeholders):**
```bash
git fetch origin
git worktree add -b feature/my-assigned-module ../kv-my-assigned-module origin/main
cd ../kv-my-assigned-module
gemini
# In the CLI: /memory show, /skills list, /agents list, /tools
# Paste the human-approved ASSIGN; optionally invoke @scoped-implementer
```
Do **not** run two writable agents inside the same checkout; if main lacks a needed contract, the human integrator merges the prerequisite first or supplies an explicit compatible base commit. A GitHub issue may be created with the optional [agent task template](../.github/ISSUE_TEMPLATE/agent-task.md) when several workers need a durable assignment.

**Default no-orchestrator path:** if a single developer can deliver the vertical slice, do that. Parallelise only after a runnable seam has been established and two tasks can change non-overlapping files. Do not split a tightly coupled perception/control integration into multiple concurrent writers before the interface is tested.

| Stage | Sender → receiver | Minimum message and action | Stop / escalation |
| --- | --- | --- | --- |
| 1. Select the slice | Human + integrator → builders | Named beneficiary, expected simulated action, baseline, one failing-world scenario; [PRODUCT](PRODUCT.md) and [EVENT](EVENT.md) are gates | No chosen product or simulator: no speculative robot implementation |
| 2. Assign bounded work | Integrator → one developer-agent session | **Goal**, owner, **base commit**, unique branch/worktree, exclusive paths, input/output fixture, testable done criterion, limits and the issue/PR link if necessary | Two workers need same contract file: integrator resolves seam and sole writer first |
| 3. Execute | Gemini/Codex/ChatGPT worker → own branch | Read scoped docs only; make code+test changes, capture exact commands, output and failures; don't alter other worker's paths | Shared contract/API change, unsafe action, credits or deployment: stop and ask |
| 4. Block | Worker → issue/PR comment or human | **BLOCKED**: reproducible symptom; commit/seed; decision needed; 1–2 options and recommendation | No silent guesses, redundant chats or repeated polling without a human response |
| 5. Handoff | Worker → integrator, as a PR or concise comment | **READY FOR REVIEW**: branch/commit; touched paths; observed tests **with results**; representative trace; interface version; residual risks | Model assertions without a real run are explicitly **not tested** |
| 6. Review/integrate | Independent reviewer + human integrator | Review diff, check latest base, run merged contract + end-to-end scene; record current proven demo commit | **INTEGRATED** only after real checks, otherwise leave PR open; final submission requires human authorisation |

**Minimal copy/paste exchange (for parallel work, not every tiny fix):**

```text
ASSIGN — goal: ... | owner: ... | base SHA: ... | branch: ... | owned paths: ...
contract/fixture: ... | done when: ... | actual command: ... | constraints: ...
BLOCKED — branch/SHA: ... | observed failure: ... | decision needed: ...
HANDOFF — PR/SHA: ... | files: ... | commands run and results: ... | demo/fixture: ...
           contract changed?: ... | untested claims/risks: ...
```

**What travels across models:** versioned repository files and explicit contracts, one fixture, a known base SHA and a short conclusion. Do **not** transmit entire chat histories, hidden reasoning, secret keys or raw private organiser/team content. The team repository is public. For actual source contributions, use the [PR handoff template](../.github/PULL_REQUEST_TEMPLATE.md); small direct fixes may stay lightweight.

**When to call which model:** use ChatGPT for integrator-level code and reviews when it has GitHub access; route a **specific, isolated** inspection or implementation to Gemini 3.8 Flash; escalate genuinely ambiguous design/failure diagnosis to Argon after inspecting traces and baseline. Separate an independent evaluation pass from the implementing agent. The model allocation is a tested-*later* hypothesis, not proof any Gemini model is better for this project. Compare completion, code-review defects, integration rework, elapsed time, cost and tool availability across real tasks; use the smallest model meeting the verified bar. The rationale and first-party references live in [RESEARCH](RESEARCH.md#development-agent-communication-and-model-choice-evidence--7-november-2026).

**No pretending to orchestrate:** a comment on GitHub doesn't start a worker; a Gemini subagent within one CLI session is not an autonomous server; the Interactions API doesn't synchronise Git branches; passing a synthetic test isn't a live robot. Never claim multi-agent collaboration ran just because definitions exist.

## GitHub-native coordination surface (no new service required)

The repo is public and **GitHub Discussions is disabled** as observed 7 Nov; do not presume access to enable it. Use GitHub **Issues for substantial independent tasks / decisions / blocking questions**, **issue comments for focused agent-to-agent threads**, and **PRs + review comments for source changes and handoffs**. Keep each conversation attached to the relevant issue/PR, with direct links and `@mentions` only for authorised human teammates. No `chat/`, `threads/`, `inbox/`, `messages/` or per-agent running transcript folders in Git: they create merge contention, noise and accidentally public logs.

Start with an issue only when there are actual parallel workstreams, not an empty coordination board. Suggested concise issue body:

```text
Outcome / independent pass signal:
Owner (human or assigned agent role):
Exclusive paths and branch:
Input contract/fixture; dependent issue/PR:
Blocked on / approval needed:
```

For a blocked worker, comment on its issue with **decision required + options/evidence + branch/commit** and stop editing shared paths until the human/integrator resolves it. Worker PRs link the issue if one exists and use [the concise PR handoff fields](../.github/PULL_REQUEST_TEMPLATE.md); the integrator merges only after checking current branch head, contracts and combined tests. Public repo means **never post private team chat, participant credentials, keys, unpublished restricted source or individual user data** to an issue or PR. If the team later opts for GitHub Discussions or a hosted A2A/message broker, reassess based on proven bottlenecks, hosting/auth/cost/visibility, not novelty. A2A permits remote-to-remote agent calls but would require a running service and secure authentication: **not needed just to coordinate Git branches**.

**Availability:** issue/PR comments are persisted coordination records, not a live chat bus or autonomous scheduler. Each agent must be explicitly run and fetch current issue/PR state; a chat assistant cannot be a continuous background coordinator merely by being named “lead”. One named human integrator remains accountable for final merge and submission.

## Active Gemini/Codex entrypoints and retained Claude compatibility

All workers follow the same written contract and handoff template above; provider-specific agent/subagent facilities are **optional execution mechanisms**, not separate task owners. In particular:

- **Gemini CLI — primary:** `GEMINI.md` imports `AGENTS.md`, task skills are in `.agents/skills/`, and specialised read-only agent definitions can live under `.gemini/agents/`. Check `/memory show`, `/skills list` and `/agents list` in the actual installation. Main worker has a separate Git branch/worktree from any concurrent local CLI sessions.
- **Codex — optional:** reads `AGENTS.md` and `.agents/skills/` when used by a teammate; same branch, fixture, scope and evidence contract.
- **Claude — legacy compatibility, not team default:** existing `CLAUDE.md`, `.claude/agents/` and `.claude/skills/` remain intact to avoid silently breaking a teammate's setup or the present CI parity checker. **Do not configure new workflows to depend on Claude.** Remove/migrate legacy mirrors and associated CI only after ownership review and replacing the parity test; do not delete them as collateral of model routing.
- **Ordinary hosted chats:** retrieve the repo and applicable skill explicitly; never assume a local CLI, subagent, worktree or shell exists.

Pass the same exact contract version, test fixture and observed output regardless of which model was used. Do not make multiple models edit the same working tree simultaneously; isolate each concurrent assignment in a branch/worktree, review the PR and rerun integration tests after merging. The human team remains responsible for final decisions.

## Practical parallel work without ceremony

**Why:** two agents can start from the same snapshot, or one may push while another is still editing. Separate *branches* prevent direct overwrite; separate *worktrees* prevent local file collisions. A pull request plus an up-to-date-base check catches integration conflicts. None of this requires a scheduler or permanent lock service.

| Situation | Minimum safe action |
| --- | --- |
| One agent, tiny independent change | Follow team branch convention, inspect changed files, run focused tests, commit; no extra ticket/meeting |
| Two agents editing different features | Give each its own branch + scope; local CLI agents get distinct worktrees/checkouts; GitHub API agents only write their branch |
| Both need the same contract or integration file | One owner finishes the narrow contract/fixture first; others implement against that version, then integrator merges |
| Someone merged or pushed since your read | Fetch/read the latest target HEAD and relevant files; merge/rebase safely and retest. Do not overwrite with a stale snapshot |
| PR is ready | Review changed paths and actual tests, merge in dependency order, run the combined smoke/demo check; no separate status report |

### Git mechanics by execution environment

- **This ChatGPT instance using GitHub tools:** GitHub API access is not a live checkout. Read the latest branch/commit and relevant file SHAs before writing. For a multi-file update, create a tree based on the latest tree, create a commit with that branch HEAD as parent, and update **your own branch** with an expected-head/lease check when available. An API conflict/403/rejected ref means **stop, inspect, reconcile and retry**—never force-update, recreate the whole tree from a stale snapshot or silently discard teammates' files. Read back the commit/PR and checks. For a one-file contents update, use the current blob SHA supplied by the API.
- **Claude Code, Codex CLI, Gemini CLI (local):** These normally edit the working directory and may run `git` through shell tools, subject to their permissions. For concurrent sessions use separate worktrees (`git worktree add -b feature/name ../worktree-name main`) or independent clones; fetch before integration, inspect `git status` and `git diff`, commit only owned paths, and `git push` the branch. If push rejects a non-fast-forward, fetch and reconcile—never `git push --force`. Gemini's built-in worktree feature is optional and may be experimental; ordinary Git worktrees are sufficient.
- **Hosted chats other than local CLIs:** A repo link or Markdown instruction is not a shared live checkout. Give the agent explicit GitHub access or a patch, and ask for a branch/PR, verification and a readback. Do not assume it can run tests locally.

### Three short prompts humans can give agents

1. **Set the seam first:** “Own the minimal running integration path. Agree the observation/action interfaces and one test fixture with the team, then make the smallest end-to-end demo run. Don't spend time specifying modules we haven't chosen.”
2. **Parallel feature:** “Work on **[feature]** on a fresh branch; own **[paths]** only. Respect **[interface version]**, add focused tests, and open a PR with observed results. Another agent is working on **[other paths]**—don't edit them.”
3. **Integrate safely:** “Review branches **[A]** and **[B]** against the latest main. Resolve interface mismatches without discarding either feature, rerun combined tests and the real demo. Report exact commit and blockers.”

Use these when needed, not automatically before every change. **No requirements for task tickets, artificial agent hierarchies, a daily sync, global locks, a shared progress document or a pre-merge committee.** If you can't name independently editable modules yet, **don't run parallel implementation agents**; parallel research/review is still fine. Model-generated explanations and claims are not substitutes for executable evidence.
