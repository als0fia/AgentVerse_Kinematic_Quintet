# Team ownership, parallel agents and integration

The human team decides roles. **No named teammate assignments have been verified.** An AI agent is a worker within human-approved scope, not a new registered team member. The organiser's 3–5 human team size does not specify an AI-agent quota; no separate AI-agent cap has been verified. Use **one human-led integration path, with as many bounded specialist tasks as can be independently tested**, not one monolithic shared AI session.

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

## Selected rice project: five-human work split (proposed, not yet assigned by name)

**The team is building one RiceWatch flood-warning inspection-and-diagnosis demo.** Physical flood mitigation is an optional P1 extension, **not** needed by the first architecture, the five mandatory workstreams or the submission. [PRODUCT](PRODUCT.md) owns the scope, [ARCHITECTURE](ARCHITECTURE.md) owns module seams and [CONTRACTS](CONTRACTS.md) is the **only interface authority**. These roles are *ownership suggestions*, not claims the five humans have confirmed assignments or that five autonomous coding agents exist. A team of four can combine roles 1 and 5; spare capacity should help test a running demo, not invent new frameworks.

| Human role / specialty | Exclusive ownership after first seam | Deliverable and pass signal | Boundary / risk |
| --- | --- | --- | --- |
| **1 — Integrator / architecture owner** | `src/ricebot/contracts.py`, `src/ricebot/runner.py`, root package/dependencies, integration tests, `docs/CONTRACTS.md`, integration PRs | First working MuJoCo or approved fallback episode: real sensor → bounded wheel motion → new sensor; one frozen tested `v0` fixture and one run command | **Sole writer** of shared seam/runtime composition; approves code integration, not external submission |
| **2 — Physical robot design + low-level control (human-led)** | `src/ricebot/robot/`, wheeled model/MJCF, motion/safety controller and its tests | Credible rover footprint, wheel joints, range/odometry, bounded motion and stop/obstacle behavior; human approves sensor/actuator layout | Do not have an agent silently select hardware, fabricate terrain handling or build a complex manipulator. Coordinate a second local water sensor with role 3; gate only if P1 approved |
| **3 — Field simulation and sensor modelling (mitigation optional)** | `src/ricebot/world/`, irrigation scene, water-level ground truth hidden from agent, station gauge drift and rover's location-gated local water sampling, change injectors and tests. Gate/valve actor **only optional P1 after P0** | repeatable true-rise/false-gauge episodes, location-dependent rover water-level samples and a mid-episode obstacle; independent hidden truth. Gate actuation optional and separately gated | World model **never passes true cause** to agent; synthetic hydrology labelled, no real flood-performance claims |
| **4 — Mission supervisor / active sensing and planning** | `src/ricebot/agent/` and narrow planner tests | inspect vs wait vs escalate decision using `ObservationV0`; replan on changed evidence; final verified/false/unresolved report from independent water reading. **No mitigation request in P0** | No direct simulator internals, no shared contract edits, no LLM controlling servo ticks. Include ordinary planning alternative |
| **5 — Independent evaluation + demo/visualisation** | `src/ricebot/evaluation/`, `src/ricebot/viewer/`, `tests/ricebot/evaluation/`, product metrics/evidence | non-LLM hidden-truth grading, same-seed baseline, readable observed sensor + actual robot display, ≤60s continuous demo once working | UI is **read-only**; don't tune metrics to make agent win; don't spend entire event polishing graphics |

**Robot design approval:** Role 2 owns the *physical* concept (body, wheel configuration, footprint, sensor positions, traction assumptions, reach, where the rover can physically sample water; a gate actuator is a **later optional** separate design choice). Role 3 supplies feasible environment geometry. Role 1 arbitrates the *software boundary*. Human team explicitly agrees the initial robot before agents generate MJCF/geometry. Suggested minimal choice: wheeled inspection rover on an accessible paddy service bank. Physically manipulating a gate, changing water level, driving through mud and towing another rover are **not required** by P0.

### Sequential integration gates before broad parallel work

1. **Gate 0 — human product/robot check (~one short meeting):** confirm the **rover sensing location and physically feasible water-probe range**, and choose human integrator and role owners. **Do not spend the initial integration gate on mitigation mechanics.** Keep alternative robot suggestions short. Team checks available local simulator; no automatic hardware/Modal/Isaac commitment.
2. **Gate 1 — first integrator implementation (one agent, one worktree):** implement the `v0` observation, bounded control, physical rover, simulator reset/step and **one nominal + one changed-world episode**. A mock-only bus is not sufficient. Freeze one fixture and tested package/command; integrate into `main` before others branch against it.
3. **Gate 2 — focused parallel modules:** roles 2–5 each branch from **latest integrated `main`** with unique paths, the identical `v0` fixture, one observable definition of done and their own module tests. World owner must make separate true-rise and drifting-gauge cases; the agent owner can create a pure-function inspection/diagnosis policy on agreed fixture data; UI/evaluation can work from captured traces. No parallel schema rewrites.
4. **Gate 3 — integrate P0 in dependency order:** robot/controller → world sensors (including independent local water probe and false-gauge) → evidence-based incident investigator → evaluation/UI. Rerun a **full unexpected-event episode** and record current known-good SHA. The human integrator resolves any contract change. Keep a runnable fallback at each merge.
5. **Gate 4 — time-permitting only (never a submission gate):** physical outlet mitigation, remote link loss, second scout, robot rescue and cross-domain story *in that order of utility*. A complete **safe inspection and diagnosis** P0 is sufficient even without any mitigation actuator; describe optional mitigation as future work.

### How humans should direct developer agents

**Default: small, vertical, verifiable features—not repeated unrestricted passes across an entire area.** An agent should implement one bounded capability + relevant tests, present exact evidence and stop. Human owners may run a **design pass** before coding if the interface or robot geometry needs thought, and an **independent review/debug pass** after, but do not ask three agents to rewrite the same subsystem in parallel. A whole-area pass is warranted only for an explicitly assigned integrator-level refactor *after* a working seam and failing reproduction exist.

**Good assignments:** “Implement the `ObservationV0` validator, reject stale readings, add 3 exact tests; touch only `src/ricebot/contracts.py` and its tests”; “Make the MuJoCo ray sensor report actual range on a moving rover; same fixtures, show trace”; “On changed outlet evidence, choose safe INSPECT rather than blindly open the gate; test one false alarm and one true case.”

**Bad assignments:** “Build the whole AI”, “Make rice farming autonomous”, “Improve the robotics”, “Design the entire rover + planning stack + UI” or “Add a thousand general-purpose agents.”

**Standard copy/paste first assignment for the integrator** (only after a human takes ownership and verifies clean `main`):

```text
ASSIGN: Build the first executable Rice flood-monitoring robotics architecture.
Read AGENTS.md, docs/PRODUCT.md, docs/ARCHITECTURE.md,
docs/CONTRACTS.md, docs/RUN.md and the embodied-loop/contracts-first skills.
Base: latest verified main SHA. Make a new owned branch/worktree.
Own: shared v0 types, runner, minimal simulator adapter and integration tests.
Use local MuJoCo if it actually installs; report/choose an organiser-allowed
fallback with the human if not. Implement one physically moving rover, one
actual simulated range sensor, observation-driven bounded control, reset(seed),
and a mid-episode new obstacle detected through that sensor. No LLM, cloud,
water-CFD, gate manipulator or second rover in this first PR.
Deliver: exact clean-install/run/test commands and actual output; a sensor and
action trace, reproducible nominal and changed episode, v0 contract fixture;
failing cases; latency and known limits. Do not assert success without a run.
Do not edit other contributors' owned modules or merge/submit automatically.
```

**Templates for specialist agent commands:** each owner passes `base SHA + owned paths + v0 fixture + one changed-scene case + exact tests + stop conditions`, not their entire conceptual workstream. If two features collide at the contract, **one agent pauses** and asks the integrator. For robot design, ask the agent to produce **two small feasible models and tests**, then let the human choose before implementation. To reduce context, provide only the relevant `PRODUCT` excerpt, `ARCHITECTURE` boundary and `CONTRACTS` type, not private research or all chat history.

**Cadence:** each owner delivers a runnable component after each short slice. Reassign idle people to integration/evaluation instead of multiplying speculative tasks. Show exact real test outcomes and the strongest blocker at each sync. Humans own decisions about physics realism, domain safety, budget, merge and final submission.

## Model and context routing for human-led coding (not robot app architecture)

**Goal:** use frontier reasoning for genuine synthesis, and smaller/faster isolated contexts for bounded work. **Do not** flood a lightweight worker with an entire repository or conversation. These are recommended *roles*, not a fixed model requirement, API access grant or one agent per person.

| Job | Preferred route if accessible | Context/tools / escalation |
| --- | --- | --- |
| **ChatGPT with connected GitHub** — cross-component design, code editing, integration review, interface ownership | **ChatGPT** (currently available via the user's conversation/GitHub connector); a human remains merge/mission owner | Inspect latest GitHub branch and selected files; write scoped commits on owned branch with SHA lease; inspect CI and code diff. **This is API-backed GitHub editing, not a shell, robot runtime, or automatic dispatch to Gemini.** |
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
