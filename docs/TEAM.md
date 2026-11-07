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

## Conflict and rollback

If branches conflict: integration lead preserves both contributors' intended behaviour and reruns affected interface and end-to-end tests. If new work breaks the demo, revert the **specific** bad change on a new safe commit or switch back to known-good commit; never force/reset others' history without human approval. Keep demo-safe branch/tag only when the team actively uses it.

## Gemini-first model and context routing (adopted team workflow, not app architecture)

**Goal:** use frontier reasoning for genuine synthesis, and smaller/faster isolated contexts for bounded work. **Do not** flood a lightweight worker with an entire repository or conversation. These are recommended *roles*, not a fixed model requirement, API access grant or one agent per person.

| Job | Preferred route if accessible | Context/tools / escalation |
| --- | --- | --- |
| Human-facing coordination, engineering plan, cross-module decisions, hard debugging | **Gemini 4 Argon** (documented API model `gemini-4-argon`); MEDIUM ordinarily, HIGH when complexity justifies it | Brief + decision constraints + relevant files/contracts + tool-backed repo search/test results; can invoke CLI subagents when supported. **Never** ingest all chat history by default |
| Bounded code comprehension, documentation lookup, narrow test proposals, short implementation reviews | **Gemini 3.8 Flash** (documented `gemini-3.8-flash`); LOW for light tasks, MEDIUM as needed | Only named files, diff, exact interface/test fixture; read-only tools for research/review; return a short evidence report and source/line references |
| Optional private/offline narrow tasks | **Gemma 4** where hardware and licensed local weights **really exist** | Use for bounded classification/summarisation only after local capability test; not a substitute for complex review, and no assumed installation |
| Live video, speech or presentation media | **Gemini Live / Speech, Banana, Veo etc only if selected product needs them** | Kept separate from coding-agent coordination; exact availability and API IDs require preflight |
| Physics, control loops, evaluation, safety gating | **Deterministic controller/simulator/test harness** | LLM may propose, diagnose or choose experiments, but never be the unchecked high-rate actuator |

**Execution route:** Gemini CLI is preferred when a teammate has it installed; `GEMINI.md` loads common instructions and `.agents/skills/` loads procedures on demand. Its built-in `codebase_investigator` / `generalist` and local `.gemini/agents/` definitions can have separate contexts and limited tools. CLI subagents are *not* independent teammates; official CLI docs indicate subagents cannot recursively call other subagents. Verify `/agents list`, `/skills list`, `/memory show`, the selected model, and actual available tools locally before relying on delegation. Do not assume a Gemini CLI session, remote A2A service, API key, Antigravity environment or a paid quota exists just because a file or slide names it.

**API route, when implementing a real Gemini-backed product:** prefer Google's documented Interactions API if it fits the app and its privacy requirements. `previous_interaction_id` can resume the *same* conversation, **but tools, system instructions and generation settings must be supplied again on each new call**. Never share an interaction ID across unrelated worker roles to simulate a giant shared context. The provider stores state under its service/retention terms, and implicit caching can reduce reprocessing but does **not** guarantee a 95% hit rate. Validate actual SDK versions/model IDs and cost before coding; no API calls or cloud charges are authorised by this document.

**Per-worker context packet (bounded):** goal and expected observable output; latest branch/commit; relevant 1–3 docs/interfaces and file paths; constraints and permitted tools; one test fixture; owner/contact issue or PR; time/budget limit; known uncertainties. A worker replies with **result + evidence + exact changed files/tests + blocker**, not pages of unreviewed chain-of-thought or all intermediate terminal logs. Pull additional context via search/read tools *as needed*, not blanket embedding. Escalate when a change touches shared contracts, architecture, spending or human approvals.

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

For a blocked worker, comment on its issue with **decision required + options/evidence + branch/commit** and stop editing shared paths until the human/integrator resolves it. Worker PRs link the issue; the integrator merges only after checking current branch head, contracts and combined tests. Public repo means **never post private team chat, participant credentials, keys, unpublished restricted source or individual user data** to an issue or PR. If the team later opts for GitHub Discussions or a hosted A2A/message broker, reassess based on proven bottlenecks, hosting/auth/cost/visibility, not novelty. A2A permits remote-to-remote agent calls but would require a running service and secure authentication: **not needed just to coordinate Git branches**.

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
