# Team ownership, parallel agents and integration

The human team decides roles. **No named teammate assignments have been verified.** An AI agent is a worker within human-approved scope, not a new registered team member.

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

## Working across Claude, Codex, Gemini and humans

All workers follow the same written contract and handoff template above; provider-specific agent/subagent facilities are **optional execution mechanisms**, not separate task owners. In particular:

- **Claude Code:** `CLAUDE.md` imports `AGENTS.md`; `.claude/skills/` exposes complete skills; `.claude/agents/` provides the existing four Claude subagents. Check discovery with `/skills` and `/agents` in the actual Claude installation.
- **Codex:** reads `AGENTS.md` and `.agents/skills/`. Invoke the matching skill via `$skill-name` if the environment supports it, or read the file directly.
- **Gemini CLI:** `GEMINI.md` imports `AGENTS.md`; `.agents/skills/` is a documented workspace skills alias. Verify with `/memory show` and `/skills list` after trusting the workspace.
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
