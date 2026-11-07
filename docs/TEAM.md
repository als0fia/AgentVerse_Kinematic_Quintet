# Team ownership, parallel agents and integration

The human team decides roles. **No named teammate assignments have been verified.** An AI agent is a worker within human-approved scope, not a new registered team member.

## One-writer ownership protocol

1. Integration lead creates/assigns one GitHub issue per independent deliverable (or explicitly records the assignment in PR description if Issues unavailable).
2. Each agent/person receives a **named branch or git worktree**, owned paths, acceptance test, current contract version and merge dependency. Prefer different modules/files; avoid parallel modifications of the same root documents or shared schema.
3. Contract change proposal goes to integration owner **before** consumers implement against it. Freeze one integration interface for the first working vertical slice.
4. Each branch tests against the current agreed contract, includes fixture/test evidence, and submits a reviewed PR. Shared source remains integrated by a single owner in order; no force pushes or silent overwrites.
5. Review the integrated demo continuously. A green module unit test is not an end-to-end pass.

## Assignment record (create in task/PR, not a competing document)

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

## Agent-to-agent handoff (put in PR description)

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
