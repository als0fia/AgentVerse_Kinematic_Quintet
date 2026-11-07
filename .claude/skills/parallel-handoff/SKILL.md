---
name: parallel-handoff
description: Coordinate several people and coding agents working on independent feature branches/worktrees with clear ownership, immutable interface versions and reviewable handoffs.
---

# Parallel work and handoff

1. Read `docs/TEAM.md`, root AGENTS, current issues/PRs and chosen product/contract. Verify actual write permissions for every worker.
2. Have the human/integrator assign **exclusive paths** and one branch/worktree per independent task. Do not run parallel edits on the same root configuration or schema.
3. Give each worker a task envelope: acceptance test, owner, exact files, contract version, fixture, dependency and completion artifact.
4. Prioritise early end-to-end integration before distributing more features. Integrator merges branches in dependency order.
5. Demand PR handoff: changed files/commit, exact commands + results, relevant trace, known failures, assumptions and next integration step. A role prompt isn't evidence an agent did work.
6. If branches conflict, preserve both intentions, resolve in integrator branch and rerun contract and end-to-end checks; never force-push over team work.

Deliver: explicit task ownership + reviewable acceptance and handoffs, not theatrical multi-agent output.
