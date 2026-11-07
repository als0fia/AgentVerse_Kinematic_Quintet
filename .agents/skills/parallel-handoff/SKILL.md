---
name: parallel-handoff
description: Coordinate several people and coding agents working on independent feature branches/worktrees with clear ownership, immutable interface versions and reviewable handoffs.
---

# Parallel work and handoff

1. Check the newest branch HEAD and the current [team coordination rules](../../../docs/TEAM.md). For two simultaneous tasks, communicate **only** owner, branch, changed paths and a testable result; add contract/version/dependencies when the tasks share an interface. An issue is optional.
2. Start with a **working vertical slice and minimal stable seam** before splitting dependent components. Delegate only independently editable/testable features; otherwise finish the shared interface first.
3. Give local CLI agents separate worktrees/checkouts and all agents separate branches; do not let two agents edit one working tree or shared schema at once.
4. GitHub-connector agents must base their commit on the current parent SHA and update their own branch with a checked expected head (if supported). If a write is rejected or a branch advanced, re-read/rebase: never force-update or silently replace a teammate's work. Local git agents fetch and push normally; handle non-fast-forward rejection rather than `--force`.
5. Each worker reports branch/PR, files, test command and **observed** result; the integrator reviews and merges, then reruns affected end-to-end tests. Keep the handoff short and avoid separate status logs.
6. For a small independent solo fix, avoid ceremony: make the scoped change, run the focused check and report the commit. Never turn these protections into a required planning session.

Deliver: explicit task ownership + reviewable acceptance and handoffs, not theatrical multi-agent output.
