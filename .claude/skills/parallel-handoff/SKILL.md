---
name: parallel-handoff
description: Coordinate separately started ChatGPT, Gemini CLI and human coding workers on isolated Git branches/worktrees, GitHub issues/PRs, reviewable test evidence and no-overwrite handoffs.
---

# Parallel work and agent-to-agent handoff

1. Read the [one-writer team communication protocol](../../../docs/TEAM.md#communication-protocol-between-developer-agents-chatgpt--gemini--humans) and verify latest base HEAD. The human integrator owns final merge/decisions. **A GitHub issue/PR does not start a Gemini session; ChatGPT cannot directly command a teammate's CLI.**
2. **First build one actual sensor → decision → action vertical slice.** Parallelise only genuinely independent tasks; otherwise the integrator first freezes the smallest required typed contract + fixture in `docs/CONTRACTS.md`.
3. For each parallel task send the minimal **ASSIGN**: outcome / beneficiary relevance; owner; base SHA; unique branch and owned file paths; relevant contract, one fixture, expected test/command; permissions, time/spend limit, one stop condition. Read only relevant files. No giant conversation transcripts.
4. On one machine, give concurrent CLI sessions separate **Git worktrees**. ChatGPT GitHub-connector workers operate only their own branches, build from fresh commit/tree and use expected-head update; local git workers fetch/reconcile non-fast-forward instead of force-push.
5. **BLOCKED:** post failing command/trace/commit and one concrete decision required to the designated issue/PR; stop altering shared files while blocked. Escalate schema changes, unverified GPU credits, unsafe robot commands and public deployment/spending to the human.
6. **HANDOFF:** PR with changed paths/commit, actual test commands **and observed outcomes**, one sample trace, contract version (if relevant), limitations and next integration action. Use the [PR handoff template](../../../.github/pull_request_template.md); an unrun suggestion is never “tested”.
7. **REVIEW → INTEGRATE:** an independent reviewer checks code+impact claims and edge cases; human integrator reviews the latest base, merges deliberately, reruns combined smoke and mid-episode disturbance cases. Keep failure truthful.
8. Fast lane: a tiny independent solo fix may skip issue and delegation; follow branch convention, run its focused check and report commit. **Never** create agent-count theatre, a remote messaging server or new status diaries just for collaboration.

Deliver: concise reproducible evidence that can cross models and tool environments, not a promise that tools/agents were launched.
