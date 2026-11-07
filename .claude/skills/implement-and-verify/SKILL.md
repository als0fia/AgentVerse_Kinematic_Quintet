---
name: implement-and-verify
description: Implement a scoped feature with a failing test, a minimal working change, integration verification and exact evidence rather than optimistic completion claims.
---

# Implement and verify

1. Read root `AGENTS.md`, the assigned task (issue/PR or direct request), the **relevant** product/contract docs and current tests; inspect overlapping teammate changes. Do not require a new issue or unrelated reading for a tiny fix.
2. Confirm branch/worktree and **exclusive owned paths**. Ask integrator before touching shared contract files.
3. First run an existing known-good test/smoke command. For changed behaviour, add a failing test and observe the expected failure (RED); avoid false-positive mocks.
4. Implement simplest behaviour that passes (GREEN). Refactor without moving the interface or expanding scope unnecessarily.
5. Run direct, integration and real minimal end-to-end checks when environment allows. Do not invent a hardware or model result.
6. Summarise exact commands, observed results, relevant commit, source/AI attribution, failure handling and unverified steps in the PR template.
7. Keep demo operating. If regression appears, find root cause rather than adding speculative conditionals.

Deliver: bounded diff, test evidence, interface compatibility, handoff, remaining limitations.
