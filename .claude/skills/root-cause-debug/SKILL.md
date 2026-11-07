---
name: root-cause-debug
description: Investigate reproducible robotics or agent failures with trace evidence, hypothesis, minimal root-cause fix and regression verification.
---

# Root-cause debugging

1. Report expected vs observed, command, commit, seed, environment, model/robot/simulator versions and first failure trace. Redact keys/user data.
2. Reproduce the smallest case. Locate the first violated invariant and isolate producer/consumer/error propagation.
3. Write a specific falsifiable causal hypothesis; test it, do not stack speculative patches or mute exceptions.
4. Construct a regression test demonstrating the bug; apply the **minimal cause-level fix**.
5. Rerun regression, adjacent contract tests, and the actual user journey where possible. If an external dependency is unavailable, say so explicitly.
6. Include effect on safety, units, action bounds, latency and fallback; record in issue/PR, not a duplicate running log.

Deliver: reproduction, root cause and evidence level, corrected code, tests observed, risks/rollback.
