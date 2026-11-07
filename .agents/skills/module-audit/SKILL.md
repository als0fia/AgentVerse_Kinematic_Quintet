---
name: module-audit
description: Independently audit a program subsystem for correctness, modularity, contract compliance, documentation, replaceability, ownership and verified physics or tests; propose focused fixes without silent refactors.
---

# Audit, streamline and split a subsystem

Use on explicit request to audit/review/streamline a program section. **Read-only by default.** Read root `AGENTS.md`, [COMPONENTS](../../../docs/COMPONENTS.md), current code/PR and the matching tests. Do not drag unrelated files into context.

1. **Pin evidence:** inspect current main SHA, active feature branch, actual source/tests and relevant requirements; report stale docs and incomplete local-only prototypes separately. Hosted model cannot claim local command execution.
2. **Reproduce:** with permitted runtime, run the smallest test then integration probe; collect full observed failure/seed/trace. Otherwise explicitly write `NOT RUN`. Do not infer a pass from code inspection, issue closure or CI unrelated to embodiment.
3. **Audit correctness:** nominal/invalid/timeout paths, stale samples, control limits, replay/reset, thread/task ownership, unexpected scene change.
4. **Audit seams:** producer/consumer API, units, coordinate frames, actor IDs, clocks, null/unknown, errors, versions; ensure a real robot can replace mock asset without world/kernel rewrite (retest geometry).
5. **Audit responsibility:** avoid duplicate schedulers, worlds, planners and write owners; inspect field vs robot vs Isaac composition vs kernel; isolate evaluators' hidden truth from planning and make viewer read-only.
6. **Audit usability:** per-component README exists **only for meaningful implemented components**; run instructions, test commands, limitations, licence and handoff match reality.
7. **Deliver findings:** severity CRITICAL / MAJOR / MINOR, exact file/line/trace, smallest fix, owner/dependency, executed vs unexecuted tests and merge-readiness `ready | conditional | blocked`. Keep business impact and judge 20% relevance requirement visible where product-facing.
8. **Escalate:** no implicit code or docs changes, no Github writes, spending, simulator install, billing or PR merges on an audit-only request. When explicit implementation is later approved, create one isolated owned PR and rerun actual affected checks.

Do not manufacture test coverage, model performance, hardware, plant success or guaranteed interoperability. A useful audit identifies real gaps without forcing a framework rewrite.
