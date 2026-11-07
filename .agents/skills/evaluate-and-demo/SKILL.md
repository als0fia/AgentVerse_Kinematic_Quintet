---
name: evaluate-and-demo
description: Evaluate a real hackathon robotics/agent candidate against comparable baselines, report honest metrics and prepare a robust live demo and submission preflight.
---

# Evaluate and demonstrate

1. Read `docs/PRODUCT.md`, `docs/TESTING.md`, `docs/DEMO.md`, current actual rubric and exact submission deadline.
2. Agree a real-world outcome and an independent deterministic verifier. Include unsuccessful attempts and meaningful edge cases.
3. Compare candidate against serious baseline using identical `case_id`/`seed` pairs; validate raw data using `scripts/evaluate.py`. Verify real hardware/simulation vs mock separately.
4. Examine representative failures, cost/latency, repeatability, sample size and sim-to-real gap; never infer statistical significance from tiny samples.
5. Rehearse *problem → input → actual operation → independent result → surprise/edge case → limitations*; prepare tested recorded fallback explicitly labelled.
6. Validate README install/run on the tested commit; seek human approval before public deployment, spending, final submission or communications.
7. Check participant portal and confirmation receipt. A finished demo/README is **not** submission.

Deliver: command/results, baseline proof, failure breakdown, true demo readiness and submission blockers.
