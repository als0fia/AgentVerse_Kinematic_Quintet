---
name: evaluate-and-demo
description: Evaluate a real hackathon robotics/agent candidate against comparable baselines, report honest metrics and prepare a robust live demo and submission preflight.
---

# Evaluate and demonstrate

1. Read `docs/PRODUCT.md`, `docs/TESTING.md`, `docs/DEMO.md`, current actual rubric and exact submission deadline.
2. Agree a real-world outcome and an independent deterministic verifier. Include unsuccessful attempts and meaningful edge cases.
3. Compare candidate against a serious baseline using identical `case_id`/`seed` pairs; validate raw data using `scripts/evaluate.py`. For Robotics measure observation-driven action, recovery from one mid-run change, true local decision autonomy and safe failure; verify real simulation vs fake animation separately.
4. Examine representative failures, cost/latency, repeatability, sample size and sim-to-real gap; never infer statistical significance from tiny samples.
5. Rehearse *problem → sensor input → actual action → introduced mid-run change → independent result → limitations*. Capture a **single continuous ≤60-second** simulation take showing perception beside actions. Prepare the required one-page PDF summary and conditional five-minute pitch; label any separate fallback clearly.
6. Validate README install/run on the tested commit; seek human approval before public deployment, spending, final submission or communications.
7. Check participant portal and confirmation receipt. A finished demo/README is **not** submission.

Deliver: command/results, baseline proof, failure breakdown, true demo readiness and submission blockers.
