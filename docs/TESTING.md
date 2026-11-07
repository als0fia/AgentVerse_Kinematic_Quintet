# Testing, baseline and independent evidence

**Current tests cover only the repository scaffolding and evaluator. No project capability has yet been tested.**

## Judge-aligned verification map — confirmed Robotics rubric

The [official weight table and source photo provenance](EVENT.md#confirmed-robotics-judging-rubric--clear-organiser-photograph-7-november-2026) are authoritative. Tests should yield **objective observations**, not fabricated scores: (a) trace every bounded action to timestamped simulated sensor data; (b) surprise the running episode and capture perceived change, replan and actual downstream effect; (c) measure bounded/local behaviour, uncertainty and optional network loss; (d) show that the **real beneficiary's most critical failure mode is handled on screen** with an evidence-supported meaningful benefit; (e) record originality relative to a serious existing controller/robot and actual implementation quality. A single polished 60-second episode does not replace repeated seeds and independent baseline comparisons. Model-generated prose cannot grade its own performance.

## Selected rice-field evaluation plan (proposal; none executed)

**Task claim to prove:** an actuated rover can interpret changing real simulated sensor measurements, investigate an abnormal water/drainage incident, adapt after a previously unannounced obstruction, and safely recommend or physically perform a bounded mitigating action. Normal shallow ponding **must not** count as a dangerous flood. If outlet actuation is not physically implemented, the result is **inspection and diagnosis**, not demonstrated mitigation.

| Scenario | Environmental surprise and observation boundary | Independent result |
| --- | --- | --- |
| **Nominal field check** | Sensor indicates abnormal rise; rover uses actual local range/depth/odometry or camera measurements to reach an inspection point; no privileged drain state | Verified movement, measured inspection and incident report |
| **Unexpected route obstruction (must run)** | After rover has begun moving, environment introduces new obstacle or slippery section **unknown to policy**; rover perceives and replans or stops safely | No collision/crop-no-go violation; recovered/abstained, latency and energy |
| **Normal ponding / false gauge** | Water stands at a plausible controlled level, or one gauge reports faulty high water while independent site check disagrees | Avoids unnecessary mitigation; uncertainty surfaced |
| **Drain blockage / gate failure (P1)** | Actual modelled drain restriction or contact-limited gate is introduced; rover must measure and **physically interact** if such skill exists | Independent flow/depth model changes or appropriately refuses; no fake mitigation from a changed UI label |
| **Damaged rover / offline control (P2)** | Only after rover P0/P1 works: local immobilisation or control-link break | Safe stop/recovery and optional genuine helper interaction, not a text-only rescue |

**Comparators on identical unseen seeds, budgets and safety bounds:** (A) fixed gauge threshold + predetermined inspection waypoint, (B) credible reactive replanner and Bayesian investigation / simple rule-based gate operation, (C) proposed active-investigation/mitigation policy, with optional slow-model ablation. Record task success with denominator, false alarms and unnecessary intervention, delay to verifying cause, contact/crop disturbance, collision/no-go breaches, local command continuity, distance/energy, simulated water-proxy improvement **only if mitigation actually runs**, and computation p95. Evaluate decisions with **hidden simulator truth only in grader**, never passed as sensor input.

**Human-led robot design acceptance:** have the team confirm selected chassis operating zone (bund versus paddy), geometry/inertia, mobility actuators, collisions, traction/clearance, sensor mounting, accessible drain/gate and safety envelopes. The first tested simulator asset must be explicitly labelled **simulation-only**, with untested hardware suitability. Include a test for a physically plausible restriction (too-narrow bund, low traction, unreachable gate or low battery) and safe abstention. Cite researched chassis trade-offs in [ARCHITECTURE](ARCHITECTURE.md), not assumed measured machine abilities.

**Go/no-go gates:** (0) physics-actuated rover and sensed motion; (1) observed surprise changes rover's action; (2) independent seeded grader and baseline; (3) true gate actuator and water response if feasible; (4) optional second rover and domain adapters. No LLM, multi-agent coordination or environment-transfer claim before those specific behaviours are working.

## Verification ladder

| Layer | What must pass | Evidence |
| --- | --- | --- |
| L0: static | Syntax, types and schema validity (chosen stack) | Exact command and failure trace |
| L1: component | Normal case, boundary, error, timeout and invalid input | Deterministic tests and fixture |
| L2: integration | Real producer/consumer, units, frame and version compatibility | End-to-end contract tests |
| L3: product journey | One actual visible user problem solved, observed by independent checker | Command, data/seed, video/log or hardware evidence |
| L4: comparison | Strong non-LLM/product/research baseline on comparable cases | Same cases/seeds, success/failure/latency |
| L5: stress/safety | Missing sensor/API, ambiguous scene, off-nominal world, safe bounded failure | Failure category + reproduction |

## Robotics judge-oriented episode tests (mandatory behavior)

Use **two connected demonstrations**, not an impressive-looking static screenshot:

1. **Nominal:** reset known seed, move a simulated robot or other embodied actor toward an actual goal using current sensor observations; log every chosen action and its observation timestamp. Verify useful effect by the simulator's independent state checker.
2. **Mid-run surprise:** before completion change **one cause** (move/block obstacle, reposition target, disable signal, change terrain). Keep recording the same continuous episode. The agent must **detect it from allowed sensor data**, change course or halt safely, and attempt recovery; compare to fixed-script baseline on an identical change.
3. **Local autonomy / degraded connectivity:** isolate or disable external model API/network where feasible, then note which decisions continue locally, which fail and how they fail. Treat a hosted simulator stream as a separate remote dependency; report both honestly.
4. **Novelty and safety:** demonstrate one hard non-tutorial scenario, record bounded actuation or action envelope, missing-sensor/timeouts and failure handling. Explain a real user's benefit and compare the nearest deterministic controller.

**Keep an actual score sheet:** success per seed (numerator/denominator), disturbance-detection delay, replan latency, collision/safety violations, time to goal, number/frequency of remote decisions, and control-loop latency percentiles. Use whatever metrics fit the **selected project**; don't invent scores. The supplied evaluator's `elapsed_ms` is **episode duration**, not per-step model/control latency. Store extra trace metrics in a project-owned fixture or structured log only after agreeing their schema.

**Ablations:** scripted path (no observation), reactive deterministic baseline, proposed agent. If an LLM is involved, separately compare with LLM reasoning disabled or replaced by a simple planner. Keep allowed sensor input separate from evaluator ground truth; don't let privileged scene coordinates leak into the candidate policy.

**Proof footage:** one **≤60s continuous take** with simultaneous sensor/perception and action view; inject a disturbance clearly during that take and preserve the run's identifying seed/commit. Rehearsal recording is not test evidence until the underlying episode and logs can be reproduced.

## Evaluation protocol

1. Write the hypothesis and actual user-relevant measure in `docs/PRODUCT.md` **before** tuning.
2. Lock the same case IDs, seeds, hardware/sim settings, metric definitions and time budgets for baseline and candidate. Independent evaluator owns ground-truth checks, not the agent being judged.
3. Record one JSONL row per attempted episode in the [episode contract](../schemas/episode.schema.json). Include failed attempts, not only successes. The runner, not an LLM's answer, decides `success`.
4. Evaluate with:

```bash
python scripts/evaluate.py --candidate PATH_TO_CANDIDATE.jsonl --baseline PATH_TO_BASELINE.jsonl
```

The tool reports sample counts, success percentages, mean/median/p95 episode latency, paired wins/losses and failure-reason counts. It **does not** prove statistical significance or real-world transfer. Synthetic tests under `tests/fixtures` verify only the evaluator.
5. Report full denominator, failure modes, cost/latency and any limitations (sim-to-real gap, cherry-picking, nonstationary models, untested hardware). Save only non-sensitive small evidence to version control; do not commit user data, secret traces or bulk checkpoints.

## Debugging recipe

Capture expected vs observed, exact build/commit and test command, seed, environment/model versions, structured trace and first failing boundary. Form one hypothesis; reproduce using a minimal fixture; change cause rather than suppressing exception; add regression; rerun at least direct and integration tests. If unavailable, state exactly what remains unverified.

## Results are not marketing prose

A model's confidence is not ground truth. A replay is not autonomous operation. A green unit test is not a hardware demonstration. An unrun command is not a passed test. Use the same factual language in README, pitch and judge submission.
