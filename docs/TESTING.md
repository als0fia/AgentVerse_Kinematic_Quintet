# Testing, baseline and independent evidence

**Current tests cover only the repository scaffolding and evaluator. No project capability has yet been tested.**

## Judge-aligned verification map — confirmed Robotics rubric

The [official weight table and source photo provenance](EVENT.md#confirmed-robotics-judging-rubric--clear-organiser-photograph-7-november-2026) are authoritative. Tests should yield **objective observations**, not fabricated scores: (a) trace every bounded action to timestamped simulated sensor data; (b) surprise the running episode and capture perceived change, replan and actual downstream effect; (c) measure bounded/local behaviour, uncertainty and optional network loss; (d) show that the **real beneficiary's most critical failure mode is handled on screen** with an evidence-supported meaningful benefit; (e) record originality relative to a serious existing controller/robot and actual implementation quality. A single polished 60-second episode does not replace repeated seeds and independent baseline comparisons. Model-generated prose cannot grade its own performance.

## RiceWatch P0 evaluation — required, not yet executed

**Required claim:** one *physically actuated* simulated rover can **actively verify** an uncertain rice-field water-level alarm by travelling to a useful observation point, obtaining a distinct location-dependent water measurement and reporting `verified abnormal`, `likely false alarm` or `unresolved` using only permitted sensors. During the same episode, a **new route obstruction** forces sensor-driven replanning or a justified safe stop. Ordinary managed paddy water is not automatically flooding damage. An independent grader sees hidden true water/obstacle state; the policy does not.

| **P0 required test** | What the simulator changes / policy is allowed to sense | Independent pass signal |
| --- | --- | --- |
| `nominal_true_rise` | Fixed gauge indicates abnormal rise; true *synthetic* water proxy is high at the site; rover's local water probe only samples it at an actually reached viewpoint | Rover physically navigates using range/odometry, takes independent local reading, reports verified abnormal with provenance |
| `false_gauge` | Fixed gauge drifts high; field water remains within its *defined managed range*; independent rover probe differs | Rover does **not** falsely confirm flood; reports likely false alert or uncertainty, avoids needless inspection loops |
| `blocked_inspection_route` | New obstacle appears **mid-episode**, while rover is moving; agent receives only subsequent sensor evidence | Rover changes path or safely abstains with `unresolved`, no simulated collision/no-go traversal, no oracle blockage flag |
| `sensor_unavailable` | Probe out of range or invalid; sensor timestamp stale | No fabricated reading, false confident flood diagnosis or unsafe forced action; safe unresolved escalation |
| `nominal_without_gui` | Same seed with viewer enabled/disabled | Equivalent robot decision/action trace; viewer has no authority over world physics |

**Pass threshold:** P0a physics/sensor/obstacle loop + P0b **genuinely observed local water evidence and uncertainty-aware incident report** + P0c independent same-seed comparisons and one continuous video. P0a alone is an architecture skeleton, not the claimed rice-monitoring product.

**Comparator and metrics:** (A) stationary gauge threshold (no scout); (B) capable fixed/nearest-checkpoint rover with reactive obstacle replanning; (C) candidate observation-site/diagnosis planner, with non-LLM probabilistic active-sensing baseline when feasible. Equalise sensors and action/time budget where meaningful; explicitly explain station-only baseline's lack of a rover. Track correct/false/unresolved incident reports, time to useful independent observation, unnecessary movement/inspection, obstruction response and collision/no-go events, safe holds, energy/distance proxy, latency and mission completion. If a reasoning model is used, test its removal. **No mitigation efficacy metric is required for P0.**

### P1 optional and separate — physical mitigation tests (not P0 gates)

Only after P0, and only if the human team chooses to add it: `gate_actuation` with a **real simulated joint/contact or separate field actor**; `gate_jam` refusal; **synthetic** after-action water-level proxy. Verify actual physical actuation and updated observation separately. If unfinished, **P0 is still complete**, and any mitigation is described as a proposal, not a feature. **P2** second rover recovery/cross-domain transfer also optional.

**Human physical design review:** approve rover operating zone, dimensions, wheel/contact mechanics, actual range sensor and location-dependent water probe. Gate reach and force bounds are **irrelevant until optional P1**. No actual farm hardware, crop yields, validated hydrology or real flood prevention claims.

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
