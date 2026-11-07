# Testing, baseline and independent evidence

**Current tests cover only the repository scaffolding and evaluator. No project capability has yet been tested.**

## Verification ladder

| Layer | What must pass | Evidence |
| --- | --- | --- |
| L0: static | Syntax, types and schema validity (chosen stack) | Exact command and failure trace |
| L1: component | Normal case, boundary, error, timeout and invalid input | Deterministic tests and fixture |
| L2: integration | Real producer/consumer, units, frame and version compatibility | End-to-end contract tests |
| L3: product journey | One actual visible user problem solved, observed by independent checker | Command, data/seed, video/log or hardware evidence |
| L4: comparison | Strong non-LLM/product/research baseline on comparable cases | Same cases/seeds, success/failure/latency |
| L5: stress/safety | Missing sensor/API, ambiguous scene, off-nominal world, safe bounded failure | Failure category + reproduction |

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
