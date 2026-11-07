# Contracts and invariants

**Contract register is empty until a system is selected.** This file is the canonical authority for inter-module messages. Don't invent robotics sensor schemas before the project requires them.

## How to add a contract

Copy this miniature specification into a new subsection for **every shared boundary**:

```text
ID/version: observation.v1
Producer/consumer: <specific module names>
Transport/API entrypoint: <file/function/route/topic>
Input/output shape: <schema or exact type path>
Fields: <names, types, required/optional, allowed range, nullable semantics>
Units/frames/timing: <explicit SI units, coordinate frames, clock semantics>
Invariants: <preconditions, postconditions, ordering, idempotency>
Failure: <typed error, retry limit, timeout, safe fallback>
Determinism: <seed, stable replay, external service variation>
Test fixture: <path and pass/fail test command>
Owner/version-change protocol: <person/issue/PR and compatibility policy>
```

## Rules that prevent integration defects

- Distinguish measured observation from inference or confidence. No model-generated claim is automatically a ground-truth measurement.
- Validate at module boundaries; reject unknown units, corrupt/partial inputs, invalid transitions and stale timestamps with a useful error. Do not hide errors by making up defaults.
- Version any breaking field change (including meaning, coordinate frame and units), and update producer + consumer + fixtures together.
- Preserve correlation IDs, episode/case IDs and random seeds through logs and metrics.
- Keep a test double or fixture for unavailable hardware/cloud models. Mark synthetic data as synthetic; passing a mock test is not a hardware result.
- Use normal type checking, JSON Schema, dataclasses, protobuf, typed TypeScript or equivalent **once the actual stack is chosen**; avoid competing parallel schemas.

## Evidence artifact contract (implemented)

The evaluation utility accepts newline-delimited JSON (`.jsonl`), one record per attempted test episode:

```json
{"case_id":"item-01","seed":1,"success":true,"elapsed_ms":125.0,"failure_reason":null}
```

Field contract: `case_id` non-empty string; `seed` non-negative integer (not bool); `success` boolean; `elapsed_ms` finite nonnegative number (milliseconds); optional `failure_reason` string or `null`. The pairing key is (`case_id`, `seed`), unique within each input file. Baseline comparison requires exactly identical keys and independently produced results. See `schemas/episode.schema.json`, `scripts/evaluate.py` and `docs/TESTING.md`. This is **evaluation metadata**, not a robot control protocol.
