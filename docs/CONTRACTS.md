# Contracts and invariants

**Product direction is selected: rice-field flood monitoring, investigation and safe response. The actual simulator interfaces remain unimplemented.** This file will own versioned types and fixtures when the human integrator chooses the stack; the following are **draft boundary proposals only**, not published Python/ROS schemas.

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

## Proposed shared boundaries for first robot episode (not yet implemented)

| Boundary | Proposed producer → consumer | Candidate fields (not locked names/types) | Required constraint |
| --- | --- | --- | --- |
| **Field / incident observation** | Simulated gauge/field adapter → incident detector | episode ID, sensor ID, monotonic timestamp, position/frame, sampled water depth (m) or rate of change (m/s), sensor uncertainty | Measured/observable **not** privileged actual cause, blockage flag or true future depth; classify normal paddy water against case-specific baseline |
| **Robot observation** | Physical simulator + sensor adapter → local controller/estimator | pose estimate with frame and uncertainty, range/camera or contact measurements, estimated wheel slip, battery proxy, link status, timestamp | Reject stale/invalid sensor values; do not treat unobservable simulator truth as measured pose |
| **Incident belief and mission** | Detector/investigator/allocator → rover task selector | candidate cause and confidence/evidence provenance; inspect point and information value; urgency, safe limits, capabilities, task ID/TTL | Belief may remain uncertain; permit inspect, wait, abort and human escalation |
| **Robot action** | Approved local controller → actuated simulator | bounded velocity / steering / optional gate-contact command, frame, duration, force/speed caps, task version, safe-stop condition | Controller must not execute stale commands, travel through crop no-go regions or exceed safe collision/energy limits |
| **Outcome feedback** | Simulated sensors → observer; hidden truth → *independent evaluator only* | observed rover motion, changed water proxy if present, gate state if physically sensed, failure/retry and timestamps; separate grader pass/fail | Explicitly distinguish a physical gate actuation from an advisory recommendation; log every failed attempt |

**Human-led embodiment contract:** after choosing the physical scene and candidate robot body, establish actual geometry, wheel/track and contact dynamics, approximate mass/inertia, coordinate frames, joint identifiers, actuator force/speed caps, splash clearance, traversable surface restrictions, sensor mounting, battery/return reserve and a stop policy. Do not freeze fields or ask independent agents to code consumers until the integrator approves **one measured executable rover/sensor fixture**. An in-paddy amphibious body is a separate simulator and risk model from a bund-side inspection rover. Never claim field deployment or mechanically verified load capacities from a conceptual MJCF/URDF.

**Rescue extension:** leave room in the incident/task model for `robot_failed`/dispatch/helper outcomes, but **do not add fictional rescue payloads or force an extra worker before a real second-robot capability is selected**. Likewise, reforestation/space need separate real-world adapters, not only a renamed environment string.

## Evidence artifact contract (implemented)

The evaluation utility accepts newline-delimited JSON (`.jsonl`), one record per attempted test episode:

```json
{"case_id":"item-01","seed":1,"success":true,"elapsed_ms":125.0,"failure_reason":null}
```

Field contract: `case_id` non-empty string; `seed` non-negative integer (not bool); `success` boolean; `elapsed_ms` finite nonnegative number (milliseconds); optional `failure_reason` string or `null`. The pairing key is (`case_id`, `seed`), unique within each input file. Baseline comparison requires exactly identical keys and independently produced results. See `schemas/episode.schema.json`, `scripts/evaluate.py` and `docs/TESTING.md`. This is **evaluation metadata**, not a robot control protocol.
