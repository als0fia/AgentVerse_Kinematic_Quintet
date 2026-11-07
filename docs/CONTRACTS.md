# Contracts and invariants — Antenna rice planting

**A narrow rice.v1 Python interface is now implemented and unit-tested; the Isaac Sim adapter and embodied product are NOT.** The human integrator freezes concrete Python types only after a real Isaac Sim rover/sensor/drop fixture, updating producers, consumers, tests and samples together. This supersedes the earlier rice-flood-alert-only `rice.v0` *draft* as the proposed product API, but preserves the **implemented JSONL evidence contract** below. Never require water-control gates, physical rescue, other worlds or LLM calls for the planting task.

## World/robot/composition seam (developer agreement)

Use [the implemented types](../src/ricebot/contracts.py) and [`SimulationPort`](../src/ricebot/runtime.py) as-is until the **single kernel integrator** approves a concrete schema delta. [COMPONENTS](COMPONENTS.md#the-actual-swap-boundary-do-not-invent-another-kernel) specifies the minimal shared fixture (IDs, metric frame/time, sample radius, motion safety limits, body clearance, sensor and hopper/release geometry). The world owner emits location-gated **observations**, the robot owner publishes physical body/actuator/sensor mappings, and the Isaac composition owner makes a real step produce the expected frame/contact. The engine holds evaluator-only soil truth: a mock rover is a substitute physical asset, **not** a fake observation/planting oracle. ChatGPT/Gemini assignments are interchangeable; package responsibility is not.

## Implemented rice.v1 Python core (actual code, NOT Isaac physics)

The current importable implementation is `src/ricebot/contracts.py`, `mission.py`, `control.py` and `runtime.py`; its contract tests are `tests/test_ricebot_foundation.py`. The *broader* types below are still proposed refinements. Shared code is owned by **one human integrator**; don't introduce parallel incompatible message formats.

| Implemented object | Boundary |
| --- | --- |
| `Site`, `Pose2D` | Metric public site coordinates, permitted/no-go area; not hidden soil ground truth. |
| `SoilSample`, `SensorKind` | Per-robot timestamped *local* 0..1 synthetic moisture/firmness/stand-gap value, measured location and validity. Measurements outside sampling radius or with reused conflicting IDs are refused; unknown is not zero. |
| `RobotObservation`, `ObservationFrame` | Estimated robot pose, physically reported front range or `None`, seed inventory and local samples from the current tick; no hidden event or future soil field. |
| `RobotCapability`, `TaskIntent`, `RobotCommand` | SAMPLE / PLANT / HOLD intents, short TTL, bounded velocity intentions and optional seed-release request only at reachable sites. |
| `PlantFeedback` | Physics adapter reports *observed* release/contact and task ID, not independently scored establishment success. Core blocks blind repeats while awaiting feedback. |
| `MeasuredFieldMap` | Deduplicated local site means, sample counts and source IDs. Not interpolation, GP mapping or calibrated uncertainty yet. |
| `SimpleMissionPlanner`, `LocalController`, `MissionCore` | Inspectable non-LLM baseline and one tick of measured-data → task → drive/release commands, with stop conditions. Not an obstacle-aware route planner or certified controller. |
| `SimulationPort.reset(seed)`, `.step(commands)` | **Unimplemented Isaac Sim adapter boundary.** It must own actual articulation, range/odometry/soil sensing and rigid-body seed drop/contact; clamp physical limits and refuse stale commands. |
| `ReadOnlyView.consume`, `EpisodeRunner.run` | Optional viewer subscriber, small runner. Mock-port tests do not prove embodiment. |

No new general swarm message bus, mitigation actuator, plant-growth model or cloud service is needed. The next owner supplies **real Isaac physics** before anyone claims the rice robot works.

## Mission-visible messages (proposed, exclusive boundaries)

| Contract | Producer → consumer | Minimum semantics |
| --- | --- | --- |
| `RobotObservationV1` | Isaac Sim sensor bridge → map, mission planner, local controller | episode, robot ID, monotonic sim time, estimated pose + frame and covariance/validity, local range/odometry, local soil/stand proxy if in sample radius, timestamps, link validity; **never privileged true position/soil-quality map** |
| `SoilSampleV1` | rover local probe → shared field map | unique sample ID, actor, `(x_m,y_m)` and spatial uncertainty, time, measured moisture proxy (unit-defined) / local firmness proxy / vegetation coverage where supported, noise/quality/valid flag, sampling radius; no hidden establishment score |
| `FieldBeliefV1` | map estimator → planner, viewer | cell/region ID, estimated soil property means + uncertainty, observed stand/gap evidence, source sample IDs, validity/age, traversability/no-go, unknown distinct from zero; no oracle quality |
| `RobotCapabilityV1` | actor registry → allocator | actor ID, ability to NAVIGATE / SAMPLE / PLANT (planter only), working hopper count, reachable seed release zone, allowed terrain and optional battery proxy |
| `TaskIntentV1` | coordinator → one rover | task ID, actor ID, mode NAVIGATE / SAMPLE / PLANT / HOLD / ESCALATE, physical target pose/site, evidence/reason IDs, issued and expiry time, expected postcondition; cannot dispatch PLANT to scout |
| `DriveCommandV1` | local bounded controller → Isaac Sim articulations | task/actor IDs, issued/expiry time, linear `m/s`, angular `rad/s` (or wheel targets with explicit units); nonfinite, stale and out-of-limit commands result in HOLD |
| `PlantCommandV1` | authorised planter controller → Isaac Sim dropper | target site, actor, seed proxy ID, issued/TTL, geometry/reach/velocity conditions, tool command; decrement inventory only if a **real physical release** occurred |
| `PlantResultV1` | physics/contact detector → estimator and grader | actual reported release, timestamp, observed landing pose/site, failure/uncertainty, remaining inventory; **success** is independently assigned against hidden site truth |
| `StepResultV1` | Isaac Sim step → runner | next sampled observations, public contact/release results, task/status; **no hidden soil/ideal planting map or upcoming event** |
| `EvaluatorTruthV1` | scene/evaluator → scorer exclusively | true synthetic soil, established-stand mask, true beneficial sites, actual seed contact placements, collisions and scenario surprises; never enters planner/actuator or UI control |

**Important realism:** simulated soil moisture/firmness at a robot's location is a **synthetic, explicitly modelled probe**, not a PhysX-native validated agronomy sensor. A camera can infer local vegetation coverage if a vision layer genuinely computes it; it cannot measure pH/nutrients or ground strength from RGB by assertion. The map must label interpolated predictions versus measured samples. A `PLANT` string is not planting: a seed/seedling proxy must leave a physically actuated hopper and contact the target ground. Physics-derived landing and inventory changes are logged separately from claims of agronomic suitability.

## Executable seam for first agent

```text
scene.reset(seed) -> per-robot RobotObservationV1
scene.step({actor_id: bounded_drive_or_dropper_action}) -> StepResultV1
map.update(real_samples_only) -> FieldBeliefV1
planner.decide(robot_observations, field_belief, actor_capabilities) -> TaskIntentV1[]
local_controller.step(observation, intent) -> bounded drive/dropper command
evaluator.score(hidden_scene_truth, recorded_actions/contacts) -> evidence JSONL
viewer.consume(read_only_scene_and_trace) -> render frame
```

**P0a** tests: Isaac Sim physically changes rover pose via wheel articulation; ray/range/contact changes after motion; new obstacle inserted mid-episode is sensed, not announced; reset seed/clock repeatability; invalid drive command safe HOLD. **P0b** tests: rover releases seed physics proxy into scene, contact/landing outcome measured; no ghost inventory decrement, no PLANT for scout, invalid site/out-of-reach rejects. **P0c**: second rover's measured local sample changes the shared map and estimated site value; two actor commands never overwrite each other or collide; different sample provenance and timestamps, no true map leakage; newly sensed soil/route surprise revises plan. **P0d**: matched-seed uniform/fixed-plan and deterministic map-informed baseline, independent outcome checks, headless/viewer parity, traces/video.

**Frames/time:** explicit world coordinate metric frame, per-robot pose estimate and uncertainty, sample radius in metres, sim seconds and strictly increasing episode-local sequence/clock. Missing/invalid is not `0`; stale messages/commands expire. Action collision/no-go/hopper checks are deterministic and cannot be overridden by a model explanation. Human designer owns robot mechanics and physical command bounds; integrator owns only shared schema; evaluators never instruct robots using hidden truth.

## Optional capability contracts — not needed to build

Only if human-approved after demonstrated P0, define `RobotRecoveryRequestV2` or `MitigationRequestV2` against real additional actors and physics. Rescue requires actual helper behaviour; flood-control requires physical gate/outlet actuation and a transparently synthetic water model. Do not create unused mandatory fields, routes or mocks for these systems in `rice.v1`. Reforestation/Antenna/space need new physical environments and validation, not just changing the name of a scenario.

## Shared-interface change rules

- One integrator owns the public contract and common runner; specialist agents propose small tested schema changes with producers, consumers and units.
- Fix evidence vs belief vs hidden simulator truth as separate data flows; prevent developer “cheats” through simulator APIs.
- Build a minimal fixture **after** a working physics scene and freeze it before five parallel agents branch. Separate actor-local controllers from mission-level planner, read-only rendering from engine.
- Keep unsupported/cloud features behind optional interfaces; no invented installed Isaac Sim version or run results.

## Evidence artifact contract (implemented)

The evaluation utility accepts newline-delimited JSON (`.jsonl`), one record per attempted test episode:

```json
{"case_id":"item-01","seed":1,"success":true,"elapsed_ms":125.0,"failure_reason":null}
```

Field contract: `case_id` non-empty string; `seed` non-negative integer (not bool); `success` boolean; `elapsed_ms` finite nonnegative number (milliseconds); optional `failure_reason` string or `null`. The pairing key is (`case_id`, `seed`), unique within each input file. Baseline comparison requires exactly identical keys and independently produced results. See `schemas/episode.schema.json`, `scripts/evaluate.py` and `docs/TESTING.md`. This is **evaluation metadata**, not a robot control protocol.
