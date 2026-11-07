# Contracts and invariants — selected rice monitoring project

**Interface proposal `rice.v0`, not yet implemented or integration-tested.** The human integrator is the **sole owner of shared contract changes** and must freeze an executable fixture before other people/agents develop producers and consumers in parallel. Architecture [here](ARCHITECTURE.md); actual product [here](PRODUCT.md). Other applications must adapt their sensors and actions behind the same *types of boundaries*, not pretend a universal rover/sensor exists.

## v0 seam: simulator, supervisor, robot controller and UI

**Suggested Python dataclasses/protocols for first coding agent** (actual names/type paths become authoritative when code and tests exist, not this unexecuted documentation):

| Contract | Producer → consumer | Minimum fields and constraints |
| --- | --- | --- |
| `ObservationV0` | simulator sensor sampler → estimator/supervisor/controller | `episode_id: str`, `seq: int`, `sim_time_s: float` (monotonic), `robot_id: str`, `robot_status`, `pose_estimate_xy_yaw: [float,float,float]?` (odometry estimate, not true pose), `sensor_samples: list[SensorSampleV0]`, `link_ok: bool`; optional `battery_fraction: float?` only if simulated |
| `SensorSampleV0` | simulation adapter → perception | `sensor_id`, `kind` (`front_range`, `water_level`, `wheel_odometry`, etc.), `source_actor_id`, `sample_time_s`, `value`, `unit`, `valid`, `location_id?`, `confidence?`; no arbitrary hidden fault labels or true water map |
| `TaskIntentV0` | supervisor/planner → local controller/actor registry | `episode_id`, `issued_at_s`, `mode` = `INSPECT` / `HOLD` / `REQUEST_MITIGATION` / `ESCALATE`, optional `target_site_id`, reason/evidence IDs, `expires_at_s`; `REQUEST_MITIGATION` is **future v1 and must fail closed while unsupported** |
| `DriveCommandV0` | local safety/controller → simulator actuator adapter | `episode_id`, `issued_at_s`, `ttl_s > 0`, `linear_m_s` and `angular_rad_s` finite within human-agreed physical speed/turn limits; `HOLD` maps to zero bounded control |
| `StepResultV0` | `sim.step(command)` → runner | **next** `ObservationV0`, `done: bool`, optional safe failure code and public status; **no environment truth or hidden injected event in policy-visible return** |
| `RenderPacketV0` | runner/sim renderer → UI, read-only | RGB/depth/scene view when available, observation subset, selected intent, timestamp and status. Visual overlay may optionally show *evaluator-only annotations for judges* but never feed them back to agent |
| `EvaluatorTruthV0` | simulator evaluator → independent scoring only | world blockage, true water proxy state, geometry/collisions, intervention effect and scenario event schedule; never exported to the planner, sensor adaptor or policy |

**Spatial/temporal rules:** metres, seconds, radians, `m/s` and `rad/s`, explicit world/body frames, one episode-local simulation clock and monotonic observation sequence numbers; `yaw` angle range and coordinate frame documented in the chosen engine. A reading of 0 is **not** the same as unavailable; use `valid=false`/`null`. All commands must include freshness/TTL, may be refused with typed errors, and always end in a safe zero-velocity fallback on expiry or missing sensor. Battery/flow are *synthetic proxies* until implementation and calibration are verified.

**Sample only — illustrative fixture, not measured output:**

```json
{
  "episode_id": "rice-demo-007",
  "seq": 12,
  "sim_time_s": 6.0,
  "robot_id": "scout-1",
  "robot_status": "ACTIVE",
  "pose_estimate_xy_yaw": [0.8, 0.1, 0.0],
  "sensor_samples": [
    {
      "sensor_id": "front-ray",
      "kind": "front_range",
      "source_actor_id": "scout-1",
      "sample_time_s": 6.0,
      "value": 0.45,
      "unit": "m",
      "valid": true
    },
    {
      "sensor_id": "drain-probe",
      "kind": "water_level",
      "source_actor_id": "station-outlet",
      "sample_time_s": 5.9,
      "value": 0.33,
      "unit": "m",
      "valid": true,
      "location_id": "outlet"
    }
  ],
  "link_ok": true
}
```

**The fact that the station reports an alarming level is a *measurement*, not simulator truth.** Fault scenarios must be able to return a drifting station measurement while the actual level differs. A scout's probe must only measure when it genuinely reaches the modelled sensing area.

## v0 call sequence and first fixtures

1. `sim.reset(seed: int, scenario_id: str) -> ObservationV0`; deterministic initial robot/world state and sensor readings.
2. `supervisor.decide(observation) -> TaskIntentV0`; initial implementation may choose `INSPECT(site)` or `HOLD`, with explicit provenance and an optional simple Bayesian alternative.
3. `local_controller.step(observation, intent) -> DriveCommandV0`; bounded velocity and collision/range checks even if supervisor unavailable.
4. `sim.step(drive_command) -> StepResultV0`; advances physics, returns **new** sensor readings. `scenario` alone injects surprise into world physics and is invisible to planner. `renderer.consume` is optional and read-only.
5. `evaluator.score(episode trace, hidden simulator truth) -> episode JSONL`; independent pass/failure and metrics. Existing `scripts/evaluate.py` accepts the final metadata records, not motor commands.

**Minimum v0 tests:** `reset` same seed reproducibility; real `step` causes physical pose/sensor change; stale/NaN/out-of-bounds action refusal; sensor drop is invalid rather than zero; new obstacle only discovered via range; GUI disabled produces equivalent action trace; no hidden truth passed into `supervisor.decide`; local HOLD works without station/model; same-seed baseline pairing. If the simulator or robot geometry forces an interface adjustment, the **integrator changes one versioned contract and updates producers, consumers and tests together**.

## v1 extensibility (design only; explicitly out of v0)

- **FieldActorCapabilities**: `actor_id`, allowed `inspect`/`open_gate`/`assist` actions, position/safety limits, energy and freshness. Avoid permanent hard-coded `if rice` conditionals inside the planner.
- **MitigationRequestV1**: target actor/site, requested bounded gate movement, expiry, authorisation and expected observation for verification. The physical actuator must change its simulated joint; simplified water proxy responds *through the world model*, not via the agent rewriting its success state.
- **RecoveryRequestV1**: failed robot ID, observed symptoms, local self-recovery history and constraints; helper allocation and any simulated physical assistance require a genuine second actor. Dispatch and towing are not synonymous.
- **Link partition**: station may go away, but local controller remains independent and can hold/retreat; all rejoined reports are time-stamped and stale assignment commands rejected.

## Shared-interface change protocol

**Only the integrator edits this document or `src/ricebot/contracts.py` during parallel development.** Others propose a small PR/issue delta containing field name, data type, units/semantics, producer and consumer, fixture, migration and failure handling. Never silently add required fields or move simulator truth into policy inputs. Human robot designer approves actuator capability and geometry changes. Frontend/UI and evaluator cannot dictate control interfaces by using hidden world state.

## Rules that prevent integration defects

- Distinguish measured observation from inference or confidence. No model-generated claim is automatically ground truth.
- Reject unknown units, corrupt/partial inputs, invalid transitions, stale timestamps and forbidden actor commands. Do not hide failures with guessed defaults.
- Version breaking semantic changes and update both sides, fixtures and tests; keep episode IDs, seeds, robot IDs and source timestamps through logs.
- Keep test doubles for unavailable hardware/cloud models. Mark synthetic data as synthetic; mock tests are not hardware or physical simulation results.
- Use Python dataclasses / Protocol / validation for a light first implementation; avoid two concurrent JSON/pydantic/protobuf schemata without an actual need.

## Evidence artifact contract (implemented)

The evaluation utility accepts newline-delimited JSON (`.jsonl`), one record per attempted test episode:

```json
{"case_id":"item-01","seed":1,"success":true,"elapsed_ms":125.0,"failure_reason":null}
```

Field contract: `case_id` non-empty string; `seed` non-negative integer (not bool); `success` boolean; `elapsed_ms` finite nonnegative number (milliseconds); optional `failure_reason` string or `null`. The pairing key is (`case_id`, `seed`), unique within each input file. Baseline comparison requires exactly identical keys and independently produced results. See `schemas/episode.schema.json`, `scripts/evaluate.py` and `docs/TESTING.md`. This is **evaluation metadata**, not a robot control protocol.
