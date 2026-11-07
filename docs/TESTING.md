# Testing, baseline and independent evidence

**Current user-directed product scope — Antenna, rescue-first (7 Nov 2026):** The point of the system is **risk-aware robot-to-robot physical recovery** while robots carry out useful rice-field sensing, mapping and planting. The earlier planting-first/rescue-optional instructions below record existing work, not the current primary research question. [D-0008](DECISIONS.md) remains the canonical **naming** decision; new [D-0009](DECISIONS.md) records the **scope** correction. The merged rigid-docking MJCF and `test_mujoco_rigid_docking_cam_latch_and_tandem_extraction` show a **scripted physics extraction test** (victim moves >0.08 m), but not an autonomous sensed-fault→rescue→mission-continuation episode, learned policy or live Jetson inference. [SWARM_EDGE_PLAN](SWARM_EDGE_PLAN.md) / [RESCUE_EDGE_RESEARCH](RESCUE_EDGE_RESEARCH.md).

**Current verification layers:** repository/tooling tests, Python `rice.v1` scripted tests, **and MuJoCo 4.0.0 prototype physics tests** (`tests/test_mujoco_sim.py`). Full CI installs [pinned optional MuJoCo](../requirements-mujoco.txt) with Python 3.11+ before discovery. These results **do not establish Isaac Sim integration, field agronomy or a complete judge demo**. Report CI and locally observed results separately.

**Update after merged PR #28 (7 Nov 2026):** `src/ricebot/agent/mycelial_swarm.py` now implements **simulated Policy B**: separate `LocalMycelialMap` and `LocalSlamGrid` instances for each rover, event-triggered `MycelialGossipPacket` exchanges with modelled distance/link blackout, provenance deduplication, and task bids. Tests include isolated vs connected maps, blackout recovery and changed planting target. It runs inside a shared Python/MuJoCo simulation; **no physical UWB or Wi-Fi HaLow radios, deployed distributed processes, Jetson inference or integrated autonomous physical rescue have been verified**. Policy A's `CooperativeSwarmPlanner` remains a centralised baseline. The merged PR #24 separate scripted MuJoCo rigid-docking/tandem extraction test is physical simulation mechanics, not the full sensed-rescue mission.

## Rescue-first test matrix (new direction)

A **scripted physical rigid dock/extraction test** exists and asserts a >0.08 m backward victim shift in MuJoCo. Distinguish that result from full rescue autonomy. Integrate a true stalled robot (not sensor dropout or intentional HOLD), a risk-sensitive decision, physical dock/traction execution, actual recovery outcome and continuing soil mapping/planting. Compare no rescue, always rescue, calibrated risk-aware deterministic policy and optional contextual model on matched seeds: physical success, second helper stuck/damaged, false dispatch, travel/energy, remaining task utility and correct abstention. Model-parallel inference has its own separate real hardware failure/latency test. Keep physical safety deterministic even if a compute shard disappears. See [RESCUE_EDGE_RESEARCH](RESCUE_EDGE_RESEARCH.md).

## Component/module audit and swap verification

For an explicit independent audit, use [module-audit](../.agents/skills/module-audit/SKILL.md) and [COMPONENTS](COMPONENTS.md#repeatable-independent-audit-any-feature-on-demand). Check ownership, matching code/docs/interfaces and **actual tests executed versus unavailable** before declaring a component merge-ready. World/robot mock-to-real swap must revalidate real physics motion, sensor/obstacle response, safety limits, seed release/contact and no hidden truth leakage on the **selected Isaac host**; pure Python `rice.v1` tests, the [toy MuJoCo smoke](WORKSTATION-SETUP.md#c-optional-lightweight-local-physics-smoke-not-final-engine), and even the full MuJoCo prototype tests **cannot validate the unbuilt Isaac adapter on their own**.

## Judge-aligned verification map — confirmed Robotics rubric

The [official weight table and source photo provenance](EVENT.md#confirmed-robotics-judging-rubric--clear-organiser-photograph-7-november-2026) are authoritative. Tests should yield **objective observations**, not fabricated scores: (a) trace every bounded action to timestamped simulated sensor data; (b) surprise the running episode and capture perceived change, replan and actual downstream effect; (c) measure bounded/local behaviour, uncertainty and optional network loss; (d) show that the **real beneficiary's most critical failure mode is handled on screen** with an evidence-supported meaningful benefit; (e) record originality relative to a serious existing controller/robot and actual implementation quality. A single polished 60-second episode does not replace repeated seeds and independent baseline comparisons. Model-generated prose cannot grade its own performance.

## Existing planting/fieldwork evaluation — supporting tests, not current rescue-first P0

**Historical planting-first test objective:** a physically driven planter drops seed proxies into valid missing-stand patches; a second scout adds distinct soil/stand measurements. These tests preserve useful mission context, **not** the primary current rescue-first proof. Both plan around physically modelled terrain/obstacles and an injected mid-episode surprise. A policy sees only local sensors, timestamps, estimated poses and the shared belief map—not the true suitability, contact success precomputed by the world, or surprise flag. Score **physically measured seed landings** separately from synthetic establishment quality.

| Test case | Scenario and permitted sensing | Grader acceptance |
| --- | --- | --- |
| `rover_range_motion` (first integration) | one wheel-driven rover; actual ray sensor; new obstacle while moving | real sampled sensor changes → actual different bounded wheel commands or safe hold, not UI animation |
| `physical_seed_drop` (next mandatory capability) | planter reaches accessible gap, hopper releases rigid-body seed proxy | released seed leaves tool, physics/contact shows landed cell; inventory reduces only after real release, out-of-reach PLANT refused |
| `two_robot_soil_sampling` | two actuated rovers measure **different local** synthetic moisture/firmness/stand indicators | each sample timestamped, with source actor and physical sampling radius; no hidden-world oracle; merging samples changes shared estimated map |
| `heterogeneous_gap_sites` | missing-stand cells vary in synthetic suitability; some should be skipped to avoid seed waste/crop trampling | site selector prioritises plausible positive marginal benefit and respects known no-go/seed budget; can abstain |
| `cooperative_route_allocation` | shared survey/plant tasks and obstacles | assigns compatible sampling/planting tasks, avoids duplicate sample and rover collision; actual planner actions change robot motions |
| `changed_site_or_blocked_route` | route obstruction or surprising low soil reading mid-episode | fresh sensor → new belief/route/site → real changed physical travel and later valid drop or justified safe abort |
| `baseline_replay` | identical seeds/world, budget and measured observations, comparable plans | scores uniform/grid, capable deterministic sampled-map+route planner and optional agent; report seeds, failures, variance |
| `headless_viewer_parity` | same run with viewer on/off | same simulator sensor/action/seed-contact trace (where deterministic), no UI-to-physics control |

**Current rescue-first P0 gates:** genuine immobilisation vs missing sensor/intentional HOLD; a safety-checked rescue-or-abstain decision; typed helper actuation driving real simulated docking/tow; independently measured movement, helper outcome and continued useful work. MuJoCo is the present integrated engine; Isaac is not a current prerequisite. The table above remains a secondary field/seed regression suite, not a new simulator or second product objective.

**Primary rescue comparisons:** no rescue; always dispatch; numerical risk-aware dispatch with the same tested rescue skill; optional contextual/tool-using supervisor. Compare extraction success, collateral loss, correct refusal, response cost and mission continuation on matching incident seeds and observations. The older planting and Policy A/B comparisons remain secondary tests; do not substitute them for rescue results.

**Primary measures:** true/false immobilisation detection, dock/contact and actual victim extraction, helper stall or second casualty, crop/no-go damage, rescue delay/energy, safe abstention and useful work resumed. **Secondary:** physical seed contact, local sensor provenance, field coverage and synthetic establishment proxies. No seed landing implies germination, and no simplified MuJoCo friction implies validated real mud suction. The externally reported 12/12 adaptive technique benchmark is not a checked-in reproducible test on `main`; do not quote it as a verified Antenna result.

### Optional scenarios — NOT part of current rescue-first P0 or required video

**Additional crop/flood scenarios:** suspect blocked drainage → additional inspection; only count intervention if a physically actuated gate/outlet and a visibly labelled simplified water model are implemented and verified. Reforestation/space are transfer hypotheses, not extra required simulated scenes.

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
