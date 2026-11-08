# Testing, baseline and independent evidence

## Independent release verification (8 November 2026)

Branch: `fix/antenna-final-codex-verification`, derived from `52a31d1ec81a68cf3d3ad4adbbf6e600963798c6`. Local Windows Python 3.13.14 / MuJoCo 4.0.0: **142 tests passed, no skips, 112.231 s**. Repository integrity, native skill parity and whitespace checks also pass. Reproduction and launch commands are at the top of [RUN.md](RUN.md).

The fixes make explicit Mycelial selection authoritative, rebuild seeded scenes on changed-seed reset, apply the evaluator's learner to the acting controller, distinguish turning from immobilization, bound rescue actuator commands, and correct evaluation duration, site denominators, latch/mobility evidence and whole-fleet collision diagnostics. Existing golden tests now request cooperative explicitly; their physical assertions were preserved.

The continuous cooperative golden fixture (seed 7, 220 ticks) verifies latch, extraction displacement >=0.18 m, resumed self-propelled motion and two subsequent viable plantings. The high-risk refusal fixture verifies no rescue dispatch, no secondary casualty and two viable plantings; always-rescue deliberately demonstrates helper loss. Actual Windows GUI rendering, live six-rover playback/planting, pause, manual-to-autonomous control and scenario switching were observed in an isolated application window, which was then closed. Automated GUI/reset tests also passed. No other worker's window or checkout was modified.

[Recorded fleet measurements](../experiments/release_verification_20261108.json) cover 18 continuous 240-tick randomized episodes: seeds 3/8, fleets 2/4/6, scale 1.6, cooperative plus Mycelial normal/isolated communication (six matched gossip pairs). All rovers moved >0.08 m in every run; each run physically planted 3-9 viable seeds. Geometry tests verify enlarged MuJoCo boundary dimensions and identical expanded-fleet sampling/planting capabilities. The record retains actual policy, per-rover motion, extraction, collision and failure evidence.

**Remaining limitations:** two seed-3 four-rover Mycelial runs had collisions (normal and isolated communication); the seed-3 six-rover Mycelial rescue lost a helper. Many runs did not finish all viable work or recover stranded robots. Mycelial extracted victims in three expanded runs, but does not reproduce the controlled two-rover golden rescue: use the explicit cooperative demo command. The existing binary success/failure classifier retains two-rover assumptions; use physical fleet metrics, not its success rate, for expanded-field comparisons. This small study is not statistical validation. Soil/traps and radio traffic are simulated; no live model calls, hardware validation, real agronomic evidence or submission recording were made.

Latest pushed Argon comparison: `430f26e485d8738c0a0f517c48b32e1568547a95`. Only reviewed turn/stall and bounded actuator hunks were adapted; no blind merge, and its unpushed local work was unavailable. Starting upstream remained `52a31d1` at verification.

## Latest integrated evidence and limitations (8 November 2026)

**Provenance:** integrated ABCDE session/rescue baseline [PR #63](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/63); read-only [MuJoCo diagnostics #62](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/62); orchestrator safety/feedback [#69](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/69); seeded randomization [#70](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/70). The PR #48 / 78-test snapshots below are historical. [Combined `main` CI after #69 + #70](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/actions/runs/42444609370) passed **113 tests (112 passed, one GUI test skipped)** plus static repository/skill checks on commit `56e90e29`.

| Capability | Evidence checked | Important limit / missing proof |
| --- | --- | --- |
| Controlled rescue and refusal | Existing continuous golden rescue and high-risk refusal MuJoCo regression fixtures; diagnostics #62 adds read-only physics traces | Reproducible configured incidents are **not** proof of broad emergent failure physics or real-world rescue reliability. |
| Randomized Episode | #70 tests seed replay for 2 and 8 and asserts 12 seeds (1–12) produce ≥3 distinct site-visit sequences and ≥2 outcome classes including rescue and refusal | Six predetermined safe spawn archetypes plus jitter; four permitted patch sites and sampled synthetic soil. Mud immobilization is still induced by entering a preconfigured hazard radius, with wheel-friction/suction modelling; **not** a learned or geology-emergent immobilization mechanism. No broad statistical failure-rate claim. |
| Rollover stabilization | #69 tests a stranded victim's stationary HOLD, deployed spades, no latch, no peer extraction dispatch, and safety veto for incorrect model outputs | `TIP_OVER_OR_TRENCH` is not yet propagated as that failure mode by the canonical cooperative planner; no complete live rollover episode is proven. |
| Pitch-matched peer extraction | #69 checks `PITCH_NOSE_DOWN_STALL` decision, actuator setpoint, bounds and controller command generation | Canonical planner currently classifies incidents only as `NO_GO_HAZARD`, `CONTAGIOUS_MUD_TRAP`, or `ASYMMETRIC_SLIP_OR_SUCTION`. The orchestrator's nominal -0.10 rad pitch setpoint and `LocalController`'s separate +0.14 rad close-approach command are **not** identical executed actions. No end-to-end MuJoCo pitch extraction was demonstrated. |
| Reusing an existing episode session with a new seed | The GUI builds a fresh `EpisodeSession` for each run, and simulator reset regenerates seeded truth/poses | In-place `EpisodeSession.reset(new_seed)` after stepping does not also reset `MissionCore`/planner state; a fresh-session or policy reset contract is needed. Track [#71](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/issues/71). |
| Desktop GUI | Local agent report and Tkinter tests including random controls | Headless GitHub CI **skips** the Tkinter display test (`DISPLAY` unavailable). A visible GUI launch, playback and recording on the actual demonstration computer still require a real smoke test. |
| Autonomous model reasoning | Offline `DeterministicReasoner` fallback and bounded safety gate are present | Do **not** call fallback output a live LLM or claim measured benefit over a competent deterministic baseline until model-enabled matched incidents are recorded. |
| Communications / scoring | Mycelial peer-map/gossip tests and evaluator/grader summaries | Cooperative policy uses a shared map; RF setting is N/A there. Apparent 0-byte communication / 100% savings from shared-map mode must not be marketed as real radio efficiency. |
| Deployment and submission | Repo code, simulations and CI | No verified real robot, UWB/HaLow hardware deployment, Isaac equivalence, continuous final judge video, PDF or submission receipt in these tests. |

**Independent-check protocol:** after merging #69 and #70, run the complete `unittest` suite including diagnostics from #62, both fixed rescue/refusal fixtures, `test_randomized_episode_12_seed_variation_and_reproducibility`, `scripts/check_repo.py`, and the read-only diagnostics smoke test. Separately run the graphical GUI on the actual demonstration PC and preserve seed/commit/observed trace. If a check cannot be run, explicitly label it unverified rather than treating a green unrelated CI job as proof.
**Historical PR #48 milestone (7 Nov 2026, superseded by #63/#69/#70):** The single `scripts/run_mujoco_paddypatch.py --scenario golden_rescue` MuJoCo path now includes a typed `TaskMode.RESCUE`, a bounded docking/extraction command, `RecoveryFeedback` and post-rescue planting. [PR #48](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/48) merged; [combined GitHub CI](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/actions/runs/42418413738) ran **78 tests successfully**. The integrated test now checks two seed drops, maximum recorded equality-constraint force below 250 N and pitch below 0.12 rad in its fixture. **This proves the checked-in simulation/test path only:** helper survivability across hazardous traction, policy-only sensing without privileged simulator fault flags, independent multi-seed baselines, live model benefit, physical hardware and a finished submission video/PDF are *not* established by that CI.

**Current user-directed product scope — Antenna, rescue-first (7 Nov 2026):** The point of the system is **risk-aware robot-to-robot physical recovery** while robots carry out useful rice-field sensing, mapping and planting. The earlier planting-first/rescue-optional instructions below record existing work, not the current primary research question. [D-0008](DECISIONS.md) remains the canonical **naming** decision; new [D-0009](DECISIONS.md) records the **scope** correction. The merged rigid-docking MJCF and `test_mujoco_rigid_docking_cam_latch_and_tandem_extraction` show a **scripted physics extraction test** (victim moves >0.08 m), but not an autonomous sensed-fault→rescue→mission-continuation episode, learned policy or live Jetson inference. [SWARM_EDGE_PLAN](SWARM_EDGE_PLAN.md) / [RESCUE_EDGE_RESEARCH](RESCUE_EDGE_RESEARCH.md).

**Current verification layers:** repository/tooling tests, Python `rice.v1` scripted tests, **and MuJoCo 4.0.0 prototype physics tests** (`tests/test_mujoco_sim.py`). Full CI installs [pinned optional MuJoCo](../requirements-mujoco.txt) with Python 3.11+ before discovery. These results **do not establish Isaac Sim integration, field agronomy or a complete judge demo**. Report CI and locally observed results separately.

**Update after merged PR #28 (7 Nov 2026):** `src/ricebot/agent/mycelial_swarm.py` now implements **simulated Policy B**: separate `LocalMycelialMap` and `LocalSlamGrid` instances for each rover, event-triggered `MycelialGossipPacket` exchanges with modelled distance/link blackout, provenance deduplication, and task bids. Tests include isolated vs connected maps, blackout recovery and changed planting target. It runs inside a shared Python/MuJoCo simulation; **no physical UWB or Wi-Fi HaLow radios, deployed distributed processes, Jetson inference or integrated autonomous physical rescue have been verified**. Policy A's `CooperativeSwarmPlanner` remains a centralised baseline. The merged PR #24 separate scripted MuJoCo rigid-docking/tandem extraction test is physical simulation mechanics, not the full sensed-rescue mission.

## Rescue-first test matrix (new direction)

A **scripted physical rigid dock/extraction test** exists and asserts a >0.08 m backward victim shift in MuJoCo. Distinguish that result from full rescue autonomy. Integrate a true stalled robot (not sensor dropout or intentional HOLD), a risk-sensitive decision, physical dock/traction execution, actual recovery outcome and continuing soil mapping/planting. Compare no rescue, always rescue, calibrated risk-aware deterministic policy and optional contextual model on matched seeds: physical success, second helper stuck/damaged, false dispatch, travel/energy, remaining task utility and correct abstention. Model-parallel inference has its own separate real hardware failure/latency test. Keep physical safety deterministic even if a compute shard disappears. See [RESCUE_EDGE_RESEARCH](RESCUE_EDGE_RESEARCH.md).

## Component/module audit and swap verification

**Current focused handoff (7 Nov 2026):** The [version-aware simulation, perception and visualiser boundary audit](audits/2026-11-07-simulation-perception-viewer.md) pins findings against post-PR #50 source, including an urgent RescueAssessment/UI/advisor compatibility regression, simulator-oracle sensing, terminal docking control, terrain collision, and read-only viewer parity. **Recheck each finding against the actual checked-out SHA** before changing code; its source inspection is not a local MuJoCo or GUI test.



For an explicit independent audit, use [module-audit](../.agents/skills/module-audit/SKILL.md) and [COMPONENTS](COMPONENTS.md#repeatable-independent-audit-any-feature-on-demand). Check ownership, matching code/docs/interfaces and **actual tests executed versus unavailable** before declaring a component merge-ready. World/robot mock-to-real swap must revalidate real physics motion, sensor/obstacle response, safety limits, seed release/contact and no hidden truth leakage on the **selected Isaac host**; pure Python `rice.v1` tests, the [toy MuJoCo smoke](WORKSTATION-SETUP.md#c-optional-lightweight-local-physics-smoke-not-final-engine), and even the full MuJoCo prototype tests **cannot validate the unbuilt Isaac adapter on their own**.

## Judge-aligned verification map — confirmed Robotics rubric

The [official weight table and source photo provenance](EVENT.md#confirmed-robotics-judging-rubric--clear-organiser-photograph-7-november-2026) are authoritative. Tests should yield **objective observations**, not fabricated scores: (a) trace every bounded action to timestamped simulated sensor data; (b) surprise the running episode and capture perceived change, replan and actual downstream effect; (c) measure bounded/local behaviour, uncertainty and optional network loss; (d) show that the **real beneficiary's most critical failure mode is handled on screen** with an evidence-supported meaningful benefit; (e) record originality relative to a serious existing controller/robot and actual implementation quality. A single polished 60-second episode does not replace repeated seeds and independent baseline comparisons. Model-generated prose cannot grade its own performance.

## Antenna P0 acceptance — rescue first, with fieldwork regression

**P0 has passed its *automated golden-episode integration test* on `main` after PR #48 (78/78 CI tests passed).** This verifies the selected scripted fixture, including simulated fault handling, rescue command/extraction and two ground-contact seed placements; it does **not** prove the broader independent rescue acceptance matrix below. Multi-seed perturbations, independent ground-truth separation, safe refusal under difficult traction, second-casualty outcomes and a continuous judge video remain to be verified.

| Required scenario | Input / no-leak constraint | Independent pass/fail |
| --- | --- | --- |
| `nominal_rice_work` | 2 physically moving MuJoCo rovers, local timestamped soil/range sensors; real seed-proxy release | seed contacts field, two local rover data sources, safe commands; existing partial tests pass in CI |
| `missing_sensor_not_stranded` | front-range becomes invalid while rover is stationary or moving; no privileged stuck flag | **no** peer rescue dispatch solely due to missing range; planner regression test added |
| `safe_stop_not_stranded` | physical obstacle/range stop, planned HOLD or zero motor command | no false positive despite zero displacement; command history is authoritative |
| `commanded_motion_stall` | sustained issued positive wheel command, observed pose/yaw near-constant across frames, valid sensor status | bounded incident detection/triage; require genuine simulated physical stall in the final scene, not synthetic detector-only fixture |
| `rescue_vs_abstain` | same incident with safe route, high-risk helper route, mission urgency and negative control | compare never-rescue, always-rescue and risk-aware deterministic policy; demonstrate rational abstention |
| `bounded_rigid_dock_extract` | previously merged rigid cam/latch/pitch-lift; no teleport, tether or raw LLM torques | independent coupling/contact, victim displacement, force/geometry/interlock, secondary casualty and abort evidence; existing scripted mechanical test is **not** the integrated result |
| `resume_fieldwork` | after actual recovery, continued job availability and new local observations | real sampled map/seed-contact result after the incident; otherwise safe task reallocation and explicit failure |
| `surprise_and_link_outage` | mid-episode changed hazard, invalid sensors, radio blackout; hidden world values private to grader | changed bounded action, no unsafe agent command, measured latency and deterministic safe fallback |
| `view_without_control` | same seed with UI on/off, on a real graphical host | read-only view must not alter model actions or outcome; **not covered by CI's headless MuJoCo suite** |

**Fieldwork baselines still required:** original deterministic grid/coverage, single-rover planning, measured-map task/route allocation and actual physics seed-contact are reusable reference tasks, not the entire submission. Evaluate LLM context against a competent numerical rescue optimiser on matched incident states before calling the LLM essential.

**Prior milestone:** the old Isaac planting-first P0a–P0d and rice-only acceptance matrix are preserved in Git history; Isaac remains an optional later translation, not a blocker for current MuJoCo rescue integration.

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
