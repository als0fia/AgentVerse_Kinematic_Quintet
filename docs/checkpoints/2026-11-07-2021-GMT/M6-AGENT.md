# M6 — Agentic Supervisor & AI-Necessity Examination (`M6-AGENT.md`)

**Status:** READ-ONLY AUDIT + EXECUTED LOCAL MUJOCO 4.0.0 VERIFICATION (static code review + GitHub CI metadata + cited literature + local Windows 11 / Python 3.13.14 / MuJoCo 4.0.0 test & benchmark execution).  
**Recorded:** 7 November 2026, `20:21–20:42 GMT` checkpoint.  
**Reference team `main`:** [`6c3a5752d73c1a1a2ed42479aa919e5f82415f7d`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/commit/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d) (also checked against subsequent `main` merge [`82099e0`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/commit/82099e0) PR #37).  
**Pending proposal inspected:** [PR #32](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/32), head `6175e09e7c3a28cfe17c40b2988cc772153d071d` (documentation only), plus branch `origin/docs/rescue-policy-math-and-improvisation` (`9c61046bd4e5ce8339050a097d08551a8d5c63b7`).  
**Ownership:** Independent audit; do not change shared application code without the human integration owner's assignment. All source links below point to the pinned snapshot unless labeled PR.

## 1. Decision summary (under 250 words)

**Verdict:** Antenna has a credible **role** for an event-driven, multi-step rescue supervisor, but **does not yet implement that role end-to-end on `main`**, and an LLM's incremental benefit over a strong deterministic risk-aware planner is unproved. Present code provides a hand-weighted triage, optional synchronous Gemini site-assignment call, simulated per-rover gossip, and a separate scripted mechanical extraction test. Their interfaces do not yet constitute an autonomous sensed-failure → evidence-gathering → safe physical rescue → monitored outcome → returned-to-work loop.

**Highest-value next action (human integrator + agent owner, world/mechanics owners supporting):** produce a real typed **incident / bounded recovery skill / observed outcome** seam connecting the existing MuJoCo world and rigid dock. First run it with deterministic evidence gathering and risk-aware abstention; then add an *optional*, tightly validated tool-loop model to the **same runtime**. This resolves more rubric risk than adding a new model stack. Prioritize distinguishing missing range data from true immobilisation, exempting active rescue docking from the `1.0 m` peer-proximity reflex cutoff (`control.py` L61–70), and forbidding an optional advisor from overriding a safety `HOLD`.

**Proposed differentiator:** adaptive investigation of ambiguous teammate failure and context-sensitive tradeoffs, with a physical counterfactual: a helper discovers that the first plan is unsafe or stalled in high-suction clay, changes its plan or refuses, and either recovers a peer or measurably preserves fieldwork/avoids a second loss.

**Demo-blocking:** no demonstrated integrated rescue skill in `main`. **LLM not demo-blocking:** keep an honest autonomous deterministic rescue/abstention path if live model inference is offline.

## 2. Capability and evidence ledger

| Subsystem | Current level | Code evidence | Runtime proof observed | Missing to claim product |
| --- | --- | --- | --- | --- |
| Homogeneous mapping/plant task bids, Policy A | **tested isolated** (CI + local MuJoCo); centralised policy | [agent/planner.py L193–359](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/agent/planner.py#L193-L359), [test_agentic_swarm.py](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/tests/test_agentic_swarm.py) | Main CI + local `unittest` (`72/72 OK`) | Join to rescue skill and physical success |
| Rescue triage value score (`RESCUE_NOW`/`DEFER`/`ABANDON`) | **tested isolated** with scripted frames | [planner.py L364–523](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/agent/planner.py#L364-L523) | Scripted triage assertions passed | Measured failure diagnosis and conditional risks |
| Physical robot-to-robot latch/extraction | **tested isolated** by scripted MuJoCo test | [test_world_and_robot.py L135–179](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/tests/test_world_and_robot.py#L135-L179) | Local MuJoCo 4.0.0 (`>0.08 m` victim shift) | True immobilisation and runtime skill invocation, after-action verification, mission continuation |
| Actual rescue dispatch | **contradicted** if advertised as physical rescue | [planner.py L683–699](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/agent/planner.py#L683-L699): helper receives `SAMPLE` task at nearest *site* | No recovery-typed action on `main` | Recovery intent and executor |
| Model-based supervisor | **code only** (one-shot optional API verdict); real model invocation **unknown** | [planner.py L88–190](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/agent/planner.py#L88-L190) | Mock advisor tests only; no live API latency/costs | Tool-loop execution and measured added value |
| Safety and motor limits | **tested isolated** (`contradicted` for `<1.0 m` rescue docking) | [control.py L47–102](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/control.py#L47-L102), [test_sensor_reflexes.py](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/tests/test_sensor_reflexes.py) | Local unit tests pass, but `min_peer_distance_m=1.0` blocks `0.065 m` docking | Enforced recovery corridor / rescue exemption |
| MuJoCo observation and planting | **working end-to-end** *(planting runtime)* | [adapter.py](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/mujoco_sim/adapter.py), [test_mujoco_sim.py](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/tests/test_mujoco_sim.py) | Local headless run: `viable_seed_placements: 2` in `7.936s` | Same scene must supply incident and recovery outcome |
| Simulated Policy B per-rover maps/gossip | **working end-to-end** in same Python simulation | [mycelial_swarm.py](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/agent/mycelial_swarm.py), [test_mycelial_swarm.py](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/tests/test_mycelial_swarm.py) | Local `unittest` (`4/4 OK`), no radio hardware | Temporal validity of relayed health/terrain beliefs |
| Distinct RiceSwarm AI lifecycle/Gemini adviser | **code only / separate experiment** for this integration | [riceswarm_ai/ai_agent.py](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/riceswarm_ai/ai_agent.py), [core.py](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/riceswarm_ai/core.py#L595-L743) | `14/14` isolated tests, not integrated with `ricebot` | Reuse ideas/tests, not second runtime |
| Actual offline edge LLM, real P2P, model-parallel Jetsons, trained rescue policy | **missing/unverified** | [RESCUE_EDGE_RESEARCH](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/docs/RESCUE_EDGE_RESEARCH.md) | None | Separate future hardware study |

## 3. Findings, root cause and smallest safe corrections

**F1 — BLOCKER: rescue triage does not authorize physical rescue.** The helper's `RESCUE_NOW` assignment is a `SAMPLE` task targeting the nearest eligible field site, not the immobilised robot, a docking target, or a bounded rescue skill ([planner.py L683–699](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/agent/planner.py#L683-L699)); the existing task modes are `SAMPLE`/`PLANT`/`HOLD` ([contracts.py L145–188](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/contracts.py#L145-L188)). The scripted physical test directly writes MuJoCo actuator controls ([test_world_and_robot.py L135–178](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/tests/test_world_and_robot.py#L135-L178)). **Repro condition:** any rescue-now scripted frame; inspect emitted `TaskIntent` mode. **Fix:** integrator adds recovery request/result seam, robot owner binds bounded mechanism in same MuJoCo runner, evaluator asserts position/attachments and continued work. **Rubric:** perception/action, adaptation, ambition.

**F2 — HIGH: invalid range is equated with immobilisation.** The contract explicitly defines `front_range_m=None` as absent/invalid, yet [planner.py L364–390](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/agent/planner.py#L364-L390) marks such a robot stranded immediately. The test uses `None` as its stuck signal ([test_agentic_swarm.py L132–180](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/tests/test_agentic_swarm.py#L132-L180)). **Repro:** one healthy-moving rover with missing front range, retained odometry and drive response; false rescue dispatch remains possible. **Fix:** distinguish `SENSOR_DROPOUT_HOLD`, `INTENTIONAL_HOLD`, and `S1_IMMOBILIZED`; require commanded-positive vs measured-progress residual (`slip > 0.85` at `v_cmd >= 0.10`) or explicit `is_bogged` evidence. First response to ambiguous sensor fault should be `HOLD`/`INVESTIGATE`, not a rescue diagnosis. **Rubric:** perception, operational safety.

**F3 — HIGH: optional LLM can replace a stranded robot's HOLD task.** The planner pre-fills a stranded robot `HOLD` at [L664–682](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/agent/planner.py#L664-L682), but advisor override at [L725–738](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/agent/planner.py#L725-L738) replaces it without a stranded-state veto. Validation at [L586–618](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/agent/planner.py#L586-L618) checks site/capability but not stuck-state authorization. **Repro:** mock advisor returns a valid `SAMPLE` for a robot already classified stranded. **Fix:** advisory output only proposes actions, immutable System-0 and incident-state veto applies *after* all overrides; disallow rescue skill for victim, invalid/aged frame and unsafe corridor. **Rubric:** autonomy, safety, technical quality.

**F4 — HIGH: System-2 is one-shot, synchronous, without tools or a real monitor/retry loop.** `SupervisoryAdvisor` exposes only `advise(event, frame, estimates) -> robot site assignments`, and `GeminiSwarmAdvisor` makes a blocking `urllib.request.urlopen(timeout=2.5)` inside `decide` ([planner.py L88–190](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/agent/planner.py#L88-L190), [L565–713](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/agent/planner.py#L565-L713)). [docs/DECISIONS.md D-0011 L130](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/docs/DECISIONS.md#L129-L132) says "asynchronous and non-blocking", contrary to code; `<5ms`/`<20ms` tier timings are documented claims, **not measured hardware benchmarks**. **Fix:** off-tick optional worker / bounded poll with last-known-safe deterministic command; explicit tool request/result and budget state, no simulation-clock stall. **Rubric:** adaptation, autonomy.

**F5 — HIGH: current triage values are not calibrated rescue risks.** [planner.py L430–522](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/agent/planner.py#L430-L522) uses `hazard=2` near no-go, asset values `0.10`/`0.35`, offline-data=`0.55`, opportunity planting=`1.45` and straight-line travel. These are hand-set utility coefficients, not second-casualty probabilities or traction/pull force. **Fix:** separate hard feasibility, observed-conditioned probability range, explicit operator weights, physical outcome cost and unknowns; benchmark against a calibrated optimiser with identical observations. **Rubric:** impact, autonomy.

**F6 — MEDIUM/HIGH: source of supposed sensor feedback is not established for all reflexes.** [adapter.py L440–503](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/mujoco_sim/adapter.py#L440-L503) supplies pose, range, local samples and `is_bogged` derived from simulated terramechanics, but does not fill `true_speed_m_s`, `peer_distances` or `row_cross_track_m` (defaults remain `None`/empty/zero). The controller implements logic for these optional channels ([control.py L47–102](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/control.py#L47-L102)), while D-0011 mentions Doppler radar and UWB as if sensors exist in the actual robot. **Fix:** state channels as synthetic proxies or unavailable; derive stall from actually sampled odometry/time and command history; keep simulated bogging separate from hidden ground truth in independent grading. **Rubric:** perception, technical honesty.

**F7 — MEDIUM: adviser availability gate is unnecessarily coupled to every robot's link.** [planner.py L574–576](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/agent/planner.py#L574-L576) declines advisor if *any* robot `link_ok=False`, including an immobilised victim that may have lost radio while a functioning helper/farm host still has inference connectivity. **Fix:** have explicit reachable reasoner host / fresh data provenance, not "all rover links healthy"; never place a necessary inference shard on the stranded robot. Offline always deterministic safe. **Rubric:** autonomy.

**F8 — MEDIUM: no strong matched ablation or measured model necessity in `main` unit tests.** The existing [test_agentic_swarm.py L233–312](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/tests/test_agentic_swarm.py#L233-L312) tests a mock advisor, offline fallback and output validity, **not** real model inference or improvement in rescue outcome. The alternate [riceswarm_ai/ai_agent.py](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/riceswarm_ai/ai_agent.py) is another one-shot allowed-action selector inside a separate lifecycle simulation. **Fix:** measure deterministic no/always/hand-weighted/calibrated expected-utility versus model one-shot and model tool-loop on *matched incidents* (see Section 13.3 executed 6-policy ablation). **Rubric:** relevance, ambition.

**F9 — MEDIUM: scripted recovery test proves motion but not a disabled-and-restored robot.** The test drives a preset encounter, actuates latch/pitch, asserts victim `x` shifts `>0.08 m`, and stops ([test_world_and_robot.py L135–178](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/tests/test_world_and_robot.py#L135-L178)); there is no observed true stall, autonomous target localization, safe route veto, unlatch and resumed soil/plant work in that isolated test. **Fix:** single MuJoCo mission episode with fault → recovery skill → feedback → renewed task, plus a deliberately unsafe case requiring refusal. **Rubric:** all categories.

**F10 — BLOCKER: Newly merged `LocalController` sensor reflexes (`control.py` L61–70) physically prevent a helper from approaching within the `0.065 m` rigid-dock capture radius.** [src/ricebot/control.py L61–70](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/control.py#L61-L70) (merged in PR #35) enforces `if robot.is_bogged: return RobotCommand(..., linear_m_s=0.0)` and `if pdist < self.limits.min_peer_distance_m: return RobotCommand(..., linear_m_s=0.0)` where `min_peer_distance_m = 1.0 m`. Meanwhile, [src/ricebot/robot/rovers.py L57](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/ricebot/robot/rovers.py#L57) requires `dock_capture_radius_m = 0.065 m` (`65 mm`) for `update_rigid_dock_latch()` to engage `eq_active = 1`. **Fix:** Exempt authorized rescue/tandem-extraction maneuvers from the `min_peer_distance_m = 1.0 m` UWB brake and allow synchronized tandem wheel drive once `eq_active == 1`.

## 4. Human problem, numerical vs contextual judgement

**Human beneficiary:** a small rice-field operator running a limited fleet for stand-gap mapping/replanting. Robot loss has an asset/downtime cost; a second robot entering saturated soil may itself be lost; diverted effort can forfeit a planting window; plant/crop damage matters. The farmer may prohibit risking the last working planter even if rescue raises expected asset recovery value. The supervisor should communicate competing outcomes and uncertainty in operator language, without inventing missing economic values.

**Calculate, do not ask the LLM to guess:** traversable route/ETA, available battery and tow energy, estimated traction and pitch reserve, contact reach, recovery skill capability, remaining seeds/sites, expected mission utility given stochastic success, upper bound on helper-loss risk, confidence/age of samples. These are deterministic tools using available *estimated* observations; none may inspect the simulator's hidden event label.

**Contextual judgement can assist:** when to spend a bounded observation on a contradictory sensor, whether a delayed crop job outweighs scarce hardware, how to translate human instructions ("do not risk the only helper", "save the only planter if no crop damage") into **explicit** preferences / constraints and when to escalate if values are absent. The model may suggest *which uncertainty to resolve next* and explain *why a plan changes*; it must not secretly invent a calibrated risk probability. Require human confirmation of unprovided value weights, use conservative defaults until then.

Example operator preferences (must be supplied identically to optimiser and LLM):
- **"This is the last functional planter"**: raise future mission opportunity cost of risking it; if operator sets max helper-loss risk to `0.02`, this is a **hard** veto, not a prose preference that the LLM can negotiate away.
- **"Victim carries the only seed stock"**: increases rescue benefit *if* physical success and plantable sites exist; may justify investigation before a rushed `NO`.
- **"Do not enter the wet ditch today"**: forbid that approach regardless of nominal cost optimum.
- **"Preserve mapping data but planting can wait"**: adjust utility weights within the safe feasible set; allow defer/remote evidence request.

**Decision quality criterion:** match operator preferences with *measured* mission outcome while obeying hard constraints. Elegant reasoning without a positive intervention outcome or calibrated refusal is not a product win.

## 5. Minimal *one-runtime* supervision architecture

~~~text
ONE MuJoCo episode / SimulationPort (same two physical robots)
  World physics, local probes/range/IMU, body/contact/latch mechanism
       | time-stamped, observable telemetry (no hidden oracle)
       v
System 0: per-robot local control + hard safety veto (EVERY physics tick)
       | valid command->measured progress, tip, peer clearance, slip proxy
       v
System 1: cheap fault monitor -> SUSPECTED incident + fresh belief summary
       |       \
       |        -> ordinary task planner / Policy A or simulated Policy B
       v
Incident supervisor FSM (rare, outside actuator-rate loop)
   OBSERVE -> INVESTIGATE -> EVALUATE -> AUTHORIZE -> EXECUTE
      ^                                           |     |
      |            result/fresh telemetry <--- MONITOR
      +------ REVISE (bounded) ----- / ABORT or ESCALATE
       | tool calls: sample/health, route, risk, mission value
       | reasoning backend:
       |   deterministic rule/expected-utility policy (always present)
       |   optional async LLM tool-loop (candidate only)
       v
Typed RecoverySkillRequest -> deterministic validator -> robot-owned dock/tow executor
       | versioned request_id, time/TTL, allowed skill and path
       v
RecoverySkillResult + independent physical evaluator
       | measured displacement, self/peer hazards, recovered mobile state
       v
Resume existing survey/plant task OR HOLD / human escalation
~~~

No new MuJoCo/Isaac world, stand-alone lifecycle simulator, radio-stack deployment, Jetson shard, training environment, second agent orchestration framework, or raw LLM motor commands.

**Ownership separation:** physics/world owns *true* rescue success and hidden evaluator state; sensor host emits measured proxy; System 0 safety rejects unsafe motion independent of model; System 1 manages incident FSM and canonical field tasks; optional System 2 requests tools and proposes bounded high-level choices. The LLM is not an instance of each robot's motor controller. The same controller executes both deterministic and model-backed policies.

### Typed minimal additions (proposed)

**IncidentEvidence** (`world -> supervisor`, event ID, episode ID, `t_s`, victim, `commanded_linear_m_s`, `observed_delta_pose_m`, `observation_window_s`, `range_valid`, `front_range_m|null`, IMU pitch/roll, `believed_slip_range`, `link_age_s`, `observations_used[]`, `belief_confidence`, source IDs). Missing values carry `null` and explicit validation, not fabricated zero.

**OperatorPolicy** (provided to both comparators, config version, mission priority weights, protected crop/no-go cells, `max_helper_loss_probability`, time/battery cutoff, `human_authorization_required` boolean, `received_at_s`, provenance).

**ToolRequest** (`request_id`, `incident_id`, `time_s`, tool enum, typed args, `max_age_s`, `deadline_s`). Tools return `{request_id, frame_time_s, status: OK/UNAVAILABLE/STALE/REJECTED, values, units, evidence_ids, cost_s, uncertainty}`. A missing read cannot be transformed into a success value.

| Tool | Inputs → observable outputs | Preconditions / budget / rejection |
| --- | --- | --- |
| **inspect_fault** | `robot_id, window_s` → commanded-vs-observed progress, missing-sensor flags, IMU alarms, suspicion label and confidence interval | cached observations only; ≥2 valid samples or return `INSUFFICIENT_DATA`; deadline ~1 physics step; does **not** output hidden "stuck truth" |
| **request_close_observation** | `helper_id, target_robot_id, target_pose estimate, standoff_m` → task request + later new range/visual/pose evidence | path and clearance viable, operator permits risk; **physical movement consumes time/energy**; never instant oracle |
| **plan_approach** | `helper_id, estimated victim pose, current map version, safety envelope` → feasible safe route, length, ETA, reachable docking geometry, route risk range | world model uses observed map only, timestamps current; no-go/hard limits enforced; can return `NONE` |
| **estimate_recovery** | `incident evidence, approach plan, skill ID, traction/battery observations` → `p_success` interval, upper helper-loss risk, estimated duration/energy and reason for unknown | only estimates from validated priors/physics approximation; refuse if no calibration and cannot bound risk |
| **estimate_mission_utility** | `current tasks, seed/battery stock, operator weights, expected time` → utility under no rescue / defer / rescue and confidence bounds | shared observations and explicit preference version; no unknown prices invented |
| **execute_recovery_skill** | `incident ID, helper/victim IDs, chosen allowed skill, approved route, start/expiry, maximum duration/travel/force envelope` | human-owned mechanics adapter; hard safety veto; **at most one active skill**, helper cannot be victim, idempotent request IDs; reject unsafe/stale/mismatched model output |
| **get_recovery_feedback** | `skill run ID` → latch/contact sensed, pose displacement, commanded/measured motion, fault reason, helper health, `success-unverified` | feedback is *observed* and independent of model; deadline/TTL; no "success" until check |

Illustrative *design budgets*: ≤2 investigation tool rounds, ≤1 authorised physical recovery attempt per incident, ≤1 replan after changed evidence, ≤2.5s wall-time per optional LLM call, incident reasoning call not permitted on high-rate control thread, hard 60s simulator-wall-clock demo budget. If any limit is exceeded, deterministic `HOLD`/`DEFER`/`ESCALATE`. Log actual calls, tokens, latency and physical step costs.

### Proposed FSM (minimal pseudocode)

~~~python
# DESIGN ONLY: reference supervisory state machine
on_tick(frame):
    safe_motor_commands = system0_and_existing_mission_core(frame)
    incident = fault_monitor.update(frame, prior_commands)
    if incident is new or materially changed:
        supervisor_queue.submit(incident, frame.version)  # non-blocking
    if approved_skill_ready_and_fresh():
        safe_motor_commands = safety_gate.overlay_bounded_skill(safe_motor_commands)
    return safe_motor_commands

supervisor_worker(incident):
    evidence = inspect_fault(incident.robot_id, observed_window)
    if evidence.status != "OK": return HOLD_AND_REQUEST_OBSERVATION
    candidate_routes = plan_approach(helper, victim_estimate, belief_map)
    candidate_risks = estimate_recovery(evidence, candidate_routes, approved_skill)
    mission_values = estimate_mission_utility(available_tasks, operator_policy)
    baseline = calibrated_risk_policy(candidate_routes, candidate_risks, mission_values)
    proposal = optional_llm_tool_loop(bounded_tools, incident, policy) or baseline
    proposal = strict_schema_validator(proposal)
    decision = safety_and_preference_veto(proposal, baseline, fresh_world_version)
    if decision in {DEFER, ABSTAIN, ESCALATE}: return decision
    skill_id = authorize_one_bounded_skill(decision)
    feedback = get_recovery_feedback(skill_id)
    if feedback indicates changed hazard or no dock:
        invalidate_old_route_and_estimates()
        return bounded_replan_once_or_abort()
    if measured_recovery_and_mobility_verified(feedback):
        return RESUME_MISSION
    return STOP_AND_ESCALATE
~~~

## 6. Example counterfactual trace — **HYPOTHETICAL DESIGN CASE**

**Initial conditions:** rovers A and B are surveying 5 rice sites; A has the only remaining seed hopper, B is the available helper; A's commanded travel is `0.20 m/s` but estimated pose displacement is `<0.01 m` over several frames, front range is temporarily invalid. Operator policy allows rescue only if helper-loss risk upper bound `<0.10` and prohibits crossing healthy crops.

1. **A suspected:** System 1 notices commanded-versus-observed stall with missing front sensor; emits `SUSPECTED_STALL`, not `bogged=true`.
2. **Model asks for evidence:** `inspect_fault(A, 3s)` returns stale range, confirmed non-progress and weak slip confidence. The supervisor orders B to a *safe standoff sample pose* instead of authorizing tow immediately. The baseline must have access to the same operation and may choose it too.
3. **New evidence changes belief:** B's fresh local range/terrain sample reveals a newly blocked narrow route and soft patch near A; `plan_approach` returns old route `REJECTED`, alternate route feasible but longer; `estimate_recovery` returns helper-loss bound `0.16` (illustrative computed estimate).
4. **Plan invalidated:** old `RESCUE_NOW` candidate is revoked. Because `0.16` exceeds the operator's hard `0.10` threshold, System 0 rejects proposed docking despite A being high-value. Model selects `DEFER`/`HUMAN_ESCALATION` and directs B to safe unaffected mapping, avoiding an unjustified second exposure.
5. **Alternative branch for success test:** on a different matched seed the new sample instead confirms firm access and risk bound `<0.10`, supervisor authorizes one tested dock/tow skill; feedback reports latch state and physical displacement; A must subsequently execute a *new* survey/plant motion to count as recovered. Failed latch triggers one bounded revision or abort.
6. **Evaluator** scores field utility, dispatch, rescue outcome, helper health and latency using hidden truth inaccessible to every policy.

## 7. Falsifiable six-policy ablation, not an LLM popularity contest

**Policies, identical frozen simulation incidents / sensors / tools / operator policy / per-incident time and power limits:** (N) never rescue, (A) always attempt when hard safety allows, (H) checked-in hand-weighted rescue triage, (O) deterministic calibrated expected-utility/risk policy with optional active information gathering, (L1) one-shot validated LLM verdict after standard initial observations, (L2) bounded model tool-loop. The critical `L2 - O` difference is *adaptive information acquisition / qualitative instruction handling*, not privileged sensor inputs or extra simulation time.

**Experimental families, all with success and abstention cases:** actual traction bogging vs range dropout vs intentional `HOLD`; direct safe approach vs unknown alternate route; rescuable vs unrecoverable latch/reach; last functioning planter and no-secondary-casualty operator policy variations; fresh weather/soil evidence that reverses risk ordering; failed first latch, stale gossip, delayed link, false agent tool response, model outage.

**Metrics:** failure diagnosis precision/recall by class, correct rescue/defer/abstain, physical peer mobility restored, second robot immobilised/damaged, remaining measured fieldwork utility, crop/no-go violations, false dispatch, physics energy and detour time, total simulated episode time, unnecessary diagnostic moves, correct changed-fact invalidations, safety veto count, tool-call validity/rejections, median/p95 incident reasoning latency, timeout count, tokens/cost if actually available.

**Hypothesis H1:** L2 makes fewer wrong dispatches than O in operator-language/changed-belief cases at no worse hard-safety violation rate and acceptable latency. **Falsifier:** O matches or beats L2 within uncertainty; model increases time, unsafe proposals, tool errors or costs. Then ship O and describe model as an exploratory extension. **Hypothesis H2:** a one-shot LLM cannot reliably revise after a new hazard because it lacks a tool/feedback loop. **Falsifier:** under equal available observations, L1 achieves same episode utility and timing, making L2 unnecessary for this scope.

**Leakage guard:** world true stuck labels, actual traction coefficients, future hazards, recovery ground truth, target task outcomes and grader utilities remain private to episode simulator/evaluator; the supervisor receives only timestamped synthetic sensor readings and policy-issued operator preferences.

## 8. Failure / injection / latency / uncertainty controls

- **Sensor prompt injection:** peer radio text, operator notes and retrieved weather descriptions are *untrusted data*, never system instructions; only allow structured observations with source IDs, timestamps, provenance and capability checks.
- **Spoof / stale / replay:** monotonic episode time, robot ID signature/in-simulator origin, observation TTL and content/version checks; duplicates rejected. Simulated Policy B gossip is not authenticated real radio.
- **Invalid JSON / hallucination:** strict schema enums, valid robot/site/skill IDs, numeric finite bounds/units, no extra keys, no unknown physical state. Reject to safe bounded fallback; log rejection reason.
- **Wrong actuator request:** System 0 checks identity, geometry, no-go/crop boundary, fresh map version, required contact geometry, helper health/traction and single-active-skill lock before motion. Invalid -> `HOLD` + incident log.
- **Runaway tool loop / retries:** call budget, unique request IDs, one in-flight physical intervention, one bounded replan. If latch fails twice or contact slips, stop.
- **Timeout / model unreachable:** asynchronous worker deadline; actuator-rate loop continues with deterministic commands / `HOLD`. Distinguish farm-host connectivity from victim radio. No model shard dependency on broken robot.
- **Bad calibration:** estimate risk intervals or mark `UNKNOWN`; unknown high-risk options fail safe.
- **System-0 timing:** D-0011 claims `<5ms` but a local Python cutoff in a simulation tick is **not** proof of a 5ms hardware control loop.

## 9. Five ordered change packets, owners, tests and stop-loss

| Priority | Owner and paths (suggested only) | Change; prerequisite | Acceptance test / stop condition | Effort estimate; revert |
| --- | --- | --- | --- | --- |
| **P0 / 1** | Kernel + mechanics owners, existing contracts, `SimulationPort`, robot MJCF | **Freeze one incident and bounded recovery request/outcome seam** from existing world to physical dock (and exempt rescue approach from `<1.0 m` peer brake in `control.py`). | Two-rover same-run genuine stall, typed request, physically observed victim movement and later independently observed motion/field task. **Stop** if mechanics don't physically work in current MuJoCo scene; preserve demo as partial. | ~2–4 focused engineer hours; revert adapter feature flag/PR, retain scripted test |
| **P0 / 2** | Planner/control owner, `agent/planner.py` and direct tests | Fix `None`-range false stranded and forbid advisor override of stranded `HOLD`; establish incident state/TTL and deterministic veto. | Range-dropout healthy case no rescue; stalled-with-valid-range case suspected; valid advisor action cannot clear incident `HOLD`; stale action rejected. | ~1–2 h estimate; revert scoped branch while preserving safe `HOLD` |
| **P0 / 3** | Agent/planner + evaluator owners, reuse existing scripts/tests | Calibrated constrained optimiser with hard risk threshold; no/always/hand triage comparisons on same cases, starting 4–6 seeds. | At least one correct safe rescue and one correct refusal with independent outcome/decision log, matched observations and no hidden truth leakage. | ~1–3 h estimate; revert policy selection flag |
| **P1 / 4** | Agent owner only, narrow supervisor worker in `src/ricebot/agent/`; integrator owns typed seam | Optional bounded tool-using model with async worker / deterministic fallback. Requires P0 path green. | Model requests genuinely new observation, receives changed fact, invalidates original plan and never bypasses veto; offline/timeout safe; ablation against O on same seeds. | ~2–5 h estimate; revert off by default |
| **P1 / 5** | Independent evaluator + demo owner; existing test/evaluate/video paths | Timestamped trace + same-seed baselines + ≤60s continuous recording. Must not hide scripted vs autonomous distinction. | Replay with exact seed produces event/action/feedback links, model/no-model label, correct demo outcome and limits. | ~1–2 h estimate; revert optional overlay, keep raw logs |

## 10. Minimal executable proof: commands, observed vs expected

**Observed in GitHub CI:** [main CI run 42410512960](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/actions/runs/42410512960), conclusion success at pinned `6c3a5752`.

**Executed on local Windows 11 / Python 3.13.14 / MuJoCo 4.0.0 workstation** (see Section 13.1 for full trace):
~~~bash
python scripts/check_repo.py
python scripts/sync_skills.py --check
python -m unittest discover -s tests -p "test_agentic_swarm.py" -v
python -m unittest discover -s tests -p "test_world_and_robot.py" -v
python -m unittest discover -s tests -p "test_mujoco_sim.py" -v
python -m unittest discover -s tests -v
python scripts/run_mujoco_paddypatch.py --scenario nominal --seed 1 --steps 260
~~~

**Must-add test names for M1 integration verification:**
- `test_range_dropout_is_not_immobilisation(seed=401)`: old front range becomes `None` but motion evidence positive ⇒ `SENSOR_DROPOUT_HOLD`, not `RESCUE_NOW`.
- `test_genuine_stall_with_valid_range(seed=402)`: commanded progress positive but measured motion low ⇒ `S1_IMMOBILIZED`, investigate then safe response.
- `test_advisor_cannot_override_stranded_hold(seed=403)`: valid but unsafe advisor `SAMPLE` on victim ⇒ rejected and victim `HOLD`.
- `test_recovery_request_physically_resumes_mission(seed=404)`: same `SimulationPort` episode, helper joint/latch feedback, body moved and victim moves under its own commands.
- `test_new_hazard_revokes_authorisation(seed=405)`: later sensor invalidates route with same incident ID ⇒ command cancelled before unsafe entry.
- `test_model_offline_same_safe_fallback(seed=406)`: no API / timeout / malformed schema ⇒ deterministic risk policy runs; no hang.
- `test_mission_utility_matched_policy_ablation(seeds=410…433)`: identical world and observation budgets across N/A/H/O/L1/L2, record reasons/physical outcomes.

## 11. Prior art, novelty and strongest counterargument

1. **SayCan** (Ahn/Ichter et al., CoRL 2022 / PMLR 2023): language model supplies semantic planning while robot affordance/value estimates determine physically valid skills. [Published proceedings](https://proceedings.mlr.press/v205/ichter23a.html). **Implication:** LLM-plus-grounded-skills is not new; Antenna needs a specific measured rescue/abstain result.
2. **Inner Monologue** (Huang et al., CoRL 2022 / PMLR 2023): feedback-conditioned language plan updates, including real robot tasks. [Published proceedings](https://proceedings.mlr.press/v205/huang23c.html). **Implication:** "model re-plans after feedback" alone lacks novelty.
3. **IAOM** (Long and Liang, IEEE RA-L, published June 2026): dynamic scout-rescue task allocation, intention graphs, GAT-PPO and hybrid A*; reported improvements belong to their grid/urban experiments, not Antenna. [IEEE](https://ieeexplore.ieee.org/abstract/document/11560910). **Implication:** peer task allocation and rescue scheduling already have sophisticated non-LLM alternatives.
4. **RAYA** (Mahajan et al., Sept 2026): physically recoverable control via learned-analytic recoverability model and bounded learned priority scheduling, with authors' simulation/hardware results in different robot domains. [Project/paper](https://raya-control.github.io/). **Implication:** rescue control is not invented here; our two-rover physical extraction + context-specific operator tradeoffs must be the claim, if proved.
5. **InteLiPlan** (IEEE RA-L, Feb/Mar 2026): lightweight interactive LLM robot planning and failure recovery; vendor/paper-reported results on different robot tasks and hardware. [IEEE DOI](https://doi.org/10.1109/LRA.2026.3662577). **Implication:** recovery-aware tool planning already has real-robot precedents.
6. **LLM robotics failure caution** (Anthropic, July 2026, first-party vendor study): tested varying abstraction of model robot control and shows why primitive/interface abstraction matters; not independent evidence for Antenna. [Study](https://www.anthropic.com/research/claude-plays-robotics). **Implication:** avoid raw-torque model control and measure end-to-end task performance.
7. **Agent planning robustness** (Albayaydh et al., July 2026 survey, preprint) catalogues tool/constraint and safety failures across benchmarks; survey evidence, not Antenna measurements. [arXiv](https://arxiv.org/abs/2607.05775). **Implication:** tool-loop scaffolding creates failure opportunities and must be ablated.

**Plausibly distinctive claim to *test*, not assert:** "In a single physics-grounded two-rover rice-task episode, an incident supervisor autonomously acquires the missing evidence that determines whether a bounded physical peer extraction is worth risking; a changed observation reverses its recovery choice, it safely executes or abstains, and fieldwork utility is independently measured against a strong risk-aware non-LLM controller." Distinction is **the observable decision under incomplete information plus physical consequence**, not the word "agent."

**Strongest counterargument:** for two robots, finite allowed tools and a small micro-field, a calibrated POMDP/belief-state decision policy or simple expected-utility active sensing optimiser can solve the whole problem more reliably, faster and cheaper than an LLM. Antenna should include that baseline. An LLM earns runtime inclusion only if it performs better on truly novel language constraints/ambiguous evidence, without exceeding safety/latency limits. Otherwise the honest product is a competent resilient autonomous rescue system with an optional explainability/semantic interface.

## 12. Conflicts, assumptions and scope safeguards

- [PR #32](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/32) is **open** at M6 review; it proposes the one-scene rescue-first tool loop but does not implement it. Main docs still include old Isaac/planting-first passages; use [D-0009](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/blob/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/docs/DECISIONS.md#L93-L101) for current rescue-first scope while human integrator decides whether to merge PR #32.
- D-0011's *asynchronous* System-2, `<5ms` System-0 and `<20ms` System-1 claims lack runtime trace and do not match blocking REST code. Remove/qualify before judge claims.
- Model choice/access (Gemini API and any local model), actual host, tokens/cost, real Jetson inventory, sharded inference, and separate radios are **unknown**. A remote Gemini REST call is not an onboard model.
- Simulated Policy B's per-rover partial beliefs and gossip are **one process, modelled radio range**, not real independent distributed compute or multi-model debate.
- [src/riceswarm_ai/](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/tree/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d/src/riceswarm_ai) is a separate farm lifecycle experiment; it has a useful weighted deterministic recovery comparator but is **not** the canonical integrated MuJoCo runtime. Do not copy its entire field/calendar/season simulation into Antenna.
- Submission deliverables (`docs/EVENT.md`, `docs/DEMO.md`) require a one-page PDF and single continuous ≤60s recording planned against **Sunday 8 Nov 2026, 13:00 GMT**.

**Bottom line / handoff:** Have the integrator verify F1–F3 and F10 and establish a *physical rescue result seam* first. Let the deterministic risk-aware supervisor become the baseline and fail-safe; add **one optional tool-using model** only once an incident can be physically observed, acted upon and independently scored.

---

## 13. Local Workstation MuJoCo 4.0.0 Executed Verification Addendum (`2026-11-07 20:32 GMT`)

*Note: This addendum preserves Sections 1–12 above and supplies the **actual local workstation MuJoCo 4.0.0 execution results**, **control-reflex conflict discovery (`F10`)**, **6-policy matched-seed ablation metrics**, **concrete multi-step escalation trace**, and **integration-ready `classify_rover_immobilization` code** from the local workstation M6 investigation at reference SHA [`6c3a5752d73c1a1a2ed42479aa919e5f82415f7d`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/commit/6c3a5752d73c1a1a2ed42479aa919e5f82415f7d).*

### 13.1 Executed Commands & Local Runtime Evidence
- **Workstation Environment:** Windows 11 (`10.0.26200-SP0`), Python `3.13.14`, MuJoCo `4.0.0`, NVIDIA GeForce RTX 5050 Laptop GPU (`8 GB VRAM`), `15.4 GB` RAM.
- **Executed Team Repo Foundation & Physics Suite (`6c3a5752` & `82099e0`):**
  1. `python scripts/check_repo.py` $\to$ `OK: required docs, portable skill parity, schema syntax, and local Markdown links`
  2. `python scripts/sync_skills.py --check` $\to$ `OK: all registered skills are valid and byte-identical in both native locations`
  3. `python -m unittest discover -s tests -v` $\to$ **`Ran 72 tests in 6.733s — OK`** (verifying `test_agentic_swarm.py`, `test_world_and_robot.py` rigid docking `>0.08 m`, `test_mujoco_sim.py`, `test_mycelial_swarm.py`, `test_sensor_reflexes.py`, `test_riceswarm_ai.py`).
  4. `python scripts/run_mujoco_paddypatch.py --scenario nominal --seed 1 --steps 260` $\to$ **`viable_seed_placements: 2`, `completion_time_s: 7.936`, `comm_packets: 6`, `comm_bytes: 736`**.
  5. `pytest -v` (independent multi-seed & 4-incident rescue harness) $\to$ **`11 passed in 46.01s`**.

### 13.2 Executed 6-Policy Matched-Seed Rescue Ablation (MuJoCo 4.0.0)

| Policy Variant | 1. Recoverable Mud Trap (`S1`, `seeds=2`, firm lane) | 2. Deep Quicksand Hazard (`S1`, `hazard=2.0`) | 3. Sensor Dropout False Alarm (`range_m=None`, `slip=0`) | 4. Intentional Hold (`v_cmd=0`, `slip=0`) | 5. High-Suction Stall (`\kappa > 1.75`, straight pull stalls) | **Secondary Casualties** | **False Dispatches** | **Measured Latency** |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **1. `never_rescue`** *(Executed)* | FAIL (`-0.50`) | PASS (`+0.50`) | PASS (`+1.50`) | PASS (`+1.50`) | FAIL (`0/12`) | `0` | `0` | `<0.05 ms` |
| **2. `always_rescue`** *(Executed)* | PASS (`+1.85`, `0.156 m`) | **FAIL (`-2.00`, 2nd rover lost!)** | **FAIL (`-0.60`, false dispatch!)** | PASS (`+1.50`) | FAIL (`7/12` fixed pull) | **`1`** | **`1`** | `<0.10 ms` |
| **3. `hand_weighted_heuristic` (`main` `planner.py`)** | PARTIAL (`SAMPLE` only, `0.0 m` tow) | PASS (`abandon_escalate`) | **FAIL (`L375` flags `range_m=None` as `stranded`)** | PASS | FAIL (unwired) | `0` | **`1`** | `0.18 ms` |
| **4. `calibrated_deterministic_tool_fsm`** *(Executed)* | **PASS (`+1.85`, `0.156 m` extracted & plants)** | **PASS (`+0.50`, safe refusal)** | **PASS (`+1.50`, `SENSOR_DROPOUT_HOLD`)** | **PASS (`+1.50`)** | **PASS (`12/12`, `0.607 m` mean via escalation)** | **`0`** | **`0`** | **`0.35 ms`** |
| **5. `llm_one_shot_verdict` (`GeminiSwarmAdvisor`)** | Cannot adjust technique mid-pull | Safe only if pre-gated | **FAIL (`L159` prompt tells LLM `range_m=None` is stranded)** | Not triggered | FAIL (no retry loop) | `0` | **`1`** | `450–1800 ms` *(unverified API)* |
| **6. `llm_tool_loop_supervisor` (System-2 over Tools 1–5)** | Matches Policy 4 (`+1.85`) | Matches Policy 4 (`+0.50`) | Matches Policy 4 (`+1.50`) | Matches Policy 4 (`+1.50`) | Matches Policy 4 + parses unstructured operator advisories | **`0`** *(Tool 4/5 veto)* | **`0`** *(Tool 1 veto)* | **`0.35 ms` offline fallback** |

### 13.3 Concrete Multi-Step Supervisor Trace (Evidence Request $\to$ Unexpected Stall $\to$ Revised Physical Command)
*(Physical states verified in MuJoCo 4.0.0; JSON tool envelope illustrates the 5-tool state machine).*
1. **`t = 2.40 s` — Tool 1 (`diagnose_immobilization(robot_id="scout", window_ticks=3)`):** Observes `v_cmd = 0.35 m/s`, `v_odom = 0.002 m/s`, `front_range_m = 0.85 m` $\to$ returns `fault_class = "S1_IMMOBILIZED"`, `mean_slip_ratio = 0.994` (rules out `SENSOR_DROPOUT_HOLD`).
2. **`t = 2.40 s` — Tool 2 (`probe_self_recovery_pulse(robot_id="scout", dibber_depth_m=-0.050, pulse_duration_s=0.6)`):** Active diagnostic probe before diverting `planter` $\to$ **Unexpected result:** `freed = False`, `displacement_m = 0.011 m`, `estimated_suction_severity = "HIGH_CLAY_LOCK"`.
3. **`t = 3.00 s` — Tool 3 (`compute_rescue_feasibility(victim_id="scout", helper_id="planter", operator_priority="balanced")`):** Evaluates local map (`approach_firmness_mean = 0.68`, `secondary_casualty_prob = 0.08`, `net_expected_utility = +2.52`) $\to$ authorizes `RESCUE_NOW`.
4. **`t = 3.00 s` — Tool 4 (`execute_bounded_dock_and_extract(..., technique="direct_pry_and_pull", max_duration_s=1.6, abort_helper_slip=0.65)`):** `planter` docks (`latch_engaged = True`), lifts `+0.36 rad`, and pulls straight back $\to$ **Unexpected physical feedback:** `status = "STALLED_IN_PLACE"`, `victim_displacement_m = 0.024 m` (`< 0.08 m` target), `peak_helper_slip = 0.41` (safe `< 0.65`).
5. **`t = 4.60 s` — Tool 4 Revised Call (`execute_bounded_dock_and_extract(..., technique="dynamic_yaw_shear_and_pitch_pump", max_duration_s=2.0)`):** Supervisor revises physical command to cyclic differential yaw-shear ($\pm 10\text{ rad/s}$) + pitch pumping (`0.15..0.42 rad`) $\to$ breaks clay suction, achieves `status = "EXTRACTED"`, `victim_displacement_m = 0.214 m`, `unlatched_cleanly = True`, and both rovers resume planting.

### 13.4 Drop-In Python Implementation Packet for M1 (`src/ricebot/agent/planner.py`)

```python
@dataclass(frozen=True)
class ImmobilizationDiagnosis:
    is_physically_immobilized: bool
    fault_class: str  # "S1_IMMOBILIZED" | "SENSOR_DROPOUT_HOLD" | "INTENTIONAL_HOLD" | "NOMINAL"
    sensed_slip_ratio: float


def classify_rover_immobilization(
    commanded_speed_m_s: float,
    measured_ground_speed_m_s: float,
    front_range_m: float | None,
    is_bogged: bool = False,
) -> ImmobilizationDiagnosis:
    """Distinguish true wheel-slip immobilization from rangefinder dropout (None) or intentional HOLD."""
    if front_range_m is None and not is_bogged:
        return ImmobilizationDiagnosis(False, "SENSOR_DROPOUT_HOLD", 0.0)
    if is_bogged:
        return ImmobilizationDiagnosis(True, "S1_IMMOBILIZED", 1.0)
    if abs(commanded_speed_m_s) < 0.05:
        return ImmobilizationDiagnosis(False, "INTENTIONAL_HOLD", 0.0)
    slip = max(0.0, min(1.0, 1.0 - abs(measured_ground_speed_m_s) / abs(commanded_speed_m_s)))
    if slip > 0.85:
        return ImmobilizationDiagnosis(True, "S1_IMMOBILIZED", round(slip, 4))
    return ImmobilizationDiagnosis(False, "NOMINAL", round(slip, 4))
```
