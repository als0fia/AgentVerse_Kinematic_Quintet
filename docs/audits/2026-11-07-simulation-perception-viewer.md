# Antenna simulation, perception and visualiser boundary audit — agent repair handoff

**Audit performed:** 7 November 2026, ~22:13 GMT.  
**Source baseline:** [main at e0f016e](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/commit/e0f016ee16c7e76aed98dc74e67177636731e2ce), immediately after merged [PR #50](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/50). The prior review was based on [16768dc (PR #48)](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/commit/16768dc6d8f7d5edd3dd5e34c9ec23454dcfbd29), and **some findings have changed since that older version**.  
**Method:** GitHub source/diff/tests review. **No local Python, MuJoCo, display or end-to-end episode executed by this auditor.** A green [GitHub Actions run for this baseline](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/actions/runs/42419561033) is not a substitute for the unexercised GUI, hidden-state and rescue-branch checks below.  
**Scope:** the actual Antenna team application, especially its MuJoCo world, robot sensing, rescue control and judge-facing side-by-side visualiser. No application code changed in this audit.

**Read before acting:** [root AGENTS](../../AGENTS.md), [architecture](../ARCHITECTURE.md), [contracts](../CONTRACTS.md), [component ownership](../COMPONENTS.md), [testing](../TESTING.md), [demo](../DEMO.md), [team branch protocol](../TEAM.md) and the [module audit skill](../../.agents/skills/module-audit/SKILL.md). These documents own existing requirements; this audit is an evidence-backed **repair worklist**, not another architecture specification.

## 1. Invariants the finished system must enforce

1. **One authoritative physics episode:** one MuJoCo Model/Data pair and one owned reset/step loop determine robot motion, contacts, terrain reactions, faults and recorded outcomes. Analytical soil force proxies may legitimately feed that model; decorative rendering must not substitute for dynamics. Isaac Sim is not a second hackathon obligation.
2. **A robot never receives evaluator/private truth by accident:** ground-truth map, exact global poses, trap flags, synthetic fault schedule, hidden crop viability and perfect peer state must be separate from the **explicitly modelled, time-stamped, valid/invalid sensor interface**. Clearly declared idealised virtual GNSS/IMU/UWB may be used initially, but never mislabelled as inference or as real deployed hardware.
3. **Controller versus simulator:** strategic agent chooses RESCUE/DEFER/REFUSE and bounded actions; robot-side deterministic/specialist control uses permitted observations for approach, docking and safety; MuJoCo executes actuators and reports physical contact. The simulator should not quietly steer a helper using target truth while the agent is credited for that precision.
4. **Viewer has no unaccounted control authority:** viewer can render 3D physics, robot belief and evaluator-only truth with **unambiguous labels**. Pause/reset, scenario selection and operator-injected disturbances are legitimate through a **separate logged session/experiment API**; viewer code must not write qpos, model parameters or controller state directly.
5. **Comparable evidence:** dashboard, headless test, recording and grader use the same scene/seed/runtime configuration and trace; benchmark numbers are computed from named runs or marked illustrative; green unit CI is not a measured live demo.
6. **Preserve the product:** two robots, current rigid docking, risk-sensitive rescue-or-abstain, meaningful fieldwork after the event and existing physical golden-fixture success. Do not replace the MuJoCo stack or install new frameworks to achieve an elegant diagram.

### Current intended ownership flow

- World definition: src/ricebot/world/ (terrain functions, hidden soil/stand distribution, scenario inputs). The simulation adapter composes its physical world into MuJoCo.
- Robot embodiment: src/ricebot/robot/ (geometry, mounts, actuators, joint limits and rescue coupling).
- Physics and sensor adapter: src/ricebot/mujoco_sim/adapter.py (MuJoCo dynamics, permitted virtual sensor synthesis and actual contact feedback).
- Typed boundary: src/ricebot/contracts.py and runtime.py (one SimulationPort; explicit observations, commands and feedback).
- Robot/mission intelligence: src/ricebot/control.py and agent/ (local estimation, bounded skill commands, coordination and high-level rescue verdicts).
- Presenter/evaluator: src/ricebot/viewer/ and independent grading; they may see truths **only for display/scoring** and must not feed it back.
- scripts/run_mujoco_paddypatch.py is an entrypoint/configuration adapter, not a second implementation of simulation or rescue.

## 2. Version reconciliation — changes since the prior review

**Prior [16768dc] finding now PARTLY SUPERSEDED:** the live UI formerly had no golden rescue scenario/trap and ran only the old baseline runtimes. [PR #50](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/50) added a golden-rescue selector, trap configuration, cooperative policy and a 200-step bound in src/ricebot/viewer/live_app.py. **Do not report “no golden rescue UI” as if still true.** It remains unproved that the GUI and headless runner reproduce the **same authoritative** execution: they still independently instantiate/configure simulator and agent and the UI still manually steps physics, mirrors a second runtime and mutates model settings.

**New critical compatibility regression from PR #50:** the Mycelial rescue branch constructs RescueAssessment with attributes not defined by its shared dataclass. The GUI also reads nonexistent assessment attributes. The optional Mycelial advisor invokes a nonexistent recommend() method and swallows its exception. These were not in the older audit; fix them before making any viewer/AI claims. A passing CI run does not show these paths were exercised.

**Rule for the fixing agent:** compare each finding against the actual checked-out SHA, freshest origin/main and active PRs. Mark it PRESENT / FIXED / PARTIAL / DIFFERENT / NOT REPRODUCED, recording fresh line numbers. Never blindly cherry-pick fixes from this timestamped audit into an older branch.

## 3. Findings at audited baseline (exact evidence)

Severity CRITICAL = invalid perception/autonomy claim or immediately broken product path; MAJOR = incorrect product boundary or risky demo/correctness gap; MINOR = lower-priority fidelity/representation issue. “Observed” below means **verified by source code**, not a runtime reproduction.

### CRITICAL C1 — PR #50 Mycelial rescue constructor uses an incompatible dataclass

- **Evidence:** [canonical RescueAssessment fields](../../src/ricebot/agent/planner.py#L42-L64) are stranded_robot_id, verdict, nearest_site_id, rescue_benefit, opportunity_cost, hazard_risk, net_rescue_value, helper_robot_id and reason. But [Mycelial rescue branch](../../src/ricebot/agent/mycelial_swarm.py#L983-L1070) passes rescuer_robot_id, expected_saved_benefit, rescue_opportunity_cost, net_rescue_utility and rationale. Those names are not parameters of RescueAssessment.
- **Consequence:** in a scenario with a bogged robot and an eligible reachable helper, Python will raise TypeError at that constructor if this code path is reached. New Mycelial rescue path is not verified by normal golden-rescue fixture (which selects CooperativeSwarmPlanner).
- **Minimal fix:** use **one canonical RescueAssessment contract**, map the new branch's values to its existing names, supply honest hazard_risk and reason, avoid two incompatible RescueAssessment classes. Also update all Mycelial/UI consumers. Add a focused bogged-peer test that reaches the branch and checks RESCUE/DEFER and actual command/feedback.
- **Owner:** agent/planner + shared-contract integrator. Preserve existing golden test.

### CRITICAL C2 — PR #50 UI triage inspector reads nonexistent RescueAssessment attributes

- **Evidence:** [live_app.py triage inspector](../../src/ricebot/viewer/live_app.py#L1525-L1544) renders ass.rescuer_robot_id, ass.net_rescue_utility, ass.expected_saved_benefit and ass.rescue_opportunity_cost. The actual shared class has helper_robot_id, net_rescue_value, rescue_benefit and opportunity_cost.
- **Consequence:** once any rescue assessment exists, the inspector is expected to fail with AttributeError during redraw, **including a central golden scenario** that otherwise generates valid assessments. **Source-level prediction; GUI runtime was NOT RUN.**
- **Minimal fix:** use canonical properties or a presentation-only adapter, test inspector formatting with a real central planner assessment and Mycelial assessment, including empty/no-helper/DEFER. Do not conceal the exception.
- **Owner:** viewer; coordinate with C1 first.

### MAJOR M1 — PR #50 optional System-2 hook calls the wrong advisor API

- **Evidence:** [SupervisoryAdvisor/GeminiSwarmAdvisor](../../src/ricebot/agent/planner.py#L90-L130) expose advise(event, frame, estimates), but [Mycelial runtime](../../src/ricebot/agent/mycelial_swarm.py#L1082-L1099) calls self.advisor.recommend(...) with unrelated arguments and suppresses all exceptions.
- **Consequence:** the described Mycelial “System-2 AI” path can silently do nothing. Do not claim an LLM made a successful rescue decision from this path.
- **Minimal fix:** select **one** explicitly typed advisor interface, call it with valid event/observation context and handle absent key/offline/timeout/invalid suggestions with bounded deterministic fallback, visible status and no direct torque authority. Test using a stub adapter (no paid calls or keys).
- **Owner:** agent + integrator; keep external AI optional.

### CRITICAL C3 — simulator's bogged truth is handed to the robot as perception

- **Evidence:** [adapter.py ObservationFrame](../../src/ricebot/mujoco_sim/adapter.py#L797-L904) populates RobotObservation.is_bogged directly from the simulator's private _bogged_origins / computed belly-out state. Both [CooperativeSwarmPlanner stranded detection](../../src/ricebot/agent/planner.py#L381-L439) and [Mycelial rescue triage](../../src/ricebot/agent/mycelial_swarm.py#L983-L1021) accept this as an immediate trigger.
- **Consequence:** high-level fault perception is partially oracle-assisted. A wheel/pose-command mismatch detector **exists** in CooperativeSwarmPlanner, but the test also asserts an explicit synthetic is_bogged flag, so it does not establish “no oracle”.
- **Minimal fix:** retain private bogged flag for fixture/evaluator/debug only; add an independently derived RobotFaultEvidence from commanded wheel/motion, observed odometry/range and validity/time; force mission rescue dispatch from observed evidence. A dedicated perfect status signal is permissible only if modelled explicitly as a real sensor and labelled as such, not a hidden engine flag.
- **Tests:** equal outcome after hiding internal fault flag when real sensor symptoms persist; no rescue for intentional HOLD, no valid command, invalid range, stale odometry, pure obstacle brake; true fault leads to bounded action or safe refusal.
- **Owner:** adapter sensor producer + contract integrator + planner.

### CRITICAL C4 — simulator steers docking using exact privileged target geometry

- **Evidence:** [adapter.py step rescue substeps](../../src/ricebot/mujoco_sim/adapter.py#L621-L731) reads the exact MuJoCo 3D front/rear docking-site positions, computes heading error and emits wheel commands in the adapter, bypassing robot-side estimated sensing. The coupling condition in [robot/rovers.py](../../src/ricebot/robot/rovers.py#L484-L526) checks site distance and activates an equality constraint.
- **Consequence:** physical latch/tow is really simulated, but the helper's precise terminal guidance is simulator-assisted. This should not be presented as wholly onboard sensor-guided autonomy.
- **Minimal fix:** controller/robot skill owns a bounded phase machine using **estimated** relative range/bearing (or explicit camera-derived detections if actually implemented); adapter only executes actuator requests, collision/contact and latch mechanics. If this risks breaking the golden path before submission, preserve and label the existing truth-guided terminal servo as a provisional baseline and put the sensor-guided skill behind a verified opt-in test.
- **Tests:** perturb true target pose without updating a permitted observation, and verify controller output cannot follow hidden changes; noisy range/bearing, missing target, unsafe angle, near contact and abort; reproduce solver forces and docking under several seeds.
- **Owner:** robot/control owner + MuJoCo integrator + evaluator. **Do not rewrite robot MJCF without agreement.**

### MAJOR M2 — visual heightfield is not an authoritative collidable terrain surface

- **Evidence:** [adapter.py MJCF](../../src/ricebot/mujoco_sim/adapter.py#L52-L105) defines paddy_relief_mesh as hfield with contype=0 and conaffinity=0, while [world/terrace.py](../../src/ricebot/world/terrace.py#L525-L575) defines a collidable field_ground flat plane. The heightfield is presently a **visual overlay**, not a fully collidable incline.
- **Consequence:** visually apparent hills/bunds may not match wheel contact. Do not call heightfield appearance a physics-verified climb.
- **Minimal fix:** decide explicitly which surface is collidable and how its elevations align with the plane, wheel radius and obstacles; test contacts, slope grade, friction and safety *before* switching collision on. Avoid doubling contact surfaces, silent wheel penetration or destabilising the golden rescue. An analytical soil model may supply forces; it must be documented as a proxy.
- **Owner:** world + MuJoCo/mechanics integrator. **High risk: separate scoped PR only after demo freeze if needed.**

### MAJOR M3 — mud forces/pose are partly imposed outside normal MuJoCo dynamics

- **Evidence:** [adapter.py terramechanics](../../src/ricebot/mujoco_sim/adapter.py#L333-L373) computes soil slip/sinkage analytically and directly sets qpos vertical offset when bellied out. [step loop](../../src/ricebot/mujoco_sim/adapter.py#L530-L625) alters wheel friction, scales wheel commands, applies drag and directly damps qvel.
- **Consequence:** MuJoCo runs real rigid-body dynamics, but “physical mud” includes externally imposed proxies; direct pose manipulation is especially hazardous during contact/constraint solving. It is neither a solved deformable-soil interaction nor purely decorative.
- **Minimal fix:** separate documented soil->force/friction/drive-effect interfaces from camera-only visual approximation, remove direct pose correction when physics-safe and tested, record all tuning; do not make an overnight full Bekker/CFD implementation. Compare identical commands/seed across terrain conditions.
- **Owner:** world/physics + mechanical owner.

### MAJOR M4 — live GUI remains a writable alternative episode owner

- **Evidence:** [live_app.py _init_simulation](../../src/ricebot/viewer/live_app.py#L186-L257) constructs its own sim, local/cooperative runtime and mirrored runtime; [sizing handler](../../src/ricebot/viewer/live_app.py#L269-L285) writes mass/radius/width dicts, invokes two private simulation methods and calls mj_forward; [step_once](../../src/ricebot/viewer/live_app.py#L917-L936) calls sim.step and both decision runtimes. [Headless run_demo](../../scripts/run_mujoco_paddypatch.py#L318-L395) separately sets up its engine/policy/scene.
- **Consequence:** PR #50 makes the golden rescue selectable, but **does not prove headless/GUI parity**. View selection and interactive sizing can change actual dynamics. Two runtimes processing the same frame for UI purposes create misleading metrics and can trigger the C1 crash. The README describes the UI as read-only; the actual app is not.
- **Minimal fix:** add **one reusable session/configuration factory and one owner of step/reset**, potentially extending existing EpisodeRunner. UI consumes immutable frame/agent/evaluator telemetry; scenario/mass controls invoke a logged operator experiment command via session owner, not raw MuJoCo internals. Mirror policy is explicitly a *non-acting shadow baseline* that does not write commands or present simulated communications as transmitted.
- **Tests:** same seed/config, headless and GUI tick trace equality and final rescue outcome; viewer-only toggles do not mutate physics, actions or timestamps; pause/reset/camera safe. A snapshot replay is allowed but must be labelled “replay”.
- **Owner:** integrator + viewer. Change in small steps; preserve current runnable entrypoint.

### MAJOR M5 — observations lack clean validity/provenance boundaries

- **Evidence:** [adapter.py _read_min_front_range_m](../../src/ricebot/mujoco_sim/adapter.py#L788-L796) returns 4.0 when all three MuJoCo range rays are invalid, despite [contracts.py RobotObservation](../../src/ricebot/contracts.py#L78-L109) defining None as invalid. [adapter observation](../../src/ricebot/mujoco_sim/adapter.py#L797-L904) publishes pose from exact framepos, speed from exact qvel and peer distance from both true poses.
- **Consequence:** a missing obstacle reading is interpreted as clear, and idealised pose/speed/UWB may be mistaken for measured sensors. For a safety-critical rescue decision this is more than a naming nit.
- **Minimal fix:** return None for all invalid rays; represent any ideal virtual GNSS/IMU/odometry/UWB as explicit sensor channels with proper timestamp, frame, units, valid/invalid, noise/age assumptions. Keep simulator state available to the independent evaluator only. Backwards-compatible minimum fixes first; contract changes require one integrator.
- **Tests:** all-invalid rays, stale/time-shifted sensors, noise, occluded peer and sensor loss; conservative no-motion/no-rescue response.

### MAJOR M6 — “decentralised” nodes receive every peer's exact pose

- **Evidence:** [mycelial_swarm.py decide](../../src/ricebot/agent/mycelial_swarm.py#L805-L830) forms peer_poses from the entire ObservationFrame and supplies it directly to every local node. [process_local_sensors](../../src/ricebot/agent/mycelial_swarm.py#L385-L458) derives UWB ranges and bearings from that complete pose table; local SLAM filters hits and local controller uses peer positions regardless of isolated-map semantics.
- **Consequence:** separate local belief grids and message accounting exist, but the simulated robots are not informationally isolated at the pose layer when links fail.
- **Minimal fix:** session/sensor simulator emits *per-robot* permitted relative measurements (range/bearing/validity) and timestamped received packets. Each node receives its own observation and legitimate messages only. Central evaluator may assemble a complete frame but must not pass truth to local processing. Do not build real radios.
- **Tests:** change or withhold a peer's pose during blackout; isolated node must not observe the new peer pose until a valid permitted sensor/message; no unreported direct pose leakage.
- **Owner:** agent + adapter/contracts integrator.

### MAJOR M7 — soil probe returns exact hidden truth rather than a sensor reading process

- **Evidence:** [world/terrace.py sample_local_soil](../../src/ricebot/world/terrace.py#L462-L490) emits the exact _hidden_site_truth values when chassis position is within a site radius, once per robot/site, irrespective of an actual probe actuation/contact check. [robot mount](../../src/ricebot/robot/rovers.py#L241-L246) exists visually.
- **Consequence:** locality gate is a useful prototype, but there is no measurement noise, depth, independent probe actuation, uncertainty or realistic spatial measurement footprint. Do not claim measured actual soil properties or physical cone-index probe testing.
- **Minimal fix:** keep location gating and deterministic seeds, but add configurable measurement validity/noise/bias and actual probe-tip/contact eligibility for a later validated version. For immediate deadline, label it “synthetic local probe” and avoid invented physical accuracy.
- **Owner:** world/sensor integrator, no new soil simulator.

### MAJOR M8 — presentation metrics and ground truth are not strongly separated

- **Evidence:** [live_app.py fixed multiseed popup](../../src/ricebot/viewer/live_app.py#L841-L879) embeds numerical policy/yield/latency/wire-byte claims; corresponding referenced sim/benchmark_harness.py and docs/BENCHMARK_AND_STATUS.md were absent from audited git tree. [live_app.py benchmark](../../src/ricebot/viewer/live_app.py#L933-L1015) calculates some metrics from truth and model, but approximates continuous 10 Hz stream bytes via a constant. [viewer/dashboard.py export_bundle](../../src/ricebot/viewer/dashboard.py#L178-L207) correctly labels ground_truth_layers; make the same privilege separation explicit everywhere.
- **Consequence:** a polished popup can be mistaken for independently reproduced results. Truth overlays in themselves are legitimate evaluator views, but they must never feed control decisions and need labels.
- **Minimal fix:** remove/hide the hardcoded popup or mark “historical/unverified”; show run SHA, seed, policy and source for any measured values; distinguish sensed, estimated, modelled, evaluator truth and illustrative economics. When selecting an estimated elevation layer, [live_app.py](../../src/ricebot/viewer/live_app.py#L1286-L1326) actually uses evaluator terrain elevation: label it so the judge is not told it is a robot belief.
- **Owner:** evaluator + viewer.

### MINOR N1 — scenario materials may disagree with changed soil truth

- **Evidence:** [world/terrace.py terrain XML](../../src/ricebot/world/terrace.py#L525-L575) hardcodes mud/loam bed materials at patch_a/d versus b/c; [generate_hidden_soil_truth](../../src/ricebot/world/terrace.py#L88-L125) can shift viable patches with the scenario/seed.
- **Consequence:** an “unknown” surface can be visually pre-labelled mud/healthy even when scenario truth has changed; may leak a visual hint to a camera without matching the underlying simulation.
- **Minimal fix:** make materials follow scene setup or use neutral material for unknown terrain; verify camera cannot trivially infer hidden soil truth from a hard-coded color label.

### MINOR N2 — simulated camera is not operational agent perception yet

- **Evidence:** [robot/rovers.py camera asset](../../src/ricebot/robot/rovers.py#L276-L285) declares an ego camera, and [live_app.py renderer](../../src/ricebot/viewer/live_app.py#L1226-L1250) displays camera views. The robot ObservationFrame does not transport image/depth/detection results; the planner/control path is not vision-driven.
- **Consequence:** valid camera mount/visual demo is not proof of robot visual perception. Avoid adding a heavyweight VLM just for the word “AI”.
- **Minimal fix:** label it “camera viewpoint only” until a simple useful perception output (e.g. target-relative bearing with confidence) is actually computed from that view and consumed.

### MINOR N3 — “degraded sensor” scenario is selectable but has no degradation adapter

- **Evidence:** [live_app.py scenario selector](../../src/ricebot/viewer/live_app.py#L356-L390) includes sensor_degraded; [world/terrace.py generator](../../src/ricebot/world/terrace.py#L88-L108) treats it like nominal; inspected [adapter.py](../../src/ricebot/mujoco_sim/adapter.py) has no scenario-specific invalid/stale sensor injection.
- **Consequence:** presentation control overstates failure-handling coverage.
- **Minimal fix:** implement an explicitly configured dropout/noise scenario with sensor validity and tests, or remove the selector/claim temporarily.

## 4. Practical repair sequence — avoid a destructive architecture rewrite

**Priority 0 / immediate compatibility and truthfulness (small PR; preferred first agent assignment):**

1. On fresh owned branch, reconcile C1 and C2 to *one existing dataclass*. Add targeted tests that construct a bogged Mycelial frame with reachable helper and render both Mycelial and central rescue assessment without exceptions. C1 may also block the golden dashboard because the UI always updates Mycelial shadow state.
2. Correct M1 advisor API or disable that unsupported route visibly; do not silently label a deterministic decision “AI”. Stub offline/failure tests, no paid model calls.
3. Make dangerous display claims evidence-safe: remove hardcoded benchmark popup claims, label the viewer's ground-truth/estimated/synthetic channels. Make missing front-range return None if it does not break tests; add a regression check.
4. Actually run focused tests and **the untouched headless golden episode**, then locally launch the real --dashboard golden scenario and inspect a rescue frame. Record test commands, outputs, commit, screenshots/logs and residual regressions. If GUI/physics aren't available, say NOT RUN; do not claim success.

**Priority 1 / meaningful architecture corrections (separate PR(s), one integrator):**

5. Give headless and GUI a shared episode/session configuration. Move UI engine mutation and stepping behind one owner; support explicit experiment controls. Verify same-seed physics/command/output parity and that camera/belief view toggles do not change behaviour.
6. Add a strict simulator-truth → virtual sensor → per-robot observation → decision boundary; fix is_bogged oracle and peer pose leakage; maintain existing valid deterministic fault detector. Prove zero privileged-state dependence with differential tests.
7. Move terminal docking guidance into a bounded robot skill using permitted relative observations; MuJoCo only actuates/collides. Keep the old golden skill as a clearly labelled **simulator-assisted baseline** until the new skill passes contact/force/abort tests.

**Priority 2 / physical fidelity (separate, explicitly reviewed high-risk scope):**

8. Validate/migrate visual heightfield to physical contact and consistent soil forces; remove raw pose adjustment only when golden mechanics remain stable. Add multiple-seed/substep contact, force, roll/pitch and helper-survival checks.
9. Improve synthetic probe realism, actual camera input if needed, scenario-material consistency and honest sensor degradation only where measurable. No new Isaac/CFD/VLM/swarm platform.

**Submission guardrail:** If a risky fix makes the existing golden test or live video unreliable, keep the working pinned demo SHA and label remaining shortcuts rather than disguising or silently fixing by teleporting. Prioritise believable perception → safe rescue/abstain → observed physical outcome.

## 5. Acceptance and regression matrix (proposed, NOT RUN here)

| ID | Test to implement/run | Success means |
| --- | --- | --- |
| T1 | PR #50 Mycelial bogged/healthy reachable helper; same condition through side-by-side golden UI | No TypeError/AttributeError; identical canonical triage fields; no hidden frame crash |
| T2 | Mock advisor implements actual interface; missing key, offline, invalid recommendation | Honest called/not-called status; bounded deterministic safe fallback; no model-controlled torque |
| T3 | Hide simulator bogged flag while keeping realistic command/odometry evidence | Fault detection relies on measured mismatch, not oracle; HOLD/dropout/blocked approaches are not false rescue |
| T4 | Perturb unobserved target true pose; hold observation fixed | Robot controller command unchanged until permitted sensor updates; docking proceeds/aborts from permitted data |
| T5 | Compare headless and UI with identical policy/seed/scenario for every tick | Same commands, sim time, contact, recovery result; non-control UI operations never affect state |
| T6 | All three ranges invalid; GNSS/peer disconnected; stale timestamps | Unknown is None/invalid, never “4 m clear”; safe HOLD or explicit uncertain investigation |
| T7 | Peer isolated and physically moved; withheld message | Remote node cannot see current peer pose/map/contact by direct frame assembly |
| T8 | Terrain contact and fixed command replay (dry/soft/incline) | On-screen geometry equals collidable geometry; no physically unjustified qpos snap |
| T9 | Dock/extract multiple seeds and off-axis/over-force failures | Recovery actually moves victim, aborts unsafe cases, reports forces/contact *at substep resolution* and no second casualty |
| T10 | GUI truth display, benchmark popup, camera view and “sensor degraded” | Every displayed panel correctly states measured/estimated/evaluator-only; fixed historical results not passed off as live; degradation truly injected or hidden |
| T11 | Existing suites + real golden headless episode | No regression to known-good paths, recording possible from one authoritative scene |

### Suggested local verification commands (Windows PowerShell; adjust if environment differs)

~~~powershell
git status --short --branch
git fetch origin
git rev-parse HEAD
git rev-parse origin/main
git merge-base HEAD origin/main
git log --oneline -8
git diff --name-status HEAD...origin/main

# In an isolated testable checkout, after installed dependencies are verified:
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe -m unittest tests.test_mujoco_sim -v
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --scenario golden_rescue --headless
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --scenario golden_rescue --dashboard
~~~

If using bash/macOS/Linux, replace the Python executable with the actual supported venv path. Running these commands in the audited GitHub API session was **NOT** possible; the fixing agent should report real output from their environment. Check package installation and the authoritative runbook before assuming a venv exists.

## 6. Instructions for an agent working from a different revision

1. **Start with version detection, not automatic modifications.** Use git status/fetch/log/merge-base and list open PRs. If the checkout is behind origin/main, inspect the changed files and choose whether to start a fresh worktree from current main. Do not overwrite a teammate's dirty checkout, rebase uncommitted work, force push or overwrite an active owner.
2. **Compare code by semantic fingerprint, not line number.** Search for RescueAssessment declaration/constructor; is_bogged producer/consumer; exact peer_poses assembly; adapter docking-site steering; visual hfield collision; GUI private sim writes; benchmark-popup constants; advisor call signature. For each finding report PRESENT / FIXED / PARTIAL / DIFFERENT / NOT REPRODUCED with exact commit/path/line and test.
3. **Check PR #50 and its successors before repairing.** Older branch may lack Mycelial rescue/UI fields entirely; don't “fix” C1/C2 by adding a new partially compatible rescue path. Newer branch may have fixed them. Prefer current integrated main for the first isolated repair unless human integrator specifically says otherwise.
4. **Respect the sole-writer seam** for contracts.py/runtime.py/adapter.py and robot mechanics. If another teammate owns a file, prepare a minimal issue/PR suggestion instead of touching it; coordinate with human integrator.
5. **Build/merge incrementally.** First PR is narrow P0 compatibility, observable status and tests; subsequent PRs for viewer/session separation, perception/peer information isolation and physics contact. No monolithic refactor, no new simulation engine. Run the scoped tests and actual golden demo after each meaningful change.
6. **Report reality:** base SHA, updated HEAD, paths, commands/results, tested seed/scenario, frame-level evidence, remaining assumptions, rollback and exact PR URL. Human integrator approves merges; no paid API calls, credentials, external messages, submission or public deployment without permission.

**Best one-agent first task:** C1+C2+M1 on existing fields, then same-seed golden GUI/headless smoke checks. The larger P1 boundaries must not be declared fixed merely because those crashes are fixed.
