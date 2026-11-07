# Antenna — Progressive Five-Agent Reconciliation

**Architecture and execution plan, 7 November 2026.** Source review of main at `cb68abb68bf9d00d7b4debbd3b71c52103243483`; this is a snapshot, **not** a freeze for the future reconciliation. P0–P5 development and teammate branches can advance meanwhile. This plan is documentation only; five coding AIs are not automatically running or assigned. A human integrator approves actual implementation ownership, interface changes, reviews and merges.

**Objective:** end with ONE running Antenna rescue-first system, which retains valuable code and visual work from different teammates and accepts the later human-authored agent architecture. Do not merge five different end-to-end simulators or accidentally undo P0–P5 improvements.

**Start here:** [root agent rules](../AGENTS.md), [product](PRODUCT.md), [architecture](ARCHITECTURE.md), [contracts](CONTRACTS.md), [variant registry](SIMULATOR-VARIANTS.md), [team workflow](TEAM.md), [boundary audit](audits/2026-11-07-simulation-perception-viewer.md), [tests](TESTING.md), [Isaac migration](MUJOCO-PROTOTYPE-AND-ISAAC-TRANSLATION.md). The variant registry remains the **only authoritative per-feature adopt/adapt/park/reject index**. This file is the execution protocol, not a duplicate inventory.

## 1. Does the simulator visualiser need to be kernel-centred?

**Centred on the kernel's authority and data, yes. Embedded inside the kernel, no.** The current `src/ricebot/runtime.py` already defines a narrow `SimulationPort` (reset/step), `MissionCore` (observe/plan/control), `EpisodeRunner`, and a `ReadOnlyView` subscriber. The current `src/ricebot/viewer/dashboard.py` recorder broadly fits this idea.

**Source-confirmed current architectural gap:** the GUI `src/ricebot/viewer/live_app.py` creates its own simulation/agent instances, calls `sim.step` and sometimes two decisions per frame, and changes model parameters through simulator internals. The CLI runner `scripts/run_mujoco_paddypatch.py` also owns a separate loop. This is **not** proven GUI/headless parity. The [boundary audit](audits/2026-11-07-simulation-perception-viewer.md) details further sensor-truth and control leakage.

**Target runtime:** one session owns episode identity, clock, reset, scenario events, physics step, permitted sensor outputs, selected fleet policy, validated commands and recorded results. The existing typed Python kernel remains the shared interface; it should not be confused with a second physics simulator or a requirement for all front ends to be Python.

~~~text
  operator configuration / logged scenario events
                     |
             ONE EpisodeSession
                     |
  ONE SimulationPort per episode: MuJoCo OR Isaac/PhysX
                     |
  sensor bridge: observed measurements / confidence / timestamps
                     |
  Antenna core: maps, coordination, numerical fallback, optional LLM
                     |
           bounded safety-validated actions
                     |
              physics + feedback
                     |
      typed telemetry + grader-only independent truth
            /                 |                 \
      3D renderer       2D/web viewer       evaluator/replay
~~~

Viewers are **subscribers**, and may be separate processes or languages (desktop Tk, React, Isaac viewport). They can send *typed operator requests* (pause, reset, fault injection, parameter experiment) into a session command queue; only the session owner validates/applies them and logs the action. A user control must **not** directly write MuJoCo `qpos`, physics material values, planner belief, or wheel commands. A recording or map must visibly distinguish the robot's estimated belief from independent simulator truth.

No need to integrate the legacy six-robot lifecycle clock or a mock TypeScript physics loop into the physical rescue scene. Reuse **features** (layouts, incident inspector, route planning, visual styles, sensors, legitimate rescue skills) after provenance and compatibility review, not whole conflicting application engines.

## 2. Five AI workstreams — domain specialists, not five competing integrators

Start each AI with a **read-only inventory of its domain**. After the interface owner approves the exact task and exclusive files, each may work on a separate branch/worktree. One shared contract writer; no concurrent writable working tree.

| Agent | Responsibility | Typical exclusive scope | Must prove |
| --- | --- | --- | --- |
| **A — Core/session/integration interfaces** | Single episode runner, schema evolution, trace/replay, GUI/headless behaviour parity, cross-domain contract approvals | `src/ricebot/runtime.py`, `contracts.py`, shared entrypoint + contract tests; sole owner | One physics step and one chosen policy decision per tick; consistent CLI/GUI run; typed trace and backwards-compatible interfaces |
| **B — Physics/robot/mechanical rescue** | Reconcile robot geometries, terramechanics, collider/terrain, seed mechanism, legitimate dock/latch/pull skill; optional isolated Isaac adapter | `src/ricebot/robot/`, `mujoco_sim/`, mechanical fixtures; coordinate world collider ownership with C | No teleport-based recovery, actual collision/force/displacement, realistic limits, helper safety, reproducible simulated extraction |
| **C — Sensing/world/peer information** | Observation validity, local terrain/probe/vision evidence, estimated fault recognition, independent per-rover belief, gossip, blackout and gateway test transport | `src/ricebot/world/`, sensor adapters, per-robot observation/comms fixtures; shared `mycelial_swarm.py` edits must be handed off to D | No privileged bogged flag or global peer pose to local policy; negative tests for HOLD, dropout and stale communications |
| **D — Fleet agent/decision/learning** | Accept human teammate's upcoming complete agent proposal, reconcile mathematical fallback, residual learner, Policy A/B, bounded recovery action choice, optional nearby LLM service, Isaac Lab skill tasks | `src/ricebot/agent/`, decision+server tests; consumes A's contracts and B's approved robot skill interface | Offline safe choice, actual tool-use/LLM validation, compare strong deterministic baseline, learned control only with checkpoints/held-out evidence |
| **E — UI/evaluation/demo** | Reconcile Tk/HTML/React/3D best visual features as viewers; implement truth-labelled metric display, real experiment logs and judging scenarios | `src/ricebot/viewer/`, evaluator, independent front-end/test fixtures; no sim or core ownership | Same-session real state; no fabricated charts, no hidden world feedback to robots, continuous verified rescue/abstention episode |

**Shared file exceptions:** `mycelial_swarm.py` needs C/D coordination, `world/terrace.py` needs B/C coordination, `runtime.py` remains A-only; mechanical actuator/agent instruction schemas are agreed by A, B and D before changes. If boundaries overlap, agents **first produce an interface proposal + fixture**, then one owner implements it. Five AIs can work concurrently without five AIs editing the same API.

**Review distribution:** after each domain PR, another agent reviews boundaries and tests (A↔E, B↔C, D↔A). Final human integrator checks merge order, verifies no lost features and runs the combined scene. The reviewer is not a second writer on the author's branch.

## 3. Two continuous lanes: P0–P5 now, reconciliation later

**Lane 1 — active feature development continues.** Team programmers and their existing AIs keep finishing P0–P5 tasks in owned branches. They do not need to stop or wait for a grand redesign. Their PR should contain a short feature receipt:

~~~text
Capability ID / user-visible outcome:
Source owner, source branch + exact commit SHA:
Current main baseline and unique delta (not already merged):
Touched files, API/contract version, dependencies:
Real observations vs privileged simulator truth:
Test command, host, seed, log or video, result and known failures:
Feature status: in progress / ready for review / integrated / superseded:
Why it matters to the rescue-first product:
~~~

**Lane 2 — reconciliation preparation keeps reading.** Agents A–E inventory their own domains and compare each new PR/source branch **read-only**, recording adopt/adapt/park/reject proposals via the single [variant registry](SIMULATOR-VARIANTS.md) steward. A newly pushed commit invalidates an earlier approval until the new diff/tests are assessed.

**Version rules:**
1. Each *source feature* is referenced by exact commit SHA and actual diff. The PR tip may move without notice. Unpushed local work does not appear on GitHub: ask the human owner before assuming it is absent.
2. Each integration PR starts from **fresh main at integration time**, not the November 7 snapshot above. If main advanced, re-evaluate overlap, preserve already merged functionality and rerun the affected tests. Avoid blind cherry-picks/branch merges.
3. Promotion to **INTEGRATED** means merged into canonical main *and* supported by actual specified combined tests; a historical feature branch should not be merged again.
4. A newly introduced P0–P5 feature can be implemented directly on main through a reviewed PR and marked already integrated. The later reconciliation must inspect and keep it, not overwrite it with an older version.
5. Keep one report per feature in its PR, one shared registry, one canonical integration test plan. Do not create multiple competing “merged simulator” repos or stale mega-branches.

**Current overlap to check:** open [PR #54](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/54) includes M1 camera/ramp and physical rescue changes and has active updates; do not overwrite/rebase it without owner agreement. [PR #57](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/57) has **already landed** a linear residual recovery learner in `src/ricebot/agent/rescue.py`; it is not a general learned robotics policy or a second main simulator. PR #58 adds research/contract cautions. Revalidate current SHA when starting.

**Incoming complete agent architecture:** D should intake the teammate's diagram/source/notes as a **candidate**, compare against actual code and numerical rescue baseline, and propose the smallest set of typed incident/tool/action/feedback seams. A owns shared contract approval; B confirms physical skill feasibility, C confirms each input's sensor provenance, E writes independent tests. Do not delay independent P0/P1 work or “integrate” unimplemented agent promises as code.

## 4. P0–P5 allocation and dependencies

| Existing task group | Lead(s) | Work that can start now | Integration gate |
| --- | --- | --- | --- |
| **P0** repair boundary failures, golden rescue, UI/CLI divergence | A + B + C + E | A fixes session+type bugs; B physical docking; C honest fault evidence; E viewer parity | Independent same-scene rescue/abstain evidence and no hidden agent oracle |
| **P1** offline constrained rescue, per-robot mapping/blackout ([P1 research & proposal](P1-OFFLINE-INTELLIGENCE-RESEARCH-AND-PROPOSAL.md)) | C + D | D numeric strategy/uncertainty; C peer validity and isolation | With LLM disabled, no second casualty from unapproved actions; required negative controls |
| **P2** incident protocol, asynchronous server, numeric tool loop | D with A/C interface review | Build fake-server timeout and stale-response tests now | No server controls torque; expiry, safety veto and logging verified |
| **P3** agricultural network backhaul and regional feasibility | C + D + human | Site/country-specific research, fake latency/packet-loss fixtures | No unsupported satellite promises, no purchases or infrastructure without approval |
| **P4** Isaac physics, trained docking and visual upgrade | B + D + E | Small Isaac/Isaac Lab prototype **separate from working MuJoCo scene**; measure resources | Actual executable physics, contact/sensor traces, policy checkpoint if trained |
| **P5** comparison baselines, result logs, demo | E + D + A | Define scenario bank and metrics now; run repeated offline tests | Measured outcomes; physics/AI attribution; dependable single continuous demo |

**Early contract handshake:** when two agents need to exchange a new incident, uncertain peer pose, recovery phase or advisor result, A writes a minimal typed fixture (field, units, coordinate frame, timestamp, confidence, ownership, deadline and validation) and merges it first. Other agents implement against that tested interface. This is a **small dependency**, not a pause on all other development.

## 5. The MSI is a candidate lightweight Isaac host, not an officially supported full Isaac host

User-reported workstation: AMD Ryzen AI 9 HX 370, **32 GB RAM, RTX 5070 Ti Laptop GPU with 12 GB VRAM**, ample SSD free space; actual Windows version, NVIDIA driver and simulator compatibility **not yet checked**.

NVIDIA [Isaac Sim 6.2 requirements](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/requirements.html) list **32 GB RAM, 16 GB VRAM and an RTX 4080-class GPU** as minimum supported x86-64 workstation configuration, plus a validated driver and Windows 11 or supported Ubuntu. This MSI is below the published GPU/VRAM minimum, despite otherwise capable CPU/RAM/storage. Do not call installation or GPU performance verified by its model number alone. Test the official Compatibility Checker; record `winver`, `nvidia-smi`, exact Isaac version, power/thermal state, and crash/VRAM logs.

**Permitted exploratory approach (does not replace the canonical demo until validated):**
1. Start with **one** articulated rover and a real flat collidable rigid surface, actual wheel drives/joints, a single simple physics range sensor and 1 camera only if needed. No complex vegetation, enormous USD assets, path tracing, multi-camera RTX/LiDAR or broad parallel environments.
2. Keep **physics correct before visuals**: rigid contacts, wheel traction, obstacle contact, masses/inertia, bounded torques, stable step size and physical force/contact checks. First use the engine's simple physics sensors; no scripted teleport or truth-flag masquerading as perception. A painted visual mud patch **does not imply physically accurate bogging**.
3. Compare headless/no-rendering and NVIDIA [RTX Minimal rendering](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/reference_material/rendering_modes.html) on the host. Minimise texture size/resolution, lights, sensor count and offscreen buffers. Lower graphics can help VRAM but does **not** guarantee that the 12 GB device supports Isaac Sim or that physics is realistic.
4. Add second rover, validated latch/towing and small collidable terrain changes only if the smoke test runs reliably. Log resource use and physics-step force/motion errors. Physically model/responsibly label soil traction rather than claiming wet soil from visual appearance.
5. For **learning-first** if full Isaac is blocked, [Isaac Lab 3.0](https://docs.nvidia.com/isaaclab/3.0/isaac-lab/installation) runs with the **Newton backend without full Isaac Sim/RTX**; benchmark a **small headless articulated policy task** on that workstation, and add visuals separately. Different backends need independent physical validation. Do not promise full Isaac/Isaac Lab training throughput or model parity.
6. Keep MuJoCo as tested simulation backend/fallback. A separate Isaac adapter is promoted **only** after physics, sensor, command, demo and resource gates pass. If the MSI fails at startup, report the blocker rather than bypassing system requirements with a false “success.”

**Hardware task acceptance:** report compatibility result, executable version, installed driver, RAM/VRAM peak, per-step speed, continuous 1-rover motion, measured contact/range change, deterministic reset and whether any GUI rendering worked. No spending on cloud GPU without explicit approval.

## 6. Reconciliation phases and decision gates

- **R0 — parallel source audit:** Five agents separately produce a per-domain feature map from actual current main, live PR tips, source owners, unique commits, screenshots/logs if supplied, known tests and boundary violations. Each states **ADOPT / ADAPT / PARK / REJECT / IN PROGRESS**; an actively developed source is not silently rewritten.
- **R1 — contract integration:** A chooses one episode session and stable sensor/command/agent API with fixtures and backward compatibility. Review by B/C/D/E. Independent features continue on own branches.
- **R2 — parallel domain patches:** B merges reviewed physics/mechanics, C sensor/network, D agent and learning, E viewer/evaluator, each on independent up-to-date branches and against pre-agreed interfaces. Prefer *small sequential accepted PRs* per domain rather than a single cross-cutting merge; keep A as the sole kernel writer.
- **R3 — integration candidate:** A/human integrator assembles merged features from newest main. Each specialist verifies its own capability survived, and another AI reviews interactions outside its domain. Check GUI vs headless same seed, physically observable rescue/abstain, safe numerical fallback, LLM-offline, false sensor alarms, relay partitions, mechanical force, true resume and evaluator metrics.
- **R4 — freeze demonstrable release:** name the exact tested commit, seed, scenario, environment, model/provider status and run command; capture a genuine continuous episode and keep a known-good MuJoCo fallback. Feature branches remain available for future changes, but no untested work lands in the recorded release candidate.

**Stop/punt rules:** never trade away a confirmed working physical rescue for unverified newer visuals, a made-up benchmark, a scripted judge scene or a claimed LLM workflow without real tool calls. An AI may report uncertainty and recommend parking a feature. Never give an untrusted model the authority to override robot safety. Judge-facing evidence must distinguish real results from estimates, demonstrations and suggestions.

## 7. Five ready-to-assign briefs

These can be handed to **five separately launched coding AIs**. Before implementation, the human integrator assigns specific unique branches, file owners and fixtures. First pass is read-only; not permission to change human teammates' work.

**AI A — Core/session:** “From fresh main, audit every place that creates, resets, advances or writes the physical simulation and agent decisions across CLI, dashboard, standalone scenes and alternate UI. Propose one shared EpisodeSession/SimulationPort lifecycle and typed telemetry/command seam; identify breaking changes, adapters, GUI/headless parity fixtures and exact owned files. Do not change another agent's modules without a contract proposal. Report the implementation PR sequence and a smoke test.”

**AI B — Physics/mechanics:** “From fresh main and open M1 PR status, inventory every unique rover/world/traction/latch/ramp/rescue mechanical feature across merged and candidate versions. Check actual contacts and physical measurements, preserve agreed chassis, and flag simulator-directed geometry/teleport shortcuts. Propose adopt/adapt/park with source SHAs and tests. If authorised for an Isaac spike on a verified workstation, start only with one wheel-actuated rover and measured simple collider/range, not a full visual rewrite.”

**AI C — Sensor/network:** “Inventory true sensors vs simulator truth, field/probe and camera perception, per-rover SLAM and gossip, peer ranging/occlusion, dropout and connectivity across existing branches. Specify timestamp/uncertainty/provenance tests and no-oracle integration proposals. Prepare separate fault/blackout fixtures without editing the agent-policy source while it is assigned elsewhere.”

**AI D — Agent/learning:** “Read the present agent/planner/residual-learning stack and the human teammate's architecture when delivered. Compare the numerical rescue baseline, offline P0/P1, proposed nearby asynchronous LLM tool loop and optional learned rescue skill against current bounded contracts and physical outcomes. Detect duplicate rescue types and untrusted model-weight gossip; propose only measured, validated additions. Deliver a deterministic-vs-LLM-vs-model-off test plan and unique code ownership.”

**AI E — UI/evaluator:** “Inventory Tk dashboard, HTML export, main MuJoCo HUD and independent React UI. Extract visual/design features without importing parallel physics or sample state. Propose how all views observe a single session, handle logged operator events, show each robot's belief separately from hidden truth and report physically measured benchmark outcomes. Define a single continuous rescue/abstain demo test and whether Isaac low-graphics RTX Minimal is sufficient.”

**Required final receipt from each AI:** feature matrix with exact source SHA; what is already merged and newly changed; touched-path recommendation; interface conflicts/owner approvals; commands/tests actually run with logs; all unverified assumptions; minimal PR plan; independent evidence needed. One full system, five domain specialists, one human merge authority.
