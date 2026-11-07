# Rescue-first collective robotics — swarm, edge and inference architecture

**Update after merged PR #28 (7 Nov 2026):** `src/ricebot/agent/mycelial_swarm.py` now implements **simulated Policy B**: separate `LocalMycelialMap` and `LocalSlamGrid` instances for each rover, event-triggered `MycelialGossipPacket` exchanges with modelled distance/link blackout, provenance deduplication, and task bids. Tests include isolated vs connected maps, blackout recovery and changed planting target. It runs inside a shared Python/MuJoCo simulation; **no physical UWB or Wi-Fi HaLow radios, deployed distributed processes, Jetson inference or integrated autonomous physical rescue have been verified**. Policy A's `CooperativeSwarmPlanner` remains a centralised baseline. The merged PR #24 separate scripted MuJoCo rigid-docking/tandem extraction test is physical simulation mechanics, not the full sensed-rescue mission.

**Updated 7 November 2026.** The user explicitly clarifies that the **core product is intelligent physical robot-to-robot rescue**; the rice-field operation is a useful, measurable mission (local soil/stand sensing, mapping, seed planting where demonstrated). This **supersedes the earlier planting-first/rescue-optional framing**. Preserve `rice.v1`, MuJoCo physics, and working agricultural features. [PRODUCT](PRODUCT.md) / [D-0009](DECISIONS.md) / [research](RESCUE_EDGE_RESEARCH.md).

## Current implementation versus claimed capabilities

**PR #23 (merged):** `src/ricebot/agent/planner.py` contains `SwarmRobotAgent.generate_bids`, `CooperativeSwarmPlanner`, `evaluate_rescue_worth` with `RESCUE_NOW / DEFER_RESCUE / ABANDON_ESCALATE`, `homogeneous_capabilities`, and optional `GeminiSwarmAdvisor`. Existing tests in `tests/test_agentic_swarm.py` use scripted observations and task-intent assertions.

**Not implemented/verified:** physical P2P wireless hardware and independently deployed auction participants (the **simulated** Policy B now has distinct maps/gossip); **autonomous mission-triggered** physical extraction (a scripted MJCF test does exist); a learned physical extraction controller; Jetson-mounted LLMs; or a single LLM sharded across devices. **Policy A** centrally sorts bids against one shared map in one process, a useful deterministic baseline. **Policy B** has separate per-rover belief instances and simulated range-limited messages in one runtime; neither is evidence of real distributed radios. **Separately**, PR #24 merged `src/ricebot/robot/rovers.py` and the `paddy_ant_rigid_rescue.xml` asset with an actual **scripted MuJoCo tandem extraction test**: dock/latch/pitch-lift/reverse results in >0.08 m victim movement. This is **physical test evidence**, not an autonomous rescue workflow. Software `can_plant=True` for all robots does not mean physical robots have identical rescue tools.

**Critical honesty:** `front_range_m=None` indicates a missing/invalid sensor, **not proof of stuck wheels**. A `rescue_assist(...)` reason still assigns only a nearby `SAMPLE` waypoint. The **separate scripted docking test** does physically attach and move a peer, but the planner and skill are **not wired together**. `GeminiSwarmAdvisor` is an optional **internet REST API** dependent on user configuration, not an offline Jetson inference result. Its fallback tests use mocked advice.

## Proposed three-layer embodied architecture

**System 0 — local robot:** safety-critical motor limits, observed motion vs commanded motion, collision/traction monitoring, safe HOLD, and one approved helper recovery skill (bounded push/tow or attachment). Use a deterministic physical controller as baseline. A **learned** recovery policy requires actual training, checkpoint provenance, held-out friction/mass tests and measured extraction; never infer learning from a task label or scripted manoeuvre.

**System 1 — cooperative belief:** local observations with provenance, timestamps and uncertainty, task/health/capability claims. The current central map/auction is Policy A. **Simulated Policy B now exercises** independent per-node maps, event-triggered packets, deduplication, limited range and scheduled blackout scenarios; real radio/protocol transport, deployed node separation and robustness to hardware failures remain unverified. No hidden simulator ground truth may enter mission logic.

**System 2 — rare incident reasoner:** when a credible incident is detected, consider calculated probability/risk of rescue, possible second-casualty cost, available hardware/skills, remaining fieldwork and seed budget, qualitative operator priorities, and uncertain weather/terrain context. Decide **self-recover, investigate, send help, defer or abandon/escalate**. A deterministic safety/feasibility gate can reject every model suggestion; high-rate control never waits on an LLM.

## One supervisor, two decision timescales (proposal, not an existing subsystem)

**Live incident:** fleet objectives and qualitative operator preferences are interpreted at the mission level; deterministic tools compute travel, risk, resource opportunity costs and legal/safety limits. A proposed tool-using supervisor may request fresh status, compare candidate help/abstain plans, dispatch a **bounded** recovery skill, read simulator-confirmed progress and revise/abort if the first attempt stalls. That is more agentic than a single `GeminiSwarmAdvisor.advise(...)` call, but the current advisor is not such an implemented tool-runner. Keep motor-rate decisions local and safe if the model/API disappears.

**Start/end of simulated workday (follow-on, same supervisor):** inspect the same mission logs for coverage, redundant proximity, availability for rescue and losses; hypothesise a *bounded configuration change* (e.g., existing travel cost or scout peer-proximity preference), call the existing episode runner/evaluator on matched seeds, compare against the unchanged policy and deploy only if a validator approves; otherwise roll back and retain the negative result. This is not independent neural-network training or a second simulation. Do not add it before the physical rescue P0 is credible, and never imply the feedback loop is already implemented.

**Current scope freeze:** one MuJoCo simulator, one homogeneous rover asset, one small field, one tested physical rescue manoeuvre plus at most one sensor-triggered correction/abort, one selected coordination integration path (Policy A or simulated Policy B; the other remains a comparator), and one independent outcome grader. Four learned rescue skills, real terramechanics, separate SLAM world engines, full rice growth, Isaac translation, radio hardware and multi-Jetson inference are research or future extensions, **not independent weekend build targets**.

## Optional offline reasoning: do not confuse three different concepts

| Mode | Meaning | Key limitation |
| --- | --- | --- |
| Deterministic risk-aware optimiser | Offline numerical task and recovery policy | Strong mandatory baseline; may already solve the decision |
| Single local small LLM | Quantised model hosted on an available verified onboard/farm computer | Capacity and reasoning quality |
| **Model-parallel distributed LLM** | **One larger model**, layers/weights partitioned across several devices; activations pass between them during every generated token | Per-token communication, memory/KV overhead, worker loss |
| Multiple independent small models | Several separate policies exchange bids, arguments and observations | **Not** one larger neural network; consensus challenges |
| Farm-local PC | Optional offline hub | On-premise machine cost and dependency |
| Cloud API | Optional reference comparator, not core dependency | Internet, privacy, billing; must be explicitly approved |

**Rescue paradox:** if a disabled rover loses power or radio, it may also lose a required LLM shard. For a two-device split model, **the remaining rover may be unable to run it when rescue is requested**. A physically stuck rover can still contribute if its radio/computer works, but this cannot be assumed. Keep a **self-contained deterministic triage and safe-stop fallback** on every functioning robot.

## Jetson and inference research — verified sources, no team hardware claim

- [Original NVIDIA Jetson Nano](https://developer.nvidia.com/embedded/jetson-nano): **4 GB LPDDR4, Maxwell GPU with 128 CUDA cores, 25.6 GB/s**. Not interchangeable with Orin Nano.
- [Orin Nano Super Developer Kit](https://www.nvidia.com/en-gb/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/): **8 GB shared LPDDR5, 67 INT8 TOPS, 102 GB/s, 7–25 W**. NVIDIA's [current marketplace](https://marketplace.nvidia.com/en-us/enterprise/robotics-edge/jetson-orin-nano-super-developer-kit/) lists **US $399**; [December 2024 launch announcement](https://developer.nvidia.com/blog/nvidia-jetson-orin-nano-developer-kit-gets-a-super-boost/) described $249. Neither proves UK retail price or production robot economics. **Previous "$150 Jetson hub" claim was unsupported.**
- [llama.cpp RPC](https://github.com/ggml-org/llama.cpp/blob/master/tools/rpc/README.md): remote `ggml` compute across hosts, explicitly **fragile and insecure proof-of-concept**. Never expose an RPC server on public/sensitive networks.
- [CoSEI, IEEE TMC, published 11 Sep 2026](https://ieeexplore.ieee.org/abstract/document/11686281/): authors report up to **5.4× lower latency than conventional pipeline baselines** on an eight-node edge cluster; not a Jetson robot benchmark or a 5.4× gain over one large GPU.
- [Prism / Qazi et al., May 2026](https://arxiv.org/abs/2605.25682): **Jetson Orin Nano over Wi-Fi**, noting CPU↔GPU staging/network bottlenecks and adaptive partitioning; not proof all larger LLMs are practical.
- [RAYA, Sep 2026](https://raya-control.github.io/) investigates learned recoverability and bounded intervention; [IAOM, 2026](https://ieeexplore.ieee.org/abstract/document/11560910/) covers dynamic multi-robot task allocation. Both challenge novelty claims.

**No actual team-owned Jetson SKU/count, split LLM inference run, on-robot power budget, trained autonomous rescue policy or production cost is verified in the repository.** Ask human operators for models/JetPack/RAM/radio and permission before installations or purchases. A desktop simulated node is not an Orin experiment.

## Rescue-first implementation and evidence gates

1. **Actual physical incident:** field/world developer makes a reproducible immobilisation/traction/actuator failure, measured via permitted robot signals; test false sensor fault and stationary HOLD separately.
2. **Physical ability:** robot/mechanical owner defines one safe helper manoeuvre and actuator; simulate actual contacts, forces, displacement and second-robot risk. A rendezvous alone fails.
3. **Strategic decision:** agent owner compares no rescue, always rescue, and calibrated risk-aware deterministic policy with optional contextual model on matched incident inputs. Include cases in which refusal is correct.
4. **Task continuity:** return to genuine rice mapping/planting after recovery and grade independently. Document whether low-level policy was **trained** or classical.
5. **Optional parallel studies:** validate the now-implemented simulated partial-belief/gossip Policy B against Policy A and defined communication baselines; independently test single local quantised model, then real split inference on approved Jetsons with tokens/s, RAM, watts, RTT, network bytes, node failure. Neither blocks the physical rescue demo.
6. **Judge proof:** one continuous ≤60-second sensor → failure → evaluated intervention → actual physical assistance **or safe abstention** → measured mission consequence. A scripted physical dock/extraction check is verified by the merged test; no claim of a full rescue-first **autonomous mission** until perception, decision, command and recovered task are integrated and tested.

**Owned paths:** retain current kernel integrator as sole shared-schema writer, world owner for fault/soil, robot owner for mechanical recovery tool, planner owner for triage/decision, evaluator for independent scoring. ChatGPT/Gemini developer ownership is interchangeable. [Research experiment packet](RESCUE_EDGE_RESEARCH.md).
