# Rescue-first collective robotics — swarm, edge and inference architecture

**Updated 7 November 2026.** The user explicitly clarifies that the **core product is intelligent physical robot-to-robot rescue**; the rice-field operation is a useful, measurable mission (local soil/stand sensing, mapping, seed planting where demonstrated). This **supersedes the earlier planting-first/rescue-optional framing**. Preserve `rice.v1`, MuJoCo physics, and working agricultural features. [PRODUCT](PRODUCT.md) / [D-0008](DECISIONS.md) / [research](RESCUE_EDGE_RESEARCH.md).

## Current implementation versus claimed capabilities

**PR #23 (merged):** `src/ricebot/agent/planner.py` contains `SwarmRobotAgent.generate_bids`, `CooperativeSwarmPlanner`, `evaluate_rescue_worth` with `RESCUE_NOW / DEFER_RESCUE / ABANDON_ESCALATE`, `homogeneous_capabilities`, and optional `GeminiSwarmAdvisor`. Existing tests in `tests/test_agentic_swarm.py` use scripted observations and task-intent assertions.

**Not implemented/verified:** a true P2P wireless auction; independently maintained per-robot maps and gossip; physically towing/pushing a disabled rover; a learned physical extraction controller; Jetson-mounted LLMs; or a single LLM sharded across devices. The current planner centrally sorts robot bids against **one shared map in one process**. It is a useful deterministic baseline, not a fully decentralised swarm. Software `can_plant=True` for all robots does not mean physical robots have identical rescue tools.

**Critical honesty:** `front_range_m=None` indicates a missing/invalid sensor, **not proof of stuck wheels**. A `rescue_assist(...)` reason currently assigns a helper a nearby `SAMPLE` waypoint; it is **not physically assisting, attaching to or freeing** the other rover. `GeminiSwarmAdvisor` is an optional **internet REST API** dependent on user configuration, not an offline Jetson inference result. Its fallback tests use mocked advice.

## Proposed three-layer embodied architecture

**System 0 — local robot:** safety-critical motor limits, observed motion vs commanded motion, collision/traction monitoring, safe HOLD, and one approved helper recovery skill (bounded push/tow or attachment). Use a deterministic physical controller as baseline. A **learned** recovery policy requires actual training, checkpoint provenance, held-out friction/mass tests and measured extraction; never infer learning from a task label or scripted manoeuvre.

**System 1 — cooperative belief:** local observations with provenance, timestamps and uncertainty, task/health/capability claims. The current central map/auction is the baseline. Proposed distributed form uses **independent partial belief maps**, limited-range event-triggered messages, duplicate-sample suppression, explicit task reservation and partition handling. No hidden simulator ground truth may enter mission logic.

**System 2 — rare incident reasoner:** when a credible incident is detected, consider calculated probability/risk of rescue, possible second-casualty cost, available hardware/skills, remaining fieldwork and seed budget, qualitative operator priorities, and uncertain weather/terrain context. Decide **self-recover, investigate, send help, defer or abandon/escalate**. A deterministic safety/feasibility gate can reject every model suggestion; high-rate control never waits on an LLM.

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

**No actual team-owned Jetson SKU/count, split LLM inference run, on-robot power budget, trained rescue policy or production cost is verified in the repository.** Ask human operators for models/JetPack/RAM/radio and permission before installations or purchases. A desktop simulated node is not an Orin experiment.

## Rescue-first implementation and evidence gates

1. **Actual physical incident:** field/world developer makes a reproducible immobilisation/traction/actuator failure, measured via permitted robot signals; test false sensor fault and stationary HOLD separately.
2. **Physical ability:** robot/mechanical owner defines one safe helper manoeuvre and actuator; simulate actual contacts, forces, displacement and second-robot risk. A rendezvous alone fails.
3. **Strategic decision:** agent owner compares no rescue, always rescue, and calibrated risk-aware deterministic policy with optional contextual model on matched incident inputs. Include cases in which refusal is correct.
4. **Task continuity:** return to genuine rice mapping/planting after recovery and grade independently. Document whether low-level policy was **trained** or classical.
5. **Optional parallel studies:** per-node partial beliefs/gossip with communication dropout; single local quantised model, then real split inference on approved Jetsons with tokens/s, RAM, watts, RTT, network bytes, node failure. Neither blocks the physical rescue demo.
6. **Judge proof:** one continuous ≤60-second sensor → failure → evaluated intervention → actual physical assistance **or safe abstention** → measured mission consequence. No claim of a full rescue-first product until physical action has been verified.

**Owned paths:** retain current kernel integrator as sole shared-schema writer, world owner for fault/soil, robot owner for mechanical recovery tool, planner owner for triage/decision, evaluator for independent scoring. ChatGPT/Gemini developer ownership is interchangeable. [Research experiment packet](RESCUE_EDGE_RESEARCH.md).
