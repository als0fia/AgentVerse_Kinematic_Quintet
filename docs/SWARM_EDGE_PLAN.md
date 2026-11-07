# Rescue-first collective robotics — swarm, edge and inference architecture

**Update after merged PR #28 (7 Nov 2026):** `src/ricebot/agent/mycelial_swarm.py` now implements **simulated Policy B**: separate `LocalMycelialMap` and `LocalSlamGrid` instances for each rover, event-triggered `MycelialGossipPacket` exchanges with modelled distance/link blackout, provenance deduplication, and task bids. Tests include isolated vs connected maps, blackout recovery and changed planting target. It runs inside a shared Python/MuJoCo simulation; **no physical UWB or Wi-Fi HaLow radios, deployed distributed processes, Jetson inference or integrated autonomous physical rescue have been verified**. Policy A's `CooperativeSwarmPlanner` remains a centralised baseline. The merged PR #24 separate scripted MuJoCo rigid-docking/tandem extraction test is physical simulation mechanics, not the full sensed-rescue mission.

**Updated 7 November 2026.** The user explicitly clarifies that the **core product is intelligent physical robot-to-robot rescue**; the rice-field operation is a useful, measurable mission (local soil/stand sensing, mapping, seed planting where demonstrated). This **supersedes the earlier planting-first/rescue-optional framing**. Preserve `rice.v1`, MuJoCo physics, and working agricultural features. [PRODUCT](PRODUCT.md) / [D-0009](DECISIONS.md) / [research](RESCUE_EDGE_RESEARCH.md).

## Active topology: local autonomy + nearby shared LLM server (D-0014)

**Chosen direction, 7 November 2026:** The LLM is hosted **once, on a shared server near the deployment**, not split layer-by-layer across each rover's Jetson. The actual server location is a site-specific choice:
1. **Farm or cooperative base station with capable GPU**: local IP connection from field gateway, no external internet required for LLM reasoning; higher capital, power and maintenance requirements.
2. **Regional on-premises/hosted inference server**: one server serves several farms over surveyed cellular/fixed backhaul, potentially lower per-farm cost; requires functioning backhaul and regional hosting/security review.
3. **Satellite broadband gateway backhaul**: fallback where terrestrial broadband is poor and **lawful service/plan exists at the farm address**; uplinks the *farm gateway*, not necessarily each rover. Never make motor safety or a continuous rescue servo depend on satellite. The country/region and available provider/coverage are not selected; verify live availability before deployment.

### Explicit data plane and authority boundary

~~~text
                          Operator mission priorities
                                    |
             Farm / cooperative gateway (packet router, queue, logs)
              |                                              |
       local field robot network                    local IP / cellular / fixed
     (surveyed Wi-Fi/HaLow, etc.)                  or licensed satellite backhaul
              |                                              |
    +---------+----------+                       Nearby GPU inference server
    |                    |                       (System 2 tool-using LLM)
 Rover A              Rover B                                  |
    |                    |                       Advisory proposals / evidence
 System 0: motor/physical safety                          [NOT TORQUES]
 System 1: local map + numerical triage + tasks               |
    |                    |                                    |
    +--- peer observations, bids and alerts -----------------+
                                    |
                  On-robot deterministic policy validator
               (physical limits, leases, capabilities, uncertainty)
                                    |
                        bounded authorised rescue skill
                                    |
                      measured contact/outcome/continuation
~~~

There is **one safety authority per robot**: local on-robot control and a validator. The server never receives a raw MuJoCo state oracle disguised as a sensor, never writes robot joints directly, and never becomes the sole owner of mission continuity. Each robot's local state and peer packets have individual timestamps and validity; the server receives only data the deployed robot/gateway can actually obtain.

**Three planes, deliberately separate:**
- **Robot/peer link:** local sampled observations, state estimates, compact gossip, bids, SOS and execution acknowledgements, with a bounded safety-oriented protocol. Use conventional 2.4/5 GHz Wi-Fi where coverage exists, consider 802.11ah Wi-Fi HaLow for surveyed longer-range low-bandwidth IP communication, or other locally lawful technologies. UWB is a *ranging* option with separate radio considerations; ESP-NOW is not the same as HaLow.
- **Gateway/backhaul link:** an ordinary encrypted IP uplink between field gateway and server, which may use LTE/4G/5G, wired/local WLAN or a legal satellite terminal. LoRaWAN's low data rate may be appropriate for tiny alerts/telemetry, not as an unexamined default for a high-bandwidth LLM session or video; country-specific spectrum and traffic limits apply.
- **Decision/actuation:** server returns bounded **strategic suggestions**, not streams of servo commands. Dedicated local controllers use onboard sensing and their approved robot rescue policy, whether learned or deterministic.

**Practical preference:** use a local gateway even when cellular coverage is good; it buffers/retries messages, records incidents and isolates robots from internet configuration. One nearby GPU host can be shared by farms, but do not assume a particular commercial provider, subscription, power budget or hardware ownership. Even a farm-side inference host can fail; every robot's System 0/1 remains independent of it.

### What the LLM actually does when summoned

An SOS is first diagnosed locally from actual issued commands, wheel/pose movement, IMU/slip/validity evidence, *not a direct simulator is_bogged truth flag*.

1. **Trigger and freeze context:** new credible fault, conflicting risk estimates, route/traction surprise, operator urgency or a failed rescue attempt. Local numeric planner immediately produces a default safe action. The server request is asynchronous: the episode ticks continue.
2. **Compact request to server:** schema version, incident/episode ID, robot IDs, timestamp, evidence+confidence, constrained local map/terrain estimates, available rescue tools/capabilities, operator priorities, local numerical risk/utility estimate, allowed action catalogue and decision deadline. Do not send secret keys, raw hidden-world state or video by default. Send an image/crop only if an actual sensor and bandwidth policy justify it.
3. **Bounded tool loop:** LLM can call a *read-only* calculate_rescue_risk, estimate_route_and_travel_cost, query_local_evidence, compare_counterfactuals and request_additional_measurement tool with limited iterations/deadlines. Numeric tools return units, assumptions, uncertainty and provenance. A request to gather more observations becomes a **local validated task**, not a raw sensor/motor instruction.
4. **Typed proposal:** RESCUE_NOW / INVESTIGATE / SELF_RECOVER / DEFER_RESCUE / ABANDON_ESCALATE plus selected helper/approved skill, short structured rationale, assumptions and an uncertainty field. It may request a new evaluation after a particular observed result.
5. **Local validation and execution:** physical feasibility, minimum evidence quality, available communications, no-go constraints, safety margins, target uniqueness, freshness, human requirements and bounded command authority are rechecked. Only validated intents translate into pretested robot skills. Rejected advice is logged with reason.
6. **Observe and revise:** local skill reports progress/contact/force/failure/rescue displacement. The server may propose a second safe experiment or abort, but local controller never waits for this during physical danger.

**Minimum message properties (design, not implemented):** schema_version, incident_id, episode_id, robot_id, robot_state_version, observed_at, expires_at, action_id, allowed_actions, source/provenance, safety_constraints, proposal_id and idempotent acceptance/rejection. Avoid global wall-clock assumptions; use local monotonic clocks and bounded uncertainty/time conversion. A late response for an earlier snapshot must be rejected even if its text sounds sensible.

**Existing code is not this service:** GeminiSwarmAdvisor currently makes one synchronous REST call (timeout bounded), supplies a task assignment schema and falls back when absent/invalid. It does not have the asynchronous queue/tool-loop/monitoring contract above; separate implementations are required. Current Mycelial advisor interface and rescue assessment mismatch are identified in [boundary audit](audits/2026-11-07-simulation-perception-viewer.md) and must be repaired first.

### Fallback mathematics — beyond a heuristic weighted sum

**Already implemented (simulation/prototype):** CooperativeSwarmPlanner evaluates rescue_benefit - opportunity_cost - hazard_risk using **hand-tuned numbers**; if adviser is absent, the deterministic planner still issues tasks and a bounded LocalController still drives/halts. Simulated Policy B computes local map updates, task bids and selective peer packet gossip. Neither is proven as a complete fault-tolerant real distributed autonomy system.

**Proposed improvement:** evaluate a small set of strategies via constrained expected outcomes rather than let unbounded arbitrary weights dominate safety decisions:

    strategy in {safe_hold, investigate, self_recover, rescue_with_helper_j, defer, escalate}
    expected_net_value(strategy) =
       expected_mission_value_preserved
       - expected_helper_damage_or_loss
       - intervention_time_and_energy
       - crop/field_damage_cost

    Subject to:
      P(second_casualty | evidence, strategy) <= approved risk budget;
      no-go, collision, force, tilt and actuator hard limits;
      sufficient fresh sensing and evidence; valid reachable helper;
      known authority/lease and communications assumptions.

If risk inputs are uncertain, use conservative intervals or robust worst-case evaluation, not a guessed precise probability. The calibrated numerical solver chooses the best **feasible** strategy and may choose investigate/defer/abstain. LLM output may influence qualitative operator priorities, hypothesised causes, which observation to request or counterfactual strategies, but **not waive constraints**. No AI benefit is claimed unless it beats this strong baseline on identical sensor information.

**“Emergent intelligence” boundary:** independent robot maps, gossip and auctions can create useful adaptive group behaviour without LLM calls. They do not magically supply missing sensor evidence, guaranteed global consensus, or risk calibration. During partitions, preserve local safety and task ownership leases; if a helper cannot establish target presence/route authority, do not dispatch it blindly.

### What happens when links or inference disappear?

| Disruption | Robot/field action | Required test |
| --- | --- | --- |
| Model API unconfigured, server dead or timeout | Local numerical planner selects conservative strategy; unchanged System 0 holds safety | Offline before incident |
| Cellular/satellite gateway unavailable | Peer/local radio and base maps may continue; queue alert for reconnect; no stale server decision | Outage before SOS / reconnect |
| Robot-peer network partition | Each rover retains its own sensor/estimate; avoid claiming global map/auctions or unverified rescue dispatch | Isolate after mapping / second robot moves |
| LLM reply arrives late, repeated or conflicting | Reject by incident/state/version/expiry and idempotency; log fallback | Delay/duplicate/out-of-order injection |
| Link drops during docking/towing | Bounded skill may proceed **only while permitted local sensors and explicit time/force constraints hold**, otherwise emergency stop/abort; never wait for server torque | Blackout at latch, under tow, after success |
| Sensor invalid, physically unsafe route or high uncertainty | Stop/investigate/defer/human escalation, even with confident LLM text | Dropout, risky helper, false stuck |
| LLM suggests unapproved command or fabricated numerical claim | Typed validator rejects, no motor effect | Adversarial/unparseable response |

### Evidence and incremental implementation

**Order:** (1) repair already-known A0–A6 boundary/interface issues; (2) establish a fully disconnected **calibrated deterministic** rescue/abstain baseline; (3) add event telemetry and async advisor protocol with fake/stub server; (4) connect one local LAN server and simulate LTE/satellite latency, jitter, loss and outages; (5) test real owned server hardware/connectivity only after host/site approvals; (6) add bounded tool-using LLM and compare actual benefit vs deterministic policy in MuJoCo/Isaac on matched incidents.

**No invented deployment metrics:** record actual numerical optimiser decision latency, model input/output tokens, time-to-first-response, server GPU/RAM, joules/request if instrumented, backhaul RTT/jitter/dropout, message size, recovery outcomes and second casualty. No demonstrated Jetson process distribution, satellite subscription, guaranteed radio range, cloud GPU or server uptime exists in the current repository.

**Sources for network feasibility, not procurement:** [GSMA Asia-Pacific 2025](https://www.gsma.com/solutions-and-impact/connectivity-for-good/mobile-economy/wp-content/uploads/2025/08/The-Mobile-Economy-Asia-Pacific-2025.pdf), [ITU remote-agriculture IoT architecture study (under study)](https://www.itu.int/ITU-T/workprog/wp_item.aspx?isn=24020), [Starlink current address-based availability](https://starlink.com/map) and [Starlink performance caveats](https://www.starlink.com/legal/documents/DOC-1431-92252-65). Regional licensing/availability vary; do not choose a default country or guarantee service.

## Current implementation versus claimed capabilities

**PR #23 (merged):** `src/ricebot/agent/planner.py` contains `SwarmRobotAgent.generate_bids`, `CooperativeSwarmPlanner`, `evaluate_rescue_worth` with `RESCUE_NOW / DEFER_RESCUE / ABANDON_ESCALATE`, `homogeneous_capabilities`, and optional `GeminiSwarmAdvisor`. Existing tests in `tests/test_agentic_swarm.py` use scripted observations and task-intent assertions.

**Not implemented/verified:** physical P2P wireless hardware and independently deployed auction participants (the **simulated** Policy B now has distinct maps/gossip); **autonomous mission-triggered** physical extraction (a scripted MJCF test does exist); a learned physical extraction controller; Jetson-mounted LLMs; or a single LLM sharded across devices. **Policy A** centrally sorts bids against one shared map in one process, a useful deterministic baseline. **Policy B** has separate per-rover belief instances and simulated range-limited messages in one runtime; neither is evidence of real distributed radios. **Separately**, PR #24 merged `src/ricebot/robot/rovers.py` and the `paddy_ant_rigid_rescue.xml` asset with an actual **scripted MuJoCo tandem extraction test**: dock/latch/pitch-lift/reverse results in >0.08 m victim movement. This is **physical test evidence**, not an autonomous rescue workflow. Software `can_plant=True` for all robots does not mean physical robots have identical rescue tools.

**Critical honesty:** `front_range_m=None` indicates a missing/invalid sensor, **not proof of stuck wheels**. A `rescue_assist(...)` reason still assigns only a nearby `SAMPLE` waypoint. The **separate scripted docking test** does physically attach and move a peer, but the planner and skill are **not wired together**. `GeminiSwarmAdvisor` is an optional **internet REST API** dependent on user configuration, not an offline Jetson inference result. Its fallback tests use mocked advice.

## Proposed three-layer embodied architecture

**System 0 — local robot:** safety-critical motor limits, observed motion vs commanded motion, collision/traction monitoring, safe HOLD, and one approved helper recovery skill (bounded push/tow or attachment). Use a deterministic physical controller as baseline. A **learned** recovery policy requires actual training, checkpoint provenance, held-out friction/mass tests and measured extraction; never infer learning from a task label or scripted manoeuvre.

**System 1 — cooperative belief:** local observations with provenance, timestamps and uncertainty, task/health/capability claims. The current central map/auction is Policy A. **Simulated Policy B now exercises** independent per-node maps, event-triggered packets, deduplication, limited range and scheduled blackout scenarios; real radio/protocol transport, deployed node separation and robustness to hardware failures remain unverified. No hidden simulator ground truth may enter mission logic.

**System 2 — rare incident reasoner:** when a credible incident is detected, consider calculated probability/risk of rescue, possible second-casualty cost, available hardware/skills, remaining fieldwork and seed budget, qualitative operator priorities, and uncertain weather/terrain context. Decide **self-recover, investigate, send help, defer or abandon/escalate**. A deterministic safety/feasibility gate can reject every model suggestion; high-rate control never waits on an LLM.

## Historical inference alternatives — archived, superseded by nearby-server choice D-0014

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
5. **Optional parallel studies:** validate simulated partial-belief/gossip Policy B against Policy A; build the nearby-server async advisory + local offline numerical fallback in D-0014 and benchmark it under latency/outage. Model-parallel multi-Jetson inference remains historical research only, not the selected deployment. Neither blocks the physical rescue demo.
6. **Judge proof:** one continuous ≤60-second sensor → failure → evaluated intervention → actual physical assistance **or safe abstention** → measured mission consequence. A scripted physical dock/extraction check is verified by the merged test; no claim of a full rescue-first **autonomous mission** until perception, decision, command and recovered task are integrated and tested.

**Owned paths:** retain current kernel integrator as sole shared-schema writer, world owner for fault/soil, robot owner for mechanical recovery tool, planner owner for triage/decision, evaluator for independent scoring. ChatGPT/Gemini developer ownership is interchangeable. [Research experiment packet](RESCUE_EDGE_RESEARCH.md).
