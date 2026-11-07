# Rescue-first collective robotics: falsifiable research and edge-compute preflight

**Update after merged PR #28 (7 Nov 2026):** `src/ricebot/agent/mycelial_swarm.py` now implements **simulated Policy B**: separate `LocalMycelialMap` and `LocalSlamGrid` instances for each rover, event-triggered `MycelialGossipPacket` exchanges with modelled distance/link blackout, provenance deduplication, and task bids. Tests include isolated vs connected maps, blackout recovery and changed planting target. It runs inside a shared Python/MuJoCo simulation; **no physical UWB or Wi-Fi HaLow radios, deployed distributed processes, Jetson inference or integrated autonomous physical rescue have been verified**. Policy A's `CooperativeSwarmPlanner` remains a centralised baseline. The merged PR #24 separate scripted MuJoCo rigid-docking/tandem extraction test is physical simulation mechanics, not the full sensed-rescue mission.

**Research synthesis 7 November 2026.** User's confirmed goal: the collective identifies a robot failure, integrates numerical risk and qualitative mission urgency, and selects/executes physical peer assistance or safe refusal. Rice-field mapping and planting provide realistic useful work rather than the primary invention. No Jetson hardware or pooled LLM has been proven. **PR #24 supplies a scripted MuJoCo rigid docking/cam-latch/tandem extraction test that asserts >0.08 m victim movement; a complete autonomous mission-triggered rescue is unverified.** [Plan](SWARM_EDGE_PLAN.md) / [product](PRODUCT.md).

## What merits research next

| Priority | Experiment | Comparator | Observable proof |
| --- | --- | --- | --- |
| **P0** physical helper | One robot physically extracts another from a constrained stuck episode | no help, bounded deterministic tow/push | true physics contact/displacement; neither/one/both robots failed |
| **P0** risky decision | Rescue now, investigate, defer or safely abandon under uncertain soil and qualitative urgency | never rescue, always rescue, **calibrated risk optimiser** | secondary loss, mission value, regret, time/energy, safe abstention |
| **P0** continuing work | Successful recovery enables further sensing/mapping/planting | one multifunction rover and current field policy without recovery | map accuracy/freshness or physical seed contact after intervention |
| **P1** learned skill | Train locomotion/contact skill across changed mass, traction and geometry | classical bounded controller | checkpoint, training setup, unseen-seed success and safety |
| **P1** local contextual model | Does slow edge inference actually outperform optimised thresholds for ambiguous incidents? | same data, model off, small local model | matched outcomes, token latency, watts, confidence and no-network |
| **P2** P2P map and bids | Robots operate on partial local beliefs, gossip, delay/partitions | existing one-process shared-map auction | per-robot maps, unique evidence IDs, message bytes and collision-free assignments |
| **P2** distributed LLM | One quantised model sharded across actual constrained devices | single device, offline farm box, several independent reasoners | RAM/KV, time-to-first-token, tokens/s, communication, watts, node-loss |

**Four separate hypotheses:** physical rescue, agentic triage, decentralised information sharing and distributed model-parallel inference. A working example of one does not prove the others.

## State of the existing team code

At the reviewed commit after PR #23, `CooperativeSwarmPlanner` centrally sorts all robot bids against one `MeasuredFieldMap`, not over peer radio. `evaluate_rescue_worth` uses hand-set seed/distance/hazard coefficients; `front_range_m=None` is treated as stranded despite missing range also meaning sensor failure. `rescue_assist` routes a helper via `SAMPLE`; there is **a separate scripted MuJoCo physical docking/extraction test** from PR #24, but **no planner-connected bounded recovery action or independent mission-level recovery-result contract**. Optional `GeminiSwarmAdvisor` calls the internet REST API when configured; mocked tests do not establish offline intelligence. No distributed Jetson inference, **real radio-connected** maps or **trained** extraction controller. Simulated Policy B partial maps and gossip now exist. Scripted rigid-dock mechanics exist. Treat those features as **unimplemented**, not bugs already repaired by this research doc.

## Technical sources and limits

1. [NVIDIA original Jetson Nano](https://developer.nvidia.com/embedded/jetson-nano) vs [Jetson Orin Nano Super](https://www.nvidia.com/en-gb/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/): radically different generation and 4 vs 8 GB memory. Current [NVIDIA marketplace](https://marketplace.nvidia.com/en-us/enterprise/robotics-edge/jetson-orin-nano-super-developer-kit/) US $399 dev-kit listing, older [2024 launch](https://developer.nvidia.com/blog/nvidia-jetson-orin-nano-developer-kit-gets-a-super-boost/) $249. Neither confirms ownership or farm BOM. On Orin, CPU/GPU share memory; account for OS, vision, motor controllers, model weights, KV cache and buffers.
2. [CoSEI, IEEE Transactions on Mobile Computing, published 11 Sep 2026](https://ieeexplore.ieee.org/abstract/document/11686281/): optimised distributed LLM partitioning/synchronisation. **Authors' up-to-5.4× gain is versus conventional eight-node pipeline baseline**; not direct evidence for our Jetsons, Wi-Fi links or rescue quality.
3. [Prism — profiling-driven distributed Transformers on embedded Jetson Orin Nano Wi-Fi, May 2026](https://arxiv.org/abs/2605.25682): real CPU/GPU staging and network-exchange bottlenecks. Adaptive compression/partitioning reduces overhead relative to full-tensor static splitting; does not make every large LLM fast or robust.
4. [llama.cpp RPC](https://github.com/ggml-org/llama.cpp/blob/master/tools/rpc/README.md): experimental remote offload; maintainers warn it is **fragile/insecure**. Research only on private permitted lab network. A broken radio/model shard can halt the whole model.
5. [RAYA, Sept 2026](https://raya-control.github.io/): physically recoverable motion with a learned-analytic controller, relevant prior art but **not** multi-rover rice towing. [IAOM 2026](https://ieeexplore.ieee.org/abstract/document/11560910/): learned dynamic robot task allocation/coalitions, so auctions and recovery coordination alone are not novel.
6. [IRRI rice establishment](https://www.knowledgebank.irri.org/images/docs/planting-establishment.pdf) and [robot+probe spatial mapping prior art](https://www.sciencedirect.com/science/article/pii/S0168169925010385): rice-field relevance and alternatives; **synthetic soil scores are not validated agricultural yield predictors**.

## Required failure tests

- A robot *stops by command* vs actually stalls despite drive command vs only loses its front sensor vs loses electricity/radio. Never equate these.
- Rescuer physically able to push/tow vs insufficient force/reach, hazardous approach, second casualty or bad weather/traction proxy.
- Delayed/stale/duplicated peer observations; private hidden-truth only for evaluator, not policy.
- Safe recovery choice with no helper, high-risk site, no local model, API timeout and no communication.
- **Split-model paradox:** a two-node LLM may lose its weights/compute when one rover fails; require a functioning local fallback and measure node drop *during* incident.
- Novelty baselines: physically competent classical controller; risk-aware optimisation; single rover vs multiple rovers; independently running local small models vs actual sharded larger model. A more articulate explanation does not establish better decisions.

## Resource and implementation order

1. **Immediately:** world+robot owners build physical stuck/traction/contact recovery; keep existing MuJoCo rice sensor/seed work. Independent grader compares a real recovered vs unrecovered robot.
2. **Then:** integrator designs a *backwards-compatible* typed incident/recovery action-result seam. Agent owner builds numeric baseline then optional contextual reasoner; safety gate checks intervention feasibility.
3. **Optional separate compute research:** confirm actual Jetson SKU/JetPack/device count/network and permission; first benchmark single-device offline model, then model partition over real devices. Record quantisation, compatible runtime, tokens/s, time to first token, RAM, link throughput and energy. Never pretend desktop emulation is verified Jetson.
4. **Finally:** verify and profile the now-implemented **simulated** per-robot partial maps and gossip against Policy A, then research real radio/deployed-node semantics and leader/partition protocols. Keep kernel and working physics uninterrupted.

**No hardware purchases, paid inference, public network service, deployment or final hackathon submission authorised by this document.**
