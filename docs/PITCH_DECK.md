# Antenna: 5-Minute Pitch Deck (Shortlist Presentation)

**Track:** AgentVerse 2026 — Embodied AI & Robotics  
**Product:** Antenna — Rescue-First Collective Robotics for Extreme Paddy Agriculture  
**Verified Judging Rubric Alignment:** Perception (25%), Adaptation (25%), Autonomy (15%), Relevance & Impact (20%), Ambition & Technical Quality (15%)  
**Presentation Time:** 5 Minutes (300 seconds)  
**Presenter:** Team Antenna  

---

## Slide 1: The Critical Human Need & Terramechanics Reality
**Timing:** 0:00 – 0:50 (50 seconds)  
**Visual:** High-contrast split slide: Real flooded paddy field with a disabled agricultural machine sinking in deep silt vs. 3D MuJoCo Bekker-Wong soil model showing wheel sinkage ($z > 0.04\,\text{m}$) and belly-out drag.

* **The Problem:** Wet Direct-Seeded Rice (WDSR) produces 40% of global grain but faces a severe agricultural labor crisis in East/Southeast Asia and erratic extreme monsoons.
* **Why Conventional Field Robotics Fails:** Puddled paddy soils exhibit non-Newtonian plastic rheology. A single rover operating alone inevitably encounters an unmapped silt sinkhole, suffers excessive wheel slip ($s > 0.85$), bellies out, and becomes an abandoned asset that blocks irrigation and ruins crops.
* **Our Core Thesis:** Autonomous agriculture is untenable without **risk-aware collective rescue**. Antenna transforms robots from isolated brittle machines into a resilient, self-recovering mycelial swarm.

---

## Slide 2: The Physical Innovation — Homogeneous "Paddy Ant" Embodiment
**Timing:** 0:50 – 1:40 (50 seconds)  
**Visual:** 3D exploded view of the homogeneous Paddy Ant chassis highlighting the 4-in-1 toolhead: conical seed dropper, soil penetrometer dibber, underbelly rotary weeders, and the rear/front **dual-action cam-latch and pitch-lift rescue coupler**.

* **Homogeneous Modularity:** Every rover in the swarm shares identical chassis mechanics, electronics, and firmware:
  * **Chassis:** 4-wheel skid-steer with deep grouser lugs for terramechanics tractive thrust.
  * **Sensing:** 3-beam forward LiDAR rangefinder, downward soil moisture & firmness penetrometer, Doppler slip estimator, and multi-node UWB ranging & AoA transceiver.
  * **Mechanical Rescue Coupler:** A passive rear strike-bar mated to an active front cam-latch ($F_{\text{latch}} > 35\,\text{N}$) backed by a high-torque pitch-lift actuator. When latched, the helper lifts the victim's drive axle by $4^\circ$ to break hydrostatic mud suction before applying synchronized reverse tandem towing.

---

## Slide 3: Hierarchical Architecture — Deterministic Reflexes Meets Swarm Intelligence
**Timing:** 1:40 – 2:30 (50 seconds)  
**Visual:** 2-tier architecture diagram showing the millisecond deterministic control loop on-device vs. the asynchronous event-driven Mycelial Swarm Gossip layer.

* **Why an LLM Alone Cannot Drive Motors:** Direct torque generation by neural networks leads to catastrophic digging in mud. We enforce a strict separation of concerns:
  1. **Tier 1 — Deterministic Edge Reflexes (On-Device, < 5 ms):**
     * Anti-digging traction throttle (cuts wheel spin when Doppler slip exceeds threshold).
     * Bekker sinkage and stability monitor (auto-cuts drive if chassis pitch/roll exceeds safety limits).
     * UWB anti-collision mutual avoidance.
  2. **Tier 2 — Asynchronous Decentralized Gossip (Policy B, Event-Driven):**
     * Peer-to-peer UWB packet exchange (no central cloud or base station).
     * Continuous 2D Gaussian Process spatial belief fields for soil moisture, bearing firmness, and WDSR planting fitness $\Phi(x, y)$.
  3. **Tier 3 — Autonomous Triage:** Evaluates stranded distance, battery budget, Bekker pull resistance, and peer risk to output typed verdicts: `RESCUE_NOW`, `DEFER_RESCUE`, or `SAFE_ABSTAIN`.

---

## Slide 4: Live 60-Second Episode Replay & Adaptation to Change
**Timing:** 2:30 – 3:45 (75 seconds)  
**Visual:** Single continuous simulation video playing on screen with real-time HUD telemetry, Doppler slip, and distress beaconing.

* **Phase 1 (Autonomous Fieldwork):** Scout and Planter explore separate zones, mapping stand gaps and soil firmness over decentralized UWB gossip packets (820 bytes total vs 35.8 kB for centralized RPC).
* **Phase 2 (The Mid-Episode Disturbance):** At $t = 5.6\,\text{s}$, the Planter rover hits an unmapped deep silt depression ($F = 0.28$, $M = 0.92$). Drive wheels slip, chassis sinks, and forward velocity drops to zero.
* **Phase 3 (Autonomous Triage):** Scout receives distress beacon, computes extraction mechanics ($\Delta F = 24.8\,\text{N}$, well within the dual-rover tractive capacity of $48\,\text{N}$), and commits to `RESCUE_NOW`.
* **Phase 4 (Physical Rigid Docking & Extraction):** Scout approaches, guides into the rear strike bar, locks the cam-latch, activates pitch-lift, and tandem-extracts the Planter $0.18\,\text{m}$ clear of the mud basin.
* **Phase 5 (Mission Resumption):** Cam-latch uncouples; Planter safely navigates to Patch B and Patch C, dispensing seeds with verified physical contact at $t = 13.84\,\text{s}$ and $t = 17.94\,\text{s}$.

---

## Slide 5: Quantitative Proof & Baseline Comparison
**Timing:** 3:45 – 4:25 (40 seconds)  
**Visual:** Benchmark table comparing 8 evaluated scenarios across autonomy, communication resilience, and mission success.

| Metric / Scenario | Single Rover Baseline | Centralized Policy A | Decentralized Policy B (Antenna) | Golden Rescue Episode (Antenna) |
| :--- | :---: | :---: | :---: | :---: |
| **Mission Success (Viable Seeds Planted)** | 0 / 2 (stalls in mud) | 2 / 2 (fails in blackout) | **2 / 2 (fully resilient)** | **2 / 2 (after mud extraction)** |
| **Comms Traffic Overhead** | N/A | 35,840 bytes (280 pkts) | **820 bytes (6 pkts)** | 49,920 bytes (triage + full mission) |
| **Resilience to RF Link Blackout** | N/A | Fails (aborts mission) | **100% (converges on reconnect)** | 100% |
| **Mud Immobilization Recovery** | 0% (Fatal stall) | 0% (No physical rescue) | N/A | **100% (Verified 0.18 m tow & resume)** |
| **Mid-Episode Dynamic Obstacle Response** | Blind collision | Safe stop only | **Autonomous SLAM reroute** | Coordinated replanning |

* **Key Takeaway:** Antenna is the **only** evaluated architecture that survives both communication severance and physical mud immobilization.

---

## Slide 6: Edge Feasibility & Commercial Roadmap
**Timing:** 4:25 – 5:00 (35 seconds)  
**Visual:** Physical hardware specs: NVIDIA Jetson Orin Nano Super edge computer, Semtech UWB module, low-power LoRa fallback, and real-world farm deployment unit economics.

* **Edge Compute Budget:**
  * Deterministic C++/Python kinematics and Bekker observers: < 2.5% CPU utilization on Jetson Nano.
  * Spatial GPR / Mycelial gossip updates: 12 ms inference latency.
  * No cloud dependency: 100% offline operational autonomy.
* **Unit Economics:**
  * Target manufacturing BOM: ~$1,450 per homogeneous Paddy Ant.
  * Payback period for smallholder cooperative: < 1.4 harvest seasons through reduced seed loss and labor savings.
* **Conclusion:** Antenna delivers real embodied intelligence where it matters most: keeping robots working, adapting to extreme environments, and recovering teammates when physical nature pushes back.

---

## Speaker Defense Q&A Cheatsheet (For Judges' Tough Questions)

1. **"Why not just build one large, heavy tractor that doesn't get stuck?"**
   * *Answer:* Soil compaction! Heavy machinery destroys the delicate hardpan layer in rice paddies, causing catastrophic water percolation loss. Lightweight (~4 kg) swarms eliminate soil degradation, and their collective towing capacity equals heavy machinery without the destructive ground pressure.
2. **"Does an LLM control the motors?"**
   * *Answer:* Absolutely not. In physical terramechanics, unconstrained token generation would cause runaway wheel digging. Deterministic skid-steer controllers and Terramechanics observers control motor currents; the AI handles multi-agent task auctions, spatial belief fusion, and risk triage.
3. **"What if both rovers get stuck?"**
   * *Answer:* Our triage model explicitly computes the helper's terrain risk. If the soil leading to the stranded rover has firmness below threshold $F_{\text{crit}} < 0.30$, the helper outputs `SAFE_ABSTAIN`, deploys an anchor spade, and relays a human dispatch distress beacon rather than risking a double immobilization.
