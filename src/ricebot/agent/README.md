# Decentralised Mycelial SLAM, Gossip & Task Negotiation (`src/ricebot/agent/`)

**Implementation scope:** the UWB ranging / Wi-Fi HaLow / ESP-NOW terminology refers to **simulated messages and distance/link models** within `DecentralizedMycelialRuntime`, not live radio hardware or separately deployed processors. Compare Policy A vs B only using tested same-seed scenarios and actual logged messages; the claimed >95% bandwidth saving in PR text requires a defined continuous-streaming baseline and comparable measurement before being treated as a verified result. No distributed LLM or integrated autonomous docking rescue is proven.

## 1. Purpose
Implements **Policy B (`DecentralizedMycelialRuntime`)**: a zero-central-database, peer-to-peer collective intelligence architecture where each rover maintains its own private **2D SLAM Occupancy & Traversability Grid (`LocalSlamGrid`)** and **Gaussian Soil/Stand Belief Map (`LocalMycelialMap`)**, educates reachable peers via **event-triggered Wi-Fi HaLow / UWB gossip (`MycelialGossipPacket`)**, and negotiates task ownership via **capability-aware local auctions (`TaskBidProposal`)**.

Operates over the exact same `SimulationPort` (`ObservationFrame` → `Decision`) seam as **Policy A (`MissionCore` in `src/ricebot/runtime.py`)** so both architectures can be compared head-to-head on identical physical episodes.

## 2. Public Entry Points
- `DecentralizedMycelialRuntime` in `src/ricebot/agent/mycelial_swarm.py`
  - `.decide(frame: ObservationFrame) -> Decision`
  - `.nodes: dict[str, DecentralizedRobotNode]` (inspect each rover's private `.local_map` and `.slam` grid)
  - `.gossip_history: list[MycelialGossipPacket]` and `.latest_sensor_telemetry: dict[str, OnboardSensorTelemetry]`
- `CooperativeSwarmPlanner`, `SwarmRobotAgent`, `RescueVerdict`, `GeminiSwarmAdvisor` in `src/ricebot/agent/planner.py`

## 3. Input and Output Contracts Used
- Consumes `ObservationFrame`, `RobotObservation`, `SoilSample`, `PlantFeedback`, and `Site` from [docs/CONTRACTS.md](../../../docs/CONTRACTS.md).
- Synthesizes the physical sensor outputs defined in [input_output.txt](../../../input_output.txt):
  - **IMU + AHRS (`ICM-42688-P`):** Pitch, roll, yaw orientation.
  - **True Ground Speed 60GHz mmWave Radar (`BGT60TR13C`):** True velocity $v_{\text{true}}$ vs. wheel surface speed $\omega r$ and slip ratio $s = 1 - v_{\text{true}} / (\omega r)$.
  - **Row Tracking (`Hall-Effect Whiskers + VL53L8CX ToF`):** Forward rangefinder ray hits and lateral crop clearance.
  - **Swarm Ranging (`Dual Qorvo DWM3000 UWB`):** Inter-robot distance $d_{ij}$ and horizontal Angle-of-Arrival ($\text{AoA}$).
  - **Swarm Comms (`Wi-Fi HaLow 802.11ah / ESP-NOW`):** Range-limited, event-triggered peer gossip packets with byte accounting and provenance deduplication (`seen_sample_ids`).

## 4. Prohibited Dependencies
- Must **never** read hidden ground-truth soil maps (`PaddyTerraceWorld._hidden_site_truth`).
- Must **never** share Python state across `DecentralizedRobotNode` instances except through explicit `MycelialGossipPacket` transmissions when $d_{ij} \le R_{\text{comm}}$ and the link is open.

## 5. Verification Command
```powershell
python -m unittest discover -s tests -p "test_mycelial_swarm.py" -v
python scripts/run_mujoco_paddypatch.py --policy mycelial --headless
python scripts/run_mujoco_paddypatch.py --compare-policies
```

## 6. Known Limitations
- In a 2-rover configuration (`planter` + `scout`), peer-to-peer gossip is single-hop between the two rovers; multi-hop relay across $>2$ hops requires instantiating a 3rd scout capability/body.
