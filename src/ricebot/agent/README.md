# Swarm Agentic Coordination & Edge Intelligence (`src/ricebot/agent/`)

**Scope:** Homogeneous Swarm Task Allocation, Rescue Triage, Decentralised Mycelial SLAM & Closed-Loop Sensor Reflexes.  
**Implementation status:** Decoupled from physical simulation (`SimulationPort`), field mapping (`MeasuredFieldMap`), and low-level actuation (`LocalController`).

---

## 1. Package Architecture

The `src/ricebot/agent/` package implements two complementary multi-robot coordination regimes operating across identical `rice.v1` contracts:

1. **Tier-1 / Tier-2 Swarm Auction & Rescue Triage (`planner.py`)**:
   * **Homogeneous Swarm:** All rovers share identical hardware capabilities via `homogeneous_capabilities(["rover-1", "rover-2", ...])` (both soil sampling and seed planting). Roles emerge dynamically based on field estimates, seed inventory, and proximity.
   * **Tier-1 On-Rover Micro-Edge:** High-rate decentralized utility bidding (<1 ms evaluation) resolving Explore (SAMPLE) vs. Exploit (PLANT) tasks with hard safety filtering.
   * **Rescue Triage & Pareto Optimization:** Evaluates immobilised/bogged peers to determine if rescue is advantageous (`RESCUE_NOW`, `DEFER_RESCUE`, or `ABANDON_ESCALATE`) under multi-objective weights (arrival time, energy, crop safety $C=0$, soil disturbance, and hazard risk).
   * **Tier-2 Deliberative Supervisor (`GeminiSwarmAdvisor`):** System-2 REST advisor triggered strictly on discrete surprise events (route obstruction, soil quality overturn, peer stranding) with instant safe fallback to Tier-1 when offline.

2. **Policy B — Zero-Central-Map Decentralised Mycelial SLAM (`mycelial_swarm.py`)**:
   * Independent per-rover **2D SLAM Occupancy Grids (`LocalSlamGrid`)** and **Gaussian Soil Belief Maps (`LocalMycelialMap`)**.
   * Event-triggered peer gossip over simulated Wi-Fi HaLow / UWB links (`MycelialGossipPacket`) with sample provenance deduplication (`seen_sample_ids`).
   * Capability-aware local auction negotiation without any centralized database or global coordinate broker.

---

## Online residual recovery candidate (merged in PR #57)

**Source and verification:** [PR #57](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/57), original [feature commit `dcdf0d9`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/commit/dcdf0d995d9d338aeed6edb9bb9133dde3264d71), merged into [`main` at `2848593`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/commit/2848593104d4946b94311a4fc28ede8fa2885237); [combined main CI passed 81 tests](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/actions/runs/42423011257). **This is an experimental learned technique selector, not the general fleet AI orchestrator, a deep neural network or deployed robot hardware.** No verified model-versus-deterministic rescue benefit or live multi-episode physical robustness is claimed.

- **`rescue.py` — `ResidualRecoveryLearner`:** per-technique online **linear** residual correction to simplified terramechanics displacement, using four bounded features: bias, moisture, soil softness and slip/stall. It predicts displacement for alternative techniques; `update(... actual_displacement_m ...)` adjusts weights using observed outcomes. Its default coefficients and update rule are engineering assumptions, **not validated pretrained weights**.
- **`rescue.py` — `RescueEngine`:** maintains stateful incident classification, risk estimates, rescue technique candidates and a method to supply recovery feedback. Some of its mechanical setpoint proposals assume actuator pitch/latch settings not independently accepted by the canonical M4-limited golden rescue.
- **`mycelial_swarm.py`:** nodes can exchange `DistressSignal.residual_weights` through **simulated** gossip. The receiver currently imports peer weights directly; this is a candidate state-sharing mechanism, **not secure/validated model aggregation**. No physical UWB/HaLow messages were transmitted in these tests.
- **`tests/test_agentic_swarm.py`:** tests the numerical weight update, technique re-ranking and direct peer-weight import. The test doesn't show real learned rescue improvement in the canonical MuJoCo golden episode; the caller must still supply the measured physical outcome.

**Integration contract for the next reconciliation:** keep **one `SimulationPort`** and the existing bounded command/force constraints. Review the two `RescueAssessment` / `RescueVerdict` definitions (in `planner.py` and `rescue.py`) and select a canonical schema; separate permitted robot observations from simulator `is_bogged`/hidden truth; validate dropout does not spur unintended towing; gate any learned technique behind deterministic safety veto, valid actuation geometry and a measured outcome. Do not wire proposed setpoints straight to MuJoCo actuators. Use [simulator variant registry](../../../docs/SIMULATOR-VARIANTS.md) and [boundary audit](../../../docs/audits/2026-11-07-simulation-perception-viewer.md) to plan the feature-level merge and tests.

---

## 2. Sensor Integration & Closed-Loop Reflexes

As specified in [input_output.txt](../../../input_output.txt), rover observations incorporate physical telemetry for reflexive safety in `src/ricebot/control.py`:
- **IMU + AHRS (`ICM-42688-P`):** Pitch and roll angles. When $|\text{pitch}|$ or $|\text{roll}| > 0.35$ rad (~20°), immediate drive cutoff prevents rollover.
- **Doppler Radar (`BGT60TR13C`):** True velocity $v_{\text{true}}$ compared to commanded speed. If slip ratio $s = 1 - v_{\text{true}} / v_{\text{cmd}} > 0.60$, anti-digging traction control throttles linear velocity.
- **Row Tracking (Whiskers & ToF):** Lateral cross-track displacement $e_{\text{row}}$ applies corrective proportional steering to keep rovers centered in crop furrows.
- **Swarm Ranging (UWB):** Inter-robot distances $d_{ij}$. Encroachment inside the 1.0 m safety envelope triggers autonomous collision braking.
- **Bogged Detection:** If wheel torque spikes with zero displacement, `is_bogged=True` halts traction to avoid entrenchment and signals rescue triage.

---

## 3. Public Entry Points

* **Planner:**
  * `CooperativeSwarmPlanner` in `src/ricebot/agent/planner.py`
  * `SwarmRobotAgent` in `src/ricebot/agent/planner.py`
  * `RescueVerdict`, `RescueAssessment` in `src/ricebot/agent/planner.py`
  * `GeminiSwarmAdvisor`, `AmbiguityEvent` in `src/ricebot/agent/planner.py`
* **Mycelial Swarm:**
  * `DecentralizedMycelialRuntime` in `src/ricebot/agent/mycelial_swarm.py`
  * `LocalMycelialMap`, `LocalSlamGrid`, `MycelialGossipPacket` in `src/ricebot/agent/mycelial_swarm.py`

---

## 4. Prohibited Dependencies

* Must **never** inspect hidden world truth (`_hidden_site_truth` or simulator internals).
* Must **never** bypass typed `rice.v1` contracts (`ObservationFrame`, `RobotCommand`, `TaskIntent`).
* Must **never** depend on synchronous cloud LLM availability for low-level motion, collision braking, or vehicle safety.

---

## 5. Verification Commands

```powershell
# Run Swarm Agent & Triage Unit Tests
python -m unittest tests.test_agentic_swarm -v

# Run Closed-Loop Sensor Reflex Tests
python -m unittest tests.test_sensor_reflexes -v

# Run Mycelial Swarm Tests
python -m unittest tests.test_mycelial_swarm -v

# Run Repository Integrity Checks
python scripts/check_repo.py
```
