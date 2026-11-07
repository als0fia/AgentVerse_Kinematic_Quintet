# RiceSwarm AI — legacy, abstract full-season simulator (NOT Antenna's submission runtime)

**Repository status:** Retained for teammate experiments and historical reproducibility, **not** the current Antenna P0 application. The actual rescue-first MuJoCo physics/docking and rice.v1 runtime are in [`src/ricebot/`](../ricebot/) and [root README](../../README.md). This full-season six-robot environment models abstract grid/task events; its rescue decision is **not** the tested physical rigid-docking/extraction or the autonomous end-to-end episode. Do not build a parallel simulator for the submission or cite its synthetic planting/yield figures as agronomic performance.

**RiceSwarm AI** is an autonomous multi-agent robotic coordination system for sustainable rice farming in flooded Chinese paddy environments (田埂 dikes, flooded irrigation, variable puddling resistance).

## System Highlights

- **Complete Agricultural Calendar & Condition-Based Lifecycle (Days -25 to 145+):**
  - **Days -25 to -15:** Nursery establishment (seed inventory sowing, nursery moisture maintenance).
  - **Days -15 to -7:** Seedling development & health inspection.
  - **Days -7 to -1:** Field preparation & levelling, obstacle inspection, irrigation depth checks (0.05–0.10m).
  - **Day 0+:** Transplanting (condition-gated on seedling readiness, field prep completion, and empty targets).
  - **Days 1 to 7:** Early establishment & rooting monitoring.
  - **Days 8 to 14:** Establishment gap-filling (replacing missing/failed seedlings).
  - **Days 15 to 55:** Tillering & precision mechanical weeding (triggered when observed weed coverage $\ge 15\%$).
  - **Days 56 to 85:** Panicle development, heading & flowering water protection.
  - **Days 86 to 105:** Grain filling & crop maturity tracking.
  - **Days 106 to 115:** Ripening & pre-harvest drainage (lowering water depth to $\le 0.02$m for firm ground traction).
  - **Days 116 to 125+:** Harvesting (condition-gated on grain maturity $\ge 0.95$ and dry drained field).
  - **Post-harvest:** Yield recording, stubble management, and season performance reset.
- **Two-Section Paddy Plot with Staggered Batches:**
  - **Section A (Columns 1–5):** Starts at Day -25, transplant target at Day 0.
  - **Section B (Columns 6–10):** Staggered offset of +20 days (starts Day -5, transplant target Day 20).
  - Dynamically shares swarm capacity across different developmental stages (e.g. harvesting Section A while ripening/weeding Section B).
- **Swarm of 6 Homogeneous Multi-Functional Robots (`RS1`–`RS6`):**
  - Every robot simultaneously scans sensor telemetry (water depth, temperature, soil NPK/pH, weed density, mud risk), plants seedlings, mechanically weeds stands, inspects crop health, harvests mature grain, and performs peer rescues.
- **Continuous Environmental Monitoring & Risk Tiers:**
  - Continuous water tracking (depth, temperature, pH, trends).
  - 4-Tier mud risk categorization:
    - **LOW:** 0.00 – 0.29
    - **MODERATE:** 0.30 – 0.59
    - **HIGH:** 0.60 – 0.79
    - **VERY HIGH:** 0.80 – 1.00
  - Soil fertility tracking (tested at Days -7, 30, 60, 90 or post heavy-rain event).
- **Multi-Objective Mud Rescue Optimization:**
  - When a robot is immobilised in soft mud:
    $$J = w_t T + w_e E + w_c C + w_s S + w_r R$$
    Evaluating arrival time ($T$), energy consumption ($E$), crop damage ($C$, zero-tolerance constraint), soil disturbance ($S$), and recovery risk ($R$).
- **Strict Separation of Ground Truth vs. Swarm Belief:**
  - The AI coordinator operates exclusively on observed beliefs and confidence-scored maps.

---

## Quick Start & Execution

### 1. Run Interactive Visualizer (Tkinter GUI)

```bash
python src/riceswarm_ai/run_demo.py --gui
```
Or directly:
```bash
PYTHONPATH=src python -m riceswarm_ai.cli --gui
```

**GUI Controls:**
- `▶ Play Season` / `⏸ Pause`: Automatically steps through the multi-month agricultural calendar.
- `⏭ Advance 1 Day`: Steps a full 24-hour agricultural cycle (weather, crop biological growth, weed proliferation, condition evaluation, robot task batch).
- `Step Tick`: Fine-grained discrete robot movement tick.
- `↺ Reset`: Reinitialises the simulation.
- `⚠ Mud Stuck (RS2)`: Simulates a soft mud wheel-slip event to trigger real-time peer rescue coordination.
- `Reveal Hidden Ground Truth`: Toggles between swarm belief map and true environment state.
- Interactive multi-objective weight sliders ($w_t, w_e, w_c, w_s, w_r$).

---

### 2. Run Headless Full-Season Simulation (CLI)

```bash
python src/riceswarm_ai/run_demo.py --season --days 140
```
Outputs complete seasonal metrics in JSON and saves event traces:
```json
{
  "phase": "active",
  "day": 140,
  "weather": "rainy",
  "ticks": 4125,
  "crop_targets": 12,
  "planted": 12,
  "weeded": 11,
  "harvested": 10,
  "section_a": {
    "stage": "harvested",
    "planted": 6,
    "harvested": 5,
    "yield_kg": 4.25
  },
  "section_b": {
    "stage": "harvest_ready",
    "planted": 6,
    "harvested": 5,
    "yield_kg": 4.25
  },
  "rescue_attempts": 0,
  "successful_recoveries": 0,
  "human_escalations": 0,
  "unsafe_crop_movements": 0
}
```

---

### 3. Run Automated Unit Tests

```powershell
$env:PYTHONPATH = 'src'
python -m unittest discover -s src/riceswarm_ai/tests -v
```
These tests are included in the repository's CI suite, but verify the *abstract legacy lifecycle rules* rather than physical rescue, true field yields or the Antenna integrated mission. Run the commands on your own host before asserting live desktop-GUI success.
