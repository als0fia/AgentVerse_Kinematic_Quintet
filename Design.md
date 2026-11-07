# Antenna Swarm Control Console — Design & Architecture (`Design.md`)

## 1. Overview & Alignment with Repository Specifications
The **Antenna Swarm Control Console** is the interactive multi-robot operator interface for the **AgentVerse Kinematic Quintet** (`https://github.com/als0fia/AgentVerse_Kinematic_Quintet`). It is aligned with the canonical engineering documents in the repository:
- `src/ricebot/viewer/README.md` & `src/ricebot/viewer/dashboard.py` (`antenna.dashboard.v1` schema)
- `src/ricebot/robot/README.md` (`make_homogeneous_rover_spec`, `paddy_ant_rigid_rescue.xml`)
- `docs/IMMOBILIZATION_SCENARIOS.md` (`S1`–`S8` immobilization hypotheses + Negative Controls)
- `docs/ANTENNA_RF_SUBSYSTEM.md` (ESP-NOW / Sub-1GHz Wi-Fi HaLow P2P gossip, 60GHz mmWave radar, BeiDou RTK)
- `docs/DECISIONS.md` (`D-0008`, `D-0009`, `D-0011` Multi-Objective Pareto Rescue Triage, `D-0012` Standalone Edge AI Visualizer & Dynamic Obstacle Injection)
- `src/riceswarm_ai/README.md` & `src/riceswarm_ai/visualizer.py`

---

## 2. Core User Interface Features

### 2.1 Multi-Robot Paddy Field Map (`src/components/FieldMapCanvas.tsx`)
- **Color-Coded Robot Position Dots**: Displays all 5 homogeneous Medic-Ant rovers (`ANT-01` through `ANT-05`) on the $24.0\text{ m} \times 16.0\text{ m}$ flooded paddy terrace (`Section A: Batch 1 Day -25` and `Section B: Batch 2 +20d Offset`):
  - `● Nominal` (`#10b981` Emerald): Active planting, scouting, or relay within safe slip bounds ($s < 0.42$).
  - `◆ Retrieving` (`#06b6d4` Cyan): Collinear rigid docking, cam-latch, and pitch-lift rescue in progress.
  - `▲ Alert / Slip` (`#f59e0b` Amber): Elevated wheel slip ($0.42 \le s < 0.60$) or furrow drift warning.
  - `✖ Immobilized` (`#ef4444` Crimson): Bogged down in deep mud (`S1`–`S8`), motor phase torque cut off.
  - `■ Human Control` (`#a855f7` Purple): Direct human teleoperation override or Global Fleet E-Stop engaged.
- **Continuous 2D Gaussian-RBF Belief Heatmap & Perspective Switcher (`antenna.dashboard.v1`)**:
  - **Perspectives**: `Planter Belief`, `Scout Belief`, `|Planter − Scout| Gap`, and `Ground Truth (Ref)` (strictly read-only visual comparator; never leaked to planners).
  - **Continuous Field Layers**: `Moisture M(x,y)`, `Firmness F(x,y)`, `Density ρ(x,y)`, `Stand Gap G(x,y)`, `Uncertainty σ(x,y)`, `Mud Risk R(x,y)`, and `Elevation z(x,y)` with live cursor $f(x,y)$ inspection.
- **Interactive Mid-Run Dynamic Obstacle Injection (`D-0012`)**:
  - Clicking **`+ Inject Obstacle (D-0012)`** inserts a sudden mid-sequence obstacle directly along an active robot's planned path, invalidates intersecting routes, and triggers immediate A* multi-waypoint detour replanning visible on the map.

### 2.2 Explicit Natural-Language Robot Plans (`src/components/RobotRosterPanel.tsx` & `src/components/PlanTimelinePanel.tsx`)
- Every robot displays its explicit natural-language mission plan and step-by-step execution queue (e.g., *"Robot 1 is going to retrieve Robot 3 via collinear rigid docking, cam-latch & pitch-lift at (14.4m, 8.4m)"*).
- **Homogeneous Medic-Ant Dynamic Role Switching**: All 5 rovers share the identical `270×190×64 mm` sealed hull (`276 mm` outer wheel width fitting `<300 mm` rice inter-row lanes). When a rover's seed hopper reaches `0/20`, it automatically transitions from `PLANT` to `SAMPLE` (Scout) mode.

### 2.3 Emergency Human Control Mode & Rigid Docking Actuators (`src/components/EmergencyControlPanel.tsx`)
- **Per-Robot Human Control Override** & **Global Fleet E-Stop (All 5 Units)**.
- **Directional Teleop D-Pad (WASD / Arrow Keys)** with Low-RPM Mud Crawl speed limiter (`25%`–`100%`).
- **Combined Rigid Docking, Cam-Latch & Pry-Lift Actuators (`paddy_ant_rigid_rescue.xml`)**:
  - **Cam-Latch Jaw Toggle** (`LOCKED 0.60 rad` / `OPEN 0.00 rad`).
  - **Front Pitch-Lift Boom** (`-0.20` to `+0.45 rad`) to pry a stranded peer's belly out of thixotropic clay suction before backing out collinearly.
  - **Rear Deployable Hardpan Anchor Spades** (`0.85 rad` planted / `0.00 rad` stowed).
  - **Micro-Rocking Clay Breaker** (`S6`) & **Reverse-Polarity Straw Clearing** (`S5`).

### 2.4 Interactive Bekker–Wong Terramechanics Lab & P2P Mycelial Gossip (`src/components/BekkerTerramechanicsLab.tsx`)
- **Bekker–Wong Wheel–Mud Sinkage Calculator**: Interactive sliders for `Vehicle Mass m (1.5–18.0 kg, default 4.2 kg)`, `Wheel Radius r (35–110 mm, default 58 mm)`, `Wheel Width b (20–90 mm, default 40 mm)`, and `Local Mud Moisture M(x,y) (0.20–0.96)` computing live `Wheel Sinkage z (mm)`, `Belly Clearance (mm)`, `Mud Drag Rc (N)`, and Traversability Verdict.
- **P2P Mycelial Gossip Log (`MycelialGossipPacket`)** + **`Export dashboard.v1 JSON`** action.

### 2.5 Immobilization Hypotheses (`S1`–`S8`), Negative Controls & Pareto Triage (`src/components/ScenarioTriagePanel.tsx`)
- Simulates `S1` through `S8` plus explicit Negative Controls (`NEG_MISSING_LIDAR_FAULT` where `front_range_m = None`, and `NEG_INTENTIONAL_SEED_HOLD`) that prevent false-positive rescue dispatches.
- Interactive sliders for the 5 Pareto Rescue Cost Weights (`D-0011`): $J = w_t T + w_e E + w_c C + w_s S + w_r R$.
