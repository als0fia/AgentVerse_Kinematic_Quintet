# Antenna Swarm Control Console — Design & Architecture Document

## 1. Overview & Purpose

**Antenna** (`AgentVerse_Kinematic_Quintet`) is a collaborative multi-robot system engineered to automate precision rice farming in flooded paddy environments. Operating as a **Kinematic Quintet** (`ANT-01` through `ANT-05`), the fleet maps submerged paddy fields, samples soil moisture and firmness via synthetic and physical probes, detects missing crop hills, coordinates movement with sub-1GHz Wi-Fi HaLow / ESP-NOW and Dual-Antenna UWB ranging, and performs targeted seedling/seed replanting and cooperative winch recovery.

This document specifies the design, feature architecture, data contracts, and operational workflows of the **Antenna Swarm Control Console**.

---

## 2. Core User Interface Features

### 2.1 Multi-Robot Paddy Field Map (`src/components/FieldMapCanvas.tsx`)
- **Real-Time Robot Position Dots**: Displays each robot in the Kinematic Quintet on a metric coordinate grid ($24\text{ m} \times 16\text{ m}$ flooded terrace) with distinct color-coded status markers, heading indicators, UWB ranging envelopes, and callsign labels.
- **Color-Coded Status System**:
  - **Nominal / Active (`#10B981` Emerald)**: Actively planting seeds, sampling soil moisture, or navigating along crop furrows (`● Nominal`).
  - **Retrieving / Cooperative Task (`#06B6D4` Cyan)**: Dispatched to retrieve a stuck peer, towing via INA240 strain-monitored winch tether, or returning to seed depot (`◆ Retrieving`).
  - **Traction / Slip Warning (`#F59E0B` Amber)**: Elevated kinematic wheel slip ratio ($s > 0.45$) or narrow furrow clearance warning (`▲ Slip Warning`).
  - **Immobilized / Fault (`#EF4444` Crimson)**: Bogged down in plow-pan breach (`S1`), straddling an earthen bund (`S3`), trench drop-in (`S4`), or mechanical straw stall (`S5`) (`✖ Immobilized`).
  - **Human Control / Emergency Override (`#A855F7` Violet)**: Direct operator teleoperation or emergency hold active (`■ Human Control`).
- **Visualized Robot Plans & Trajectories**: Renders dashed trajectory vectors from each robot's current pose $(x, y, \psi)$ to its target waypoint, plus high-tension winch tether vectors between rescuing and stuck rovers.
- **Interactive Field Cells & Crop Sites**: Shows measured soil moisture/firmness cells (`FieldBeliefV1`), missing crop gap sites requiring replanting, established rice rows, earthen bund ridges, and irrigation drainage trenches. Operators can click any map coordinate or crop site to inspect soil belief data or dispatch a selected robot directly to that target.

### 2.2 Robot Status Roster & Plan Inspector (`src/components/RobotRosterPanel.tsx` & `src/components/PlanTimelinePanel.tsx`)
- **Explicit Natural-Language Plan Statements**: Clearly states each robot's current mission intent and next steps (e.g., *"ANT-01 (Atlas) is going to retrieve ANT-03 via UWB winch tow at (14.2m, 8.6m)"*, *"ANT-02 (Boreal) is going to plant missing crop site C-04"*).
- **Contract-Compliant Task Assignment (`TaskIntentV1`)**: Allows operators to reassign any robot to `PLANT`, `SAMPLE`, `RETRIEVE_PEER`, `NAVIGATE`, `BEACON_RELAY`, or `HOLD` modes with automatic capability validation (e.g., verifying seed hopper inventory and winch availability).
- **Step-by-Step Plan Queue**: Displays the active step, upcoming waypoints, expected postconditions, and task provenance for every robot in the swarm.

### 2.3 Emergency Human Control Mode & Fleet E-Stop (`src/components/EmergencyControlPanel.tsx`)
- **Global Fleet Emergency Stop (E-Stop)**: Single-click high-priority button in the top bar and control deck that immediately commands deterministic `HOLD` (`v_cmd = 0 m/s`, `omega_cmd = 0 rad/s`, winch brake locked) across all five rovers.
- **Single-Robot Human Teleoperation Override**: Dedicated manual control mode allowing human operators to take direct control of any selected robot using on-screen directional actuators or keyboard (`W`/`A`/`S`/`D` or Arrow keys).
- **Special Recovery Actuators**:
  - **Micro-Rocking Clay Breaker**: Applies alternating low-duty-cycle $\pm$ torque pulses to break thixotropic clay suction (`S6`) without excavating the plow pan.
  - **Winch Tether Tension Controller**: Manual wind/pay-out control with live `INA240` strain-gauge limit monitoring ($20\text{ N} \le F_{\text{pull}} \le 350\text{ N}$, auto-cutoff at $400\text{ N}$).
  - **Reverse Polarity Axle Clearing**: Sheds entangled rice straw (`S5`) from wheel hubs.
  - **Manual Seed Dropper Trigger**: Actuates a physical seed release at the robot's current coordinates when over a valid gap site.

### 2.4 200Hz Sensor-Fusion Telemetry & S1–S8 Fault Triage (`src/components/TelemetryInspector.tsx` & `src/components/ScenarioTriagePanel.tsx`)
- **STM32H7 Low-Level Sensor Readouts**: Live tabular telemetry for 60GHz Doppler mmWave Radar true ground speed ($v_{\text{true}}$), wheel encoder speed ($\omega \cdot r$), real-time kinematic slip ratio ($s = 1 - v_{\text{true}} / (\omega \cdot r)$), IMU pitch/roll ($\theta, \phi$), motor phase current ($I_{\text{motor}}$), Hall-effect whisker furrow error ($e_y$), and Dual-UWB PDoA range/bearing.
- **S1–S8 Paddy Immobilization Simulator & Multi-Objective Rescue Evaluator**: Operators can trigger any of the eight documented paddy field failure modes (`S1` through `S8`) and inspect the AI Coordinator's multi-objective cost evaluation $\mathcal{J}(\text{action})$ across `SELF_RECOVER_BOUNDED`, `DISPATCH_RESCUE_DIRECT`, `DISPATCH_RESCUE_DETOUR`, `REALLOCATE_AND_BEACON`, and `SAFE_ABSTAIN_DEFER_HUMAN`.

---

## 3. Modular Codebase Architecture

- `src/config/appConfig.ts`: Centralized configuration for field geometry, simulation tick intervals, kinematic thresholds, semantic status colors, S1–S8 fault definitions, and AI model parameters.
- `src/utils/logger.ts`: Structured info logger that records every function call with its parameters and logs any GenAI calls (stripping inline binary data).
- `src/types/antenna.ts`: TypeScript data contracts mirroring `RobotObservationV1`, `SoilSampleV1`, `FieldBeliefV1`, `RobotCapabilityV1`, `TaskIntentV1`, `DriveCommandV1`, and `S1–S8` fault diagnostics.
- `src/data/initialSwarmState.ts`: Deterministic initial seed state for the 5-robot Kinematic Quintet, paddy field cells, missing crop gap sites, and hazard zones.
- `src/logic/swarmEngine.ts`: Pure, unit-tested kinematic simulation, slip ratio math, plan description generator, task assignment validator, emergency control transitions, and multi-objective rescue scoring.
- `src/logic/swarmEngine.test.ts`: Built-in `node:test` suite verifying slip ratio calculations, robot plan formatting, emergency stop state transitions, manual teleoperation movement bounds, and cooperative rescue dispatch.
- `src/components/TopNavigation.tsx`: Strict 3-zone top bar with brand identity, view navigation links, and Global Emergency Stop trigger.
- `src/components/FieldMapCanvas.tsx`: Interactive SVG/Canvas hybrid metric map of the flooded rice terrace with robot dots, status rings, plan trajectories, crop gap markers, and click-to-command tools.
- `src/components/RobotRosterPanel.tsx`: Color-coded multi-robot status list with natural-language plan summaries and quick human override toggles.
- `src/components/EmergencyControlPanel.tsx`: Human teleoperation console with directional pad, keyboard bindings, speed limiter, winch tension override, and clay-suction breaker.
- `src/components/TelemetryInspector.tsx`: 200Hz STM32H7 sensor fusion telemetry cards and task reassignment form.
- `src/components/PlanTimelinePanel.tsx`: Fleet-wide mission plan matrix, step-by-step action breakdown, and manual task dispatcher.
- `src/components/ScenarioTriagePanel.tsx`: S1–S8 immobilization scenario injector and Multi-Objective Rescue Coordinator ($\mathcal{J}(\text{action})$) analyzer.
