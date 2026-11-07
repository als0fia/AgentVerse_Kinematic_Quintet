# M4-ROBOTICS — Physical Robot and Rescue Feasibility Audit

**Role:** M4 — Physical Robot and Rescue Feasibility Examiner (Adversarial Robotics / Mechanical Engineer)  
**Target Repository:** `https://github.com/als0fia/AgentVerse_Kinematic_Quintet`  
**Pinned Reference Snapshot (`main`):** `130e881a54b932dcbe66971674e266b344e104f6` (*Merge source-checked PR #22 research reconciliation (#34)*, 2026-11-07)  
**Latest `main` Inspected at Upload:** `82099e0` (*Merge pull request #37 from als0fia/feat/3d-mud-relief-and-wdsr-fitness*, 2026-11-07)  
**Feature / PR Branches Inspected:**
- `origin/feat/m1-antenna-integrated-rescue` (`ce32aaf18b853a583ec9c0350a1f649176f2cb3e`, M1 integration candidate)
- `origin/fix/rescue-first-stranding-and-docs-20261107` (`da6f164`, safety guard on advisor override)
- `origin/docs/antenna-single-rescue-demo-and-supervisor` (`6175e09`, Open PR #32)
- `origin/docs/rescue-policy-math-and-improvisation` (`0e67ef4`, unmerged robotics/climate branch containing `scripts/eval_rescue_techniques.py` and `scripts/sim_extreme_climate_typhoon.py`)

**Execution Environment (Verified Hardware & Physics Run):**
- **OS:** Windows 11 (`10.0.26200`)
- **Python:** `3.14.3`
- **Physics Engine:** `MuJoCo 4.0.0` (`timestep = 0.004 s`, `integrator = implicitfast`, `25 substeps/tick = 0.10 s/tick`)
- **Executed Test Suites:**
  - `python -m unittest discover -s tests -v` → **72 tests ran in 91.378 s, OK**
  - `python scripts/check_repo.py` → **OK: required docs, portable skill parity, schema syntax, and local Markdown links**
  - Standalone MuJoCo 4.0.0 forensic traces: step-by-step constraint/contact/force trace of `build_rigid_docking_rescue_mjcf()`, 24-point lateral/yaw alignment & detach sweep, 5-level mud-drag secondary-casualty stress sweep, and end-to-end mud-stall → firm-ground tandem extraction → clean detach → resumed seed-drop verification in `MujocoPaddySimulation`.

---

## Executive Verdict & Physical Readiness Ladder

> [!CAUTION]
> **Three High-Impact Forensic Findings from Live MuJoCo 4.0.0 Execution:**
> 1. **Zero Rescue Actuation in `main` (`130e881` & `82099e0`):** In [`src/ricebot/mujoco_sim/adapter.py`](../../../src/ricebot/mujoco_sim/adapter.py), `MujocoPaddySimulation.step()` zeroes all controls (`self.data.ctrl[:] = 0.0`), never actuates the docking/latch/spade servos (`ctrl[8..13]`), and never calls [`update_rigid_dock_latch()`](../../../src/ricebot/robot/rovers.py). At `130e881`, [`evaluate_bekker_terramechanics()`](../../../src/ricebot/robot/rovers.py) was completely uncalled; `82099e0` (PR #37) wired Bekker-Wong speed penalties into `adapter.py`, but still leaves `update_rigid_dock_latch()` and `ctrl[8..13]` uncalled on `main`.
> 2. **The Standalone Docking Test Passes via a `3,224 N` Constraint-Snap & `44.5°` Airborne V-Tent Jackknife:** In [`tests/test_world_and_robot.py`](../../../tests/test_world_and_robot.py) (`test_mujoco_rigid_docking_cam_latch_and_tandem_extraction`), all docking cones, rear V-funnels, strike bars, latch hooks, and anchor spades have `contype="0" conaffinity="0"` (purely visual ghost geometry). Latching activates a 3-DOF ball joint (`<connect solref="0.008 1.0"/>`) when the sites are still **`64.85 mm` apart**. This injects an instantaneous **`3,224.1 N` impulse** in `8 ms` that slams the two `4.59 kg` rovers together, launching both airborne (`z = 0.126 m` and `0.150 m`, `44.54°` pitch jackknife, 6 of 8 wheels spinning in mid-air). During the "reverse tow" phase, the helper (`planter`) actually moves **forward** (`x = 0.414 m → 0.435 m`) while the victim (`scout`) is yanked backward (`x = 0.820 m → 0.644 m`) by the 65 mm constraint snap + geometric foreshortening of the 45° V-tent.
> 3. **M1 Candidate Branch (`ce32aaf`) Widens the Snap Radius to `95 mm` and Zeroes Bogged Wheel Spin:** On `origin/feat/m1-antenna-integrated-rescue` (`ce32aaf`), `adapter.py` sets `capture_radius_m=0.095` (95 mm gap snap!) and `dock_pitch_rad = max(0.38, ...)` while setting `lin_eff = 0.0` when `is_bogged_now` (preventing wheels from spinning when bogged, which masks the wheel-slip vs ground-speed diagnostic signature). Meanwhile, unmerged branch `origin/docs/rescue-policy-math-and-improvisation` (`0e67ef4`, `scripts/eval_rescue_techniques.py`) directly overwrites `data.qpos` in Python (`case["stuck_yaw_rad"] *= 0.85`) and flips `suction_broken = True` at `step > 210` — it must **not** be cited as physical evidence.

### Physical Readiness Ladder

| Level | Capability Stage | Status on `main` (`130e881` / `82099e0`) | Evidence & Required Fix |
| :--- | :--- | :--- | :--- |
| **L0** | **Homogeneous 3D Rover & Wheel Drive** | **VERIFIED IN MAIN** | 2 identical `4.5935 kg` 4-wheel differential skid-steer units (`276 mm` outer width `< 300 mm` row), 3-ray LiDAR fan, rigid-body seed drop (`seed_proxy_0/1`) verified in [`tests/test_mujoco_sim.py`](../../../tests/test_mujoco_sim.py). |
| **L1** | **Scripted Standalone Docking Fixture** | **PARTIAL / DEFECTIVE** | [`paddy_ant_rigid_rescue.xml`](../../../src/ricebot/robot/assets/paddy_ant_rigid_rescue.xml) compiles and runs, but `capture_radius_m=0.065` + `solref="0.008 1.0"` + `pitch=+0.35 rad` causes a `3,224 N` snap and `44.5°` airborne V-tent jackknife. Fixed in our MuJoCo 4.0.0 verification by setting `capture_radius_m=0.018..0.022 m`, `solref="0.04 1.0"`, and `dock_pitch=0.08 rad` (`max_efc` drops `20×` to `159.3 N`, all wheels stay grounded at `z ≈ 0.059 m`, true reverse tow `-0.108 m`). |
| **L2** | **Physical Immobilisation in Terrace Sim** | **MISSING (`130e881`) / PARTIAL (`82099e0`)** | At `130e881`, `bed_patch_a` (`(0.85, -0.50)`) had `contype="0"` and uniform ground friction `1.2`. At `82099e0` (PR #37), `adapter.py` calls `evaluate_bekker_terramechanics()` and scales wheel velocity by `speed_penalty_factor` + `-14 N` drag when `bellied_out`, but for the `4.2 kg` rover on `patch_a` (`moisture=0.92, firmness=0.16`), `evaluate_bekker_terramechanics()` returns `sinkage_z_m = 0.0123 m < clearance_0 = 0.044 m` (`bellied_out = False`, `speed_penalty_factor = 0.7976`), so the rover **still never gets immobilized on `main` without an explicit local mud-pocket threshold or wheel-friction drop**! |
| **L3** | **On-Robot Sensed Diagnosis (Stuck vs Hold/Sensor)** | **CONFLATED IN MAIN** | [`CooperativeSwarmPlanner._update_stranded_tracking()`](../../../src/ricebot/agent/planner.py) conflates `front_range_m is None` (sensor fault) and obstacle stops (`front_range_m <= 0.35`) with physical immobilisation, and does not compare commanded wheel speed against achieved ground speed. |
| **L4** | **Bounded Recovery Skill via `SimulationPort`** | **MISSING IN MAIN (`ce32aaf` WIP)** | On `main`, [`RobotCommand`](../../../src/ricebot/contracts.py) has no recovery skill field, [`LocalController`](../../../src/ricebot/control.py) halts whenever `peer_distance < 1.0 m` or `front_range_m <= 0.35 m` (blocking docking approach), and `CooperativeSwarmPlanner` dispatches helpers using `TaskMode.SAMPLE`. Branch `ce32aaf` wires `TaskMode.RESCUE` and `RecoveryFeedback`, but needs the `capture_radius_m=0.022` / `solref="0.04 1.0"` / `dock_pitch=0.08` physics fix so it does not snap across `95 mm` or jackknife. |
| **L5** | **Post-Rescue Detach & Resumed Fieldwork** | **PROVEN IN M4 MUJOCO RUN** | Physically verified in MuJoCo 4.0.0: `scout` approaches on firm ground, latches onto `planter` at `capture_radius_m=0.022 m`, pulls `planter` `-0.2183 m` out of `patch_a` (`0.8535 m → 0.6352 m`), detaches cleanly (`eq_active=0`), and `planter` detours to `patch_b` (`(1.535, -0.368)`) and drops `seed_0` with verified ground contact at `t = 8.54 s`. |

---

## 1. Forensic Reproduction of the Current Docking & Tandem Extraction Fixture

### 1.1 Exact Geometry, Mass, Collider & Constraint Audit
Executed on [`build_rigid_docking_rescue_mjcf()`](../../../src/ricebot/robot/rovers.py) in MuJoCo 4.0.0:

- **Subtree Masses:** `planter_body = 4.5935 kg`, `scout_body = 4.5935 kg` (`chassis = 2.45 kg`, `4 × wheels = 1.40 kg`, plus tools/deck). Note: [`src/ricebot/robot/assets/paddy_ant_rigid_rescue.xml`](../../../src/ricebot/robot/assets/paddy_ant_rigid_rescue.xml) on disk is slightly stale relative to `rovers.py` (`chassis mass="2.6"` and 14 actuators in the checked-in XML vs `mass="2.45"` and 20 actuators generated by `build_rigid_docking_rescue_mjcf()`).
- **What Actually Collides vs Ghost Visuals (`contype` / `conaffinity`):**
  - **Collidable (`contype != 0`):** `field_ground` (`1/7`), `{rid}_chassis` (`2/5` and `4/5`, half-sizes `0.135 × 0.095 × 0.032 m`), and the 8 wheel cylinders `{rid}_w_fl..rr` (`2/1` and `4/1`, radius `0.058 m`, half-width `0.020 m`, `friction="1.4 0.01 0.005"`).
  - **Non-Collidable Ghost Visuals (`contype="0" conaffinity="0"`):**
    - All 6 radial wheel grouser lugs per wheel (`{rid}_lug_*`)
    - Front docking shank, cone, and rotary cam-latch hook (`{rid}_dock_shank`, `{rid}_dock_cone`, `{rid}_latch_arm`, `{rid}_latch_hook`)
    - Rear V-guide alignment funnel and strike bar (`{rid}_rear_funnel_l/r`, `{rid}_rear_strike_bar`)
    - Rear hardpan anchor spades (`{rid}_spade_l/r`) and vertical seed/soil dibber (`{rid}_dibber_rod/tip`)
    - Belly skid (`{rid}_belly_skid`) and `mud_trap_pan` (`pos="0.85 0.0 0.002"`).
- **Mechanical Consequences:**
  - The rear V-funnel (`±0.38 rad` plates) **cannot mechanically guide or self-center** an off-axis nose cone via contact forces because `contype=0`.
  - Deploying rear anchor spades (`{rid}_spade_act`) or plunging the dibber (`{rid}_dibber_act`) generates **zero ground reaction force** because their geoms never collide with `field_ground`.
  - Mechanical coupling is represented solely by activating the MuJoCo `<equality><connect>` constraint (`rigid_latch_planter_to_scout`, `mjtEq.mjEQ_CONNECT`, a **3-DOF ball-and-socket point constraint** between `planter_front_dock_site` at `x = +0.193 m` in planter frame and `scout_rear_dock_site` at `x = -0.148 m` in scout frame).

### 1.2 Step-by-Step Trace of `test_mujoco_rigid_docking_cam_latch_and_tandem_extraction`
Reproduced on MuJoCo 4.0.0 ([`tests/test_world_and_robot.py`](../../../tests/test_world_and_robot.py)):

| Phase / Step | Sim Time (s) | `planter_xyz` (m) | `scout_xyz` (m) | Dock Site Dist (m) | `eq_active` | Active Wheel Contacts (`ncon`) | Peak Constraint Force `efc_force[0]` (N) | Physical Reality of What Happened |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `t0_initial` (`step 0`) | `0.000` | `(0.4000, 0.0, 0.0680)` | `(0.8200, 0.0, 0.0560)` | `0.07991` | `0` | `8` (scout only; planter spawned 10 mm airborne!) | `2.06` | Rovers spawned only `79.91 mm` apart (`14.91 mm` outside `0.065 m` capture radius). `scout` is on high-friction ground, not stuck in mud. |
| `approach_step_5` (`step 5`) | `0.020` | `(0.4001, 0.0, 0.0664)` | `(0.8200, 0.0, 0.0565)` | `0.07977` | `0` | `8` (planter still falling) | `4.91` | `planter` wheels spin up in mid-air while dropping onto `field_ground`. |
| `latch_engaged` (`step 24`) | `0.096` | `(0.4141, 0.0, 0.0568)` | `(0.8200, 0.0, 0.0576)` | `0.06485` | `1` | `16` (all 8 wheels) | `7.20` | `planter` drove forward only `14.1 mm`; distance crosses `0.065 m` threshold; Python sets `eq_active = 1`. |
| `tow_step_1` (`step 25`) | `0.100` | `(0.4154, 0.0, 0.0569)` | `(0.8200, 0.0, 0.0576)` | `0.06355` | `1` | `16` | **`3,224.12`** | **Telekinetic Snap:** `<connect solref="0.008 1.0"/>` tries to close a `64.85 mm` gap in `8 ms`, injecting a **`3.22 kN` impulse**. |
| `tow_step_10` (`step 34`) | `0.136` | `(0.4491, 0.0, 0.0875)` | `(0.7880, 0.0, 0.0791)` | `0.00413` | `1` | `4` (6 wheels airborne) | `-69.87` | The `3.22 kN` snap + `+0.35 rad` pitch command slams the two chassis boxes together and bucks both rovers upward into the air. |
| `tow_step_60` (`step 84`) | `0.336` | `(0.4771, 0.0, 0.1261)` | `(0.6961, 0.0, 0.1351)` | `0.00019` | `1` | `4` (only 2 wheels touch ground) | `25.02` | Both rovers are jackknifed in a **`44.54°` V-tent** (`z = 126 mm` and `135 mm`). `planter` has moved **`+63 mm` forward** despite reverse wheel command! |
| `tow_step_90` (`step 114`) | `0.456` | `(0.4353, 0.0, 0.1039)` | `(0.6442, 0.0, 0.1502)` | `0.00023` | `1` | `4` (scout 100% airborne at `z=150 mm`) | `-38.84` | Test asserts `0.6442 < 0.7400` (`PASS`), even though `planter` moved **forward** (`0.400 → 0.435 m`) and `scout` is hanging `150 mm` in the air! |

### 1.3 What is Forced vs What Dynamics Emergently Allows
- **Forced / Non-Physical in Current Fixture:**
  1. `capture_radius_m = 0.065 m` (or `0.095 m` on `ce32aaf`) with `solref = "0.008 1.0"` forces a `>3,200 N` constraint snap across `65–95 mm` of empty space before the cone seats in the rear funnel.
  2. `dock_pitch_act = +0.35 rad` (or `+0.38 rad` on `ce32aaf`) across a 3-DOF `<connect>` ball joint does not lift `scout` relative to `planter`; it folds the nose boom upward while the two chassis hulls collide, causing the `44.5°` V-tent jackknife.
- **Emergently Allowed by Dynamics (When Fixed):**
  - Tightening `capture_radius_m` to **`0.018–0.022 m`**, softening `solref` to **`"0.04 1.0"`**, and bounding `dock_pitch_act` to **`0.08 rad`** reduces peak constraint force by **`20×`** (`3,226.3 N → 159.3 N`), keeps **both rovers flat on the ground (`z = 0.0594 m` and `0.0599 m`)**, and allows reverse wheel drive to pull both rovers backward along the ground (`planter_dx = -0.0945 m`, `scout_dx = -0.1075 m`).

---

## 2. Victim Diagnosis: Separating True Immobilisation from False Alarms

### 2.1 Current Diagnosis Flaws
In [`src/ricebot/agent/planner.py`](../../../src/ricebot/agent/planner.py) (`CooperativeSwarmPlanner._update_stranded_tracking()`):
1. `sensor_fault = robot.front_range_m is None` flags the robot as `stranded` on tick 1, triggering `RESCUE_NOW` in [`tests/test_agentic_swarm.py`](../../../tests/test_agentic_swarm.py). A missing rangefinder value is a **sensor fault**, not physical immobilisation.
2. `blocked_immobile` triggers when `prev_mode != TaskMode.HOLD and moved < 0.02 and robot.front_range_m <= 0.35`. Because [`LocalController`](../../../src/ricebot/control.py) intentionally commands `linear_m_s = 0.0` when `front_range_m <= 0.35`, a robot that safely stops in front of an obstacle has `moved = 0.0` and is falsely diagnosed as stranded after 2 ticks (`0.2 s`), causing **false rescue dispatch**.
3. On `ce32aaf`, `is_bogged_now` sets `lin_eff = 0.0` in `adapter.py`, which stops the wheels from spinning when bogged (`wheel_spin = 0.0 rad/s`), making a bogged rover look identical to an intentionally stopped rover at the wheel-encoder level.

### 2.2 Required On-Robot Observable Diagnostic Matrix

Victim diagnosis must use a **time-windowed (`W = 5 ticks = 0.5 s`) comparison of commanded drive (`cmd.linear_m_s`), wheel joint angular velocity (`mean(|qvel[j_fl..rr]|) * r_wheel`), achieved odometric speed (`v_true = ||Δpose|| / Δt`), actuator effort (`mean(|actuator_force[0..3]|)`), and range/attitude**:

| Failure / State Class | Commanded `|v_cmd|` (m/s) | Wheel Rim Speed `v_wheel = ω·r` (m/s) | Achieved Speed `v_true` (m/s) | Wheel Servo Torque `|τ_act|` (N·m) | `front_range_m` & Attitude (`pitch/roll`) | Correct Autonomous Action (No False Rescue) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **1. True Mud Immobilisation (Slip/Spin, S1/S2)** | `> 0.08` (for `≥ 0.5 s`) | `> 0.15` (`ω > 2.6 rad/s`) | `< 0.015` (slip `s > 0.85`) | Low/Moderate | `> 0.35 m`, `|pitch|, |roll| < 0.35 rad` | Cut wheel spin (`HOLD`), mark mud hazard, request **Physical Rescue Triage**. |
| **2. Mechanical / High-Drag Stall (S5/S6)** | `> 0.08` (for `≥ 0.5 s`) | `< 0.02` (wheels stalled) | `< 0.015` | Near saturation (`|τ| > 12 N·m`) | `> 0.35 m` | Try 1 bounded reverse pulse (`≤ 0.5 s`); if still stalled, `HOLD` & escalate/rescue. |
| **3. Intentional Stop / Planting / Sampling** | `== 0.0` | `< 0.01` | `< 0.01` | `≈ 0.0` | Any | **Nominal working state** — never flag as stranded. |
| **4. Obstacle Block (Safe Range Reflex)** | `== 0.0` (vetoed by `stop_range_m`) | `< 0.01` | `< 0.01` | `≈ 0.0` | `≤ 0.35 m` (valid obstacle hit) | **Route Obstructed** (replan waypoint or SLAM bypass) — **NOT** physical immobilisation. |
| **5. Degraded Traction (Crawling)** | `> 0.10` | `> 0.10` | `0.03 .. 0.08` (`0.4 < s ≤ 0.8`) | Moderate | `> 0.35 m` | Throttle wheel speed (`LocalController` anti-digging); continue without rescue call. |
| **6. Bad Rangefinder Sensor** | `== 0.0` (vetoed by `None` range) | `< 0.01` | `< 0.01` | `≈ 0.0` | `front_range_m is None`, `link_ok=True` | **Sensor Fault** (`HOLD` or cautious reverse backup); do **not** tow into hazard unless payload recovery requires it. |
| **7. Dead Compute / Radio Blackout** | Unknown to peers (`link_ok=False`) | Unknown remotely | `Δpose == 0` via peer UWB/LiDAR | Unknown remotely | `link_ok=False` for `> 1.0 s` | Do not assume stuck on first dropped packet; healthy rover can approach to inspect (`INVESTIGATE`) outside hazard radius. |
| **8. Rollover / Ditch Tip Hazard (S4)** | Cut to `0.0` by IMU reflex | `< 0.01` | `< 0.01` | `≈ 0.0` | `|pitch| > 0.35 rad` or `|roll| > 0.35 rad` | **Immediate Safety `HOLD` + `ABANDON_ESCALATE`** (refuse rescue to avoid tipping helper). |

---

## 3. Rescue Geometry, Alignment Tolerance, Overconstraint & Secondary-Casualty Stress Tests

### 3.1 Alignment Tolerance & Detach Sweep (24 MuJoCo 4.0.0 Trials)
Executed across lateral offsets `Δy ∈ [0.00, 0.08] m` and yaw offsets `Δψ ∈ [0°, 15°]`:

| Lateral Offset `Δy` (m) | Yaw Offset `Δψ` (deg) | Min Dock Site Dist (m) | Latch Engaged | Extracted Displacement `Δx` (m) | Clean Detach (`cap_r=0.065` vs `cap_r=0.022`) |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `0.000` | `0.0°` | `0.0649` | `True` | `0.1758` | `False` (at `0.065`, V-tent) / `True` (at `0.022`) |
| `0.000` | `5.0°` | `0.0641` | `True` | `0.1335` | `False` / `True` |
| `0.000` | `10.0°` | `0.0646` | `True` | `0.0407` | `False` / `True` |
| `0.000` | `15.0°` | `0.0643` | `True` | `0.0336` | `False` / `True` |
| `0.020` | `0.0°` | `0.0642` | `True` | `0.1144` | `False` / `True` |
| `0.040` | `0.0°` | `0.0645` | `True` | `0.0122` | `False` / `True` |
| `0.055` | `0.0°` | `0.0644` | `True` | `0.0385` | `False` / `True` |
| `0.065` | `0.0°` | `0.0650` | `False` | `0.0000` | N/A (Failed to lock) |
| `0.080` | `0.0°` | `0.0800` | `False` | `0.0000` | N/A (Failed to lock) |

- **Closed-Loop Funnel Servoing + Grounded Detach:** Because the rear funnel geoms have `contype=0`, the rescuer controller must steer toward `victim_rear_dock_site` during the final `0.35 m` approach and only engage `eq_active = 1` when `dock_site_dist_m <= 0.022 m` and `|yaw_error| <= 0.20 rad`. Under `solref="0.04 1.0"`, `capture_radius_m=0.022 m`, and `dock_pitch=0.08 rad`, setting `latch_act = 0.0` sets `eq_active = 0` immediately and backing away at `-0.25 m/s` for `0.8 s` cleanly separates the two rovers (`eq_after_detach = 0`, separation `> 0.12 m`).

### 3.2 Mechanical Overconstraint & Secondary Casualty Stress Test (Mud Drag Sweep)
Resisting mud drag `F_drag ∈ {0, 10, 25, 45, 80} N` and downward suction `-0.5 * F_drag` applied to `scout_body` in MuJoCo 4.0.0:

| Applied Mud Drag `F_drag` (N) | Victim Pulled `Δx_victim` (m) | Helper Moved `Δx_helper` (m) | Outcome & Secondary Casualty Verdict |
| :--- | :--- | :--- | :--- |
| **`0 N`** | `+0.1906` | `-0.0057` | Extracted easily. |
| **`10 N`** | `+0.0734` | `+0.0074` | Marginal extraction; near single-helper traction limit if victim wheels do not assist. |
| **`25 N`** | **`+0.0002`** | **`-0.0641`** | **BOTH IMMOBILIZED (Secondary Casualty):** Rescuer's 4 wheels slip against the `25 N` load (`max_thrust_n ≈ 21–25 N`). |
| **`45 N`** | **`-0.0072`** | **`-0.0714`** | **BOTH IMMOBILIZED / DRAGGED INTO MUD:** Rescuer is pulled forward into the trap. |
| **`80 N`** | **`-0.0319`** | **`-0.0962`** | **CATASTROPHIC:** Rescuer is dragged `96 mm` forward into the hazard zone. |

> [!IMPORTANT]
> **Physical Proof of Why Safe Refusal (`ABANDON_ESCALATE`) is Necessary:**  
> A single `4.59 kg` rover on paddy soil (`friction = 1.2–1.4`) generates a maximum drawbar pull of **`~21–25 N`** (matching `evaluate_bekker_terramechanics` on `patch_a`: `max_thrust_n = 21.27 N`, `compaction_resistance_n = 10.31 N`, net `drawbar_pull_margin_n = 10.96 N`). Whenever victim mud resistance exceeds **`20–25 N`** (or the helper must enter a low-friction mud zone `r > 0.34 m` to reach the victim's rear dock), **dispatching the helper physically traps both robots**.

---

## 4. Bounded Recovery Command Interface & Safety Gate

1. **`TaskMode` & `RobotCommand` Contract ([`src/ricebot/contracts.py`](../../../src/ricebot/contracts.py)):**
   - Add `RESCUE = "rescue"` to `TaskMode` and `RecoveryFeedback` to `ObservationFrame` (as begun on `ce32aaf`).
   - Bound `dock_pitch_rad` in `[0.0, 0.15] rad` (never `0.38 rad`, which causes the V-tent jackknife) and `latch_rad` in `[0.0, 0.65] rad`.
2. **Deterministic Controller State Machine ([`src/ricebot/control.py`](../../../src/ricebot/control.py) / [`src/ricebot/mujoco_sim/adapter.py`](../../../src/ricebot/mujoco_sim/adapter.py)):**
   - Bypass `min_peer_distance_m = 1.0 m` and `stop_range_m = 0.35 m` strictly with respect to `recovery_target_id` during `TaskMode.RESCUE` at bounded approach speed (`linear_m_s <= 0.25 m/s`).
   - **Phase 1 (`APPROACH_AND_ALIGN`, timeout `4.0 s`):** Servo toward `victim_rear_dock_pos`; `latch_rad = 0.55`, `dock_pitch_rad = 0.0`.
   - **Phase 2 (`LATCH_AND_PRY_PULL`, timeout `3.0 s`):** Only engage latch at `capture_radius_m = 0.022 m` with `solref = "0.04 1.0"`; command `dock_pitch_rad = 0.08 rad` and reverse (`linear_m_s = -0.35 m/s`) until victim displacement `>= 0.18 m`.
   - **Abort / Safety Veto:** If `eq_active == 1` for `> 1.0 s` and backward displacement `< 0.02 m` (or `|pitch| > 0.30 rad`), unlatch (`latch_rad = 0.0`), back away `0.15 m`, and transition to `HOLD` / `ABANDON_ESCALATE`.
   - **Phase 3 (`DETACH_AND_CLEAR`, `0.8 s`):** Set `latch_rad = 0.0`, `dock_pitch_rad = 0.0`, reverse at `-0.20 m/s` until `dock_site_dist_m > 0.10 m` and `eq_active == 0`.

---

## 5. Minimum World Mechanics, Learned Policy Audit & End-to-End Fieldwork Proof

- **Minimum World Mechanics (`~20 lines` in `MujocoPaddySimulation`):** Set `geom_priority = 1` on the 8 wheels; when a rover is within `r <= 0.16 m` of `patch_a` (`(0.85, -0.50)`), set wheel `geom_friction[gid, 0] = 0.002` and viscous drag `xfrc_applied[bid, 0:2] = -35.0 * qvel` while letting its wheel servos spin normally (`ω = 6.03 rad/s`). A single rover inside `r <= 0.16 m` spins its wheels while stalling (`solo_dx = +0.0035 m` over `1.5 s`, slip `s = 0.993`), whereas a helper docking from `0.341 m` behind stands on firm ground (`friction = 1.4`) and pulls the victim out of the `0.16 m` mud pocket. If the hazard radius exceeds `0.34 m`, the helper also loses traction and **must refuse rescue (`ABANDON_ESCALATE`)**.
- **Learned Policy Audit:** Zero RL checkpoints or neural policies exist anywhere in the repo or branches; `scripts/eval_rescue_techniques.py` on `origin/docs/rescue-policy-math-and-improvisation` mutates `data.qpos` directly and must not be claimed as learned or physical.
- **End-to-End Fieldwork Proof (Executed in MuJoCo 4.0.0):**
  1. `planter` stalls in `patch_a` (`0.8500 → 0.8535 m` over `1.5 s`, `v_true = 0.0026 m/s`).
  2. `scout` docks (`capture_radius_m = 0.022 m`, `solref = "0.04 1.0"`), pulls `planter` `-0.2183 m` to firm soil (`x = 0.6352 m`), and detaches (`eq_active = 0`).
  3. `planter` detours via `y = -0.18 m` to `patch_b` (`(1.535, -0.368)`) and drops `seed_0` with verified ground contact at `t = 8.54 s` (`Pose2D(1.5316, -0.3651, 0.0)`, `completed_drops = 1`).

---

## 6. Smallest Change Packet to M1 and M6

- **To M1 (Integration Engineer — adjustments to `origin/feat/m1-antenna-integrated-rescue` / `ce32aaf`):**
  1. In [`src/ricebot/robot/rovers.py`](../../../src/ricebot/robot/rovers.py) and [`src/ricebot/mujoco_sim/adapter.py`](../../../src/ricebot/mujoco_sim/adapter.py): reduce `capture_radius_m` from `0.095` / `0.065` to **`0.022 m`**, change `<connect ... solref="0.008 1.0"/>` to **`solref="0.04 1.0"`**, and reduce `dock_pitch_rad` during extraction from `0.38` to **`0.08 rad`** so the rovers stay grounded (`z ≈ 0.059 m`) instead of snapping at `3.2 kN` into a `44.5°` V-tent.
  2. In [`src/ricebot/mujoco_sim/adapter.py`](../../../src/ricebot/mujoco_sim/adapter.py): do **not** zero `lin_eff = 0.0` when `is_bogged_now`; let the bogged rover's wheels spin against reduced wheel friction (`geom_priority=1`, `geom_friction=0.002`) + viscous drag so `v_wheel >> v_true` (`slip_ratio > 0.85`) is physically observable.
- **To M6 (Agentic Supervisor):**
  1. In [`src/ricebot/agent/planner.py`](../../../src/ricebot/agent/planner.py): separate `front_range_m is None` (sensor fault) and `front_range_m <= 0.35` (obstacle stop) from true mud immobilisation (`v_cmd > 0.08` and `v_true < 0.015` for `≥ 5 ticks`), and output `ABANDON_ESCALATE` whenever hazard radius `> 0.34 m` or required pull `≥ 22 N`.
