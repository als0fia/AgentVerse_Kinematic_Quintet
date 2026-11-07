# Robot Embodiment & Homogeneous Ant Mechanics (`src/ricebot/robot/`)

## 1. Purpose & Homogeneous Ant-Colony Design (Role 2)
Owns the physical rover embodiment (`RoverChassisSpec`), 4-wheel differential skid-steer kinematics (`compute_4wheel_skid_steer`), forward 3-ray LiDAR and odometry sensor mount frames, articulated farming tools (seed-hopper release chute, vertical seed-dibber/soil-probe plunger, counter-rotating underbelly finger weeders), **combined rigid docking, cam-locking & pry-lift rescue mechanism**, and robot-only articulation/MJCF builders (`build_rovers_and_seeds_xml`, `build_rigid_docking_rescue_mjcf`, `export_robot_mjcf_assets`).

### Key Mechanical & Agronomic Features
1. **100% Homogeneous "Medic-Ant" Swarm Units (`make_homogeneous_rover_spec`):**
   - Inspired by Matabele medic ants (*Megaponera analis*), **every robot in the swarm shares 100% identical physical hardware** (`can_sample=True`, `can_plant=True`, `can_rescue=True`). There is no fragile "scout-only" or "rescue-only" hardware variant.
   - Operational specialization (`SAMPLE`, `PLANT`, `HOLD`, or `RESCUE`) emerges dynamically in software from real-time seed inventory (`seeds_remaining`), proximity to unverified stand gaps, and opportunity cost when a teammate becomes bogged down.
2. **Dimensional Envelope & Southern China Paddy Row Fit (`RoverChassisSpec`):**
   - **Crop-Row Clearance:** Fits inside standard **`300 mm`** machine-transplanted/direct-seeded rice inter-row lanes without trampling living rice tillers (`no_go_radius_m`).
   - **Sealed Waterproof Hull:** `270 × 190 × 64 mm` (`chassis_half_x_m=0.135`, `chassis_half_y_m=0.095`, `chassis_half_z_m=0.032`), hull mass `2.45 kg` (`~4.3 kg` total unit mass in MuJoCo; scales to `<35–50 kg`, `<10–15 kPa` low ground-pressure field rover with buoyant HDPE belly skid plate).
   - **4-Wheel Open-Lug Differential Paddy Drive:** `170 mm` wheelbase (`wx=0.085 m`), `236 mm` center track (`half_track_m=0.118 m`), **`276 mm` total outer wheel-to-wheel width**, with four `Ø116 × 40 mm` revolute wheels (`wheel_radius_m=0.058`, `wheel_half_width_m=0.020`, 6 radial herringbone grouser lugs per wheel, `max_wheel_rad_s=14.0 rad/s`, `max_linear_m_s=0.45 m/s`, `max_angular_rad_s=1.40 rad/s`).
3. **Articulated Farming & Agronomic Payload (Present on Every Unit):**
   - **Conformal Seed Hopper & Gravity Release Chute (`{id}_hopper_outer`, `{id}_seed_chute_tube`):** Centrally mounted (`x=+0.034 m, y=0.0 m, z=0.076 m`) with bottom release chute at `hopper_drop_z_m=0.032 m`. Dispenses 6-DOF rigid-body pre-germinated seed pellets (`seed_proxy_0`, `seed_proxy_1`, radius `22 mm`, mass `30 g`) when stopped over a verified viable stand gap.
   - **Articulated Vertical Seed-Dibber & Soil-Probe Plunger (`{id}_dibber_arm`, `{id}_dibber_j`):** Vertical slide-joint plunger (`-52 mm` to `+10 mm` stroke, actuator `{id}_dibber_act`) that physically plunges down into the paddy mud during soil probing (`MOISTURE`, `FIRMNESS`, `STAND_GAP`) and seed placement, and can also push against the hardpan during self-recovery.
   - **Dual Counter-Rotating Underbelly Finger Weeders (`{id}_weeder_l`, `{id}_weeder_r`):** Twin vertical-axis rotary star-tine weeders (`{id}_weeder_l_act`, `{id}_weeder_r_act`, `±18 rad/s`) that spin under the flanks while traversing crop rows to uproot early-stage inter-row weeds.
   - **Conformal Bio-Sprayer Tank, 4-Nozzle Rear Boom & Front Tactile Crop Whiskers (`{id}_spray_tank`, `{id}_spray_boom`, `{id}_whisker_l/r`).**
4. **Combined Rigid Docking, Cam-Latch & Pry-Lift Rescue Mechanism (No Slack Cable / No Winch):**
   - Because robots operate single-file inside a `300 mm` row, a rescuer can only approach **collinearly** (front-to-back). A flexible cable goes slack when pushing and drags a lightweight rescuer into the same mud hole when pulling. Instead, every unit carries a 4-part rigid coupling & suction-breaking mechanism:
     1. **Front Active Pitch-Lift Docking Boom & Tapered Cone (`{id}_dock_boom`, `{id}_dock_cone`, `{id}_front_dock_site`):** Mounted on the front nose (`x=+0.135 m, z=0.010 m`) with an active pitch hinge (`{id}_dock_pitch`, `-0.20` to `+0.45 rad` / `+26°`, `kp=35.0`) to pry/lift a stuck robot's chassis upward (`+Z`) and break hydrostatic mud suction.
     2. **Internal Rotary Cam-Locking Jaw (`{id}_latch_jaw`, `{id}_latch_j`, `{id}_latch_hook`):** Actuated rotary hook (`0.0` to `0.65 rad`, `kp=15.0`) inside the front cone. When seated in a teammate's rear funnel (`distance <= 0.065 m`) with `{id}_latch_act >= 0.30 rad`, `update_rigid_dock_latch()` engages `rigid_latch_{rescuer}_to_{target}`, locking both 4-wheel units into a **single rigid 8-wheel articulated train** that can pull in reverse or push forward without jackknifing.
     3. **Rear Passive V-Guide Alignment Funnel & Strike Bar (`{id}_rear_funnel_l/r`, `{id}_rear_strike_bar`, `{id}_rear_dock_site`):** Angled guide plates (`±0.38 rad`) at `x=-0.150 m` that self-center a mating nose cone onto the horizontal strike bar (`x=-0.148 m`).
     4. **Rear Deployable Hardpan Anchor Spades (`{id}_spade_arm`, `{id}_spade_j`, `{id}_spade_l/r`):** Twin folding Danforth-style flukes (`0.0` to `0.85 rad`, `kp=20.0`) at `x=-0.132 m` that dig into the compacted hardpan to anchor a stationary rescuer during high-load prying.
5. **Perception & Navigation Suite:**
   - **Forward 3-Ray LiDAR / Rangefinder Turret (`{id}_lidar_pod`, `{id}_range_c`, `{id}_range_l`, `{id}_range_r`):** Elevated at `x=+0.112..0.140 m, z=0.050 m` (clear above the front docking cone), casting a 3-beam horizontal fan (`0°` center, `+18°` left, `-18°` right, `4.0 m` cutoff) for deterministic obstacle stops (`<= 0.32–0.35 m`).
   - **BeiDou (BDS-3) RTK GNSS Mast & Odometry Site (`{id}_rtk_mast`, `{id}_rtk_dome`, `{id}_odom_site`):** Top-mounted mast (`z=0.135 m`) providing `Pose2D(x_m, y_m, yaw_rad)` odometry.

---

## 2. Mathematical & Programmatic Basis of the Saving Procedure (Research & Planning)

### 2.1 Terramechanical & Rigid-Coupling Physics Formulation

1. **Entrapment & Wheel-Slip Detection Criterion:**
   Each robot monitors its longitudinal wheel slip ratio $s_i \in [0, 1]$ and displacement rate:
   $$s_i = 1 - \frac{\|v_{\text{odom}}\|}{r_w |\bar{\omega}_w|}, \qquad \text{Stranded} \iff \left(s_i > 0.85 \;\land\; \|\Delta p_i\| < 0.02\text{ m over } N_{\text{stuck}}\text{ ticks}\right) \;\lor\; (\text{range\_m} = \varnothing)$$
   In [`CooperativeSwarmPlanner._update_stranded_tracking`](../agent/planner.py), this increments `_stuck_counts[robot_id]` and triggers a discrete `AmbiguityEvent(kind="robot_stranded")`.

2. **Agentic "Worth-Saving" Triage Utility ([`evaluate_rescue_worth`](../agent/planner.py)):**
   Before dispatching a rescuer, the swarm evaluates net rescue value $V_{\text{rescue}}$:
   $$V_{\text{rescue}} = \underbrace{\left(w_{\text{seed}} N_{\text{needed}} + w_{\text{data}} \mathbb{I}_{\neg\text{link}} + V_{\text{base}}\right)}_{B_{\text{rescue}}\text{ (Rescue Benefit)}} - \underbrace{\left(C_{\text{plant}}\mathbb{I}_{\text{can\_plant\_now}} + w_{\text{dist}} d(p_{\text{helper}}, p_{\text{stuck}})\right)}_{C_{\text{opp}}\text{ (Opportunity Cost)}} - R_{\text{hazard}}$$
   - $V_{\text{rescue}} > 0 \implies \texttt{RESCUE\_NOW}$ (helper immediately diverts to assist).
   - $V_{\text{rescue}} \le 0 \;\land\; B_{\text{rescue}} \ge 0.60 \;\land\; \mathbb{I}_{\text{can\_plant\_now}} \implies \texttt{DEFER\_RESCUE}$ (helper finishes high-value seed drop first, then rescues).
   - $R_{\text{hazard}} \ge 1.5 \;\lor\; (N_{\text{needed}} = 0 \;\land\; V_{\text{rescue}} \le 0) \implies \texttt{ABANDON\_ESCALATE}$ (stranded unit enters safe `HOLD` for human retrieval without risking a second robot).

3. **Mud Suction Seal vs. Pitch-Lift & Yaw-Shear Extraction Mechanics:**
   A bogged chassis in saturated clay slurry (`moisture > 0.85`, `firmness < 0.20`) experiences vertical hydrostatic suction $F_{\text{suction}} = \kappa_{\text{mud}} A_{\text{belly}} \Delta z_{\text{sink}}$ and cohesive shear resistance $F_{\text{drag}}$.
   - **Why a slack winch cable fails:** Horizontal cable tension $T$ cannot reduce normal suction $\Delta z_{\text{sink}}$ and requires $T > F_{\text{drag}} + \mu_{\text{suction}} F_{\text{suction}}$, which exceeds the single rescuer's tire traction $F_{\text{trac,max}} = \mu_{\text{mud}} m g$ and pulls the rescuer into the mud hole.
   - **How the Rigid Pitch-Lift + Cam-Latch breaks suction:**
     1. **Hardpan Spade Anchoring:** Deploying rear spades ($\theta_{\text{spade}} \to 0.75\text{ rad}$) adds passive Mohr-Coulomb soil wedge resistance $F_{\text{spade}} = c A_s + K_p \gamma z_s^2 w_s$.
     2. **Active Pitch-Boom Prying ($\theta_{\text{pitch}} \in [0, 0.45\text{ rad}]$):** Once the rotary cam-latch locks the equality constraint (`eq_active = 1` at $d_{\text{dock}} \le r_{\text{capture}}$), pitching the front boom upward generates an upward vertical pry force at the rear strike bar:
        $$F_{z,\text{pry}} = \frac{\tau_{\text{pitch}}}{L_{\text{boom}}\cos\theta_{\text{pitch}}}, \qquad F_{\text{suction,eff}} = \max\!\left(0,\; F_{\text{suction}} - F_{z,\text{pry}} - F_{\text{shear,osc}}\right)$$
     3. **Dynamic Yaw-Shear & Pitch-Pumping (for Deep Thixotropic Clay):** Applying alternating differential wheel torque $\tau_{\text{yaw}}(t) = A\sin(\omega t)$ and cyclic pitch pumping $\theta_{\text{pitch}}(t) = \theta_0 + \Delta\theta\cos(\omega t)$ introduces lateral shear strain rate $\dot{\gamma}$, liquefying thixotropic paddy clay and breaking the vacuum seal before the 8-wheel tandem reverse pull ($F_{\text{pull},8\text{W}} = 2 F_{\text{trac,4W}}$).

---

### 2.2 Comparison of Proposed Control & AI Paradigms

Following the fast/slow embodied agent literature in [docs/RESEARCH.md](../../../docs/RESEARCH.md#robotics-decision-model-research-fast-execution-slow-investigation-7-nov-2026) (`REA`, `Fast-VLA`), we compare three architectures for executing the saving procedure:

| Architecture | How It Works | Strengths | Weaknesses / Failure Modes |
| :--- | :--- | :--- | :--- |
| **1. Pure Predefined Rule Set (Fixed FSM)** | Hardcoded straight approach $\to$ latch if within cone tolerance $\to$ fixed pitch-lift ($+0.35\text{ rad}$) $\to$ straight reverse pull. | Zero training cost; deterministic; sub-ms execution; works offline. | **58.3% success in randomized mud trials:** misses the rear funnel under moderate yaw/lateral skew and stalls in deep clay suction (`suction > 1.75`). |
| **2. End-to-End Learned Neural Policy (Pure RL / PPO)** | Neural network $\pi_\theta(u_t \mid o_t)$ maps raw 6-DOF poses, joint angles, and slip directly to 7 actuators at control rate. | Can improvise compound rocking/wiggling maneuvers in novel mud geometries. | High sample complexity for contact-rich `<equality>` latch docking; risks violating `300 mm` crop-row boundaries during exploration; black-box safety risk. |
| **3. Hybrid Hierarchical Skill + Learned Parameter/Residual Policy *(Recommended Best Solution)*** | **System 2 (Slow / Event-Triggered):** [`CooperativeSwarmPlanner`](../agent/planner.py) + optional [`GeminiSwarmAdvisor`](../agent/planner.py) performs worth-saving triage and selects a discrete **Rescue Skill** from a 4-technique library.<br>**System 1 (Adaptive / Learned):** Contextual policy / residual learner adapts continuous skill parameters $(\theta_{\text{pitch}}, \omega_{\text{rock}}, A_{\text{yaw}}, v_{\text{align}})$ to observed sinkage $\Delta z$, yaw skew $\Delta\psi$, and slip feedback.<br>**System 0 (Fast $250\text{ Hz}$ Mathematical Controller):** Closed-loop $SE(2)$ funnel servoing + crop-row safety barrier ($|y - y_{\text{row}}| \le 0.04\text{ m}$). | Combines **100% deterministic crop-row safety** with **AI generalization and multi-technique improvisation** across random yaw skews and deep clay suction traps. | Requires maintaining the 4-skill library seam and logging skill selection for evaluation. |

---

### 2.3 Multi-Technique Rescue Library & 12-Scenario Randomized MuJoCo Benchmark

To verify that homogeneous robots can rescue each other across randomized field conditions, we evaluated **12 randomized mud-entrapment scenarios** (`seed=42`, varying lateral offset $\Delta y \in [-30, +30]\text{ mm}$, yaw skew $\Delta\psi \in [-17^\circ, +17^\circ]$, sinkage depth $z \in [37.6, 44.1]\text{ mm}$, and clay suction factor $\kappa \in [0.99, 2.06]$) across four composable techniques:

1. **`SELF_OSCILLATE_DIBBER`:** Unassisted $15\text{ Hz}$ grouser torque oscillation + vertical dibber plunger (`-50 mm`) hardpan push-off (succeeds only in shallow slurry $\kappa < 1.05$, saving peer travel).
2. **`DIRECT_PRY_AND_PULL`:** Closed-loop $SE(2)$ approach + cam-latch lock + spade anchor + static pitch-lift ($+0.36\text{ rad}$) + 8-wheel tandem reverse pull.
3. **`BUMPER_REALIGN_THEN_PULL`:** When $|\Delta\psi| > 8.5^\circ$ or $|\Delta y| > 15\text{ mm}$, uses closed-loop proportional funnel servoing ($\omega = k_\psi (\text{atan2}(\Delta y, \Delta x) - \psi)$) and tapered cone-funnel compliance to straighten the bogged robot's yaw before locking the cam-latch.
4. **`DYNAMIC_YAW_SHEAR_AND_PITCH_PUMP` ("Ant Wiggle"):** When static pull stalls in deep clay ($\kappa > 1.75$), anchors rear spades ($+0.75\text{ rad}$) and applies $8\text{ Hz}$ alternating yaw-shear ($\pm 10\text{ rad/s}$) + cyclic pitch-boom pumping ($0.15 \leftrightarrow 0.42\text{ rad}$) to break the clay vacuum seal before towing.

#### Observed MuJoCo 4.0.0 Benchmark Summary (`12` Randomized Mud Scenarios)

| Method | Successful Extractions | Rigid Cam-Latch Lock Rate | Mean Backward Extraction Distance | Primary Failure Mode Observed |
| :--- | :--- | :--- | :--- | :--- |
| **1. `self_recovery_only`** | **1 / 12 (8.3%)** | `0 / 12` (N/A) | `0.053 m` | Wheels churn in place; cannot overcome clay suction $\kappa \ge 1.10$ alone. |
| **2. `fixed_predefined_pull`** | **7 / 12 (58.3%)** | `9 / 12 (75.0%)` | `0.386 m` | Misses rear funnel in 3 high-skew cases; stalls after latching in 2 deep-suction cases ($\kappa > 1.75$). |
| **3. `adaptive_multi_technique` (Hybrid)** | **12 / 12 (100.0%)** | **12 / 12 (100.0%)** | **`0.607 m`** | None — dynamically switches between `self_oscillate_dibber` (1), `direct_pry_and_pull` (3), `bumper_realign_then_pull` (4), and `dynamic_yaw_shear_and_pitch_pump` (4). |

---

## 3. Public Entry Points
- `RoverChassisSpec`, `PLANTER_SPEC`, `SCOUT_SPEC`, `DEFAULT_CAPABILITIES`, `make_homogeneous_rover_spec` in [`src/ricebot/robot/rovers.py`](rovers.py)
- `compute_4wheel_skid_steer(spec, linear_m_s, angular_rad_s) -> (w_fl, w_fr, w_rl, w_rr)`
- `build_rovers_and_seeds_xml() -> (bodies_xml, actuators_xml, sensors_xml)` (composed by `src/ricebot/mujoco_sim/adapter.py`)
- `build_rigid_docking_rescue_mjcf() -> str` & `update_rigid_dock_latch(model, data, rescuer_id, target_id) -> bool`
- `export_robot_mjcf_assets(output_dir=None) -> (single_unit_xml_path, tandem_rescue_xml_path)`

## 4. Standalone MJCF Assets
- [`src/ricebot/robot/assets/paddy_ant_unit.xml`](assets/paddy_ant_unit.xml): Single homogeneous Paddy Ant unit with all 10 actuators (4 wheels + front pitch-lift boom + rotary cam-latch + rear hardpan anchor spades + vertical seed/soil dibber plunger + 2 counter-rotating finger weeders) and 3-ray LiDAR fan.
- [`src/ricebot/robot/assets/paddy_ant_rigid_rescue.xml`](assets/paddy_ant_rigid_rescue.xml): Two-robot collinear rigid docking, cam-latching, pry-lift suction-breaking, and tandem 8-wheel extraction scene.

## 5. Input and Output Contracts Used
- Consumes bounded `(linear_m_s, angular_rad_s)` velocity commands from `RobotCommand` and outputs 4-wheel angular velocity targets in `rad/s`.
- Exports `RobotCapability` tuples (`DEFAULT_CAPABILITIES`) matching [docs/CONTRACTS.md](../../../docs/CONTRACTS.md).

## 6. Prohibited Dependencies
- Must **never** own field patch layouts, hidden soil truth, or location-gated soil sampling (those belong to [src/ricebot/world/README.md](../world/README.md)).
- Must **never** import `MissionCore` or modify mission planning rules.

## 7. Verification & Live MuJoCo Simulation Commands
```powershell
cd C:\Users\julia\Coding\Agentverse2026

# 1. Run all 50 unit + world/robot + agentic swarm + MuJoCo physics tests
.\.venv\Scripts\python.exe -m unittest discover -s tests -v

# 2. Launch the full 3D Rice Terrace simulation (Farming + Mud Entrapment + Rigid Cam-Latch Rescue + Re-Planting)
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --view

# 3. Launch the zoomed-in 3D mechanical demonstration (Weeding + Dibber Plunging + Rigid Docking Rescue)
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --scenario closeup_mechanics --view
```

## 8. Known Limitations
- Wheel slip amplification (`1.85x`) is calibrated for 4-wheel differential skid-steer on the flat paddy terrace plane (`friction="1.4 0.01 0.005"`). When translating to `src/ricebot/isaac/`, the same `compute_4wheel_skid_steer` helper maps directly to `ArticulationAction(joint_velocities=...)` on the 4 wheel revolute joints.
