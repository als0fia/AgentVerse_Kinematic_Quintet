# Robot Embodiment & Homogeneous Ant Mechanics (`src/ricebot/robot/`)

## 1. Purpose & Homogeneous Ant-Colony Design (Role 2)
Owns the physical rover embodiment (`RoverChassisSpec`), 4-wheel differential skid-steer kinematics (`compute_4wheel_skid_steer`), forward 3-ray LiDAR and odometry sensor mount frames, seed-hopper release nozzle geometry, **combined rigid docking, cam-locking & pry-lift rescue mechanism**, and robot-only articulation/MJCF builders (`build_rovers_and_seeds_xml`, `build_rigid_docking_rescue_mjcf`, `export_robot_mjcf_assets`).

### Key Mechanical & Agronomic Features
1. **100% Homogeneous "Medic-Ant" Swarm Units (`make_homogeneous_rover_spec`):**
   - Inspired by Matabele medic ants (*Megaponera analis*), **every robot in the swarm shares 100% identical physical hardware** (`can_sample=True`, `can_plant=True`, `can_rescue=True`). There is no fragile "scout-only" or "rescue-only" hardware variant.
   - Operational specialization (`SAMPLE`, `PLANT`, `HOLD`, or `RESCUE`) emerges dynamically in software from real-time seed inventory (`seeds_remaining`), proximity to unverified stand gaps, and opportunity cost when a teammate becomes bogged down.
2. **Dimensional Envelope & Southern China Paddy Row Fit (`RoverChassisSpec`):**
   - **Crop-Row Clearance:** Fits inside standard **`300 mm`** machine-transplanted/direct-seeded rice inter-row lanes without trampling living rice tillers (`no_go_radius_m`).
   - **Sealed Waterproof Hull:** `270 × 190 × 64 mm` (`chassis_half_x_m=0.135`, `chassis_half_y_m=0.095`, `chassis_half_z_m=0.032`), hull mass `2.6 kg` (`~4.3 kg` total unit mass in MuJoCo; scales to `<35–50 kg`, `<10–15 kPa` low ground-pressure field rover with `150 mm` buoyant belly skid plate).
   - **4-Wheel Differential Paddy Drive:** `170 mm` wheelbase (`wx=0.085 m`), `236 mm` center track (`half_track_m=0.118 m`), **`276 mm` total outer wheel-to-wheel width**, with four `Ø116 × 40 mm` revolute paddle/lug wheels (`wheel_radius_m=0.058`, `wheel_half_width_m=0.020`, `max_wheel_rad_s=14.0 rad/s`, `max_linear_m_s=0.45 m/s`, `max_angular_rad_s=1.40 rad/s`).
3. **Farming & Agronomic Payload (Present on Every Unit):**
   - **Conical Seed Hopper & Gravity Release Nozzle (`{id}_hopper_outer`, `{id}_hopper_inner`):** Centrally mounted (`x=+0.025 m, y=0.0 m, z=0.075 m`) with bottom release chute at `hopper_drop_z_m=0.032 m`. Dispenses 6-DOF rigid-body pre-germinated seed pellets (`seed_proxy_0`, `seed_proxy_1`, radius `22 mm`, mass `30 g`) only when stopped (`linear_m_s == 0`, `angular_rad_s == 0`) over a verified viable stand gap.
   - **Subsurface Soil Moisture & Cone-Index Firmness Probe (`{id}_probe_boom`, `{id}_probe_tip`):** Rear-left articulated probe mast (`x=-0.075 m, y=-0.048 m`) with subsurface sensor tip (`z=0.018 m`) that measures local `MOISTURE`, `FIRMNESS`, and `STAND_GAP` within `probe_radius_m = 0.28 m` to distinguish viable loam gaps from waterlogged *Lannitian* mud traps.
4. **Combined Rigid Docking, Cam-Latch & Pry-Lift Rescue Mechanism (No Slack Cable / No Winch):**
   - Because robots operate single-file inside a `300 mm` row, a rescuer can only approach **collinearly** (front-to-back). A flexible cable goes slack when pushing and drags a lightweight rescuer into the same mud hole when pulling. Instead, every unit carries a 4-part rigid coupling & suction-breaking mechanism:
     1. **Front Active Pitch-Lift Docking Boom & Tapered Cone (`{id}_dock_boom`, `{id}_dock_cone`, `{id}_front_dock_site`):** Mounted on the front nose (`x=+0.135 m, z=0.012 m`) with an active pitch hinge (`{id}_dock_pitch`, `-0.20` to `+0.45 rad` / `+26°`, `kp=35.0`) to pry/lift a stuck robot's chassis upward (`+Z`) and break hydrostatic mud suction.
     2. **Internal Rotary Cam-Locking Jaw (`{id}_latch_jaw`, `{id}_latch_j`, `{id}_latch_hook`):** Actuated rotary hook (`0.0` to `0.65 rad`, `kp=15.0`) inside the front cone. When seated in a teammate's rear funnel (`distance <= 0.065 m`) with `{id}_latch_act >= 0.30 rad`, `update_rigid_dock_latch()` engages `rigid_latch_{rescuer}_to_{target}`, locking both 4-wheel units into a **single rigid 8-wheel articulated train** that can pull in reverse or push forward without jackknifing.
     3. **Rear Passive V-Guide Alignment Funnel & Strike Bar (`{id}_rear_funnel_l/r`, `{id}_rear_strike_bar`, `{id}_rear_dock_site`):** Angled guide plates (`±0.35 rad`) at `x=-0.150 m` that self-center a mating nose cone onto the horizontal strike bar (`x=-0.148 m`).
     4. **Rear Deployable Hardpan Anchor Spades (`{id}_spade_arm`, `{id}_spade_j`, `{id}_spade_l/r`):** Twin folding Danforth-style flukes (`0.0` to `0.85 rad`, `kp=20.0`) at `x=-0.132 m` that dig into the compacted hardpan to anchor a stationary rescuer during high-load prying.
5. **Perception & Navigation Suite:**
   - **Forward 3-Ray LiDAR / Rangefinder Turret (`{id}_lidar_pod`, `{id}_range_c`, `{id}_range_l`, `{id}_range_r`):** Elevated at `x=+0.112..0.140 m, z=0.048 m` (clear above the front docking cone), casting a 3-beam horizontal fan (`0°` center, `+18°` left, `-18°` right, `4.0 m` cutoff) for deterministic obstacle stops (`<= 0.32–0.35 m`).
   - **BeiDou (BDS-3) RTK GNSS Mast & Odometry Site (`{id}_rtk_mast`, `{id}_rtk_dome`, `{id}_odom_site`):** Top-mounted mast (`z=0.135 m`) providing `Pose2D(x_m, y_m, yaw_rad)` odometry.

## 2. Public Entry Points
- `RoverChassisSpec`, `PLANTER_SPEC`, `SCOUT_SPEC`, `DEFAULT_CAPABILITIES`, `make_homogeneous_rover_spec` in [`src/ricebot/robot/rovers.py`](rovers.py)
- `compute_4wheel_skid_steer(spec, linear_m_s, angular_rad_s) -> (w_fl, w_fr, w_rl, w_rr)`
- `build_rovers_and_seeds_xml() -> (bodies_xml, actuators_xml, sensors_xml)` (composed by `src/ricebot/mujoco_sim/adapter.py`)
- `build_rigid_docking_rescue_mjcf() -> str` & `update_rigid_dock_latch(model, data, rescuer_id, target_id) -> bool`
- `export_robot_mjcf_assets(output_dir=None) -> (single_unit_xml_path, tandem_rescue_xml_path)`

## 3. Standalone MJCF Assets
- [`src/ricebot/robot/assets/paddy_ant_unit.xml`](assets/paddy_ant_unit.xml): Single homogeneous Paddy Ant unit with all 7 actuators (4 wheels + front pitch-lift boom + rotary cam-latch + rear hardpan anchor spades) and 3-ray LiDAR fan.
- [`src/ricebot/robot/assets/paddy_ant_rigid_rescue.xml`](assets/paddy_ant_rigid_rescue.xml): Two-robot collinear rigid docking, cam-latching, pry-lift suction-breaking, and tandem 8-wheel extraction scene.

## 4. Input and Output Contracts Used
- Consumes bounded `(linear_m_s, angular_rad_s)` velocity commands from `RobotCommand` and outputs 4-wheel angular velocity targets in `rad/s`.
- Exports `RobotCapability` tuples (`DEFAULT_CAPABILITIES`) matching [docs/CONTRACTS.md](../../../docs/CONTRACTS.md).

## 5. Prohibited Dependencies
- Must **never** own field patch layouts, hidden soil truth, or location-gated soil sampling (those belong to [src/ricebot/world/README.md](../world/README.md)).
- Must **never** import `MissionCore` or modify mission planning rules.

## 6. Verification & Standalone MuJoCo Inspection Commands
```powershell
# Run all unit + world/robot + MuJoCo rigid docking & terrace physics tests
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_world_and_robot.py" -v
.\.venv\Scripts\python.exe -m unittest discover -s tests -v

# Export standalone MJCF assets and open in interactive MuJoCo 3D viewer
.\.venv\Scripts\python.exe -c "import sys; sys.path.insert(0, 'src'); from ricebot.robot import export_robot_mjcf_assets; print(export_robot_mjcf_assets())"
.\.venv\Scripts\python.exe -m mujoco.viewer --mjcf=src/ricebot/robot/assets/paddy_ant_unit.xml
.\.venv\Scripts\python.exe -m mujoco.viewer --mjcf=src/ricebot/robot/assets/paddy_ant_rigid_rescue.xml
```

## 7. Known Limitations
- Wheel slip amplification (`1.85x`) is calibrated for 4-wheel differential skid-steer on the flat paddy terrace plane (`friction="1.4 0.01 0.005"`). When translating to `src/ricebot/isaac/`, the same `compute_4wheel_skid_steer` helper maps directly to `ArticulationAction(joint_velocities=...)` on the 4 wheel revolute joints.
