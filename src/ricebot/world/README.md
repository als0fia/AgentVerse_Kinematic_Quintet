# World, Sensors and Peer Information (`src/ricebot/world/`)

## 1. Purpose, Beneficiary Outcome & Current Scope
- **Purpose:** Owns the rice-terrace world geometry, public traversability/no-go constraints, hidden synthetic spatial soil/stand-gap distribution (`DEFAULT_HIDDEN_SOIL_TRUTH`), location-gated local soil probe sampler (`PaddyTerraceWorld.sample_local_soil`), multi-level Chinese terraced rice-paddy navigation network (`MultiLevelTerraceEnvironment`, `MultiLevelPathPlanner`), photorealistic rice field textures and materials (`src/ricebot/world/field_textures.py`), and peer-to-peer radio transport and RF blackout simulation (`src/ricebot/world/comms.py`).
- **What actually works:**
  - **Multi-Level Chinese Terraced Rice Fields (`terrace_network.py`):** Configurable stepped hillside environment (`TerraceLevel`, `RampConnection`, `MultiLevelTerraceEnvironment`) supporting both $1.0\,\text{m}$ step demonstrations (`0.0m, 1.0m, 2.0m` with a $0.50\,\text{m}$ / `1/2 m` agricultural paddy rover) and $0.5\,\text{m}$ step demonstrations (`0.0m, 0.5m, 1.0m`), with recessed-flank flooded paddy shelves, contoured stone retaining walls, golden earthen bunds (`田埂`), solid earth-supported ramps (`Ramp AB`, `Ramp BC`), and 3D green rice seedling rows.
  - **Multi-Level Topological & Geometric Navigation (`MultiLevelPathPlanner`):** Plans collision-free routes across terrace levels (`plan_path`) and simulates continuous time-stepped trajectories (`simulate_continuous_navigation` returning `TrajectoryState`) with continuous surface elevation $z(t)$, signed ramp pitch $\theta_{\text{pitch}}(t)$, wheel rotation $\phi(t)$, speed reduction at ramp entrances, blocked-ramp rejection (Test 3), steep-slope rejection (Test 4), and crop-zone protection (Test 5).
  - **Photorealistic Textures & Materials (`field_textures.py`):** Seamless textures in `assets/textures/` for organic paddy loam (`paddy_viable_loam.png`), compacted golden bund earth (`terrace_bund_earth.png`), weathered stone retaining walls (`terrace_retaining_wall.png`), sky-reflecting water (`paddy_water_mirror.png`), saturated *Lannitian* mud (`paddy_mud_hazard.png`), and submerged rice rows (`rice_field_paddy.png`), with coplanar heightfield z-fighting suppression.
  - **6-Site Baseline & Sensor Locality (`terrace.py`):** Models 2 *Lannitian* waterlogged mud hazards (`patch_a`, `patch_d`), 2 viable missing-stand loam patches (`patch_b`, `patch_c`), and 2 protected living rice seedling stands (`crop_nw`, `crop_ne`). Enforces strict sensor locality (`probe_radius_m = 0.28 m`) and sensor noise drift under `sensor_degraded` (Audit Finding N3).
  - **Epistemic Peer Isolation & RF Blackout (`comms.py`):** Simulates Sub-1 GHz / UWB local radio propagation (`comm_range_m = 1.2 m`) with intermittent (60.5%) and complete valley RF blackout modes (Audit Findings M6, T6, T7).
- **Out of scope:** Canonical `PLANTER_SPEC` / `SCOUT_SPEC` chassis and rigid-docking rescue mechanics (owned by Agent B in [`src/ricebot/robot/`](../robot/README.md)), mission planning/agent policy (owned by Agent D in [`src/ricebot/agent/`](../agent/README.md)), and canonical kernel contracts (owned by Agent A in [`src/ricebot/contracts.py`](../contracts.py)).

## 2. Human Owner, Agent & Boundaries
- **Role Ownership:** Agent C — Sensing, World, and Peer Information ([`docs/FIVE-AGENT-RECONCILIATION.md`](../../../docs/FIVE-AGENT-RECONCILIATION.md), [`docs/COMPONENTS.md`](../../../docs/COMPONENTS.md)).
- **Upstream/Downstream Contract:** Consumes `Pose2D`, `Site`, `SoilSample`, `SensorKind`, `RobotObservation` from [`src/ricebot/contracts.py`](../contracts.py) (see [`docs/CONTRACTS.md`](../../../docs/CONTRACTS.md)). Consumed by the simulation composition layer (`src/ricebot/mujoco_sim/adapter.py`), multi-level terrace runner (`scripts/run_multi_level_terrace_navigation.py`), and agent fleet coordination (`src/ricebot/agent/mycelial_swarm.py`).

## 3. Public Interface
- **Baseline 6-Site World & Soil Probe (`src/ricebot/world/terrace.py`):**
  - `DEFAULT_SITES: tuple[Site, ...]`, `DEFAULT_HIDDEN_SOIL_TRUTH`, `UNEXPECTED_BAD_SOIL_TRUTH`
  - `PaddyTerraceWorld(sites=DEFAULT_SITES, probe_radius_m=0.28, hidden_truth=None)`
    - `.reset(seed: int) -> None`
    - `.sample_local_soil(robot_id: str, pose: Pose2D, time_s: float, noise_std: float = 0.0, scenario: str = "nominal") -> tuple[SoilSample, ...]`
  - `build_world_terrain_xml(obstacle_xy=(0.75, -0.25), use_3d_terraces: bool = False) -> str`
- **Multi-Level Terrace Network & Continuous Planner (`src/ricebot/world/terrace_network.py`):**
  - `TerraceLevel`, `RampConnection`, `NavigationWaypoint`, `TrajectoryState`
  - `MultiLevelTerraceEnvironment(elevation_a=0.0, elevation_b=0.5, elevation_c=1.0, ramp_slope_deg=10.0, ramp_width_m=0.55, robot_max_slope_deg=15.0, robot_width_m=0.276, robot_length_m=0.50)`
    - `.get_elevation_and_pitch(x_m: float, y_m: float) -> tuple[float, float]`
    - `.locate_region(x_m: float, y_m: float) -> str`
    - `.set_ramp_accessibility(ramp_id: str, accessible: bool) -> None`
  - `MultiLevelPathPlanner(env: MultiLevelTerraceEnvironment)`
    - `.plan_path(start_xy, goal_xy, robot_max_slope_deg=15.0, robot_width_m=0.276) -> list[NavigationWaypoint]`
    - `.simulate_continuous_navigation(start_xy, goal_xy, dt_s=0.05, wheel_radius_m=0.095) -> list[TrajectoryState]`
- **Field Textures (`src/ricebot/world/field_textures.py`):**
  - `build_textured_asset_xml(textures_dir=None) -> str`
  - `enhance_mjcf_with_field_textures(mjcf_xml: str, textures_dir=None) -> str`
- **Peer Communications & RF Blackout (`src/ricebot/world/comms.py`):**
  - `PaddyP2PCommsChannel(comm_range_m=1.2, mode="normal", intermittent_cycle_s=4.0, intermittent_blackout_fraction=0.605)`
    - `.evaluate_link(tx_id, tx_pose, rx_id, rx_pose, time_s) -> PeerLinkStatus`
    - `.filter_peer_observations_for_robot(query_robot_id, query_pose, all_robot_observations, time_s) -> list[RobotObservation]`

## 4. Setup & Exact Run / Test Commands
```powershell
# Run scoped unit tests for world, multi-level navigation, textures, and comms
$env:PYTHONPATH="."
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_multi_level_navigation.py" -v
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_world_and_robot.py" -v
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_field_textures.py" -v
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_comms_transport.py" -v

# Run 3D multi-level terrace navigation (1.0m steps, ~0.50m robot) and generate PNG renders
.\.venv\Scripts\python.exe scripts/run_multi_level_terrace_navigation.py

# Launch interactive 3D MuJoCo viewer
.\.venv\Scripts\python.exe scripts/run_multi_level_terrace_navigation.py --viewer --viewer-seconds 30
```

## 5. Invariants & Failure Handling
- **No Oracle Leakage:** `_hidden_site_truth` is private; out-of-range queries (`dist > probe_radius_m`) and protected sites (`permitted=False`) return `()`.
- **No Teleportation or Wall/Crop Penetration:** `MultiLevelPathPlanner` rejects blocked ramps (`is_accessible=False`), steep slopes (`slope_deg > robot_max_slope_deg`), and narrow passages (`width_m < robot_width_m`), and routes around cultivated rice zones via designated access lanes.
- **Epistemic Peer Isolation:** Peer observations are strictly filtered by physical radio reach (`comm_range_m`) and environmental RF blackout; unobserved peer state is never passed to local policies.
- **Regression Safety:** `use_3d_terraces=False` in `build_world_terrain_xml` preserves exact flat planar terrain for baseline 6-site `golden_rescue` compatibility.

## 6. Extending / Replacing
- Edit `src/ricebot/world/terrace_network.py` and `scripts/run_multi_level_terrace_navigation.py` to configure terrace elevations, ramp slopes, or switchback corridors without modifying `src/ricebot/robot/` or `src/ricebot/contracts.py`.
- Edit `src/ricebot/world/terrace.py` to adjust 6-site coordinates or synthetic moisture/firmness distributions.
- Edit `src/ricebot/world/comms.py` to adjust radio propagation parameters or valley blackout schedules.
