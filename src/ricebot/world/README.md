# World, Sensors and Peer Information (`src/ricebot/world/`)

## 1. Purpose, Beneficiary Outcome & Current Scope
- **Purpose:** Owns the 6-patch rice terrace layout (`DEFAULT_SITES`), public traversability/no-go constraints, hidden synthetic spatial soil/stand-gap distribution (`DEFAULT_HIDDEN_SOIL_TRUTH`), location-gated local soil probe sampler (`PaddyTerraceWorld.sample_local_soil`), 3D multi-level terraced field landscape with 40° ramps (`build_world_terrain_xml`), photorealistic rice field textures and materials (`src/ricebot/world/field_textures.py`), and peer-to-peer radio transport and RF blackout simulation (`src/ricebot/world/comms.py`).
- **What actually works:**
  - Models Southern China rice terraces with 1.0m height drops between tiers, dedicated 40° incline ramps, and impassable stone retaining walls.
  - Full photorealistic materials for young rice paddy crop canopy, water reflection, sinuous earthen bunds, and retaining walls.
  - Models 2 *Lannitian* waterlogged mud hazards (`patch_a`, `patch_d`), 2 viable missing-stand loam patches (`patch_b`, `patch_c`), and 2 protected living rice seedling stands (`crop_nw`, `crop_ne`).
  - Enforces strict sensor locality: `sample_local_soil(robot_id, pose, time_s)` emits `SoilSample` measurements **only** when a rover is within `probe_radius_m = 0.28 m` of a permitted site center.
  - Supports epistemic realism and sensor noise drift under the `sensor_degraded` scenario (Audit Finding N3).
  - Simulates Sub-1 GHz / UWB local radio propagation (`comm_range_m = 1.2 m`) with intermittent (60.5%) and complete valley RF blackout modes, enforcing epistemic isolation so rovers never receive out-of-range peer poses (Audit Findings M6, T6, T7).
- **Out of scope:** Rover chassis, wheel joints, skid-steer kinematics, seed hopper geometry (owned by [`src/ricebot/robot/`](../robot/README.md)), mission planning/agent policy (owned by [`src/ricebot/agent/`](../agent/README.md)), and canonical kernel contracts (owned by Agent A in [`src/ricebot/contracts.py`](../contracts.py)).

## 2. Human Owner, Agent & Boundaries
- **Role Ownership:** Agent C — Sensing, World, and Peer Information ([`docs/FIVE-AGENT-RECONCILIATION.md`](../../../docs/FIVE-AGENT-RECONCILIATION.md), [`docs/COMPONENTS.md`](../../../docs/COMPONENTS.md)).
- **Upstream/Downstream Contract:** Consumes `Pose2D`, `Site`, `SoilSample`, `SensorKind`, `RobotObservation` from `src/ricebot/contracts.py`. Consumed by the simulation composition layer (`src/ricebot/mujoco_sim/adapter.py`) and agent fleet coordination (`src/ricebot/agent/mycelial_swarm.py`).

## 3. Public Interface
- `DEFAULT_SITES: tuple[Site, ...]`
- `PaddyTerraceWorld(sites=DEFAULT_SITES, probe_radius_m=0.28, hidden_truth=None)`
  - `.reset(seed: int) -> None`
  - `.sample_local_soil(robot_id: str, pose: Pose2D, time_s: float, noise_std: float = 0.0, scenario: str = "nominal") -> tuple[SoilSample, ...]`
- `build_world_terrain_xml(obstacle_xy=(0.75, -0.25), use_3d_terraces: bool = False) -> str`
- `build_textured_asset_xml(textures_dir=None) -> str`
- `enhance_mjcf_with_field_textures(mjcf_xml: str) -> str`
- `PaddyP2PCommsChannel(comm_range_m=1.2, mode="normal", intermittent_cycle_s=4.0, intermittent_blackout_fraction=0.605)`
  - `.evaluate_link(tx_id, tx_pose, rx_id, rx_pose, time_s) -> PeerLinkStatus`
  - `.filter_peer_observations_for_robot(query_robot_id, query_pose, all_robot_observations, time_s) -> list[RobotObservation]`

## 4. Setup & Exact Run / Test Commands
```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_world_and_robot.py" -v
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_field_textures.py" -v
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_comms_transport.py" -v
```

## 5. Invariants & Failure Handling
- **No Oracle Leakage:** `_hidden_site_truth` is private; out-of-range queries (`dist > probe_radius_m`) and protected sites (`permitted=False`) return `()`.
- **Epistemic Peer Isolation:** Peer observations are strictly filtered by physical radio reach (`comm_range_m`) and environmental RF blackout; unobserved peer state is never passed to local policies.
- **Idempotent Sampling per Episode:** Each `(robot_id, site_id)` pair emits one local sample trio per episode reset so sample IDs remain deterministic and unique.
- **Regression Safety:** `use_3d_terraces=False` preserves exact flat planar terrain for baseline 6-site golden rescue compatibility.

## 6. Extending / Replacing
- Edit `src/ricebot/world/terrace.py` to adjust site coordinates, synthetic moisture/firmness distributions, or terrace geometry without touching `src/ricebot/robot/`.
- Edit `src/ricebot/world/comms.py` to adjust radio propagation parameters, canopy attenuation, or valley blackout schedules.
