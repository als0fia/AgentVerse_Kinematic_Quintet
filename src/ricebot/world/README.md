# World and Sensor Host (`src/ricebot/world/`)

## 1. Purpose, Beneficiary Outcome & Current Scope
- **Purpose:** Owns the 6-patch rice terrace layout (`DEFAULT_SITES`), public traversability/no-go constraints, hidden synthetic spatial soil/stand-gap distribution (`DEFAULT_HIDDEN_SOIL_TRUTH`), location-gated local soil probe sampler (`PaddyTerraceWorld.sample_local_soil`), and the world-only terrain/bunds/crops MJCF fragment (`build_world_terrain_xml`).
- **What actually works:**
  - Models a bunded Southern China rice plot with 2 *Lannitian* waterlogged mud hazards (`patch_a`, `patch_d`), 2 viable missing-stand loam patches (`patch_b`, `patch_c`), and 2 protected living rice seedling stands (`crop_nw`, `crop_ne`).
  - Enforces strict sensor locality: `sample_local_soil(robot_id, pose, time_s)` emits `SoilSample` measurements (`moisture_proxy`, `firmness_proxy`, `stand_gap_proxy`) **only** when a rover is within `probe_radius_m = 0.28 m` of a permitted site center.
- **Out of scope:** Rover chassis, wheel joints, skid-steer kinematics, seed hopper geometry (owned by [`src/ricebot/robot/`](../robot/README.md)), and mission planning (`src/ricebot/mission.py`).

## 2. Human Owner, Agent & Boundaries
- **Role Ownership:** Role 3 — World and sensor host ([`docs/COMPONENTS.md`](../../../docs/COMPONENTS.md), Issue #11).
- **Upstream/Downstream Contract:** Consumes `Pose2D`, `Site`, `SoilSample`, `SensorKind` from `src/ricebot/contracts.py` ([`docs/CONTRACTS.md`](../../../docs/CONTRACTS.md)). Consumed by the composition layer (`src/ricebot/mujoco_sim/` now; `src/ricebot/isaac/` on the Isaac Sim host).

## 3. Public Interface
- `DEFAULT_SITES: tuple[Site, ...]`
- `PaddyTerraceWorld(sites=DEFAULT_SITES, probe_radius_m=0.28, hidden_truth=None)`
  - `.reset(seed: int) -> None`
  - `.sample_local_soil(robot_id: str, pose: Pose2D, time_s: float) -> tuple[SoilSample, ...]`
- `build_world_terrain_xml(obstacle_xy=(0.75, -0.25)) -> str`

## 4. Setup & Exact Run / Test Commands
```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_world_and_robot.py" -v
```

## 5. Invariants & Failure Handling
- **No Oracle Leakage:** `_hidden_site_truth` is private; out-of-range queries (`dist > probe_radius_m`) and protected sites (`permitted=False`) return `()`.
- **Idempotent Sampling per Episode:** Each `(robot_id, site_id)` pair emits one local sample trio per episode reset so sample IDs remain deterministic and unique.

## 6. Extending / Replacing
- Edit `src/ricebot/world/terrace.py` to adjust site coordinates, synthetic moisture/firmness distributions, or terrain obstacles without touching `src/ricebot/robot/`.
