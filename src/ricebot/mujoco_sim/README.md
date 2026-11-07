# MuJoCo Rapid-Prototyping Adapter (`src/ricebot/mujoco_sim/`)

## 1. Purpose, Beneficiary Outcome & Current Scope
- **Purpose:** Provides a fast, zero-cloud-spend **MuJoCo 4.0.0** implementation of the `SimulationPort` protocol (`reset(seed)` and `step(commands)`) for rapid prototyping on 8 GB VRAM laptops, before translating the physical scene to a teammate's **NVIDIA Isaac Sim** workstation.
- **What actually works:**
  - Two physically actuated rovers (`planter` and `scout`) with 2D planar velocity + yaw actuators and collision boxes.
  - Real MuJoCo raycast rangefinders (`mjSENS_RANGEFINDER`) on both rovers detecting unannounced mid-episode obstacles.
  - Location-gated synthetic soil/stand probes (`moisture_proxy`, `firmness_proxy`, `stand_gap_proxy`) available only when a rover is within `probe_radius_m` (`0.22 m`) of a permitted site.
  - Genuine 6-DOF rigid-body seed proxies (`seed_proxy_0`, `seed_proxy_1`) carried in the planter's hopper, released on a valid `RobotCommand(release_seed=True)`, and verified via MuJoCo's physical contact buffer (`mjData.contact`) against `field_ground` before emitting `PlantFeedback`.
- **Out of scope:** NVIDIA Isaac Sim USD/PhysX execution (owned by `src/ricebot/isaac/` on a 16 GB+ VRAM host), CFD hydrology, and real agronomic yield prediction.

## 2. Ownership & Upstream/Downstream Boundaries
- **Consumes:** `src/ricebot/contracts.py` (`ObservationFrame`, `RobotObservation`, `RobotCommand`, `PlantFeedback`, `SoilSample`, `Site`, `Pose2D`) and `src/ricebot/runtime.py` (`SimulationPort`).
- **Never modifies:** `src/ricebot/contracts.py`, `src/ricebot/mission.py`, `src/ricebot/control.py`, or `src/ricebot/runtime.py`.
- **Translation to Isaac Sim:** Because `MujocoPaddySimulation` strictly implements `SimulationPort`, `MissionCore` and `EpisodeRunner` run unmodified when `MujocoPaddySimulation` is swapped for `IsaacPaddySimulation`.

## 3. Public Interface
- `MujocoPaddySimulation(sites=DEFAULT_SITES, probe_radius_m=0.22, substeps_per_tick=20, inject_obstacle_at_s=None, obstacle_xy=(0.95, 0.0))`
  - `.reset(seed: int) -> ObservationFrame`
  - `.step(commands: tuple[RobotCommand, ...]) -> ObservationFrame`
  - `.completed_drops -> tuple[SeedDropEvent, ...]` (independent physics contact record for evaluation)

## 4. Setup & Exact Run / Test Commands
Requires Python 3.10+ with `mujoco` installed in `.venv`:

```powershell
# Run automated unit + physics integration tests
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_mujoco_sim.py" -v

# Run the Antenna MuJoCo simulator demo (headless trace + optional interactive 3D viewer)
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --headless
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --viewer-seconds 8
```

## 5. Invariants & Failure Handling
- **No Oracle Leakage:** Hidden site truth (`_hidden_site_truth`) is never passed to `MissionCore`; rovers must physically drive within `0.22 m` of a site center to sample it.
- **Command Safety:** Expired/stale `RobotCommand` messages, non-zero drive velocities during `release_seed=True`, or plant requests from `scout` are rejected at the physics bridge.
- **Physical Seed Contact:** `PlantFeedback(released=True, landing=...)` is only emitted when `mjData.contact` reports a physical collision between `seed_X_geom` and `field_ground`.
