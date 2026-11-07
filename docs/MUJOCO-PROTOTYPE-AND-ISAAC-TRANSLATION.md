# MuJoCo Rapid-Prototyping SimulationPort & Isaac Sim Translation Protocol

## 1. Why MuJoCo First → Isaac Sim Next
- **Hardware Constraint on 8 GB VRAM Laptops:** NVIDIA Isaac Sim 5.1 specifies a minimum of **32 GB System RAM** and **16 GB GPU VRAM** for supported workstation operation. Developer laptops with **8 GB VRAM / ~16 GB RAM** can run **MuJoCo 4.0.0** at `>1,000x` real-time with zero cloud spend.
- **Architectural Seam (`SimulationPort`):** The PaddyPatch core (`src/ricebot/contracts.py`, `src/ricebot/mission.py`, `src/ricebot/control.py`, `src/ricebot/runtime.py`) is 100% simulator-independent and interacts with the physical world exclusively through:
  ```python
  class SimulationPort(Protocol):
      def reset(self, seed: int) -> ObservationFrame: ...
      def step(self, commands: tuple[RobotCommand, ...]) -> ObservationFrame: ...
  ```
- **Zero-Rewrite Guarantee:** Everything built against `MujocoPaddySimulation` (`src/ricebot/mujoco_sim/`) runs unmodified when `IsaacPaddySimulation` (`src/ricebot/isaac/`) is plugged into `EpisodeRunner(sim, core, view)` on the teammate's Isaac Sim machine.

---

## 2. Running the MuJoCo PaddyPatch Simulator Locally

From the repository root using a Python 3.10+ `.venv` with `mujoco` installed (`pip install mujoco`):

```powershell
# 1. Run headless episode (dual-rover soil sampling + physical seed drop + ground-contact JSON trace)
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --headless

# 2. Launch interactive 3D MuJoCo viewer showing both rovers, site markers, and physical seed drop
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --viewer-seconds 8

# 3. Run mid-episode obstacle surprise scenario (obstacle injected at t=1.5s, rangefinder triggers safe HOLD)
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --scenario obstacle --headless

# 4. Run full unit + MuJoCo physics test suite
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

---

## 3. Exact 1-to-1 Translation Protocol to Isaac Sim (`src/ricebot/isaac/`)

When translating [`MujocoPaddySimulation`](../src/ricebot/mujoco_sim/adapter.py) into `src/ricebot/isaac/` on the Isaac Sim workstation, preserve every coordinate frame, unit, and invariant below:

| Boundary / Component | MuJoCo Prototype (`src/ricebot/mujoco_sim/adapter.py`) | Isaac Sim Target (`src/ricebot/isaac/` on ≥16 GB VRAM host) | Shared Contract Invariant (`src/ricebot/contracts.py`) |
| :--- | :--- | :--- | :--- |
| **World Coordinate Frame & Units** | Right-handed metric $(x, y, z)$ in metres, yaw in radians $(-\pi, \pi]$, monotonic `sim_time_s` in seconds (`dt = 0.005 s`, 20 substeps = `0.10 s` per tick). | OpenUSD stage with `UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)` and `metersPerUnit = 1.0`; physics step `0.10 s` per `SimulationPort.step()`. | All `Pose2D(x_m, y_m, yaw_rad)` and timestamps must be finite and strictly increasing (`frame.time_s > last_time`). |
| **Field Sites (`DEFAULT_SITES`)** | `patch_a` $(0.90, -0.45)$, `patch_b` $(1.55, 0.00)$, `patch_c` $(1.55, 0.50)$, and `protected_crop` $(0.75, 0.55, \text{permitted}=\text{False})$. | Identical `Site` tuple and USD Xform markers at the exact same $(x, y)$ coordinates. | Protected sites (`permitted=False`) must never be sampled or crossed by `LocalController`. |
| **Dual Rover Bodies (`planter`, `scout`)** | `planter_body` starting at $(0.10, 0.00, 0.07)$ with `seeds_remaining=2`; `scout_body` starting at $(0.10, -0.45, 0.07)$ with `seeds_remaining=0`. | Two USD robot articulations (`planter`, `scout`) spawned at identical initial poses; wheel/planar controllers accepting bounded `(linear_m_s, angular_rad_s)`. | Expired commands (`not (issued_at_s <= now_s < expires_at_s)`) must be rejected at the simulator bridge (safe zero-velocity hold). |
| **Front Range Sensor (`front_range_m`)** | `mjSENS_RANGEFINDER` mounted at $+x$ nose of each rover (`planter_front_range`, `scout_front_range`), capped at `4.0 m`. | PhysX raycast sensor (`isaacsim.sensors.physics` / `physx_scene_query_interface().raycast_closest`) along rover $+x$ axis, max `4.0 m`. | Returns `float` in metres (`>= 0.0`) or `None` if sensor is offline; `< 0.35 m` triggers `LocalController` safe stop. |
| **Location-Gated Soil & Stand Probes (`SoilSample`)** | `_local_samples_for_robot()` emits `MOISTURE`, `FIRMNESS`, and `STAND_GAP` samples **only** when `hypot(robot.x - site.x, robot.y - site.y) <= 0.22 m`. | Identical `_local_samples_for_robot()` helper querying the private synthetic truth table only when rover odometry is within `0.22 m` of a site center. | Hidden field truth (`_hidden_site_truth`) is **never** exposed outside the simulator/evaluator; `sample.sampled_at_s == frame.time_s`. |
| **Physical Seed Release & Contact (`PlantFeedback`)** | On valid `RobotCommand(release_seed=True)` from `planter` with `linear_m_s == 0` and `angular_rad_s == 0`, releases 6-DOF `freejoint` sphere `seed_proxy_X` from hopper at $z = 0.06\text{ m}$ and verifies physical collision between `seed_X_geom` and `field_ground` in `mjData.contact`. | On valid `release_seed=True`, releases rigid-body USD sphere (`UsdPhysics.RigidBodyAPI` + `UsdPhysics.CollisionAPI`) from hopper and reads PhysX contact report with `field_ground` before emitting `PlantFeedback`. | Inventory decrements **only** on physical release; `PlantFeedback(released=True, landing=Pose2D(...))` is emitted **only** upon physical ground contact; `scout` can never release seeds. |
| **Mid-Episode Surprise (`inject_obstacle_at_s`)** | Moves mocap body `mid_run_obstacle` onto the active lane $(0.95, 0.00, 0.08)$ at $t \ge t_{\text{inject}}$ without notifying the policy. | Moves/enables USD rigid collider obstacle at $(0.95, 0.00, 0.08)$ at $t \ge t_{\text{inject}}$; policy discovers it strictly via `front_range_m`. | No oracle event flag or privileged obstacle coordinate may cross `SimulationPort`. |

---

## 4. Verification Checklist Before Swapping `MujocoPaddySimulation` → `IsaacPaddySimulation`

1. Instantiate `IsaacPaddySimulation(sites=DEFAULT_SITES)` on the Isaac Sim host.
2. Run the exact same 3 tests in [`tests/test_mujoco_sim.py`](../tests/test_mujoco_sim.py) against `IsaacPaddySimulation`:
   - `test_nominal_episode_samples_both_rovers_and_physically_plants_seed`
   - `test_mid_episode_obstacle_triggers_rangefinder_and_safe_hold`
   - `test_scout_cannot_release_seed_and_expired_commands_rejected`
3. Confirm `EpisodeRunner(isaac_sim, MissionCore(MeasuredFieldMap(DEFAULT_SITES), SimpleMissionPlanner(DEFAULT_CAPABILITIES))).run(seed=7, steps=120)` completes without changing a single line of `src/ricebot/contracts.py`, `src/ricebot/mission.py`, `src/ricebot/control.py`, or `src/ricebot/runtime.py`.
