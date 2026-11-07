# MuJoCo Rapid-Prototyping, Isaac Sim Translation Protocol & Simulation Design Backlog

## 1. Why MuJoCo First → Isaac Sim Next
- **Hardware Constraint on 8 GB VRAM Laptops:** NVIDIA Isaac Sim 5.1 specifies a minimum of **32 GB System RAM** and **16 GB GPU VRAM** for supported workstation operation. Developer laptops with **8 GB VRAM / ~16 GB RAM** run **MuJoCo 4.0.0** at `>1,000x` real-time with zero cloud spend.
- **Three-Way Architectural Split ([docs/COMPONENTS.md](COMPONENTS.md)):**
  1. **World & Sensor Host ([src/ricebot/world/README.md](../src/ricebot/world/README.md)):** Owns the 6-patch bunded rice terrace (`DEFAULT_SITES`), hidden synthetic spatial soil/stand-gap truth (`DEFAULT_HIDDEN_SOIL_TRUTH`), location-gated soil probe sampler (`PaddyTerraceWorld.sample_local_soil`), and world terrain geometry (`build_world_terrain_xml`).
  2. **Robot Embodiment ([src/ricebot/robot/README.md](../src/ricebot/robot/README.md)):** Owns the 4-wheel differential-drive rover specs (`PLANTER_SPEC`, `SCOUT_SPEC`), skid-steer wheel velocity kinematics (`compute_4wheel_skid_steer`), 3-ray forward LiDAR fan, odometry mount frames, and 6-DOF seed-hopper release embodiment (`build_rovers_and_seeds_xml`).
  3. **Simulator Composition Adapter ([src/ricebot/mujoco_sim/README.md](../src/ricebot/mujoco_sim/README.md) now, `src/ricebot/isaac/` next):** Composes `src/ricebot/world/` + `src/ricebot/robot/` behind `SimulationPort` (`reset(seed) -> ObservationFrame`, `step(commands) -> ObservationFrame`).
- **Zero-Rewrite Guarantee:** Everything in `src/ricebot/contracts.py`, `src/ricebot/mission.py`, `src/ricebot/control.py`, `src/ricebot/runtime.py`, `src/ricebot/world/terrace.py`, and `src/ricebot/robot/rovers.py` is simulator-agnostic and reused directly when `IsaacPaddySimulation` (`src/ricebot/isaac/`) is plugged into `EpisodeRunner(sim, core, view)` on a ≥16 GB VRAM machine.

---

## 2. Running the 3D MuJoCo PaddyPatch Simulator Locally

From the repository root (`cd repo_mirror`) using a Python 3.10+ `.venv` with `mujoco` installed:

```powershell
# 1. Launch interactive 3D MuJoCo viewer (bunded terrace, 2 wheeled rovers, live laser rays & belief-map rings)
..\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --viewer-seconds 12

# 2. Run headless episode (dual-rover soil sampling + physical seed drop + ground-contact JSON trace)
..\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --headless

# 3. Run mid-episode obstacle surprise scenario (obstacle injected at t=1.5s, LiDAR fan triggers safe HOLD)
..\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --scenario obstacle --viewer-seconds 10

# 4. Run all 41 unit + world/robot + MuJoCo physics tests
..\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

---

## 3. Exact 1-to-1 Translation Protocol to Isaac Sim (`src/ricebot/isaac/`)

When translating [`MujocoPaddySimulation`](../src/ricebot/mujoco_sim/adapter.py) into `src/ricebot/isaac/` on the Isaac Sim workstation, reuse `PaddyTerraceWorld` (`src/ricebot/world/`) and `compute_4wheel_skid_steer` / `RoverChassisSpec` (`src/ricebot/robot/`) directly:

| Boundary / Component | MuJoCo Prototype (`src/ricebot/mujoco_sim/`) | Isaac Sim Target (`src/ricebot/isaac/` on ≥16 GB VRAM host) | Shared Module / Contract Invariant |
| :--- | :--- | :--- | :--- |
| **World Coordinate Frame & Units** | Right-handed metric $(x, y, z)$ in metres, yaw in radians $(-\pi, \pi]$, monotonic `sim_time_s` (`dt = 0.004 s`, 25 substeps = `0.10 s` per tick). | OpenUSD stage with `UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)` and `metersPerUnit = 1.0`; physics step `0.10 s` per `SimulationPort.step()`. | All `Pose2D(x_m, y_m, yaw_rad)` and timestamps must be finite and strictly increasing (`frame.time_s > last_time`). |
| **6-Patch Rice Terrace (`DEFAULT_SITES`)** | `patch_a` $(0.85, -0.50)$ & `patch_d` $(1.60, 0.45)$ (*Lannitian* mud hazards); `patch_b` $(1.60, -0.50)$ & `patch_c` $(2.35, -0.50)$ (viable stand gaps); `crop_nw` $(0.85, 0.45)$ & `crop_ne` $(2.35, 0.45)$ (`permitted=False` rice stands). | Import `DEFAULT_SITES` from `src/ricebot/world/` and spawn matching USD Xform beds, perimeter bund colliders, and seedling meshes. | Protected sites (`permitted=False`) must never be sampled or crossed by `LocalController`. |
| **Dual 4-Wheel Rovers (`planter`, `scout`)** | 6-DOF `freejoint` chassis + 4 revolute wheels (`_fl`, `_fr`, `_rl`, `_rr`) driven by `compute_4wheel_skid_steer(spec, lin, ang)`. | Two USD `Articulation` prims driven by `ArticulationAction(joint_velocities=compute_4wheel_skid_steer(spec, lin, ang))`. | Expired commands (`not (issued_at_s <= now_s < expires_at_s)`) are rejected at the simulator bridge (zero wheel velocity). |
| **3-Ray Forward LiDAR Fan (`front_range_m`)** | 3 `mjSENS_RANGEFINDER` rays (`_range_c`, `_range_l` at $+18^\circ$, `_range_r` at $-18^\circ$), returning `min(c, l, r)` capped at `4.0 m`. | 3 PhysX raycasts (`raycast_closest`) at $0^\circ, +18^\circ, -18^\circ$ in the rover nose frame, returning `min(c, l, r)`. | Returns `float` in metres (`>= 0.0`) or `None` if offline; `< 0.35 m` triggers `LocalController` safe stop. |
| **Location-Gated Soil Probes (`SoilSample`)** | Calls `self.world.sample_local_soil(robot_id, pose, sim_t)` (`PaddyTerraceWorld` in `src/ricebot/world/terrace.py`). | Calls the **exact same** `self.world.sample_local_soil(robot_id, pose, sim_t)` instance. | Hidden field truth is **never** exposed outside `PaddyTerraceWorld`/evaluator; `sample.sampled_at_s == frame.time_s`. |
| **Physical Seed Release & Contact (`PlantFeedback`)** | Releases 6-DOF sphere `seed_proxy_X` from hopper chute ($z = 0.032\text{ m}$) on valid stationary `release_seed=True` and checks `mjData.contact` against `field_ground`. | Releases USD rigid-body sphere (`UsdPhysics.RigidBodyAPI` + `CollisionAPI`) from hopper chute and reads PhysX contact report with `field_ground`. | Inventory decrements **only** on physical release; `PlantFeedback(released=True, landing=Pose2D(...))` emitted **only** upon ground contact; `scout` can never release seeds. |

---

## 4. Simulation Design Backlog & Next Ideas (Pre-Code Proposals for Team Review)

Below are structured simulation ideas designed to strengthen physical realism, multi-agent coordination value, and evaluation rigor while strictly respecting the [docs/COMPONENTS.md](COMPONENTS.md) module boundaries. **These are documented as design specifications before writing code so the world, robot, planner, and evaluation owners can align first.**

### Idea 1 — Spatially Continuous Paddy Terramechanics & Mud Drag Zones (`src/ricebot/world/`)
- **Problem:** Currently, *Lannitian* deep mud patches (`patch_a`, `patch_d`) are unsafe to plant in (high moisture, low firmness), but driving across them has the same wheel friction as firm drained loam.
- **Proposed Simulation Enhancement:**
  - Add a position-dependent traction/viscous-drag modifier inside the simulator step when a rover's wheels overlap a high-moisture mud pan ($M(x, y) > 0.80, F(x, y) < 0.25$): reduce effective wheel-ground friction and apply a proportional velocity damping drag.
  - **Why it helps the product:** Makes `scout`'s lighter chassis (`2.4 kg`) safer for probing suspect mud pans while giving the heavier `planter` (`3.2 kg` + seed hopper) a physical incentive to avoid routing through soft mud pans once `scout` maps them on the shared `MeasuredFieldMap`.

### Idea 2 — Seed-Controlled Probe Noise & Active Multi-Sample Fusion (`src/ricebot/world/` + `src/ricebot/agent/`)
- **Problem:** Deterministic noise-free soil probes mean a single sample immediately yields zero physical variance, under-utilizing `MeasuredFieldMap`'s sample-count uncertainty reduction (`uncertainty = 1 / sqrt(count)`).
- **Proposed Simulation Enhancement:**
  - Allow optional deterministic, `seed`-controlled Gaussian probe noise $\epsilon \sim \mathcal{N}(0, \sigma_{\text{sensor}}^2)$ in `PaddyTerraceWorld.sample_local_soil` for borderline patches (e.g., firmness near the `0.35` threshold or moisture near `0.80`).
  - Allow periodic re-sampling after a minimum cooldown $\Delta t_{\text{resample}}$ (with distinct timestamped `sample_id = f"{robot_id}:{site_id}:{kind.value}:{tick}"`) so a borderline patch can be confirmed by a second reading before committing scarce seed inventory.

### Idea 3 — Hopper Drop Dispersion & Motion-Induced Landing Error (`src/ricebot/robot/`)
- **Problem:** When `planter` stops at the outer edge of `probe_radius_m` (`0.22 m` from site center) and drops a seed vertically, the seed lands ~`0.21 m` from the exact patch center.
- **Proposed Simulation Enhancement:**
  - Model the physical offset of the rear/center seed-drop nozzle in the rover body frame $(x_{\text{nozzle}}, y_{\text{nozzle}})$ plus small soil-impact restitution on uneven loam so `PlantFeedback.landing` reflects nozzle alignment relative to `site.center`.
  - **Why it helps evaluation:** Enables the independent evaluator (`src/ricebot/evaluation/`) to score both **patch eligibility** (did the seed land on a viable stand gap rather than a mud hazard?) and **placement precision** ($\|\text{landing} - \text{site.center}\|_2 \le r_{\text{target}}$).

### Idea 4 — Four Standardized Benchmark Scenarios for the ≤60s Demo & Grader (`src/ricebot/world/` + `src/ricebot/evaluation/`)
To make the final evaluation and video demo falsifiable across both MuJoCo and Isaac Sim, standardize 4 named scenarios selectable via `--scenario`:
1. **`nominal` (Cooperative Scout + Planter Division of Labor):**
   - `scout` sweeps nearby unknown patches (`patch_a`, `patch_d`) and flags *Lannitian* mud hazards into `MeasuredFieldMap`; `planter` skips rejected mud pans and plants its 2 seeds into `patch_b` and `patch_c`.
2. **`obstacle` (Unannounced Mid-Episode Bund Collapse / Crate Obstruction):**
   - At $t = 1.5\text{ s}$, an unannounced obstacle enters the active transit lane. Forward 3-ray LiDAR detects `front_range_m < 0.35 m`, triggering immediate safe `HOLD` or waypoint bypass around `no_go_radius_m`.
3. **`shifted_moisture` (Hidden Spatial Regime Shift Across Seeds):**
   - Across randomized episode seeds (`seed=1..20`), which of `patch_a..patch_d` are waterlogged mud hazards vs. viable stand gaps permutes deterministically from `seed`. A blind/static policy plants into mud ~50% of the time, whereas the shared-map scout+planter policy achieves 100% viable seed placement.
4. **`sensor_degraded` (Rangefinder Dropout / Stale Command Fault Injection):**
   - Injects a 1.0-second rangefinder dropout (`front_range_m = None`) or expired command window to prove `LocalController` enforces fail-safe zero-velocity hold with zero collisions and zero ghost seed drops.

### Idea 5 — Matched-Seed Baseline Comparison Table (`src/ricebot/evaluation/`)
When implementing Issue #13 (`src/ricebot/evaluation/`), evaluate every scenario seed across three matched policies using the exact same `MujocoPaddySimulation` / `IsaacPaddySimulation` physics:
- **Baseline 0 — Uniform / Blind Grid Planter:** Visits permitted sites in fixed index order without soil probing; wastes seeds on waterlogged *Lannitian* mud pans (`patch_a`, `patch_d`).
- **Baseline 1 — Single-Rover Planter Alone (`scout` disabled):** `planter` must physically drive to every patch itself to sample before planting; succeeds eventually but incurs higher travel distance and episode completion time.
- **Proposed Method — Cooperative Dual-Rover Shared Belief Map (`scout` + `planter`):** `scout` screens candidate patches in parallel while `planter` services verified stand gaps, maximizing **Viable Seed Establishment Score** per unit time with zero mud-hazard drops and zero protected-crop intrusions.
