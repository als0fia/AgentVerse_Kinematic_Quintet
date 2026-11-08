> **Submission release note (8 Nov 2026):** The current desktop application has a tested native 3D/2D engineering interface and a judge-facing **Presentation Mode**. Launch Presentation Mode with **python scripts/run_mujoco_paddypatch.py --presentation --scenario golden_rescue --policy cooperative --seed 7**. The screenshots, provisional architecture proposals and next-step statements further down this file record earlier development phases and do **not** supersede the [current launch guide](../../../docs/RUN.md), [release test evidence](../../../docs/TESTING.md) or [submission instructions](../../../docs/SUBMISSION.md). The acting controller, MuJoCo contact outcomes and viewer remain distinct. The GUI is not proof of real field hardware or live LLM use.

# Read-Only Swarm Belief & Terramechanics UI (`src/ricebot/viewer/`)

## 1. Provisional Status & Why This UI Exists (Replacing the Over-Engineered Legacy Visualizer)

- **Provisional Status:** This module (`src/ricebot/viewer/`) — comprising the native side-by-side 3D MuJoCo + 2D Top-Down desktop app ([`live_app.py`](live_app.py)) and the standalone HTML dashboard ([`dashboard.py`](dashboard.py)) — is our **current provisional UI representation** for inspecting the `rice.v1` MuJoCo physics simulation and Policy B (`DecentralizedMycelialRuntime`).
- **Why We Do Not Use the Legacy `riceswarm_ai/visualizer.py` System as Our Primary UI:**
  The legacy `src/riceswarm_ai/visualizer.py` Tkinter grid GUI models a 160-day macro-agricultural calendar ("Section A / Section B staggered batches", nursery sowing Days -25 to -15, vegetative tillering, grain maturity percentages, and 6 abstract grid-stepping dots `RS1`–`RS6`). That UI is **over-engineered around macro-season calendar management** and **displays a bunch of random farm-lifecycle metrics that we do not care about**, while failing to represent our core embodied robotics architecture:
  1. **No Per-Robot Decentralised Belief Representation:** It assumes a single global grid coordinator rather than showing separate per-robot `LocalMycelialMap` + `LocalSlamGrid` instances with no central database.
  2. **No Continuous 2D Spatial Terrain $f(x,y)$:** It uses coarse discrete grid cells rather than continuous 2D spatial surfaces for elevation/hilliness $z(x,y)$, moisture $M(x,y)$, cone firmness $F(x,y)$, mud depth $h_{\text{mud}}(x,y)$, plant density $\rho(x,y)$, and epistemic uncertainty $\sigma(x,y)$.
  3. **No Physical Wheel–Mud Terramechanics or 3D Physics:** It has no Bekker–Wong sinkage $z$, belly clearance margin $c_b - z$, wheel slip ratio $s$, or 3D MuJoCo rigid-body/actuation state.
  4. **Not a `ReadOnlyView` Subscriber:** It mutates its own standalone grid state rather than subscribing passively to `SimulationPort` (`ObservationFrame` / `Decision`).

---

## 2. Core UI Requirements: What We Need a UI For & What It Is Meant to Show

Per [`docs/COMPONENTS.md`](../../../docs/COMPONENTS.md) (Role 5) and [`docs/DEMO.md`](../../../docs/DEMO.md), our UI exists strictly as a **read-only engineering & judge inspection window** into the physical simulation (`SimulationPort`) and decentralized swarm intelligence (`DecentralizedMycelialRuntime`). It must never drive physics or leak hidden ground truth into the planner.

### What We Need the UI For
1. **Proving Decentralised Local Intelligence (No Central Map/Database):** Visually demonstrating to judges that `planter` and `scout` hold **separate private belief maps** that diverge when out of wireless range or during RF blackouts, and reconcile via compact peer-to-peer UWB/Wi-Fi HaLow gossip packets when they meet.
2. **Inspecting Continuous 2D Spatial Beliefs vs. Hidden Ground Truth:** Allowing instant side-by-side comparison between what each robot *believes* about continuous field functions $f(x,y)$ (interpolated from its own local soil probes + gossip) and the evaluator-only ground truth.
3. **Monitoring Real-Time Physical Terramechanics & Critical Failure Modes:** Showing physical wheel–mud interaction (Bekker–Wong sinkage $z$, belly clearance, slip ratio $s$, and `S1_BELLIED_OUT` immobilisation risk) alongside interactive vehicle mass/wheel-sizing trade-offs that directly affect 3D rover traction.
4. **Auditing Embodied Task Execution & Safety:** Tracking physical 3D/2D rover poses $(x, y, \theta)$, forward 3-ray LiDAR obstacle detection, local 2D SLAM hazard cells (`obstacle` vs. `mud_slip`), consensus task bids (`SAMPLE`, `PLANT`, `HOLD`, rescue assist), and verified physical seed-drop contacts.

### What the UI Is Meant to Show (The 5 Core Representations)
| Requirement | What It Represents | How Our Current Provisional Solution Shows It |
| --- | --- | --- |
| **1. Synchronised 3D Physics + 2D Top-Down View** | Physical 3D MuJoCo world alongside internal AI state | **Left pane** renders live 3D MuJoCo physics (`mujoco.Renderer`) with 3D belief spheres, SLAM boxes, and UWB links; **Right pane** renders the interactive 2D top-down belief heatmap. |
| **2. Per-Robot World-Model Switcher** | Private onboard memory per rover vs. evaluator truth | Click directly on **`PLANTER`** or **`SCOUT`** on the 2D canvas (or toggle **`Planter Local Belief`**, **`Scout Local Belief`**, **`\|Planter − Scout\| Belief Gap`**, **`Hidden Ground Truth`**) to inspect that specific robot's world model. |
| **3. Continuous 2D Field Heatmaps $f(x,y)$** | Smooth spatial RBF belief surfaces & terrain | 7 selectable layers over the 2D terrace canvas: Soil Moisture $M(x,y)$, Cone Firmness $F(x,y)$, Plant Density $\rho(x,y)$, Stand Gap $G(x,y)$, Epistemic Uncertainty $\sigma(x,y)$, Mud/SLAM Risk $R(x,y)$, and Elevation/Hilliness $z(x,y)$. |
| **4. Live Robot Telemetry & UWB/HaLow Gossip Feed** | Onboard 200 Hz sensor suite & P2P packet trace | Per-robot inspector & cards showing pose, remaining seeds, active task + reason, slip ratio $s$, sinkage $z$ (mm), belly clearance (mm), and a scrolling log of `MycelialGossipPacket` transmissions (distance $d_{ij}$, byte size, dialogue summary). |
| **5. Bekker–Wong Terramechanics & Sizing Lab** | Physical wheel–mud sinkage & vehicle sizing | Real-time sliders for vehicle mass $m$, wheel radius $r$, and wheel width $b$ (plus a dedicated curve explorer tab) computing sinkage $z$, belly clearance, mud drag $R_c$, and `TRAVERSABLE` vs. `BELLIED OUT / S1 STUCK` status in live 3D physics. |

---

## 3. Current Provisional Solution Architecture (`src/ricebot/viewer/`)

1. **`LiveSwarmDashboardApp` ([`live_app.py`](live_app.py)):**
   - Native desktop application (`--dashboard` / `--gui` in [`scripts/run_mujoco_paddypatch.py`](../../../scripts/run_mujoco_paddypatch.py)) presenting:
     - **Tab 1:** Side-by-Side Live 3D MuJoCo Physics View + Interactive 2D Top-Down Robot Belief Heatmap & Inspector + Live Bekker–Wong Sizing Sliders + UWB Gossip Log.
     - **Tab 2:** Analytical Bekker–Wong Pressure-Sinkage & Janosi–Hanamoto Shear Traction Curve Explorer across vehicle masses ($1.5\text{–}16\text{ kg}$) and wheel geometries.
     - **Tab 3:** Optional embedded wrapper for the legacy 6-robot seasonal simulator for backwards reference.
2. **`SwarmBeliefDashboardRecorder` & `render_dashboard_html` ([`dashboard.py`](dashboard.py)):**
   - Implements the `ReadOnlyView` protocol (`consume(frame: ObservationFrame, decision: Decision) -> None`) from [`src/ricebot/runtime.py`](../runtime.py).
   - Exports a clean, UI-agnostic JSON schema (`antenna.dashboard.v1` via `.export_bundle()`) and compiles multi-scenario bundles into a zero-dependency standalone HTML/Canvas dashboard (`--export-dashboard`).
3. **3D MuJoCo HUD Overlays ([`scripts/run_mujoco_paddypatch.py`](../../../scripts/run_mujoco_paddypatch.py)):**
   - Renders split dual-sphere belief beacons above each site (lower sphere = Scout belief, upper sphere = Planter belief), 3D SLAM hazard boxes, LiDAR rays, and glowing cyan/gold UWB gossip pulses between rover masts.

---

## 4. Setup & Exact Run / Test Commands

```powershell
# 1. Launch the native Side-by-Side 3D MuJoCo + 2D Top-Down Belief Dashboard Desktop UI:
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --dashboard

# 2. Export the standalone interactive HTML Top-Down Dashboard across all 4 Mycelial scenarios:
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --export-dashboard artifacts\antenna_swarm_dashboard.html

# 3. Run the live 3D MuJoCo passive viewer with real-time 3D belief & gossip HUD overlays:
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --policy mycelial --scenario bad_soil --viewer-seconds 15

# 4. Run unit & integration verification for the viewer and Mycelial belief heatmaps:
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_mycelial_swarm.py" -v
```

## 5. Invariants & Extending
- **Strictly Read-Only Belief & Oracle Separation:** `ground_truth_layers` are used strictly for visual comparison and independent grading; neither `planter` nor `scout` ever reads hidden ground truth.
- **Next UI Steps:** Connect autonomous physical rigid-docking rescue state (`DOCKING`, `LATCHED`, `ANCHOR_PULL`, `EXTRACTED`) directly into the top-down canvas and telemetry cards as the closed-loop rescue skill is integrated.


## Judge-facing Presentation Mode (8 Nov 2026)

Presentation Mode is an alternative layout of the **same** live Tkinter/MuJoCo
application, not a separate simulator or replay. It does not mutate the
physics, policy, controller, planner, safety gates or evaluation.

From the repository root, in Windows PowerShell:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements-mujoco.txt
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --presentation --scenario golden_rescue --policy cooperative --rescue-policy react_orchestrator --seed 7
```

The normal `--dashboard` command still opens the complete Engineering UI.
Select **Engineering / Advanced** in the top strip to reveal all controls,
sliders, technical telemetry and belief selectors again, without resetting
the episode. Select **Presentation Mode** to hide them. Presentation mode
starts playing automatically; use the visible Play/Pause and Restart buttons
when recording one continuous screen capture. Camera choices **Field**,
**Rescue** and **Auto frame** only change the visual camera, never robot
movement. Auto framing follows observed rescue incidents when present.

The 3D pane renders the actual MuJoCo bodies, simulated terrain and
observation-linked hazard markers. The 2D pane displays fused *estimated*
traversability risk from the existing onboard maps, discovered SLAM hazards,
observed site outcomes, each rover's actual pose history and physical
helper-approach positions recorded only after assignment. Red marks an
observed stranded rover; amber marks an actually selected helper. Event
headlines derive from fault evidence, rescue assessment and verdict,
latched/extraction feedback, and the recovered rover's subsequent task.

**Line semantics:** Solid rover trails = **past observed motion**. Amber
solid trail = **past helper motion since rescue assignment**. Dashed lines =
**straight-line direction to the currently assigned destination**, *not*
a collision-free route or computed waypoint plan. No path is presented as
computed unless the runtime actually exposes one. Fused estimates and
simulated sensors are not ground-truth field measurements.

Run `python -m unittest discover -s tests -v` and
`python scripts/check_repo.py` for regression checks. Headless CI skips
the graphical Tk test; verify actual 3D/2D rendering, typography, viewport
size, and full 60-second screen recording on Windows before claiming
presentation readiness.
