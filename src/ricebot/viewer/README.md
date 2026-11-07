# Read-Only Swarm Belief & Terramechanics Viewer (`src/ricebot/viewer/`)

## 1. Purpose, Beneficiary Outcome & Current Scope
- **Purpose:** Implements Role 5 (`ReadOnlyView` subscriber in [`docs/COMPONENTS.md`](../../../docs/COMPONENTS.md)) to record per-robot continuous 2D belief heatmaps, UWB/HaLow peer-to-peer gossip logs, and analytical Bekker–Wong wheel–mud sinkage telemetry during simulated episodes, and export them as a modular JSON bundle (`antenna.dashboard.v1`) or a standalone interactive HTML top-down dashboard.
- **What actually works:**
  - `SwarmBeliefDashboardRecorder.consume(frame, decision)` passively records each robot's private continuous 2D RBF belief fields (`moisture`, `firmness`, `plant_density`, `stand_gap`, `uncertainty`, `traversability_risk`), SLAM hazard cells, and Bekker–Wong sinkage states (`TerramechanicsSinkageState`).
  - `render_dashboard_html(bundles_by_scenario, output_path)` produces a self-contained HTML dashboard with a top-down 2D canvas, robot belief switcher (`Planter Belief`, `Scout Belief`, `|Planter - Scout| Gap`, `Hidden Ground Truth`), continuous heatmap layer selector, timeline scrubber, and interactive Bekker–Wong vehicle mass/wheel-sizing calculator.
- **Out of scope:** Mutating physics state, issuing motor commands, or exposing `ground_truth_layers` to any planner or agent node.

## 2. Human Owner, Agent & Boundaries
- **Role Ownership:** Role 5 — Intelligence evaluator & read-only viewer ([`docs/COMPONENTS.md`](../../../docs/COMPONENTS.md)).
- **Upstream/Downstream Contract:** Implements `ReadOnlyView` from `src/ricebot/runtime.py` and consumes `ObservationFrame`, `Decision` ([`docs/CONTRACTS.md`](../../../docs/CONTRACTS.md)), `DecentralizedMycelialRuntime` ([`src/ricebot/agent/README.md`](../agent/README.md)), `ContinuousPaddyTerrain` ([`src/ricebot/world/README.md`](../world/README.md)), and `evaluate_bekker_terramechanics` ([`src/ricebot/robot/README.md`](../robot/README.md)).

## 3. Public Interface
- `SwarmBeliefDashboardRecorder(runtime, terrain=None, sites=DEFAULT_SITES, grid_nx=32, grid_ny=22, snapshot_stride=5)`
  - `.consume(frame: ObservationFrame, decision: Decision) -> None`
  - `.export_bundle(scenario_name: str) -> dict[str, Any]`
- `render_dashboard_html(bundles_by_scenario: dict[str, dict[str, Any]], output_path: str | Path | None = None) -> str`

## 4. Setup & Exact Run / Test Commands
```powershell
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --export-dashboard artifacts\antenna_swarm_dashboard.html
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_mycelial_swarm.py" -v
```

## 5. Invariants & Failure Handling
- **Strictly Read-Only:** `consume()` never mutates `ObservationFrame`, `Decision`, `MujocoPaddySimulation`, or `DecentralizedMycelialRuntime`.
- **UI-Agnostic JSON Schema (`antenna.dashboard.v1`):** External UI teammates can consume `export_bundle()` directly without depending on the HTML renderer.

## 6. Extending / Replacing
- Edit `src/ricebot/viewer/dashboard.py` to add new dashboard panels or connect a live WebSocket/HTTP frontend to `export_bundle()`.
