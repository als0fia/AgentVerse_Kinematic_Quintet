"""Modular Read-Only Swarm Belief & Continuous Terramechanics Dashboard (`src/ricebot/viewer/`).

Implements the `ReadOnlyView` protocol (`consume(frame, decision)`) from `src/ricebot/runtime.py`
to record and export a modular, UI-agnostic telemetry & continuous heatmap bundle without ever
mutating physics or leaking hidden world truth into `MissionCore` / `DecentralizedMycelialRuntime`.

Features:
- Captures per-tick snapshots of:
  1. Top-down 2D robot poses, headings, active task targets, forward LiDAR range, and seed inventory.
  2. Each robot's private continuous 2D Gaussian-Process / RBF belief heatmaps (`moisture`,
     `firmness`, `plant_density`, `stand_gap`, `uncertainty`, `traversability_risk`).
  3. Continuous 2D ground-truth terrain layers (`elevation_m`, `mud_depth_m`, `moisture`,
     `firmness`, `plant_density`, `stand_gap`) strictly for post-hoc evaluation & visual comparison.
  4. Analytical Bekker-Wong / Janosi-Hanamoto wheel-mud sinkage & immobilization risk telemetry
     (`TerramechanicsSinkageState`) for each robot, plus a vehicle-sizing trade-off curve across
     different robot masses and wheel dimensions.
  5. Event-triggered UWB + Wi-Fi HaLow peer-to-peer gossip packets (`MycelialGossipPacket`).
- Generates a self-contained, interactive top-down HTML dashboard (`render_dashboard_html`) that
  can be opened in any browser or embedded in the IDE, while keeping the JSON data contract clean
  so any external UI teammate can consume `export_bundle()` directly.
"""
from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
from typing import Any

from ..agent import (
    DecentralizedMycelialRuntime,
    evaluate_continuous_belief_field,
    sample_continuous_belief_grid,
)
from ..contracts import Decision, ObservationFrame, Site
from ..robot import (
    PLANTER_SPEC,
    SCOUT_SPEC,
    RoverChassisSpec,
    evaluate_bekker_terramechanics,
)
from ..world import ContinuousPaddyTerrain, DEFAULT_SITES


class SwarmBeliefDashboardRecorder:
    """Modular `ReadOnlyView` subscriber that captures per-robot continuous belief heatmaps & terramechanics."""

    def __init__(
        self,
        runtime: Any,
        terrain: ContinuousPaddyTerrain | None = None,
        sites: tuple[Site, ...] = DEFAULT_SITES,
        grid_nx: int = 32,
        grid_ny: int = 22,
        snapshot_stride: int = 5,
    ) -> None:
        self.runtime = runtime
        self.terrain = terrain or ContinuousPaddyTerrain(sites=sites)
        self.sites = sites
        self.grid_nx = grid_nx
        self.grid_ny = grid_ny
        self.snapshot_stride = max(1, snapshot_stride)
        self._tick_count = 0
        self.frames: list[dict[str, Any]] = []
        self._specs: dict[str, RoverChassisSpec] = {
            "planter": PLANTER_SPEC,
            "scout": SCOUT_SPEC,
        }

    def consume(self, frame: ObservationFrame, decision: Decision) -> None:
        """Record a read-only telemetry & heatmap frame on key events or every `snapshot_stride` ticks."""
        planner_obj = getattr(self.runtime, "planner", self.runtime)
        raw_assessments = getattr(self.runtime, "last_rescue_assessments", None) or getattr(planner_obj, "last_rescue_assessments", None) or {}
        raw_evidence = getattr(self.runtime, "last_fault_evidence", None) or getattr(planner_obj, "last_fault_evidence", None) or {}
        raw_orch = getattr(self.runtime, "last_orchestrator_decisions", None) or getattr(planner_obj, "last_orchestrator_decisions", None) or {}
        gossip_pkts = getattr(self.runtime, "latest_gossip_packets", ()) or ()
        nodes_map = getattr(self.runtime, "nodes", {}) or {}
        tel_map = getattr(self.runtime, "latest_sensor_telemetry", {}) or {}

        is_key_event = bool(
            self._tick_count == 0
            or frame.plant_feedback
            or frame.recovery_feedback
            or decision.ingested_samples > 0
            or gossip_pkts
            or raw_assessments
            or any(c.release_seed or c.recovery_target_id for c in decision.commands)
            or (self._tick_count % self.snapshot_stride == 0)
        )
        self._tick_count += 1
        if not is_key_event:
            return

        intents_by_id = {i.robot_id: i for i in decision.intents}
        cmds_by_id = {c.robot_id: c for c in decision.commands}

        robots_snapshot: dict[str, Any] = {}
        for r_obs in frame.robots:
            rid = r_obs.robot_id
            node = nodes_map.get(rid)
            spec = self._specs.get(rid, PLANTER_SPEC)
            tel = tel_map.get(rid)
            ev = raw_evidence.get(rid)
            slip = tel.slip_ratio if tel is not None else (ev.slip_ratio if ev is not None else (0.90 if r_obs.is_bogged else 0.08))

            # Evaluate true local terrain beneath the rover for physical sinkage telemetry
            true_pt = self.terrain.evaluate(r_obs.pose.x_m, r_obs.pose.y_m)
            sinkage_state = evaluate_bekker_terramechanics(
                spec,
                moisture=true_pt.moisture,
                firmness=true_pt.firmness,
                mud_depth_m=true_pt.mud_depth_m,
                slope_magnitude=true_pt.slope_magnitude,
                slip_ratio=slip,
            )

            belief_grid = (
                sample_continuous_belief_grid(
                    node.local_map,
                    nx=self.grid_nx,
                    ny=self.grid_ny,
                )
                if node is not None
                else {}
            )
            local_point_belief = (
                evaluate_continuous_belief_field(node.local_map, r_obs.pose.x_m, r_obs.pose.y_m)
                if node is not None
                else {}
            )

            intent = intents_by_id.get(rid)
            cmd = cmds_by_id.get(rid)
            robots_snapshot[rid] = {
                "robot_id": rid,
                "pose": {
                    "x_m": round(r_obs.pose.x_m, 4),
                    "y_m": round(r_obs.pose.y_m, 4),
                    "yaw_rad": round(r_obs.pose.yaw_rad, 4),
                },
                "seeds_remaining": r_obs.seeds_remaining,
                "front_range_m": r_obs.front_range_m,
                "link_ok": r_obs.link_ok,
                "is_bogged": r_obs.is_bogged,
                "task_mode": intent.mode.value if intent else "hold",
                "target_site_id": intent.target_site_id if intent else None,
                "target_robot_id": intent.target_robot_id if intent else None,
                "recovery_technique": (cmd.recovery_technique if cmd and cmd.recovery_technique else (intent.recovery_technique if intent else "")),
                "task_reason": intent.reason if intent else "",
                "command": {
                    "linear_m_s": round(cmd.linear_m_s, 3) if cmd else 0.0,
                    "angular_rad_s": round(cmd.angular_rad_s, 3) if cmd else 0.0,
                    "release_seed": bool(cmd.release_seed) if cmd else False,
                    "recovery_target_id": cmd.recovery_target_id if cmd else None,
                    "dock_pitch_rad": round(cmd.dock_pitch_rad, 3) if cmd else 0.0,
                    "latch_rad": round(cmd.latch_rad, 3) if cmd else 0.0,
                    "spade_rad": round(cmd.spade_rad, 3) if cmd else 0.0,
                    "recovery_technique": cmd.recovery_technique if cmd else "",
                },
                "sensor_telemetry": asdict(tel) if tel is not None else {},
                "terramechanics": asdict(sinkage_state),
                "local_point_belief": local_point_belief,
                "known_samples_count": len(node.local_map.all_samples) if node else 0,
                "slam_cells": [
                    asdict(cell) for cell in node.local_map.slam.cells.values()
                ]
                if node
                else [],
                "planted_sites": sorted(node.local_map.planted_sites) if node else [],
                "belief_heatmaps": belief_grid,
            }

        gossip_events = [
            {
                "packet_id": pkt.packet_id,
                "sender_id": pkt.sender_id,
                "receiver_id": pkt.receiver_id,
                "sent_at_s": pkt.sent_at_s,
                "uwb_distance_m": pkt.uwb_distance_m,
                "byte_size": pkt.byte_size,
                "dialogue_summary": list(pkt.dialogue_summary),
            }
            for pkt in gossip_pkts
        ]

        serialized_assessments = {
            str(k): (v.to_dict() if hasattr(v, "to_dict") else asdict(v))
            for k, v in raw_assessments.items()
        }
        serialized_evidence = {
            str(k): asdict(v)
            for k, v in raw_evidence.items()
        }
        serialized_orch = {
            str(k): {
                "verdict": str(getattr(v.verdict, "value", v.verdict)),
                "selected_helper_id": getattr(v, "helper_robot_id", getattr(v, "selected_helper_id", None)),
                "selected_technique": str(
                    getattr(
                        getattr(v, "chosen_technique", getattr(v, "selected_technique", "")),
                        "value",
                        getattr(v, "chosen_technique", getattr(v, "selected_technique", "")),
                    )
                ),
                "safety_veto_applied": bool(getattr(v, "safety_veto_applied", False)),
                "fallback_used": bool(getattr(v, "fallback_used", False)),
                "rationale": str(getattr(v, "rationale", "")),
                "tool_trace": [
                    {
                        "step": getattr(s, "round_index", getattr(s, "step_index", 0)),
                        "tool": s.tool_name,
                        "observation": s.observation,
                    }
                    for s in getattr(v, "tool_trace", ())
                ],
            }
            for k, v in raw_orch.items()
        }

        self.frames.append(
            {
                "time_s": frame.time_s,
                "ingested_samples": decision.ingested_samples,
                "total_gossip_packets": getattr(self.runtime, "total_gossip_packets", 0),
                "total_gossip_bytes": getattr(self.runtime, "total_gossip_bytes", 0),
                "gossip_events": gossip_events,
                "plant_feedback": [asdict(fb) for fb in frame.plant_feedback],
                "recovery_feedback": [asdict(rf) for rf in frame.recovery_feedback],
                "rescue_assessments": serialized_assessments,
                "fault_evidence": serialized_evidence,
                "orchestrator_decisions": serialized_orch,
                "robots": robots_snapshot,
            }
        )

    def export_bundle(self, scenario_name: str = "nominal") -> dict[str, Any]:
        """Export the complete UI-agnostic JSON telemetry & continuous field bundle."""
        ground_truth_grid = self.terrain.sample_grid(nx=self.grid_nx, ny=self.grid_ny)
        sites_meta = [
            {
                "site_id": s.site_id,
                "x_m": s.center.x_m,
                "y_m": s.center.y_m,
                "permitted": s.permitted,
                "no_go_radius_m": s.no_go_radius_m,
            }
            for s in self.sites
        ]
        return {
            "schema_version": "antenna.dashboard.v1",
            "scenario": scenario_name,
            "grid": {
                "nx": self.grid_nx,
                "ny": self.grid_ny,
                "xs": ground_truth_grid["xs"],
                "ys": ground_truth_grid["ys"],
                "x_bounds": ground_truth_grid["x_bounds"],
                "y_bounds": ground_truth_grid["y_bounds"],
            },
            "sites": sites_meta,
            "ground_truth_layers": ground_truth_grid["layers"],
            "frames": self.frames,
        }


def render_dashboard_html(
    bundles_by_scenario: dict[str, dict[str, Any]],
    output_path: str | Path | None = None,
) -> str:
    """Render a self-contained, interactive top-down HTML dashboard from one or more scenario bundles."""
    payload_json = json.dumps(bundles_by_scenario)
    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>Antenna Swarm — Top-Down Belief Heatmap & Terramechanics Dashboard</title>
  <script src="https://www.gstatic.com/antigravity/web/dev/tailwindcss.min.js"></script>
  <style>
    body {{
      background-color: #0b1120;
      color: #e2e8f0;
      font-family: ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    }}
    .card {{
      background-color: #111827;
      border: 1px solid #1e293b;
      border-radius: 0.75rem;
    }}
    input[type="range"] {{
      accent-color: #38bdf8;
    }}
  </style>
</head>
<body class="p-4 md:p-6">
  <div class="max-w-7xl mx-auto space-y-4">
    <!-- Header & Global Controls -->
    <div class="card p-4 flex flex-col lg:flex-row lg:items-center lg:justify-between gap-4">
      <div>
        <div class="flex items-center gap-2">
          <span class="px-2.5 py-0.5 rounded-full text-xs font-semibold bg-sky-500/20 text-sky-300 border border-sky-500/30">Antenna Swarm · Policy B</span>
          <span class="px-2.5 py-0.5 rounded-full text-xs font-semibold bg-emerald-500/20 text-emerald-300 border border-emerald-500/30">Continuous 2D RBF Belief & Bekker Terramechanics</span>
        </div>
        <h1 class="text-xl font-bold mt-1 text-white">Top-Down Swarm Belief Heatmap & Wheel–Mud Sinkage Inspector</h1>
        <p class="text-xs text-slate-400">Select any robot to inspect its private internal belief map (no central database), switch continuous heatmap layers, or test vehicle mass/wheel sizing in mud.</p>
      </div>

      <div class="flex flex-wrap items-center gap-3">
        <div>
          <label class="block text-xs text-slate-400 mb-1">Scenario / Comms</label>
          <select id="scenarioSelect" class="bg-slate-800 border border-slate-700 text-slate-100 text-xs rounded-lg px-3 py-2"></select>
        </div>
        <div>
          <label class="block text-xs text-slate-400 mb-1">Perspective (Robot Belief vs Ground Truth)</label>
          <div class="inline-flex rounded-lg bg-slate-800 p-1 border border-slate-700" id="robotTabs">
            <button data-view="planter" class="view-btn px-3 py-1.5 rounded-md text-xs font-semibold bg-sky-600 text-white">Planter Belief</button>
            <button data-view="scout" class="view-btn px-3 py-1.5 rounded-md text-xs font-semibold text-slate-300 hover:text-white">Scout Belief</button>
            <button data-view="divergence" class="view-btn px-3 py-1.5 rounded-md text-xs font-semibold text-slate-300 hover:text-white">|Planter − Scout| Gap</button>
            <button data-view="truth" class="view-btn px-3 py-1.5 rounded-md text-xs font-semibold text-slate-300 hover:text-white">Hidden Ground Truth</button>
          </div>
        </div>
      </div>
    </div>

    <!-- Main Grid: Left Canvas + Right Telemetry & Terramechanics Lab -->
    <div class="grid grid-cols-1 lg:grid-cols-12 gap-4">
      <!-- Top-Down 2D Field Canvas (7 cols) -->
      <div class="lg:col-span-7 card p-4 flex flex-col justify-between space-y-3">
        <div class="flex flex-wrap items-center justify-between gap-2">
          <div>
            <label class="block text-xs text-slate-400 mb-1">Continuous Heatmap Layer</label>
            <div class="flex flex-wrap gap-1.5" id="layerButtons">
              <button data-layer="moisture" class="layer-btn px-2.5 py-1 rounded text-xs font-medium bg-sky-600 text-white">Soil Moisture M(x,y)</button>
              <button data-layer="firmness" class="layer-btn px-2.5 py-1 rounded text-xs font-medium bg-slate-800 text-slate-300">Cone Firmness F(x,y)</button>
              <button data-layer="plant_density" class="layer-btn px-2.5 py-1 rounded text-xs font-medium bg-slate-800 text-slate-300">Plant Density ρ(x,y)</button>
              <button data-layer="stand_gap" class="layer-btn px-2.5 py-1 rounded text-xs font-medium bg-slate-800 text-slate-300">Stand Gap G(x,y)</button>
              <button data-layer="uncertainty" class="layer-btn px-2.5 py-1 rounded text-xs font-medium bg-slate-800 text-slate-300">Epistemic Uncertainty σ(x,y)</button>
              <button data-layer="traversability_risk" class="layer-btn px-2.5 py-1 rounded text-xs font-medium bg-slate-800 text-slate-300">Mud / SLAM Risk R(x,y)</button>
              <button data-layer="elevation_m" class="layer-btn px-2.5 py-1 rounded text-xs font-medium bg-slate-800 text-slate-300">Elevation & Hilliness z(x,y)</button>
            </div>
          </div>
        </div>

        <div class="relative w-full bg-slate-950 rounded-lg border border-slate-800 overflow-hidden flex items-center justify-center">
          <canvas id="fieldCanvas" width="720" height="460" class="w-full h-auto cursor-crosshair"></canvas>
        </div>

        <!-- Timeline Scrubber -->
        <div class="bg-slate-900/90 p-3 rounded-lg border border-slate-800 space-y-2">
          <div class="flex items-center justify-between text-xs">
            <div class="flex items-center gap-2">
              <button id="playBtn" class="px-3 py-1 rounded bg-emerald-600 hover:bg-emerald-500 text-white font-semibold">Play</button>
              <span id="timeLabel" class="font-mono text-slate-200">t = 0.00 s</span>
            </div>
            <div id="hoverCoordLabel" class="font-mono text-slate-400">Hover map to inspect continuous f(x,y)</div>
          </div>
          <input id="timeSlider" type="range" min="0" max="10" value="0" class="w-full" />
        </div>
      </div>

      <!-- Right Column: Selected Robot Telemetry, Gossip Feed & Terramechanics Lab (5 cols) -->
      <div class="lg:col-span-5 space-y-4">
        <!-- Per-Robot Status Cards -->
        <div class="card p-4 space-y-3">
          <div class="flex items-center justify-between">
            <h2 class="text-sm font-bold text-white">Live Robot State & Onboard Sensor Suite (200 Hz)</h2>
            <span id="gossipBadge" class="text-xs font-mono px-2 py-0.5 rounded bg-cyan-500/20 text-cyan-300">0 pkts (0 B)</span>
          </div>
          <div id="robotCardsContainer" class="grid grid-cols-1 sm:grid-cols-2 gap-3"></div>
        </div>

        <!-- D Rescue Orchestrator & B Physical Recovery Status -->
        <div class="card p-4 space-y-2">
          <div class="flex items-center justify-between">
            <h2 class="text-sm font-bold text-white">D Rescue Orchestrator & B Rigid-Dock Recovery</h2>
            <span id="rescueVerdictBadge" class="text-xs font-mono px-2 py-0.5 rounded bg-emerald-500/20 text-emerald-300">NOMINAL FIELDWORK</span>
          </div>
          <div id="rescueOrchestratorPanel" class="text-xs font-mono bg-slate-950 p-2.5 rounded border border-slate-800 space-y-1.5"></div>
        </div>

        <!-- Interactive Bekker-Wong Vehicle Sizing & Mud Sinkage Calculator -->
        <div class="card p-4 space-y-3">
          <div class="flex items-center justify-between">
            <div>
              <h2 class="text-sm font-bold text-white">Bekker–Wong Wheel–Mud Sinkage & Sizing Lab</h2>
              <p class="text-xs text-slate-400">Test how robots of different masses & wheel sizes sink or belly-out in mud:</p>
            </div>
            <span id="sinkageVerdictBadge" class="px-2 py-0.5 rounded text-xs font-semibold bg-emerald-500/20 text-emerald-300">TRAVERSABLE</span>
          </div>

          <div class="grid grid-cols-2 gap-3 text-xs">
            <div>
              <div class="flex justify-between"><span class="text-slate-400">Vehicle Mass (m):</span><span id="massVal" class="font-mono text-white">4.2 kg</span></div>
              <input id="massSlider" type="range" min="1.5" max="18.0" step="0.5" value="4.2" class="w-full mt-1" />
            </div>
            <div>
              <div class="flex justify-between"><span class="text-slate-400">Wheel Radius (r):</span><span id="radiusVal" class="font-mono text-white">58 mm</span></div>
              <input id="radiusSlider" type="range" min="35" max="110" step="2" value="58" class="w-full mt-1" />
            </div>
            <div>
              <div class="flex justify-between"><span class="text-slate-400">Wheel Width (b):</span><span id="widthVal" class="font-mono text-white">40 mm</span></div>
              <input id="widthSlider" type="range" min="20" max="90" step="2" value="40" class="w-full mt-1" />
            </div>
            <div>
              <div class="flex justify-between"><span class="text-slate-400">Local Mud Moisture:</span><span id="mudMoistureVal" class="font-mono text-white">0.88</span></div>
              <input id="mudMoistureSlider" type="range" min="0.20" max="0.96" step="0.02" value="0.88" class="w-full mt-1" />
            </div>
          </div>

          <div id="bekkerReadout" class="grid grid-cols-3 gap-2 bg-slate-900 p-2.5 rounded-lg border border-slate-800 text-xs font-mono"></div>
        </div>

        <!-- Peer-to-Peer Mycelial Gossip Log -->
        <div class="card p-4 space-y-2">
          <h2 class="text-sm font-bold text-white">Mycelial Peer-to-Peer Gossip Log (UWB + Wi-Fi HaLow)</h2>
          <div id="gossipLog" class="h-36 overflow-y-auto space-y-1.5 text-xs font-mono bg-slate-950 p-2.5 rounded border border-slate-800"></div>
        </div>
      </div>
    </div>
  </div>

  <script>
    const BUNDLES = {payload_json};
    const scenarioNames = Object.keys(BUNDLES);
    let currentScenario = scenarioNames[0];
    let currentView = "planter"; // "planter" | "scout" | "divergence" | "truth"
    let currentLayer = "moisture";
    let currentFrameIdx = 0;
    let playTimer = null;

    const scenarioSelect = document.getElementById("scenarioSelect");
    scenarioNames.forEach(name => {{
      const opt = document.createElement("option");
      opt.value = name;
      opt.textContent = name;
      scenarioSelect.appendChild(opt);
    }});

    scenarioSelect.addEventListener("change", (e) => {{
      currentScenario = e.target.value;
      currentFrameIdx = 0;
      syncSliderBounds();
      renderAll();
    }});

    document.querySelectorAll(".view-btn").forEach(btn => {{
      btn.addEventListener("click", () => {{
        currentView = btn.dataset.view;
        document.querySelectorAll(".view-btn").forEach(b => {{
          b.className = "view-btn px-3 py-1.5 rounded-md text-xs font-semibold " +
            (b.dataset.view === currentView ? "bg-sky-600 text-white" : "text-slate-300 hover:text-white");
        }});
        renderAll();
      }});
    }});

    document.querySelectorAll(".layer-btn").forEach(btn => {{
      btn.addEventListener("click", () => {{
        currentLayer = btn.dataset.layer;
        document.querySelectorAll(".layer-btn").forEach(b => {{
          b.className = "layer-btn px-2.5 py-1 rounded text-xs font-medium " +
            (b.dataset.layer === currentLayer ? "bg-sky-600 text-white" : "bg-slate-800 text-slate-300");
        }});
        renderAll();
      }});
    }});

    const timeSlider = document.getElementById("timeSlider");
    timeSlider.addEventListener("input", (e) => {{
      currentFrameIdx = parseInt(e.target.value, 10);
      renderAll();
    }});

    const playBtn = document.getElementById("playBtn");
    playBtn.addEventListener("click", () => {{
      if (playTimer) {{
        clearInterval(playTimer);
        playTimer = null;
        playBtn.textContent = "Play";
      }} else {{
        playBtn.textContent = "Pause";
        playTimer = setInterval(() => {{
          const frames = BUNDLES[currentScenario].frames;
          currentFrameIdx = (currentFrameIdx + 1) % frames.length;
          timeSlider.value = currentFrameIdx;
          renderAll();
        }}, 220);
      }}
    }});

    function syncSliderBounds() {{
      const frames = BUNDLES[currentScenario].frames;
      timeSlider.max = Math.max(0, frames.length - 1);
      timeSlider.value = currentFrameIdx;
    }}

    function colorMap(val, layer) {{
      let v = Math.max(0, Math.min(1, val));
      if (layer === "elevation_m") {{
        v = Math.max(0, Math.min(1, (val + 0.04) / 0.07));
      }}
      if (layer === "moisture" || layer === "mud_depth_m") {{
        // Dry tan -> cyan -> deep waterlogged blue/purple
        const r = Math.round(190 * (1 - v) + 30 * v);
        const g = Math.round(150 * (1 - v) + 110 * (1 - Math.abs(v - 0.5)) + 30 * v);
        const b = Math.round(80 * (1 - v) + 235 * v);
        return `rgb(${{r}},${{g}},${{b}})`;
      }}
      if (layer === "firmness" || layer === "stand_gap") {{
        // Red (soft/low) -> Amber -> Emerald (firm/viable)
        const r = Math.round(220 * (1 - v) + 25 * v);
        const g = Math.round(50 * (1 - v) + 205 * v);
        const b = Math.round(60 * (1 - v) + 110 * v);
        return `rgb(${{r}},${{g}},${{b}})`;
      }}
      if (layer === "plant_density") {{
        // Soil brown -> lush paddy green
        const r = Math.round(110 * (1 - v) + 22 * v);
        const g = Math.round(85 * (1 - v) + 185 * v);
        const b = Math.round(60 * (1 - v) + 65 * v);
        return `rgb(${{r}},${{g}},${{b}})`;
      }}
      // Uncertainty or Traversability Risk: Dark slate -> Amber -> Bright Crimson
      const r = Math.round(30 * (1 - v) + 245 * v);
      const g = Math.round(55 * (1 - v) + 145 * Math.sin(v * Math.PI));
      const b = Math.round(90 * (1 - v) + 40 * v);
      return `rgb(${{r}},${{g}},${{b}})`;
    }}

    function getActiveGrid(bundle, frame) {{
      const gt = bundle.ground_truth_layers;
      if (currentView === "truth" || currentLayer === "elevation_m") {{
        return gt[currentLayer] || gt["moisture"];
      }}
      if (currentView === "divergence") {{
        const gP = frame.robots.planter.belief_heatmaps[currentLayer];
        const gS = frame.robots.scout.belief_heatmaps[currentLayer];
        if (!gP || !gS) return gt["moisture"];
        return gP.map((row, rIdx) => row.map((v, cIdx) => Math.min(1.0, Math.abs(v - gS[rIdx][cIdx]) * 2.2)));
      }}
      const rData = frame.robots[currentView] || frame.robots.planter;
      return (rData.belief_heatmaps && rData.belief_heatmaps[currentLayer]) || gt["moisture"];
    }}

    const canvas = document.getElementById("fieldCanvas");
    const ctx = canvas.getContext("2d");

    function worldToCanvas(x_m, y_m, grid) {{
      const [xMin, xMax] = grid.x_bounds;
      const [yMin, yMax] = grid.y_bounds;
      const pad = 36;
      const w = canvas.width - 2 * pad;
      const h = canvas.height - 2 * pad;
      const cx = pad + ((x_m - xMin) / (xMax - xMin)) * w;
      const cy = canvas.height - pad - ((y_m - yMin) / (yMax - yMin)) * h;
      return [cx, cy];
    }}

    function canvasToWorld(cx, cy, grid) {{
      const [xMin, xMax] = grid.x_bounds;
      const [yMin, yMax] = grid.y_bounds;
      const pad = 36;
      const w = canvas.width - 2 * pad;
      const h = canvas.height - 2 * pad;
      const x_m = xMin + ((cx - pad) / w) * (xMax - xMin);
      const y_m = yMin + ((canvas.height - pad - cy) / h) * (yMax - yMin);
      return [x_m, y_m];
    }}

    canvas.addEventListener("mousemove", (evt) => {{
      const rect = canvas.getBoundingClientRect();
      const cx = (evt.clientX - rect.left) * (canvas.width / rect.width);
      const cy = (evt.clientY - rect.top) * (canvas.height / rect.height);
      const bundle = BUNDLES[currentScenario];
      const frame = bundle.frames[currentFrameIdx];
      const [x_m, y_m] = canvasToWorld(cx, cy, bundle.grid);
      const gridData = getActiveGrid(bundle, frame);
      const nx = bundle.grid.nx;
      const ny = bundle.grid.ny;
      const [xMin, xMax] = bundle.grid.x_bounds;
      const [yMin, yMax] = bundle.grid.y_bounds;
      const ix = Math.max(0, Math.min(nx - 1, Math.round(((x_m - xMin) / (xMax - xMin)) * (nx - 1))));
      const iy = Math.max(0, Math.min(ny - 1, Math.round(((y_m - yMin) / (yMax - yMin)) * (ny - 1))));
      const val = gridData[iy][ix];
      document.getElementById("hoverCoordLabel").textContent =
        `Cursor (${{x_m.toFixed(2)}}m, ${{y_m.toFixed(2)}}m) -> ${{currentLayer}} = ${{val.toFixed(3)}}`;
    }});

    function drawField() {{
      const bundle = BUNDLES[currentScenario];
      const frame = bundle.frames[currentFrameIdx];
      const grid = bundle.grid;
      const layerData = getActiveGrid(bundle, frame);

      ctx.fillStyle = "#090d16";
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      const pad = 36;
      const plotW = canvas.width - 2 * pad;
      const plotH = canvas.height - 2 * pad;
      const cellW = plotW / grid.nx;
      const cellH = plotH / grid.ny;

      // 1. Draw continuous 2D heatmap cells
      for (let iy = 0; iy < grid.ny; iy++) {{
        for (let ix = 0; ix < grid.nx; ix++) {{
          const val = layerData[iy][ix];
          ctx.fillStyle = colorMap(val, currentLayer);
          const rx = pad + ix * cellW;
          const ry = canvas.height - pad - (iy + 1) * cellH;
          ctx.fillRect(rx, ry, Math.ceil(cellW), Math.ceil(cellH));
        }}
      }}

      // 2. Draw Subtle Crop-Row Guide Lines (300mm spacing)
      ctx.strokeStyle = "rgba(255,255,255,0.12)";
      ctx.lineWidth = 1;
      for (let yRow = -0.75; yRow <= 0.75; yRow += 0.30) {{
        const [x0, y0] = worldToCanvas(-0.20, yRow, grid);
        const [x1, y1] = worldToCanvas(2.90, yRow, grid);
        ctx.beginPath();
        ctx.moveTo(x0, y0);
        ctx.lineTo(x1, y1);
        ctx.stroke();
      }}

      // 3. Draw Sites (Patches & Protected Rice Stands)
      bundle.sites.forEach(site => {{
        const [sx, sy] = worldToCanvas(site.x_m, site.y_m, grid);
        const rPx = (site.no_go_radius_m / (grid.x_bounds[1] - grid.x_bounds[0])) * plotW;

        ctx.beginPath();
        ctx.arc(sx, sy, rPx, 0, Math.PI * 2);
        if (!site.permitted) {{
          ctx.fillStyle = "rgba(22, 163, 74, 0.35)";
          ctx.strokeStyle = "#4ade80";
          ctx.lineWidth = 2;
        }} else {{
          const plantedByAny = Object.values(frame.robots).some(r => (r.planted_sites || []).includes(site.site_id));
          ctx.fillStyle = plantedByAny ? "rgba(34, 197, 94, 0.38)" : "rgba(15, 23, 42, 0.28)";
          ctx.strokeStyle = plantedByAny ? "#22c55e" : "rgba(255,255,255,0.65)";
          ctx.lineWidth = plantedByAny ? 3 : 1.5;
        }}
        ctx.fill();
        ctx.stroke();

        ctx.fillStyle = "#ffffff";
        ctx.font = "bold 11px monospace";
        ctx.textAlign = "center";
        ctx.fillText(site.site_id, sx, sy - rPx - 5);
      }});

      // 4. Draw SLAM Hazard Cells discovered by the currently selected robot (or both)
      const slamRobots = (currentView === "planter" || currentView === "scout")
        ? [frame.robots[currentView]]
        : Object.values(frame.robots);
      slamRobots.forEach(r => {{
        (r.slam_cells || []).forEach(cell => {{
          const [cx, cy] = worldToCanvas(cell.x_m, cell.y_m, grid);
          ctx.fillStyle = cell.kind === "obstacle" ? "rgba(239, 68, 68, 0.85)" : "rgba(168, 85, 247, 0.75)";
          ctx.strokeStyle = "#fff";
          ctx.lineWidth = 1.5;
          ctx.fillRect(cx - 8, cy - 8, 16, 16);
          ctx.strokeRect(cx - 8, cy - 8, 16, 16);
        }});
      }});

      // 5. Draw UWB / Wi-Fi HaLow Peer-to-Peer Gossip Link
      const rPlanter = frame.robots.planter;
      const rScout = frame.robots.scout;
      if (rPlanter && rScout) {{
        const [px, py] = worldToCanvas(rPlanter.pose.x_m, rPlanter.pose.y_m, grid);
        const [sx, sy] = worldToCanvas(rScout.pose.x_m, rScout.pose.y_m, grid);
        const hasGossipNow = (frame.gossip_events || []).length > 0;
        ctx.beginPath();
        ctx.moveTo(px, py);
        ctx.lineTo(sx, sy);
        ctx.strokeStyle = hasGossipNow ? "#22d3ee" : "rgba(56, 189, 248, 0.35)";
        ctx.lineWidth = hasGossipNow ? 3.5 : 1.5;
        if (!hasGossipNow) ctx.setLineDash([5, 5]);
        ctx.stroke();
        ctx.setLineDash([]);
      }}

      // 6. Draw Each Robot Chassis, Wheel Lugs, Heading Vector & Task Target Line
      Object.values(frame.robots).forEach(r => {{
        const [rx, ry] = worldToCanvas(r.pose.x_m, r.pose.y_m, grid);
        const isSelected = (r.robot_id === currentView);

        // Task intent line
        if (r.target_site_id) {{
          const tSite = bundle.sites.find(s => s.site_id === r.target_site_id);
          if (tSite) {{
            const [tx, ty] = worldToCanvas(tSite.x_m, tSite.y_m, grid);
            ctx.beginPath();
            ctx.moveTo(rx, ry);
            ctx.lineTo(tx, ty);
            ctx.strokeStyle = r.robot_id === "planter" ? "rgba(250, 204, 21, 0.85)" : "rgba(74, 222, 128, 0.85)";
            ctx.lineWidth = 2;
            ctx.stroke();
          }}
        }}

        // Robot body oriented by yaw_rad
        ctx.save();
        ctx.translate(rx, ry);
        ctx.rotate(-r.pose.yaw_rad);

        if (isSelected) {{
          ctx.beginPath();
          ctx.arc(0, 0, 22, 0, Math.PI * 2);
          ctx.strokeStyle = "#38bdf8";
          ctx.lineWidth = 2.5;
          ctx.stroke();
        }}

        ctx.fillStyle = r.robot_id === "planter" ? "#2563eb" : "#16a34a";
        ctx.strokeStyle = "#f8fafc";
        ctx.lineWidth = 1.5;
        ctx.fillRect(-14, -10, 28, 20);
        ctx.strokeRect(-14, -10, 28, 20);

        // Front docking cone indicator
        ctx.fillStyle = "#f97316";
        ctx.beginPath();
        ctx.moveTo(14, -5);
        ctx.lineTo(21, 0);
        ctx.lineTo(14, 5);
        ctx.closePath();
        ctx.fill();
        ctx.restore();

        ctx.fillStyle = "#ffffff";
        ctx.font = "bold 11px sans-serif";
        ctx.textAlign = "center";
        ctx.fillText(r.robot_id.toUpperCase() + ` (seeds:${{r.seeds_remaining}})`, rx, ry + 26);
      }});
    }}

    function renderRobotCards() {{
      const bundle = BUNDLES[currentScenario];
      const frame = bundle.frames[currentFrameIdx];
      document.getElementById("timeLabel").textContent = `t = ${{frame.time_s.toFixed(2)}} s (Frame ${{currentFrameIdx + 1}}/${{bundle.frames.length}})`;
      document.getElementById("gossipBadge").textContent = `${{frame.total_gossip_packets}} pkts (${{frame.total_gossip_bytes}} B)`;

      const container = document.getElementById("robotCardsContainer");
      container.innerHTML = "";
      Object.values(frame.robots).forEach(r => {{
        const tm = r.terramechanics || {{}};
        const tel = r.sensor_telemetry || {{}};
        const isActive = (currentView === r.robot_id);
        const borderCls = isActive ? "border-sky-500 ring-1 ring-sky-500/50" : "border-slate-800";
        const card = document.createElement("div");
        card.className = `p-3 rounded-lg bg-slate-900 border ${{borderCls}} space-y-1.5 text-xs cursor-pointer`;
        card.onclick = () => {{
          currentView = r.robot_id;
          document.querySelectorAll(".view-btn").forEach(b => {{
            b.className = "view-btn px-3 py-1.5 rounded-md text-xs font-semibold " +
              (b.dataset.view === currentView ? "bg-sky-600 text-white" : "text-slate-300 hover:text-white");
          }});
          renderAll();
        }};
        card.innerHTML = `
          <div class="flex items-center justify-between">
            <span class="font-bold text-white uppercase">${{r.robot_id}}</span>
            <span class="px-2 py-0.5 rounded text-xs font-mono bg-slate-800 text-sky-300">${{r.task_mode.toUpperCase()}} → ${{r.target_site_id || "NONE"}}</span>
          </div>
          <div class="text-slate-400 font-mono">Pose: (${{r.pose.x_m.toFixed(2)}}m, ${{r.pose.y_m.toFixed(2)}}m) | Seeds: ${{r.seeds_remaining}}</div>
          <div class="text-slate-400 font-mono">Slip s: ${{(tel.slip_ratio || 0).toFixed(2)}} | Sinkage z: ${{((tm.sinkage_z_m || 0)*1000).toFixed(1)}}mm</div>
          <div class="text-slate-400 font-mono">Belly Clear: ${{((tm.belly_clearance_m || 0)*1000).toFixed(1)}}mm | Risk: ${{((tm.immobilization_risk || 0)*100).toFixed(0)}}%</div>
          <div class="text-slate-300 truncate" title="${{r.task_reason}}">Reason: ${{r.task_reason || "—"}}</div>
        `;
        container.appendChild(card);
      }});

      // Render cumulative gossip log up to currentFrameIdx
      const logEl = document.getElementById("gossipLog");
      logEl.innerHTML = "";
      let count = 0;
      for (let i = currentFrameIdx; i >= 0; i--) {{
        const f = bundle.frames[i];
        (f.gossip_events || []).forEach(pkt => {{
          count++;
          const div = document.createElement("div");
          div.className = "p-1.5 rounded bg-slate-900 border border-slate-800 text-slate-300";
          div.innerHTML = `<span class="text-cyan-400 font-semibold">[t=${{pkt.sent_at_s.toFixed(2)}}s | d=${{pkt.uwb_distance_m.toFixed(2)}}m | ${{pkt.byte_size}}B]</span> ${{pkt.dialogue_summary.join(" ; ")}}`;
          logEl.appendChild(div);
        }});
      }}
      if (count === 0) {{
        logEl.innerHTML = `<div class="text-slate-500">No gossip packets transmitted yet at t=${{frame.time_s.toFixed(2)}}s.</div>`;
      }}

      // Render D Rescue Orchestrator & B Physical Recovery panel
      const rescuePanel = document.getElementById("rescueOrchestratorPanel");
      const rescueBadge = document.getElementById("rescueVerdictBadge");
      const assessments = frame.rescue_assessments || {{}};
      const orchMap = frame.orchestrator_decisions || {{}};
      const recEvents = frame.recovery_feedback || [];
      const assessmentKeys = Object.keys(assessments);
      if (recEvents.length > 0) {{
        const rf = recEvents[recEvents.length - 1];
        rescueBadge.className = "text-xs font-mono px-2 py-0.5 rounded bg-cyan-500/20 text-cyan-300 border border-cyan-500/40";
        rescueBadge.textContent = rf.extracted ? "EXTRACTED CLEAR" : rf.phase.toUpperCase();
        rescuePanel.innerHTML = `
          <div class="text-cyan-300 font-semibold">${{rf.helper_robot_id.toUpperCase()}} → ${{rf.target_robot_id.toUpperCase()}} (${{rf.phase}})</div>
          <div class="text-slate-300">Latched: ${{rf.latched}} | Extracted: ${{rf.extracted}} | Disp: ${{(rf.victim_displacement_m || 0).toFixed(3)}} m</div>
          <div class="text-slate-400">Technique: ${{rf.technique || "direct_pry_and_pull"}} | Peak Force: ${{(rf.peak_constraint_force_n || 0).toFixed(1)}} N | Helper Safe: ${{rf.helper_safe}}</div>
        `;
      }} else if (assessmentKeys.length > 0) {{
        const sid = assessmentKeys[0];
        const a = assessments[sid];
        const o = orchMap[sid] || {{}};
        const vStr = (a.verdict || "rescue_now").toUpperCase();
        rescueBadge.className = vStr === "RESCUE_NOW"
          ? "text-xs font-mono px-2 py-0.5 rounded bg-amber-500/20 text-amber-300 border border-amber-500/40"
          : "text-xs font-mono px-2 py-0.5 rounded bg-rose-500/20 text-rose-300 border border-rose-500/40";
        rescueBadge.textContent = `${{sid.toUpperCase()}}: ${{vStr}}`;
        const tools = (o.tool_trace || []).map(t => t.tool).join(" → ") || "deterministic_triage";
        rescuePanel.innerHTML = `
          <div class="text-amber-300 font-semibold">Victim: ${{sid.toUpperCase()}} | Helper: ${{a.helper_robot_id || "NONE (REFUSED)"}} | Net U: ${{(a.net_rescue_value || 0).toFixed(2)}}</div>
          <div class="text-slate-300">Technique: ${{a.mechanical_technique || o.selected_technique || "none"}} | Safety Gate Veto: ${{Boolean(a.safety_veto_applied)}}</div>
          <div class="text-sky-300 truncate">ReAct Tools: ${{tools}}</div>
          <div class="text-slate-400">${{a.reason || o.rationale || ""}}</div>
        `;
      }} else {{
        rescueBadge.className = "text-xs font-mono px-2 py-0.5 rounded bg-emerald-500/20 text-emerald-300";
        rescueBadge.textContent = "NOMINAL FIELDWORK";
        rescuePanel.innerHTML = `<div class="text-slate-500">All rovers nominal. Sensed FaultEvidence monitoring active (0 bogged rovers).</div>`;
      }}
    }}

    function updateBekkerLab() {{
      const mass = parseFloat(document.getElementById("massSlider").value);
      const r_mm = parseFloat(document.getElementById("radiusSlider").value);
      const b_mm = parseFloat(document.getElementById("widthSlider").value);
      const m = parseFloat(document.getElementById("mudMoistureSlider").value);

      document.getElementById("massVal").textContent = `${{mass.toFixed(1)}} kg`;
      document.getElementById("radiusVal").textContent = `${{r_mm.toFixed(0)}} mm`;
      document.getElementById("widthVal").textContent = `${{b_mm.toFixed(0)}} mm`;
      document.getElementById("mudMoistureVal").textContent = m.toFixed(2);

      const r_m = r_mm / 1000.0;
      const b_m = b_mm / 1000.0;
      const f = Math.max(0.10, 1.0 - 0.85 * m);
      const mudDepth = 0.04 + 0.30 * Math.pow(m, 1.6) * Math.pow(1.0 - f, 1.2);
      const clearance0 = Math.max(0.020, 0.044 + (r_m - 0.058));

      const kc = 1.5 + 24.0 * Math.pow(f, 1.4) * (1.0 - 0.55 * m);
      const kphi = 45.0 + 920.0 * Math.pow(f, 1.5) * (1.0 - 0.60 * m);
      const n = 0.52 + 0.58 * f;
      const k_eq = Math.max(28000.0, (kc / Math.max(0.015, b_m) + kphi) * 950.0);
      const wWheel = (mass * 9.81) / 4.0;
      const chord = Math.sqrt(2.0 * r_m * 0.035);
      const z0 = Math.pow(wWheel / (b_m * chord * k_eq), 1.0 / n);
      const z = Math.min(mudDepth, z0 + 0.008 * (1.0 - f));
      const bellyClear = clearance0 - z;
      const belliedOut = bellyClear <= 0.0;

      const rc = 4.0 * b_m * k_eq * Math.pow(z, n + 1.0) / (n + 1.0);
      const badge = document.getElementById("sinkageVerdictBadge");
      if (belliedOut) {{
        badge.className = "px-2 py-0.5 rounded text-xs font-semibold bg-rose-500/20 text-rose-300 border border-rose-500/40";
        badge.textContent = "S1 BELLY-OUT / STUCK (RESCUE REQ)";
      }} else if (bellyClear < 0.012) {{
        badge.className = "px-2 py-0.5 rounded text-xs font-semibold bg-amber-500/20 text-amber-300 border border-amber-500/40";
        badge.textContent = "HIGH SLIP / MARGINAL CLEARANCE";
      }} else {{
        badge.className = "px-2 py-0.5 rounded text-xs font-semibold bg-emerald-500/20 text-emerald-300 border border-emerald-500/40";
        badge.textContent = "TRAVERSABLE (SAFE CLEARANCE)";
      }}

      document.getElementById("bekkerReadout").innerHTML = `
        <div><span class="text-slate-400 block">Sinkage z:</span><span class="text-white font-bold">${{(z * 1000).toFixed(1)}} mm</span></div>
        <div><span class="text-slate-400 block">Belly Clear:</span><span class="${{belliedOut ? 'text-rose-400' : 'text-emerald-400'}} font-bold">${{(bellyClear * 1000).toFixed(1)}} mm</span></div>
        <div><span class="text-slate-400 block">Mud Drag Rc:</span><span class="text-white font-bold">${{rc.toFixed(1)}} N</span></div>
      `;
    }}

    ["massSlider", "radiusSlider", "widthSlider", "mudMoistureSlider"].forEach(id => {{
      document.getElementById(id).addEventListener("input", updateBekkerLab);
    }});

    function renderAll() {{
      drawField();
      renderRobotCards();
      updateBekkerLab();
    }}

    syncSliderBounds();
    renderAll();
  </script>
</body>
</html>
"""
    if output_path is not None:
        out = Path(output_path)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(html, encoding="utf-8")
    return html
