#!/usr/bin/env python3
"""Run PaddyPatch rice.v1 3D Rice-Terrace & 4-Wheel Rover Simulator in MuJoCo.

Supports two interchangeable coordination architectures over the identical `SimulationPort`:
- **Policy B (`--policy mycelial`, default):** Decentralised Local Intelligence Network
  (`DecentralizedMycelialRuntime` in `src/ricebot/agent/mycelial_swarm.py`):
  - **No central map or database:** Each rover (`planter`, `scout`) maintains its own private
    2D SLAM Occupancy & Traversability Grid (`LocalSlamGrid`) and Gaussian Soil/Stand Belief
    Map (`LocalMycelialMap`).
  - **Mycelial Gossip over UWB + Wi-Fi HaLow:** Rovers measure inter-robot distance $d_{ij}$
    and bearing ($\\text{AoA}$) via Dual Qorvo DWM3000 UWB ranging and exchange compact,
    event-triggered `MycelialGossipPacket`s when within wireless range.
  - **Consensus Task Bidding:** Rovers compute capability-aware bids (`SAMPLE`, `PLANT`) and
    settle on a single task owner without a central supervisor.
  - **Local SLAM Obstacle Bypass:** Rovers map mid-episode obstacles into their local SLAM grid
    and steer smoothly around them.
- **Policy A (`--policy centralized` or `--scenario golden_rescue`):** Offline Centralised Coordinator
  (`MissionCore` in `src/ricebot/runtime.py` + `MeasuredFieldMap` + `CooperativeSwarmPlanner` / `SimpleMissionPlanner`).
  - **`--scenario golden_rescue` (Antenna Canonical Rescue-First Golden Episode):**
    Two physical MuJoCo rovers sample the terrace; `planter` becomes physically bogged in the
    waterlogged Lannitian mud basin at `patch_d` (`is_bogged=True` with valid LiDAR range);
    `CooperativeSwarmPlanner` evaluates `RescueVerdict.RESCUE_NOW` (`net=+0.66`) and dispatches
    `scout` with `TaskMode.RESCUE`; `scout` docks rigidly via its front cam-latch (`rigid_latch_scout_to_planter`),
    pitch-lifts `planter`'s chassis to break mud suction, extracts `planter` `>= 0.18 m` clear of
    the mud trap, unlatches, and `planter` resumes autonomous fieldwork to plant both viable gaps (`patch_b`, `patch_c`).
- **Evaluation Matrix (`--compare-policies`):** Benchmarks Single-Rover, Policy A (Centralised),
  and Policy B (Decentralised Mycelial) across Normal Comms, Intermittent Comms, Unexpected Bad
  Soil Sample, and Mid-Episode Obstacle Surprise.
- **Top-Down Belief & Terramechanics Dashboard (`--export-dashboard <path.html>`):** Exports an
  interactive top-down HTML dashboard showing each robot's private continuous 2D belief heatmaps
  and Bekker-Wong wheel-mud sinkage curves.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from math import cos, hypot, sin
from pathlib import Path
import sys
import time
from typing import Any, cast

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import mujoco
import mujoco.viewer
import numpy as np

from ricebot.agent import CooperativeSwarmPlanner, DecentralizedMycelialRuntime, SoilSafetyEnvelope
from ricebot.contracts import Decision, ObservationFrame, RobotCapability, SensorKind, TaskMode
from ricebot.control import ControlLimits, LocalController
from ricebot.mission import MeasuredFieldMap, SimpleMissionPlanner
from ricebot.mujoco_sim.adapter import DEFAULT_CAPABILITIES, DEFAULT_SITES, MujocoPaddySimulation
from ricebot.runtime import (
    EpisodeConfig,
    EpisodeRunner,
    EpisodeSession,
    MissionCore,
    create_mujoco_episode_session,
)
from ricebot.world import (
    DEFAULT_HIDDEN_SOIL_TRUTH,
    UNEXPECTED_BAD_SOIL_TRUTH,
    generate_hidden_soil_truth,
)


class TelemetryRecorder:
    """ReadOnlyView subscriber that records key events and travel distance without touching physics."""

    def __init__(self) -> None:
        self.events: list[dict[str, object]] = []
        self.last_poses: dict[str, tuple[float, float]] = {}
        self.travel_distance_m: dict[str, float] = {}

    def consume(self, frame: ObservationFrame, decision: Decision) -> None:
        for r in frame.robots:
            prev = self.last_poses.get(r.robot_id)
            if prev is not None:
                step_d = hypot(r.pose.x_m - prev[0], r.pose.y_m - prev[1])
                self.travel_distance_m[r.robot_id] = round(
                    self.travel_distance_m.get(r.robot_id, 0.0) + step_d, 4
                )
            self.last_poses[r.robot_id] = (r.pose.x_m, r.pose.y_m)

        has_rescue = any(i.mode == TaskMode.RESCUE for i in decision.intents) or bool(frame.recovery_feedback)
        if (
            frame.plant_feedback
            or frame.recovery_feedback
            or decision.ingested_samples > 0
            or any(c.release_seed for c in decision.commands)
            or has_rescue
        ):
            self.events.append(
                {
                    "time_s": frame.time_s,
                    "ingested_samples": decision.ingested_samples,
                    "robots": {
                        r.robot_id: {
                            "pose": (r.pose.x_m, r.pose.y_m, r.pose.yaw_rad),
                            "front_range_m": r.front_range_m,
                            "seeds_remaining": r.seeds_remaining,
                            "is_bogged": r.is_bogged,
                        }
                        for r in frame.robots
                    },
                    "intents": [
                        {
                            "robot_id": i.robot_id,
                            "mode": i.mode.value,
                            "target": i.target_site_id,
                            "target_robot_id": i.target_robot_id,
                            "reason": i.reason,
                        }
                        for i in decision.intents
                    ],
                    "commands": [
                        {
                            "robot_id": c.robot_id,
                            "linear_m_s": round(c.linear_m_s, 3),
                            "angular_rad_s": round(c.angular_rad_s, 3),
                            "release_seed": c.release_seed,
                            "target_site_id": c.target_site_id,
                            "recovery_target_id": c.recovery_target_id,
                            "dock_pitch_rad": c.dock_pitch_rad,
                            "latch_rad": c.latch_rad,
                        }
                        for c in decision.commands
                    ],
                    "plant_feedback": [asdict(fb) for fb in frame.plant_feedback],
                    "recovery_feedback": [asdict(rf) for rf in frame.recovery_feedback],
                }
            )


def _site_color_rgba(est: Any, planted: bool, envelope: SoilSafetyEnvelope) -> list[float]:
    if planted:
        return [0.15, 0.95, 0.30, 0.92]  # Bright Green: Physically Planted
    if not est.complete:
        return [0.95, 0.70, 0.15, 0.60]  # Amber: Unknown / Unexplored in this robot's local map
    if envelope.eligible(est):
        return [0.15, 0.85, 0.98, 0.88]  # Cyan: Verified Viable Stand Gap
    return [0.95, 0.20, 0.18, 0.88]      # Red: Rejected Waterlogged Lannitian Mud Hazard


def _draw_viewer_hud_overlays(
    viewer: Any,
    sim: MujocoPaddySimulation,
    policy_runtime: Any,
    frame: ObservationFrame,
    decision: Decision,
) -> None:
    """Render live 3D per-robot belief beacons, SLAM cells, UWB gossip links, and LiDAR rays."""
    scn = viewer.user_scn
    scn.ngeom = 0
    identity_mat = np.eye(3, dtype=np.float64).flatten()
    planted_sites = {d.target_site_id for d in sim.completed_drops if d.contact_observed_at_s is not None}
    envelope = SoilSafetyEnvelope()

    def add_sphere(pos: list[float], radius: float, rgba: list[float]) -> None:
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            g,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([radius, 0.0, 0.0], dtype=np.float64),
            np.array(pos, dtype=np.float64),
            identity_mat,
            np.array(rgba, dtype=np.float32),
        )
        scn.ngeom += 1

    def add_box(pos: list[float], half_sizes: list[float], rgba: list[float]) -> None:
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            g,
            mujoco.mjtGeom.mjGEOM_BOX,
            np.array(half_sizes, dtype=np.float64),
            np.array(pos, dtype=np.float64),
            identity_mat,
            np.array(rgba, dtype=np.float32),
        )
        scn.ngeom += 1

    def add_line(p1: list[float], p2: list[float], width: float, rgba: list[float]) -> None:
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            g,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            identity_mat,
            np.array(rgba, dtype=np.float32),
        )
        mujoco.mjv_connector(
            g,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            width,
            np.array(p1, dtype=np.float64),
            np.array(p2, dtype=np.float64),
        )
        scn.ngeom += 1

    obs_by_id = {r.robot_id: r for r in frame.robots}

    if isinstance(policy_runtime, DecentralizedMycelialRuntime):
        # 1. Split Dual-Rover Belief Beacons above each site:
        #    - Lower Sphere (z=0.17m) = Scout's private local map belief
        #    - Upper Sphere (z=0.26m) = Planter's private local map belief
        planter_node = policy_runtime.nodes.get("planter")
        scout_node = policy_runtime.nodes.get("scout")
        for site_id, site in sim.sites.items():
            if not site.permitted:
                continue
            cx, cy = site.center.x_m, site.center.y_m
            is_planted = site_id in planted_sites
            if scout_node is not None:
                s_est = scout_node.local_map.estimate(site_id)
                s_rgba = _site_color_rgba(s_est, is_planted, envelope)
                add_sphere([cx, cy, 0.17], 0.036, s_rgba)
            if planter_node is not None:
                p_est = planter_node.local_map.estimate(site_id)
                p_rgba = _site_color_rgba(p_est, is_planted, envelope)
                add_sphere([cx, cy, 0.26], 0.046, p_rgba)
                add_line([cx, cy, 0.01], [cx, cy, 0.22], 0.005, p_rgba)

        # 2. Render 3D Local SLAM Occupancy / Mud-Slip Hazard Cells discovered by the rovers
        seen_slam_coords: set[tuple[int, int]] = set()
        for node in policy_runtime.nodes.values():
            for key, cell in node.local_map.slam.cells.items():
                if key in seen_slam_coords:
                    continue
                seen_slam_coords.add(key)
                if cell.kind == "obstacle":
                    add_box([cell.x_m, cell.y_m, 0.04], [0.06, 0.06, 0.04], [1.0, 0.30, 0.10, 0.55])
                else:
                    add_box([cell.x_m, cell.y_m, 0.015], [0.07, 0.07, 0.012], [0.75, 0.15, 0.85, 0.45])

        # 3. Render UWB / Wi-Fi HaLow Peer-to-Peer Wireless Gossip Link between Rover Masts
        if "planter" in obs_by_id and "scout" in obs_by_id:
            p_pose = obs_by_id["planter"].pose
            s_pose = obs_by_id["scout"].pose
            # Active gossip packet transmitted recently (within 0.6s) -> bright glowing cyan/gold pulse
            recent_gossip = any(
                abs(frame.time_s - pkt.sent_at_s) <= 0.6 for pkt in policy_runtime.gossip_history
            )
            tel_p = policy_runtime.latest_sensor_telemetry.get("planter")
            dist_ps = tel_p.uwb_peer_ranges_m.get("scout", 99.0) if tel_p else 99.0
            if recent_gossip:
                add_line(
                    [p_pose.x_m, p_pose.y_m, 0.19],
                    [s_pose.x_m, s_pose.y_m, 0.16],
                    0.011,
                    [0.10, 0.98, 0.92, 0.92],
                )
                mid_x = 0.5 * (p_pose.x_m + s_pose.x_m)
                mid_y = 0.5 * (p_pose.y_m + s_pose.y_m)
                add_sphere([mid_x, mid_y, 0.18], 0.028, [0.98, 0.92, 0.15, 0.95])
            elif policy_runtime._is_link_open(frame.time_s, dist_ps):
                add_line(
                    [p_pose.x_m, p_pose.y_m, 0.19],
                    [s_pose.x_m, s_pose.y_m, 0.16],
                    0.003,
                    [0.25, 0.80, 0.95, 0.30],
                )
    else:
        # Centralised Policy A HUD
        field_map = policy_runtime.field_map
        for site_id, site in field_map.sites.items():
            if not site.permitted:
                continue
            est = field_map.estimate(site_id)
            cx, cy = site.center.x_m, site.center.y_m
            rgba = _site_color_rgba(est, site_id in planted_sites, envelope)
            add_sphere([cx, cy, 0.22], 0.048, rgba)
            add_line([cx, cy, 0.01], [cx, cy, 0.18], 0.006, rgba)

    # 4. Draw active task assignment vectors & forward rangefinder laser rays for each rover
    for intent in decision.intents:
        if intent.robot_id not in obs_by_id:
            continue
        r = obs_by_id[intent.robot_id]
        rx, ry, ryaw = r.pose.x_m, r.pose.y_m, r.pose.yaw_rad

        if r.is_bogged:
            # Red distress beacon above bogged rover
            add_sphere([rx, ry, 0.24], 0.042, [1.0, 0.12, 0.12, 0.92])

        beam_len = min(r.front_range_m if r.front_range_m is not None else 0.75, 0.75)
        bx = rx + beam_len * cos(ryaw)
        by = ry + beam_len * sin(ryaw)
        beam_rgba = (
            [1.0, 0.15, 0.15, 0.85]
            if (r.front_range_m is not None and r.front_range_m <= 0.40)
            else [0.25, 0.95, 0.55, 0.45]
        )
        add_line([rx, ry, 0.08], [bx, by, 0.08], 0.004, beam_rgba)

        if intent.mode == TaskMode.RESCUE and intent.target_robot_id in obs_by_id:
            peer_r = obs_by_id[intent.target_robot_id]
            add_line(
                [rx, ry, 0.14],
                [peer_r.pose.x_m, peer_r.pose.y_m, 0.14],
                0.009,
                [1.0, 0.45, 0.10, 0.92],
            )
        elif intent.target_site_id and intent.target_site_id in sim.sites:
            tsite = sim.sites[intent.target_site_id]
            line_rgba = (
                [0.98, 0.84, 0.15, 0.82]
                if intent.mode == TaskMode.PLANT
                else ([0.25, 0.68, 0.98, 0.65] if intent.robot_id == "planter" else [0.25, 0.92, 0.45, 0.65])
            )
            add_line([rx, ry, 0.11], [tsite.center.x_m, tsite.center.y_m, 0.18], 0.005, line_rgba)


def run_demo(
    scenario: str = "nominal",
    policy: str = "mycelial",
    comms: str = "normal",
    seed: int = 7,
    steps: int = 140,
    viewer_seconds: float = 0.0,
    single_rover_only: bool = False,
    rescue_policy: str = "react_orchestrator",
) -> dict[str, object]:
    cfg = EpisodeConfig(
        scenario=scenario,
        policy=policy,
        comms=comms,
        seed=seed,
        steps=steps,
        single_rover_only=single_rover_only,
        rescue_policy=rescue_policy,
    )
    steps = cfg.steps
    truth = generate_hidden_soil_truth(cfg.scenario, cfg.seed)
    recorder = TelemetryRecorder()
    session = create_mujoco_episode_session(cfg, views=(recorder,))
    sim = cast(MujocoPaddySimulation, session.sim)
    runtime: Any = session.policy

    if viewer_seconds > 0:
        session.reset(seed)
        start_wall = time.time()
        with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
            viewer.cam.lookat[:] = [1.25, -0.08, 0.06]
            viewer.cam.distance = 2.95
            viewer.cam.elevation = -34.0
            viewer.cam.azimuth = 112.0
            for _ in range(steps):
                if not viewer.is_running():
                    break
                tick = session.step_once()
                _draw_viewer_hud_overlays(viewer, sim, runtime, tick.frame, tick.decision)
                viewer.sync()
                time.sleep(0.045)
            while viewer.is_running() and (time.time() - start_wall) < viewer_seconds:
                if session.current_frame is not None and session.last_decision is not None:
                    _draw_viewer_hud_overlays(viewer, sim, runtime, session.current_frame, session.last_decision)
                viewer.sync()
                time.sleep(0.05)
    else:
        session.run(seed=seed, steps=steps)

    # Evaluate physical seed placements against hidden truth (Independent Grader check)
    envelope = SoilSafetyEnvelope()
    viable_seed_placements = 0
    for drop in sim.completed_drops:
        site_truth = truth.get(drop.target_site_id)
        if site_truth is not None and drop.contact_observed_at_s is not None:
            if (
                site_truth[SensorKind.STAND_GAP] >= envelope.min_gap
                and envelope.moisture_range[0] <= site_truth[SensorKind.MOISTURE] <= envelope.moisture_range[1]
                and envelope.firmness_range[0] <= site_truth[SensorKind.FIRMNESS] <= envelope.firmness_range[1]
            ):
                viable_seed_placements += 1

    if isinstance(runtime, DecentralizedMycelialRuntime):
        per_robot_maps = {
            rid: {
                site_id: asdict(node.local_map.estimate(site_id))
                for site_id in ("patch_a", "patch_b", "patch_c", "patch_d")
            }
            for rid, node in runtime.nodes.items()
        }
        gossip_log = [
            {
                "sent_at_s": p.sent_at_s,
                "sender": p.sender_id,
                "receiver": p.receiver_id,
                "uwb_distance_m": p.uwb_distance_m,
                "uwb_aoa_rad": p.uwb_aoa_rad,
                "byte_size": p.byte_size,
                "dialogue": list(p.dialogue_summary),
            }
            for p in runtime.gossip_history
        ]
        slam_cells_mapped = sum(len(n.local_map.slam.cells) for n in runtime.nodes.values())
        comm_bytes = runtime.total_gossip_bytes
        comm_packets = runtime.total_gossip_packets
        site_estimates = per_robot_maps["planter"]
    else:
        per_robot_maps = {}
        gossip_log = []
        slam_cells_mapped = 0
        # Centralised coordinator streams every robot frame + command continuously to central state
        comm_packets = steps * 2
        comm_bytes = steps * 2 * 128
        site_estimates = {
            site_id: asdict(runtime.field_map.estimate(site_id))
            for site_id in ("patch_a", "patch_b", "patch_c", "patch_d")
        }

    last_plant_time_s = (
        max((d.contact_observed_at_s for d in sim.completed_drops if d.contact_observed_at_s is not None), default=None)
    )
    trace = session.export_trace()

    return {
        "engine": f"MuJoCo {mujoco.__version__} (3D Rice Terrace + 4-Wheel Articulated Rovers SimulationPort)",
        "episode_id": trace.episode_id,
        "policy": cfg.effective_policy_label,
        "rescue_policy": rescue_policy,
        "comms_mode": comms,
        "scenario": scenario,
        "seed": seed,
        "steps": steps,
        "trace_ticks": len(trace.ticks),
        "obstacle_injected": sim.obstacle_injected,
        "viable_seed_placements": viable_seed_placements,
        "total_seeds_dropped": len(sim.completed_drops),
        "completed_recoveries": [asdict(r) for r in sim.completed_recoveries],
        "end_state_bogged_robots": sorted(sim._bogged_origins.keys()),
        "fault_evidence": {k: asdict(v) for k, v in session.fault_evidence.items()},
        "orchestrator_decisions": {k: v.to_dict() for k, v in session.orchestrator_decisions.items()},
        "rescue_assessments": {k: v.to_dict() for k, v in session.rescue_assessments.items()},
        "recovery_monitoring": [m if isinstance(m, dict) else asdict(m) for m in session.recovery_monitoring],
        "completion_time_s": last_plant_time_s,
        "travel_distance_m": recorder.travel_distance_m,
        "total_travel_m": round(sum(recorder.travel_distance_m.values()), 3),
        "comm_packets": comm_packets,
        "comm_bytes": comm_bytes,
        "slam_cells_mapped": slam_cells_mapped,
        "site_estimates": site_estimates,
        "per_robot_local_maps": per_robot_maps,
        "mycelial_gossip_log": gossip_log,
        "completed_seed_drops": [asdict(d) for d in sim.completed_drops],
        "key_events": recorder.events,
    }


def run_policy_comparison_matrix(seed: int = 7) -> dict[str, object]:
    """Execute the Policy A vs Policy B vs Cooperative Rescue evaluation matrix."""
    cases = [
        ("1_normal_comms_single_rover", "nominal", "centralized", "normal", True, 165, "react_orchestrator"),
        ("2_normal_comms_policy_A_centralized", "nominal", "centralized", "normal", False, 140, "react_orchestrator"),
        ("3_normal_comms_policy_B_mycelial", "nominal", "mycelial", "normal", False, 140, "react_orchestrator"),
        ("4_intermittent_comms_policy_B_mycelial", "nominal", "mycelial", "intermittent", False, 145, "react_orchestrator"),
        ("5_unexpected_bad_soil_policy_B_mycelial", "bad_soil", "mycelial", "normal", False, 155, "react_orchestrator"),
        ("6_obstacle_surprise_policy_A_centralized", "obstacle", "centralized", "normal", False, 155, "react_orchestrator"),
        ("7_obstacle_surprise_policy_B_mycelial_slam", "obstacle", "mycelial", "normal", False, 155, "react_orchestrator"),
        ("8_golden_rescue_episode_cooperative_swarm", "golden_rescue", "cooperative", "normal", False, 195, "react_orchestrator"),
        ("9_high_risk_refusal_episode_cooperative_swarm", "high_risk_abstain", "cooperative", "normal", False, 175, "react_orchestrator"),
    ]
    results: dict[str, object] = {}
    for label, scen, pol, comm, single, st, rpol in cases:
        res = run_demo(
            scenario=scen,
            policy=pol,
            comms=comm,
            seed=seed,
            steps=st,
            viewer_seconds=0.0,
            single_rover_only=single,
            rescue_policy=rpol,
        )
        results[label] = {
            "policy": res["policy"],
            "rescue_policy": res["rescue_policy"],
            "scenario": res["scenario"],
            "comms_mode": res["comms_mode"],
            "viable_seed_placements": res["viable_seed_placements"],
            "completed_recoveries": res["completed_recoveries"],
            "end_state_bogged_robots": res["end_state_bogged_robots"],
            "completion_time_s": res["completion_time_s"],
            "total_travel_m": res["total_travel_m"],
            "comm_packets": res["comm_packets"],
            "comm_bytes": res["comm_bytes"],
            "slam_cells_mapped": res["slam_cells_mapped"],
            "sample_gossip_messages": (res["mycelial_gossip_log"][:3] if res["mycelial_gossip_log"] else []),
        }
    return results


def run_baseline_comparison(seed: int = 7) -> dict[str, object]:
    """Compare Agent D's ReAct orchestrator against numerical_triage, always_rescue, and never_rescue baselines."""
    scenarios = [
        ("golden_rescue", 195, "planter"),
        ("high_risk_abstain", 175, "scout"),
    ]
    rescue_policies = ("react_orchestrator", "numerical_triage", "always_rescue", "never_rescue")
    comparison: dict[str, dict[str, object]] = {}
    for scen, st, primary_victim in scenarios:
        scen_results: dict[str, object] = {}
        for rpol in rescue_policies:
            res = run_demo(
                scenario=scen,
                policy="cooperative",
                comms="normal",
                seed=seed,
                steps=st,
                viewer_seconds=0.0,
                rescue_policy=rpol,
            )
            bogged_end = cast(list[str], res["end_state_bogged_robots"])
            secondary_casualties = [r for r in bogged_end if r != primary_victim]
            scen_results[rpol] = {
                "rescue_policy": rpol,
                "viable_seed_placements": res["viable_seed_placements"],
                "completed_recoveries_count": len(cast(list[Any], res["completed_recoveries"])),
                "completed_recoveries": res["completed_recoveries"],
                "end_state_bogged_robots": bogged_end,
                "secondary_casualties": secondary_casualties,
                "orchestrator_decisions": res["orchestrator_decisions"],
                "completion_time_s": res["completion_time_s"],
                "total_travel_m": res["total_travel_m"],
            }
        comparison[scen] = scen_results
    return comparison


def export_multi_scenario_dashboard(output_path: str | Path, seed: int = 7) -> Path:
    """Run the key Mycelial & Rescue scenarios and export a single interactive top-down HTML dashboard."""
    from ricebot.viewer import SwarmBeliefDashboardRecorder, render_dashboard_html

    scenarios_cfg = [
        ("1. Golden Rescue Episode (Planter Bogged -> Scout Extracts)", "golden_rescue", "normal", 195),
        ("2. High-Risk Refusal (Scout Bogged -> Protect Seed-Carrier Planter)", "high_risk_abstain", "normal", 175),
        ("3. Nominal Comms (Mycelial SLAM + Gossip)", "nominal", "normal", 130),
        ("4. Intermittent RF Blackouts (Diverge & Reconcile)", "nominal", "intermittent", 135),
        ("5. Surprise Bad Soil (Scout Educates Planter)", "bad_soil", "normal", 145),
        ("6. Mid-Episode Obstacle Surprise (SLAM Bypass)", "obstacle", "normal", 145),
    ]
    bundles: dict[str, dict[str, Any]] = {}
    for label, scen, comm, st in scenarios_cfg:
        cfg = EpisodeConfig(
            scenario=scen,
            policy="mycelial",
            comms=comm,
            seed=seed,
            steps=st,
            rescue_policy="react_orchestrator",
        )
        session = create_mujoco_episode_session(cfg)
        sim = cast(MujocoPaddySimulation, session.sim)
        runtime = cast(DecentralizedMycelialRuntime, session.policy)
        dash_rec = SwarmBeliefDashboardRecorder(
            runtime=runtime,
            terrain=sim.world.continuous_terrain,
            sites=DEFAULT_SITES,
            grid_nx=32,
            grid_ny=22,
            snapshot_stride=6,
        )
        session.views = (dash_rec,)
        session.run(seed=seed, steps=st)
        bundles[label] = dash_rec.export_bundle(scenario_name=label)

    out = Path(output_path)
    render_dashboard_html(bundles, output_path=out)
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario",
        choices=(
            "nominal",
            "obstacle",
            "bad_soil",
            "shifted_moisture",
            "sensor_degraded",
            "golden_rescue",
            "high_risk_abstain",
        ),
        default="nominal",
    )
    parser.add_argument(
        "--policy",
        choices=("mycelial", "cooperative", "centralized", "single_rover"),
        default="mycelial",
    )
    parser.add_argument(
        "--rescue-policy",
        choices=("react_orchestrator", "numerical_triage", "always_rescue", "never_rescue"),
        default="react_orchestrator",
    )
    parser.add_argument("--comms", choices=("normal", "intermittent", "isolated"), default="normal")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--steps", type=int, default=140)
    parser.add_argument("--headless", action="store_true", help="Run without launching interactive viewer")
    parser.add_argument("--viewer-seconds", type=float, default=0.0, help="Launch interactive 3D MuJoCo viewer for N seconds")
    parser.add_argument("--compare-policies", action="store_true", help="Run the Policy A vs Policy B evaluation matrix")
    parser.add_argument("--compare-baselines", action="store_true", help="Run the matched-scenario rescue baseline comparison")
    parser.add_argument("--export-dashboard", type=str, default=None, help="Export interactive top-down HTML belief & terramechanics dashboard to path")
    parser.add_argument(
        "--dashboard",
        "--gui",
        action="store_true",
        dest="dashboard",
        help="Launch native side-by-side 3D MuJoCo + 2D Top-Down Belief Heatmap desktop window",
    )
    args = parser.parse_args()

    if args.dashboard:
        from ricebot.viewer import launch_live_dashboard

        launch_live_dashboard(
            scenario=args.scenario,
            comms=args.comms,
            policy=args.policy,
            rescue_policy=args.rescue_policy,
            seed=args.seed,
            auto_play=True,
        )
        return 0

    if args.export_dashboard:
        out = export_multi_scenario_dashboard(args.export_dashboard, seed=args.seed)
        print(json.dumps({"dashboard_exported_to": str(out.resolve())}, indent=2))
        return 0

    if args.compare_baselines:
        comparison = run_baseline_comparison(seed=args.seed)
        print(json.dumps(comparison, indent=2))
        return 0

    if args.compare_policies:
        matrix = run_policy_comparison_matrix(seed=args.seed)
        print(json.dumps(matrix, indent=2))
        return 0

    v_sec = 0.0 if args.headless else args.viewer_seconds
    summary = run_demo(
        scenario=args.scenario,
        policy=args.policy,
        comms=args.comms,
        seed=args.seed,
        steps=args.steps,
        viewer_seconds=v_sec,
        rescue_policy=args.rescue_policy,
    )
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
