#!/usr/bin/env python3
"""Run PaddyPatch rice.v1 3D Rice-Terrace & 4-Wheel Rover Simulator in MuJoCo.

Features:
- Full 3D bunded rice terrace with irrigation canal, 3D green rice seedling stands,
  glossy "Lannitian" waterlogged mud hazard pans (`patch_a`, `patch_d`), and viable
  missing-stand loam beds (`patch_b`, `patch_c`).
- Two 4-wheel differential-drive rovers (`planter` with seed hopper + BeiDou RTK mast,
  `scout` with soil cone-index probe mast), each driven by 4 revolute wheel actuators
  rolling via ground friction.
- Live 3D HUD visual overlays in the interactive MuJoCo viewer (`viewer.user_scn`):
  - Floating status beacons above each patch:
    - Amber = Unexplored / Unknown Soil
    - Red = Sampled Mud Hazard (Rejected: moisture > 0.80 / firmness < 0.20)
    - Cyan = Sampled Viable Stand Gap (Ready for Planting)
    - Bright Gold/Green = Physically Planted & Ground Contact Verified!
  - Live target vectors from each rover to its active assigned patch (`SAMPLE` vs `PLANT`).
  - Live forward LiDAR / rangefinder laser rays (`3-ray fan` per rover).
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from math import cos, sin
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import mujoco
import mujoco.viewer
import numpy as np

from ricebot.contracts import Decision, ObservationFrame, TaskMode
from ricebot.control import ControlLimits, LocalController
from ricebot.mission import MeasuredFieldMap, SimpleMissionPlanner
from ricebot.mujoco_sim.adapter import DEFAULT_CAPABILITIES, DEFAULT_SITES, MujocoPaddySimulation
from ricebot.runtime import EpisodeRunner, MissionCore


class TelemetryRecorder:
    """ReadOnlyView subscriber that records a concise trace without touching physics."""

    def __init__(self) -> None:
        self.events: list[dict[str, object]] = []

    def consume(self, frame: ObservationFrame, decision: Decision) -> None:
        if frame.plant_feedback or decision.ingested_samples > 0 or any(c.release_seed for c in decision.commands):
            self.events.append(
                {
                    "time_s": frame.time_s,
                    "ingested_samples": decision.ingested_samples,
                    "robots": {
                        r.robot_id: {
                            "pose": (r.pose.x_m, r.pose.y_m, r.pose.yaw_rad),
                            "front_range_m": r.front_range_m,
                            "seeds_remaining": r.seeds_remaining,
                        }
                        for r in frame.robots
                    },
                    "intents": [
                        {"robot_id": i.robot_id, "mode": i.mode.value, "target": i.target_site_id, "reason": i.reason}
                        for i in decision.intents
                    ],
                    "commands": [
                        {
                            "robot_id": c.robot_id,
                            "linear_m_s": round(c.linear_m_s, 3),
                            "angular_rad_s": round(c.angular_rad_s, 3),
                            "release_seed": c.release_seed,
                            "target_site_id": c.target_site_id,
                        }
                        for c in decision.commands
                    ],
                    "plant_feedback": [asdict(fb) for fb in frame.plant_feedback],
                }
            )


def _draw_viewer_hud_overlays(
    viewer: Any,
    sim: MujocoPaddySimulation,
    field_map: MeasuredFieldMap,
    planner: SimpleMissionPlanner,
    frame: ObservationFrame,
    decision: Decision,
) -> None:
    """Render live 3D belief-map beacons, LiDAR rays, and task lines into viewer.user_scn."""
    scn = viewer.user_scn
    scn.ngeom = 0
    identity_mat = np.eye(3, dtype=np.float64).flatten()
    planted_sites = {d.target_site_id for d in sim.completed_drops if d.contact_observed_at_s is not None}

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

    # 1. Draw floating belief-map beacons above each candidate site
    for site_id, site in field_map.sites.items():
        if not site.permitted:
            continue
        est = field_map.estimate(site_id)
        cx, cy = site.center.x_m, site.center.y_m
        if site_id in planted_sites:
            # Bright gold/green beacon: physically planted!
            rgba = [0.15, 0.95, 0.30, 0.90]
            radius = 0.055
        elif not est.complete:
            # Amber beacon: unknown / unsampled patch
            rgba = [0.95, 0.70, 0.15, 0.65]
            radius = 0.038
        elif planner._eligible(est):
            # Cyan beacon: sampled & verified viable stand gap
            rgba = [0.15, 0.85, 0.98, 0.85]
            radius = 0.048
        else:
            # Red beacon: sampled waterlogged mud hazard (rejected!)
            rgba = [0.95, 0.20, 0.18, 0.85]
            radius = 0.048

        add_sphere([cx, cy, 0.22], radius, rgba)
        add_line([cx, cy, 0.01], [cx, cy, 0.18], 0.006, rgba)

    # 2. Draw active task assignment vectors & forward rangefinder laser rays for each rover
    obs_by_id = {r.robot_id: r for r in frame.robots}
    for intent in decision.intents:
        r = obs_by_id[intent.robot_id]
        rx, ry, ryaw = r.pose.x_m, r.pose.y_m, r.pose.yaw_rad

        # Forward LiDAR ray visualization (capped to 0.75m visual beam length for clarity)
        beam_len = min(r.front_range_m if r.front_range_m is not None else 0.75, 0.75)
        bx = rx + beam_len * cos(ryaw)
        by = ry + beam_len * sin(ryaw)
        beam_rgba = (
            [1.0, 0.15, 0.15, 0.85]
            if (r.front_range_m is not None and r.front_range_m <= 0.40)
            else [0.25, 0.95, 0.55, 0.45]
        )
        add_line([rx, ry, 0.08], [bx, by, 0.08], 0.004, beam_rgba)

        # Target assignment line from rover to target patch
        if intent.target_site_id and intent.target_site_id in field_map.sites:
            tsite = field_map.sites[intent.target_site_id]
            line_rgba = (
                [0.98, 0.84, 0.15, 0.80]
                if intent.mode == TaskMode.PLANT
                else ([0.25, 0.68, 0.98, 0.65] if intent.robot_id == "planter" else [0.25, 0.92, 0.45, 0.65])
            )
            add_line([rx, ry, 0.11], [tsite.center.x_m, tsite.center.y_m, 0.18], 0.005, line_rgba)


def run_demo(
    scenario: str = "nominal",
    seed: int = 7,
    steps: int = 135,
    viewer_seconds: float = 0.0,
) -> dict[str, object]:
    inject_at = 1.6 if scenario == "obstacle" else None
    sim = MujocoPaddySimulation(
        sites=DEFAULT_SITES,
        probe_radius_m=0.28,
        substeps_per_tick=25,
        inject_obstacle_at_s=inject_at,
        obstacle_xy=(0.75, -0.25),
    )
    field_map = MeasuredFieldMap(DEFAULT_SITES, sample_radius_m=0.6)
    planner = SimpleMissionPlanner(DEFAULT_CAPABILITIES)
    controller = LocalController(
        ControlLimits(speed_m_s=0.38, turn_rad_s=1.15, stop_range_m=0.32, site_reach_m=0.22)
    )
    core = MissionCore(field_map, planner, controller)
    recorder = TelemetryRecorder()

    if viewer_seconds > 0:
        frame = sim.reset(seed)
        start_wall = time.time()
        with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
            # Frame the entire bunded rice terrace, irrigation canal, 3D crops, and both 4-wheel rovers
            viewer.cam.lookat[:] = [1.25, -0.08, 0.06]
            viewer.cam.distance = 2.95
            viewer.cam.elevation = -34.0
            viewer.cam.azimuth = 112.0
            for _ in range(steps):
                if not viewer.is_running():
                    break
                decision = core.decide(frame)
                recorder.consume(frame, decision)
                _draw_viewer_hud_overlays(viewer, sim, field_map, planner, frame, decision)
                frame = sim.step(decision.commands)
                viewer.sync()
                time.sleep(0.045)
            # Keep window open for remaining viewer_seconds if finished early
            while viewer.is_running() and (time.time() - start_wall) < viewer_seconds:
                _draw_viewer_hud_overlays(viewer, sim, field_map, planner, frame, decision)
                viewer.sync()
                time.sleep(0.05)
    else:
        runner = EpisodeRunner(sim, core, view=recorder)
        runner.run(seed=seed, steps=steps)

    estimates = {
        site_id: asdict(field_map.estimate(site_id))
        for site_id in ("patch_a", "patch_b", "patch_c", "patch_d")
    }
    return {
        "engine": f"MuJoCo {mujoco.__version__} (3D Rice Terrace + 4-Wheel Articulated Rovers SimulationPort)",
        "scenario": scenario,
        "seed": seed,
        "steps": steps,
        "obstacle_injected": sim.obstacle_injected,
        "site_estimates": estimates,
        "completed_seed_drops": [asdict(d) for d in sim.completed_drops],
        "key_events": recorder.events,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=("nominal", "obstacle"), default="nominal")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--steps", type=int, default=135)
    parser.add_argument("--headless", action="store_true", help="Run without launching interactive viewer")
    parser.add_argument("--viewer-seconds", type=float, default=0.0, help="Launch interactive 3D MuJoCo viewer for N seconds")
    args = parser.parse_args()

    v_sec = 0.0 if args.headless else args.viewer_seconds
    summary = run_demo(scenario=args.scenario, seed=args.seed, steps=args.steps, viewer_seconds=v_sec)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
