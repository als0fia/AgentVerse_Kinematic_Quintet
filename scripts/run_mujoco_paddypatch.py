#!/usr/bin/env python3
"""Run PaddyPatch rice.v1 cooperative mapping & physical seed-drop in MuJoCo.

Demonstrates:
1. Two physically simulated rovers (`planter` and `scout`) connected to the
   repo's unmodified `MissionCore`, `MeasuredFieldMap`, `SimpleMissionPlanner`,
   and `LocalController` (`src/ricebot/`).
2. Scout rover sampling `patch_a` (waterlogged, moisture=0.91 -> rejected by planner).
3. Planter rover sampling `patch_b` (viable stand gap -> selected for PLANT),
   physically releasing rigid-body `seed_proxy_0` from its hopper, and MuJoCo's
   contact solver confirming ground landing (`PlantFeedback`).
4. Optional mid-episode obstacle surprise (`--scenario obstacle`) where an
   unannounced obstacle blocks the lane and the rangefinder triggers a safe stop.
5. Optional interactive 3D MuJoCo viewer (`--viewer-seconds N`) and offscreen
   snapshot rendering (`--save-snapshot PATH`).
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import mujoco
import mujoco.viewer

from ricebot.contracts import Decision, ObservationFrame
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


def run_demo(
    scenario: str = "nominal",
    seed: int = 7,
    steps: int = 120,
    viewer_seconds: float = 0.0,
) -> dict[str, object]:
    inject_at = 1.5 if scenario == "obstacle" else None
    sim = MujocoPaddySimulation(
        sites=DEFAULT_SITES,
        probe_radius_m=0.22,
        substeps_per_tick=20,
        inject_obstacle_at_s=inject_at,
        obstacle_xy=(0.95, 0.00),
    )
    field_map = MeasuredFieldMap(DEFAULT_SITES, sample_radius_m=0.6)
    planner = SimpleMissionPlanner(DEFAULT_CAPABILITIES)
    controller = LocalController(ControlLimits(speed_m_s=0.35, turn_rad_s=1.0, stop_range_m=0.35, site_reach_m=0.18))
    core = MissionCore(field_map, planner, controller)
    recorder = TelemetryRecorder()

    if viewer_seconds > 0:
        frame = sim.reset(seed)
        start_wall = time.time()
        with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
            # Frame the entire rice plot and both rovers clearly from an angled bird's-eye view
            viewer.cam.lookat[:] = [0.90, 0.00, 0.05]
            viewer.cam.distance = 2.4
            viewer.cam.elevation = -42.0
            viewer.cam.azimuth = 90.0
            for step_idx in range(steps):
                if not viewer.is_running():
                    break
                decision = core.decide(frame)
                recorder.consume(frame, decision)
                frame = sim.step(decision.commands)
                viewer.sync()
                time.sleep(0.05)
            # Keep window open for remaining viewer_seconds if finished early
            while viewer.is_running() and (time.time() - start_wall) < viewer_seconds:
                viewer.sync()
                time.sleep(0.05)
    else:
        runner = EpisodeRunner(sim, core, view=recorder)
        runner.run(seed=seed, steps=steps)

    estimates = {
        site_id: asdict(field_map.estimate(site_id))
        for site_id in ("patch_a", "patch_b", "patch_c")
    }
    return {
        "engine": f"MuJoCo {mujoco.__version__} (rapid-prototyping SimulationPort)",
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
    parser.add_argument("--steps", type=int, default=120)
    parser.add_argument("--headless", action="store_true", help="Run without launching interactive viewer")
    parser.add_argument("--viewer-seconds", type=float, default=0.0, help="Launch interactive 3D MuJoCo viewer for N seconds")
    args = parser.parse_args()

    v_sec = 0.0 if args.headless else args.viewer_seconds
    summary = run_demo(scenario=args.scenario, seed=args.seed, steps=args.steps, viewer_seconds=v_sec)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
