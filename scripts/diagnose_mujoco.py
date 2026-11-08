#!/usr/bin/env python3
"""Run the existing Antenna MuJoCo simulation under independent forensic probes.

Examples:
    python scripts/diagnose_mujoco.py --scenario golden_rescue --seed 7
    python scripts/diagnose_mujoco.py --scenario golden_rescue --no-scripted-trap --seed 7 --seed 9
    python scripts/diagnose_mujoco.py --scenario nominal --policy mycelial --seed 7
Outputs are ignored by Git under artifacts/diagnostics by default.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ricebot.agent import CooperativeSwarmPlanner, DecentralizedMycelialRuntime
from ricebot.contracts import RobotCapability
from ricebot.control import ControlLimits, LocalController
from ricebot.diagnostics import EpisodeDiagnosticPort, MujocoPhysicsProbe
from ricebot.mission import MeasuredFieldMap, SimpleMissionPlanner
from ricebot.mujoco_sim.adapter import DEFAULT_CAPABILITIES, DEFAULT_SITES, MujocoPaddySimulation
from ricebot.runtime import EpisodeRunner, MissionCore
from ricebot.world import generate_hidden_soil_truth


SCENARIOS = (
    "nominal", "obstacle", "bad_soil", "shifted_moisture",
    "sensor_degraded", "golden_rescue",
)


def build_episode(scenario: str, policy: str, comms: str, seed: int,
                  scripted_trap: bool) -> tuple[MujocoPaddySimulation, object]:
    sim = MujocoPaddySimulation(
        sites=DEFAULT_SITES,
        probe_radius_m=0.28,
        substeps_per_tick=25,
        inject_obstacle_at_s=1.6 if scenario == "obstacle" else None,
        obstacle_xy=(0.75, -0.25),
        hidden_truth=generate_hidden_soil_truth(scenario, seed),
        trap_robot_id="planter" if scripted_trap else None,
        trap_site_id="patch_d",
        trap_radius_m=0.28,
    )
    if policy == "mycelial" and scenario != "golden_rescue":
        runtime = DecentralizedMycelialRuntime(
            sites=DEFAULT_SITES, capabilities=DEFAULT_CAPABILITIES,
            comm_range_m=2.15, comm_mode=comms,
        )
    else:
        field_map = MeasuredFieldMap(DEFAULT_SITES, sample_radius_m=0.6)
        planner = (
            CooperativeSwarmPlanner(DEFAULT_CAPABILITIES)
            if scenario == "golden_rescue" or policy == "cooperative"
            else SimpleMissionPlanner(DEFAULT_CAPABILITIES)
        )
        controller = LocalController(
            ControlLimits(speed_m_s=0.38, turn_rad_s=1.15,
                          stop_range_m=0.32, site_reach_m=0.22)
        )
        runtime = MissionCore(field_map, planner, controller)
    return sim, runtime


def run_one(*, scenario: str, policy: str, comms: str, seed: int,
            steps: int, scripted_trap: bool, out_dir: Path) -> dict:
    sim, runtime = build_episode(scenario, policy, comms, seed, scripted_trap)
    stem = f"{scenario}-{policy}-seed{seed}-{'trap' if scripted_trap else 'natural'}"
    audit = EpisodeDiagnosticPort(
        sim, MujocoPhysicsProbe(sim), out_dir / (stem + ".jsonl"),
        scenario=scenario, policy=policy, scripted_trap=scripted_trap,
    )
    try:
        EpisodeRunner(audit, runtime, view=audit).run(seed=seed, steps=steps)
        result = audit.finish()
        # Outcome reports are deliberately kept separate from audit checks.
        result["sim_outcomes"] = {
            "completed_seed_drops": len(sim.completed_drops),
            "completed_recoveries": [asdict(r) for r in sim.completed_recoveries],
        }
        return result
    except Exception:
        # Even a broken episode can be diagnosed from the partial trace.
        if not audit._closed:
            audit._event("episode_exception", "error",
                         audit.previous.time_s if audit.previous else 0.0,
                         [], {"traceback": traceback.format_exc()},
                         "Reproduce from the seed and last good tick in the JSONL trace.")
            result = audit.finish()
        else:
            raise
        result["sim_outcomes"] = {"incomplete": True}
        return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=SCENARIOS, default="golden_rescue")
    parser.add_argument("--policy", choices=("cooperative", "centralized", "mycelial"),
                        default="cooperative")
    parser.add_argument("--comms", choices=("normal", "intermittent", "isolated"),
                        default="normal")
    parser.add_argument("--seed", type=int, action="append",
                        help="Repeat to audit independent seeds; default is 7.")
    parser.add_argument("--steps", type=int, default=195)
    parser.add_argument("--no-scripted-trap", action="store_true",
                        help="Disable configured immobilization to probe emergent failures.")
    parser.add_argument("--out-dir", type=Path, default=Path("artifacts/diagnostics"))
    parser.add_argument("--fail-on", choices=("none", "error", "warning"),
                        default="none", help="Optional machine-readable CI failure gate.")
    args = parser.parse_args()
    if args.steps < 1 or any(s < 0 for s in (args.seed or [7])):
        parser.error("steps must be positive and seeds must be nonnegative")
    args.out_dir.mkdir(parents=True, exist_ok=True)
    scripted = args.scenario == "golden_rescue" and not args.no_scripted_trap
    reports = [
        run_one(
            scenario=args.scenario, policy=args.policy, comms=args.comms,
            seed=seed, steps=args.steps, scripted_trap=scripted, out_dir=args.out_dir,
        )
        for seed in (args.seed or [7])
    ]
    for result in reports:
        stem = Path(result["trace_jsonl"]).stem
        (args.out_dir / (stem + "-summary.json")).write_text(
            json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8"
        )
    index = {
        "runs": len(reports),
        "seed_summaries": [
            {"seed": r["seed"], "trace": r["trace_jsonl"],
             "counts": r["event_counts"], "outcomes": r["sim_outcomes"]}
            for r in reports
        ],
    }
    print(json.dumps(index, indent=2))
    severity_levels = {"error": {"error"},
                       "warning": {"error", "warning"},
                       "none": set()}
    gate = severity_levels[args.fail_on]
    return 1 if any(e["severity"] in gate for r in reports for e in r["findings"]) else 0


if __name__ == "__main__":
    raise SystemExit(main())
