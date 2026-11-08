#!/usr/bin/env python3
"""Matched, recorded Antenna coordination, safety and fleet-scaling experiments.

Uses exactly the same EpisodeSession, MuJoCo physics, independent evaluator and
seeded field layouts as the desktop application. No simulated outcome is invented.

Example (short smoke):
    python scripts/run_scaled_field_study.py --seeds 3 --fleets 4 \
      --scales 1.6 --policies mycelial --comms normal isolated \
      --rescue-policies react_orchestrator --steps 40

For longer trials increase --steps and seed count. The same seed/scale/fleet
defines the same physical field across candidate policies and comms modes.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from statistics import mean

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ricebot.evaluation import run_canonical_episode  # noqa: E402
from ricebot.runtime import EpisodeConfig  # noqa: E402


def run_study(
    seeds: list[int],
    fleets: list[int],
    scales: list[float],
    policies: list[str],
    comms_modes: list[str],
    rescue_policies: list[str],
    steps: int,
) -> dict[str, object]:
    """Execute matched cases without changing the world between policy variants."""
    if not seeds or not fleets or not scales or steps < 1:
        raise ValueError("nonempty seeds/fleets/scales and positive steps required")
    results: list[dict[str, object]] = []
    for seed in seeds:
        for fleet in fleets:
            for scale in scales:
                for policy in policies:
                    modes = comms_modes if policy == "mycelial" else ["normal"]
                    for comm_mode in modes:
                        for rescue in rescue_policies:
                            cfg = EpisodeConfig(
                                scenario="randomized",
                                policy=policy,
                                comms=comm_mode,
                                rescue_policy=rescue,
                                seed=seed,
                                num_rovers=fleet,
                                map_scale=scale,
                                gossip_delay_s=0.4,
                                active_mycelial_in_rescue=(policy == "mycelial"),
                                include_shadow_mycelial=False,
                                steps=steps,
                            )
                            record = run_canonical_episode(cfg, rescue_policy_override=rescue)
                            if record.num_rovers != fleet or abs(record.map_scale - scale) > 1e-8:
                                raise AssertionError("Effective configuration silently changed")
                            if policy == "mycelial" and record.active_policy_type != "DecentralizedMycelialRuntime":
                                raise AssertionError("Mycelial trial executed the wrong policy")
                            if policy == "cooperative" and record.active_policy_type != "MissionCore":
                                raise AssertionError("Cooperative trial executed the wrong policy")
                            results.append(record.to_full_dict())

    # Paired comparisons are calculated ONLY when all physical inputs match.
    lookup = {
        (r["seed"], r["num_rovers"], r["map_scale"], r["rescue_policy"],
         r["coordination_policy"], r["comms_mode"]): r
        for r in results
    }
    comparisons: list[dict[str, object]] = []
    for r in results:
        if r["coordination_policy"] != "mycelial" or r["comms_mode"] != "normal":
            continue
        key = (r["seed"], r["num_rovers"], r["map_scale"], r["rescue_policy"], "mycelial", "isolated")
        no_gossip = lookup.get(key)
        if no_gossip is None:
            continue
        comparisons.append({
            "seed": r["seed"],
            "num_rovers": r["num_rovers"],
            "map_scale": r["map_scale"],
            "rescue_policy": r["rescue_policy"],
            "comparison": "active_mycelial_gossip_vs_isolated",
            "extra_viable_placements": r["viable_seed_placements"] - no_gossip["viable_seed_placements"],
            "extra_sites_visited": r["patches_visited"] - no_gossip["patches_visited"],
            "collision_difference": r["unexpected_robot_collisions"] - no_gossip["unexpected_robot_collisions"],
            "belief_divergence_difference": round(
                r["map_divergence_l1"] - no_gossip["map_divergence_l1"], 5
            ),
            "gossip_packets": r["gossip_packets_sent"],
            "gossip_bytes": r["gossip_bytes_sent"],
            "wall_time_difference_ms": round(
                r["episode_wall_elapsed_ms"] - no_gossip["episode_wall_elapsed_ms"], 2
            ),
        })
    mean_viable = mean(float(r["viable_seed_placements"]) for r in results) if results else None
    return {
        "schema": "antenna.scaled-field-study.v1",
        "notes": [
            "Synthetic MuJoCo terrain and physical seed-contact fixture; no agricultural yield validation",
            "Map area, site count and fleet size are recorded per trial",
            "LLM fallback without a valid API response is deterministic, not evidence of LLM benefit",
            "The binary success field retains historical two-rover semantics; rely on measured fleet metrics for N-rover studies",
            "Gossip communication volumes are simulated packets, not measured hardware radio traffic",
        ],
        "runs": results,
        "paired_comparisons": comparisons,
        "summary": {"trials": len(results), "matched_gossip_pairs": len(comparisons),
                    "mean_viable_seed_placements": mean_viable},
    }


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--seeds", nargs="+", type=int, default=[3, 8])
    p.add_argument("--fleets", nargs="+", type=int, default=[2, 4, 6])
    p.add_argument("--scales", nargs="+", type=float, default=[1.0, 1.6])
    p.add_argument("--policies", nargs="+", choices=["mycelial", "cooperative"], default=["mycelial", "cooperative"])
    p.add_argument("--comms", nargs="+", choices=["normal", "isolated", "intermittent"], default=["normal", "isolated"])
    p.add_argument("--rescue-policies", nargs="+", choices=["react_orchestrator", "calibrated_triage", "always_rescue", "no_rescue"], default=["react_orchestrator"])
    p.add_argument("--steps", type=int, default=180)
    p.add_argument("--out", type=Path, default=ROOT / "artifacts" / "scaled_field_study.json")
    args = p.parse_args()
    study = run_study(args.seeds, args.fleets, args.scales,
                      args.policies, args.comms, args.rescue_policies, args.steps)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(study, indent=2, sort_keys=True), encoding="utf-8")
    print(f"Wrote {study['summary']['trials']} real simulated episodes to {args.out}")
    print(f"Matched gossip pairs: {study['summary']['matched_gossip_pairs']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
