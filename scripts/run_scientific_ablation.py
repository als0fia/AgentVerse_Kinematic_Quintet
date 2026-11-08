#!/usr/bin/env python3
"""Run Antenna's canonical scientific evaluation, baseline comparison, and component ablation suite.

Usage:
    python scripts/run_scientific_ablation.py --out-dir artifacts/ablation
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import json
from pathlib import Path
from statistics import mean, median
import subprocess
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ricebot.evaluation import (  # noqa: E402
    EpisodeEvaluationRecord,
    run_canonical_episode,
    run_multi_attempt_retry_ablation,
    run_residual_learner_ablation_study,
)
from ricebot.runtime import EpisodeConfig  # noqa: E402

RESCUE_POLICIES = (
    "no_rescue",
    "always_rescue",
    "calibrated_triage",
    "react_orchestrator",
)


def write_schema_jsonl(records: list[EpisodeEvaluationRecord], path: Path) -> None:
    """Write strict schemas/episode.schema.json records to a JSONL file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        for rec in records:
            f.write(json.dumps(rec.to_schema_record(), sort_keys=True) + "\n")


def run_official_evaluator(
    candidate_jsonl: Path,
    baseline_jsonl: Path,
    out_json: Path,
) -> dict[str, Any]:
    """Invoke scripts/evaluate.py --candidate ... --baseline ... --out ... and return parsed JSON."""
    if out_json.exists():
        out_json.unlink()
    cmd = [
        sys.executable,
        str(ROOT / "scripts" / "evaluate.py"),
        "--candidate",
        str(candidate_jsonl),
        "--baseline",
        str(baseline_jsonl),
        "--out",
        str(out_json),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True, check=True)
    return json.loads(proc.stdout)


def summarize_records(records: list[EpisodeEvaluationRecord]) -> dict[str, Any]:
    """Aggregate descriptive statistics across a list of EpisodeEvaluationRecord objects."""
    if not records:
        return {}
    n = len(records)
    return {
        "episodes": n,
        "successes": sum(1 for r in records if r.success),
        "success_rate": round(sum(1 for r in records if r.success) / n, 4),
        "yield_only_successes": sum(1 for r in records if r.yield_only_success),
        "yield_only_success_rate": round(
            sum(1 for r in records if r.yield_only_success) / n, 4
        ),
        "viable_seed_placements_total": sum(r.viable_seed_placements for r in records),
        "viable_sites_total": sum(r.viable_sites_total for r in records),
        "viable_seed_placements_mean": round(
            mean(r.viable_seed_placements for r in records), 3
        ),
        "viable_yield_rate_mean": round(mean(r.viable_yield_rate for r in records), 4),
        "post_rescue_viable_placements_total": sum(
            r.post_rescue_viable_placements for r in records
        ),
        "wasted_mud_drops_total": sum(r.wasted_mud_drops for r in records),
        "false_abstentions_total": sum(r.false_abstentions for r in records),
        "duplicate_plantings_total": sum(r.duplicate_plantings for r in records),
        "rescues_commanded_total": sum(r.rescue_commanded_count for r in records),
        "physical_latch_engaged_episodes": sum(
            1 for r in records if r.physical_latch_engaged
        ),
        "physical_extractions_verified_total": sum(
            r.physical_extractions_verified for r in records
        ),
        "commanded_without_physical_extraction_total": sum(
            r.commanded_without_physical_extraction for r in records
        ),
        "independent_mobility_resumed_total": sum(
            r.independent_mobility_resumed_count for r in records
        ),
        "max_victim_extraction_displacement_m": round(
            max(r.max_victim_extraction_displacement_m for r in records), 4
        ),
        "secondary_casualties_total": sum(r.secondary_casualties for r in records),
        "unsafe_severe_mud_dispatches_total": sum(
            r.unsafe_severe_mud_dispatches for r in records
        ),
        "policy_veto_episodes": sum(1 for r in records if r.policy_veto_count > 0),
        "unexpected_robot_collisions_total": sum(
            r.unexpected_robot_collisions for r in records
        ),
        "max_sampled_contact_force_n": round(
            max(r.max_sampled_contact_force_n for r in records), 3
        ),
        "gossip_packets_mean": round(mean(r.gossip_packets_sent for r in records), 2),
        "gossip_bytes_mean": round(mean(r.gossip_bytes_sent for r in records), 1),
        "stale_shared_belief_events_total": sum(
            r.stale_shared_belief_events for r in records
        ),
        "map_divergence_l1_mean": round(mean(r.map_divergence_l1 for r in records), 4),
        "decision_latency_mean_ms": round(
            mean(r.decision_latency_mean_ms for r in records), 4
        ),
        "decision_latency_median_ms": round(
            median(r.decision_latency_median_ms for r in records), 4
        ),
        "decision_latency_p95_ms": round(
            max(r.decision_latency_p95_ms for r in records), 4
        ),
        "episode_decision_elapsed_mean_ms": round(
            mean(r.episode_decision_elapsed_ms for r in records), 3
        ),
        "episode_wall_elapsed_mean_ms": round(
            mean(r.episode_wall_elapsed_ms for r in records), 2
        ),
    }


def run_rescue_baseline_and_ablation1(
    out_dir: Path,
    diag_dir: Path,
) -> tuple[dict[str, list[EpisodeEvaluationRecord]], dict[str, Any]]:
    """Run matched rescue baseline comparison & Ablation 1 across golden_rescue, high_risk_abstain, and randomized (1..12)."""
    records_by_policy: dict[str, list[EpisodeEvaluationRecord]] = {
        pol: [] for pol in RESCUE_POLICIES
    }

    cases: list[tuple[str, str, int, int]] = [
        ("golden_rescue", "golden_rescue", 7, 220),
        ("high_risk_abstain", "high_risk_abstain", 7, 220),
    ]
    for s in range(1, 13):
        cases.append(("randomized", "randomized", s, 220))

    for pol in RESCUE_POLICIES:
        for case_id, scenario, seed, steps in cases:
            cfg = EpisodeConfig(
                scenario=scenario,
                seed=seed,
                steps=steps,
                rescue_policy=pol,
            )
            rec = run_canonical_episode(
                cfg,
                case_id=case_id,
                policy_label=pol,
                rescue_policy_override=pol,
                diagnostic_trace_dir=diag_dir if seed == 7 else None,
            )
            records_by_policy[pol].append(rec)

    jsonl_paths: dict[str, Path] = {}
    for pol, recs in records_by_policy.items():
        p = out_dir / f"rescue_{pol}.jsonl"
        write_schema_jsonl(recs, p)
        jsonl_paths[pol] = p

    paired_evals = {
        "react_vs_no_rescue": run_official_evaluator(
            jsonl_paths["react_orchestrator"],
            jsonl_paths["no_rescue"],
            out_dir / "eval_react_vs_no_rescue.json",
        ),
        "react_vs_always_rescue": run_official_evaluator(
            jsonl_paths["react_orchestrator"],
            jsonl_paths["always_rescue"],
            out_dir / "eval_react_vs_always_rescue.json",
        ),
        "react_vs_calibrated_triage": run_official_evaluator(
            jsonl_paths["react_orchestrator"],
            jsonl_paths["calibrated_triage"],
            out_dir / "eval_react_vs_calibrated_triage.json",
        ),
    }

    retry_ablation = run_multi_attempt_retry_ablation()

    summary = {
        "overall_by_policy": {
            pol: summarize_records(recs) for pol, recs in records_by_policy.items()
        },
        "golden_rescue_seed7": {
            pol: next(
                r.to_full_dict()
                for r in recs
                if r.case_id == "golden_rescue" and r.seed == 7
            )
            for pol, recs in records_by_policy.items()
        },
        "high_risk_abstain_seed7": {
            pol: next(
                r.to_full_dict()
                for r in recs
                if r.case_id == "high_risk_abstain" and r.seed == 7
            )
            for pol, recs in records_by_policy.items()
        },
        "randomized_12_seeds_by_policy": {
            pol: summarize_records([r for r in recs if r.case_id == "randomized"])
            for pol, recs in records_by_policy.items()
        },
        "paired_evaluator_reports": paired_evals,
        "multi_attempt_retry_ablation": retry_ablation,
        "per_episode_records": {
            pol: [r.to_full_dict() for r in recs]
            for pol, recs in records_by_policy.items()
        },
    }
    return records_by_policy, summary


def run_two_factor_interaction_experiment(
    records_by_policy: dict[str, list[EpisodeEvaluationRecord]],
) -> dict[str, Any]:
    """Analyze Rescue Policy (4 levels) x Hazard Severity (recoverable, severe, moderate) across randomized seeds 1..12."""
    severities = ("recoverable", "severe", "moderate")
    grid: dict[str, dict[str, Any]] = {}
    seed_groups: dict[str, list[int]] = defaultdict(list)

    for rec in records_by_policy["react_orchestrator"]:
        if rec.case_id == "randomized":
            seed_groups[rec.mud_severity].append(rec.seed)

    for pol, recs in records_by_policy.items():
        grid[pol] = {}
        for sev in severities:
            cell_recs = [
                r for r in recs if r.case_id == "randomized" and r.mud_severity == sev
            ]
            grid[pol][sev] = summarize_records(cell_recs)

    return {
        "factors": {
            "factor_a_rescue_policy": list(RESCUE_POLICIES),
            "factor_b_mud_severity": list(severities),
            "seeds_by_severity": {k: sorted(v) for k, v in seed_groups.items()},
        },
        "interaction_cells": grid,
    }


def run_coordination_and_comms_ablation(
    out_dir: Path,
) -> dict[str, Any]:
    """Run Field Coordination comparison (single_rover vs cooperative vs mycelial) & Ablation 2 (Comms modes)."""
    # Core 8 non-trap scenarios/seeds
    base_cases: list[tuple[str, str, int, int]] = [
        ("nominal", "nominal", 7, 140),
        ("bad_soil", "bad_soil", 7, 140),
        ("sensor_degraded", "sensor_degraded", 7, 165),
    ]
    for s in range(1, 6):
        base_cases.append(("shifted_moisture", "shifted_moisture", s, 140))

    # Run across 5 configurations:
    # 1. single_rover
    # 2. cooperative (shared map)
    # 3. mycelial_normal (P2P gossip, comm_mode="normal")
    # 4. mycelial_intermittent (P2P gossip, comm_mode="intermittent")
    # 5. mycelial_isolated (P2P gossip disabled, comm_mode="isolated")
    configs = [
        ("single_rover", "single_rover", "normal", True),
        ("cooperative", "cooperative", "normal", False),
        ("mycelial_normal", "mycelial", "normal", False),
        ("mycelial_intermittent", "mycelial", "intermittent", False),
        ("mycelial_isolated", "mycelial", "isolated", False),
    ]

    records_by_cfg: dict[str, list[EpisodeEvaluationRecord]] = {
        label: [] for label, _, _, _ in configs
    }

    for label, pol, comms, single_only in configs:
        for case_id, scenario, seed, steps in base_cases:
            cfg = EpisodeConfig(
                scenario=scenario,
                policy=pol,
                comms=comms,
                seed=seed,
                steps=steps,
                single_rover_only=single_only,
                include_shadow_mycelial=(pol != "mycelial"),
            )
            rec = run_canonical_episode(
                cfg,
                case_id=case_id,
                policy_label=label,
            )
            records_by_cfg[label].append(rec)

    jsonl_paths: dict[str, Path] = {}
    for label, recs in records_by_cfg.items():
        p = out_dir / f"coord_{label}.jsonl"
        write_schema_jsonl(recs, p)
        jsonl_paths[label] = p

    paired_evals = {
        "mycelial_normal_vs_single_rover": run_official_evaluator(
            jsonl_paths["mycelial_normal"],
            jsonl_paths["single_rover"],
            out_dir / "eval_mycelial_vs_single_rover.json",
        ),
        "mycelial_normal_vs_cooperative": run_official_evaluator(
            jsonl_paths["mycelial_normal"],
            jsonl_paths["cooperative"],
            out_dir / "eval_mycelial_vs_cooperative.json",
        ),
        "mycelial_normal_vs_isolated": run_official_evaluator(
            jsonl_paths["mycelial_normal"],
            jsonl_paths["mycelial_isolated"],
            out_dir / "eval_mycelial_normal_vs_isolated.json",
        ),
        "mycelial_normal_vs_intermittent": run_official_evaluator(
            jsonl_paths["mycelial_normal"],
            jsonl_paths["mycelial_intermittent"],
            out_dir / "eval_mycelial_normal_vs_intermittent.json",
        ),
    }

    return {
        "summary_by_configuration": {
            label: summarize_records(recs) for label, recs in records_by_cfg.items()
        },
        "paired_evaluator_reports": paired_evals,
        "per_episode_records": {
            label: [r.to_full_dict() for r in recs]
            for label, recs in records_by_cfg.items()
        },
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=ROOT / "artifacts" / "ablation",
        help="Output directory for JSONL episode logs and paired evaluation reports.",
    )
    args = parser.parse_args(argv)
    out_dir: Path = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    diag_dir = out_dir / "diagnostics"
    diag_dir.mkdir(parents=True, exist_ok=True)

    print("[1/4] Running Matched Rescue Baselines & Ablation 1 (Triage & Orchestrator)...")
    rescue_records, rescue_summary = run_rescue_baseline_and_ablation1(out_dir, diag_dir)

    print("[2/4] Running Two-Factor Interaction Experiment (Rescue Policy x Hazard Severity)...")
    interaction_summary = run_two_factor_interaction_experiment(rescue_records)

    print("[3/4] Running Field Coordination & Ablation 2 (Comms / Peer Sharing)...")
    coord_summary = run_coordination_and_comms_ablation(out_dir)

    print("[4/4] Running Ablation 3 (Adaptive Residual Recovery Learner)...")
    learner_summary = run_residual_learner_ablation_study()

    master_report = {
        "schema_version": "rice.v1.scientific_ablation",
        "commit": "a835e986f83c4c08f071e6820f172dc7c03e1505",
        "suite_1_rescue_baselines_and_ablation_1": rescue_summary,
        "suite_2_coordination_and_ablation_2_comms": coord_summary,
        "suite_3_ablation_3_residual_learner": learner_summary,
        "suite_4_two_factor_interaction": interaction_summary,
    }

    master_path = out_dir / "scientific_ablation_summary.json"
    master_path.write_text(json.dumps(master_report, indent=2) + "\n", encoding="utf-8")
    print(f"Done. Wrote master ablation summary to {master_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
