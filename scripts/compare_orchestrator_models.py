#!/usr/bin/env python3
"""Opt-in, capped comparison of live LLM orchestration on matched MuJoCo incidents.

Each model sees the same model-independent seed, scenario, world and controller
configuration. No live network/API call occurs unless --allow-paid-calls is
supplied AND GEMINI_API_KEY is present. Different model identifiers are NOT
automatically presumed to have different sizes or capabilities.

Example:
    python scripts/compare_orchestrator_models.py \
        --models gemini-2.5-flash gemini-2.5-pro \
        --seeds 3 8 --scenarios golden_rescue high_risk_abstain \
        --max-calls 4 --allow-paid-calls
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ricebot.evaluation import run_canonical_episode  # noqa: E402
from ricebot.runtime import EpisodeConfig  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--models", nargs="+", required=True)
    parser.add_argument("--seeds", nargs="+", type=int, default=[7])
    parser.add_argument("--scenarios", nargs="+",
                        choices=["golden_rescue", "high_risk_abstain", "randomized"],
                        default=["golden_rescue", "high_risk_abstain"])
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--max-calls", type=int, default=4)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--out", type=Path, default=ROOT / "artifacts" / "llm_model_comparison.json")
    parser.add_argument("--allow-paid-calls", action="store_true",
                        help="Required explicit permission to invoke the configured provider.")
    args = parser.parse_args()
    if not args.allow_paid_calls:
        parser.error("Network LLM experimentation requires explicit --allow-paid-calls")
    if not os.environ.get("GEMINI_API_KEY"):
        parser.error("GEMINI_API_KEY must be set securely in the process environment")
    if args.max_calls <= 0 or args.timeout <= 0 or args.steps <= 0:
        parser.error("max-calls, timeout and steps must be positive")

    rows: list[dict[str, object]] = []
    for scenario in args.scenarios:
        for seed in args.seeds:
            for model in args.models:
                cfg = EpisodeConfig(
                    scenario=scenario, policy="cooperative",
                    rescue_policy="react_orchestrator", seed=seed, steps=args.steps,
                    orchestrator_model=model, orchestrator_timeout_s=args.timeout,
                    enable_live_llm=True, llm_max_calls=args.max_calls,
                    include_shadow_mycelial=False,
                )
                result = run_canonical_episode(cfg)
                rows.append(result.to_full_dict())
                print(
                    f"{scenario} seed={seed} model={model} "
                    f"calls={result.llm_requests_sent} received={result.llm_responses_received} "
                    f"physical_extractions={result.physical_extractions_verified} "
                    f"secondary_casualties={result.secondary_casualties}"
                )
    report = {
        "schema": "antenna.llm-model-matched.v1",
        "model_ids": args.models,
        "note": (
            "Model results must be interpreted using actual successful response counts. "
            "Fallback outcomes do not demonstrate model benefit. Models share the "
            "same deterministic scenario/seed, bounded control policy and safety gate. "
            "Cost/tokens are not measured by this runner."
        ),
        "runs": rows,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(f"Wrote {len(rows)} recorded live-model trials to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
