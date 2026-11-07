#!/usr/bin/env python3
"""Compare two real, matched experiment logs. Standard library only.

Examples under tests/fixtures are SYNTHETIC unit-test inputs, not hack results.
"""
from __future__ import annotations

import argparse
import json
import math
from collections import Counter
from pathlib import Path
from statistics import mean, median


class EvidenceError(ValueError):
    """An episode file is invalid or experiments cannot be compared."""


def load_episodes(path: str | Path) -> dict[tuple[str, int], dict]:
    """Read strict JSONL; require every attempt to have a unique case+seed key."""
    path = Path(path)
    episodes: dict[tuple[str, int], dict] = {}
    try:
        with path.open(encoding="utf-8") as f:
            for line_no, raw in enumerate(f, start=1):
                if not raw.strip():
                    continue
                try:
                    obj = json.loads(raw)
                except json.JSONDecodeError as exc:
                    raise EvidenceError(f"{path}:{line_no}: invalid JSON: {exc.msg}") from exc
                if not isinstance(obj, dict):
                    raise EvidenceError(f"{path}:{line_no}: each record must be an object")
                required = {"case_id", "seed", "success", "elapsed_ms"}
                allowed = required | {"failure_reason"}
                missing, extra = required - obj.keys(), obj.keys() - allowed
                if missing or extra:
                    raise EvidenceError(f"{path}:{line_no}: missing={sorted(missing)}, unexpected={sorted(extra)}")
                cid, seed, success, elapsed = (obj[k] for k in ("case_id", "seed", "success", "elapsed_ms"))
                if not isinstance(cid, str) or not cid.strip():
                    raise EvidenceError(f"{path}:{line_no}: case_id must be a nonempty string")
                if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
                    raise EvidenceError(f"{path}:{line_no}: seed must be a nonnegative integer")
                if not isinstance(success, bool):
                    raise EvidenceError(f"{path}:{line_no}: success must be boolean")
                if isinstance(elapsed, bool) or not isinstance(elapsed, (float, int)) or not math.isfinite(elapsed) or elapsed < 0:
                    raise EvidenceError(f"{path}:{line_no}: elapsed_ms must be finite and nonnegative")
                reason = obj.get("failure_reason")
                if reason is not None and not isinstance(reason, str):
                    raise EvidenceError(f"{path}:{line_no}: failure_reason must be a string or null")
                key = cid, seed
                if key in episodes:
                    raise EvidenceError(f"{path}:{line_no}: duplicate (case_id, seed) = {key!r}")
                episodes[key] = obj
    except OSError as exc:
        raise EvidenceError(f"Cannot read {path}: {exc}") from exc
    if not episodes:
        raise EvidenceError(f"{path}: no episodes; cannot report a rate with denominator zero")
    return episodes


def percentile_nearest_rank(values: list[float], percent: float) -> float:
    """Nearest-rank percentile; useful descriptive statistic, not a confidence bound."""
    ordered = sorted(values)
    index = max(0, math.ceil(percent / 100 * len(ordered)) - 1)
    return float(ordered[index])


def summary(episodes: dict[tuple[str, int], dict]) -> dict:
    rows = list(episodes.values())
    times = [float(r["elapsed_ms"]) for r in rows]
    failures = Counter((r.get("failure_reason") or "unspecified") for r in rows if not r["success"])
    passed = sum(r["success"] for r in rows)
    return {
        "episodes": len(rows),
        "successes": passed,
        "success_rate": round(passed / len(rows), 6),
        "latency_ms": {
            "mean": round(mean(times), 3),
            "median": round(median(times), 3),
            "p95_nearest_rank": round(percentile_nearest_rank(times, 95), 3),
        },
        "failure_reasons": dict(sorted(failures.items())),
    }


def compare(candidate: dict[tuple[str, int], dict], baseline: dict[tuple[str, int], dict] | None = None) -> dict:
    result = {"status": "descriptive_only_not_statistical_proof", "candidate": summary(candidate)}
    if baseline is None:
        return result
    cand_keys, base_keys = set(candidate), set(baseline)
    if cand_keys != base_keys:
        only_candidate = sorted(cand_keys - base_keys)
        only_baseline = sorted(base_keys - cand_keys)
        raise EvidenceError(
            "Case/seed sets differ; cannot claim a controlled comparison. "
            f"candidate_only={only_candidate[:5]!r}, baseline_only={only_baseline[:5]!r}"
        )
    b = summary(baseline)
    c = result["candidate"]
    wins = sum(candidate[k]["success"] and not baseline[k]["success"] for k in cand_keys)
    losses = sum(baseline[k]["success"] and not candidate[k]["success"] for k in cand_keys)
    result["baseline"] = b
    result["paired_comparison"] = {
        "matched_episodes": len(cand_keys),
        "success_delta_percentage_points": round(100 * (c["success_rate"] - b["success_rate"]), 3),
        "mean_latency_delta_ms": round(c["latency_ms"]["mean"] - b["latency_ms"]["mean"], 3),
        "candidate_only_successes": wins,
        "baseline_only_successes": losses,
        "both_same_success_status": len(cand_keys) - wins - losses,
    }
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", type=Path, required=True, help="JSONL episode records")
    parser.add_argument("--baseline", type=Path, help="JSONL with identical (case_id, seed) set")
    parser.add_argument("--out", type=Path, help="Write JSON report, refusing to overwrite an existing file")
    args = parser.parse_args(argv)
    try:
        result = compare(load_episodes(args.candidate), load_episodes(args.baseline) if args.baseline else None)
        rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
        if args.out:
            try:
                with args.out.open("x", encoding="utf-8") as f:
                    f.write(rendered)
            except OSError as exc:
                raise EvidenceError(f"Cannot create output {args.out}: {exc}") from exc
        print(rendered, end="")
        return 0
    except EvidenceError as exc:
        parser.exit(status=2, message=f"Evidence error: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
