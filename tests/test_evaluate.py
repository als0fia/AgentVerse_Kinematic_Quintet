import json
import tempfile
import unittest
from pathlib import Path

from scripts.evaluate import EvidenceError, compare, load_episodes, percentile_nearest_rank, summary


FIXTURE = Path(__file__).parent / "fixtures"


class EvaluationTests(unittest.TestCase):
    def write_rows(self, rows):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "episodes.jsonl"
        path.write_text("\n".join(json.dumps(x) for x in rows) + "\n", encoding="utf-8")
        return path

    def test_summary_and_pairing_on_synthetic_fixtures(self):
        c = load_episodes(FIXTURE / "candidate.jsonl")
        b = load_episodes(FIXTURE / "baseline.jsonl")
        result = compare(c, b)
        self.assertEqual(result["candidate"]["episodes"], 3)
        self.assertEqual(result["baseline"]["successes"], 1)
        self.assertEqual(result["paired_comparison"]["candidate_only_successes"], 1)
        self.assertEqual(result["paired_comparison"]["baseline_only_successes"], 0)
        self.assertEqual(result["candidate"]["failure_reasons"], {"synthetic-timeout": 1})

    def test_zero_denominator_rejected(self):
        self.assertRaises(EvidenceError, load_episodes, self.write_rows([]))

    def test_duplicate_pair_rejected(self):
        row = {"case_id": "same", "seed": 1, "success": True, "elapsed_ms": 10}
        self.assertRaises(EvidenceError, load_episodes, self.write_rows([row, row]))

    def test_invalid_types_rejected(self):
        for field, value in [("seed", True), ("success", 1), ("elapsed_ms", -1), ("elapsed_ms", float("inf")), ("failure_reason", 7)]:
            with self.subTest(field=field, value=value):
                row = {"case_id": "a", "seed": 1, "success": True, "elapsed_ms": 10}
                row[field] = value
                self.assertRaises(EvidenceError, load_episodes, self.write_rows([row]))

    def test_mismatched_cases_are_not_compared(self):
        c = load_episodes(FIXTURE / "candidate.jsonl")
        b = load_episodes(FIXTURE / "baseline.jsonl")
        del b[("synthetic-c", 2)]
        self.assertRaises(EvidenceError, compare, c, b)

    def test_latency_percentile(self):
        self.assertEqual(percentile_nearest_rank([1, 3, 2, 5, 4], 95), 5)
        self.assertEqual(percentile_nearest_rank([1, 3, 2, 5, 4], 50), 3)


if __name__ == "__main__":
    unittest.main()
