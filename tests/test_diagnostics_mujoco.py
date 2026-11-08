"""One real MuJoCo smoke check for the standalone diagnostics CLI."""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo not installed")
class MujocoDiagnosticsSmokeTests(unittest.TestCase):
    def test_real_headless_episode_writes_parseable_physics_trace(self):
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run([
                sys.executable, "scripts/diagnose_mujoco.py",
                "--scenario", "nominal", "--policy", "centralized",
                "--seed", "7", "--steps", "6", "--out-dir", directory,
            ], cwd=ROOT, capture_output=True, text=True, timeout=90)
            self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
            index = json.loads(result.stdout)
            self.assertEqual(index["runs"], 1)
            trace = Path(index["seed_summaries"][0]["trace"])
            self.assertTrue(trace.exists())
            rows = [json.loads(line) for line in trace.read_text().splitlines()]
            self.assertEqual(rows[0]["kind"], "run")
            ticks = [row for row in rows if row["kind"] == "tick"]
            self.assertEqual(len(ticks), 6)
            self.assertIn("contacts", ticks[-1]["truth"])
            self.assertIn("sinkage_m", ticks[-1]["truth"]["robots"]["planter"])
            self.assertEqual(rows[-1]["kind"], "summary")


if __name__ == "__main__":
    unittest.main()
