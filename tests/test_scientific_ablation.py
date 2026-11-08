"""Unit and integration tests for Antenna's scientific evaluation and ablation suite (Agent C)."""
from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from ricebot.evaluation import (
     canonical_rescue_policy_label,
    resolve_engine_rescue_policy,
    run_canonical_episode,
    run_multi_attempt_retry_ablation,
    run_residual_learner_ablation_study,
)
from ricebot.runtime import EpisodeConfig

ROOT = Path(__file__).resolve().parents[1]


class ScientificAblationTest(unittest.TestCase):
    def test_rescue_policy_alias_mapping(self) -> None:
        self.assertEqual(resolve_engine_rescue_policy("no_rescue"), "never_rescue")
        self.assertEqual(resolve_engine_rescue_policy("never_rescue"), "never_rescue")
        self.assertEqual(resolve_engine_rescue_policy("always_rescue"), "always_rescue")
        self.assertEqual(resolve_engine_rescue_policy("calibrated_triage"), "calibrated_triage")
        self.assertEqual(resolve_engine_rescue_policy("react_orchestrator"), "react_orchestrator")
        self.assertEqual(canonical_rescue_policy_label("never_rescue"), "no_rescue")
        self.assertEqual(canonical_rescue_policy_label("no_rescue"), "no_rescue")
        with self.assertRaises(ValueError):
            resolve_engine_rescue_policy("invalid_policy")

    def test_golden_rescue_physical_extraction_vs_no_rescue_and_session_isolation(self) -> None:
        cfg_react = EpisodeConfig(
            scenario="golden_rescue",
            policy="cooperative",
            seed=7,
            steps=220,
            rescue_policy="react_orchestrator",
        )
        rec_react = run_canonical_episode(cfg_react, rescue_policy_override="react_orchestrator")
        self.assertTrue(rec_react.success)
        self.assertEqual(rec_react.rescue_commanded_count, 1)
        self.assertTrue(rec_react.physical_latch_engaged)
        self.assertEqual(rec_react.physical_extractions_verified, 1)
        self.assertGreaterEqual(rec_react.max_victim_extraction_displacement_m, 0.18)
        self.assertGreaterEqual(rec_react.independent_mobility_resumed_count, 1)
        self.assertEqual(rec_react.viable_seed_placements, 2)
        self.assertEqual(rec_react.post_rescue_viable_placements, 2)
        self.assertEqual(rec_react.currently_bogged_at_end, ())

        # Run no_rescue immediately afterward to verify fresh session isolation (Issue #71)
        cfg_no = EpisodeConfig(
            scenario="golden_rescue",
            policy="cooperative",
            seed=7,
            steps=220,
            rescue_policy="no_rescue",
        )
        rec_no = run_canonical_episode(cfg_no, rescue_policy_override="no_rescue")
        self.assertFalse(rec_no.success)
        self.assertEqual(rec_no.failure_reason, "recoverable_planter_abandoned")
        self.assertEqual(rec_no.rescue_commanded_count, 0)
        self.assertEqual(rec_no.physical_extractions_verified, 0)
        self.assertEqual(rec_no.viable_seed_placements, 0)
        self.assertIn("planter", rec_no.currently_bogged_at_end)

    def test_high_risk_abstain_distinguishes_commanded_rescue_from_physical_extraction(self) -> None:
        cfg_always = EpisodeConfig(
            scenario="high_risk_abstain",
            policy="cooperative",
            seed=7,
            steps=220,
            rescue_policy="always_rescue",
        )
        rec_always = run_canonical_episode(cfg_always, rescue_policy_override="always_rescue")
        # always_rescue commands a rescue, but helper gets bogged in secondary hazard (0 extractions)
        self.assertEqual(rec_always.rescue_commanded_count, 1)
        self.assertEqual(rec_always.physical_extractions_verified, 0)
        self.assertEqual(rec_always.commanded_without_physical_extraction, 1)
        self.assertGreaterEqual(rec_always.secondary_casualties, 1)
        self.assertFalse(rec_always.success)

        cfg_react = EpisodeConfig(
            scenario="high_risk_abstain",
            policy="cooperative",
            seed=7,
            steps=220,
            rescue_policy="react_orchestrator",
        )
        rec_react = run_canonical_episode(cfg_react, rescue_policy_override="react_orchestrator")
        self.assertEqual(rec_react.rescue_commanded_count, 0)
        self.assertEqual(rec_react.secondary_casualties, 0)
        self.assertEqual(rec_react.viable_seed_placements, 2)
        self.assertTrue(rec_react.success)

    def test_ablation1_react_orchestrator_vs_calibrated_triage_on_severe_mud_and_retries(self) -> None:
        # On randomized seed 5 (mud_severity='severe', planter bogged in firmness < 0.18):
        # react_orchestrator probes perimeter and vetoes rescue per OperatorPolicy,
        # whereas calibrated_triage blindly dispatches helper into severe mud.
        cfg_react = EpisodeConfig(
            scenario="randomized",
            policy="cooperative",
            seed=5,
            steps=220,
            rescue_policy="react_orchestrator",
        )
        rec_react = run_canonical_episode(cfg_react, rescue_policy_override="react_orchestrator")
        self.assertEqual(rec_react.mud_severity, "severe")
        self.assertGreater(rec_react.policy_veto_count, 0)
        self.assertEqual(rec_react.rescue_commanded_count, 0)
        self.assertEqual(rec_react.unsafe_severe_mud_dispatches, 0)
        self.assertTrue(rec_react.success)

        cfg_cal = EpisodeConfig(
            scenario="randomized",
            policy="cooperative",
            seed=5,
            steps=220,
            rescue_policy="calibrated_triage",
        )
        rec_cal = run_canonical_episode(cfg_cal, rescue_policy_override="calibrated_triage")
        self.assertEqual(rec_cal.mud_severity, "severe")
        self.assertEqual(rec_cal.rescue_commanded_count, 1)
        self.assertEqual(rec_cal.unsafe_severe_mud_dispatches, 1)
        self.assertFalse(rec_cal.success)
        self.assertEqual(rec_cal.failure_reason, "unsafe_severe_mud_dispatch")

        # Verify multi-attempt retry progression and incident-scoped reset
        retry_res = run_multi_attempt_retry_ablation()
        attempts = retry_res["attempts_on_same_incident"]
        self.assertEqual(attempts[0]["chosen_technique"], "dynamic_yaw_shear_and_pitch_pump")
        self.assertEqual(attempts[1]["chosen_technique"], "direct_pry_and_pull")
        self.assertNotEqual(attempts[0]["chosen_technique"], attempts[1]["chosen_technique"])
        self.assertEqual(
            retry_res["new_incident_after_reset"]["chosen_technique"],
            "dynamic_yaw_shear_and_pitch_pump",
        )

    def test_ablation2_peer_gossip_comms_normal_vs_isolated_and_single_rover(self) -> None:
        rec_normal = run_canonical_episode(
            EpisodeConfig(scenario="bad_soil", policy="mycelial", comms="normal", seed=7, steps=140),
            policy_label="mycelial_normal",
        )
        rec_isolated = run_canonical_episode(
            EpisodeConfig(scenario="bad_soil", policy="mycelial", comms="isolated", seed=7, steps=140),
            policy_label="mycelial_isolated",
        )
        rec_single = run_canonical_episode(
            EpisodeConfig(
                scenario="bad_soil",
                policy="single_rover",
                comms="normal",
                seed=7,
                steps=140,
                single_rover_only=True,
            ),
            policy_label="single_rover",
        )
        self.assertGreater(rec_normal.gossip_packets_sent, 0)
        self.assertEqual(rec_isolated.gossip_packets_sent, 0)
        self.assertEqual(rec_isolated.gossip_bytes_sent, 0)
        self.assertGreater(rec_isolated.map_divergence_l1, rec_normal.map_divergence_l1)
        self.assertEqual(rec_normal.viable_seed_placements, 2)
        self.assertLess(rec_single.viable_seed_placements, rec_normal.viable_seed_placements)

    def test_ablation3_residual_learner_and_schema_evaluator_compatibility(self) -> None:
        learner_res = run_residual_learner_ablation_study()
        self.assertTrue(learner_res["trajectories_identical_in_canonical_planner"])
        cf = learner_res["counterfactual_select_technique_adaptation"]
        self.assertNotEqual(
            cf["initial_selected_technique"],
            cf["after_stall_selected_technique"],
        )

        # Verify strict schemas/episode.schema.json compatibility with scripts/evaluate.py
        import subprocess
        import sys

        rec_cand = run_canonical_episode(
            EpisodeConfig(scenario="golden_rescue", policy="cooperative", seed=7, steps=220, rescue_policy="react_orchestrator")
        )
        rec_base = run_canonical_episode(
            EpisodeConfig(scenario="golden_rescue", policy="cooperative", seed=7, steps=220, rescue_policy="no_rescue")
        )
        with tempfile.TemporaryDirectory() as td:
            cand_p = Path(td) / "cand.jsonl"
            base_p = Path(td) / "base.jsonl"
            out_p = Path(td) / "eval.json"
            cand_p.write_text(json.dumps(rec_cand.to_schema_record()) + "\n", encoding="utf-8")
            base_p.write_text(json.dumps(rec_base.to_schema_record()) + "\n", encoding="utf-8")
            proc = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "scripts" / "evaluate.py"),
                    "--candidate",
                    str(cand_p),
                    "--baseline",
                    str(base_p),
                    "--out",
                    str(out_p),
                ],
                capture_output=True,
                text=True,
                check=True,
            )
            parsed = json.loads(proc.stdout)
            self.assertEqual(parsed["paired_comparison"]["candidate_only_successes"], 1)
            self.assertEqual(parsed["paired_comparison"]["baseline_only_successes"], 0)


if __name__ == "__main__":
    unittest.main()
