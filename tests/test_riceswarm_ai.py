from __future__ import annotations
import json
import unittest
from unittest.mock import Mock
import xml.etree.ElementTree as ET
from riceswarm_ai.ai_agent import AgentAdvisedRecoveryPolicy, GeminiRecoveryAdviser
from riceswarm_ai.core import (
    CropStage,
    CropState,
    Field,
    MudRiskTier,
    RecoveryPolicy,
    RecoveryWeights,
    RiceSwarmSimulation,
    Robot,
    Role,
    Status,
    Terrain,
    get_mud_risk_tier,
    shortest_path,
)
from riceswarm_ai.mujoco_bridge import build_mjcf


class SimulationTests(unittest.TestCase):
    def test_6_robot_swarm_initialisation_and_coverage(self):
        sim = RiceSwarmSimulation(introduce_mud=False)
        self.assertEqual(len(sim.robots), 6)
        for name, r in sim.robots.items():
            self.assertTrue(r.can_plant)
            self.assertTrue(r.can_weed)
            self.assertGreaterEqual(r.recovery_capacity, 3.0)

    def test_multi_step_cultivation_planting_and_weeding(self):
        sim = RiceSwarmSimulation(introduce_mud=False)
        result = sim.run(max_steps=200)
        self.assertGreater(result["planted"], 0)
        self.assertGreater(result["weeded"], 0)
        self.assertEqual(result["unsafe_crop_movements"], 0)

    def test_simultaneous_scanning_updates_belief_map(self):
        sim = RiceSwarmSimulation()
        init_confidence = sum(c.confidence >= 0.9 for c in sim.field.belief.values())
        sim.step()
        post_confidence = sum(c.confidence >= 0.9 for c in sim.field.belief.values())
        # All 6 robots scan simultaneously on every step
        self.assertGreater(post_confidence, init_confidence)

    def test_crop_protection_during_navigation(self):
        sim = RiceSwarmSimulation()
        sim.field.mark_planted((4, 5))
        self.assertEqual(sim.field.belief[(4, 5)].crop_state, CropState.PLANTED)
        path = shortest_path(sim.field, (4, 4), (4, 6))
        self.assertIsNotNone(path)
        self.assertNotIn((4, 5), path)

    def test_mud_immobilisation_notifies_peers_and_dispatches_best_responder(self):
        sim = RiceSwarmSimulation()
        # Trigger RS2 immobilisation
        sim.trigger_immobilisation("RS2")
        self.assertEqual(sim.robots["RS2"].status, Status.STUCK)
        d = sim.coordinator.active_rescue_decision
        self.assertIsNotNone(d)
        self.assertEqual(d.action, "cooperative_rescue")
        # Ensure a different peer from RS1..RS6 is selected as responder
        self.assertIn(d.selected_responder, [r for r in sim.robots if r != "RS2"])
        # Selected responder is adjacent to stuck robot
        self.assertIn(d.rescue_stance, list(sim.field.neighbors(sim.robots["RS2"].pos)))
        # Verify normalised components are in [0, 1]
        for comp in d.normalised_components["cooperative_rescue"].values():
            self.assertGreaterEqual(comp, 0.0)
            self.assertLessEqual(comp, 1.0)

    def test_full_mud_rescue_and_resumption(self):
        sim = RiceSwarmSimulation(introduce_mud=True)
        sim.trigger_immobilisation("RS2")
        out = sim.run(max_steps=350)
        self.assertGreaterEqual(out["successful_recoveries"], 1)
        self.assertEqual(out["unsafe_crop_movements"], 0)

    def test_weeding_priority_targets_weed_dense_planted_stands(self):
        sim = RiceSwarmSimulation()
        sim.field.mark_planted((2, 2))
        sim.field.mark_planted((4, 2))
        sim.field.weed_density[(2, 2)] = 0.20
        sim.field.weed_density[(4, 2)] = 0.90
        sim.field.observe((2, 2), tick=1)
        sim.field.observe((4, 2), tick=1)
        ranked = sim.coordinator.compute_priority_weeding_map()
        self.assertEqual(ranked[0][0], (4, 2))
        self.assertGreater(ranked[0][1], ranked[1][1])

    def test_mujoco_xml_validity_for_6_robots(self):
        xml_str = build_mjcf(Field())
        root = ET.fromstring(xml_str)
        bodies = {b.attrib.get("name") for b in root.findall(".//body")}
        expected = {f"robot_RS{i}" for i in range(1, 7)}
        self.assertEqual(bodies, expected)

    def test_gemini_invalid_action_rejected_safely(self):
        mock_adviser = Mock()
        mock_adviser.select.return_value = ("drive_through_crops", "shortcut")
        sim = RiceSwarmSimulation(policy=AgentAdvisedRecoveryPolicy(mock_adviser))
        sim.trigger_immobilisation("RS2")
        d = sim.coordinator.active_rescue_decision
        self.assertIsNotNone(d)
        self.assertEqual(d.action, "cooperative_rescue")
        self.assertIn("ValueError", sim.policy.last_agent_error)

    def test_two_section_staggered_calendar_phases(self):
        sim = RiceSwarmSimulation(start_day=-25)
        # Day -25: Section A is Nursery, Section B is Nursery (offset +20 means eff day -45)
        self.assertEqual(sim.field.sections["A"].calendar_offset, 0)
        self.assertEqual(sim.field.sections["B"].calendar_offset, 20)
        sim.advance_day()
        # Advance to Day 0: Section A is Transplanting, Section B is still Field Prep (eff day -20)
        while sim.current_day < 0:
            sim.advance_day()
        self.assertEqual(sim.field.sections["A"].current_stage, CropStage.TRANSPLANTING)
        self.assertEqual(sim.field.sections["B"].current_stage, CropStage.NURSERY_ESTABLISHMENT)

        # Advance to Day 20: Section A is Early Tillering, Section B reaches Transplanting
        while sim.current_day < 20:
            sim.advance_day()
        self.assertEqual(sim.field.sections["A"].current_stage, CropStage.EARLY_TILLERING)
        self.assertEqual(sim.field.sections["B"].current_stage, CropStage.TRANSPLANTING)

    def test_continuous_environmental_monitoring_and_tiers(self):
        self.assertEqual(get_mud_risk_tier(0.15), MudRiskTier.LOW)
        self.assertEqual(get_mud_risk_tier(0.45), MudRiskTier.MODERATE)
        self.assertEqual(get_mud_risk_tier(0.68), MudRiskTier.HIGH)
        self.assertEqual(get_mud_risk_tier(0.88), MudRiskTier.VERY_HIGH)

        sim = RiceSwarmSimulation(start_day=-8)
        rep = sim.advance_day()  # Day -7: periodic soil sample trigger
        self.assertTrue(rep.soil_sampled)
        self.assertIsNotNone(rep.mud_risk_tier)
        self.assertGreater(rep.avg_water_depth, 0.0)

    def test_pre_harvest_drainage_and_harvesting(self):
        sim = RiceSwarmSimulation(start_day=100)
        # Set plants in section A to mature
        for pt in sim.field.sections["A"].targets:
            sim.field.mark_planted(pt)
            sim.field.plants[pt].grain_maturity = 0.98
            sim.field.plants[pt].stage = CropStage.HARVEST_READY
        # Advance through pre-harvest drainage (Day 106-115)
        while sim.current_day < 118:
            sim.advance_day()
        # In harvest ready stage with drained water, coordinator should assign harvest
        harvest_candidates = sim.coordinator.compute_priority_harvesting_map()
        self.assertGreater(len(harvest_candidates), 0)
        # Run ticks to execute harvest
        sim.run(max_steps=100)
        self.assertGreater(len(sim.field.harvested), 0)
        self.assertEqual(sim.unsafe_moves, 0)


if __name__ == "__main__":
    unittest.main()
