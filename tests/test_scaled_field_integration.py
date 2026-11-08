"""Contract tests for physically expanded Antenna fields and acting policy selection."""
from __future__ import annotations

from math import hypot
import unittest

from ricebot.agent import DecentralizedMycelialRuntime
from ricebot.runtime import EpisodeConfig, create_mujoco_episode_session
from ricebot.world import build_field_layout, generate_hidden_soil_truth


class ScaledFieldIntegrationTest(unittest.TestCase):
    def test_generated_sites_and_soil_are_seeded_and_reproducible(self) -> None:
        base = build_field_layout(1.0, seed=7, scenario="nominal")
        large = build_field_layout(1.6, seed=7, scenario="randomized")
        copy = build_field_layout(1.6, seed=7, scenario="randomized")
        alternate = build_field_layout(1.6, seed=8, scenario="randomized")
        self.assertEqual(len(base.sites), 6)
        self.assertGreaterEqual(len(large.sites), 12)
        self.assertEqual(large, copy)
        self.assertNotEqual(
            [(s.site_id, s.center.x_m, s.center.y_m) for s in large.sites],
            [(s.site_id, s.center.x_m, s.center.y_m) for s in alternate.sites],
        )
        self.assertGreater(large.x_bounds[1] - large.x_bounds[0],
                           base.x_bounds[1] - base.x_bounds[0])
        truth = generate_hidden_soil_truth("randomized", 7, large.sites)
        self.assertEqual(set(truth), {s.site_id for s in large.sites if s.permitted})
        self.assertEqual(truth, generate_hidden_soil_truth("randomized", 7, large.sites))

    def test_large_field_has_real_geometry_and_multiple_capable_rovers(self) -> None:
        cfg = EpisodeConfig(scenario="nominal", policy="mycelial",
                            seed=7, steps=4, num_rovers=4, map_scale=1.6)
        session = create_mujoco_episode_session(cfg)
        frame = session.reset(7)
        sim = session.sim
        self.assertEqual(len(frame.robots), 4)
        self.assertGreater(len(sim.sites), 6)
        self.assertIsInstance(session.policy, DecentralizedMycelialRuntime)
        self.assertTrue(all(spec.can_sample and spec.can_plant
                            for spec in sim.rover_specs))
        self.assertTrue(all(spec.initial_seeds == 2 for spec in sim.rover_specs))
        self.assertAlmostEqual(float(sim.model.geom("bund_north").size[0]), 1.85 * 1.6, places=3)
        self.assertAlmostEqual(float(sim.model.geom("bund_north").pos[1]), 1.05 * 1.6, places=3)
        positions = [r.pose for r in frame.robots]
        for i, a in enumerate(positions):
            for b in positions[i + 1:]:
                self.assertGreater(hypot(a.x_m - b.x_m, a.y_m - b.y_m), 0.40)
        for _ in range(4):
            session.step_once()

    def test_randomized_expanded_field_uses_requested_mycelial_controller(self) -> None:
        cfg = EpisodeConfig(scenario="randomized", policy="mycelial",
                            seed=3, steps=3, num_rovers=3, map_scale=1.6,
                            active_mycelial_in_rescue=True, gossip_delay_s=0.8)
        session = create_mujoco_episode_session(cfg)
        self.assertIsInstance(session.policy, DecentralizedMycelialRuntime)
        self.assertEqual(session.policy.gossip_delay_s, 0.8)
        self.assertEqual(len(session.reset(3).robots), 3)
        for _ in range(3):
            session.step_once()


if __name__ == "__main__":
    unittest.main()
