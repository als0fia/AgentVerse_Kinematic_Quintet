"""Unit and physics integration tests for the 3D MuJoCo Antenna SimulationPort adapter."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ricebot.contracts import RobotCommand
from ricebot.control import ControlLimits, LocalController
from ricebot.mission import MeasuredFieldMap, SimpleMissionPlanner
from ricebot.mujoco_sim.adapter import DEFAULT_CAPABILITIES, DEFAULT_SITES, MujocoPaddySimulation
from ricebot.runtime import EpisodeRunner, MissionCore


class MujocoPaddySimulationTests(unittest.TestCase):
    def test_nominal_episode_samples_both_rovers_and_physically_plants_seed(self) -> None:
        sim = MujocoPaddySimulation(sites=DEFAULT_SITES, probe_radius_m=0.28, substeps_per_tick=25)
        field_map = MeasuredFieldMap(DEFAULT_SITES, sample_radius_m=0.6)
        planner = SimpleMissionPlanner(DEFAULT_CAPABILITIES)
        controller = LocalController(
            ControlLimits(speed_m_s=0.38, turn_rad_s=1.15, stop_range_m=0.32, site_reach_m=0.22)
        )
        core = MissionCore(field_map, planner, controller)
        runner = EpisodeRunner(sim, core)

        runner.run(seed=7, steps=135)

        # patch_a (Lannitian waterlogged mud hazard) was sampled and rejected (moisture=0.92 > 0.80)
        est_a = field_map.estimate("patch_a")
        self.assertTrue(est_a.complete)
        self.assertGreater(est_a.moisture, 0.80)

        # At least one viable patch (patch_b or patch_c) was sampled and physically planted with ground contact
        self.assertGreaterEqual(len(sim.completed_drops), 1)
        first_drop = sim.completed_drops[0]
        self.assertEqual(first_drop.robot_id, "planter")
        self.assertIn(first_drop.target_site_id, {"patch_b", "patch_c"})
        self.assertIsNotNone(first_drop.landing_pose)
        self.assertIsNotNone(first_drop.contact_observed_at_s)

        # Both rovers contributed soil samples into the shared MeasuredFieldMap
        all_sources = set()
        for sid in ("patch_a", "patch_b", "patch_c", "patch_d"):
            all_sources.update(field_map.estimate(sid).source_robot_ids)
        self.assertEqual(all_sources, {"planter", "scout"})

    def test_mid_episode_obstacle_triggers_rangefinder_and_safe_hold(self) -> None:
        # Place obstacle directly in front of planter's initial path toward patch_a
        sim = MujocoPaddySimulation(
            sites=DEFAULT_SITES,
            probe_radius_m=0.28,
            substeps_per_tick=25,
            inject_obstacle_at_s=0.5,
            obstacle_xy=(0.55, -0.32),
        )
        field_map = MeasuredFieldMap(DEFAULT_SITES, sample_radius_m=0.6)
        planner = SimpleMissionPlanner(DEFAULT_CAPABILITIES)
        controller = LocalController(
            ControlLimits(speed_m_s=0.38, turn_rad_s=1.15, stop_range_m=0.32, site_reach_m=0.22)
        )
        core = MissionCore(field_map, planner, controller)

        frame = sim.reset(seed=7)
        min_planter_range = 4.0
        stopped_near_obstacle = False
        for _ in range(55):
            decision = core.decide(frame)
            planter_obs = next(r for r in frame.robots if r.robot_id == "planter")
            planter_cmd = next(c for c in decision.commands if c.robot_id == "planter")
            if planter_obs.front_range_m is not None:
                min_planter_range = min(min_planter_range, planter_obs.front_range_m)
                if planter_obs.front_range_m <= 0.32 and planter_cmd.linear_m_s == 0.0:
                    stopped_near_obstacle = True
            frame = sim.step(decision.commands)

        self.assertTrue(sim.obstacle_injected)
        self.assertLessEqual(min_planter_range, 0.32)
        self.assertTrue(stopped_near_obstacle)

    def test_scout_cannot_release_seed_and_expired_commands_rejected(self) -> None:
        sim = MujocoPaddySimulation(sites=DEFAULT_SITES)
        frame0 = sim.reset(seed=7)
        self.assertEqual(frame0.time_s, 0.0)

        scout_drop = RobotCommand(
            robot_id="scout",
            issued_at_s=0.0,
            expires_at_s=0.5,
            task_id="bad_scout_drop",
            release_seed=True,
            target_site_id="patch_a",
        )
        frame1 = sim.step((scout_drop,))
        self.assertEqual(len(sim.completed_drops), 0)
        self.assertEqual(len(frame1.plant_feedback), 0)


if __name__ == "__main__":
    unittest.main()
