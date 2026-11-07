"""Unit tests verifying the strict architectural split between `src/ricebot/world/` and `src/ricebot/robot/`."""
from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ricebot.contracts import Pose2D, SensorKind
from ricebot.robot import (
    DEFAULT_CAPABILITIES,
    PLANTER_SPEC,
    SCOUT_SPEC,
    build_rovers_and_seeds_xml,
    compute_4wheel_skid_steer,
)
from ricebot.world import (
    DEFAULT_SITES,
    PaddyTerraceWorld,
    build_world_terrain_xml,
)


class WorldModuleTest(unittest.TestCase):
    def test_world_emits_samples_only_inside_probe_radius_and_once_per_robot_site(self) -> None:
        world = PaddyTerraceWorld(sites=DEFAULT_SITES, probe_radius_m=0.28)
        world.reset(7)

        # Far from all patches -> empty
        far_samples = world.sample_local_soil("scout", Pose2D(0.10, -0.55, 0.0), 0.1)
        self.assertEqual(far_samples, ())

        # Inside protected crop patch (permitted=False) -> empty
        crop_samples = world.sample_local_soil("scout", Pose2D(0.85, 0.45, 0.0), 0.2)
        self.assertEqual(crop_samples, ())

        # Inside patch_a (0.85, -0.50) -> emits 3 valid samples (moisture, firmness, stand_gap)
        patch_a_samples = world.sample_local_soil("scout", Pose2D(0.84, -0.49, 0.0), 0.3)
        self.assertEqual(len(patch_a_samples), 3)
        by_kind = {s.kind: s.value for s in patch_a_samples}
        self.assertAlmostEqual(by_kind[SensorKind.MOISTURE], 0.92)
        self.assertAlmostEqual(by_kind[SensorKind.FIRMNESS], 0.16)

        # Second call by same rover at same site in same episode returns () to avoid duplicate sample_id
        repeat_samples = world.sample_local_soil("scout", Pose2D(0.85, -0.50, 0.0), 0.4)
        self.assertEqual(repeat_samples, ())

        # Other rover (planter) can still sample patch_a independently
        planter_samples = world.sample_local_soil("planter", Pose2D(0.85, -0.50, 0.0), 0.5)
        self.assertEqual(len(planter_samples), 3)

        # Reset clears episode state
        world.reset(8)
        after_reset = world.sample_local_soil("scout", Pose2D(0.85, -0.50, 0.0), 0.1)
        self.assertEqual(len(after_reset), 3)

    def test_world_terrain_xml_contains_only_world_elements(self) -> None:
        xml = build_world_terrain_xml(obstacle_xy=(0.75, -0.25))
        self.assertIn('geom name="field_ground"', xml)
        self.assertIn('geom name="bund_north"', xml)
        self.assertIn('geom name="bed_patch_a"', xml)
        self.assertIn('body name="mid_run_obstacle"', xml)
        # Must NOT contain rover chassis or wheel joints
        self.assertNotIn("planter_body", xml)
        self.assertNotIn("scout_body", xml)


class RobotModuleTest(unittest.TestCase):
    def test_rover_specs_and_capabilities(self) -> None:
        self.assertTrue(PLANTER_SPEC.can_plant)
        self.assertTrue(PLANTER_SPEC.can_sample)
        self.assertEqual(PLANTER_SPEC.initial_seeds, 2)
        self.assertFalse(SCOUT_SPEC.can_plant)
        self.assertTrue(SCOUT_SPEC.can_sample)
        self.assertEqual(SCOUT_SPEC.initial_seeds, 0)
        self.assertEqual(len(DEFAULT_CAPABILITIES), 2)

    def test_compute_4wheel_skid_steer_kinematics(self) -> None:
        # Zero command -> zero wheel speeds
        w0 = compute_4wheel_skid_steer(PLANTER_SPEC, 0.0, 0.0)
        self.assertEqual(w0, (0.0, 0.0, 0.0, 0.0))

        # Pure forward command -> all 4 wheels positive and equal
        wf = compute_4wheel_skid_steer(PLANTER_SPEC, 0.29, 0.0)
        self.assertGreater(wf[0], 0.0)
        self.assertAlmostEqual(wf[0], wf[1])
        self.assertAlmostEqual(wf[0], wf[2])
        self.assertAlmostEqual(wf[1], wf[3])

        # Pure left spot turn (positive angular_rad_s) -> right wheels faster than left wheels
        wt = compute_4wheel_skid_steer(SCOUT_SPEC, 0.0, 1.0)
        self.assertLess(wt[0], 0.0)     # FL backward
        self.assertGreater(wt[1], 0.0)  # FR forward

        # Extreme command clips cleanly to max_wheel_rad_s
        wc = compute_4wheel_skid_steer(PLANTER_SPEC, 10.0, 10.0)
        for w in wc:
            self.assertLessEqual(abs(w), PLANTER_SPEC.max_wheel_rad_s)

    def test_rovers_xml_contains_only_robot_and_seed_elements(self) -> None:
        bodies_xml, actuators_xml, sensors_xml = build_rovers_and_seeds_xml()
        self.assertIn('body name="planter_body"', bodies_xml)
        self.assertIn('body name="scout_body"', bodies_xml)
        self.assertIn('body name="seed_proxy_0"', bodies_xml)
        self.assertIn('velocity name="planter_act_fl"', actuators_xml)
        self.assertIn('rangefinder name="planter_range_c"', sensors_xml)
        # Must NOT contain world terrain beds or bunds
        self.assertNotIn("bund_north", bodies_xml)
        self.assertNotIn("bed_patch_a", bodies_xml)


if __name__ == "__main__":
    unittest.main()
