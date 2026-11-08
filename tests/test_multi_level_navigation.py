"""Validation tests for Multi-Level Chinese Terraced Rice Fields and Inter-Terrace Navigation.

Demonstrates and verifies:
- Test 1: Lower to upper terrace navigation (Terrace A -> Ramp AB -> Terrace B -> Ramp BC -> Terrace C)
- Test 2: Upper to lower terrace navigation (Terrace C -> Ramp BC -> Terrace B -> Ramp AB -> Terrace A)
- Test 3: Blocked crossing handling (Inaccessible ramp detected and aborted without wall penetration)
- Test 4: Invalid steep crossing rejection (Crossings exceeding robot max safe slope are rejected)
- Test 5: Collision and crop protection (Routes through safe corridors without traversing rice crops)
- Continuous elevation and pitch articulation across ramps and terraces
"""
from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ricebot.world import (
    MultiLevelPathPlanner,
    MultiLevelTerraceEnvironment,
    NavigationWaypoint,
)


class MultiLevelTerraceNavigationTests(unittest.TestCase):
    def setUp(self) -> None:
        # Default environment: Terrace A (0.0m), Terrace B (0.5m), Terrace C (1.0m)
        # Ramp slope: 10.0 degrees (~2.84m horizontal run)
        # Robot limits: max safe slope 15.0 degrees, robot width 0.276m
        self.env = MultiLevelTerraceEnvironment(
            elevation_a=0.0,
            elevation_b=0.5,
            elevation_c=1.0,
            ramp_slope_deg=10.0,
            ramp_width_m=0.55,
            robot_max_slope_deg=15.0,
            robot_width_m=0.276,
        )
        self.planner = MultiLevelPathPlanner(self.env)

    def test_environment_configuration_and_geometry(self) -> None:
        """Verify terrace elevations, ramp slopes, widths, and horizontal runs."""
        self.assertEqual(self.env.terrace_a.elevation_m, 0.0)
        self.assertEqual(self.env.terrace_b.elevation_m, 0.5)
        self.assertEqual(self.env.terrace_c.elevation_m, 1.0)

        # Ramp AB geometry
        ramp_ab = self.env.ramp_ab
        self.assertAlmostEqual(ramp_ab.elevation_diff_m, 0.5, places=3)
        self.assertAlmostEqual(ramp_ab.slope_deg, 10.0, places=1)
        self.assertGreaterEqual(ramp_ab.horizontal_run_m, 2.80)
        self.assertLessEqual(ramp_ab.horizontal_run_m, 2.88)
        self.assertGreater(ramp_ab.width_m, 0.276)  # Clears robot width

        # Ramp BC geometry
        ramp_bc = self.env.ramp_bc
        self.assertAlmostEqual(ramp_bc.elevation_diff_m, 0.5, places=3)
        self.assertAlmostEqual(ramp_bc.slope_deg, 10.0, places=1)
        self.assertGreaterEqual(ramp_bc.horizontal_run_m, 2.80)

    def test_1_lower_to_upper_terrace_ascent(self) -> None:
        """Test 1: Place robot on Terrace A and navigate to Terrace C.

        Verifies:
        - Route traverses: Terrace A -> Ramp AB -> Terrace B -> Ramp BC -> Terrace C.
        - Robot climbs continuously through 0.0m -> 0.5m -> 1.0m elevation.
        - Approach speed is reduced at ramp entrances.
        - Body pitch pitches upward (~10 degrees) along the inclined ramps.
        """
        start_xy = (-1.50, -0.75)  # Terrace A access lane
        goal_xy = (10.00, 0.75)   # Terrace C destination

        waypoints = self.planner.plan_path(
            start_xy=start_xy,
            goal_xy=goal_xy,
            robot_max_slope_deg=15.0,
            robot_width_m=0.276,
        )
        self.assertGreater(len(waypoints), 5)

        # 1. Verify sequence of regions traversed
        zone_sequence = [wp.zone_tag for wp in waypoints]
        self.assertIn("terrace_a", zone_sequence)
        self.assertIn("ramp_ab", zone_sequence)
        self.assertIn("terrace_b", zone_sequence)
        self.assertIn("ramp_bc", zone_sequence)
        self.assertIn("terrace_c", zone_sequence)

        # Verify correct order: ramp_ab must appear before ramp_bc
        idx_ab = zone_sequence.index("ramp_ab")
        idx_b = zone_sequence.index("terrace_b")
        idx_bc = zone_sequence.index("ramp_bc")
        self.assertLess(idx_ab, idx_b)
        self.assertLess(idx_b, idx_bc)

        # 2. Verify elevation rises monotonically from 0.0m to 1.0m
        z_vals = [wp.z_m for wp in waypoints]
        self.assertAlmostEqual(z_vals[0], 0.0, places=2)
        self.assertAlmostEqual(z_vals[-1], 1.0, places=2)
        for i in range(len(z_vals) - 1):
            self.assertLessEqual(z_vals[i], z_vals[i + 1] + 1e-4)

        # 3. Verify pitch articulation along ramps
        pitch_vals = [wp.pitch_rad for wp in waypoints if wp.zone_tag in ("ramp_ab", "ramp_bc")]
        climbing_pitches = [p for p in pitch_vals if p > 0.05]
        self.assertTrue(len(climbing_pitches) > 0)
        # 10 degrees is ~0.1745 rad
        self.assertAlmostEqual(climbing_pitches[0], math.radians(10.0), delta=0.04)

        # 4. Verify reduced speed during ramp entry
        ramp_entry_speeds = [wp.speed_m_s for wp in waypoints if "Enter" in wp.comment]
        for spd in ramp_entry_speeds:
            self.assertLessEqual(spd, 0.18)

    def test_2_upper_to_lower_terrace_descent(self) -> None:
        """Test 2: Place robot on Terrace C and navigate down to Terrace A.

        Verifies:
        - Route traverses: Terrace C -> Ramp BC (descent) -> Terrace B -> Ramp AB (descent) -> Terrace A.
        - Continuous elevation descent from 1.0m down to 0.0m.
        - Body pitch pitches downward (-10 degrees) during ramp descent.
        """
        start_xy = (10.00, 0.75)   # Terrace C
        goal_xy = (-1.50, -0.75)  # Terrace A

        waypoints = self.planner.plan_path(
            start_xy=start_xy,
            goal_xy=goal_xy,
            robot_max_slope_deg=15.0,
            robot_width_m=0.276,
        )
        self.assertGreater(len(waypoints), 5)

        zone_sequence = [wp.zone_tag for wp in waypoints]
        self.assertIn("terrace_c", zone_sequence)
        self.assertIn("ramp_bc", zone_sequence)
        self.assertIn("terrace_b", zone_sequence)
        self.assertIn("ramp_ab", zone_sequence)
        self.assertIn("terrace_a", zone_sequence)

        idx_bc = zone_sequence.index("ramp_bc")
        idx_b = zone_sequence.index("terrace_b")
        idx_ab = zone_sequence.index("ramp_ab")
        self.assertLess(idx_bc, idx_b)
        self.assertLess(idx_b, idx_ab)

        # Verify elevation drops monotonically from 1.0m to 0.0m
        z_vals = [wp.z_m for wp in waypoints]
        self.assertAlmostEqual(z_vals[0], 1.0, places=2)
        self.assertAlmostEqual(z_vals[-1], 0.0, places=2)
        for i in range(len(z_vals) - 1):
            self.assertGreaterEqual(z_vals[i], z_vals[i + 1] - 1e-4)

        # Verify negative (downward) pitch on descent
        pitch_vals = [wp.pitch_rad for wp in waypoints if wp.zone_tag in ("ramp_ab", "ramp_bc")]
        descending_pitches = [p for p in pitch_vals if p < -0.05]
        self.assertTrue(len(descending_pitches) > 0)
        self.assertAlmostEqual(descending_pitches[0], -math.radians(10.0), delta=0.04)

    def test_3_blocked_crossing_detected_and_safely_aborted(self) -> None:
        """Test 3: Make Ramp AB inaccessible and request route from Terrace A to Terrace C.

        Verifies:
        - Planner recognizes that Ramp AB is blocked.
        - Does NOT attempt to penetrate the retaining wall.
        - Rejects route with an explicit, traceable error.
        """
        self.env.set_ramp_accessibility("ramp_ab", accessible=False)
        self.assertFalse(self.env.ramp_ab.is_accessible)

        with self.assertRaises(RuntimeError) as ctx:
            self.planner.plan_path(
                start_xy=(-1.50, -0.75),
                goal_xy=(10.00, 0.75),
                robot_max_slope_deg=15.0,
                robot_width_m=0.276,
            )
        self.assertIn("blocked", str(ctx.exception).lower())

        # Restore accessibility
        self.env.set_ramp_accessibility("ramp_ab", accessible=True)
        self.assertTrue(self.env.ramp_ab.is_accessible)

    def test_4_invalid_steep_crossing_rejected(self) -> None:
        """Test 4: Configure a ramp exceeding the robot's maximum safe slope (e.g. 25 deg > 15 deg).

        Verifies:
        - Navigation planner rejects crossings steeper than the robot's mobility limit.
        """
        # Configure steep environment with 25-degree ramp
        steep_env = MultiLevelTerraceEnvironment(
            elevation_a=0.0,
            elevation_b=0.5,
            elevation_c=1.0,
            ramp_slope_deg=25.0,  # Exceeds robot 15-degree limit!
            ramp_width_m=0.55,
            robot_max_slope_deg=15.0,
            robot_width_m=0.276,
        )
        steep_planner = MultiLevelPathPlanner(steep_env)
        goal_c = (steep_env.terrace_c.bounds_x[0] + 0.5, 0.75)

        with self.assertRaises(RuntimeError) as ctx:
            steep_planner.plan_path(
                start_xy=(-1.50, -0.75),
                goal_xy=goal_c,
                robot_max_slope_deg=15.0,  # Limit is 15 deg
                robot_width_m=0.276,
            )
        self.assertIn("exceeds robot limit", str(ctx.exception))

    def test_5_collision_and_crop_protection(self) -> None:
        """Test 5: Destination across a planted rice crop bed.

        Verifies:
        - Robot does not drive straight through protected rice seedlings or retaining walls.
        - Routes through designated inter-row or access corridors.
        """
        # On Terrace A: Cultivated rice bed is at [-2.60, -0.40], y in [0.10, 1.30]
        # Robot starts at south lane (-1.50, -0.75) and wants to reach north lane (-0.50, 1.40)
        start_xy = (-2.00, -0.75)
        goal_xy = (-0.50, 1.40)

        waypoints = self.planner.plan_path(start_xy, goal_xy)
        self.assertGreater(len(waypoints), 1)

        # Verify that no waypoint is inside the cultivated zone
        for wp in waypoints:
            in_crop = self.env.terrace_a.is_in_cultivated_zone(wp.x_m, wp.y_m)
            self.assertFalse(in_crop, f"Waypoint ({wp.x_m}, {wp.y_m}) penetrates cultivated rice crop!")

    def test_continuous_terrain_surface_elevation_and_pitch(self) -> None:
        """Verify continuous 3D surface elevation and pitch calculations."""
        # On Terrace A horizontal bed
        za, pa = self.env.get_elevation_and_pitch(-1.0, -0.5)
        self.assertEqual(za, 0.0)
        self.assertEqual(pa, 0.0)

        # On Terrace B horizontal bed
        zb, pb = self.env.get_elevation_and_pitch(4.0, 0.0)
        self.assertEqual(zb, 0.5)
        self.assertEqual(pb, 0.0)

        # On Terrace C horizontal bed
        zc, pc = self.env.get_elevation_and_pitch(9.5, 0.0)
        self.assertEqual(zc, 1.0)
        self.assertEqual(pc, 0.0)

        # Along Ramp AB corridor (x in [0, 2.84], y = -0.75)
        z_mid, p_mid = self.env.get_elevation_and_pitch(1.42, -0.75)
        self.assertGreater(z_mid, 0.15)
        self.assertLess(z_mid, 0.35)
        self.assertGreater(p_mid, 0.10)  # Nose up climbing pitch

    def test_1m_step_terraces_and_half_meter_robot_continuous_simulation(self) -> None:
        """Verify 1.0m terrace level differences (0.0m, 1.0m, 2.0m) with a 0.50m robot and continuous trajectory."""
        env_1m = MultiLevelTerraceEnvironment(
            elevation_a=0.0,
            elevation_b=1.0,
            elevation_c=2.0,
            ramp_slope_deg=12.0,
            ramp_width_m=0.75,
            robot_max_slope_deg=15.0,
            robot_width_m=0.38,
            robot_length_m=0.50,
        )
        planner_1m = MultiLevelPathPlanner(env_1m)

        self.assertEqual(env_1m.terrace_a.elevation_m, 0.0)
        self.assertEqual(env_1m.terrace_b.elevation_m, 1.0)
        self.assertEqual(env_1m.terrace_c.elevation_m, 2.0)
        self.assertEqual(env_1m.robot_length_m, 0.50)
        self.assertEqual(env_1m.robot_width_m, 0.38)
        self.assertAlmostEqual(env_1m.ramp_ab.elevation_diff_m, 1.0, places=3)
        self.assertAlmostEqual(env_1m.ramp_bc.elevation_diff_m, 1.0, places=3)

        start_a = (-1.50, -0.75)
        goal_c = (env_1m.terrace_c.bounds_x[0] + 1.50, 0.75)

        ascent_states = planner_1m.simulate_continuous_navigation(start_a, goal_c, dt_s=0.10)
        self.assertGreater(len(ascent_states), 50)
        self.assertAlmostEqual(ascent_states[0].z_m, 0.0, places=2)
        self.assertAlmostEqual(ascent_states[-1].z_m, 2.0, places=2)

        # Verify no vertical teleportation (max step dz <= 0.05m per 0.10s tick)
        for i in range(len(ascent_states) - 1):
            dz = abs(ascent_states[i + 1].z_m - ascent_states[i].z_m)
            self.assertLess(dz, 0.05)

        descent_states = planner_1m.simulate_continuous_navigation(goal_c, start_a, dt_s=0.10)
        self.assertGreater(len(descent_states), 50)
        self.assertAlmostEqual(descent_states[0].z_m, 2.0, places=2)
        self.assertAlmostEqual(descent_states[-1].z_m, 0.0, places=2)
        self.assertTrue(any(s.pitch_rad < -0.10 for s in descent_states))


if __name__ == "__main__":
    unittest.main()
