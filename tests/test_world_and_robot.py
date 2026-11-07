"""Unit and MuJoCo physics tests verifying the strict architectural split between `src/ricebot/world/` and `src/ricebot/robot/`, homogeneous ant hardware parity, and rigid docking/locking rescue mechanics."""
from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest

import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ricebot.contracts import Pose2D, SensorKind
from ricebot.robot import (
    DEFAULT_CAPABILITIES,
    PLANTER_SPEC,
    SCOUT_SPEC,
    build_rigid_docking_rescue_mjcf,
    build_rovers_and_seeds_xml,
    compute_4wheel_skid_steer,
    export_robot_mjcf_assets,
    update_rigid_dock_latch,
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
    def test_rover_specs_and_homogeneous_hardware_parity(self) -> None:
        # Operational role flags for P0 baseline fixture
        self.assertTrue(PLANTER_SPEC.can_plant)
        self.assertTrue(PLANTER_SPEC.can_sample)
        self.assertEqual(PLANTER_SPEC.initial_seeds, 2)
        self.assertFalse(SCOUT_SPEC.can_plant)
        self.assertTrue(SCOUT_SPEC.can_sample)
        self.assertEqual(SCOUT_SPEC.initial_seeds, 0)
        self.assertEqual(len(DEFAULT_CAPABILITIES), 2)

        # Homogeneous physical hardware: identical wheel radius, track width, hull, and row fit (<300mm)
        self.assertEqual(PLANTER_SPEC.wheel_radius_m, SCOUT_SPEC.wheel_radius_m)
        self.assertEqual(PLANTER_SPEC.half_track_m, SCOUT_SPEC.half_track_m)
        self.assertEqual(PLANTER_SPEC.hopper_drop_z_m, SCOUT_SPEC.hopper_drop_z_m)
        self.assertLess(PLANTER_SPEC.outer_width_m, 0.300)

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

    def test_rovers_xml_contains_homogeneous_work_and_rigid_dock_elements(self) -> None:
        bodies_xml, actuators_xml, sensors_xml = build_rovers_and_seeds_xml()
        for rid in ("planter", "scout"):
            self.assertIn(f'body name="{rid}_body"', bodies_xml)
            self.assertIn(f'camera name="{rid}_front_camera"', bodies_xml)
            self.assertIn(f'geom name="{rid}_cam_lens"', bodies_xml)
            self.assertIn(f'geom name="{rid}_hopper_outer"', bodies_xml)
            self.assertIn(f'geom name="{rid}_probe_boom"', bodies_xml)
            self.assertIn(f'body name="{rid}_dock_boom"', bodies_xml)
            self.assertIn(f'body name="{rid}_latch_jaw"', bodies_xml)
            self.assertIn(f'site name="{rid}_front_dock_site"', bodies_xml)
            self.assertIn(f'site name="{rid}_rear_dock_site"', bodies_xml)
            self.assertIn(f'position name="{rid}_dock_pitch_act"', actuators_xml)
            self.assertIn(f'position name="{rid}_latch_act"', actuators_xml)
            self.assertIn(f'position name="{rid}_spade_act"', actuators_xml)

        self.assertIn('body name="seed_proxy_0"', bodies_xml)
        self.assertIn('velocity name="planter_act_fl"', actuators_xml)
        self.assertIn('rangefinder name="planter_range_c"', sensors_xml)
        # Must NOT contain world terrain beds or bunds
        self.assertNotIn("bund_north", bodies_xml)
        self.assertNotIn("bed_patch_a", bodies_xml)

    def test_mujoco_rigid_docking_cam_latch_and_tandem_extraction(self) -> None:
        """Simulate two homogeneous ants docking rigidly, locking the cam-latch, pitch-lifting, and towing."""
        xml = build_rigid_docking_rescue_mjcf()
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)

        initial_victim_x = float(data.sensor("scout_pos").data[0])

        # Step 1: Rescuer (planter at x=0.40) drives forward to dock into stranded peer (scout at x=0.82)
        w_fwd = compute_4wheel_skid_steer(PLANTER_SPEC, 0.35, 0.0)
        latch_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "planter_latch_act")
        pitch_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "planter_dock_pitch_act")

        locked = False
        for _ in range(120):
            data.ctrl[0:4] = w_fwd
            data.ctrl[latch_act_id] = 0.55  # Command cam-latch hook closed on contact
            mujoco.mj_step(model, data)
            if update_rigid_dock_latch(model, data, rescuer_id="planter", target_id="scout"):
                locked = True
                break

        self.assertTrue(locked, "Rescuer front docking cone failed to rigidly lock onto peer rear strike bar")

        # Step 2: Actuate front docking boom pitch-lift (+0.35 rad) and reverse to tow peer backward
        w_rev = compute_4wheel_skid_steer(PLANTER_SPEC, -0.35, 0.0)
        for _ in range(90):
            data.ctrl[0:4] = w_rev
            data.ctrl[latch_act_id] = 0.55
            data.ctrl[pitch_act_id] = 0.35
            update_rigid_dock_latch(model, data, rescuer_id="planter", target_id="scout")
            mujoco.mj_step(model, data)

        final_victim_x = float(data.sensor("scout_pos").data[0])
        # Verify the stranded peer was physically pulled backward along the row by the rigid latch
        self.assertLess(final_victim_x, initial_victim_x - 0.08)

    def test_export_standalone_mjcf_assets(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            single_path, rescue_path = export_robot_mjcf_assets(tmpdir)
            self.assertTrue(single_path.is_file())
            self.assertTrue(rescue_path.is_file())
            # Verify both exported MJCF files compile cleanly in MuJoCo
            m1 = mujoco.MjModel.from_xml_path(str(single_path))
            m2 = mujoco.MjModel.from_xml_path(str(rescue_path))
            self.assertGreater(m1.nu, 4)
            self.assertGreater(m2.neq, 0)


if __name__ == "__main__":
    unittest.main()
