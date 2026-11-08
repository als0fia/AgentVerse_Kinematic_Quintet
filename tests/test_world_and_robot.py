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
    detect_ramp_and_compute_visual_servo,
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
        for suffix in ("a", "b", "c", "d", "e", "f"):
            self.assertIn(f'geom name="bed_patch_{suffix}"', xml)
            self.assertIn(f'geom name="puddle_patch_{suffix}"', xml)
        self.assertIn('body name="mid_run_obstacle"', xml)
        # Must NOT contain rover chassis or wheel joints
        self.assertNotIn("planter_body", xml)
        self.assertNotIn("scout_body", xml)

    def test_dynamic_3d_patch_puddles_across_seeds_and_4wheel_mud_sinkage_tilt(self) -> None:
        from ricebot.mujoco_sim.adapter import MujocoPaddySimulation

        sim = MujocoPaddySimulation(
            sites=DEFAULT_SITES,
            scenario="shifted_moisture",
            initial_poses={"planter": Pose2D(0.85, -0.50, 0.25), "scout": Pose2D(1.60, -0.50, -0.15)},
        )
        self.assertTrue(sim._hfield_dirty)

        # Verify field_ground, paddy_relief_mesh, bed_patch_a..f, and puddle_patch_a..f are in group 2
        for gname in (
            "field_ground",
            "paddy_relief_mesh",
            *[f"bed_patch_{s}" for s in ("a", "b", "c", "d", "e", "f")],
            *[f"puddle_patch_{s}" for s in ("a", "b", "c", "d", "e", "f")],
        ):
            gid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_GEOM, gname)
            self.assertGreaterEqual(gid, 0, f"Missing geom {gname}")
            self.assertEqual(int(sim.model.geom_group[gid]), 2, f"Expected group=2 for {gname}")

        # Reset across two seeds with different hidden soil truth and verify 3D beds/puddles update dynamically
        sim.reset(2)
        mud_set_seed2: set[str] = set()
        for suffix in ("a", "b", "c", "d"):
            pid = f"patch_{suffix}"
            bed_gid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_GEOM, f"bed_patch_{suffix}")
            puddle_gid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_GEOM, f"puddle_patch_{suffix}")
            is_mud = sim.world._hidden_site_truth[pid][SensorKind.MOISTURE] > 0.80
            if is_mud:
                mud_set_seed2.add(pid)
                self.assertAlmostEqual(float(sim.model.geom_rgba[bed_gid, 0]), 0.20, places=2)
                self.assertAlmostEqual(float(sim.model.geom_pos[puddle_gid, 2]), 0.0022, places=4)
                self.assertAlmostEqual(float(sim.model.geom_rgba[puddle_gid, 3]), 0.68, places=2)
            else:
                self.assertAlmostEqual(float(sim.model.geom_rgba[bed_gid, 0]), 0.46, places=2)
                self.assertAlmostEqual(float(sim.model.geom_pos[puddle_gid, 2]), -1.0, places=2)
                self.assertAlmostEqual(float(sim.model.geom_rgba[puddle_gid, 3]), 0.0, places=2)

        sim.reset(5)
        mud_set_seed5 = {
            f"patch_{s}"
            for s in ("a", "b", "c", "d")
            if float(
                sim.model.geom_rgba[
                    mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_GEOM, f"puddle_patch_{s}"), 3
                ]
            )
            > 0.5
        }
        self.assertNotEqual(mud_set_seed2, mud_set_seed5)

        # Verify 4-wheel mud sinkage (z < 0), chassis qpos[z] dip, and non-zero pitch_rad / roll_rad when bogged
        sim_bog = MujocoPaddySimulation(
            sites=DEFAULT_SITES,
            scenario="nominal",
            initial_poses={"planter": Pose2D(0.85, -0.50, 0.20), "scout": Pose2D(1.60, -0.50, 0.0)},
        )
        sim_bog.reset(7)
        sim_bog.inject_mud_fault("planter")
        frame = sim_bog.step(())
        planter_obs = next(r for r in frame.robots if r.robot_id == "planter")
        w_heights = sim_bog.latest_wheel_heights["planter"]
        self.assertEqual(len(w_heights), 4)
        self.assertTrue(all(zw < 0.0 for zw in w_heights))
        planter_z = float(sim_bog.data.qpos[sim_bog._planter_qpos_adr + 2])
        self.assertLess(planter_z, 0.048)
        self.assertGreaterEqual(planter_z, 0.036)
        self.assertNotEqual(planter_obs.pitch_rad, 0.0)
        self.assertNotEqual(planter_obs.roll_rad, 0.0)
        self.assertLessEqual(abs(planter_obs.pitch_rad), 0.22)
        self.assertLessEqual(abs(planter_obs.roll_rad), 0.22)


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
            self.assertIn(f'geom name="{rid}_battery_pack"', bodies_xml)
            self.assertIn(f'geom name="{rid}_solar_deck"', bodies_xml)
            self.assertIn(f'geom name="{rid}_compute_pack"', bodies_xml)
            self.assertIn(f'geom name="{rid}_uwb_transceiver"', bodies_xml)
            self.assertIn(f'geom name="{rid}_imu_board"', bodies_xml)
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
        for _ in range(140):
            data.ctrl[0:4] = w_fwd
            data.ctrl[latch_act_id] = 0.55  # Command cam-latch hook closed on near-contact (<= 22 mm)
            mujoco.mj_step(model, data)
            if update_rigid_dock_latch(model, data, rescuer_id="planter", target_id="scout", capture_radius_m=0.022):
                locked = True
                break

        self.assertTrue(locked, "Rescuer front docking cone failed to rigidly lock onto peer rear strike bar")

        # Step 2: Actuate bounded pitch-lift (+0.08 rad) and reverse to tow peer backward while grounded
        w_rev = compute_4wheel_skid_steer(PLANTER_SPEC, -0.35, 0.0)
        max_efc_force = 0.0
        max_chassis_z = 0.0
        for _ in range(110):
            data.ctrl[0:4] = w_rev
            data.ctrl[4:8] = w_rev
            data.ctrl[latch_act_id] = 0.55
            data.ctrl[pitch_act_id] = 0.08
            update_rigid_dock_latch(model, data, rescuer_id="planter", target_id="scout", capture_radius_m=0.022)
            mujoco.mj_step(model, data)
            if len(data.efc_force) > 0:
                max_efc_force = max(max_efc_force, float(abs(data.efc_force).max()))
            max_chassis_z = max(
                max_chassis_z,
                float(data.sensor("planter_pos").data[2]),
                float(data.sensor("scout_pos").data[2]),
            )

        final_victim_x = float(data.sensor("scout_pos").data[0])
        # Verify the stranded peer was physically pulled backward while staying grounded (< 0.075 m) and without >500 N snap
        self.assertLess(final_victim_x, initial_victim_x - 0.08)
        self.assertLess(max_chassis_z, 0.075)
        self.assertLess(max_efc_force, 500.0)

    def test_ego_camera_ramp_vision_detection_and_visual_servoing(self) -> None:
        """Verify RGB-D ramp detection, bearing error extraction, and closed-loop visual servoing commands."""
        import numpy as np

        # Synthetic 120x160 RGB-D frame with an earthen ramp shifted to the left of image center (u in [20, 60])
        rgb = np.full((120, 160, 3), 30, dtype=np.uint8)
        rgb[40:105, 20:60, 0] = 210  # Warm terracotta/amber R
        rgb[40:105, 20:60, 1] = 135  # G
        rgb[40:105, 20:60, 2] = 45   # B
        depth = np.full((120, 160), 1.2, dtype=np.float32)
        for row in range(40, 105):
            depth[row, 20:60] = 0.95 - 0.006 * (row - 40)

        det = detect_ramp_and_compute_visual_servo(rgb, depth, current_pitch_rad=0.0, current_z_m=0.058)
        self.assertTrue(det.detected)
        self.assertGreater(det.pixel_count, 500)
        self.assertLess(det.centroid_u_px, 80.0)
        self.assertGreater(det.bearing_error_rad, 0.14)
        self.assertGreater(det.steering_cmd_rad_s, 0.20)
        self.assertEqual(det.phase, "ORIENTING_TO_RAMP")

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
