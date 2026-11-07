"""Unit tests verifying the sensor suite closed-loop control reflexes."""
from __future__ import annotations

import unittest

from ricebot.contracts import (
    Pose2D,
    RobotCapability,
    RobotObservation,
    Site,
    TaskIntent,
    TaskMode,
)
from ricebot.control import ControlLimits, LocalController


class SensorReflexTests(unittest.TestCase):
    def setUp(self) -> None:
        self.limits = ControlLimits(
            speed_m_s=0.30,
            turn_rad_s=1.0,
            stop_range_m=0.30,
            site_reach_m=0.20,
            max_pitch_roll_rad=0.35,  # 20 deg
            max_slip_ratio=0.60,
            min_peer_distance_m=1.0,
            kp_row=1.0,
        )
        self.controller = LocalController(self.limits)
        self.capability = RobotCapability("rover-1", can_sample=True, can_plant=True)
        self.sites = {
            "site-a": Site("site-a", Pose2D(2.0, 0.0, 0.0)),
        }
        self.task = TaskIntent(
            task_id="t1",
            robot_id="rover-1",
            mode=TaskMode.SAMPLE,
            issued_at_s=0.0,
            expires_at_s=1.0,
            target_site_id="site-a",
        )

    def test_imu_tip_over_halt(self) -> None:
        # Severe pitch breach (>20 deg / 0.35 rad) triggers immediate drive cut
        obs = RobotObservation(
            robot_id="rover-1",
            pose=Pose2D(0.0, 0.0, 0.0),
            time_s=0.1,
            front_range_m=2.0,
            seeds_remaining=2,
            pitch_rad=0.40,  # ~23 deg pitch tilt
            roll_rad=0.0,
        )
        cmd = self.controller.command(obs, self.task, self.sites, self.capability)
        self.assertEqual(cmd.linear_m_s, 0.0)
        self.assertEqual(cmd.angular_rad_s, 0.0)

    def test_bogged_down_safety_cutoff(self) -> None:
        # Mud sinkage / bogged status cuts drive to prevent digging trenches
        obs = RobotObservation(
            robot_id="rover-1",
            pose=Pose2D(0.0, 0.0, 0.0),
            time_s=0.1,
            front_range_m=2.0,
            seeds_remaining=2,
            is_bogged=True,
        )
        cmd = self.controller.command(obs, self.task, self.sites, self.capability)
        self.assertEqual(cmd.linear_m_s, 0.0)
        self.assertEqual(cmd.angular_rad_s, 0.0)

    def test_uwb_anti_collision_brake(self) -> None:
        # Peer distance < 1.0 m commands immediate brake
        obs = RobotObservation(
            robot_id="rover-1",
            pose=Pose2D(0.0, 0.0, 0.0),
            time_s=0.1,
            front_range_m=2.0,
            seeds_remaining=2,
            peer_distances=(("rover-2", 0.65),),  # 65 cm distance -> inside 1.0 m envelope
        )
        cmd = self.controller.command(obs, self.task, self.sites, self.capability)
        self.assertEqual(cmd.linear_m_s, 0.0)
        self.assertEqual(cmd.angular_rad_s, 0.0)

    def test_row_centering_steer(self) -> None:
        # Whisker / ToF row cross-track error adds corrective steering
        obs_centered = RobotObservation(
            robot_id="rover-1",
            pose=Pose2D(0.0, 0.0, 0.0),
            time_s=0.1,
            front_range_m=2.0,
            seeds_remaining=2,
            row_cross_track_m=0.0,
        )
        cmd_centered = self.controller.command(obs_centered, self.task, self.sites, self.capability)

        obs_drifted = RobotObservation(
            robot_id="rover-1",
            pose=Pose2D(0.0, 0.0, 0.0),
            time_s=0.1,
            front_range_m=2.0,
            seeds_remaining=2,
            row_cross_track_m=0.08,  # 8 cm right of row center
        )
        cmd_drifted = self.controller.command(obs_drifted, self.task, self.sites, self.capability)

        # Corrective steer added in direction of cross-track error
        self.assertGreater(cmd_drifted.angular_rad_s, cmd_centered.angular_rad_s)

    def test_anti_digging_traction_control(self) -> None:
        # Doppler radar reports low true velocity -> slip ratio > 0.6 -> throttle down
        obs_good_grip = RobotObservation(
            robot_id="rover-1",
            pose=Pose2D(0.0, 0.0, 0.0),
            time_s=0.1,
            front_range_m=2.0,
            seeds_remaining=2,
            true_speed_m_s=0.30,  # 100% grip
        )
        cmd_good = self.controller.command(obs_good_grip, self.task, self.sites, self.capability)

        obs_slipping = RobotObservation(
            robot_id="rover-1",
            pose=Pose2D(0.0, 0.0, 0.0),
            time_s=0.1,
            front_range_m=2.0,
            seeds_remaining=2,
            true_speed_m_s=0.05,  # Heavy slip in mud (true speed << commanded)
        )
        cmd_slipping = self.controller.command(obs_slipping, self.task, self.sites, self.capability)

        # Anti-digging throttle back
        self.assertLess(cmd_slipping.linear_m_s, cmd_good.linear_m_s)


if __name__ == "__main__":
    unittest.main()
