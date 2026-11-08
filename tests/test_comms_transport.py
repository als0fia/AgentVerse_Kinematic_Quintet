"""Unit tests for peer-to-peer radio transport, ranging, and blackout simulation (`src/ricebot/world/comms.py`).

Verifies:
- Sub-1 GHz / UWB range limits (`comm_range_m`).
- Intermittent and complete RF blackout modes.
- Epistemic isolation: filtering peer observations so isolated nodes cannot see remote peer state.
"""
from __future__ import annotations

from pathlib import Path
import sys
import unittest

# Ensure src/ is on sys.path
_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT / "src"))

from ricebot.contracts import Pose2D, RobotObservation
from ricebot.world.comms import (
    ChannelStatus,
    PaddyP2PCommsChannel,
    PeerLinkStatus,
)


class CommsTransportTest(unittest.TestCase):
    def setUp(self) -> None:
        self.ch_normal = PaddyP2PCommsChannel(comm_range_m=1.2, mode="normal")
        self.ch_blackout = PaddyP2PCommsChannel(comm_range_m=1.2, mode="blackout")
        self.ch_intermittent = PaddyP2PCommsChannel(
            comm_range_m=1.2,
            mode="intermittent",
            intermittent_cycle_s=4.0,
            intermittent_blackout_fraction=0.605,
        )

    def test_in_range_and_out_of_range_links(self) -> None:
        p_origin = Pose2D(0.0, 0.0, 0.0)
        p_near = Pose2D(0.8, 0.0, 0.0)    # 0.8m <= 1.2m -> In range
        p_far = Pose2D(2.5, 0.0, 0.0)     # 2.5m > 1.2m -> Out of range

        link_near = self.ch_normal.evaluate_link("scout", p_near, "planter", p_origin, 1.0)
        self.assertTrue(link_near.in_range)
        self.assertEqual(link_near.status, ChannelStatus.CONNECTED)
        self.assertLess(link_near.packet_loss_rate, 0.25)
        self.assertGreater(link_near.snr_db, 10.0)

        link_far = self.ch_normal.evaluate_link("scout", p_far, "planter", p_origin, 1.0)
        self.assertFalse(link_far.in_range)
        self.assertEqual(link_far.status, ChannelStatus.BLACKOUT)
        self.assertEqual(link_far.packet_loss_rate, 1.0)

    def test_blackout_mode_suppresses_all_transmissions(self) -> None:
        p_origin = Pose2D(0.0, 0.0, 0.0)
        p_touching = Pose2D(0.2, 0.0, 0.0)  # Very close

        link = self.ch_blackout.evaluate_link("scout", p_touching, "planter", p_origin, 1.0)
        self.assertTrue(link.in_range)
        self.assertEqual(link.status, ChannelStatus.BLACKOUT)
        self.assertEqual(link.packet_loss_rate, 1.0)

    def test_intermittent_duty_cycle(self) -> None:
        p_origin = Pose2D(0.0, 0.0, 0.0)
        p_near = Pose2D(0.5, 0.0, 0.0)

        # In intermittent mode with cycle=4.0s and blackout_fraction=0.605:
        # t=1.0s -> phase = 1.0/4.0 = 0.25 < 0.605 -> Blackout active
        self.assertTrue(self.ch_intermittent.is_blackout_active(1.0))
        link_blocked = self.ch_intermittent.evaluate_link("scout", p_near, "planter", p_origin, 1.0)
        self.assertEqual(link_blocked.status, ChannelStatus.BLACKOUT)

        # t=3.0s -> phase = 3.0/4.0 = 0.75 > 0.605 -> Connected
        self.assertFalse(self.ch_intermittent.is_blackout_active(3.0))
        link_open = self.ch_intermittent.evaluate_link("scout", p_near, "planter", p_origin, 3.0)
        self.assertEqual(link_open.status, ChannelStatus.CONNECTED)

    def test_filter_peer_observations_epistemic_isolation(self) -> None:
        """Verify rovers never receive out-of-range or blacked-out peer observations (Audit M6, T7)."""
        obs_planter = RobotObservation(
            robot_id="planter",
            pose=Pose2D(0.0, 0.0, 0.0),
            time_s=1.0,
            front_range_m=1.5,
            seeds_remaining=2,
            true_speed_m_s=0.1,
            is_bogged=False,
        )
        obs_scout_near = RobotObservation(
            robot_id="scout",
            pose=Pose2D(0.6, 0.0, 0.0),
            time_s=1.0,
            front_range_m=2.0,
            seeds_remaining=0,
            true_speed_m_s=0.2,
            is_bogged=False,
        )
        obs_scout_far = RobotObservation(
            robot_id="scout",
            pose=Pose2D(3.0, 0.0, 0.0),
            time_s=1.0,
            front_range_m=2.0,
            seeds_remaining=0,
            true_speed_m_s=0.2,
            is_bogged=False,
        )

        # 1. Normal mode, peer near: Planter receives Scout observation
        received_near = self.ch_normal.filter_peer_observations_for_robot(
            query_robot_id="planter",
            query_pose=obs_planter.pose,
            all_robot_observations=[obs_planter, obs_scout_near],
            time_s=1.0,
        )
        self.assertEqual(len(received_near), 1)
        self.assertEqual(received_near[0].robot_id, "scout")

        # 2. Normal mode, peer far: Planter receives NOTHING (out of range)
        received_far = self.ch_normal.filter_peer_observations_for_robot(
            query_robot_id="planter",
            query_pose=obs_planter.pose,
            all_robot_observations=[obs_planter, obs_scout_far],
            time_s=1.0,
        )
        self.assertEqual(len(received_far), 0)

        # 3. Blackout mode, peer near: Planter receives NOTHING (blackout)
        received_blackout = self.ch_blackout.filter_peer_observations_for_robot(
            query_robot_id="planter",
            query_pose=obs_planter.pose,
            all_robot_observations=[obs_planter, obs_scout_near],
            time_s=1.0,
        )
        self.assertEqual(len(received_blackout), 0)


if __name__ == "__main__":
    unittest.main()
