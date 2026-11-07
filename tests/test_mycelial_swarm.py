"""Unit and MuJoCo physics tests for Policy B: Decentralised Mycelial SLAM, Gossip & Task Negotiation."""
from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ricebot.agent import DecentralizedMycelialRuntime
from ricebot.contracts import (
    ObservationFrame,
    Pose2D,
    RobotObservation,
    SensorKind,
    SoilSample,
)
from ricebot.mujoco_sim import MujocoPaddySimulation
from ricebot.robot import DEFAULT_CAPABILITIES
from ricebot.runtime import EpisodeRunner
from ricebot.world import DEFAULT_SITES, UNEXPECTED_BAD_SOIL_TRUTH


class MycelialSwarmUnitTests(unittest.TestCase):
    def test_robots_maintain_separate_local_maps_when_isolated_and_converge_when_in_range(self) -> None:
        # 1. Under isolated comms, Scout's discovery stays strictly in Scout's private map
        isolated_runtime = DecentralizedMycelialRuntime(
            sites=DEFAULT_SITES,
            capabilities=DEFAULT_CAPABILITIES,
            comm_range_m=2.0,
            comm_mode="isolated",
        )
        sample_a = (
            SoilSample("s1", "scout", "patch_a", SensorKind.MOISTURE, 0.92, 0.1, Pose2D(0.85, -0.50, 0.0)),
            SoilSample("s2", "scout", "patch_a", SensorKind.FIRMNESS, 0.16, 0.1, Pose2D(0.85, -0.50, 0.0)),
            SoilSample("s3", "scout", "patch_a", SensorKind.STAND_GAP, 0.85, 0.1, Pose2D(0.85, -0.50, 0.0)),
        )
        frame = ObservationFrame(
            episode_id="ep-iso",
            time_s=0.1,
            robots=(
                RobotObservation("planter", Pose2D(0.10, -0.05, 0.0), 0.1, 1.5, 2, ()),
                RobotObservation("scout", Pose2D(0.85, -0.50, 0.0), 0.1, 1.5, 0, sample_a),
            ),
        )
        isolated_runtime.decide(frame)
        self.assertTrue(isolated_runtime.nodes["scout"].local_map.estimate("patch_a").complete)
        self.assertFalse(isolated_runtime.nodes["planter"].local_map.estimate("patch_a").complete)
        self.assertEqual(isolated_runtime.total_gossip_packets, 0)

        # 2. Under normal comms within UWB/HaLow range, Scout educates Planter via compact gossip packet
        connected_runtime = DecentralizedMycelialRuntime(
            sites=DEFAULT_SITES,
            capabilities=DEFAULT_CAPABILITIES,
            comm_range_m=2.0,
            comm_mode="normal",
        )
        connected_runtime.decide(frame)
        self.assertTrue(connected_runtime.nodes["scout"].local_map.estimate("patch_a").complete)
        self.assertTrue(connected_runtime.nodes["planter"].local_map.estimate("patch_a").complete)
        self.assertGreaterEqual(connected_runtime.total_gossip_packets, 1)

        # 3. Subsequent tick with no new samples sends 0 duplicate sample gossip (event-triggered deduplication)
        pkts_before = connected_runtime.total_gossip_packets
        frame_2 = ObservationFrame(
            episode_id="ep-iso",
            time_s=0.2,
            robots=(
                RobotObservation("planter", Pose2D(0.12, -0.05, 0.0), 0.2, 1.5, 2, ()),
                RobotObservation("scout", Pose2D(0.86, -0.50, 0.0), 0.2, 1.5, 0, ()),
            ),
        )
        connected_runtime.decide(frame_2)
        self.assertEqual(connected_runtime.total_gossip_packets, pkts_before)

    def test_intermittent_comms_reconciles_maps_after_blackout_and_plants_both_seeds(self) -> None:
        sim = MujocoPaddySimulation(sites=DEFAULT_SITES)
        runtime = DecentralizedMycelialRuntime(
            sites=DEFAULT_SITES,
            capabilities=DEFAULT_CAPABILITIES,
            comm_range_m=2.15,
            comm_mode="intermittent",
        )
        runner = EpisodeRunner(sim, runtime)
        runner.run(seed=7, steps=145)

        self.assertEqual(len(sim.completed_drops), 2)
        planted_targets = {d.target_site_id for d in sim.completed_drops}
        self.assertEqual(planted_targets, {"patch_b", "patch_c"})
        self.assertGreater(runtime.total_gossip_packets, 0)

    def test_unexpected_bad_soil_propagates_from_scout_to_planter_and_changes_plant_target(self) -> None:
        # In UNEXPECTED_BAD_SOIL_TRUTH, patch_b is waterlogged mud while patch_d (north row) is viable!
        sim = MujocoPaddySimulation(sites=DEFAULT_SITES, hidden_truth=UNEXPECTED_BAD_SOIL_TRUTH)
        runtime = DecentralizedMycelialRuntime(
            sites=DEFAULT_SITES,
            capabilities=DEFAULT_CAPABILITIES,
            comm_range_m=2.15,
            comm_mode="normal",
        )
        runner = EpisodeRunner(sim, runtime)
        runner.run(seed=7, steps=155)

        planted_targets = {d.target_site_id for d in sim.completed_drops}
        # Must NOT plant in waterlogged patch_a or patch_b; must plant in viable patch_c and/or patch_d
        self.assertNotIn("patch_a", planted_targets)
        self.assertNotIn("patch_b", planted_targets)
        self.assertTrue(planted_targets.issubset({"patch_c", "patch_d"}))
        self.assertGreaterEqual(len(sim.completed_drops), 1)


if __name__ == "__main__":
    unittest.main()
