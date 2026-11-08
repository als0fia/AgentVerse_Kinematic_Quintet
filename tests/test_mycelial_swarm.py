"""Unit and MuJoCo physics tests for Policy B: Decentralised Mycelial SLAM, Gossip & Task Negotiation."""
from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ricebot.agent import (
    CooperativeSwarmPlanner,
    DecentralizedMycelialRuntime,
    SwarmRobotAgent,
    evaluate_continuous_belief_field,
    sample_continuous_belief_grid,
)
from ricebot.contracts import (
    ObservationFrame,
    Pose2D,
    RobotObservation,
    SensorKind,
    SoilSample,
    TaskMode,
)
from ricebot.mission import SiteEstimate
from ricebot.mujoco_sim import MujocoPaddySimulation
from ricebot.robot import DEFAULT_CAPABILITIES, PLANTER_SPEC, evaluate_bekker_terramechanics
from ricebot.runtime import EpisodeRunner
from ricebot.viewer import SwarmBeliefDashboardRecorder, render_dashboard_html
from ricebot.world import ContinuousPaddyTerrain, DEFAULT_SITES, UNEXPECTED_BAD_SOIL_TRUTH


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

        # 2. Under normal comms within UWB/HaLow range (default gossip_delay_s=0.4s),
        # Scout transmits compact gossip packet at t=0.1s; while in flight (t=0.1s, 0.2s),
        # Planter and Scout maintain distinct local maps until delivery at t=0.5s.
        connected_runtime = DecentralizedMycelialRuntime(
            sites=DEFAULT_SITES,
            capabilities=DEFAULT_CAPABILITIES,
            comm_range_m=2.0,
            comm_mode="normal",
        )
        connected_runtime.decide(frame)
        self.assertTrue(connected_runtime.nodes["scout"].local_map.estimate("patch_a").complete)
        self.assertFalse(connected_runtime.nodes["planter"].local_map.estimate("patch_a").complete)
        self.assertGreaterEqual(connected_runtime.total_gossip_packets, 1)
        self.assertGreaterEqual(len(connected_runtime._in_flight_gossip), 1)

        # 3. Subsequent tick at t=0.2s (still in transit) sends 0 duplicate sample gossip (event-triggered deduplication)
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
        self.assertFalse(connected_runtime.nodes["planter"].local_map.estimate("patch_a").complete)

        # 4. At t=0.5s (after gossip_delay_s=0.4s), the in-flight packet arrives and educates Planter
        frame_3 = ObservationFrame(
            episode_id="ep-iso",
            time_s=0.5,
            robots=(
                RobotObservation("planter", Pose2D(0.14, -0.05, 0.0), 0.5, 1.5, 2, ()),
                RobotObservation("scout", Pose2D(0.87, -0.50, 0.0), 0.5, 1.5, 0, ()),
            ),
        )
        connected_runtime.decide(frame_3)
        self.assertTrue(connected_runtime.nodes["planter"].local_map.estimate("patch_a").complete)
        self.assertIn("patch_a", connected_runtime.nodes["planter"].local_map.probed_sites)
        self.assertTrue(all(pkt.sent_at_s > 0.1 for _, _, pkt in connected_runtime._in_flight_gossip))

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

    def test_continuous_terrain_and_bekker_terramechanics_and_dashboard_export(self) -> None:
        terrain = ContinuousPaddyTerrain(sites=DEFAULT_SITES)
        pt_mud = terrain.evaluate(0.85, -0.50)  # patch_a (Lannitian mud hazard)
        pt_loam = terrain.evaluate(1.60, -0.50)  # patch_b (viable loam gap)
        pt_crop = terrain.evaluate(0.85, 0.45)  # crop_nw (protected dense rice stand)

        self.assertGreater(pt_mud.moisture, 0.85)
        self.assertGreater(pt_mud.mud_depth_m, pt_loam.mud_depth_m)
        self.assertGreater(pt_loam.firmness, pt_mud.firmness)
        self.assertGreater(pt_crop.plant_density, 0.80)

        # Bekker-Wong terramechanics: heavier machine or smaller wheels sinks deeper in mud
        light_rover = evaluate_bekker_terramechanics(
            PLANTER_SPEC,
            moisture=pt_mud.moisture,
            firmness=pt_mud.firmness,
            mud_depth_m=pt_mud.mud_depth_m,
            mass_override_kg=3.5,
            wheel_radius_override_m=0.065,
        )
        heavy_rover = evaluate_bekker_terramechanics(
            PLANTER_SPEC,
            moisture=pt_mud.moisture,
            firmness=pt_mud.firmness,
            mud_depth_m=pt_mud.mud_depth_m,
            mass_override_kg=14.0,
            wheel_radius_override_m=0.042,
        )
        self.assertGreater(heavy_rover.sinkage_z_m, light_rover.sinkage_z_m)
        self.assertGreater(heavy_rover.compaction_resistance_n, light_rover.compaction_resistance_n)
        self.assertTrue(heavy_rover.bellied_out)
        self.assertGreater(heavy_rover.immobilization_risk, light_rover.immobilization_risk)

        # Verify ReadOnlyView dashboard recorder & HTML export
        sim = MujocoPaddySimulation(sites=DEFAULT_SITES)
        runtime = DecentralizedMycelialRuntime(
            sites=DEFAULT_SITES,
            capabilities=DEFAULT_CAPABILITIES,
            comm_range_m=2.15,
            comm_mode="normal",
        )
        recorder = SwarmBeliefDashboardRecorder(
            runtime=runtime,
            terrain=sim.world.continuous_terrain,
            sites=DEFAULT_SITES,
            grid_nx=12,
            grid_ny=8,
            snapshot_stride=10,
        )
        runner = EpisodeRunner(sim, runtime, view=recorder)
        runner.run(seed=7, steps=35)

        bundle = recorder.export_bundle("test_nominal")
        self.assertEqual(bundle["schema_version"], "antenna.dashboard.v1")
        self.assertGreater(len(bundle["frames"]), 0)
        p_belief = evaluate_continuous_belief_field(runtime.nodes["planter"].local_map, 1.60, -0.50)
        self.assertIn("moisture", p_belief)
        self.assertIn("uncertainty", p_belief)

        with tempfile.TemporaryDirectory() as tmpdir:
            out_html = Path(tmpdir) / "dash.html"
            html_str = render_dashboard_html({"test_nominal": bundle}, output_path=out_html)
            self.assertTrue(out_html.is_file())
            self.assertIn("Bekker", html_str)

    def test_two_tier_camera_fov_cone_discovery_vs_physical_stop_and_probe(self) -> None:
        """Verify Tier-1 camera FOV cone discovery (`discovered_sites` + `continuous_grid` frustum update)
        vs. Tier-2 physical penetrometer stop-and-probe (`probed_sites` + high-precision confirmation)."""
        runtime = DecentralizedMycelialRuntime(
            sites=DEFAULT_SITES,
            capabilities=DEFAULT_CAPABILITIES,
            comm_range_m=2.15,
            comm_mode="isolated",
        )
        planter_node = runtime.nodes["planter"]
        self.assertEqual(len(planter_node.local_map.discovered_sites), 0)
        self.assertEqual(len(planter_node.local_map.probed_sites), 0)

        # Tier 1: Planter at (0.15, -0.50, yaw=0.0) looks East toward patch_a at (0.85, -0.50) (dist=0.70m <= 0.95m)
        # while patch_c at (2.35, -0.50) is outside camera range (dist=2.20m > 0.95m).
        obs_scan = RobotObservation("planter", Pose2D(0.15, -0.50, 0.0), 0.1, 1.5, 2, ())
        planter_node.process_local_sensors(obs_scan, {"planter": obs_scan.pose})

        self.assertIn("patch_a", planter_node.local_map.discovered_sites)
        self.assertNotIn("patch_c", planter_node.local_map.discovered_sites)
        self.assertNotIn("patch_a", planter_node.local_map.probed_sites)

        # Forward camera frustum reduces continuous_grid uncertainty inside FOV cone (at x=0.60, y=-0.50)
        # while behind the rover (x=-0.20, y=0.75) remains at prior uncertainty 1.0
        unc_in_cone = planter_node.local_map.continuous_grid.uncertainty_at(0.60, -0.50)
        unc_outside_cone = planter_node.local_map.continuous_grid.uncertainty_at(-0.20, 0.75)
        self.assertLess(unc_in_cone, 0.75)
        self.assertAlmostEqual(unc_outside_cone, 1.0, places=3)

        # Tier-1 visual discovery reduces site uncertainty to intermediate level (< 1.0, > 0.30)
        unc_tier1 = planter_node.local_map.uncertainty("patch_a")
        self.assertLess(unc_tier1, 0.90)
        self.assertGreater(unc_tier1, 0.35)

        # Tier 2: Physical stop-and-probe penetrometer samples at patch_a confirm subsurface firmness
        probe_samples = (
            SoilSample("p1", "planter", "patch_a", SensorKind.MOISTURE, 0.91, 0.2, Pose2D(0.85, -0.50, 0.0)),
            SoilSample("p2", "planter", "patch_a", SensorKind.FIRMNESS, 0.17, 0.2, Pose2D(0.85, -0.50, 0.0)),
            SoilSample("p3", "planter", "patch_a", SensorKind.STAND_GAP, 0.84, 0.2, Pose2D(0.85, -0.50, 0.0)),
        )
        obs_probe = RobotObservation("planter", Pose2D(0.85, -0.50, 0.0), 0.2, 1.5, 2, probe_samples)
        planter_node.process_local_sensors(obs_probe, {"planter": obs_probe.pose})

        self.assertIn("patch_a", planter_node.local_map.probed_sites)
        self.assertTrue(planter_node.local_map.estimate("patch_a").complete)
        self.assertLessEqual(planter_node.local_map.uncertainty("patch_a"), 0.08)

    def test_distinct_per_robot_belief_maps_and_delayed_gossip_reconciliation(self) -> None:
        """Verify Planter and Scout maintain genuinely distinct `continuous_grid` belief maps
        as they scan different regions, and reconcile after `gossip_delay_s`."""
        runtime = DecentralizedMycelialRuntime(
            sites=DEFAULT_SITES,
            capabilities=DEFAULT_CAPABILITIES,
            comm_range_m=2.5,
            comm_mode="normal",
            gossip_delay_s=0.4,
        )
        scout_samples = (
            SoilSample("sc1", "scout", "patch_d", SensorKind.MOISTURE, 0.54, 0.1, Pose2D(1.55, 0.35, 0.0)),
            SoilSample("sc2", "scout", "patch_d", SensorKind.FIRMNESS, 0.68, 0.1, Pose2D(1.55, 0.35, 0.0)),
            SoilSample("sc3", "scout", "patch_d", SensorKind.STAND_GAP, 0.80, 0.1, Pose2D(1.55, 0.35, 0.0)),
        )
        # At t=0.1s, Planter scans southern lane (y=-0.50) while Scout probes northern lane patch_d (1.55, 0.35)
        frame_t1 = ObservationFrame(
            episode_id="ep-delay",
            time_s=0.1,
            robots=(
                RobotObservation("planter", Pose2D(0.15, -0.50, 0.0), 0.1, 1.5, 2, ()),
                RobotObservation("scout", Pose2D(1.55, 0.35, 0.0), 0.1, 1.5, 0, scout_samples),
            ),
        )
        runtime.decide(frame_t1)

        p_grid_t1 = runtime.nodes["planter"].local_map.continuous_grid
        s_grid_t1 = runtime.nodes["scout"].local_map.continuous_grid

        # Before gossip arrives, Planter has lower uncertainty in southern lane (0.60, -0.50) than Scout,
        # and Scout has lower uncertainty at northern patch_d (1.55, 0.35) than Planter!
        self.assertLess(p_grid_t1.uncertainty_at(0.60, -0.50), s_grid_t1.uncertainty_at(0.60, -0.50))
        self.assertLess(s_grid_t1.uncertainty_at(1.55, 0.35), p_grid_t1.uncertainty_at(1.55, 0.35))
        self.assertNotIn("patch_d", runtime.nodes["planter"].local_map.probed_sites)
        self.assertIn("patch_d", runtime.nodes["scout"].local_map.probed_sites)

        # Sampled 2D heatmaps for GUI are also distinct between Planter and Scout at t=0.1s
        p_heat_t1 = sample_continuous_belief_grid(runtime.nodes["planter"].local_map, nx=18, ny=12)
        s_heat_t1 = sample_continuous_belief_grid(runtime.nodes["scout"].local_map, nx=18, ny=12)
        self.assertNotEqual(p_heat_t1["uncertainty"], s_heat_t1["uncertainty"])

        # At t=0.3s (< 0.1 + 0.4s), packet is still in transit
        frame_t3 = ObservationFrame(
            episode_id="ep-delay",
            time_s=0.3,
            robots=(
                RobotObservation("planter", Pose2D(0.20, -0.50, 0.0), 0.3, 1.5, 2, ()),
                RobotObservation("scout", Pose2D(1.55, 0.35, 0.0), 0.3, 1.5, 0, ()),
            ),
        )
        runtime.decide(frame_t3)
        self.assertFalse(runtime.nodes["planter"].local_map.estimate("patch_d").complete)

        # At t=0.5s (>= 0.1 + 0.4s), delayed gossip packet is delivered and reconciles Planter's map
        frame_t5 = ObservationFrame(
            episode_id="ep-delay",
            time_s=0.5,
            robots=(
                RobotObservation("planter", Pose2D(0.25, -0.50, 0.0), 0.5, 1.5, 2, ()),
                RobotObservation("scout", Pose2D(1.55, 0.35, 0.0), 0.5, 1.5, 0, ()),
            ),
        )
        runtime.decide(frame_t5)
        self.assertTrue(runtime.nodes["planter"].local_map.estimate("patch_d").complete)
        self.assertIn("patch_d", runtime.nodes["planter"].local_map.probed_sites)
        self.assertLess(
            runtime.nodes["planter"].local_map.continuous_grid.uncertainty_at(1.55, 0.35),
            0.65,
        )

    def test_expected_information_gain_task_bidding_and_reset(self) -> None:
        """Verify Shannon/Gaussian Expected Information Gain (EIG) scoring on both
        `DecentralizedRobotNode` and `SwarmRobotAgent`, and `.reset()` on both runtimes."""
        runtime = DecentralizedMycelialRuntime(
            sites=DEFAULT_SITES,
            capabilities=DEFAULT_CAPABILITIES,
            comm_range_m=2.15,
            comm_mode="normal",
        )
        scout_node = runtime.nodes["scout"]
        eig_unvisited = scout_node.local_map.expected_information_gain("patch_a")

        # Tier-1 visual scan reduces EIG moderately (tau ~ 2.5), while Tier-2 probe collapses EIG
        obs_scan = RobotObservation("scout", Pose2D(0.25, -0.50, 0.0), 0.1, 1.5, 0, ())
        scout_node.process_local_sensors(obs_scan, {"scout": obs_scan.pose})
        eig_visual = scout_node.local_map.expected_information_gain("patch_a")
        self.assertLess(eig_visual, eig_unvisited)
        self.assertGreater(eig_visual, 2.0)

        probe_samples = (
            SoilSample("e1", "scout", "patch_a", SensorKind.MOISTURE, 0.90, 0.2, Pose2D(0.85, -0.50, 0.0)),
            SoilSample("e2", "scout", "patch_a", SensorKind.FIRMNESS, 0.18, 0.2, Pose2D(0.85, -0.50, 0.0)),
            SoilSample("e3", "scout", "patch_a", SensorKind.STAND_GAP, 0.82, 0.2, Pose2D(0.85, -0.50, 0.0)),
        )
        obs_probe = RobotObservation("scout", Pose2D(0.85, -0.50, 0.0), 0.2, 1.5, 0, probe_samples)
        scout_node.process_local_sensors(obs_probe, {"scout": obs_probe.pose})
        eig_probed = scout_node.local_map.expected_information_gain("patch_a")
        self.assertLess(eig_probed, eig_visual)
        self.assertLess(eig_probed, 1.25)

        # Verify compute_task_bids embeds EIG in SAMPLE proposals and skips already-probed patch_a
        bids = scout_node.compute_task_bids(obs_probe)
        sample_bids = [b for b in bids if b.mode == TaskMode.SAMPLE]
        self.assertTrue(len(sample_bids) > 0)
        self.assertTrue(all(b.site_id != "patch_a" for b in sample_bids))
        self.assertIn("EIG=", sample_bids[0].proposal_text)

        # Verify SwarmRobotAgent.compute_bids (alias for generate_bids) & compute_expected_information_gain
        agent = SwarmRobotAgent(DEFAULT_CAPABILITIES[1])
        sites_map = {s.site_id: s for s in DEFAULT_SITES if s.permitted}
        estimates_map = {
            sid: SiteEstimate(
                site_id=sid,
                moisture=0.55 if sid == "patch_a" else None,
                firmness=0.65 if sid == "patch_a" else None,
                stand_gap=0.78 if sid == "patch_a" else None,
                sample_count=3 if sid == "patch_a" else 0,
            )
            for sid in sites_map
        }
        dists = {sid: 1.0 for sid in sites_map}
        eig_a = agent.compute_expected_information_gain("patch_a", sites_map, estimates_map)
        eig_b = agent.compute_expected_information_gain("patch_b", sites_map, estimates_map)
        self.assertGreater(eig_b, eig_a)
        agent_bids = agent.compute_bids(obs_scan, sites_map, estimates_map, dists, [])
        self.assertTrue(any(b.mode == TaskMode.SAMPLE and "EIG=" in b.reason for b in agent_bids))

        # Verify .reset() on DecentralizedMycelialRuntime and CooperativeSwarmPlanner
        runtime.reset()
        self.assertEqual(len(runtime.nodes["scout"].local_map.all_samples), 0)
        self.assertEqual(len(runtime.nodes["scout"].local_map.discovered_sites), 0)
        self.assertEqual(len(runtime.nodes["scout"].local_map.probed_sites), 0)
        self.assertEqual(len(runtime._in_flight_gossip), 0)
        self.assertIsNone(runtime._last_episode_id)

        planner = CooperativeSwarmPlanner(DEFAULT_CAPABILITIES)
        planner._stuck_counts["planter"] = 3
        planner.reset()
        self.assertEqual(len(planner._stuck_counts), 0)
        self.assertIsNone(planner._last_episode)


if __name__ == "__main__":
    unittest.main()
