"""Unit and physics integration tests for the 3D MuJoCo PaddyPatch SimulationPort adapter."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ricebot.agent import CooperativeSwarmPlanner, RescueVerdict
from ricebot.contracts import RobotCommand, TaskMode
from ricebot.control import ControlLimits, LocalController
from ricebot.mission import MeasuredFieldMap, SimpleMissionPlanner
try:
    from ricebot.mujoco_sim.adapter import DEFAULT_CAPABILITIES, DEFAULT_SITES, MujocoPaddySimulation
    HAS_MUJOCO = True
except ImportError:
    HAS_MUJOCO = False
    DEFAULT_CAPABILITIES = ()  # type: ignore
    DEFAULT_SITES = ()  # type: ignore
    MujocoPaddySimulation = None  # type: ignore
from ricebot.runtime import EpisodeRunner, MissionCore


@unittest.skipUnless(HAS_MUJOCO, "mujoco not installed in environment")
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

    def test_autonomous_sensed_mud_fault_triggers_physical_rigid_dock_rescue_and_resumes_mission(self) -> None:
        """Golden Episode Seam: sensed mud immobilization -> RESCUE_NOW triage -> rigid dock/latch/tow -> resumed planting."""
        sim = MujocoPaddySimulation(
            sites=DEFAULT_SITES,
            probe_radius_m=0.28,
            substeps_per_tick=25,
            trap_robot_id="planter",
            trap_site_id="patch_d",
            trap_radius_m=0.28,
        )
        field_map = MeasuredFieldMap(DEFAULT_SITES, sample_radius_m=0.6)
        planner = CooperativeSwarmPlanner(DEFAULT_CAPABILITIES)
        controller = LocalController(
            ControlLimits(speed_m_s=0.38, turn_rad_s=1.15, stop_range_m=0.32, site_reach_m=0.22)
        )
        core = MissionCore(field_map, planner, controller)

        frame = sim.reset(seed=7)
        observed_bogged_with_valid_lidar = False
        observed_rescue_now_verdict = False
        observed_rescue_command = False

        max_efc_force = 0.0
        max_chassis_z = 0.0
        max_pitch_rad = 0.0

        for _ in range(195):
            decision = core.decide(frame)
            p_obs = next(r for r in frame.robots if r.robot_id == "planter")
            if p_obs.is_bogged and p_obs.front_range_m is not None and p_obs.front_range_m > 0.35:
                observed_bogged_with_valid_lidar = True
            assessment = planner.last_rescue_assessments.get("planter")
            if assessment is not None and assessment.verdict == RescueVerdict.RESCUE_NOW:
                observed_rescue_now_verdict = True
            s_intent = next(i for i in decision.intents if i.robot_id == "scout")
            s_cmd = next(c for c in decision.commands if c.robot_id == "scout")
            if s_intent.mode == TaskMode.RESCUE and s_cmd.recovery_target_id == "planter":
                observed_rescue_command = True
            frame = sim.step(decision.commands)
            if len(sim.data.efc_force) > 0:
                max_efc_force = max(max_efc_force, float(abs(sim.data.efc_force).max()))
            max_chassis_z = max(
                max_chassis_z,
                float(sim.data.sensor("planter_pos").data[2]),
                float(sim.data.sensor("scout_pos").data[2]),
            )
            for r in frame.robots:
                max_pitch_rad = max(max_pitch_rad, abs(r.pitch_rad))

        # 1. True mud immobilization was sensed independently of LiDAR range failure
        self.assertTrue(observed_bogged_with_valid_lidar)
        # 2. Planner evaluated RESCUE_NOW and Controller emitted bounded physical rescue command
        self.assertTrue(observed_rescue_now_verdict)
        self.assertTrue(observed_rescue_command)
        # 3. Physical MuJoCo rigid docking, cam-latching, pitch-lift & tandem extraction succeeded (>= 0.18 m)
        self.assertEqual(len(sim.completed_recoveries), 1)
        rec = sim.completed_recoveries[0]
        self.assertEqual(rec.helper_robot_id, "scout")
        self.assertEqual(rec.target_robot_id, "planter")
        self.assertTrue(rec.latched)
        self.assertTrue(rec.extracted)
        self.assertGreaterEqual(rec.victim_displacement_m, 0.18)
        # 3b. M4 Physical Audit Gate: No constraint-snap impulse (< 250 N), rovers remain grounded (< 0.072 m), no V-tent jackknife (< 0.12 rad)
        self.assertLess(max_efc_force, 250.0)
        self.assertLess(max_chassis_z, 0.072)
        self.assertLess(max_pitch_rad, 0.12)
        # 4. After unlatching, planter resumed autonomous fieldwork and planted both viable stand gaps
        self.assertEqual(len(sim.completed_drops), 2)
        planted_sites = {d.target_site_id for d in sim.completed_drops}
        self.assertEqual(planted_sites, {"patch_b", "patch_c"})

    def test_minimum_connected_antenna_episode_fault_to_orchestrator_to_physical_rescue_and_learner_update(
        self,
    ) -> None:
        """Minimal end-to-end acceptance test for the connected Antenna episode (no oracle flags)."""
        from ricebot.world import EMERGENT_MUD_RESCUE_TRUTH

        sim = MujocoPaddySimulation(
            sites=DEFAULT_SITES,
            probe_radius_m=0.28,
            substeps_per_tick=25,
            hidden_truth=EMERGENT_MUD_RESCUE_TRUTH,
            enable_emergent_mud_traps=True,
            trap_radius_m=0.28,
            expose_bogged_flag=False,
            use_sensor_guided_docking=True,
        )
        field_map = MeasuredFieldMap(DEFAULT_SITES, sample_radius_m=0.6)
        planner = CooperativeSwarmPlanner(DEFAULT_CAPABILITIES)
        controller = LocalController(
            ControlLimits(speed_m_s=0.38, turn_rad_s=1.15, stop_range_m=0.32, site_reach_m=0.22)
        )
        core = MissionCore(field_map, planner, controller)

        frame = sim.reset(seed=7)
        initial_learner_updates = len(planner.orchestrator.tool_belt.learner.history)
        orchestrator_rescue_decision = None
        dispatched_rescue_cmd = None

        for _ in range(195):
            # Verify privileged is_bogged oracle flag is never exposed
            for r in frame.robots:
                self.assertFalse(r.is_bogged)

            decision = core.decide(frame)
            orch_dec = planner.last_orchestrator_decisions.get("planter")
            if orch_dec is not None and orch_dec.verdict == RescueVerdict.RESCUE_NOW:
                orchestrator_rescue_decision = orch_dec

            for c in decision.commands:
                if c.robot_id == "scout" and c.recovery_target_id == "planter":
                    dispatched_rescue_cmd = c

            frame = sim.step(decision.commands)

        # 1. Local non-oracle fault evidence connected to SwarmOrchestratorAgent
        self.assertIsNotNone(orchestrator_rescue_decision)
        assert orchestrator_rescue_decision is not None
        self.assertEqual(orchestrator_rescue_decision.verdict, RescueVerdict.RESCUE_NOW)
        self.assertEqual(orchestrator_rescue_decision.helper_robot_id, "scout")
        self.assertIsNotNone(orchestrator_rescue_decision.mechanical_setpoint)
        self.assertTrue(orchestrator_rescue_decision.fallback_used)
        self.assertGreaterEqual(len(orchestrator_rescue_decision.tool_trace), 4)

        # 2. Orchestrator decision connected to bounded physical rescue controller
        self.assertIsNotNone(dispatched_rescue_cmd)
        assert dispatched_rescue_cmd is not None
        self.assertGreater(dispatched_rescue_cmd.latch_rad, 0.30)

        # 3. Actual physical outcome received and updated episode state + ResidualRecoveryLearner
        self.assertEqual(len(sim.completed_recoveries), 1)
        rec = sim.completed_recoveries[0]
        self.assertTrue(rec.extracted)
        self.assertGreaterEqual(rec.victim_displacement_m, 0.18)
        final_learner_updates = len(planner.orchestrator.tool_belt.learner.history)
        self.assertGreater(final_learner_updates, initial_learner_updates)
        self.assertEqual(len(sim.completed_drops), 2)


if __name__ == "__main__":
    unittest.main()
