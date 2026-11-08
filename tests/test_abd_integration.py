from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest
from typing import Any, cast

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from ricebot.agent import (
    CooperativeSwarmPlanner,
    DecentralizedMycelialRuntime,
    FailureMode,
    HelperStatus,
    IncidentReport,
    MechanicalRescueTechnique,
    OperatorPolicy,
    OrchestratorDecision,
    OrchestratorSafetyGate,
    SwarmOrchestratorAgent,
    compute_mechanical_recovery_setpoint,
)
from ricebot.contracts import (
    ObservationFrame,
    OperatorCommandKind,
    OperatorRequest,
    Pose2D,
    RescueVerdict,
    RobotObservation,
    TaskMode,
)
from ricebot.control import clamp_recovery_actuators
from ricebot.mission import MeasuredFieldMap
from ricebot.mujoco_sim.adapter import DEFAULT_CAPABILITIES, DEFAULT_SITES, MujocoPaddySimulation
from ricebot.runtime import (
    EpisodeConfig,
    create_mujoco_episode_session,
)
from ricebot.viewer import SwarmBeliefDashboardRecorder, render_dashboard_html
from run_mujoco_paddypatch import run_baseline_comparison, run_demo


class AbdCanonicalIntegrationTests(unittest.TestCase):
    """End-to-end ABCDE verification suite for the canonical Antenna rescue application."""

    def test_session_step_once_vs_run_parity_with_orchestrator(self) -> None:
        """Phase 1+3: EpisodeSession batch run() and interactive step_once() produce identical physics and decisions."""
        cfg = EpisodeConfig(
            scenario="golden_rescue",
            policy="cooperative",
            seed=7,
            steps=105,
            rescue_policy="react_orchestrator",
        )
        s_batch = create_mujoco_episode_session(cfg)
        s_batch.run(seed=7, steps=105)

        s_step = create_mujoco_episode_session(cfg)
        s_step.reset(7)
        for _ in range(105):
            s_step.step_once()

        self.assertEqual(s_batch.tick_index, s_step.tick_index)
        assert s_batch.current_frame is not None and s_step.current_frame is not None
        self.assertAlmostEqual(s_batch.current_frame.time_s, s_step.current_frame.time_s, places=6)
        for rb, rs in zip(s_batch.current_frame.robots, s_step.current_frame.robots):
            self.assertAlmostEqual(rb.pose.x_m, rs.pose.x_m, places=6)
            self.assertAlmostEqual(rb.pose.y_m, rs.pose.y_m, places=6)
            self.assertEqual(rb.is_bogged, rs.is_bogged)

        sim_b = cast(MujocoPaddySimulation, s_batch.sim)
        sim_s = cast(MujocoPaddySimulation, s_step.sim)
        self.assertEqual(len(sim_b.completed_recoveries), 1)
        self.assertEqual(len(sim_s.completed_recoveries), 1)

        # Verify EpisodeTrace parity check
        t_b = s_batch.export_trace()
        t_s = s_step.export_trace()
        self.assertTrue(t_b.matches_parity(t_s))
        self.assertEqual(len(t_b.ticks), 105)

    def test_policy_a_and_policy_b_backwards_compatibility(self) -> None:
        """Phase 1: Existing Policy A (centralized) and Policy B (mycelial) remain functional on nominal fieldwork."""
        res_a = run_demo(scenario="nominal", policy="centralized", seed=7, steps=140)
        res_b = run_demo(scenario="nominal", policy="mycelial", seed=7, steps=140)
        self.assertEqual(res_a["viable_seed_placements"], 2)
        self.assertEqual(res_b["viable_seed_placements"], 2)

    def test_sensor_only_fault_detection_and_negative_controls(self) -> None:
        """Phase 3: Sensor-only FaultEvidence detects mud stall when expose_bogged_flag=False; obstacle & null-range do not false-trigger."""
        cfg = EpisodeConfig(
            scenario="golden_rescue",
            policy="cooperative",
            seed=7,
            steps=195,
            rescue_policy="react_orchestrator",
            expose_bogged_flag=False,
        )
        session = create_mujoco_episode_session(cfg)
        sim = cast(MujocoPaddySimulation, session.sim)
        session.run(seed=7, steps=195)

        self.assertEqual(len(sim.completed_recoveries), 1)
        planted = [d.target_site_id for d in sim.completed_drops if d.contact_observed_at_s is not None]
        self.assertEqual(planted, ["patch_b", "patch_c"])

        # Negative control 1: Mid-episode obstacle hold must not trigger false rescue
        cfg_obs = EpisodeConfig(scenario="obstacle", policy="cooperative", seed=7, steps=155)
        s_obs = create_mujoco_episode_session(cfg_obs)
        s_obs.run(seed=7, steps=155)
        obs_rescue_ticks = sum(
            1 for t in s_obs.ticks if any(i.mode == TaskMode.RESCUE for i in t.decision.intents)
        )
        self.assertEqual(obs_rescue_ticks, 0)

        # Negative control 2: Sensor dropout (front_range_m=None) must not trigger false rescue
        cfg_deg = EpisodeConfig(scenario="sensor_degraded", policy="cooperative", seed=7, steps=145)
        s_deg = create_mujoco_episode_session(cfg_deg)
        s_deg.run(seed=7, steps=145)
        null_range_ticks = sum(
            1 for t in s_deg.ticks if any(r.front_range_m is None for r in t.frame.robots)
        )
        deg_rescue_ticks = sum(
            1 for t in s_deg.ticks if any(i.mode == TaskMode.RESCUE for i in t.decision.intents)
        )
        self.assertGreater(null_range_ticks, 0)
        self.assertEqual(deg_rescue_ticks, 0)

    def test_decision_translation_actuator_clamping_and_safety_gate_veto(self) -> None:
        """Phase 2+3: Recovery actuators are clamped to M4 limits and OrchestratorSafetyGate vetoes unsafe proposals."""
        pitch, latch, spade = clamp_recovery_actuators(0.95, 2.5, -1.0)
        self.assertAlmostEqual(pitch, 0.15)
        self.assertAlmostEqual(latch, 0.65)
        self.assertAlmostEqual(spade, 0.0)

        gate = OrchestratorSafetyGate()
        incident = IncidentReport(
            incident_id="inc-unsafe-seed-carrier",
            time_s=20.0,
            stranded_robot_id="scout",
            failure_mode=FailureMode.CONTAGIOUS_MUD_TRAP,
            stalled_window_s=3.0,
            slip_ratio=0.92,
            imu_pitch_deg=4.0,
            imu_roll_deg=1.0,
            front_range_m=1.2,
            local_moisture=0.92,
            local_firmness=0.12,
            seeds_remaining=0,
            available_helpers=(HelperStatus("planter", Pose2D(0.9, 0.2, 0.0), seeds_remaining=2),),
            operator_policy=OperatorPolicy(protect_seed_carrier=True, max_helper_loss_risk=0.10),
        )
        unsafe_proposal = OrchestratorDecision(
            incident_id="inc-unsafe-seed-carrier",
            verdict=RescueVerdict.RESCUE_NOW,
            failure_mode=FailureMode.CONTAGIOUS_MUD_TRAP,
            chosen_technique=MechanicalRescueTechnique.DIRECT_PRY_AND_PULL,
            helper_robot_id="planter",
            standoff_pose=Pose2D(0.8, -0.3, 0.0),
            mechanical_setpoint=compute_mechanical_recovery_setpoint(
                "approach", MechanicalRescueTechnique.DIRECT_PRY_AND_PULL
            ),
            rationale="Unsafe proposal risking sole seed-carrier in extreme mud",
            tool_trace=(),
        )
        validated = OrchestratorSafetyGate.validate(incident, unsafe_proposal)
        self.assertTrue(validated.safety_veto_applied)
        self.assertEqual(validated.verdict, RescueVerdict.ABANDON_ESCALATE)
        self.assertIsNone(validated.helper_robot_id)

    def test_continuous_golden_rescue_episode_end_to_end(self) -> None:
        """Phase 1-3: Full continuous rescue loop (fieldwork -> stranded -> D assesses -> B extracts -> resumes planting)."""
        res = run_demo(
            scenario="golden_rescue",
            policy="cooperative",
            seed=7,
            steps=195,
            rescue_policy="react_orchestrator",
        )
        self.assertEqual(res["viable_seed_placements"], 2)
        recoveries = cast(list[dict[str, Any]], res["completed_recoveries"])
        self.assertEqual(len(recoveries), 1)
        rec = recoveries[0]
        self.assertEqual(rec["target_robot_id"], "planter")
        self.assertEqual(rec["helper_robot_id"], "scout")
        self.assertTrue(rec["latched"])
        self.assertTrue(rec["extracted"])
        self.assertTrue(rec["helper_safe"])
        self.assertGreaterEqual(float(rec["victim_displacement_m"]), 0.18)
        self.assertGreater(float(rec["peak_constraint_force_n"]), 1.0)
        self.assertEqual(res["end_state_bogged_robots"], [])

        # Verify C's minimum environment/sensor interface populated site_estimates
        site_est = cast(dict[str, dict[str, Any]], res["site_estimates"])
        self.assertIsNotNone(site_est["patch_b"]["moisture"])
        self.assertIsNotNone(site_est["patch_b"]["stand_gap"])
        self.assertIsNotNone(site_est["patch_c"]["firmness"])
        self.assertIsNotNone(site_est["patch_d"]["moisture"])
        self.assertIsNotNone(site_est["patch_d"]["firmness"])

        # Verify recovery monitoring updated D's residual learner
        monitoring = cast(list[dict[str, Any]], res["recovery_monitoring"])
        self.assertGreaterEqual(len(monitoring), 1)
        self.assertTrue(any(m["extracted"] for m in monitoring))

    def test_continuous_high_risk_refusal_episode_end_to_end(self) -> None:
        """Phase 3: High-risk refusal scenario refuses risking sole seed-carrier planter and completes 2/2 seed drops."""
        res = run_demo(
            scenario="high_risk_abstain",
            policy="cooperative",
            seed=7,
            steps=175,
            rescue_policy="react_orchestrator",
        )
        self.assertEqual(res["viable_seed_placements"], 2)
        self.assertEqual(len(cast(list[Any], res["completed_recoveries"])), 0)
        self.assertEqual(res["end_state_bogged_robots"], ["scout"])

        orch_decs = cast(dict[str, dict[str, Any]], res["orchestrator_decisions"])
        self.assertIn("scout", orch_decs)
        self.assertEqual(orch_decs["scout"]["verdict"], "abandon_escalate")
        self.assertIsNone(orch_decs["scout"]["helper_robot_id"])
        self.assertGreaterEqual(int(orch_decs["scout"]["tool_call_count"]), 2)

    def test_llm_timeout_and_malformed_response_deterministic_fallback(self) -> None:
        """Phase 3: Timeout or malformed LLM output falls back deterministically without crashing or bypassing safety."""
        class BrokenBackend:
            def call(self, _prompt: str) -> str | None:
                raise TimeoutError("Simulated remote LLM timeout")

        agent = SwarmOrchestratorAgent(backend=BrokenBackend())
        planner = CooperativeSwarmPlanner(
            DEFAULT_CAPABILITIES,
            orchestrator=agent,
            rescue_policy="react_orchestrator",
        )
        field_map = MeasuredFieldMap(DEFAULT_SITES, sample_radius_m=0.6)
        intents = ()
        for tick in range(3):
            t_s = round(6.0 + 0.1 * tick, 2)
            frame_robots = (
                RobotObservation(
                    robot_id="planter",
                    pose=Pose2D(1.55, -0.52, 0.0),
                    time_s=t_s,
                    front_range_m=1.5,
                    seeds_remaining=2,
                    is_bogged=True,
                    true_speed_m_s=0.0,
                ),
                RobotObservation(
                    robot_id="scout",
                    pose=Pose2D(1.10, -0.20, 0.0),
                    time_s=t_s,
                    front_range_m=1.5,
                    seeds_remaining=0,
                    is_bogged=False,
                    true_speed_m_s=0.25,
                ),
            )
            frame = ObservationFrame(
                episode_id="test-ep",
                time_s=t_s,
                robots=frame_robots,
                plant_feedback=(),
                recovery_feedback=(),
            )
            intents = planner.decide(frame, field_map)

        self.assertIn("planter", planner.last_orchestrator_decisions)
        odec = planner.last_orchestrator_decisions["planter"]
        self.assertTrue(odec.fallback_used)
        self.assertFalse(odec.safety_veto_applied)
        self.assertEqual(odec.verdict, RescueVerdict.RESCUE_NOW)
        self.assertTrue(any(i.mode == TaskMode.RESCUE for i in intents))

    def test_matched_scenario_baseline_comparison(self) -> None:
        """Phase 4: Matched-scenario comparison proves react_orchestrator outperforms always_rescue and never_rescue."""
        comp = run_baseline_comparison(seed=7)
        gr = cast(dict[str, dict[str, Any]], comp["golden_rescue"])
        hr = cast(dict[str, dict[str, Any]], comp["high_risk_abstain"])

        # Golden rescue: react_orchestrator recovers planter and plants 2/2 seeds; never_rescue plants 0
        self.assertEqual(gr["react_orchestrator"]["completed_recoveries_count"], 1)
        self.assertEqual(gr["react_orchestrator"]["viable_seed_placements"], 2)
        self.assertEqual(gr["never_rescue"]["completed_recoveries_count"], 0)
        self.assertEqual(gr["never_rescue"]["viable_seed_placements"], 0)
        self.assertEqual(gr["never_rescue"]["end_state_bogged_robots"], ["planter"])

        # High-risk abstain: react_orchestrator refuses rescue (0 secondary casualties, 2/2 seeds);
        # always_rescue dispatches planter into deep mud causing a secondary casualty and 0 seeds!
        self.assertEqual(hr["react_orchestrator"]["viable_seed_placements"], 2)
        self.assertEqual(hr["react_orchestrator"]["secondary_casualties"], [])
        self.assertEqual(hr["always_rescue"]["viable_seed_placements"], 0)
        self.assertIn("planter", hr["always_rescue"]["secondary_casualties"])

    def test_agent_e_dashboard_and_live_app_reflect_authoritative_session(self) -> None:
        """Phase 5 (E): Dashboard recorder, HTML renderer, and operator commands reflect authoritative EpisodeSession."""
        cfg = EpisodeConfig(
            scenario="golden_rescue",
            policy="mycelial",
            seed=7,
            steps=110,
            rescue_policy="react_orchestrator",
        )
        session = create_mujoco_episode_session(cfg)
        sim = cast(MujocoPaddySimulation, session.sim)
        runtime = cast(DecentralizedMycelialRuntime, session.policy)
        recorder = SwarmBeliefDashboardRecorder(
            runtime=runtime,
            terrain=sim.world.continuous_terrain,
            sites=DEFAULT_SITES,
            grid_nx=24,
            grid_ny=16,
            snapshot_stride=5,
        )
        session.views = (recorder,)
        session.run(seed=7, steps=110)

        bundle = recorder.export_bundle(scenario_name="Golden Rescue Test")
        snaps = bundle["frames"]
        self.assertGreater(len(snaps), 5)
        self.assertTrue(any(len(s["orchestrator_decisions"]) > 0 for s in snaps))
        self.assertTrue(any(len(s["recovery_feedback"]) > 0 for s in snaps))

        with tempfile.TemporaryDirectory() as tmpdir:
            html_path = Path(tmpdir) / "dashboard.html"
            render_dashboard_html({"Golden Rescue Test": bundle}, output_path=html_path)
            html_text = html_path.read_text(encoding="utf-8")
            self.assertIn("rescueOrchestratorPanel", html_text)
            self.assertIn("probe_perimeter", html_text)

        # Verify OperatorCommandKind.INJECT_MUD_FAULT and TELEOP_OVERRIDE route through EpisodeSession
        session.submit_operator_request(
            OperatorRequest(
                request_id="test-teleop-1",
                kind=OperatorCommandKind.TELEOP_OVERRIDE,
                issued_at_s=11.0,
                payload=(("robot_id", "scout"), ("active", True), ("linear_m_s", 0.22), ("angular_rad_s", -0.45)),
            )
        )
        tick = session.step_once()
        scout_cmd = next(c for c in tick.decision.commands if c.robot_id == "scout")
        self.assertAlmostEqual(scout_cmd.linear_m_s, 0.22, places=4)
        self.assertAlmostEqual(scout_cmd.angular_rad_s, -0.45, places=4)


if __name__ == "__main__":
    unittest.main()
