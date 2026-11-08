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
    RobotCommand,
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
        self.assertEqual(
            res_b["viable_seed_placements"], 2,
            msg=f"Mycelial nominal regression: drops={res_b['completed_seed_drops']}; "
                f"travel={res_b['travel_distance_m']}; "
                f"estimates={res_b['site_estimates']}; events={res_b['key_events'][-12:]}"
        )

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
        self.assertLessEqual(float(rec["observed_at_s"]), 10.1)
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
            if tick > 0:
                planner.record_commands(
                    (
                        RobotCommand(
                            "planter",
                            issued_at_s=round(t_s - 0.1, 2),
                            expires_at_s=round(t_s + 0.1, 2),
                            task_id="plant:patch_d",
                            linear_m_s=0.35,
                        ),
                    )
                )
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

    def test_no_privileged_is_bogged_shortcut_and_peer_isolation_across_blackouts(self) -> None:
        """Verify is_bogged=True alone never triggers rescue without commanded motion, and peer positions never leak across blackouts."""
        planner = CooperativeSwarmPlanner(DEFAULT_CAPABILITIES, stuck_ticks_threshold=2)
        field_map = MeasuredFieldMap(DEFAULT_SITES, sample_radius_m=0.6)

        # 1. Even if is_bogged=True is set on RobotObservation, HOLD (0 commanded motion) must NEVER trigger rescue
        for tick in range(4):
            t_s = round(1.0 + 0.1 * tick, 2)
            planner.record_commands(
                (RobotCommand("planter", issued_at_s=round(t_s - 0.1, 2), expires_at_s=round(t_s + 0.1, 2), linear_m_s=0.0),)
            )
            frame = ObservationFrame(
                episode_id="ep-neg-shortcut",
                time_s=t_s,
                robots=(
                    RobotObservation("planter", Pose2D(1.55, -0.52, 0.0), t_s, 1.5, 2, is_bogged=True, true_speed_m_s=0.0),
                    RobotObservation("scout", Pose2D(1.10, -0.20, 0.0), t_s, 1.5, 0, is_bogged=False, true_speed_m_s=0.25),
                ),
            )
            planner.decide(frame, field_map)
            self.assertEqual(planner.last_rescue_assessments, {})

        # 2. Peer positions must NOT leak into uwb_peer_ranges_m during isolated or intermittent blackout or link_ok=False
        iso_runtime = DecentralizedMycelialRuntime(
            sites=DEFAULT_SITES,
            capabilities=DEFAULT_CAPABILITIES,
            comm_range_m=2.15,
            comm_mode="isolated",
        )
        iso_frame = ObservationFrame(
            episode_id="ep-peer-iso",
            time_s=1.0,
            robots=(
                RobotObservation("planter", Pose2D(0.50, -0.50, 0.0), 1.0, 1.5, 2),
                RobotObservation("scout", Pose2D(0.90, -0.50, 0.0), 1.0, 1.5, 0),
            ),
        )
        iso_runtime.decide(iso_frame)
        self.assertEqual(iso_runtime.latest_sensor_telemetry["planter"].uwb_peer_ranges_m, {})
        self.assertEqual(iso_runtime.latest_sensor_telemetry["scout"].uwb_peer_ranges_m, {})

        inter_runtime = DecentralizedMycelialRuntime(
            sites=DEFAULT_SITES,
            capabilities=DEFAULT_CAPABILITIES,
            comm_range_m=2.15,
            comm_mode="intermittent",
        )
        # t=1.5s is inside the intermittent blackout window [0.8s, 3.8s]
        blackout_frame = ObservationFrame(
            episode_id="ep-peer-inter",
            time_s=1.5,
            robots=(
                RobotObservation("planter", Pose2D(0.50, -0.50, 0.0), 1.5, 1.5, 2),
                RobotObservation("scout", Pose2D(0.90, -0.50, 0.0), 1.5, 1.5, 0),
            ),
        )
        inter_runtime.decide(blackout_frame)
        self.assertEqual(inter_runtime.latest_sensor_telemetry["planter"].uwb_peer_ranges_m, {})
        self.assertEqual(inter_runtime.latest_sensor_telemetry["scout"].uwb_peer_ranges_m, {})

    def test_live_gui_app_launch_presets_and_random_scenario(self) -> None:
        """Verify LiveSwarmDashboardApp launches, steps, switches presets, and runs Random Scenario without error."""
        import tkinter as tk
        from ricebot.viewer.live_app import LiveSwarmDashboardApp

        try:
            root = tk.Tk()
        except tk.TclError as exc:
            self.skipTest(f"Tkinter display unavailable in headless CI: {exc}")
        root.withdraw()
        try:
            app = LiveSwarmDashboardApp(
                root,
                initial_scenario="golden_rescue",
                initial_comms="normal",
                initial_policy="cooperative",
                initial_rescue_policy="react_orchestrator",
                seed=7,
            )
            for _ in range(5):
                app.step_once()
            self.assertEqual(app.step_count, 5)
            self.assertIsNotNone(app.session.current_frame)

            for preset in ("high_risk_abstain", "sensor_degraded", "golden_rescue"):
                app.scenario_var.set(preset)
                app.reset_episode()
                app.step_once()
                self.assertEqual(app.scenario_var.get(), preset)
                self.assertEqual(app.step_count, 1)

            # Verify comms applicability indicator is False under cooperative/rescue and True under mycelial nominal
            self.assertFalse(app.comms_applies)
            app.scenario_var.set("nominal")
            app.policy_var.set("mycelial")
            app.reset_episode()
            self.assertTrue(app.comms_applies)

            # Verify Randomized Episode launches scenario='randomized', resumes playback if playing, and supports custom seed entry
            app.policy_var.set("cooperative")
            app.playing = True
            prev_seed = app.seed
            app.trigger_random_scenario()
            self.assertEqual(app.scenario_var.get(), "randomized")
            self.assertNotEqual(app.seed_var.get(), prev_seed)
            self.assertIsNotNone(app.randomized_spec)
            self.assertTrue(app.playing)
            app.toggle_play()
            self.assertFalse(app.playing)

            app.seed_entry_var.set("42")
            app.apply_custom_seed()
            self.assertEqual(app.seed, 42)
            self.assertEqual(app.session.seed, 42)
            app.step_once()
            self.assertEqual(app.step_count, 1)
        finally:
            try:
                if hasattr(app, "renderer") and app.renderer is not None:
                    app.renderer.close()
            except Exception:
                pass
            root.destroy()

    def test_randomized_episode_12_seed_variation_and_reproducibility(self) -> None:
        """Verify Randomized Episode mode is deterministic per seed and produces genuine multi-seed behavioural variation."""
        from ricebot.world import generate_randomized_episode_spec

        # 1. Same seed and configuration must always reproduce the exact same episode and trace
        for check_seed in (2, 8):
            cfg_a = EpisodeConfig(scenario="randomized", policy="cooperative", seed=check_seed)
            cfg_b = EpisodeConfig(scenario="randomized", policy="cooperative", seed=check_seed)
            self.assertEqual(cfg_a, cfg_b)
            self.assertFalse(cfg_a.expose_bogged_flag)

            sess_a = create_mujoco_episode_session(cfg_a)
            sess_b = create_mujoco_episode_session(cfg_b)
            sess_a.run(seed=check_seed, steps=cfg_a.steps)
            sess_b.run(seed=check_seed, steps=cfg_b.steps)

            trace_a = sess_a.export_trace()
            trace_b = sess_b.export_trace()
            self.assertTrue(trace_a.matches_parity(trace_b))
            self.assertEqual(
                [d.target_site_id for d in sess_a.sim.completed_drops],
                [d.target_site_id for d in sess_b.sim.completed_drops],
            )
            self.assertEqual(
                [(r.helper_robot_id, r.target_robot_id, r.extracted) for r in sess_a.sim.completed_recoveries],
                [(r.helper_robot_id, r.target_robot_id, r.extracted) for r in sess_b.sim.completed_recoveries],
            )
            if check_seed == 2:
                # Verify physical extraction AND post-recovery planting on rescued randomized episode
                self.assertEqual(len(sess_a.sim.completed_recoveries), 1)
                rec2 = sess_a.sim.completed_recoveries[0]
                self.assertTrue(rec2.latched)
                self.assertTrue(rec2.extracted)
                self.assertGreaterEqual(rec2.victim_displacement_m, 0.18)
                planted_s2 = [
                    d.target_site_id
                    for d in sess_a.sim.completed_drops
                    if d.contact_observed_at_s is not None
                ]
                self.assertEqual(planted_s2, ["patch_b", "patch_c"])

        # Verify PaddyTerraceWorld.reset(new_seed) refreshes hidden soil truth when scenario is set
        from ricebot.world import PaddyTerraceWorld

        world_rand = PaddyTerraceWorld(scenario="randomized")
        world_rand.reset(2)
        truth_seed_2 = dict(world_rand._hidden_site_truth)
        world_rand.reset(5)
        truth_seed_5 = dict(world_rand._hidden_site_truth)
        self.assertNotEqual(truth_seed_2, truth_seed_5)

        world_shift = PaddyTerraceWorld(scenario="shifted_moisture")
        world_shift.reset(2)
        shift_seed_2 = dict(world_shift._hidden_site_truth)
        world_shift.reset(5)
        shift_seed_5 = dict(world_shift._hidden_site_truth)
        self.assertNotEqual(shift_seed_2, shift_seed_5)

        # 2. Evaluate 12 different seeds (1..12) through authoritative EpisodeSession
        visited_sequences: set[tuple[str, ...]] = set()
        viable_sets: set[tuple[str, ...]] = set()
        spawn_pairs: set[tuple[tuple[float, float], tuple[float, float]]] = set()
        outcome_classes: set[str] = set()
        rescued_and_planted_seeds: list[int] = []

        for s in range(1, 13):
            spec = generate_randomized_episode_spec(s)
            viable_sets.add(spec.viable_sites)
            spawn_pairs.add(
                (
                    (spec.planter_spawn.x_m, spec.planter_spawn.y_m),
                    (spec.scout_spawn.x_m, spec.scout_spawn.y_m),
                )
            )
            cfg = EpisodeConfig(scenario="randomized", policy="cooperative", seed=s)
            sess = create_mujoco_episode_session(cfg)
            sess.reset(s)
            sampled_order: list[str] = []
            seen_sites: set[str] = set()
            for _ in range(cfg.steps):
                t = sess.step_once()
                for r in t.frame.robots:
                    for sam in r.samples:
                        if sam.site_id not in seen_sites:
                            seen_sites.add(sam.site_id)
                            sampled_order.append(f"{r.robot_id}:{sam.site_id}")
            visited_sequences.add(tuple(sampled_order))

            verdicts = {od.verdict for od in sess.orchestrator_decisions.values()}
            verified_drops = [
                d.target_site_id
                for d in sess.sim.completed_drops
                if d.contact_observed_at_s is not None
            ]
            if sess.sim.completed_recoveries and any(r.extracted for r in sess.sim.completed_recoveries):
                outcome_classes.add("rescue_extracted")
                if len(verified_drops) >= 1:
                    rescued_and_planted_seeds.append(s)
            elif RescueVerdict.ABANDON_ESCALATE in verdicts:
                outcome_classes.add("refusal_abandon_escalate")
            elif not verdicts and len(verified_drops) >= 1:
                outcome_classes.add("nominal_continue_working")

        self.assertGreaterEqual(len(spawn_pairs), 6)
        self.assertGreaterEqual(len(viable_sets), 4)
        self.assertGreaterEqual(len(visited_sequences), 3)
        self.assertGreaterEqual(len(outcome_classes), 2)
        self.assertIn("rescue_extracted", outcome_classes)
        self.assertIn("refusal_abandon_escalate", outcome_classes)
        self.assertTrue(len(rescued_and_planted_seeds) >= 1)

    def test_in_place_session_reset_and_scalable_fleet_wave0(self) -> None:
        """Issue #71 + Option A: Verify EpisodeSession.reset(new_seed) and N-rover fleet execution."""
        for pol in ("cooperative", "mycelial"):
            cfg = EpisodeConfig(scenario="randomized", policy=pol, seed=2, include_shadow_mycelial=True)
            session = create_mujoco_episode_session(cfg)
            session.reset(2)
            for _ in range(5):
                session.step_once()
            session.reset(8)
            for _ in range(5):
                tick = session.step_once()
            self.assertEqual(session.seed, 8)
            self.assertEqual(len(session.ticks), 5)
            self.assertEqual(tick.frame.episode_id, "mujoco-ep-8")

        cfg_fleet = EpisodeConfig(scenario="nominal", policy="mycelial", seed=7, num_rovers=4, steps=12)
        fleet_sess = create_mujoco_episode_session(cfg_fleet)
        decisions = fleet_sess.run(seed=7, steps=12)
        self.assertEqual(len(decisions), 12)
        assert fleet_sess.current_frame is not None
        self.assertEqual(len(fleet_sess.current_frame.robots), 4)
        self.assertEqual(
            [r.robot_id for r in fleet_sess.current_frame.robots],
            ["planter", "scout", "planter_2", "scout_2"],
        )


if __name__ == "__main__":
    unittest.main()
