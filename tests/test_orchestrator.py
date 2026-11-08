"""Unit tests for the Native ReAct Swarm Orchestrator Agent (`tests/test_orchestrator.py`)."""
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ricebot.agent import (
    DeterministicReasoner,
    FailureMode,
    HelperStatus,
    IncidentReport,
    MechanicalRescueTechnique,
    OperatorPolicy,
    OrchestratorDecision,
    OrchestratorSafetyGate,
    OrchestratorToolBelt,
    RecoveryAction,
    RescueVerdict,
    ResidualRecoveryLearner,
    SwarmOrchestratorAgent,
)
from ricebot.contracts import (
    Pose2D,
    RecoveryFeedback,
    RobotCapability,
    RobotObservation,
    TaskIntent,
    TaskMode,
)
from ricebot.control import LocalController


class MockLLMBackend:
    """Mock LLM backend that simulates a ReAct tool invocation sequence."""

    def __init__(self, responses: list[str]):
        self.responses = list(responses)
        self.call_history: list[str] = []

    def call(self, prompt: str) -> str | None:
        self.call_history.append(prompt)
        if self.responses:
            return self.responses.pop(0)
        return None


class OrchestratorAgentTests(unittest.TestCase):
    """Test suite verifying the Native ReAct Swarm Orchestrator Agent."""

    def setUp(self):
        self.tool_belt = OrchestratorToolBelt()
        self.agent = SwarmOrchestratorAgent(tool_belt=self.tool_belt)

    def test_sensor_dropout_ambiguity_resolution(self):
        """Verify the orchestrator detects sensor dropout and refuses false-alarm rescue."""
        incident = IncidentReport(
            incident_id="inc-dropout-01",
            time_s=12.5,
            stranded_robot_id="rover-2",
            failure_mode=FailureMode.SENSOR_DROPOUT,
            stalled_window_s=4.0,
            slip_ratio=0.08,  # Normal traction
            imu_pitch_deg=1.2,
            imu_roll_deg=-0.5,
            front_range_m=None,  # LiDAR dropped out!
            local_moisture=0.35,
            local_firmness=0.75,
            seeds_remaining=2,
            available_helpers=(
                HelperStatus("rover-1", Pose2D(1.0, 1.0, 0.0), seeds_remaining=0),
            ),
        )

        decision = self.agent.orchestrate(incident)
        self.assertEqual(decision.verdict, RescueVerdict.ABANDON_ESCALATE)
        self.assertEqual(decision.failure_mode, FailureMode.SENSOR_DROPOUT)
        self.assertEqual(decision.chosen_technique, MechanicalRescueTechnique.NONE)
        self.assertIn("sensor dropout", decision.rationale.lower())

    def test_honest_refusal_when_seed_carrier_is_threatened(self):
        """Verify the orchestrator refuses rescue when only available helper holds seed inventory."""
        incident = IncidentReport(
            incident_id="inc-bogged-seed-risk",
            time_s=45.0,
            stranded_robot_id="rover-2",
            failure_mode=FailureMode.CONTAGIOUS_MUD_TRAP,
            stalled_window_s=6.0,
            slip_ratio=0.92,
            imu_pitch_deg=8.0,
            imu_roll_deg=-3.0,
            front_range_m=0.15,
            local_moisture=0.88,
            local_firmness=0.12,  # Liquid Lannitian mud
            seeds_remaining=0,
            available_helpers=(
                # Rover-1 is the ONLY robot left with seeds
                HelperStatus("rover-1", Pose2D(2.0, 1.5, 0.0), seeds_remaining=4),
            ),
            operator_policy=OperatorPolicy(
                protect_seed_carrier=True,
                max_helper_loss_risk=0.10,
            ),
        )

        decision = self.agent.orchestrate(incident)
        # Should decline rescue to preserve the entire seed stock
        self.assertEqual(decision.verdict, RescueVerdict.ABANDON_ESCALATE)
        self.assertIn("secondary casualty", decision.rationale.lower())
        self.assertTrue(len(decision.tool_trace) >= 2)

    def test_rescue_authorized_when_safe_helper_available(self):
        """Verify rescue is authorized when an empty helper can safely extract the stranded robot."""
        incident = IncidentReport(
            incident_id="inc-mud-safe-helper",
            time_s=30.0,
            stranded_robot_id="rover-2",
            failure_mode=FailureMode.CONTAGIOUS_MUD_TRAP,
            stalled_window_s=5.0,
            slip_ratio=0.85,
            imu_pitch_deg=5.0,
            imu_roll_deg=2.0,
            front_range_m=0.20,
            local_moisture=0.65,
            local_firmness=0.45,  # Moderate firmness
            seeds_remaining=0,
            available_helpers=(
                HelperStatus("rover-3", Pose2D(1.2, 0.8, 0.0), seeds_remaining=0, battery_ratio=0.95),
            ),
            operator_policy=OperatorPolicy(
                protect_seed_carrier=True,
                max_helper_loss_risk=0.25,
            ),
        )

        decision = self.agent.orchestrate(incident)
        self.assertEqual(decision.verdict, RescueVerdict.RESCUE_NOW)
        self.assertEqual(decision.helper_robot_id, "rover-3")
        self.assertIsNotNone(decision.mechanical_setpoint)
        self.assertIn("direct_pry_and_pull", decision.chosen_technique.value)
        self.assertTrue(decision.mechanical_setpoint.latch_rad > 0.30)  # Rigid cam-latch engaged

    def test_react_tool_calling_loop_with_mock_llm(self):
        """Verify the agent properly iterates Thought -> Tool Action -> Observation -> Final Verdict."""
        # Simulated LLM responses:
        # Turn 1: LLM decides to probe perimeter first
        resp_turn1 = json.dumps({
            "thought": "Rover-2 is stalled. I must probe the soil perimeter first.",
            "action": "probe_perimeter",
            "action_input": {"helper_id": "rover-3", "standoff_m": 1.5},
        })
        # Turn 2: LLM inspects probe result and asks for displacement prediction
        resp_turn2 = json.dumps({
            "thought": "Perimeter is firm (0.42). Now predicting recovery displacement for Dynamic Yaw Shear.",
            "action": "predict_recovery_displacement",
            "action_input": {"technique": "dynamic_yaw_shear_and_pitch_pump", "moisture": 0.70, "firmness": 0.40, "slip_ratio": 0.80},
        })
        # Turn 3: LLM emits final verdict
        resp_turn3 = json.dumps({
            "thought": "Predicted displacement is +0.038m. Authorizing rescue.",
            "verdict": "rescue_now",
            "chosen_technique": "dynamic_yaw_shear_and_pitch_pump",
            "helper_robot_id": "rover-3",
            "rationale": "Perimeter confirmed safe and Dynamic Yaw Shear overcomes mud stiction.",
        })

        mock_backend = MockLLMBackend([resp_turn1, resp_turn2, resp_turn3])
        agent = SwarmOrchestratorAgent(backend=mock_backend, tool_belt=self.tool_belt)

        incident = IncidentReport(
            incident_id="inc-mock-llm-01",
            time_s=20.0,
            stranded_robot_id="rover-2",
            failure_mode=FailureMode.ASYMMETRIC_SLIP_OR_SUCTION,
            stalled_window_s=3.5,
            slip_ratio=0.88,
            imu_pitch_deg=4.0,
            imu_roll_deg=1.0,
            front_range_m=0.30,
            local_moisture=0.72,
            local_firmness=0.38,
            seeds_remaining=1,
            available_helpers=(
                HelperStatus("rover-3", Pose2D(0.5, 0.5, 0.0), seeds_remaining=0),
            ),
        )

        decision = agent.orchestrate(incident)
        self.assertEqual(decision.verdict, RescueVerdict.RESCUE_NOW)
        self.assertEqual(decision.helper_robot_id, "rover-3")
        self.assertEqual(decision.chosen_technique, MechanicalRescueTechnique.DYNAMIC_YAW_SHEAR_AND_PITCH_PUMP)
        self.assertEqual(len(decision.tool_trace), 2)
        self.assertEqual(decision.tool_trace[0].tool_name, "probe_perimeter")
        self.assertEqual(decision.tool_trace[1].tool_name, "predict_recovery_displacement")

    def test_system0_safety_gate_prevents_stranded_self_rescue(self):
        """Verify the safety gate overrides an invalid LLM output assigning the stranded robot as its own rescuer."""
        decision_raw = OrchestratorDecision(
            incident_id="inc-err-01",
            verdict=RescueVerdict.RESCUE_NOW,
            failure_mode=FailureMode.CONTAGIOUS_MUD_TRAP,
            chosen_technique=MechanicalRescueTechnique.DIRECT_PRY_AND_PULL,
            helper_robot_id="rover-2",  # Invalid: same as stranded robot!
            standoff_pose=None,
            mechanical_setpoint=None,
            rationale="Hallucinated self rescue",
            tool_trace=(),
        )
        incident = IncidentReport(
            incident_id="inc-err-01",
            time_s=10.0,
            stranded_robot_id="rover-2",
            failure_mode=FailureMode.CONTAGIOUS_MUD_TRAP,
            stalled_window_s=5.0,
            slip_ratio=0.90,
            imu_pitch_deg=0.0,
            imu_roll_deg=0.0,
            front_range_m=0.10,
            local_moisture=0.85,
            local_firmness=0.15,
            seeds_remaining=0,
            available_helpers=(HelperStatus("rover-2", Pose2D(0, 0, 0), 0),),
        )

        safe_decision = OrchestratorSafetyGate.validate(incident, decision_raw)
        self.assertTrue(safe_decision.safety_veto_applied)
        self.assertEqual(safe_decision.verdict, RescueVerdict.ABANDON_ESCALATE)
        self.assertIn("Safety Veto", safe_decision.rationale)


    def test_unavailable_helper_and_contradictory_model_output_vetoed(self):
        """Verify non-operational/self helpers are excluded and invalid/contradictory LLM rescues are vetoed."""
        # 1. Deterministic selection must skip stranded self-entry and non-operational helper
        incident = IncidentReport(
            incident_id="inc-helper-filter-01",
            time_s=18.0,
            stranded_robot_id="rover-2",
            failure_mode=FailureMode.CONTAGIOUS_MUD_TRAP,
            stalled_window_s=4.0,
            slip_ratio=0.85,
            imu_pitch_deg=3.0,
            imu_roll_deg=1.0,
            front_range_m=0.25,
            local_moisture=0.65,
            local_firmness=0.45,
            seeds_remaining=0,
            available_helpers=(
                HelperStatus("rover-2", Pose2D(0.0, 0.0, 0.0), seeds_remaining=0, battery_ratio=1.0, is_operational=True),
                HelperStatus("rover-offline", Pose2D(0.5, 0.5, 0.0), seeds_remaining=0, battery_ratio=0.99, is_operational=False),
                HelperStatus("rover-3", Pose2D(1.2, 0.8, 0.0), seeds_remaining=0, battery_ratio=0.85, is_operational=True),
            ),
            operator_policy=OperatorPolicy(protect_seed_carrier=True, max_helper_loss_risk=0.25),
        )
        decision = self.agent.orchestrate(incident)
        self.assertEqual(decision.verdict, RescueVerdict.RESCUE_NOW)
        self.assertEqual(decision.helper_robot_id, "rover-3")
        self.assertFalse(decision.safety_veto_applied)

        # 2. LLM selecting a non-operational helper or technique='none' on rescue_now must be vetoed
        offline_llm = MockLLMBackend([
            json.dumps({
                "thought": "Dispatching offline helper.",
                "verdict": "rescue_now",
                "chosen_technique": "direct_pry_and_pull",
                "helper_robot_id": "rover-offline",
                "rationale": "Trying offline rover.",
            })
        ])
        offline_decision = SwarmOrchestratorAgent(backend=offline_llm, tool_belt=self.tool_belt).orchestrate(incident)
        self.assertTrue(offline_decision.safety_veto_applied)
        self.assertEqual(offline_decision.verdict, RescueVerdict.ABANDON_ESCALATE)
        self.assertIsNone(offline_decision.helper_robot_id)

        contradictory_llm = MockLLMBackend([
            json.dumps({
                "thought": "Rescue now without any technique.",
                "verdict": "rescue_now",
                "chosen_technique": "none",
                "helper_robot_id": "rover-3",
                "rationale": "Contradictory verdict and technique.",
            })
        ])
        contradictory_decision = SwarmOrchestratorAgent(
            backend=contradictory_llm, tool_belt=self.tool_belt
        ).orchestrate(incident)
        self.assertTrue(contradictory_decision.safety_veto_applied)
        self.assertEqual(contradictory_decision.verdict, RescueVerdict.ABANDON_ESCALATE)
        self.assertIsNone(contradictory_decision.helper_robot_id)

        # 3. Offline peer holding seeds must not mask sole operational seed-carrier protection in deep mud
        deep_mud_incident = IncidentReport(
            incident_id="inc-sole-operational-seed-carrier",
            time_s=25.0,
            stranded_robot_id="rover-2",
            failure_mode=FailureMode.CONTAGIOUS_MUD_TRAP,
            stalled_window_s=5.0,
            slip_ratio=0.92,
            imu_pitch_deg=6.0,
            imu_roll_deg=-2.0,
            front_range_m=0.20,
            local_moisture=0.88,
            local_firmness=0.12,
            seeds_remaining=0,
            available_helpers=(
                HelperStatus("rover-1", Pose2D(1.5, 1.0, 0.0), seeds_remaining=3, is_operational=True),
                HelperStatus("rover-dead", Pose2D(2.0, 2.0, 0.0), seeds_remaining=2, is_operational=False),
            ),
            operator_policy=OperatorPolicy(protect_seed_carrier=True, max_helper_loss_risk=0.25),
        )
        mud_decision = self.agent.orchestrate(deep_mud_incident)
        self.assertEqual(mud_decision.verdict, RescueVerdict.ABANDON_ESCALATE)

    def test_recovery_feedback_abort_revises_technique_and_prevents_repeat_failures(self):
        """Verify aborted recovery feedback revises the technique on retry, vetoes repeat LLM attempts, and abstains when exhausted."""
        incident = IncidentReport(
            incident_id="inc-retry-01",
            time_s=30.0,
            stranded_robot_id="rover-2",
            failure_mode=FailureMode.CONTAGIOUS_MUD_TRAP,
            stalled_window_s=5.0,
            slip_ratio=0.85,
            imu_pitch_deg=5.0,
            imu_roll_deg=2.0,
            front_range_m=0.20,
            local_moisture=0.65,
            local_firmness=0.45,
            seeds_remaining=1,
            available_helpers=(
                HelperStatus("rover-3", Pose2D(1.2, 0.8, 0.0), seeds_remaining=0, battery_ratio=0.95),
            ),
        )

        # Initial attempt selects DIRECT_PRY_AND_PULL
        first = self.agent.orchestrate(incident)
        self.assertEqual(first.verdict, RescueVerdict.RESCUE_NOW)
        self.assertEqual(first.chosen_technique, MechanicalRescueTechnique.DIRECT_PRY_AND_PULL)

        # Simulate safety abort feedback on DIRECT_PRY_AND_PULL
        mon = self.agent.record_recovery_feedback(
            RecoveryFeedback(
                helper_robot_id="rover-3",
                target_robot_id="rover-2",
                task_id="rescue-1",
                latched=False,
                extracted=False,
                victim_displacement_m=0.005,
                observed_at_s=32.0,
                phase="safety_abort_force_or_pitch",
                peak_constraint_force_n=195.0,
                helper_pitch_rad=0.33,
                helper_safe=False,
                technique="direct_pry_and_pull",
            )
        )
        self.assertEqual(mon["status"], "ABORT_AND_REVISE")

        # Next deterministic orchestration must revise to DYNAMIC_YAW_SHEAR_AND_PITCH_PUMP
        second = self.agent.orchestrate(incident)
        self.assertEqual(second.verdict, RescueVerdict.RESCUE_NOW)
        self.assertEqual(second.chosen_technique, MechanicalRescueTechnique.DYNAMIC_YAW_SHEAR_AND_PITCH_PUMP)

        # If an LLM attempts to re-dispatch the already-aborted DIRECT_PRY_AND_PULL, safety gate must veto it
        repeat_llm = MockLLMBackend([
            json.dumps({
                "thought": "Null action_input on inspect_incident.",
                "action": "inspect_incident",
                "action_input": None,
            }),
            json.dumps({
                "thought": "Re-trying direct pry and pull despite prior abort.",
                "verdict": "rescue_now",
                "chosen_technique": "direct_pry_and_pull",
                "helper_robot_id": "rover-3",
                "rationale": "Repeat aborted technique.",
            }),
        ])
        repeat_agent = SwarmOrchestratorAgent(backend=repeat_llm, tool_belt=self.tool_belt)
        repeat_agent.aborted_techniques_by_robot["rover-2"] = {"direct_pry_and_pull"}
        vetoed_repeat = repeat_agent.orchestrate(incident)
        self.assertTrue(vetoed_repeat.safety_veto_applied)
        self.assertEqual(vetoed_repeat.verdict, RescueVerdict.ABANDON_ESCALATE)

        # Once all viable peer extraction techniques have aborted, deterministic reasoner must abstain
        for aborted_tech in ("dynamic_yaw_shear_and_pitch_pump", "bumper_realign_then_pull"):
            self.agent.record_recovery_feedback(
                RecoveryFeedback(
                    helper_robot_id="rover-3",
                    target_robot_id="rover-2",
                    task_id=f"rescue-{aborted_tech}",
                    latched=False,
                    extracted=False,
                    victim_displacement_m=0.004,
                    observed_at_s=35.0,
                    phase="safety_abort_force_or_pitch",
                    peak_constraint_force_n=180.0,
                    helper_pitch_rad=0.32,
                    helper_safe=False,
                    technique=aborted_tech,
                )
            )
        exhausted = self.agent.orchestrate(incident)
        self.assertEqual(exhausted.verdict, RescueVerdict.ABANDON_ESCALATE)
        self.assertEqual(exhausted.chosen_technique, MechanicalRescueTechnique.NONE)

        # A new independent incident (or new episode) must not inherit stale aborted techniques
        later_incident = IncidentReport(
            incident_id="inc-independent-02",
            time_s=60.0,
            stranded_robot_id="rover-2",
            failure_mode=FailureMode.CONTAGIOUS_MUD_TRAP,
            stalled_window_s=3.0,
            slip_ratio=0.82,
            imu_pitch_deg=4.0,
            imu_roll_deg=1.0,
            front_range_m=0.25,
            local_moisture=0.65,
            local_firmness=0.45,
            seeds_remaining=1,
            available_helpers=(
                HelperStatus("rover-3", Pose2D(1.2, 0.8, 0.0), seeds_remaining=0, battery_ratio=0.95),
            ),
        )
        fresh = self.agent.orchestrate(later_incident)
        self.assertEqual(fresh.verdict, RescueVerdict.RESCUE_NOW)
        self.assertEqual(fresh.chosen_technique, MechanicalRescueTechnique.DIRECT_PRY_AND_PULL)

        # Successful extraction also clears the aborted technique history for that robot
        self.agent.aborted_techniques_by_robot["rover-2"] = {"direct_pry_and_pull"}
        self.agent.record_recovery_feedback(
            RecoveryFeedback(
                helper_robot_id="rover-3",
                target_robot_id="rover-2",
                task_id="rescue-extracted",
                latched=True,
                extracted=True,
                victim_displacement_m=0.20,
                observed_at_s=75.0,
                phase="extracted_and_unlatched",
                peak_constraint_force_n=45.0,
                helper_pitch_rad=0.05,
                helper_safe=True,
                technique="direct_pry_and_pull",
            )
        )
        self.assertNotIn("rover-2", self.agent.aborted_techniques_by_robot)

    def test_rollover_stabilization_vs_pitch_match_extraction_action_semantics(self):
        """Verify TIP_OVER_OR_TRENCH commands victim self-stabilization (ABANDON_ESCALATE) while PITCH_NOSE_DOWN_STALL authorizes physical peer extraction."""
        controller = LocalController()

        # 1. Severe rollover / trench incident (TIP_OVER_OR_TRENCH):
        # SPADE_ANCHOR_ROLLOVER_HOLD stops wheels, deploys spades (0.85 rad), and leaves latch open (0.0 rad).
        # It is a victim self-stabilization hold, NOT a peer towing action.
        trench_incident = IncidentReport(
            incident_id="inc-trench-rollover-01",
            time_s=70.0,
            stranded_robot_id="rover-2",
            failure_mode=FailureMode.TIP_OVER_OR_TRENCH,
            stalled_window_s=4.0,
            slip_ratio=0.75,
            imu_pitch_deg=34.0,
            imu_roll_deg=24.0,
            front_range_m=0.30,
            local_moisture=0.60,
            local_firmness=0.50,
            seeds_remaining=1,
            available_helpers=(
                HelperStatus("rover-3", Pose2D(1.0, 0.8, 0.0), seeds_remaining=0, battery_ratio=0.95),
            ),
            hazard_risk=0.4,
        )
        trench_decision = self.agent.orchestrate(trench_incident)
        self.assertEqual(trench_decision.verdict, RescueVerdict.ABANDON_ESCALATE)
        self.assertEqual(trench_decision.chosen_technique, MechanicalRescueTechnique.SPADE_ANCHOR_ROLLOVER_HOLD)
        self.assertIsNone(trench_decision.helper_robot_id)
        self.assertIsNone(trench_decision.mechanical_setpoint)

        trench_assessment = self.agent.to_rescue_assessment(trench_incident, trench_decision)
        self.assertEqual(trench_assessment.verdict, RescueVerdict.ABANDON_ESCALATE)
        self.assertEqual(trench_assessment.recovery_action, RecoveryAction.SAFE_ABSTAIN_DEFER_HUMAN)
        self.assertEqual(trench_assessment.mechanical_technique, MechanicalRescueTechnique.SPADE_ANCHOR_ROLLOVER_HOLD)
        self.assertIsNone(trench_assessment.helper_robot_id)

        # Verify actual low-level controller semantics for the stranded victim in HOLD with spade_anchor_rollover_hold
        victim_obs = RobotObservation(
            robot_id="rover-2",
            pose=Pose2D(0.0, 0.0, 0.0),
            time_s=70.0,
            front_range_m=0.30,
            seeds_remaining=1,
            roll_rad=0.42,
            is_bogged=True,
        )
        victim_hold_cmd = controller.command(
            victim_obs,
            TaskIntent(
                task_id="hold-rover-2",
                robot_id="rover-2",
                mode=TaskMode.HOLD,
                issued_at_s=70.0,
                expires_at_s=71.0,
                target_site_id=None,
                reason="rollover self-stabilization",
                recovery_technique=trench_decision.chosen_technique.value,
            ),
            {},
            RobotCapability("rover-2", can_sample=True, can_plant=True),
        )
        self.assertEqual(victim_hold_cmd.linear_m_s, 0.0)
        self.assertEqual(victim_hold_cmd.angular_rad_s, 0.0)
        self.assertAlmostEqual(victim_hold_cmd.spade_rad, 0.85, places=3)
        self.assertAlmostEqual(victim_hold_cmd.latch_rad, 0.0, places=3)

        # Tool belt must reject spade_anchor_rollover_hold as a peer extraction skill
        auth_err = self.tool_belt.authorize_recovery_skill(
            trench_incident,
            technique_str="spade_anchor_rollover_hold",
            helper_id="rover-3",
        )
        self.assertIn("error", auth_err)

        # 2. LLM attempting RESCUE_NOW with SPADE_ANCHOR_ROLLOVER_HOLD or on TIP_OVER_OR_TRENCH must be vetoed
        rollover_llm = MockLLMBackend([
            json.dumps({
                "thought": "Misinterpreting rollover stabilization hold as peer extraction.",
                "verdict": "rescue_now",
                "chosen_technique": "spade_anchor_rollover_hold",
                "helper_robot_id": "rover-3",
                "rationale": "Dispatching helper with rollover hold.",
            })
        ])
        vetoed_trench = SwarmOrchestratorAgent(
            backend=rollover_llm, tool_belt=self.tool_belt
        ).orchestrate(trench_incident)
        self.assertTrue(vetoed_trench.safety_veto_applied)
        self.assertEqual(vetoed_trench.verdict, RescueVerdict.ABANDON_ESCALATE)
        self.assertEqual(vetoed_trench.chosen_technique, MechanicalRescueTechnique.SPADE_ANCHOR_ROLLOVER_HOLD)
        self.assertIsNone(vetoed_trench.helper_robot_id)
        self.assertIsNone(vetoed_trench.mechanical_setpoint)

        # Even on a mud-trap incident, LLM attempting RESCUE_NOW with a non-extraction skill (spade_anchor_rollover_hold / self_oscillate_dibber) is vetoed
        mud_incident = IncidentReport(
            incident_id="inc-mud-non-extraction-veto",
            time_s=72.0,
            stranded_robot_id="rover-2",
            failure_mode=FailureMode.CONTAGIOUS_MUD_TRAP,
            stalled_window_s=4.0,
            slip_ratio=0.82,
            imu_pitch_deg=5.0,
            imu_roll_deg=2.0,
            front_range_m=0.25,
            local_moisture=0.65,
            local_firmness=0.45,
            seeds_remaining=1,
            available_helpers=(
                HelperStatus("rover-3", Pose2D(1.0, 0.8, 0.0), seeds_remaining=0, battery_ratio=0.95),
            ),
        )
        for non_peer_tech in ("spade_anchor_rollover_hold", "self_oscillate_dibber"):
            bad_llm = MockLLMBackend([
                json.dumps({
                    "thought": "Using self-only skill for peer rescue.",
                    "verdict": "rescue_now",
                    "chosen_technique": non_peer_tech,
                    "helper_robot_id": "rover-3",
                    "rationale": "Invalid peer technique.",
                })
            ])
            vetoed_non_peer = SwarmOrchestratorAgent(
                backend=bad_llm, tool_belt=self.tool_belt
            ).orchestrate(mud_incident)
            self.assertTrue(vetoed_non_peer.safety_veto_applied)
            self.assertEqual(vetoed_non_peer.verdict, RescueVerdict.ABANDON_ESCALATE)
            self.assertEqual(vetoed_non_peer.chosen_technique, MechanicalRescueTechnique.NONE)
            self.assertIsNone(vetoed_non_peer.helper_robot_id)
            self.assertIsNone(vetoed_non_peer.mechanical_setpoint)

        # 3. Recoverable moderate nose-down pitch stall (PITCH_NOSE_DOWN_STALL):
        # Supported by PITCH_MATCH_NOSE_DOWN_RECOVERY with engaged cam-latch and reverse wheel traction.
        pitch_incident = IncidentReport(
            incident_id="inc-pitch-nose-down-01",
            time_s=80.0,
            stranded_robot_id="rover-2",
            failure_mode=FailureMode.PITCH_NOSE_DOWN_STALL,
            stalled_window_s=4.0,
            slip_ratio=0.80,
            imu_pitch_deg=-22.0,
            imu_roll_deg=3.0,
            front_range_m=0.28,
            local_moisture=0.60,
            local_firmness=0.48,
            seeds_remaining=1,
            available_helpers=(
                HelperStatus("rover-3", Pose2D(0.55, 0.0, 0.0), seeds_remaining=0, battery_ratio=0.95),
            ),
            hazard_risk=0.4,
        )
        pitch_decision = self.agent.orchestrate(pitch_incident)
        self.assertEqual(pitch_decision.verdict, RescueVerdict.RESCUE_NOW)
        self.assertFalse(pitch_decision.safety_veto_applied)
        self.assertEqual(pitch_decision.helper_robot_id, "rover-3")
        self.assertEqual(
            pitch_decision.chosen_technique,
            MechanicalRescueTechnique.PITCH_MATCH_NOSE_DOWN_RECOVERY,
        )
        self.assertIsNotNone(pitch_decision.mechanical_setpoint)
        setpoint = pitch_decision.mechanical_setpoint
        assert setpoint is not None
        self.assertLess(setpoint.dock_pitch_rad, 0.0)
        self.assertGreater(setpoint.latch_rad, 0.30)
        self.assertGreater(setpoint.spade_rad, 0.0)
        self.assertTrue(all(w < 0.0 for w in setpoint.wheel_vel_rad_s))

        clamped_pitch, clamped_latch, clamped_spade = (
            self.agent.translate_setpoint_to_command_bounds(setpoint)
        )
        self.assertAlmostEqual(clamped_pitch, -0.10, places=3)
        self.assertGreater(clamped_latch, 0.30)
        self.assertAlmostEqual(clamped_spade, 0.35, places=3)

        helper_obs = RobotObservation(
            robot_id="rover-3",
            pose=Pose2D(0.25, 0.0, 3.14159),
            time_s=80.0,
            front_range_m=0.25,
            seeds_remaining=0,
        )
        rescue_cmd = controller.command(
            helper_obs,
            TaskIntent(
                task_id="rescue-rover-3",
                robot_id="rover-3",
                mode=TaskMode.RESCUE,
                issued_at_s=80.0,
                expires_at_s=81.0,
                target_site_id="s-1",
                target_robot_id="rover-2",
                reason="pitch-matched peer extraction",
                recovery_technique=pitch_decision.chosen_technique.value,
            ),
            {},
            RobotCapability("rover-3", can_sample=True, can_plant=False),
            peer_pose=Pose2D(0.0, 0.0, 0.0),
        )
        self.assertGreater(rescue_cmd.linear_m_s, 0.0)
        self.assertAlmostEqual(rescue_cmd.dock_pitch_rad, 0.14, places=3)
        self.assertGreater(rescue_cmd.latch_rad, 0.30)
        self.assertAlmostEqual(rescue_cmd.spade_rad, 0.25, places=3)


if __name__ == "__main__":
    unittest.main()
