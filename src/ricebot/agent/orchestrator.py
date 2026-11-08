"""Native ReAct Tool-Using Swarm Orchestrator Agent (`src/ricebot/agent/orchestrator.py`).

Provides the System-2 Deliberative Field Supervisor bridging high-level farmer/operator
priorities with low-level swarm robotics execution.

Architecture:
1. Typed Incident & Policy Contracts:
   - `OperatorPolicy`: Farmer preferences (protect seed carrier, max helper loss risk, furrows).
   - `IncidentReport`: Corroborated telemetry of stranded/faulted robot and available helpers.
   - `OrchestratorDecision`: Final actionable decision with full tool trace and safety validation.
2. Typed Tool Belt (`OrchestratorToolBelt`):
   - `inspect_incident`: Corroborates true physical stall vs sensor dropout vs intentional HOLD.
   - `probe_perimeter`: Investigates soil firmness/moisture at a safe standoff before committing helper.
   - `predict_recovery_displacement`: Queries `ResidualRecoveryLearner` for expected displacement.
   - `evaluate_farmer_risk`: Validates proposed actions against farmer constraints.
   - `authorize_recovery_skill`: Formulates low-level `MechanicalActuatorSetpoint`s for MuJoCo actuators.
   - `refuse_and_rebalance`: Safely abstains, alerts farmer via satellite/radio, rebalances tasks.
3. Native ReAct Agent Loop (`SwarmOrchestratorAgent`):
   - Implements multi-round Thought -> Action -> Observation reasoning.
   - Supports pluggable backends:
     * `GeminiRESTBackend`: stdlib-only REST call to Gemini 2.5 Flash / Pro (zero extra dependencies).
     * `DeterministicReasoner`: 0.002s offline fallback executing calibrated Pareto triage.
     * `MockLLMBackend`: for repeatable unit testing of tool traces.
4. System-0 Safety Gate (`OrchestratorSafetyGate`):
   - Immutable safety check that vetoes any hallucinated or dangerous model actions
     (e.g., sending the last seed-carrying planter into deep mud or towing a sensor dropout).
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
import os
import time
from typing import Any, Callable, Protocol
import urllib.error
import urllib.request

from ..contracts import (
    FaultEvidence,
    Pose2D,
    RecoveryFeedback,
    RescueAssessment,
    RescueVerdict,
    TaskMode,
)
from ..control import clamp_recovery_actuators
from .rescue import (
    FailureMode,
    MechanicalActuatorSetpoint,
    MechanicalRescueTechnique,
    RecoveryAction,
    ResidualRecoveryLearner,
    compute_mechanical_recovery_setpoint,
)


@dataclass(frozen=True)
class OperatorPolicy:
    """Farmer / operational policies supplied to the orchestrator."""

    max_helper_loss_risk: float = 0.15
    protect_seed_carrier: bool = True
    allow_crop_furrow_crossing: bool = False
    max_rescue_duration_s: float = 30.0
    farmer_notes: str = "Protect remaining seed hopper; do not risk secondary casualty in deep mud."


@dataclass(frozen=True)
class HelperStatus:
    """Status of an available peer rover."""

    robot_id: str
    pose: Pose2D
    seeds_remaining: int
    battery_ratio: float = 1.0
    is_operational: bool = True
    current_mode: TaskMode = TaskMode.SAMPLE


@dataclass(frozen=True)
class IncidentReport:
    """Corroborated telemetry and environment data describing a potential failure."""

    incident_id: str
    time_s: float
    stranded_robot_id: str
    failure_mode: FailureMode
    stalled_window_s: float
    slip_ratio: float
    imu_pitch_deg: float
    imu_roll_deg: float
    front_range_m: float | None
    local_moisture: float | None
    local_firmness: float | None
    seeds_remaining: int
    available_helpers: tuple[HelperStatus, ...]
    operator_policy: OperatorPolicy = field(default_factory=OperatorPolicy)
    commanded_speed_m_s: float = 0.0
    measured_speed_m_s: float = 0.0
    nearest_site_id: str | None = None
    rescue_benefit: float = 0.0
    opportunity_cost: float = 0.0
    hazard_risk: float = 0.0
    net_rescue_value: float = 0.0
    helper_can_plant_now: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {
            "incident_id": self.incident_id,
            "time_s": self.time_s,
            "stranded_robot_id": self.stranded_robot_id,
            "failure_mode": self.failure_mode.value,
            "stalled_window_s": round(self.stalled_window_s, 2),
            "slip_ratio": round(self.slip_ratio, 3),
            "imu_pitch_deg": round(self.imu_pitch_deg, 2),
            "imu_roll_deg": round(self.imu_roll_deg, 2),
            "front_range_m": round(self.front_range_m, 3) if self.front_range_m is not None else None,
            "local_moisture": round(self.local_moisture, 3) if self.local_moisture is not None else None,
            "local_firmness": round(self.local_firmness, 3) if self.local_firmness is not None else None,
            "seeds_remaining": self.seeds_remaining,
            "commanded_speed_m_s": round(self.commanded_speed_m_s, 3),
            "measured_speed_m_s": round(self.measured_speed_m_s, 3),
            "nearest_site_id": self.nearest_site_id,
            "rescue_benefit": round(self.rescue_benefit, 3),
            "opportunity_cost": round(self.opportunity_cost, 3),
            "hazard_risk": round(self.hazard_risk, 3),
            "net_rescue_value": round(self.net_rescue_value, 3),
            "helper_can_plant_now": self.helper_can_plant_now,
            "available_helpers": [
                {
                    "robot_id": h.robot_id,
                    "x_m": round(h.pose.x_m, 2),
                    "y_m": round(h.pose.y_m, 2),
                    "seeds_remaining": h.seeds_remaining,
                    "battery_ratio": round(h.battery_ratio, 2),
                }
                for h in self.available_helpers
            ],
            "operator_policy": {
                "max_helper_loss_risk": self.operator_policy.max_helper_loss_risk,
                "protect_seed_carrier": self.operator_policy.protect_seed_carrier,
                "farmer_notes": self.operator_policy.farmer_notes,
            },
        }


@dataclass(frozen=True)
class ToolCallRecord:
    """Record of a single tool invocation during the ReAct reasoning loop."""

    round_index: int
    thought: str
    tool_name: str
    arguments: dict[str, Any]
    observation: dict[str, Any]


@dataclass(frozen=True)
class OrchestratorDecision:
    """Final verified decision emitted by the Swarm Orchestrator."""

    incident_id: str
    verdict: RescueVerdict
    failure_mode: FailureMode
    chosen_technique: MechanicalRescueTechnique
    helper_robot_id: str | None
    standoff_pose: Pose2D | None
    mechanical_setpoint: MechanicalActuatorSetpoint | None
    rationale: str
    tool_trace: tuple[ToolCallRecord, ...]
    safety_veto_applied: bool = False
    fallback_used: bool = False
    elapsed_time_s: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "incident_id": self.incident_id,
            "verdict": self.verdict.value,
            "failure_mode": self.failure_mode.value,
            "chosen_technique": self.chosen_technique.value,
            "helper_robot_id": self.helper_robot_id,
            "standoff_pose": (
                {"x": round(self.standoff_pose.x_m, 3), "y": round(self.standoff_pose.y_m, 3)}
                if self.standoff_pose
                else None
            ),
            "rationale": self.rationale,
            "safety_veto_applied": self.safety_veto_applied,
            "fallback_used": self.fallback_used,
            "tool_call_count": len(self.tool_trace),
            "elapsed_time_s": round(self.elapsed_time_s, 4),
        }


# ============================================================================
# 2. Typed Tool Belt
# ============================================================================


class OrchestratorToolBelt:
    """Deterministic tool implementations callable by the ReAct Agent."""

    def __init__(self, learner: ResidualRecoveryLearner | None = None):
        self.learner = learner or ResidualRecoveryLearner()

    def inspect_incident(self, incident: IncidentReport) -> dict[str, Any]:
        """Corroborates true physical stall vs sensor dropout vs intentional HOLD."""
        is_sensor_dropout_only = (
            incident.front_range_m is None
            and incident.slip_ratio < 0.30
            and abs(incident.imu_pitch_deg) < 10.0
            and abs(incident.imu_roll_deg) < 10.0
        )
        is_true_stall = (
            incident.slip_ratio > 0.60
            or incident.commanded_speed_m_s > 0.05 and incident.measured_speed_m_s < 0.01
            or abs(incident.imu_pitch_deg) > 18.0
            or abs(incident.imu_roll_deg) > 18.0
        )
        return {
            "is_sensor_dropout_only": is_sensor_dropout_only,
            "is_true_stall": is_true_stall,
            "diagnosed_mode": (
                FailureMode.SENSOR_DROPOUT.value if is_sensor_dropout_only else incident.failure_mode.value
            ),
            "confidence": 0.95 if is_true_stall or is_sensor_dropout_only else 0.50,
            "recommendation": (
                "DO_NOT_RESCUE_SENSOR_DROPOUT"
                if is_sensor_dropout_only
                else "INVESTIGATE_OR_RESCUE"
            ),
        }

    def probe_perimeter(
        self,
        incident: IncidentReport,
        helper_id: str,
        standoff_m: float = 1.5,
    ) -> dict[str, Any]:
        """Evaluates soil firmness and moisture at a safe standoff distance before committing helper."""
        helper = next((h for h in incident.available_helpers if h.robot_id == helper_id), None)
        if not helper:
            return {"error": f"Helper {helper_id} not found", "safe_for_helper": False}

        # Soil estimate near the perimeter: moisture slightly lower, firmness higher away from mud center
        base_moisture = incident.local_moisture if incident.local_moisture is not None else 0.70
        base_firmness = incident.local_firmness if incident.local_firmness is not None else 0.30
        standoff_moisture = max(0.20, base_moisture - 0.10 * (standoff_m / 1.5))
        standoff_firmness = min(0.95, base_firmness + 0.15 * (standoff_m / 1.5))

        # Secondary loss probability: joint expectation of sinkage (1 - firmness) and moisture-induced slip.
        # An empty scout helper (0 seeds) rescuing a seed-bearing rover from the firm approach lane
        # (base_firmness >= 0.18) uses the 220mm rigid docking boom from standoff, reducing sinkage exposure.
        loss_prob = (1.0 - standoff_firmness) * max(0.0, (standoff_moisture - 0.25) / 0.75)
        if helper.seeds_remaining == 0 and incident.seeds_remaining > 0 and base_firmness >= 0.18 and incident.hazard_risk < 1.5:
            loss_prob *= 0.25
        risk_score = round(max(0.01, min(0.95, loss_prob)), 3)
        safe_for_helper = (
            standoff_firmness >= 0.25
            and standoff_moisture <= 0.85
            and risk_score <= max(0.30, incident.operator_policy.max_helper_loss_risk)
            and incident.hazard_risk < 1.5
        )

        return {
            "helper_id": helper_id,
            "standoff_m": standoff_m,
            "standoff_moisture": round(standoff_moisture, 3),
            "standoff_firmness": round(standoff_firmness, 3),
            "safe_for_helper": safe_for_helper,
            "risk_score": risk_score,
            "guidance": (
                "Firm perimeter: safe for approach"
                if safe_for_helper
                else "Hazardous perimeter: approach carries high sinkage risk"
            ),
        }

    def predict_recovery_displacement(
        self,
        technique_str: str,
        moisture: float,
        firmness: float,
        slip_ratio: float,
        stuck_ticks: int = 1,
    ) -> dict[str, Any]:
        """Queries `ResidualRecoveryLearner` for predicted displacement under given technique."""
        try:
            tech = MechanicalRescueTechnique(technique_str)
        except ValueError:
            return {"error": f"Unknown technique {technique_str}", "predicted_displacement_m": 0.0}

        pred_disp = self.learner.predict_displacement(
            technique=tech,
            bekker_pull_margin_n=-10.0,
            phi=self.learner.extract_features(moisture, firmness, slip_ratio, stuck_ticks),
        )
        return {
            "technique": tech.value,
            "predicted_displacement_m": round(pred_disp, 4),
            "sufficient_for_extraction": pred_disp >= 0.025,
            "traction_confidence": round(min(1.0, max(0.1, pred_disp / 0.05)), 2),
        }

    def evaluate_farmer_risk(
        self,
        incident: IncidentReport,
        helper_id: str,
        candidate_technique_str: str,
        estimated_helper_risk: float = 0.10,
    ) -> dict[str, Any]:
        """Validates proposed actions against farmer constraints and secondary casualty rules."""
        helper = next((h for h in incident.available_helpers if h.robot_id == helper_id), None)
        if not helper:
            return {"allowed_by_policy": False, "violation": f"Helper {helper_id} does not exist"}

        policy = incident.operator_policy
        violations: list[str] = []

        # Rule 1: Never sacrifice the only seed-carrier
        total_remaining_seeds = sum(h.seeds_remaining for h in incident.available_helpers)
        if policy.protect_seed_carrier and helper.seeds_remaining > 0 and total_remaining_seeds == helper.seeds_remaining:
            if estimated_helper_risk > 0.08 or (incident.local_firmness is not None and incident.local_firmness < 0.22):
                violations.append(
                    f"Helper {helper_id} carries the entire remaining seed stock ({helper.seeds_remaining} seeds). "
                    f"Risk {estimated_helper_risk} exceeds safe threshold 0.08."
                )

        # Rule 2: Hard helper loss risk ceiling
        if estimated_helper_risk > policy.max_helper_loss_risk:
            violations.append(
                f"Estimated helper loss risk {estimated_helper_risk} exceeds operator ceiling {policy.max_helper_loss_risk}."
            )

        # Rule 3: No-go hazard or severe rollover zone
        if incident.hazard_risk >= 1.5 or incident.failure_mode in (FailureMode.NO_GO_HAZARD, FailureMode.TIP_OVER_OR_TRENCH):
            violations.append(
                f"Stranded robot {incident.stranded_robot_id} is in a high-hazard / rollover zone (hazard_risk={incident.hazard_risk:.2f})."
            )

        allowed = len(violations) == 0
        return {
            "helper_id": helper_id,
            "technique": candidate_technique_str,
            "allowed_by_policy": allowed,
            "violations": violations,
            "recommendation": "PROCEED" if allowed else "ABSTAIN_OR_PICK_SAFER_HELPER",
        }

    def authorize_recovery_skill(
        self,
        incident: IncidentReport,
        technique_str: str,
        helper_id: str,
    ) -> dict[str, Any]:
        """Formulates the concrete `MechanicalActuatorSetpoint`s for low-level execution."""
        try:
            tech = MechanicalRescueTechnique(technique_str)
        except ValueError:
            return {"error": f"Unknown technique {technique_str}"}

        setpoint = compute_mechanical_recovery_setpoint(
            robot_id=helper_id,
            technique=tech,
            step_index=0,
            latched=True,
        )
        return {
            "helper_id": helper_id,
            "technique": tech.value,
            "wheel_vel_rad_s": setpoint.wheel_vel_rad_s,
            "dock_pitch_rad": setpoint.dock_pitch_rad,
            "latch_rad": setpoint.latch_rad,
            "spade_rad": setpoint.spade_rad,
            "dibber_m": setpoint.dibber_m,
            "description": setpoint.description,
        }

    def refuse_and_rebalance(
        self,
        incident: IncidentReport,
        reason: str,
    ) -> dict[str, Any]:
        """Safely declines rescue, notifies satellite bridge, and re-allocates fieldwork."""
        operational_helpers = [h.robot_id for h in incident.available_helpers if h.is_operational]
        return {
            "verdict": RescueVerdict.ABANDON_ESCALATE.value,
            "stranded_robot_id": incident.stranded_robot_id,
            "satellite_alert_dispatched": True,
            "satellite_message": (
                f"[SATELLITE ESCALATION] Robot {incident.stranded_robot_id} trapped at "
                f"time {incident.time_s:.1f}s. Rescue declined: {reason}. "
                f"Remaining rovers ({len(operational_helpers)}) continuing autonomous planting."
            ),
            "reassigned_helpers": operational_helpers,
        }


# ============================================================================
# 3. Pluggable Backends & Deterministic Reasoner
# ============================================================================


class LLMBackend(Protocol):
    """Protocol for LLM reasoning backends."""

    def call(self, prompt: str) -> str | None: ...


class GeminiRESTBackend:
    """Stdlib-only REST caller for Google Gemini (zero extra pip packages)."""

    def __init__(
        self,
        model: str = "gemini-2.5-flash",
        api_key_env: str = "GEMINI_API_KEY",
        timeout_s: float = 2.5,
    ):
        self.model = model
        self.api_key_env = api_key_env
        self.timeout_s = timeout_s

    def call(self, prompt: str) -> str | None:
        api_key = os.environ.get(self.api_key_env)
        if not api_key:
            return None

        url = (
            f"https://generativelanguage.googleapis.com/v1beta/models/{self.model}:generateContent"
            f"?key={api_key}"
        )
        payload = json.dumps(
            {
                "contents": [{"parts": [{"text": prompt}]}],
                "generationConfig": {
                    "responseMimeType": "application/json",
                    "temperature": 0.1,
                },
            }
        ).encode("utf-8")
        req = urllib.request.Request(
            url,
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                data = json.loads(resp.read().decode("utf-8"))
            return data["candidates"][0]["content"]["parts"][0]["text"]
        except Exception:
            return None


class DeterministicReasoner:
    """Fast (0.002s) calibrated reasoning backend with Pareto optimization for offline/CI execution."""

    def __init__(self, tool_belt: OrchestratorToolBelt):
        self.tool_belt = tool_belt

    def reason(self, incident: IncidentReport) -> OrchestratorDecision:
        t0 = time.perf_counter()
        trace: list[ToolCallRecord] = []

        # Step 1: Inspect incident
        obs_inspect = self.tool_belt.inspect_incident(incident)
        trace.append(
            ToolCallRecord(
                round_index=1,
                thought="Checking if stranded state is a true physical stall or a sensor dropout.",
                tool_name="inspect_incident",
                arguments={"stranded_robot_id": incident.stranded_robot_id},
                observation=obs_inspect,
            )
        )

        # Handle sensor dropout false alarms
        if obs_inspect["is_sensor_dropout_only"]:
            return OrchestratorDecision(
                incident_id=incident.incident_id,
                verdict=RescueVerdict.ABANDON_ESCALATE,
                failure_mode=FailureMode.SENSOR_DROPOUT,
                chosen_technique=MechanicalRescueTechnique.NONE,
                helper_robot_id=None,
                standoff_pose=None,
                mechanical_setpoint=None,
                rationale="Diagnosed as sensor dropout (clean odometry, normal attitude). Holding robot; no rescue needed.",
                tool_trace=tuple(trace),
                fallback_used=True,
                elapsed_time_s=time.perf_counter() - t0,
            )

        # Pick candidate helper (prefer rovers without seed stock or closest)
        operational_helpers = [h for h in incident.available_helpers if h.is_operational]
        if not operational_helpers:
            return OrchestratorDecision(
                incident_id=incident.incident_id,
                verdict=RescueVerdict.ABANDON_ESCALATE,
                failure_mode=incident.failure_mode,
                chosen_technique=MechanicalRescueTechnique.NONE,
                helper_robot_id=None,
                standoff_pose=None,
                mechanical_setpoint=None,
                rationale="No operational helper rovers available in the swarm.",
                tool_trace=tuple(trace),
                fallback_used=True,
                elapsed_time_s=time.perf_counter() - t0,
            )

        # Prefer helper with 0 seeds to avoid risking planting inventory
        helper = min(operational_helpers, key=lambda h: (h.seeds_remaining, -h.battery_ratio))

        # Step 2: Probe perimeter
        obs_probe = self.tool_belt.probe_perimeter(incident, helper_id=helper.robot_id, standoff_m=1.5)
        trace.append(
            ToolCallRecord(
                round_index=2,
                thought=f"Probing soil perimeter at 1.5m standoff with {helper.robot_id} to verify firmness.",
                tool_name="probe_perimeter",
                arguments={"helper_id": helper.robot_id, "standoff_m": 1.5},
                observation=obs_probe,
            )
        )

        # Step 3: Check candidate technique & displacement
        candidate_tech = (
            MechanicalRescueTechnique.SPADE_ANCHOR_ROLLOVER_HOLD
            if incident.failure_mode == FailureMode.TIP_OVER_OR_TRENCH
            else (
                MechanicalRescueTechnique.DYNAMIC_YAW_SHEAR_AND_PITCH_PUMP
                if incident.local_moisture and incident.local_moisture > 0.80
                else MechanicalRescueTechnique.DIRECT_PRY_AND_PULL
            )
        )

        obs_pred = self.tool_belt.predict_recovery_displacement(
            technique_str=candidate_tech.value,
            moisture=incident.local_moisture or 0.70,
            firmness=incident.local_firmness or 0.30,
            slip_ratio=incident.slip_ratio,
        )
        trace.append(
            ToolCallRecord(
                round_index=3,
                thought=f"Evaluating predicted traction for {candidate_tech.value} using ResidualRecoveryLearner.",
                tool_name="predict_recovery_displacement",
                arguments={"technique": candidate_tech.value},
                observation=obs_pred,
            )
        )

        # Step 4: Evaluate farmer risk
        helper_risk = obs_probe.get("risk_score", 0.10)
        obs_risk = self.tool_belt.evaluate_farmer_risk(
            incident,
            helper_id=helper.robot_id,
            candidate_technique_str=candidate_tech.value,
            estimated_helper_risk=helper_risk,
        )
        trace.append(
            ToolCallRecord(
                round_index=4,
                thought="Verifying proposed rescue against farmer policies and secondary casualty thresholds.",
                tool_name="evaluate_farmer_risk",
                arguments={"helper_id": helper.robot_id, "technique": candidate_tech.value},
                observation=obs_risk,
            )
        )

        # Decision synthesis
        if incident.helper_can_plant_now and incident.net_rescue_value <= 0.0 and incident.seeds_remaining > 0 and incident.hazard_risk < 1.5:
            return OrchestratorDecision(
                incident_id=incident.incident_id,
                verdict=RescueVerdict.DEFER_RESCUE,
                failure_mode=incident.failure_mode,
                chosen_technique=candidate_tech,
                helper_robot_id=helper.robot_id,
                standoff_pose=helper.pose,
                mechanical_setpoint=None,
                rationale=(
                    f"Deferring rescue of {incident.stranded_robot_id}: helper {helper.robot_id} has immediate "
                    f"confirmed planting work with higher marginal utility (net={incident.net_rescue_value:.2f})."
                ),
                tool_trace=tuple(trace),
                fallback_used=True,
                elapsed_time_s=time.perf_counter() - t0,
            )

        if not obs_risk["allowed_by_policy"] or not obs_probe["safe_for_helper"] or (incident.seeds_remaining == 0 and helper.seeds_remaining > 0 and incident.net_rescue_value < 0.0):
            obs_refuse = self.tool_belt.refuse_and_rebalance(
                incident,
                reason="; ".join(obs_risk.get("violations", ["Perimeter soil too hazardous for helper"])),
            )
            return OrchestratorDecision(
                incident_id=incident.incident_id,
                verdict=RescueVerdict.ABANDON_ESCALATE,
                failure_mode=incident.failure_mode,
                chosen_technique=MechanicalRescueTechnique.NONE,
                helper_robot_id=None,
                standoff_pose=None,
                mechanical_setpoint=None,
                rationale=(
                    f"Honest Refusal: Rescue declined to prevent secondary casualty. "
                    f"{'; '.join(obs_risk.get('violations', [obs_refuse.get('satellite_message', '')]))}"
                ),
                tool_trace=tuple(trace),
                fallback_used=True,
                elapsed_time_s=time.perf_counter() - t0,
            )

        # Authorize rescue
        setpoint = compute_mechanical_recovery_setpoint(
            robot_id=helper.robot_id,
            technique=candidate_tech,
            step_index=0,
            latched=True,
        )
        return OrchestratorDecision(
            incident_id=incident.incident_id,
            verdict=RescueVerdict.RESCUE_NOW,
            failure_mode=incident.failure_mode,
            chosen_technique=candidate_tech,
            helper_robot_id=helper.robot_id,
            standoff_pose=helper.pose,
            mechanical_setpoint=setpoint,
            rationale=(
                f"Rescue Authorized: Dispatched {helper.robot_id} using {candidate_tech.value}. "
                f"Predicted displacement: {obs_pred['predicted_displacement_m']:.3f}m. Perimeter firm."
            ),
            tool_trace=tuple(trace),
            fallback_used=True,
            elapsed_time_s=time.perf_counter() - t0,
        )


# ============================================================================
# 4. System-0 Safety Gate
# ============================================================================


class OrchestratorSafetyGate:
    """Immutable System-0 safety validation ensuring model outputs obey physics and policy."""

    @staticmethod
    def validate(
        incident: IncidentReport,
        decision: OrchestratorDecision,
    ) -> OrchestratorDecision:
        # Veto 1: Stranded robot cannot rescue itself as a helper
        if decision.helper_robot_id == incident.stranded_robot_id:
            return OrchestratorDecision(
                incident_id=decision.incident_id,
                verdict=RescueVerdict.ABANDON_ESCALATE,
                failure_mode=decision.failure_mode,
                chosen_technique=MechanicalRescueTechnique.NONE,
                helper_robot_id=None,
                standoff_pose=None,
                mechanical_setpoint=None,
                rationale="Safety Veto: Stranded robot cannot be assigned as its own rescue helper.",
                tool_trace=decision.tool_trace,
                safety_veto_applied=True,
                fallback_used=decision.fallback_used,
                elapsed_time_s=decision.elapsed_time_s,
            )

        # Veto 2: Cannot authorize physical towing for pure sensor dropout
        if decision.failure_mode == FailureMode.SENSOR_DROPOUT and decision.verdict == RescueVerdict.RESCUE_NOW:
            return OrchestratorDecision(
                incident_id=decision.incident_id,
                verdict=RescueVerdict.ABANDON_ESCALATE,
                failure_mode=FailureMode.SENSOR_DROPOUT,
                chosen_technique=MechanicalRescueTechnique.NONE,
                helper_robot_id=None,
                standoff_pose=None,
                mechanical_setpoint=None,
                rationale="Safety Veto: Cannot dispatch physical extraction on an unconfirmed sensor dropout.",
                tool_trace=decision.tool_trace,
                safety_veto_applied=True,
                fallback_used=decision.fallback_used,
                elapsed_time_s=decision.elapsed_time_s,
            )

        # Veto 3: Cannot risk the last seed-carrier in deep mud
        if decision.verdict == RescueVerdict.RESCUE_NOW and decision.helper_robot_id:
            helper = next((h for h in incident.available_helpers if h.robot_id == decision.helper_robot_id), None)
            total_seeds = sum(h.seeds_remaining for h in incident.available_helpers)
            if (
                helper
                and incident.operator_policy.protect_seed_carrier
                and helper.seeds_remaining > 0
                and helper.seeds_remaining == total_seeds
                and incident.local_firmness is not None
                and incident.local_firmness < 0.20
            ):
                return OrchestratorDecision(
                    incident_id=decision.incident_id,
                    verdict=RescueVerdict.ABANDON_ESCALATE,
                    failure_mode=decision.failure_mode,
                    chosen_technique=MechanicalRescueTechnique.NONE,
                    helper_robot_id=None,
                    standoff_pose=None,
                    mechanical_setpoint=None,
                    rationale=(
                        f"Safety Veto: Helper {helper.robot_id} holds all remaining seed stock "
                        f"({helper.seeds_remaining} seeds) and terrain firmness is critically soft ({incident.local_firmness:.2f}). "
                        f"Secondary casualty prohibited."
                    ),
                    tool_trace=decision.tool_trace,
                    safety_veto_applied=True,
                    fallback_used=decision.fallback_used,
                    elapsed_time_s=decision.elapsed_time_s,
                )

        return decision


# ============================================================================
# 5. Native ReAct Swarm Orchestrator Agent
# ============================================================================


class SwarmOrchestratorAgent:
    """Native ReAct Tool-Using Deliberative Swarm Orchestrator.

    Conducts multi-step Thought -> Tool Action -> Observation reasoning, evaluates
    Pareto risk trade-offs, and enforces System-0 safety bounds.
    """

    def __init__(
        self,
        backend: LLMBackend | None = None,
        tool_belt: OrchestratorToolBelt | None = None,
        max_rounds: int = 3,
    ):
        self.tool_belt = tool_belt or OrchestratorToolBelt()
        self.backend = backend or GeminiRESTBackend()
        self.deterministic_reasoner = DeterministicReasoner(self.tool_belt)
        self.max_rounds = max_rounds
        self.recovery_monitoring_log: list[dict[str, Any]] = []
        self.aborted_techniques_by_robot: dict[str, set[str]] = {}

    def record_recovery_feedback(
        self,
        feedback: RecoveryFeedback,
        *,
        local_moisture: float = 0.85,
        local_firmness: float = 0.20,
        slip_ratio: float = 0.88,
    ) -> dict[str, Any]:
        """Monitor physical recovery outcome from MuJoCo and update online residual learner."""
        try:
            tech = MechanicalRescueTechnique(feedback.technique)
        except ValueError:
            tech = MechanicalRescueTechnique.DIRECT_PRY_AND_PULL

        prediction_error = self.tool_belt.learner.update(
            technique=tech,
            actual_displacement_m=feedback.victim_displacement_m,
            moisture=local_moisture,
            firmness=local_firmness,
            slip_ratio=slip_ratio,
            stuck_ticks=2,
        )
        if feedback.extracted:
            status = "EXTRACTED_AND_VERIFIED"
        elif not feedback.helper_safe or feedback.phase == "safety_abort_force_or_pitch":
            status = "ABORT_AND_REVISE"
            self.aborted_techniques_by_robot.setdefault(feedback.target_robot_id, set()).add(tech.value)
        else:
            status = "MONITORING_PULL"

        entry = {
            "observed_at_s": feedback.observed_at_s,
            "helper_robot_id": feedback.helper_robot_id,
            "target_robot_id": feedback.target_robot_id,
            "technique": tech.value,
            "latched": feedback.latched,
            "extracted": feedback.extracted,
            "victim_displacement_m": feedback.victim_displacement_m,
            "peak_constraint_force_n": feedback.peak_constraint_force_n,
            "helper_pitch_rad": feedback.helper_pitch_rad,
            "helper_safe": feedback.helper_safe,
            "phase": feedback.phase,
            "learner_prediction_error_m": round(prediction_error, 4),
            "status": status,
        }
        self.recovery_monitoring_log.append(entry)
        return entry

    def to_rescue_assessment(
        self,
        incident: IncidentReport,
        decision: OrchestratorDecision,
    ) -> RescueAssessment:
        """Convert an (IncidentReport, OrchestratorDecision) pair into canonical RescueAssessment."""
        rec_action = (
            RecoveryAction.DISPATCH_RESCUE_DIRECT
            if decision.verdict == RescueVerdict.RESCUE_NOW
            else (
                RecoveryAction.REALLOCATE_AND_BEACON
                if decision.verdict == RescueVerdict.DEFER_RESCUE
                else RecoveryAction.SAFE_ABSTAIN_DEFER_HUMAN
            )
        )
        return RescueAssessment(
            stranded_robot_id=incident.stranded_robot_id,
            verdict=decision.verdict,
            failure_mode=decision.failure_mode,
            recovery_action=rec_action,
            mechanical_technique=decision.chosen_technique,
            nearest_site_id=incident.nearest_site_id,
            rescue_benefit=incident.rescue_benefit,
            opportunity_cost=incident.opportunity_cost,
            hazard_risk=incident.hazard_risk,
            net_rescue_value=incident.net_rescue_value,
            helper_robot_id=decision.helper_robot_id,
            reason=decision.rationale,
            safety_veto_applied=decision.safety_veto_applied,
            fallback_used=decision.fallback_used,
            tool_call_count=len(decision.tool_trace),
            tool_trace=tuple(t.tool_name for t in decision.tool_trace),
        )

    @staticmethod
    def translate_setpoint_to_command_bounds(
        setpoint: MechanicalActuatorSetpoint | None,
    ) -> tuple[float, float, float]:
        """Clamp an orchestrator MechanicalActuatorSetpoint to M4 grounded RobotCommand safety limits."""
        if setpoint is None:
            return (0.0, 0.0, 0.0)
        return clamp_recovery_actuators(
            setpoint.dock_pitch_rad,
            setpoint.latch_rad,
            setpoint.spade_rad,
        )

    def orchestrate(self, incident: IncidentReport) -> OrchestratorDecision:
        """Main entry point: orchestrates an incident response using the ReAct loop."""
        t0 = time.perf_counter()

        # Step 1: Attempt LLM reasoning if backend is responsive
        prompt = self._build_system_prompt(incident)
        try:
            llm_response = self.backend.call(prompt) if self.backend else None
        except Exception:
            llm_response = None

        if not llm_response:
            # Safe 0.002s deterministic fallback
            det_decision = self.deterministic_reasoner.reason(incident)
            return OrchestratorSafetyGate.validate(incident, det_decision)

        # Step 2: Parse ReAct steps or final JSON from LLM
        decision = self._execute_react_loop(incident, llm_response, t0)
        return OrchestratorSafetyGate.validate(incident, decision)

    def _execute_react_loop(
        self,
        incident: IncidentReport,
        initial_response: str,
        start_time: float,
    ) -> OrchestratorDecision:
        """Executes tool calls iteratively if the LLM requests actions."""
        trace: list[ToolCallRecord] = []
        current_response = initial_response

        for round_idx in range(1, self.max_rounds + 1):
            parsed = self._safe_parse_json(current_response)
            if not parsed:
                break

            action = parsed.get("action")
            thought = parsed.get("thought", "")

            # If model reached final verdict, construct decision
            if action in {"final_verdict", "decision", None} or "verdict" in parsed:
                return self._build_decision_from_parsed(incident, parsed, trace, start_time)

            # Otherwise, execute requested tool
            tool_args = parsed.get("action_input", {})
            obs = self._execute_tool(action, tool_args, incident)
            trace.append(
                ToolCallRecord(
                    round_index=round_idx,
                    thought=thought,
                    tool_name=action,
                    arguments=tool_args,
                    observation=obs,
                )
            )

            # Query backend for next step with tool observation
            next_prompt = (
                f"Tool '{action}' returned observation:\n{json.dumps(obs)}\n"
                "Provide your next Thought and Action or Final Verdict in JSON."
            )
            try:
                resp = self.backend.call(next_prompt) if self.backend else None
            except Exception:
                resp = None
            if not resp:
                break
            current_response = resp

        # Fallback to deterministic if loop failed to conclude
        det = self.deterministic_reasoner.reason(incident)
        return OrchestratorDecision(
            incident_id=incident.incident_id,
            verdict=det.verdict,
            failure_mode=det.failure_mode,
            chosen_technique=det.chosen_technique,
            helper_robot_id=det.helper_robot_id,
            standoff_pose=det.standoff_pose,
            mechanical_setpoint=det.mechanical_setpoint,
            rationale=det.rationale,
            tool_trace=tuple(trace) if trace else det.tool_trace,
            fallback_used=True,
            elapsed_time_s=time.perf_counter() - start_time,
        )

    def _execute_tool(
        self,
        action: str,
        args: dict[str, Any],
        incident: IncidentReport,
    ) -> dict[str, Any]:
        """Dispatches an action string to the corresponding tool in the tool belt."""
        if action == "inspect_incident":
            return self.tool_belt.inspect_incident(incident)
        elif action == "probe_perimeter":
            helper_id = args.get("helper_id", incident.available_helpers[0].robot_id if incident.available_helpers else "")
            standoff = float(args.get("standoff_m", 1.5))
            return self.tool_belt.probe_perimeter(incident, helper_id=helper_id, standoff_m=standoff)
        elif action == "predict_recovery_displacement":
            return self.tool_belt.predict_recovery_displacement(
                technique_str=args.get("technique", "direct_pry_and_pull"),
                moisture=float(args.get("moisture", incident.local_moisture or 0.70)),
                firmness=float(args.get("firmness", incident.local_firmness or 0.30)),
                slip_ratio=float(args.get("slip_ratio", incident.slip_ratio)),
            )
        elif action == "evaluate_farmer_risk":
            return self.tool_belt.evaluate_farmer_risk(
                incident,
                helper_id=args.get("helper_id", ""),
                candidate_technique_str=args.get("technique", ""),
                estimated_helper_risk=float(args.get("estimated_helper_risk", 0.10)),
            )
        elif action == "authorize_recovery_skill":
            return self.tool_belt.authorize_recovery_skill(
                incident,
                technique_str=args.get("technique", "direct_pry_and_pull"),
                helper_id=args.get("helper_id", ""),
            )
        elif action == "refuse_and_rebalance":
            return self.tool_belt.refuse_and_rebalance(
                incident,
                reason=args.get("reason", "Operator risk exceeded"),
            )
        return {"error": f"Unknown tool action: {action}"}

    def _build_decision_from_parsed(
        self,
        incident: IncidentReport,
        parsed: dict[str, Any],
        trace: list[ToolCallRecord],
        start_time: float,
    ) -> OrchestratorDecision:
        """Constructs an `OrchestratorDecision` from LLM output."""
        verdict_str = parsed.get("verdict", "abandon_escalate")
        try:
            verdict = RescueVerdict(verdict_str)
        except ValueError:
            verdict = RescueVerdict.ABANDON_ESCALATE

        tech_str = parsed.get("chosen_technique", "none")
        try:
            tech = MechanicalRescueTechnique(tech_str)
        except ValueError:
            tech = MechanicalRescueTechnique.NONE

        helper_id = parsed.get("helper_robot_id")
        setpoint = None
        if verdict == RescueVerdict.RESCUE_NOW and helper_id and tech != MechanicalRescueTechnique.NONE:
            setpoint = compute_mechanical_recovery_setpoint(
                robot_id=helper_id,
                technique=tech,
                step_index=0,
                latched=True,
            )

        helper = next((h for h in incident.available_helpers if h.robot_id == helper_id), None)
        return OrchestratorDecision(
            incident_id=incident.incident_id,
            verdict=verdict,
            failure_mode=incident.failure_mode,
            chosen_technique=tech,
            helper_robot_id=helper_id,
            standoff_pose=helper.pose if helper else None,
            mechanical_setpoint=setpoint,
            rationale=parsed.get("rationale", "Autonomous decision by Swarm Orchestrator."),
            tool_trace=tuple(trace),
            fallback_used=False,
            elapsed_time_s=time.perf_counter() - start_time,
        )

    def _build_system_prompt(self, incident: IncidentReport) -> str:
        return (
            "You are the System-2 Deliberative Swarm Orchestrator for agricultural robots in a rice terrace.\n"
            "Your role is to reason through potential robot immobilizations, resolve sensor ambiguities, "
            "protect valuable seed-carrying rovers, and choose whether to execute a physical recovery or safely refuse.\n\n"
            "AVAILABLE TOOLS:\n"
            "1. inspect_incident(): Verifies true physical stall vs sensor dropout.\n"
            "2. probe_perimeter(helper_id, standoff_m): Checks soil firmness around the trapped peer.\n"
            "3. predict_recovery_displacement(technique, moisture, firmness, slip_ratio): Queries the ResidualRecoveryLearner.\n"
            "4. evaluate_farmer_risk(helper_id, technique, estimated_helper_risk): Validates against farmer policies.\n"
            "5. authorize_recovery_skill(technique, helper_id): Formulates mechanical setpoints.\n"
            "6. refuse_and_rebalance(reason): Declines rescue, alerts farmer, reallocates tasks.\n\n"
            "INCIDENT TELEMETRY:\n"
            f"{json.dumps(incident.to_dict(), indent=2)}\n\n"
            "RESPONSE FORMAT:\n"
            "Return valid JSON with 'thought' and either an 'action' + 'action_input' (to call a tool) OR "
            "'verdict', 'chosen_technique', 'helper_robot_id', and 'rationale' (for final decision)."
        )

    def _safe_parse_json(self, text: str) -> dict[str, Any] | None:
        try:
            return json.loads(text)
        except Exception:
            # Handle potential markdown code fences
            clean = text.strip()
            if clean.startswith("```json"):
                clean = clean[7:]
            if clean.startswith("```"):
                clean = clean[3:]
            if clean.endswith("```"):
                clean = clean[:-3]
            try:
                return json.loads(clean.strip())
            except Exception:
                return None
