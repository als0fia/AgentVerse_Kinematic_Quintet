"""Dedicated Agentic Robot Saving, Sensor Failure Detection, Gossip Distress & Mechanical Recovery System (`src/ricebot/agent/rescue.py`).

Implements:
1. Sensor-Driven Failure & Tip-Over Detection (`detect_robot_failure_from_sensors`):
   - Uses IMU attitude (`pitch_rad`, `roll_rad`), Doppler ground speed (`true_speed_m_s`),
     wheel odometry displacement, rangefinder (`front_range_m`), and bogged state (`is_bogged`)
     with negative controls (ignoring intentional `TaskMode.HOLD` or seed release).
   - Distinguishes S4 Rollover vs Pitch-Tipped Ditch Stall, S1 Deep Mud Belly-Out (via
     Bekker-Wong `evaluate_bekker_terramechanics`), S2 Asymmetric Yaw Skew, S3/S8 Surface
     Obstruction, S6 Post-Probe Suction, and S7 Cascading Rescuer Slip.
2. Wireless Distress Packet Payload (`DistressSignal`):
   - Compact 36-byte telemetry struct embedded inside `MycelialGossipPacket` when a robot
     detects a failure, broadcasting pose, attitude, slip ratio, Bekker required pull force (N),
     and recommended mechanical technique to reachable peers.
3. Mechanical Recovery Controller (`MechanicalRescueTechnique` & `MechanicalActuatorSetpoint`):
   - Drives the homogeneous Paddy Ant's exact MuJoCo actuators (`src/ricebot/robot/rovers.py`):
     * 4-wheel grouser velocity servos (`_act_fl..rr`, `[-14, +14] rad/s`)
     * Front active pitch-lifting docking boom (`_dock_pitch_act`, `[-0.20, +0.45] rad`)
     * Rotary cam-locking jaw (`_latch_act`, `[0.0, 0.65] rad`)
     * Rear deployable hardpan anchor spades (`_spade_act`, `[0.0, 0.85] rad`)
     * Vertical plunging seed-dibber/soil-probe arm (`_dibber_act`, `[-0.052, +0.010] m`)
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import atan2, degrees, hypot, pi

from ..contracts import (
    ObservationFrame,
    Pose2D,
    RobotObservation,
    Site,
    TaskMode,
)
from ..mission import SiteEstimate
from ..robot.rovers import (
    PLANTER_SPEC,
    RoverChassisSpec,
    compute_4wheel_skid_steer,
    evaluate_bekker_terramechanics,
)


def _wrap_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


class FailureMode(str, Enum):
    """Diagnosed root cause of why a robot is immobilized or faulted (mapped to S1-S8)."""

    NONE = "none"
    SENSOR_DROPOUT = "sensor_dropout"  # front_range_m is None (sensor fault, NOT mud immobilization)
    SURFACE_OBSTRUCTION = "surface_obstruction"  # S3/S8: blocked or high-centered on firm soil
    ASYMMETRIC_SLIP_OR_SUCTION = "asymmetric_slip_or_suction"  # S2/S6: yaw-skewed rut or clay stiction
    CONTAGIOUS_MUD_TRAP = "contagious_mud_trap"  # S1: plow-pan breach / deep mud sinkage
    PITCH_NOSE_DOWN_STALL = "pitch_nose_down_stall"  # S3/S4 moderate: nose pitched into bund/lip, rear dock reachable
    TIP_OVER_OR_TRENCH = "tip_over_or_trench"  # S4 severe: lateral rollover or deep ditch drop-in
    CASCADING_TOW_OVERLOAD = "cascading_tow_overload"  # S7: rescuer experiencing secondary wheel slip
    UNKNOWN_TERRAIN_TRAP = "unknown_terrain_trap"  # immobilized in unmeasured soil (probe perimeter first!)
    NO_GO_HAZARD = "no_go_hazard"  # inside or bordering a permitted=False hazard zone


class RecoveryAction(str, Enum):
    """High-level algorithmic recovery action selected by the swarm coordinator."""

    NONE = "none"
    SELF_RECOVER_BOUNDED = "self_recover_bounded"  # Bounded self-extraction using dibber + grouser pulse
    INVESTIGATE_PERIMETER = "investigate_perimeter"  # Dispatch peer to SAMPLE perimeter before committing to tow
    DISPATCH_RESCUE_DIRECT = "dispatch_rescue_direct"  # Dispatch peer to rigid-dock and extract from firm lane
    REALLOCATE_AND_BEACON = "reallocate_and_beacon"  # Convert stuck unit to UWB/ESP-NOW beacon; reallocate tasks
    SAFE_ABSTAIN_DEFER_HUMAN = "safe_abstain_defer_human"  # Freeze actuators, deploy anchor spades & escalate to human


class MechanicalRescueTechnique(str, Enum):
    """Concrete physical rescue technique using the homogeneous Paddy Ant's mechanical actuators."""

    NONE = "none"
    SELF_OSCILLATE_DIBBER = "self_oscillate_dibber"  # Plunge vertical dibber (-50mm) + 15Hz reverse grouser pulse
    BUMPER_REALIGN_THEN_PULL = "bumper_realign_then_pull"  # SE(2) tapered cone nudge to straighten yaw skew -> latch -> pull
    PITCH_MATCH_NOSE_DOWN_RECOVERY = "pitch_match_nose_down_recovery"  # Elevate boom (+0.28 rad) to catch pitched rear bar -> level -> pull
    DIRECT_PRY_AND_PULL = "direct_pry_and_pull"  # Lock cam-latch (+0.58 rad) + lift boom (+0.36 rad) + 8-wheel reverse
    DYNAMIC_YAW_SHEAR_AND_PITCH_PUMP = "dynamic_yaw_shear_and_pitch_pump"  # Deploy spades (+0.75 rad) + 8Hz yaw wiggle + cyclic pitch pump
    SPADE_ANCHOR_ROLLOVER_HOLD = "spade_anchor_rollover_hold"  # Cut wheels (0 rad/s) + deploy rear spades (+0.85 rad) to prevent ditch slide


class RescueVerdict(str, Enum):
    """Agentic triage decision for whether a stranded/faulted robot is worth saving."""

    HEALTHY = "healthy"
    SELF_RECOVER = "self_recover"  # Bounded local self-recovery before requesting peer help
    INVESTIGATE_PERIMETER = "investigate_perimeter"  # Probe safe adjacent site first when trap odds unknown
    RESCUE_NOW = "rescue_now"  # Peer diverts immediately to save stranded robot
    DEFER_RESCUE = "defer_rescue"  # Peer finishes high-value planting first, then rescues (beacon mode)
    ABANDON_ESCALATE = "abandon_escalate"  # Not worth risking/diverting peer; safe HOLD for human


@dataclass(frozen=True)
class MechanicalActuatorSetpoint:
    """Low-level actuator commands for `src/ricebot/robot/rovers.py` (ctrl indices 0..19)."""

    robot_id: str
    technique: MechanicalRescueTechnique
    wheel_vel_rad_s: tuple[float, float, float, float]  # (w_fl, w_fr, w_rl, w_rr) in [-14.0, +14.0]
    dock_pitch_rad: float  # [-0.20, +0.45] rad
    latch_rad: float  # [0.0, 0.65] rad (>0.30 locks rigid equality constraint)
    spade_rad: float  # [0.0, 0.85] rad (rear hardpan anchor spades)
    dibber_m: float  # [-0.052, +0.010] m (vertical seed-dibber plunger)
    description: str


@dataclass(frozen=True)
class DistressSignal:
    """Compact wireless distress telemetry embedded in `MycelialGossipPacket` when a robot fails."""

    robot_id: str
    time_s: float
    pose: Pose2D
    failure_mode: FailureMode
    recommended_technique: MechanicalRescueTechnique
    pitch_deg: float
    roll_deg: float
    slip_ratio: float
    seeds_remaining: int
    required_pull_n: float
    self_recovery_ticks: int
    needs_peer_assistance: bool
    summary_text: str
    residual_weights: tuple[float, ...] = ()


@dataclass(frozen=True)
class RescueAssessment:
    """Complete cost-benefit, failure diagnosis, trap-odds, and mechanical recovery assessment."""

    stranded_robot_id: str
    verdict: RescueVerdict
    failure_mode: FailureMode = FailureMode.UNKNOWN_TERRAIN_TRAP
    recovery_action: RecoveryAction = RecoveryAction.INVESTIGATE_PERIMETER
    mechanical_technique: MechanicalRescueTechnique = MechanicalRescueTechnique.NONE
    nearest_site_id: str | None = None
    perimeter_probe_site_id: str | None = None
    rescue_benefit: float = 0.0
    opportunity_cost: float = 0.0
    hazard_risk: float = 0.0
    trap_probability: float = 0.0
    expected_trap_loss: float = 0.0
    net_rescue_value: float = 0.0
    required_pull_n: float = 0.0
    trap_odds_uncertain: bool = False
    active_compute_nodes: int = 1
    helper_robot_id: str | None = None
    reason: str = ""
    rescuer_robot_id: str | None = None
    expected_saved_benefit: float = 0.0
    rescue_opportunity_cost: float = 0.0
    net_rescue_utility: float = 0.0
    rationale: str = ""

    def to_dict(self) -> dict[str, object]:
        return {
            "stranded_robot_id": self.stranded_robot_id,
            "verdict": self.verdict.value,
            "failure_mode": self.failure_mode.value,
            "recovery_action": self.recovery_action.value,
            "mechanical_technique": self.mechanical_technique.value,
            "nearest_site_id": self.nearest_site_id,
            "perimeter_probe_site_id": self.perimeter_probe_site_id,
            "rescue_benefit": self.rescue_benefit,
            "opportunity_cost": self.opportunity_cost,
            "hazard_risk": self.hazard_risk,
            "trap_probability": self.trap_probability,
            "expected_trap_loss": self.expected_trap_loss,
            "net_rescue_value": self.net_rescue_value,
            "required_pull_n": self.required_pull_n,
            "trap_odds_uncertain": self.trap_odds_uncertain,
            "active_compute_nodes": self.active_compute_nodes,
            "helper_robot_id": self.helper_robot_id,
            "reason": self.reason,
        }


def compute_mechanical_recovery_setpoint(
    robot_id: str,
    technique: MechanicalRescueTechnique,
    *,
    step_index: int = 0,
    rescuer_pose: Pose2D | None = None,
    target_pose: Pose2D | None = None,
    latched: bool = False,
    spec: RoverChassisSpec = PLANTER_SPEC,
) -> MechanicalActuatorSetpoint:
    """Compute exact MuJoCo actuator setpoints for the homogeneous Paddy Ant's mechanical rescue rig."""
    if technique == MechanicalRescueTechnique.SPADE_ANCHOR_ROLLOVER_HOLD:
        return MechanicalActuatorSetpoint(
            robot_id=robot_id,
            technique=technique,
            wheel_vel_rad_s=(0.0, 0.0, 0.0, 0.0),
            dock_pitch_rad=0.0,
            latch_rad=0.0,
            spade_rad=0.85,  # Full hardpan spade deployment to prevent sliding deeper into trench
            dibber_m=-0.050,  # Plunge dibber as additional ground pin
            description="S4 Rollover/Trench Hold: cut wheel drive and deploy rear spades (+0.85 rad) + dibber pin (-50 mm)",
        )

    if technique == MechanicalRescueTechnique.SELF_OSCILLATE_DIBBER:
        # 15 Hz reverse grouser pulse + vertical seed-dibber plunger push-off (-50 mm)
        pulse = -14.0 if (step_index % 6 < 4) else 6.0
        dibber_pos = -0.050 if (step_index % 6 < 3) else 0.005
        return MechanicalActuatorSetpoint(
            robot_id=robot_id,
            technique=technique,
            wheel_vel_rad_s=(pulse, pulse, pulse, pulse),
            dock_pitch_rad=0.0,
            latch_rad=0.0,
            spade_rad=0.0,
            dibber_m=dibber_pos,
            description="Technique 1 (Self-Oscillate Dibber): plunge vertical dibber (-50 mm) into hardpan + reverse grouser pulse",
        )

    # For peer-assisted techniques, if not yet latched and poses are provided, servo into rear V-funnel
    if not latched and rescuer_pose is not None and target_pose is not None:
        dx = target_pose.x_m - rescuer_pose.x_m
        dy = target_pose.y_m - rescuer_pose.y_m
        desired_yaw = atan2(dy, max(0.04, abs(dx)) * (1.0 if dx >= 0 else -1.0))
        yaw_err = _wrap_angle(desired_yaw - rescuer_pose.yaw_rad)
        lin = 0.28 if abs(yaw_err) < 0.35 else 0.10
        ang = max(-1.35, min(1.35, 3.2 * yaw_err))
        wheels = compute_4wheel_skid_steer(spec, lin, ang)
        approach_pitch = (
            0.26
            if technique == MechanicalRescueTechnique.PITCH_MATCH_NOSE_DOWN_RECOVERY
            else -0.05
        )
        return MechanicalActuatorSetpoint(
            robot_id=robot_id,
            technique=technique,
            wheel_vel_rad_s=wheels,
            dock_pitch_rad=approach_pitch,
            latch_rad=0.58,  # Pre-arm rotary cam hook to lock on contact (<= 65 mm)
            spade_rad=0.0,
            dibber_m=0.0,
            description=f"SE(2) Funnel Approach ({technique.value}): align front cone with rear strike bar",
        )

    if technique == MechanicalRescueTechnique.DYNAMIC_YAW_SHEAR_AND_PITCH_PUMP:
        # "Ant Wiggle": anchor rear spades (+0.75 rad), 8 Hz alternating yaw shear, cyclic pitch pumping
        rock = 10.0 if (step_index % 16 < 8) else -10.0
        pitch_pump = 0.42 if (step_index % 12 < 6) else 0.15
        return MechanicalActuatorSetpoint(
            robot_id=robot_id,
            technique=technique,
            wheel_vel_rad_s=(rock, -rock, rock, -rock),
            dock_pitch_rad=pitch_pump,
            latch_rad=0.58,
            spade_rad=0.75,
            dibber_m=0.0,
            description="Technique 4 (Ant Wiggle): anchor spades (+0.75 rad) + cyclic pitch pump (0.15..0.42 rad) + yaw shear",
        )

    if technique == MechanicalRescueTechnique.PITCH_MATCH_NOSE_DOWN_RECOVERY:
        return MechanicalActuatorSetpoint(
            robot_id=robot_id,
            technique=technique,
            wheel_vel_rad_s=(-9.0, -9.0, -9.0, -9.0),
            dock_pitch_rad=-0.10,  # Pull elevated rear strike bar down to level nose-down peer while reversing
            latch_rad=0.58,
            spade_rad=0.35,
            dibber_m=0.0,
            description="Nose-Down Pitch Recovery: cam-latched to elevated rear bar, leveling pitch (-0.10 rad) + 8-wheel reverse",
        )

    # Default latched extraction for DIRECT_PRY_AND_PULL and BUMPER_REALIGN_THEN_PULL
    return MechanicalActuatorSetpoint(
        robot_id=robot_id,
        technique=technique,
        wheel_vel_rad_s=(-9.5, -9.5, -9.5, -9.5),
        dock_pitch_rad=0.36,  # Pry upward (+0.36 rad) to lift peer's belly off mud/bund
        latch_rad=0.58,
        spade_rad=0.0,
        dibber_m=0.0,
        description=f"Tandem Extraction ({technique.value}): cam-latch locked + boom pry-lift (+0.36 rad) + 8-wheel reverse",
    )



class ResidualRecoveryLearner:
    """Online neural residual learning loop for terramechanics recovery action selection.

    Maintains online weight vectors w[technique] in R^4 mapping terrain & entrapment state
    phi(s) = [1.0, moisture, softness, slip_or_stall] to residual physical displacement (metres).

    Total prediction:
        Delta_p_hat(s, a) = Delta_p_bekker(s, a) + w[a]^T phi(s)

    Online adaptation:
        e = Delta_p_actual - Delta_p_hat(s, a)
        w[a] <- w[a] + eta * e * phi(s) - lambda * w[a]

    Zero GPU requirement: computes lightweight dot products in pure Python/NumPy (< 10 microseconds)
    and serializes directly into wireless gossip distress beacons (24 floats = 96 bytes).
    """

    def __init__(
        self,
        learning_rate: float = 0.20,
        l2_decay: float = 0.01,
        initial_weights: dict[MechanicalRescueTechnique, tuple[float, float, float, float]] | None = None,
    ):
        self.learning_rate = learning_rate
        self.l2_decay = l2_decay
        # Feature order: [bias, moisture_penalty, softness_penalty, slip_stall_penalty]
        # In baseline mud:
        # - DIRECT_PRY_AND_PULL works well on firm mud, but suffers heavy suction penalty in high moisture/softness
        # - DYNAMIC_YAW_SHEAR_AND_PITCH_PUMP ("Ant Wiggle") has lower raw pull speed but actively liquefies/breaks suction
        self.weights: dict[MechanicalRescueTechnique, list[float]] = {}
        defaults = {
            MechanicalRescueTechnique.SELF_OSCILLATE_DIBBER: [0.020, -0.005, -0.005, -0.015],
            MechanicalRescueTechnique.BUMPER_REALIGN_THEN_PULL: [0.025, -0.008, -0.008, -0.010],
            MechanicalRescueTechnique.PITCH_MATCH_NOSE_DOWN_RECOVERY: [0.030, -0.005, -0.005, -0.005],
            MechanicalRescueTechnique.DIRECT_PRY_AND_PULL: [0.035, -0.010, -0.010, -0.012],
            MechanicalRescueTechnique.DYNAMIC_YAW_SHEAR_AND_PITCH_PUMP: [0.015, 0.008, 0.008, 0.005],
            MechanicalRescueTechnique.SPADE_ANCHOR_ROLLOVER_HOLD: [0.000, 0.000, 0.000, 0.000],
        }
        for tech in MechanicalRescueTechnique:
            if tech == MechanicalRescueTechnique.NONE:
                continue
            if initial_weights and tech in initial_weights:
                self.weights[tech] = list(initial_weights[tech])
            else:
                self.weights[tech] = list(defaults.get(tech, [0.0, 0.0, 0.0, 0.0]))
        self.history: list[dict[str, object]] = []

    def extract_features(
        self,
        moisture: float,
        firmness: float,
        slip_ratio: float,
        stuck_ticks: int,
    ) -> tuple[float, float, float, float]:
        """Compute state feature vector phi(s) = [bias, moisture, softness, slip_stall]."""
        m = max(0.0, min(1.0, moisture))
        softness = max(0.0, min(1.0, 1.0 - firmness))
        stall_factor = max(0.0, min(1.0, slip_ratio * 0.5 + min(stuck_ticks, 5) * 0.10))
        return (1.0, m, softness, stall_factor)

    def predict_displacement(
        self,
        technique: MechanicalRescueTechnique,
        bekker_pull_margin_n: float,
        phi: tuple[float, float, float, float],
    ) -> float:
        """Predict expected forward displacement (metres per recovery burst) combining physics + residual."""
        base_phys = max(0.0, min(0.05, bekker_pull_margin_n * 0.0002))
        w = self.weights.get(technique, [0.0, 0.0, 0.0, 0.0])
        residual = sum(w_i * f_i for w_i, f_i in zip(w, phi))
        return base_phys + residual

    def update(
        self,
        technique: MechanicalRescueTechnique,
        actual_displacement_m: float,
        moisture: float,
        firmness: float,
        slip_ratio: float,
        stuck_ticks: int,
        bekker_pull_margin_n: float = 0.0,
    ) -> float:
        """Single-step online gradient descent adaptation. Returns prediction error e."""
        if technique not in self.weights:
            return 0.0
        phi = self.extract_features(moisture, firmness, slip_ratio, stuck_ticks)
        pred = self.predict_displacement(technique, bekker_pull_margin_n, phi)
        err = actual_displacement_m - pred
        w = self.weights[technique]
        for i in range(len(w)):
            grad = err * phi[i]
            w[i] = w[i] + self.learning_rate * grad - self.l2_decay * w[i]
            w[i] = max(-0.50, min(0.50, w[i]))
        self.history.append({
            "technique": technique.value,
            "pred": round(pred, 4),
            "actual": round(actual_displacement_m, 4),
            "error": round(err, 4),
            "weights": [round(x, 4) for x in w],
        })
        return err

    def select_technique(
        self,
        failure_mode: FailureMode,
        moisture: float,
        firmness: float,
        slip_ratio: float,
        stuck_ticks: int,
        bekker_pull_margin_n: float,
        yaw_rad: float = 0.0,
        cross_track_m: float = 0.0,
    ) -> tuple[MechanicalRescueTechnique, dict[MechanicalRescueTechnique, float]]:
        """Counterfactually score all techniques using the residual model and pick the best safe action."""
        # 1. Hard safety overrides: S4 Tip-Over/Trench MUST deploy spades and hold
        if failure_mode in (FailureMode.TIP_OVER_OR_TRENCH, FailureMode.NO_GO_HAZARD):
            return MechanicalRescueTechnique.SPADE_ANCHOR_ROLLOVER_HOLD, {
                MechanicalRescueTechnique.SPADE_ANCHOR_ROLLOVER_HOLD: 0.0
            }
        if failure_mode == FailureMode.PITCH_NOSE_DOWN_STALL:
            return MechanicalRescueTechnique.PITCH_MATCH_NOSE_DOWN_RECOVERY, {
                MechanicalRescueTechnique.PITCH_MATCH_NOSE_DOWN_RECOVERY: 0.05
            }

        phi = self.extract_features(moisture, firmness, slip_ratio, stuck_ticks)
        scores: dict[MechanicalRescueTechnique, float] = {}

        candidates = [
            MechanicalRescueTechnique.SELF_OSCILLATE_DIBBER,
            MechanicalRescueTechnique.BUMPER_REALIGN_THEN_PULL,
            MechanicalRescueTechnique.DIRECT_PRY_AND_PULL,
            MechanicalRescueTechnique.DYNAMIC_YAW_SHEAR_AND_PITCH_PUMP,
        ]

        is_skewed = abs(yaw_rad) > 0.148 or abs(cross_track_m) > 0.015

        for cand in candidates:
            score = self.predict_displacement(cand, bekker_pull_margin_n, phi)
            # Dibber self-oscillation is viable on initial shallow slip before deep bellied-out stall
            if cand == MechanicalRescueTechnique.SELF_OSCILLATE_DIBBER and stuck_ticks > 1:
                score -= 0.10
            # Skewed candidate bump
            if cand == MechanicalRescueTechnique.BUMPER_REALIGN_THEN_PULL and is_skewed:
                score += 0.03
            scores[cand] = round(score, 4)

        best_tech = max(scores.keys(), key=lambda k: scores[k])
        return best_tech, scores

    def export_weights_flat(self) -> tuple[float, ...]:
        """Serialize all weights into a compact flat tuple suitable for gossip packet transmission."""
        flat: list[float] = []
        for tech in sorted(self.weights.keys(), key=lambda t: t.value):
            flat.extend(float(w) for w in self.weights[tech])
        return tuple(flat)

    def import_weights_flat(self, flat: tuple[float, ...]) -> None:
        """Incorporate empirical weights received over gossip from a stuck peer."""
        if not flat:
            return
        keys = sorted(self.weights.keys(), key=lambda t: t.value)
        idx = 0
        for tech in keys:
            if idx + 4 <= len(flat):
                self.weights[tech] = list(flat[idx : idx + 4])
                idx += 4


class RescueEngine:
    """Stateful multi-tick fault detector, S1-S8 failure classifier, and algorithmic recovery engine."""

    def __init__(
        self,
        obstacle_trigger_range_m: float = 0.35,
        stuck_ticks_threshold: int = 2,
        travel_cost_weight: float = 0.08,
        mud_moisture_threshold: float = 0.75,
        mud_firmness_threshold: float = 0.25,
        max_pitch_roll_rad: float = 0.35,
        rollover_abort_rad: float = 0.55,
        learner: ResidualRecoveryLearner | None = None,
    ):
        if (
            obstacle_trigger_range_m <= 0
            or stuck_ticks_threshold < 1
            or travel_cost_weight < 0
            or max_pitch_roll_rad <= 0
        ):
            raise ValueError("invalid rescue engine parameters")
        self.obstacle_trigger_range_m = obstacle_trigger_range_m
        self.stuck_ticks_threshold = stuck_ticks_threshold
        self.travel_cost_weight = travel_cost_weight
        self.mud_moisture_threshold = mud_moisture_threshold
        self.mud_firmness_threshold = mud_firmness_threshold
        self.max_pitch_roll_rad = max_pitch_roll_rad
        self.rollover_abort_rad = rollover_abort_rad
        self.learner = learner if learner is not None else ResidualRecoveryLearner()
        self._last_pose: dict[str, Pose2D] = {}
        self._stuck_counts: dict[str, int] = {}

    def detect_stranded_robots(
        self,
        frame: ObservationFrame,
        previous_targets: dict[str, tuple[TaskMode, str | None]],
    ) -> set[str]:
        """Track robot motion and onboard reflex sensors across ticks to identify immobilized/faulted robots."""
        stranded: set[str] = set()
        for robot in frame.robots:
            prev_pose = self._last_pose.get(robot.robot_id)
            prev_mode, _ = previous_targets.get(robot.robot_id, (TaskMode.HOLD, None))
            moved = (
                hypot(robot.pose.x_m - prev_pose.x_m, robot.pose.y_m - prev_pose.y_m)
                if prev_pose is not None
                else 0.0
            )
            sensor_fault = robot.front_range_m is None
            tip_over_fault = (
                abs(robot.pitch_rad) > self.max_pitch_roll_rad
                or abs(robot.roll_rad) > self.max_pitch_roll_rad
            )
            bogged_fault = robot.is_bogged
            # Negative control: only count blocked_immobile if the robot was not intentionally holding with clear range
            blocked_immobile = (
                (prev_pose is None or (prev_mode != TaskMode.HOLD and moved < 0.02))
                and robot.front_range_m is not None
                and robot.front_range_m <= self.obstacle_trigger_range_m
            )
            if sensor_fault or tip_over_fault or bogged_fault or blocked_immobile:
                self._stuck_counts[robot.robot_id] = (
                    self._stuck_counts.get(robot.robot_id, 0) + 1
                )
            else:
                self._stuck_counts[robot.robot_id] = 0
            self._last_pose[robot.robot_id] = robot.pose

            if (
                sensor_fault
                or tip_over_fault
                or bogged_fault
                or self._stuck_counts[robot.robot_id] >= self.stuck_ticks_threshold
            ):
                stranded.add(robot.robot_id)
        return stranded

    def update_recovery_learning(
        self,
        technique: MechanicalRescueTechnique,
        actual_displacement_m: float,
        moisture: float | None = None,
        firmness: float | None = None,
        slip_ratio: float = 0.85,
        stuck_ticks: int = 1,
    ) -> float:
        """Feed observed recovery attempt outcome back into the online residual learning loop."""
        return self.learner.update(
            technique=technique,
            actual_displacement_m=actual_displacement_m,
            moisture=moisture if moisture is not None else 0.65,
            firmness=firmness if firmness is not None else 0.35,
            slip_ratio=slip_ratio,
            stuck_ticks=stuck_ticks,
        )

    def _select_mechanical_technique(
        self,
        stranded_robot: RobotObservation,
        failure_mode: FailureMode,
        moisture: float | None,
        firmness: float | None,
        stuck_ticks: int,
    ) -> tuple[MechanicalRescueTechnique, float]:
        """Select the best physical actuator technique using neural residual counterfactual evaluation."""
        m_val = moisture if moisture is not None else 0.65
        f_val = firmness if firmness is not None else 0.35
        slip_est = 0.90 if stranded_robot.is_bogged else 0.35
        bekker = evaluate_bekker_terramechanics(
            PLANTER_SPEC,
            moisture=m_val,
            firmness=f_val,
            slip_ratio=slip_est,
        )
        req_pull_n = bekker.required_rescue_pull_n

        tech, _ = self.learner.select_technique(
            failure_mode=failure_mode,
            moisture=m_val,
            firmness=f_val,
            slip_ratio=slip_est,
            stuck_ticks=stuck_ticks,
            bekker_pull_margin_n=bekker.drawbar_pull_margin_n,
            yaw_rad=stranded_robot.pose.yaw_rad,
            cross_track_m=stranded_robot.row_cross_track_m,
        )
        return tech, req_pull_n

    def build_distress_signal(
        self,
        obs: RobotObservation,
        slip_ratio: float,
        local_moisture: float | None = None,
        local_firmness: float | None = None,
    ) -> DistressSignal | None:
        """Inspect onboard sensor readings for tip-over, bogging, or range dropout and build a wireless `DistressSignal`."""
        pitch_abs = abs(obs.pitch_rad)
        roll_abs = abs(obs.roll_rad)
        stuck_ticks = self._stuck_counts.get(obs.robot_id, 1)

        # 1. Check IMU Tip-Over / Pitch Stall (S4 / S3)
        if roll_abs > self.max_pitch_roll_rad or pitch_abs > self.rollover_abort_rad:
            tech, pull_n = self._select_mechanical_technique(
                obs, FailureMode.TIP_OVER_OR_TRENCH, local_moisture, local_firmness, stuck_ticks
            )
            return DistressSignal(
                robot_id=obs.robot_id,
                time_s=obs.time_s,
                pose=obs.pose,
                failure_mode=FailureMode.TIP_OVER_OR_TRENCH,
                recommended_technique=tech,
                pitch_deg=round(degrees(obs.pitch_rad), 2),
                roll_deg=round(degrees(obs.roll_rad), 2),
                slip_ratio=round(slip_ratio, 3),
                seeds_remaining=obs.seeds_remaining,
                required_pull_n=pull_n,
                self_recovery_ticks=stuck_ticks,
                needs_peer_assistance=False,  # Unsafe to dock with rolled-over unit in trench
                residual_weights=self.learner.export_weights_flat(),
                summary_text=(
                    f"DISTRESS[{obs.robot_id}]: S4 rollover/trench tilt "
                    f"(pitch={degrees(obs.pitch_rad):.1f}deg, roll={degrees(obs.roll_rad):.1f}deg) -> "
                    f"deploying spades ({tech.value}); DO NOT DOCK"
                ),
            )

        if pitch_abs > self.max_pitch_roll_rad and roll_abs <= 0.25:
            tech, pull_n = self._select_mechanical_technique(
                obs, FailureMode.PITCH_NOSE_DOWN_STALL, local_moisture, local_firmness, stuck_ticks
            )
            return DistressSignal(
                robot_id=obs.robot_id,
                time_s=obs.time_s,
                pose=obs.pose,
                failure_mode=FailureMode.PITCH_NOSE_DOWN_STALL,
                recommended_technique=tech,
                pitch_deg=round(degrees(obs.pitch_rad), 2),
                roll_deg=round(degrees(obs.roll_rad), 2),
                slip_ratio=round(slip_ratio, 3),
                seeds_remaining=obs.seeds_remaining,
                required_pull_n=pull_n,
                self_recovery_ticks=stuck_ticks,
                needs_peer_assistance=True,
                residual_weights=self.learner.export_weights_flat(),
                summary_text=(
                    f"DISTRESS[{obs.robot_id}]: nose-down pitch stall "
                    f"(pitch={degrees(obs.pitch_rad):.1f}deg) -> request peer {tech.value}"
                ),
            )

        # 2. Check Bogged / High Wheel-Slip Entrapment (S1 / S2 / S6)
        if obs.is_bogged or slip_ratio > 0.80:
            is_deep_mud = (
                (local_moisture is not None and local_moisture > self.mud_moisture_threshold)
                or (local_firmness is not None and local_firmness < self.mud_firmness_threshold)
            )
            fmode = (
                FailureMode.CONTAGIOUS_MUD_TRAP
                if is_deep_mud
                else FailureMode.ASYMMETRIC_SLIP_OR_SUCTION
            )
            tech, pull_n = self._select_mechanical_technique(
                obs, fmode, local_moisture, local_firmness, stuck_ticks
            )
            return DistressSignal(
                robot_id=obs.robot_id,
                time_s=obs.time_s,
                pose=obs.pose,
                failure_mode=fmode,
                recommended_technique=tech,
                pitch_deg=round(degrees(obs.pitch_rad), 2),
                roll_deg=round(degrees(obs.roll_rad), 2),
                slip_ratio=round(slip_ratio, 3),
                seeds_remaining=obs.seeds_remaining,
                required_pull_n=pull_n,
                self_recovery_ticks=stuck_ticks,
                needs_peer_assistance=not is_deep_mud,
                residual_weights=self.learner.export_weights_flat(),
                summary_text=(
                    f"DISTRESS[{obs.robot_id}]: {fmode.value} (slip={slip_ratio:.0%}, "
                    f"F_req={pull_n:.1f}N) -> technique={tech.value}"
                ),
            )

        # 3. Check Rangefinder Sensor Dropout
        if obs.front_range_m is None:
            return DistressSignal(
                robot_id=obs.robot_id,
                time_s=obs.time_s,
                pose=obs.pose,
                failure_mode=FailureMode.SENSOR_DROPOUT,
                recommended_technique=MechanicalRescueTechnique.DIRECT_PRY_AND_PULL,
                pitch_deg=round(degrees(obs.pitch_rad), 2),
                roll_deg=round(degrees(obs.roll_rad), 2),
                slip_ratio=round(slip_ratio, 3),
                seeds_remaining=obs.seeds_remaining,
                required_pull_n=12.0,
                self_recovery_ticks=stuck_ticks,
                needs_peer_assistance=True,
                summary_text=f"DISTRESS[{obs.robot_id}]: front ToF sensor dropout -> holding for peer escort",
            )

        return None

    def _diagnose_failure_and_trap_odds(
        self,
        stranded_robot: RobotObservation,
        nearest_site: Site | None,
        estimates: dict[str, SiteEstimate],
        sites: dict[str, Site],
    ) -> tuple[FailureMode, float, bool, float]:
        """Return (failure_mode, trap_probability, trap_odds_uncertain, hazard_risk)."""
        # 1. S4: Submerged Irrigation Trench Drop-In / Tip-Over Hazard (>20 deg pitch/roll)
        if (
            abs(stranded_robot.pitch_rad) > self.max_pitch_roll_rad
            or abs(stranded_robot.roll_rad) > self.max_pitch_roll_rad
        ):
            return FailureMode.TIP_OVER_OR_TRENCH, 0.80, False, 1.80

        # 2. Check proximity to known permitted=False hazard zones
        for s in sites.values():
            if not s.permitted:
                dist_to_hazard = hypot(
                    stranded_robot.pose.x_m - s.center.x_m,
                    stranded_robot.pose.y_m - s.center.y_m,
                )
                if dist_to_hazard <= s.no_go_radius_m + 0.25:
                    return FailureMode.NO_GO_HAZARD, 0.85, False, 2.0

        if nearest_site is None:
            return FailureMode.UNKNOWN_TERRAIN_TRAP, 0.50, True, 1.0

        nearest_est = estimates[nearest_site.site_id]
        has_soil_data = (
            nearest_est.moisture is not None and nearest_est.firmness is not None
        )

        if has_soil_data:
            assert nearest_est.moisture is not None and nearest_est.firmness is not None
            wet_excess = max(0.0, nearest_est.moisture - 0.70)
            soft_excess = max(0.0, 0.30 - nearest_est.firmness)
            trap_prob = round(min(0.95, 0.05 + 2.5 * (wet_excess + soft_excess)), 4)
            is_mud_trap = (
                nearest_est.moisture > self.mud_moisture_threshold
                or nearest_est.firmness < self.mud_firmness_threshold
                or trap_prob >= 0.55
            )
            if is_mud_trap:
                return (
                    FailureMode.CONTAGIOUS_MUD_TRAP,
                    trap_prob,
                    False,
                    round(trap_prob * 2.0, 4),
                )
            if stranded_robot.front_range_m is None:
                return FailureMode.SENSOR_DROPOUT, 0.05, False, 0.05
            if stranded_robot.is_bogged or (
                stranded_robot.true_speed_m_s is not None
                and stranded_robot.true_speed_m_s < 0.03
            ):
                return (
                    FailureMode.ASYMMETRIC_SLIP_OR_SUCTION,
                    trap_prob,
                    False,
                    round(trap_prob * 0.6, 4),
                )
            return (
                FailureMode.SURFACE_OBSTRUCTION,
                trap_prob,
                False,
                round(trap_prob * 0.5, 4),
            )

        # Soil at the stranded site has NOT been measured yet!
        if stranded_robot.front_range_m is None:
            return FailureMode.SENSOR_DROPOUT, 0.15, False, 0.15
        # Physical immobilization in unmeasured terrain -> unknown mud vs obstacle!
        return FailureMode.UNKNOWN_TERRAIN_TRAP, 0.35, True, 0.40

    def evaluate(
        self,
        stranded_robot: RobotObservation,
        healthy_peers: list[RobotObservation],
        sites: dict[str, Site],
        estimates: dict[str, SiteEstimate],
        allowed_site_ids: list[str],
        eligible_fn,
    ) -> RescueAssessment:
        """Compute full agentic rescue triage assessment, recovery action, and mechanical technique."""
        active_compute_nodes = sum(
            1 for r in ([stranded_robot] + healthy_peers) if r.link_ok
        )
        permitted_sites = [sites[sid] for sid in allowed_site_ids if sid in sites]
        nearest_site = (
            min(
                permitted_sites,
                key=lambda s: hypot(
                    s.center.x_m - stranded_robot.pose.x_m,
                    s.center.y_m - stranded_robot.pose.y_m,
                ),
            )
            if permitted_sites
            else None
        )

        failure_mode, trap_prob, uncertain, hazard_risk = (
            self._diagnose_failure_and_trap_odds(
                stranded_robot, nearest_site, estimates, sites
            )
        )
        nearest_est = estimates[nearest_site.site_id] if nearest_site else None
        stuck_ticks = self._stuck_counts.get(stranded_robot.robot_id, 2)
        mech_tech, req_pull_n = self._select_mechanical_technique(
            stranded_robot,
            failure_mode,
            nearest_est.moisture if nearest_est else None,
            nearest_est.firmness if nearest_est else None,
            stuck_ticks,
        )

        if (
            not healthy_peers
            or nearest_site is None
            or failure_mode in (FailureMode.NO_GO_HAZARD, FailureMode.TIP_OVER_OR_TRENCH)
        ):
            return RescueAssessment(
                stranded_robot_id=stranded_robot.robot_id,
                verdict=RescueVerdict.ABANDON_ESCALATE,
                failure_mode=failure_mode,
                recovery_action=RecoveryAction.SAFE_ABSTAIN_DEFER_HUMAN,
                mechanical_technique=mech_tech,
                nearest_site_id=nearest_site.site_id if nearest_site else None,
                perimeter_probe_site_id=None,
                rescue_benefit=0.0,
                opportunity_cost=0.0,
                hazard_risk=hazard_risk,
                trap_probability=trap_prob,
                expected_trap_loss=round(trap_prob * 1.5, 4),
                net_rescue_value=-hazard_risk,
                required_pull_n=req_pull_n,
                trap_odds_uncertain=uncertain,
                active_compute_nodes=active_compute_nodes,
                helper_robot_id=None,
                reason="not worth saving: high hazard risk or no available helper; safe HOLD for human",
            )

        helper = min(
            healthy_peers,
            key=lambda p: hypot(
                p.pose.x_m - stranded_robot.pose.x_m,
                p.pose.y_m - stranded_robot.pose.y_m,
            ),
        )
        rescue_dist = hypot(
            helper.pose.x_m - stranded_robot.pose.x_m,
            helper.pose.y_m - stranded_robot.pose.y_m,
        )

        # S7 Check: Is the candidate rescuer already experiencing secondary wheel slip / bogging?
        helper_slipping = helper.is_bogged or (
            helper.true_speed_m_s is not None and helper.true_speed_m_s < 0.04
        )
        if helper_slipping:
            return RescueAssessment(
                stranded_robot_id=stranded_robot.robot_id,
                verdict=RescueVerdict.ABANDON_ESCALATE,
                failure_mode=FailureMode.CASCADING_TOW_OVERLOAD,
                recovery_action=RecoveryAction.SAFE_ABSTAIN_DEFER_HUMAN,
                mechanical_technique=MechanicalRescueTechnique.SPADE_ANCHOR_ROLLOVER_HOLD,
                nearest_site_id=nearest_site.site_id,
                perimeter_probe_site_id=None,
                rescue_benefit=0.0,
                opportunity_cost=0.0,
                hazard_risk=1.5,
                trap_probability=0.75,
                expected_trap_loss=1.2,
                net_rescue_value=-1.5,
                required_pull_n=req_pull_n,
                trap_odds_uncertain=False,
                active_compute_nodes=active_compute_nodes,
                helper_robot_id=None,
                reason=f"abort rescue (S7 cascading overload): rescuer {helper.robot_id} experiencing secondary slip",
            )

        healthy_seeds = sum(p.seeds_remaining for p in healthy_peers)
        remaining_work_sites = len(allowed_site_ids)
        confirmed_gaps = sum(1 for sid in allowed_site_ids if eligible_fn(estimates[sid]))

        # 1. Rescue Benefit: Stranded Seeds + Offline Data + Pooled Swarm Compute Node Shard
        seeds_needed_from_stranded = max(
            0, min(stranded_robot.seeds_remaining, remaining_work_sites - healthy_seeds)
        )
        seed_recovery_value = 0.95 * seeds_needed_from_stranded
        offline_data_value = 0.55 if not stranded_robot.link_ok else 0.0
        compute_shard_value = (
            0.35 if stranded_robot.seeds_remaining > 0 else 0.10
        )
        rescue_benefit = round(
            seed_recovery_value + offline_data_value + compute_shard_value, 4
        )

        # 2. Expected Trap Loss (what we lose if the helper ALSO gets trapped)
        helper_value_at_risk = 0.50 + 0.40 * helper.seeds_remaining
        expected_trap_loss = round(trap_prob * helper_value_at_risk, 4)

        # S1 Contagious mud trap -> abort rescue to protect the healthy peer
        if failure_mode == FailureMode.CONTAGIOUS_MUD_TRAP or trap_prob >= 0.55:
            return RescueAssessment(
                stranded_robot_id=stranded_robot.robot_id,
                verdict=RescueVerdict.ABANDON_ESCALATE,
                failure_mode=failure_mode,
                recovery_action=RecoveryAction.SAFE_ABSTAIN_DEFER_HUMAN,
                mechanical_technique=MechanicalRescueTechnique.SPADE_ANCHOR_ROLLOVER_HOLD,
                nearest_site_id=nearest_site.site_id,
                perimeter_probe_site_id=None,
                rescue_benefit=rescue_benefit,
                opportunity_cost=0.0,
                hazard_risk=hazard_risk,
                trap_probability=trap_prob,
                expected_trap_loss=expected_trap_loss,
                net_rescue_value=round(
                    (1.0 - trap_prob) * rescue_benefit - expected_trap_loss, 4
                ),
                required_pull_n=req_pull_n,
                trap_odds_uncertain=False,
                active_compute_nodes=active_compute_nodes,
                helper_robot_id=None,
                reason=f"not worth saving: contagious mud trap ({trap_prob:.0%} trap odds); protect {helper.robot_id}",
            )

        # 3. Opportunity Cost (delaying active planting + travel distance)
        helper_can_plant_now = helper.seeds_remaining > 0 and confirmed_gaps > 0
        active_planting_cost = 1.45 if helper_can_plant_now else 0.25
        travel_cost = self.travel_cost_weight * rescue_dist
        opportunity_cost = round(active_planting_cost + travel_cost, 4)

        net_value = round(
            rescue_benefit - opportunity_cost - hazard_risk, 4
        )

        if net_value > 0.0:
            if uncertain:
                return RescueAssessment(
                    stranded_robot_id=stranded_robot.robot_id,
                    verdict=RescueVerdict.INVESTIGATE_PERIMETER,
                    failure_mode=failure_mode,
                    recovery_action=RecoveryAction.INVESTIGATE_PERIMETER,
                    mechanical_technique=MechanicalRescueTechnique.SELF_OSCILLATE_DIBBER,
                    nearest_site_id=nearest_site.site_id,
                    perimeter_probe_site_id=nearest_site.site_id,
                    rescue_benefit=rescue_benefit,
                    opportunity_cost=opportunity_cost,
                    hazard_risk=hazard_risk,
                    trap_probability=trap_prob,
                    expected_trap_loss=expected_trap_loss,
                    net_rescue_value=net_value,
                    required_pull_n=req_pull_n,
                    trap_odds_uncertain=True,
                    active_compute_nodes=active_compute_nodes,
                    helper_robot_id=helper.robot_id,
                    reason=(
                        f"probe perimeter at {nearest_site.site_id} first "
                        f"(trap_odds={trap_prob:.0%} uncertain): verify soil before rescue"
                    ),
                )
            return RescueAssessment(
                stranded_robot_id=stranded_robot.robot_id,
                verdict=RescueVerdict.RESCUE_NOW,
                failure_mode=failure_mode,
                recovery_action=RecoveryAction.DISPATCH_RESCUE_DIRECT,
                mechanical_technique=mech_tech,
                nearest_site_id=nearest_site.site_id,
                perimeter_probe_site_id=None,
                rescue_benefit=rescue_benefit,
                opportunity_cost=opportunity_cost,
                hazard_risk=hazard_risk,
                trap_probability=trap_prob,
                expected_trap_loss=expected_trap_loss,
                net_rescue_value=net_value,
                required_pull_n=req_pull_n,
                trap_odds_uncertain=False,
                active_compute_nodes=active_compute_nodes,
                helper_robot_id=helper.robot_id,
                reason=(
                    f"worth saving now (net={net_value:+.2f}, trap_odds={trap_prob:.0%}, "
                    f"mode={failure_mode.value}, tech={mech_tech.value}): carries needed seeds/compute"
                ),
            )

        if rescue_benefit >= 0.60 and helper_can_plant_now:
            return RescueAssessment(
                stranded_robot_id=stranded_robot.robot_id,
                verdict=RescueVerdict.DEFER_RESCUE,
                failure_mode=failure_mode,
                recovery_action=RecoveryAction.REALLOCATE_AND_BEACON,
                mechanical_technique=mech_tech,
                nearest_site_id=nearest_site.site_id,
                perimeter_probe_site_id=nearest_site.site_id if uncertain else None,
                rescue_benefit=rescue_benefit,
                opportunity_cost=opportunity_cost,
                hazard_risk=hazard_risk,
                trap_probability=trap_prob,
                expected_trap_loss=expected_trap_loss,
                net_rescue_value=net_value,
                required_pull_n=req_pull_n,
                trap_odds_uncertain=uncertain,
                active_compute_nodes=active_compute_nodes,
                helper_robot_id=helper.robot_id,
                reason=f"defer rescue (net={net_value:+.2f}): reallocate tasks & act as UWB/ESP-NOW beacon while peers plant",
            )

        return RescueAssessment(
            stranded_robot_id=stranded_robot.robot_id,
            verdict=RescueVerdict.ABANDON_ESCALATE,
            failure_mode=failure_mode,
            recovery_action=RecoveryAction.REALLOCATE_AND_BEACON
            if stranded_robot.link_ok
            else RecoveryAction.SAFE_ABSTAIN_DEFER_HUMAN,
            mechanical_technique=mech_tech,
            nearest_site_id=nearest_site.site_id,
            perimeter_probe_site_id=None,
            rescue_benefit=rescue_benefit,
            opportunity_cost=opportunity_cost,
            hazard_risk=hazard_risk,
            trap_probability=trap_prob,
            expected_trap_loss=expected_trap_loss,
            net_rescue_value=net_value,
            required_pull_n=req_pull_n,
            trap_odds_uncertain=uncertain,
            active_compute_nodes=active_compute_nodes,
            helper_robot_id=None,
            reason=(
                f"not worth saving (net={net_value:+.2f}, trap_odds={trap_prob:.0%}): "
                f"healthy peers can complete mission; escalate to human"
            ),
        )
