"""Small, standard-library-only rice.v1 boundary. No Isaac Sim imports here.

Only sensor observations cross into the mission code; hidden simulator field
quality and future scenario events must remain outside this module.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import isfinite
from typing import Any


def _finite(*numbers: float) -> None:
    if not all(isfinite(n) for n in numbers):
        raise ValueError("non-finite physical quantity")


@dataclass(frozen=True)
class Pose2D:
    x_m: float
    y_m: float
    yaw_rad: float

    def __post_init__(self) -> None:
        _finite(self.x_m, self.y_m, self.yaw_rad)


@dataclass(frozen=True)
class Site:
    site_id: str
    center: Pose2D
    permitted: bool = True  # publicly known traversability/no-go constraint
    no_go_radius_m: float = 0.3

    def __post_init__(self) -> None:
        _finite(self.no_go_radius_m)
        if not self.site_id or self.no_go_radius_m < 0:
            raise ValueError("invalid site ID or no-go radius")


class SensorKind(str, Enum):
    MOISTURE = "moisture_proxy"
    FIRMNESS = "firmness_proxy"
    STAND_GAP = "stand_gap_proxy"


@dataclass(frozen=True)
class SoilSample:
    sample_id: str
    robot_id: str
    site_id: str
    kind: SensorKind
    value: float  # synthetic dimensionless 0..1, NOT calibrated field science
    sampled_at_s: float
    location: Pose2D  # physical probe location, estimated; not a world-truth lookup
    valid: bool = True

    def __post_init__(self) -> None:
        if not self.sample_id or not self.robot_id or not self.site_id:
            raise ValueError("sample, robot and site IDs are required")
        _finite(self.value, self.sampled_at_s)
        if not 0 <= self.value <= 1 or self.sampled_at_s < 0:
            raise ValueError("invalid synthetic measurement or timestamp")


@dataclass(frozen=True)
class RobotCapability:
    robot_id: str
    can_sample: bool = True
    can_plant: bool = False

    def __post_init__(self) -> None:
        if not self.robot_id:
            raise ValueError("robot ID required")


@dataclass(frozen=True)
class RobotObservation:
    robot_id: str
    pose: Pose2D  # estimated/odometric pose, never true simulator position
    time_s: float
    front_range_m: float | None  # None means missing/invalid, not unlimited range
    seeds_remaining: int
    samples: tuple[SoilSample, ...] = ()
    link_ok: bool = True
    pitch_rad: float = 0.0
    roll_rad: float = 0.0
    true_speed_m_s: float | None = None
    row_cross_track_m: float = 0.0
    peer_distances: tuple[tuple[str, float], ...] = ()
    is_bogged: bool = False

    def __post_init__(self) -> None:
        if not self.robot_id:
            raise ValueError("robot ID required")
        _finite(self.time_s)
        if self.time_s < 0 or self.seeds_remaining < 0:
            raise ValueError("negative time or seed inventory")
        if self.front_range_m is not None:
            _finite(self.front_range_m)
            if self.front_range_m < 0:
                raise ValueError("negative range")
        _finite(self.pitch_rad, self.roll_rad, self.row_cross_track_m)
        if self.true_speed_m_s is not None:
            _finite(self.true_speed_m_s)
        for sample in self.samples:
            if sample.robot_id != self.robot_id or sample.sampled_at_s != self.time_s:
                raise ValueError("samples must be current, local and from the same robot")


@dataclass(frozen=True)
class PlantFeedback:
    robot_id: str
    task_id: str
    released: bool
    observed_at_s: float
    landing: Pose2D | None = None  # reported physics contact, NOT crop success

    def __post_init__(self) -> None:
        _finite(self.observed_at_s)
        if self.observed_at_s < 0 or (self.landing is not None and not self.released):
            raise ValueError("invalid planting feedback")


@dataclass(frozen=True)
class RecoveryFeedback:
    """Physics-confirmed peer rescue outcome reported by SimulationPort (never faked by planner)."""

    helper_robot_id: str
    target_robot_id: str
    task_id: str
    latched: bool
    extracted: bool
    victim_displacement_m: float
    observed_at_s: float
    phase: str = ""
    peak_constraint_force_n: float = 0.0
    helper_pitch_rad: float = 0.0
    helper_safe: bool = True
    technique: str = "direct_pry_and_pull"

    def __post_init__(self) -> None:
        _finite(
            self.victim_displacement_m,
            self.observed_at_s,
            self.peak_constraint_force_n,
            self.helper_pitch_rad,
        )
        if (
            not self.helper_robot_id
            or not self.target_robot_id
            or self.observed_at_s < 0
            or self.victim_displacement_m < 0
            or self.peak_constraint_force_n < 0
        ):
            raise ValueError("invalid recovery feedback")


@dataclass(frozen=True)
class ObservationFrame:
    episode_id: str
    time_s: float
    robots: tuple[RobotObservation, ...]
    plant_feedback: tuple[PlantFeedback, ...] = ()
    recovery_feedback: tuple[RecoveryFeedback, ...] = ()

    def __post_init__(self) -> None:
        if not self.episode_id:
            raise ValueError("episode ID required")
        _finite(self.time_s)
        ids = [r.robot_id for r in self.robots]
        if len(ids) != len(set(ids)):
            raise ValueError("duplicate robot ID")
        if self.time_s < 0 or any(r.time_s != self.time_s for r in self.robots):
            raise ValueError("frame and robot clock must match")
        if any(f.observed_at_s > self.time_s for f in self.plant_feedback):
            raise ValueError("feedback cannot come from the future")
        if any(f.observed_at_s > self.time_s for f in self.recovery_feedback):
            raise ValueError("recovery feedback cannot come from the future")


class TaskMode(str, Enum):
    SAMPLE = "sample"
    PLANT = "plant"
    HOLD = "hold"
    RESCUE = "rescue"


@dataclass(frozen=True)
class TaskIntent:
    task_id: str
    robot_id: str
    mode: TaskMode
    issued_at_s: float
    expires_at_s: float
    target_site_id: str | None = None
    target_robot_id: str | None = None
    reason: str = ""
    recovery_technique: str = ""

    def __post_init__(self) -> None:
        _finite(self.issued_at_s, self.expires_at_s)
        if not self.robot_id or not self.task_id or self.expires_at_s <= self.issued_at_s:
            raise ValueError("invalid task TTL or actor")
        if self.mode in (TaskMode.SAMPLE, TaskMode.PLANT) and not self.target_site_id:
            raise ValueError("active tasks require a site")
        if self.mode == TaskMode.RESCUE and not (self.target_robot_id or self.target_site_id):
            raise ValueError("rescue tasks require a target robot or rendezvous site")


@dataclass(frozen=True)
class RobotCommand:
    robot_id: str
    issued_at_s: float
    expires_at_s: float
    task_id: str = ""
    linear_m_s: float = 0.0
    angular_rad_s: float = 0.0
    release_seed: bool = False
    target_site_id: str | None = None
    recovery_target_id: str | None = None
    dock_pitch_rad: float = 0.0
    latch_rad: float = 0.0
    spade_rad: float = 0.0
    recovery_technique: str = ""

    def __post_init__(self) -> None:
        _finite(
            self.issued_at_s,
            self.expires_at_s,
            self.linear_m_s,
            self.angular_rad_s,
            self.dock_pitch_rad,
            self.latch_rad,
            self.spade_rad,
        )
        if not self.robot_id or self.expires_at_s <= self.issued_at_s:
            raise ValueError("invalid command TTL or actor")
        if not (-0.20 <= self.dock_pitch_rad <= 0.15):
            raise ValueError("dock_pitch_rad out of safe grounded bounds [-0.20, 0.15]")
        if not (0.0 <= self.latch_rad <= 0.65):
            raise ValueError("latch_rad out of bounds [0.0, 0.65]")
        if not (0.0 <= self.spade_rad <= 0.85):
            raise ValueError("spade_rad out of bounds [0.0, 0.85]")
        if self.release_seed and (not self.target_site_id or not self.task_id or self.linear_m_s or self.angular_rad_s):
            raise ValueError("plant release needs target and zero drive")


@dataclass(frozen=True)
class Decision:
    intents: tuple[TaskIntent, ...]
    commands: tuple[RobotCommand, ...]
    ingested_samples: int

    def __post_init__(self) -> None:
        for values in (self.intents, self.commands):
            ids = [item.robot_id for item in values]
            if len(ids) != len(set(ids)):
                raise ValueError("at most one instruction per robot")


# ---------------------------------------------------------------------------
# R1 Cross-Domain Contract Handshake Extensions (Agent A Sole-Owner Seams)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class FaultEvidence:
    """Sensor-derived fault evidence (commanded vs observed motion, range validity, attitude).

    Enforces negative-control invariants: missing front range (`front_range_m is None`) or
    zero commanded motion (`HOLD` / seed drop) can never be classified as a commanded-motion
    immobilisation stall (`is_commanded_motion_stall=True`).
    """

    robot_id: str
    observed_at_s: float
    commanded_linear_m_s: float = 0.0
    commanded_angular_rad_s: float = 0.0
    observed_displacement_m: float = 0.0
    observed_yaw_delta_rad: float = 0.0
    observed_speed_m_s: float | None = None
    slip_ratio: float = 0.0
    pitch_rad: float = 0.0
    roll_rad: float = 0.0
    front_range_m: float | None = None
    link_ok: bool = True
    stalled_ticks: int = 0
    is_commanded_motion_stall: bool = False
    confidence: float = 1.0

    def __post_init__(self) -> None:
        if not self.robot_id:
            raise ValueError("robot ID required")
        _finite(
            self.observed_at_s,
            self.commanded_linear_m_s,
            self.commanded_angular_rad_s,
            self.observed_displacement_m,
            self.observed_yaw_delta_rad,
            self.slip_ratio,
            self.pitch_rad,
            self.roll_rad,
            self.confidence,
        )
        if self.observed_at_s < 0 or self.observed_displacement_m < 0 or self. observed_yaw_delta_rad < 0:
            raise ValueError("timestamps and displacement magnitudes must be non-negative")
        if self.stalled_ticks < 0 or not (0.0 <= self.confidence <= 1.0):
            raise ValueError("invalid stalled_ticks or confidence outside [0.0, 1.0]")
        if self.observed_speed_m_s is not None:
            _finite(self.observed_speed_m_s)
        if self.front_range_m is not None:
            _finite(self.front_range_m)
            if self.front_range_m < 0:
                raise ValueError("negative front_range_m")
        actively_commanded = (
            abs(self.commanded_linear_m_s) >= 0.04 or abs(self.commanded_angular_rad_s) >= 0.10
        )
        if self.is_commanded_motion_stall:
            if not actively_commanded:
                raise ValueError("cannot claim commanded-motion stall without active drive command")
            if self.front_range_m is None:
                raise ValueError("missing rangefinder (None) is sensor invalidity, not motion stall")


class RescueVerdict(str, Enum):
    """Canonical triage decision for whether a stranded/faulted robot is worth saving."""

    HEALTHY = "healthy"
    SELF_RECOVER = "self_recover"
    INVESTIGATE_PERIMETER = "investigate_perimeter"
    RESCUE_NOW = "rescue_now"
    DEFER_RESCUE = "defer_rescue"
    ABANDON_ESCALATE = "abandon_escalate"


@dataclass(frozen=True)
class RescueAssessment:
    """Canonical cost-benefit rescue assessment reconciling Policy A, Policy B, Orchestrator D, and Viewer E fields.

    Synchronises canonical attributes (`helper_robot_id`, `rescue_benefit`, `opportunity_cost`,
    `net_rescue_value`, `reason`) and Policy B / UI inspector aliases (`rescuer_robot_id`,
    `expected_saved_benefit`, `rescue_opportunity_cost`, `net_rescue_utility`, `rationale`)
    so any producer or consumer reading either naming convention observes identical values.
    """

    stranded_robot_id: str
    verdict: RescueVerdict
    failure_mode: Any = "unknown_terrain_trap"
    recovery_action: Any = "investigate_perimeter"
    mechanical_technique: Any = "none"
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
    safety_veto_applied: bool = False
    fallback_used: bool = True
    tool_call_count: int = 0
    tool_trace: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not self.stranded_robot_id:
            raise ValueError("stranded_robot_id required")
        normalized_verdict = (
            self.verdict
            if isinstance(self.verdict, RescueVerdict)
            else RescueVerdict(self.verdict.value if isinstance(self.verdict, Enum) else str(self.verdict))
        )
        object.__setattr__(self, "verdict", normalized_verdict)
        _finite(
            self.rescue_benefit,
            self.opportunity_cost,
            self.hazard_risk,
            self.trap_probability,
            self.expected_trap_loss,
            self.net_rescue_value,
            self.required_pull_n,
            self.expected_saved_benefit,
            self.rescue_opportunity_cost,
            self.net_rescue_utility,
        )
        helper = self.helper_robot_id if self.helper_robot_id is not None else self.rescuer_robot_id
        object.__setattr__(self, "helper_robot_id", helper)
        object.__setattr__(self, "rescuer_robot_id", helper)

        benefit = self.rescue_benefit if self.rescue_benefit != 0.0 else self.expected_saved_benefit
        object.__setattr__(self, "rescue_benefit", benefit)
        object.__setattr__(self, "expected_saved_benefit", benefit)

        opp_cost = self.opportunity_cost if self.opportunity_cost != 0.0 else self.rescue_opportunity_cost
        object.__setattr__(self, "opportunity_cost", opp_cost)
        object.__setattr__(self, "rescue_opportunity_cost", opp_cost)

        net_val = self.net_rescue_value if self.net_rescue_value != 0.0 else self.net_rescue_utility
        object.__setattr__(self, "net_rescue_value", net_val)
        object.__setattr__(self, "net_rescue_utility", net_val)

        why = self.reason if self.reason else self.rationale
        object.__setattr__(self, "reason", why)
        object.__setattr__(self, "rationale", why)

    def to_dict(self) -> dict[str, object]:
        def _val(v: Any) -> object:
            return v.value if isinstance(v, Enum) else v

        return {
            "stranded_robot_id": self.stranded_robot_id,
            "verdict": self.verdict.value,
            "failure_mode": _val(self.failure_mode),
            "recovery_action": _val(self.recovery_action),
            "mechanical_technique": _val(self.mechanical_technique),
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
            "safety_veto_applied": self.safety_veto_applied,
            "fallback_used": self.fallback_used,
            "tool_call_count": self.tool_call_count,
            "tool_trace": list(self.tool_trace),
        }

    @classmethod
    def from_any(cls, assessment: Any) -> RescueAssessment:
        """Convert any duck-typed planner/rescue RescueAssessment into a canonical RescueAssessment."""
        if isinstance(assessment, cls):
            return assessment
        return cls(
            stranded_robot_id=getattr(assessment, "stranded_robot_id", ""),
            verdict=getattr(assessment, "verdict", RescueVerdict.HEALTHY),
            failure_mode=getattr(assessment, "failure_mode", "unknown_terrain_trap"),
            recovery_action=getattr(assessment, "recovery_action", "investigate_perimeter"),
            mechanical_technique=getattr(assessment, "mechanical_technique", "none"),
            nearest_site_id=getattr(assessment, "nearest_site_id", None),
            perimeter_probe_site_id=getattr(assessment, "perimeter_probe_site_id", None),
            rescue_benefit=float(getattr(assessment, "rescue_benefit", 0.0) or 0.0),
            opportunity_cost=float(getattr(assessment, "opportunity_cost", 0.0) or 0.0),
            hazard_risk=float(getattr(assessment, "hazard_risk", 0.0) or 0.0),
            trap_probability=float(getattr(assessment, "trap_probability", 0.0) or 0.0),
            expected_trap_loss=float(getattr(assessment, "expected_trap_loss", 0.0) or 0.0),
            net_rescue_value=float(getattr(assessment, "net_rescue_value", 0.0) or 0.0),
            required_pull_n=float(getattr(assessment, "required_pull_n", 0.0) or 0.0),
            trap_odds_uncertain=bool(getattr(assessment, "trap_odds_uncertain", False)),
            active_compute_nodes=int(getattr(assessment, "active_compute_nodes", 1) or 1),
            helper_robot_id=getattr(assessment, "helper_robot_id", None),
            reason=str(getattr(assessment, "reason", "") or ""),
            rescuer_robot_id=getattr(assessment, "rescuer_robot_id", None),
            expected_saved_benefit=float(getattr(assessment, "expected_saved_benefit", 0.0) or 0.0),
            rescue_opportunity_cost=float(getattr(assessment, "rescue_opportunity_cost", 0.0) or 0.0),
            net_rescue_utility=float(getattr(assessment, "net_rescue_utility", 0.0) or 0.0),
            rationale=str(getattr(assessment, "rationale", "") or ""),
            safety_veto_applied=bool(getattr(assessment, "safety_veto_applied", False)),
            fallback_used=bool(getattr(assessment, "fallback_used", True)),
            tool_call_count=int(getattr(assessment, "tool_call_count", 0) or 0),
            tool_trace=tuple(getattr(assessment, "tool_trace", ()) or ()),
        )


class OperatorCommandKind(str, Enum):
    """Typed operator/experiment request kinds queued into EpisodeSession (no direct qpos writes)."""

    PAUSE = "pause"
    RESUME = "resume"
    RESET = "reset"
    STEP_ONCE = "step_once"
    INJECT_OBSTACLE = "inject_obstacle"
    INJECT_MUD_FAULT = "inject_mud_fault"
    SET_COMMS_MODE = "set_comms_mode"
    EXPERIMENT_VEHICLE_SIZING = "experiment_vehicle_sizing"
    TELEOP_OVERRIDE = "teleop_override"


@dataclass(frozen=True)
class OperatorRequest:
    """Logged operator or experiment intervention validated and applied only by the session owner."""

    request_id: str
    kind: OperatorCommandKind
    issued_at_s: float
    source: str = "operator"
    payload: tuple[tuple[str, str | float | int | bool], ...] = ()

    def __post_init__(self) -> None:
        if not self.request_id or not self.source:
            raise ValueError("request_id and source are required")
        _finite(self.issued_at_s)
        if self.issued_at_s < 0:
            raise ValueError("negative issued_at_s")


@dataclass(frozen=True)
class TickTrace:
    """Authoritative record of one simulation step and one chosen policy decision."""

    tick_index: int
    time_s: float
    frame: ObservationFrame
    decision: Decision
    next_time_s: float
    operator_requests: tuple[OperatorRequest, ...] = ()

    def __post_init__(self) -> None:
        _finite(self.time_s, self.next_time_s)
        if self.tick_index < 0 or self.time_s < 0 or self.next_time_s < self.time_s:
            raise ValueError("invalid tick index or non-monotonic trace timestamps")


@dataclass(frozen=True)
class EpisodeTrace:
    """Complete typed episode trace supporting CLI/GUI parity verification and deterministic replay."""

    episode_id: str
    seed: int
    scenario: str
    policy: str
    comms_mode: str
    ticks: tuple[TickTrace, ...]

    def __post_init__(self) -> None:
        if not self.episode_id or self.seed < 0 or isinstance(self.seed, bool):
            raise ValueError("episode_id and non-negative integer seed required")

    def command_digest(self) -> tuple[tuple[float, tuple[tuple[str, float, float, bool, str | None, str | None], ...]], ...]:
        """Return a canonical per-tick command and timing digest for deterministic parity checks."""
        return tuple(
            (
                round(t.time_s, 6),
                tuple(
                    (
                        c.robot_id,
                        round(c.linear_m_s, 6),
                        round(c.angular_rad_s, 6),
                        c.release_seed,
                        c.target_site_id,
                        c.recovery_target_id,
                    )
                    for c in t.decision.commands
                ),
            )
            for t in self.ticks
        )

    def pose_digest(self) -> tuple[tuple[float, tuple[tuple[str, float, float, float, bool], ...]], ...]:
        """Return a canonical per-tick robot pose and bogged-state digest."""
        return tuple(
            (
                round(t.time_s, 6),
                tuple(
                    (
                        r.robot_id,
                        round(r.pose.x_m, 6),
                        round(r.pose.y_m, 6),
                        round(r.pose.yaw_rad, 6),
                        r.is_bogged,
                    )
                    for r in t.frame.robots
                ),
            )
            for t in self.ticks
        )

    def matches_parity(self, other: EpisodeTrace) -> bool:
        """Verify tick-for-tick behavioral parity (poses, commands, plant and recovery feedback) across runs."""
        if (
            self.seed != other.seed
            or self.scenario != other.scenario
            or self.policy != other.policy
            or self.comms_mode != other.comms_mode
            or len(self.ticks) != len(other.ticks)
        ):
            return False
        return (
            self.command_digest() == other.command_digest()
            and self.pose_digest() == other.pose_digest()
            and tuple(t.frame.plant_feedback for t in self.ticks)
            == tuple(t.frame.plant_feedback for t in other.ticks)
            and tuple(t.frame.recovery_feedback for t in self.ticks)
            == tuple(t.frame.recovery_feedback for t in other.ticks)
        )
