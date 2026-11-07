"""Small, standard-library-only rice.v1 boundary. No Isaac Sim imports here.

Only sensor observations cross into the mission code; hidden simulator field
quality and future scenario events must remain outside this module.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import isfinite


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

    def __post_init__(self) -> None:
        _finite(self.victim_displacement_m, self.observed_at_s)
        if (
            not self.helper_robot_id
            or not self.target_robot_id
            or self.observed_at_s < 0
            or self.victim_displacement_m < 0
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
