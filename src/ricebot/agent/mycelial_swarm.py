r"""Decentralised Mycelial Collective Intelligence, Local SLAM & Event-Triggered Gossip (`src/ricebot/agent/`).

Implements **Policy B (`DecentralizedMycelialRuntime` — Local Intelligence Network)**, operating
over the exact same `SimulationPort` (`ObservationFrame` -> `Decision`) seam as **Policy A
(`MissionCore` Centralised Coordinator)** so both architectures can be compared head-to-head on
identical 3D physical episodes.

Key Architectural Properties (Zero Central Map / Zero Cloud Dependency):
1. **Strictly Local Per-Robot State (`DecentralizedRobotNode`):**
   - Each rover (`planter`, `scout`, etc.) maintains its own private `LocalMycelialMap`:
     - **2D Log-Odds SLAM Occupancy & Traversability Grid (`LocalSlamGrid`):** Built from the
       rover's own forward 3-ray LiDAR/ToF rangefinder fan, Hall-effect crop-row whiskers, and
       60GHz mmWave radar wheel-slip detection ($s = 1 - v_{\text{true}} / (\omega r)$), plus
       peer SLAM hazard cells received over wireless gossip.
     - **Per-Site Gaussian Belief State (`GaussianChannelBelief`):** Each robot independently
       tracks posterior mean $\mu$ and precision $\tau = 1/\sigma^2$ (uncertainty $\sigma$) for
       soil moisture, cone-index firmness, and stand gap, with a provenance ledger
       (`seen_sample_ids: set[str]`) preventing duplicate counting of relayed gossip.
2. **Mycelial Information Spreading (Event-Triggered Gossip over UWB + Wi-Fi HaLow / ESP-NOW):**
   - Dual Qorvo DWM3000 UWB ranging computes physical inter-robot distance $d_{ij}$ and horizontal
     Angle-of-Arrival ($\text{AoA}$).
   - Peers communicate **only** when $d_{ij} \le R_{\text{comm}}$ (and when the radio channel is
     open under intermittent-communication schedules).
   - Communication is **event-triggered**: a rover transmits a compact `MycelialGossipPacket` only
     when it holds new provenance-tagged `SoilSample`s, newly mapped SLAM obstacle cells, or
     a newly contested task-bid negotiation.
3. **Decentralised Task Negotiation (Capability-Aware Local Auction / Consensus Bidding):**
   - Instead of obeying a central supervisor, each robot computes local task bids (`TaskBidProposal`)
     for `SAMPLE`, `REMEASURE` (when evidence is uncertain/borderline), or `PLANT` using Active
     Sensing (Upper Confidence Bound exploration bonus + capability match + SLAM route distance).
   - Reachable peers exchange compact `TaskBidProposal` messages and settle on a single task owner
     without central arbitration.
4. **Local SLAM Waypoint Bypass & Closed-Loop Slip Reflex (`input_output` Sensor Architecture):**
   - Uses each robot's local SLAM grid to compute a collision-free detour waypoint around
     unannounced mid-episode obstacles and protected living rice stands (`no_go_radius_m`).
"""
from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, pi, sin, sqrt

from ..contracts import (
    Decision,
    ObservationFrame,
    Pose2D,
    RobotCapability,
    RobotCommand,
    RobotObservation,
    SensorKind,
    Site,
    SoilSample,
    TaskIntent,
    TaskMode,
)
from ..control import ControlLimits
from ..mission import SiteEstimate


def _wrap_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


@dataclass(frozen=True)
class SoilSafetyEnvelope:
    """Agronomic eligibility envelope matching `SimpleMissionPlanner` defaults."""

    min_gap: float = 0.60
    moisture_range: tuple[float, float] = (0.25, 0.80)
    firmness_range: tuple[float, float] = (0.20, 0.90)

    def eligible(self, est: SiteEstimate) -> bool:
        return bool(
            est.complete
            and est.stand_gap is not None
            and est.moisture is not None
            and est.firmness is not None
            and est.stand_gap >= self.min_gap
            and self.moisture_range[0] <= est.moisture <= self.moisture_range[1]
            and self.firmness_range[0] <= est.firmness <= self.firmness_range[1]
        )


@dataclass(frozen=True)
class OnboardSensorTelemetry:
    """Processed digital outputs from the physical sensor suite specified in `input_output`."""

    robot_id: str
    time_s: float
    pitch_deg: float
    roll_deg: float
    yaw_rad: float
    true_ground_speed_m_s: float
    wheel_surface_speed_m_s: float
    slip_ratio: float
    slip_critical: bool
    front_tof_range_m: float | None
    lateral_crop_clearance_m: float
    uwb_peer_ranges_m: dict[str, float]
    uwb_peer_aoa_rad: dict[str, float]


@dataclass(frozen=True)
class SlamHazardCell:
    """Compact SLAM obstacle/mud-hazard cell discovered by onboard LiDAR/ToF or mmWave slip radar."""

    cell_ix: int
    cell_iy: int
    x_m: float
    y_m: float
    kind: str  # "obstacle" | "mud_slip"
    log_odds: float
    discovered_by: str
    discovered_at_s: float


@dataclass(frozen=True)
class TaskBidProposal:
    """Structured peer-to-peer task negotiation message (Consensus-Based Local Auction)."""

    robot_id: str
    site_id: str
    mode: TaskMode
    bid_score: float
    eta_s: float
    confidence: float
    proposal_text: str


@dataclass(frozen=True)
class MycelialGossipPacket:
    """Compact event-triggered Wi-Fi HaLow / ESP-NOW packet exchanged between reachable peers."""

    packet_id: str
    sender_id: str
    receiver_id: str
    sent_at_s: float
    uwb_distance_m: float
    uwb_aoa_rad: float
    samples: tuple[SoilSample, ...]
    slam_hazards: tuple[SlamHazardCell, ...]
    task_bid: TaskBidProposal | None
    byte_size: int
    dialogue_summary: tuple[str, ...]


@dataclass
class GaussianChannelBelief:
    """1D Gaussian belief state parameterized by mean and precision (inverse variance)."""

    mean: float = 0.5
    precision: float = 0.0
    count: int = 0

    def fuse(self, value: float, obs_precision: float = 4.0) -> None:
        new_prec = self.precision + obs_precision
        if self.precision == 0.0:
            self.mean = value
        else:
            self.mean = (self.precision * self.mean + obs_precision * value) / new_prec
        self.precision = new_prec
        self.count += 1

    @property
    def uncertainty(self) -> float:
        if self.count == 0 or self.precision <= 0.0:
            return 1.0
        return round(1.0 / sqrt(1.0 + self.precision * 0.25), 4)


class LocalSlamGrid:
    """Per-robot 2D log-odds SLAM occupancy & mud-slip traversability grid (resolution = 0.15m)."""

    def __init__(self, resolution_m: float = 0.15) -> None:
        self.resolution_m = resolution_m
        self.cells: dict[tuple[int, int], SlamHazardCell] = {}

    def to_cell(self, x_m: float, y_m: float) -> tuple[int, int]:
        return (int(round(x_m / self.resolution_m)), int(round(y_m / self.resolution_m)))

    def cell_center(self, ix: int, iy: int) -> tuple[float, float]:
        return (round(ix * self.resolution_m, 3), round(iy * self.resolution_m, 3))

    def integrate_lidar_hit(
        self,
        robot_id: str,
        pose: Pose2D,
        front_range_m: float | None,
        known_sites: dict[str, Site],
        peer_poses: dict[str, Pose2D],
        time_s: float,
    ) -> SlamHazardCell | None:
        """Project forward rangefinder hit into world coordinates and update local SLAM log-odds."""
        if front_range_m is None or front_range_m >= 0.65 or front_range_m <= 0.04:
            return None
        hit_x = pose.x_m + (front_range_m + 0.12) * cos(pose.yaw_rad)
        hit_y = pose.y_m + (front_range_m + 0.12) * sin(pose.yaw_rad)
        # Ignore perimeter bund walls outside the active field interior
        if not (-0.05 <= hit_x <= 2.75 and -0.78 <= hit_y <= 0.78):
            return None
        # Ignore hits that are simply inside known protected crop stands
        for site in known_sites.values():
            if not site.permitted and hypot(hit_x - site.center.x_m, hit_y - site.center.y_m) <= site.no_go_radius_m + 0.12:
                return None
        # Ignore hits that are dynamic peer robots (UWB ranging filters moving peers out of static SLAM map)
        for pid, ppose in peer_poses.items():
            if pid != robot_id and hypot(hit_x - ppose.x_m, hit_y - ppose.y_m) <= 0.42:
                return None

        ix, iy = self.to_cell(hit_x, hit_y)
        cx, cy = self.cell_center(ix, iy)
        prev = self.cells.get((ix, iy))
        new_lo = min(5.0, (prev.log_odds if prev else 0.0) + 1.5)
        cell = SlamHazardCell(
            cell_ix=ix,
            cell_iy=iy,
            x_m=cx,
            y_m=cy,
            kind="obstacle",
            log_odds=new_lo,
            discovered_by=robot_id,
            discovered_at_s=time_s,
        )
        self.cells[(ix, iy)] = cell
        return cell if prev is None else None

    def integrate_mud_slip(
        self,
        robot_id: str,
        pose: Pose2D,
        slip_ratio: float,
        time_s: float,
    ) -> SlamHazardCell | None:
        """Record high wheel-slip Lannitian mud trap discovered by 60GHz mmWave radar."""
        if slip_ratio <= 0.60:
            return None
        ix, iy = self.to_cell(pose.x_m, pose.y_m)
        cx, cy = self.cell_center(ix, iy)
        prev = self.cells.get((ix, iy))
        if prev is not None and prev.kind == "mud_slip":
            return None
        cell = SlamHazardCell(
            cell_ix=ix,
            cell_iy=iy,
            x_m=cx,
            y_m=cy,
            kind="mud_slip",
            log_odds=2.5,
            discovered_by=robot_id,
            discovered_at_s=time_s,
        )
        self.cells[(ix, iy)] = cell
        return cell

    def merge_peer_cell(self, cell: SlamHazardCell) -> bool:
        """Fuse a peer's SLAM hazard cell into this robot's local SLAM grid."""
        key = (cell.cell_ix, cell.cell_iy)
        existing = self.cells.get(key)
        if existing is None:
            self.cells[key] = cell
            return True
        if cell.log_odds > existing.log_odds:
            self.cells[key] = cell
            return True
        return False


class LocalMycelialMap:
    """Private belief map owned by a single robot (NO shared global memory with other robots)."""

    def __init__(
        self,
        owner_robot_id: str,
        sites: tuple[Site, ...],
        probe_radius_m: float = 0.60,
    ) -> None:
        self.owner_robot_id = owner_robot_id
        self.sites = {s.site_id: s for s in sites}
        self.probe_radius_m = probe_radius_m
        self.slam = LocalSlamGrid(resolution_m=0.15)
        self.seen_sample_ids: set[str] = set()
        self.all_samples: dict[str, SoilSample] = {}
        self._beliefs: dict[str, dict[SensorKind, GaussianChannelBelief]] = {
            s.site_id: {
                SensorKind.MOISTURE: GaussianChannelBelief(),
                SensorKind.FIRMNESS: GaussianChannelBelief(),
                SensorKind.STAND_GAP: GaussianChannelBelief(),
            }
            for s in sites
        }
        self._sources: dict[str, set[str]] = {s.site_id: set() for s in sites}
        self._last_updated_s: dict[str, float] = {s.site_id: 0.0 for s in sites}
        self.planted_sites: set[str] = set()
        self.attempted_sites: set[str] = set()

    def ingest_sample(self, sample: SoilSample, obs_precision: float = 4.0) -> bool:
        """Fuse a direct or peer-gossiped `SoilSample` if its `sample_id` has not been seen yet."""
        if not sample.valid or sample.sample_id in self.seen_sample_ids:
            return False
        site = self.sites.get(sample.site_id)
        if site is None or not site.permitted:
            return False
        if hypot(sample.location.x_m - site.center.x_m, sample.location.y_m - site.center.y_m) > self.probe_radius_m:
            return False

        self.seen_sample_ids.add(sample.sample_id)
        self.all_samples[sample.sample_id] = sample
        self._beliefs[sample.site_id][sample.kind].fuse(sample.value, obs_precision=obs_precision)
        self._sources[sample.site_id].add(sample.robot_id)
        self._last_updated_s[sample.site_id] = max(
            self._last_updated_s[sample.site_id], sample.sampled_at_s
        )
        return True

    def estimate(self, site_id: str) -> SiteEstimate:
        """Return this robot's local `SiteEstimate` for `site_id`."""
        ch = self._beliefs[site_id]
        total_count = sum(b.count for b in ch.values())
        return SiteEstimate(
            site_id=site_id,
            moisture=round(ch[SensorKind.MOISTURE].mean, 4) if ch[SensorKind.MOISTURE].count else None,
            firmness=round(ch[SensorKind.FIRMNESS].mean, 4) if ch[SensorKind.FIRMNESS].count else None,
            stand_gap=round(ch[SensorKind.STAND_GAP].mean, 4) if ch[SensorKind.STAND_GAP].count else None,
            sample_count=total_count,
            source_robot_ids=tuple(sorted(self._sources[site_id])),
        )

    def uncertainty(self, site_id: str) -> float:
        ch = self._beliefs[site_id]
        total_count = sum(b.count for b in ch.values())
        return round(1.0 / sqrt(total_count), 4) if total_count else 1.0

    def estimates(self) -> dict[str, SiteEstimate]:
        return {site_id: self.estimate(site_id) for site_id in self.sites}

    def needs_remeasurement(self, site_id: str, envelope: SoilSafetyEnvelope) -> bool:
        """Return True if a measured site has borderline soil confidence and only 1 robot has sampled it."""
        est = self.estimate(site_id)
        if est.sample_count == 0 or len(est.source_robot_ids) >= 2:
            return False
        if est.moisture is None or est.firmness is None:
            return True
        near_moisture_limit = abs(est.moisture - envelope.moisture_range[1]) <= 0.05
        near_firmness_limit = abs(est.firmness - envelope.firmness_range[0]) <= 0.05
        return near_moisture_limit or near_firmness_limit


class DecentralizedRobotNode:
    """Autonomous onboard intelligence for a single rover (local SLAM + belief fusion + task bidding)."""

    def __init__(
        self,
        capability: RobotCapability,
        sites: tuple[Site, ...],
        envelope: SoilSafetyEnvelope = SoilSafetyEnvelope(),
        limits: ControlLimits = ControlLimits(speed_m_s=0.38, turn_rad_s=1.15, stop_range_m=0.32, site_reach_m=0.22),
    ) -> None:
        self.capability = capability
        self.robot_id = capability.robot_id
        self.envelope = envelope
        self.limits = limits
        self.local_map = LocalMycelialMap(owner_robot_id=capability.robot_id, sites=sites)
        self.last_pose: Pose2D | None = None
        self.last_time_s: float | None = None
        self.last_commanded_linear_m_s: float = 0.0
        self.pending_plant_task: tuple[str, str, float] | None = None

    def process_local_sensors(
        self,
        obs: RobotObservation,
        peer_poses: dict[str, Pose2D],
    ) -> tuple[OnboardSensorTelemetry, int, list[SlamHazardCell]]:
        """Process raw onboard sensor readings into local SLAM & soil belief updates."""
        v_true = 0.0
        if self.last_pose is not None and self.last_time_s is not None and obs.time_s > self.last_time_s:
            dt = obs.time_s - self.last_time_s
            v_true = hypot(obs.pose.x_m - self.last_pose.x_m, obs.pose.y_m - self.last_pose.y_m) / dt

        v_wheel = abs(self.last_commanded_linear_m_s)
        raw_slip = 0.0
        if v_wheel > 0.05:
            raw_slip = max(0.0, min(1.0, 1.0 - (v_true / max(v_wheel, 1e-3))))

        ingested = 0
        for sample in obs.samples:
            if self.local_map.ingest_sample(sample, obs_precision=4.0):
                ingested += 1
            if sample.kind == SensorKind.MOISTURE and sample.value > 0.80:
                raw_slip = max(raw_slip, 0.72)

        new_hazards: list[SlamHazardCell] = []
        lidar_cell = self.local_map.slam.integrate_lidar_hit(
            self.robot_id, obs.pose, obs.front_range_m, self.local_map.sites, peer_poses, obs.time_s
        )
        if lidar_cell is not None:
            new_hazards.append(lidar_cell)

        mud_cell = self.local_map.slam.integrate_mud_slip(
            self.robot_id, obs.pose, raw_slip, obs.time_s
        )
        if mud_cell is not None:
            new_hazards.append(mud_cell)

        uwb_ranges: dict[str, float] = {}
        uwb_aoa: dict[str, float] = {}
        for pid, ppose in peer_poses.items():
            if pid == self.robot_id:
                continue
            dist = round(hypot(ppose.x_m - obs.pose.x_m, ppose.y_m - obs.pose.y_m), 4)
            bearing = round(
                _wrap_angle(atan2(ppose.y_m - obs.pose.y_m, ppose.x_m - obs.pose.x_m) - obs.pose.yaw_rad),
                4,
            )
            uwb_ranges[pid] = dist
            uwb_aoa[pid] = bearing

        min_crop_clear = 4.0
        for site in self.local_map.sites.values():
            if not site.permitted:
                d_crop = hypot(obs.pose.x_m - site.center.x_m, obs.pose.y_m - site.center.y_m) - site.no_go_radius_m
                min_crop_clear = min(min_crop_clear, max(0.0, d_crop))

        telemetry = OnboardSensorTelemetry(
            robot_id=self.robot_id,
            time_s=obs.time_s,
            pitch_deg=0.4 if raw_slip < 0.6 else -3.8,
            roll_deg=0.2 if raw_slip < 0.6 else 2.6,
            yaw_rad=obs.pose.yaw_rad,
            true_ground_speed_m_s=round(v_true, 4),
            wheel_surface_speed_m_s=round(v_wheel, 4),
            slip_ratio=round(raw_slip, 4),
            slip_critical=raw_slip > 0.60,
            front_tof_range_m=obs.front_range_m,
            lateral_crop_clearance_m=round(min_crop_clear, 4),
            uwb_peer_ranges_m=uwb_ranges,
            uwb_peer_aoa_rad=uwb_aoa,
        )
        self.last_pose = obs.pose
        self.last_time_s = obs.time_s
        return telemetry, ingested, new_hazards

    def compute_task_bids(self, obs: RobotObservation) -> list[TaskBidProposal]:
        """Compute capability-aware local auction bids using only THIS robot's local belief map."""
        bids: list[TaskBidProposal] = []
        speed_nom = max(0.22, self.limits.speed_m_s * 0.80)
        excluded = self.local_map.planted_sites | self.local_map.attempted_sites
        if self.pending_plant_task is not None:
            excluded = excluded | {self.pending_plant_task[0]}

        for site_id, site in self.local_map.sites.items():
            if not site.permitted or site_id in excluded:
                continue
            est = self.local_map.estimate(site_id)
            unc = self.local_map.uncertainty(site_id)
            dist = max(0.05, hypot(site.center.x_m - obs.pose.x_m, site.center.y_m - obs.pose.y_m))
            eta_s = round(dist / speed_nom, 2)
            conf = round(1.0 - unc, 2)

            # 1. Eligible for planting -> Planter bids high to place a seed
            if (
                self.capability.can_plant
                and obs.seeds_remaining > 0
                and self.envelope.eligible(est)
            ):
                gap_score = est.stand_gap or 0.5
                firm_score = est.firmness or 0.5
                bid_val = round(3.0 + gap_score + 0.5 * firm_score - 0.35 * dist, 4)
                bids.append(
                    TaskBidProposal(
                        robot_id=self.robot_id,
                        site_id=site_id,
                        mode=TaskMode.PLANT,
                        bid_score=bid_val,
                        eta_s=eta_s,
                        confidence=conf,
                        proposal_text=(
                            f"{site_id} stand gap confirmed (gap={gap_score:.2f}, firm={firm_score:.2f}); "
                            f"ETA {eta_s:.1f}s -> Proposal: take PLANT task"
                        ),
                    )
                )

            # 2. Unmeasured or borderline site -> Active Sensing / Thompson-like exploration bid
            if self.capability.can_sample and (
                not est.complete or self.local_map.needs_remeasurement(site_id, self.envelope)
            ):
                # Natural spatial sector partitioning so even during RF blackout robots don't collide on the same site:
                # Planter (carrying heavy seeds) prefers surveying forward along the main row (patch_b, patch_c)
                # while Scout screens the near mud trap (patch_a) and northern terrace (patch_d)
                sector_bonus = 0.0
                if self.capability.can_plant:
                    if site_id in ("patch_b", "patch_c"):
                        sector_bonus = 0.32
                else:
                    if site_id in ("patch_a", "patch_d"):
                        sector_bonus = 0.45

                uncertainty_bonus = 0.80 * unc
                bid_val = round(1.2 + sector_bonus + uncertainty_bonus - 0.45 * dist, 4)
                reason_tag = (
                    "remeasure borderline soil before planting"
                    if est.sample_count > 0
                    else "acquire initial soil & stand-gap evidence"
                )
                bids.append(
                    TaskBidProposal(
                        robot_id=self.robot_id,
                        site_id=site_id,
                        mode=TaskMode.SAMPLE,
                        bid_score=bid_val,
                        eta_s=eta_s,
                        confidence=conf,
                        proposal_text=f"{site_id} uncertainty={unc:.2f}, ETA {eta_s:.1f}s -> Proposal: {reason_tag}",
                    )
                )

        bids.sort(key=lambda b: b.bid_score, reverse=True)
        return bids

    def _select_waypoint_around_slam_obstacles(
        self,
        pose: Pose2D,
        target: Pose2D,
    ) -> Pose2D:
        """If a mapped SLAM obstacle or protected crop stand blocks the straight segment to `target`,
        select a safe intermediate bypass waypoint around it."""
        dx = target.x_m - pose.x_m
        dy = target.y_m - pose.y_m
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-4:
            return target

        # Check SLAM obstacle cells along segment
        for cell in self.local_map.slam.cells.values():
            if cell.kind != "obstacle":
                continue
            t = max(0.0, min(1.0, ((cell.x_m - pose.x_m) * dx + (cell.y_m - pose.y_m) * dy) / seg_len_sq))
            nx = pose.x_m + t * dx
            ny = pose.y_m + t * dy
            if hypot(cell.x_m - nx, cell.y_m - ny) <= 0.34 and pose.x_m < cell.x_m + 0.22:
                # Detour via the clear center-north corridor (y = +0.08m) or south corridor (y = -0.68m)
                bypass_y = 0.08 if pose.y_m >= -0.32 else -0.68
                return Pose2D(round(cell.x_m + 0.28, 3), bypass_y, 0.0)

        # Check protected crop stands along segment
        for site in self.local_map.sites.values():
            if site.permitted:
                continue
            t = max(
                0.0,
                min(
                    1.0,
                    ((site.center.x_m - pose.x_m) * dx + (site.center.y_m - pose.y_m) * dy) / seg_len_sq,
                ),
            )
            nx = pose.x_m + t * dx
            ny = pose.y_m + t * dy
            if hypot(site.center.x_m - nx, site.center.y_m - ny) <= site.no_go_radius_m + 0.04:
                return Pose2D(round(site.center.x_m + 0.34, 3), -0.05, 0.0)

        return target

    def compute_local_command(
        self,
        obs: RobotObservation,
        intent: TaskIntent,
        peer_poses: dict[str, Pose2D],
    ) -> RobotCommand:
        """Compute bounded wheel/drop command using local SLAM obstacle bypass & row-tracking steering."""
        issued_at_s = obs.time_s
        expires_at_s = round(obs.time_s + 0.2, 4)

        if self.pending_plant_task is not None:
            self.last_commanded_linear_m_s = 0.0
            return RobotCommand(
                robot_id=obs.robot_id,
                issued_at_s=issued_at_s,
                expires_at_s=expires_at_s,
                task_id=intent.task_id,
                linear_m_s=0.0,
                angular_rad_s=0.0,
            )

        if not obs.link_ok or obs.front_range_m is None or intent.mode is TaskMode.HOLD or intent.target_site_id is None:
            self.last_commanded_linear_m_s = 0.0
            return RobotCommand(
                robot_id=obs.robot_id,
                issued_at_s=issued_at_s,
                expires_at_s=expires_at_s,
                task_id=intent.task_id,
                linear_m_s=0.0,
                angular_rad_s=0.0,
            )

        target_site = self.local_map.sites[intent.target_site_id]
        dx = target_site.center.x_m - obs.pose.x_m
        dy = target_site.center.y_m - obs.pose.y_m
        dist_to_target = hypot(dx, dy)

        if dist_to_target <= self.limits.site_reach_m:
            release = (
                intent.mode is TaskMode.PLANT
                and self.capability.can_plant
                and obs.seeds_remaining > 0
                and target_site.permitted
            )
            if release:
                self.pending_plant_task = (target_site.site_id, intent.task_id, obs.time_s)
            self.last_commanded_linear_m_s = 0.0
            return RobotCommand(
                robot_id=obs.robot_id,
                issued_at_s=issued_at_s,
                expires_at_s=expires_at_s,
                task_id=intent.task_id,
                linear_m_s=0.0,
                angular_rad_s=0.0,
                release_seed=release,
                target_site_id=target_site.site_id if release else None,
            )

        # Potential-field steering combining attractive goal + local SLAM obstacle bypass + protected crop avoidance
        goal_ux = dx / dist_to_target
        goal_uy = dy / dist_to_target
        rep_x = 0.0
        rep_y = 0.0

        for site in self.local_map.sites.values():
            if not site.permitted:
                ox = obs.pose.x_m - site.center.x_m
                oy = obs.pose.y_m - site.center.y_m
                d_center = hypot(ox, oy)
                influence = site.no_go_radius_m + 0.34
                if 1e-4 < d_center < influence:
                    strength = 0.85 * ((1.0 / d_center) - (1.0 / influence)) / max(d_center, 0.12)
                    rep_x += (ox / d_center) * strength
                    rep_y += (oy / d_center) * strength

        for cell in self.local_map.slam.cells.values():
            if cell.kind != "obstacle":
                continue
            ox = obs.pose.x_m - cell.x_m
            oy = obs.pose.y_m - cell.y_m
            d_cell = hypot(ox, oy)
            influence = 0.56
            if 1e-4 < d_cell < influence:
                strength = 0.95 * ((1.0 / d_cell) - (1.0 / influence)) / max(d_cell, 0.10)
                # Tangential detour around northern side (+Y) of obstacle in southern lane
                rep_x += (ox / d_cell) * strength * 0.45 - (oy / d_cell) * strength * 1.15
                rep_y += (oy / d_cell) * strength * 0.45 + (ox / d_cell) * strength * 1.15

        for pid, ppose in peer_poses.items():
            if pid == obs.robot_id:
                continue
            px = obs.pose.x_m - ppose.x_m
            py = obs.pose.y_m - ppose.y_m
            d_peer = hypot(px, py)
            if 1e-4 < d_peer < 0.44:
                strength = 0.65 * ((1.0 / d_peer) - (1.0 / 0.44)) / max(d_peer, 0.12)
                rep_x += (px / d_peer) * strength
                rep_y += (py / d_peer) * strength

        steer_vx = goal_ux + rep_x
        steer_vy = goal_uy + rep_y
        desired_heading = atan2(steer_vy, steer_vx)
        heading_error = _wrap_angle(desired_heading - obs.pose.yaw_rad)

        # Close-in rangefinder safety reflex: if very close (< 0.20m), pivot in place;
        # if in bypass buffer (0.20m .. 0.32m) with SLAM obstacle mapped, pivot and creep along tangential detour
        if obs.front_range_m <= self.limits.stop_range_m:
            if not self.local_map.slam.cells:
                self.last_commanded_linear_m_s = 0.0
                return RobotCommand(
                    robot_id=obs.robot_id,
                    issued_at_s=issued_at_s,
                    expires_at_s=expires_at_s,
                    task_id=intent.task_id,
                    linear_m_s=0.0,
                    angular_rad_s=0.0,
                )
            pivot_dir = 1.0 if heading_error >= 0.0 else -1.0
            pivot_w = round(pivot_dir * min(self.limits.turn_rad_s, max(0.65, abs(heading_error) * 1.6)), 3)
            creep_v = 0.18 if (obs.front_range_m > 0.19 and abs(heading_error) < 0.65) else 0.0
            self.last_commanded_linear_m_s = creep_v
            return RobotCommand(
                robot_id=obs.robot_id,
                issued_at_s=issued_at_s,
                expires_at_s=expires_at_s,
                task_id=intent.task_id,
                linear_m_s=creep_v,
                angular_rad_s=pivot_w,
            )

        angular = max(
            -self.limits.turn_rad_s,
            min(self.limits.turn_rad_s, heading_error * 1.55),
        )
        linear = min(self.limits.speed_m_s, dist_to_target * 0.55) if abs(heading_error) < 0.55 else 0.0
        self.last_commanded_linear_m_s = round(linear, 3)
        return RobotCommand(
            robot_id=obs.robot_id,
            issued_at_s=issued_at_s,
            expires_at_s=expires_at_s,
            task_id=intent.task_id,
            linear_m_s=round(linear, 3),
            angular_rad_s=round(angular, 3),
        )


class DecentralizedMycelialRuntime:
    """Policy B runtime: no central map; each rover explores, gossips over UWB/HaLow, and bids on tasks."""

    def __init__(
        self,
        sites: tuple[Site, ...],
        capabilities: tuple[RobotCapability, ...],
        comm_range_m: float = 2.15,
        comm_mode: str = "normal",  # "normal" | "intermittent" | "isolated"
        envelope: SoilSafetyEnvelope = SoilSafetyEnvelope(),
        limits: ControlLimits = ControlLimits(speed_m_s=0.38, turn_rad_s=1.15, stop_range_m=0.32, site_reach_m=0.22),
    ) -> None:
        self.sites = {s.site_id: s for s in sites}
        self.capabilities = {c.robot_id: c for c in capabilities}
        self.comm_range_m = comm_range_m
        self.comm_mode = comm_mode
        self.envelope = envelope
        self.limits = limits
        self.nodes: dict[str, DecentralizedRobotNode] = {
            c.robot_id: DecentralizedRobotNode(c, sites=sites, envelope=envelope, limits=limits)
            for c in capabilities
        }
        self._last_time_s: float | None = None
        self._last_episode_id: str | None = None
        self._negotiated_conflicts: set[tuple[str, str, str]] = set()
        self.gossip_history: list[MycelialGossipPacket] = []
        self.latest_gossip_packets: tuple[MycelialGossipPacket, ...] = ()
        self.latest_sensor_telemetry: dict[str, OnboardSensorTelemetry] = {}
        self.total_gossip_bytes: int = 0
        self.total_gossip_packets: int = 0

    def _is_link_open(self, time_s: float, dist_m: float) -> bool:
        """Evaluate whether peer-to-peer Wi-Fi HaLow / UWB link can carry packets at `time_s`."""
        if dist_m > self.comm_range_m:
            return False
        if self.comm_mode == "isolated":
            return False
        if self.comm_mode == "intermittent":
            if (0.8 <= time_s <= 3.8) or (5.8 <= time_s <= 7.6):
                return False
        return True

    def decide(self, frame: ObservationFrame) -> Decision:
        if self._last_episode_id is None:
            self._last_episode_id = frame.episode_id
        if frame.episode_id != self._last_episode_id:
            raise ValueError("new episode needs a fresh DecentralizedMycelialRuntime")
        if self._last_time_s is not None and frame.time_s <= self._last_time_s:
            raise ValueError("non-increasing frame time")
        self._last_time_s = frame.time_s

        obs_by_robot = {r.robot_id: r for r in frame.robots}
        if any(rid not in self.capabilities for rid in obs_by_robot):
            raise ValueError("observation from unknown actor")
        peer_poses = {r.robot_id: r.pose for r in frame.robots}

        # 1. Process physical seed landing feedback locally on the robot that released it
        for fb in frame.plant_feedback:
            node = self.nodes.get(fb.robot_id)
            if node is not None and node.pending_plant_task is not None:
                site_id, task_id, _ = node.pending_plant_task
                if task_id == fb.task_id:
                    node.pending_plant_task = None
                    node.local_map.attempted_sites.add(site_id)
                    if fb.released and fb.landing is not None:
                        node.local_map.planted_sites.add(site_id)

        for node in self.nodes.values():
            if node.pending_plant_task is not None:
                site_id, _, sent_at_s = node.pending_plant_task
                if frame.time_s > sent_at_s + 2.0:
                    node.pending_plant_task = None
                    node.local_map.attempted_sites.add(site_id)

        # 2. Each robot independently processes its own physical sensor suite into its private local map
        total_ingested = 0
        self.latest_sensor_telemetry = {}
        for robot_id, obs in obs_by_robot.items():
            node = self.nodes[robot_id]
            telemetry, count, _ = node.process_local_sensors(obs, peer_poses)
            self.latest_sensor_telemetry[robot_id] = telemetry
            total_ingested += count

        # 3. Each robot independently computes its initial local task bids before peer negotiation
        local_bids_by_robot: dict[str, list[TaskBidProposal]] = {
            rid: self.nodes[rid].compute_task_bids(obs) for rid, obs in obs_by_robot.items()
        }

        # 4. Event-Triggered Peer-to-Peer Mycelial Gossip (ONLY between reachable peers within comm_range_m)
        cycle_packets: list[MycelialGossipPacket] = []
        robot_ids = sorted(obs_by_robot.keys())
        for sender_id in robot_ids:
            sender_node = self.nodes[sender_id]
            sender_obs = obs_by_robot[sender_id]
            sender_tel = self.latest_sensor_telemetry[sender_id]
            top_bid = local_bids_by_robot[sender_id][0] if local_bids_by_robot[sender_id] else None

            for receiver_id in robot_ids:
                if sender_id == receiver_id:
                    continue
                receiver_node = self.nodes[receiver_id]
                receiver_obs = obs_by_robot[receiver_id]
                if not (sender_obs.link_ok and receiver_obs.link_ok):
                    continue
                dist_m = sender_tel.uwb_peer_ranges_m.get(receiver_id, 99.0)
                aoa_rad = sender_tel.uwb_peer_aoa_rad.get(receiver_id, 0.0)
                if not self._is_link_open(frame.time_s, dist_m):
                    continue

                unseen_samples = tuple(
                    s
                    for sid, s in sender_node.local_map.all_samples.items()
                    if sid not in receiver_node.local_map.seen_sample_ids
                )
                unseen_hazards = tuple(
                    cell
                    for key, cell in sender_node.local_map.slam.cells.items()
                    if key not in receiver_node.local_map.slam.cells
                )
                unseen_planted = sender_node.local_map.planted_sites - receiver_node.local_map.planted_sites

                receiver_top_bid = (
                    local_bids_by_robot[receiver_id][0] if local_bids_by_robot[receiver_id] else None
                )
                conflict_key = (
                    sender_id,
                    receiver_id,
                    top_bid.site_id if top_bid else "",
                )
                new_bid_conflict = (
                    top_bid is not None
                    and receiver_top_bid is not None
                    and top_bid.site_id == receiver_top_bid.site_id
                    and conflict_key not in self._negotiated_conflicts
                )
                if new_bid_conflict:
                    self._negotiated_conflicts.add(conflict_key)

                if not unseen_samples and not unseen_hazards and not unseen_planted and not new_bid_conflict:
                    continue

                dialogue: list[str] = []
                for s in unseen_samples:
                    if s.kind == SensorKind.MOISTURE:
                        est_s = sender_node.local_map.estimate(s.site_id)
                        verdict = (
                            "VIABLE_GAP -> ready for planting"
                            if self.envelope.eligible(est_s)
                            else "MUD_HAZARD -> reject planting"
                        )
                        dialogue.append(
                            f"{sender_id}->{receiver_id}: {s.site_id} moisture={est_s.moisture}, "
                            f"firmness={est_s.firmness}, gap={est_s.stand_gap} ({verdict})"
                        )
                    receiver_node.local_map.ingest_sample(s, obs_precision=3.5)

                for cell in unseen_hazards:
                    if receiver_node.local_map.slam.merge_peer_cell(cell):
                        dialogue.append(
                            f"{sender_id}->{receiver_id}: SLAM {cell.kind} mapped at ({cell.x_m:.2f}, {cell.y_m:.2f})"
                        )

                for planted_site in unseen_planted:
                    receiver_node.local_map.planted_sites.add(planted_site)
                    receiver_node.local_map.attempted_sites.add(planted_site)
                    dialogue.append(f"{sender_id}->{receiver_id}: {planted_site} confirmed PLANTED")

                if top_bid is not None and (new_bid_conflict or unseen_samples):
                    dialogue.append(f"{sender_id}->{receiver_id}: {top_bid.proposal_text} (bid={top_bid.bid_score:.2f})")

                byte_size = 42 + 28 * len(unseen_samples) + 20 * len(unseen_hazards) + (32 if top_bid else 0)
                pkt = MycelialGossipPacket(
                    packet_id=f"pkt:{frame.time_s:.2f}:{sender_id}->{receiver_id}",
                    sender_id=sender_id,
                    receiver_id=receiver_id,
                    sent_at_s=frame.time_s,
                    uwb_distance_m=dist_m,
                    uwb_aoa_rad=aoa_rad,
                    samples=unseen_samples,
                    slam_hazards=unseen_hazards,
                    task_bid=top_bid,
                    byte_size=byte_size,
                    dialogue_summary=tuple(dialogue),
                )
                cycle_packets.append(pkt)
                self.gossip_history.append(pkt)
                self.total_gossip_bytes += byte_size
                self.total_gossip_packets += 1

        self.latest_gossip_packets = tuple(cycle_packets)

        # 5. Recompute bids after gossip education & resolve peer task ownership via Consensus Bundle Auction
        updated_bids: dict[str, list[TaskBidProposal]] = {
            rid: self.nodes[rid].compute_task_bids(obs) for rid, obs in obs_by_robot.items()
        }

        reachable_peers: dict[str, set[str]] = {rid: {rid} for rid in robot_ids}
        for rid_a in robot_ids:
            for rid_b in robot_ids:
                if rid_a == rid_b:
                    continue
                d_ab = self.latest_sensor_telemetry[rid_a].uwb_peer_ranges_m.get(rid_b, 99.0)
                if (
                    obs_by_robot[rid_a].link_ok
                    and obs_by_robot[rid_b].link_ok
                    and self._is_link_open(frame.time_s, d_ab)
                ):
                    reachable_peers[rid_a].add(rid_b)

        intents: list[TaskIntent] = []
        commands: list[RobotCommand] = []
        claimed_by_peer: dict[str, str] = {}

        all_connected_bids: list[TaskBidProposal] = []
        for rid in robot_ids:
            all_connected_bids.extend(updated_bids[rid])
        all_connected_bids.sort(key=lambda b: b.bid_score, reverse=True)

        assigned_bid_for_robot: dict[str, TaskBidProposal] = {}
        for bid in all_connected_bids:
            if bid.robot_id in assigned_bid_for_robot:
                continue
            existing_owner = claimed_by_peer.get(bid.site_id)
            if existing_owner is not None and existing_owner in reachable_peers[bid.robot_id]:
                continue
            assigned_bid_for_robot[bid.robot_id] = bid
            claimed_by_peer[bid.site_id] = bid.robot_id

        for rid in robot_ids:
            node = self.nodes[rid]
            obs = obs_by_robot[rid]
            winning_bid = assigned_bid_for_robot.get(rid)
            if winning_bid is None:
                intent = TaskIntent(
                    task_id=f"{frame.episode_id}:{frame.time_s:.3f}:{rid}:hold",
                    robot_id=rid,
                    mode=TaskMode.HOLD,
                    issued_at_s=frame.time_s,
                    expires_at_s=frame.time_s + 1.0,
                    target_site_id=None,
                    reason="local map converged; no unclaimed tasks",
                )
            else:
                intent = TaskIntent(
                    task_id=f"{frame.episode_id}:{frame.time_s:.3f}:{rid}:{winning_bid.mode.value}",
                    robot_id=rid,
                    mode=winning_bid.mode,
                    issued_at_s=frame.time_s,
                    expires_at_s=frame.time_s + 1.0,
                    target_site_id=winning_bid.site_id,
                    reason=winning_bid.proposal_text,
                )
            cmd = node.compute_local_command(obs, intent, peer_poses)
            intents.append(intent)
            commands.append(cmd)

        return Decision(
            intents=tuple(intents),
            commands=tuple(commands),
            ingested_samples=total_ingested,
        )
