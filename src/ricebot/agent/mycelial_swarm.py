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
from math import atan2, cos, hypot, log, pi, sin, sqrt
from typing import Any

try:
    import numpy as np
except ImportError:
    np = None

from .orchestrator import (
    HelperStatus,
    IncidentReport,
    OperatorPolicy,
    OrchestratorDecision,
    SwarmOrchestratorAgent,
)
from .rescue import DistressSignal, FailureMode, MechanicalRescueTechnique, RescueEngine
from ..contracts import (
    Decision,
    FaultEvidence,
    ObservationFrame,
    Pose2D,
    RescueAssessment,
    RescueVerdict,
    RobotCapability,
    RobotCommand,
    RobotObservation,
    SensorKind,
    Site,
    SoilSample,
    TaskIntent,
    TaskMode,
)
from ..control import ControlLimits, clamp_recovery_actuators
from ..mission import SiteEstimate
from .planner import SupervisoryAdvisor


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
    distress_signal: DistressSignal | None = None


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
        self.version: int = 0

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
        self.version += 1
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
        self.version += 1
        return cell

    def merge_peer_cell(self, cell: SlamHazardCell) -> bool:
        """Fuse a peer's SLAM hazard cell into this robot's local SLAM grid."""
        key = (cell.cell_ix, cell.cell_iy)
        existing = self.cells.get(key)
        if existing is None:
            self.cells[key] = cell
            self.version += 1
            return True
        if cell.log_odds > existing.log_odds:
            self.cells[key] = cell
            self.version += 1
            return True
        return False


class ContinuousBeliefGrid:
    """Per-robot 2D continuous spatial belief grid updated by Tier-1 camera FOV cone & Tier-2 penetrometer probes."""

    def __init__(
        self,
        nx: int = 36,
        ny: int = 24,
        x_bounds: tuple[float, float] = (-0.25, 2.95),
        y_bounds: tuple[float, float] = (-0.90, 0.90),
    ) -> None:
        self.nx = int(nx)
        self.ny = int(ny)
        self.x_bounds = (float(x_bounds[0]), float(x_bounds[1]))
        self.y_bounds = (float(y_bounds[0]), float(y_bounds[1]))
        self.xs = np.linspace(self.x_bounds[0], self.x_bounds[1], self.nx, dtype=np.float64)
        self.ys = np.linspace(self.y_bounds[0], self.y_bounds[1], self.ny, dtype=np.float64)
        self.X, self.Y = np.meshgrid(self.xs, self.ys)
        self.version: int = 0
        self.prior_precision: float = 0.0
        self.precision: np.ndarray = np.zeros((self.ny, self.nx), dtype=np.float64)
        self.uncertainty: np.ndarray = np.ones((self.ny, self.nx), dtype=np.float64)
        self.moisture: np.ndarray = np.full((self.ny, self.nx), 0.50, dtype=np.float64)
        self.firmness: np.ndarray = np.full((self.ny, self.nx), 0.50, dtype=np.float64)
        self.stand_gap: np.ndarray = np.full((self.ny, self.nx), 0.50, dtype=np.float64)
        self.plant_density: np.ndarray = np.full((self.ny, self.nx), 0.50, dtype=np.float64)
        self.scan_count: np.ndarray = np.zeros((self.ny, self.nx), dtype=np.int32)

    def reset(self) -> None:
        """Reset all continuous 2D belief layers to unvisited uniform priors."""
        self.precision.fill(0.0)
        self.uncertainty.fill(1.0)
        self.moisture.fill(0.50)
        self.firmness.fill(0.50)
        self.stand_gap.fill(0.50)
        self.plant_density.fill(0.50)
        self.scan_count.fill(0)
        self.version += 1

    def copy(self) -> ContinuousBeliefGrid:
        """Return a deep snapshot of this grid for delayed gossip transmission."""
        clone = ContinuousBeliefGrid(
            nx=self.nx,
            ny=self.ny,
            x_bounds=self.x_bounds,
            y_bounds=self.y_bounds,
        )
        clone.precision = self.precision.copy()
        clone.uncertainty = self.uncertainty.copy()
        clone.moisture = self.moisture.copy()
        clone.firmness = self.firmness.copy()
        clone.stand_gap = self.stand_gap.copy()
        clone.plant_density = self.plant_density.copy()
        clone.scan_count = self.scan_count.copy()
        clone.version = self.version
        return clone

    def world_to_index(self, x_m: float, y_m: float) -> tuple[int, int]:
        """Convert world `(x_m, y_m)` to nearest `(ix, iy)` grid indices."""
        tx = (x_m - self.x_bounds[0]) / max(1e-6, self.x_bounds[1] - self.x_bounds[0])
        ty = (y_m - self.y_bounds[0]) / max(1e-6, self.y_bounds[1] - self.y_bounds[0])
        ix = max(0, min(self.nx - 1, int(round(tx * (self.nx - 1)))))
        iy = max(0, min(self.ny - 1, int(round(ty * (self.ny - 1)))))
        return ix, iy

    def uncertainty_at(self, x_m: float, y_m: float) -> float:
        ix, iy = self.world_to_index(x_m, y_m)
        return float(self.uncertainty[iy, ix])

    def precision_at(self, x_m: float, y_m: float) -> float:
        ix, iy = self.world_to_index(x_m, y_m)
        return float(self.precision[iy, ix])

    def evaluate_at(self, x_m: float, y_m: float) -> dict[str, float]:
        ix, iy = self.world_to_index(x_m, y_m)
        return {
            "moisture": round(float(self.moisture[iy, ix]), 4),
            "firmness": round(float(self.firmness[iy, ix]), 4),
            "stand_gap": round(float(self.stand_gap[iy, ix]), 4),
            "plant_density": round(float(self.plant_density[iy, ix]), 4),
            "uncertainty": round(float(self.uncertainty[iy, ix]), 4),
            "precision": round(float(self.precision[iy, ix]), 4),
        }

    def integrate_camera_fov(
        self,
        pose: Pose2D,
        camera_fov_rad: float = 1.08,
        camera_range_m: float = 0.95,
        sites: dict[str, Site] | None = None,
        front_range_m: float | None = None,
    ) -> int:
        """Project forward camera frustum rays across the 2D grid and reduce uncertainty inside the FOV cone."""
        dx = self.X - pose.x_m
        dy = self.Y - pose.y_m
        dist = np.hypot(dx, dy)
        bearing = (np.arctan2(dy, dx) - pose.yaw_rad + pi) % (2.0 * pi) - pi
        half_fov = 0.5 * camera_fov_rad

        if front_range_m is not None and front_range_m < camera_range_m:
            central_mask = np.abs(bearing) <= 0.22
            cone_mask = (
                (dist <= camera_range_m)
                & (np.abs(bearing) <= half_fov)
                & (~central_mask | (dist <= front_range_m + 0.12))
            )
        else:
            cone_mask = (dist <= camera_range_m) & (np.abs(bearing) <= half_fov)

        footprint_mask = dist <= 0.18
        active_mask = cone_mask | footprint_mask
        updated_count = int(np.count_nonzero(active_mask))
        if updated_count == 0:
            return 0

        angular_weight = np.where(
            footprint_mask,
            1.0,
            np.clip(np.cos(bearing / max(1e-4, half_fov) * (0.42 * pi)), 0.25, 1.0),
        )
        tau_vis = (2.0 / (1.0 + dist * dist)) * angular_weight

        # Tier-1 visual camera scan caps visual precision at 3.2; Tier-2 physical probing reaches >= 10.0
        self.precision = np.where(
            active_mask,
            np.maximum(self.precision, np.minimum(3.2, self.precision + 0.65 * tau_vis)),
            self.precision,
        )
        self.scan_count = np.where(active_mask, self.scan_count + 1, self.scan_count)

        if sites:
            for site in sites.values():
                s_dist2 = (self.X - site.center.x_m) ** 2 + (self.Y - site.center.y_m) ** 2
                site_mask = active_mask & (s_dist2 <= 0.40 * 0.40)
                if np.any(site_mask):
                    k_site = np.exp(-s_dist2 / (2.0 * 0.28 * 0.28))
                    target_gap = 0.05 if not site.permitted else 0.76
                    alpha = np.clip(0.32 * k_site, 0.0, 0.45)
                    self.stand_gap = np.where(
                        site_mask,
                        (1.0 - alpha) * self.stand_gap + alpha * target_gap,
                        self.stand_gap,
                    )

        self.plant_density = np.clip(1.0 - self.stand_gap, 0.02, 0.98)
        self.uncertainty = np.clip(1.0 / np.sqrt(1.0 + 0.85 * self.precision), 0.04, 1.0)
        self.version += 1
        return updated_count

    def integrate_sample(
        self,
        sample: SoilSample,
        obs_precision: float = 10.0,
        length_scale_m: float = 0.44,
    ) -> None:
        """Fuse a Tier-2 physical penetrometer `SoilSample` into the continuous 2D belief grid."""
        r2 = (self.X - sample.location.x_m) ** 2 + (self.Y - sample.location.y_m) ** 2
        k = np.exp(-r2 / (2.0 * length_scale_m * length_scale_m))
        alpha = np.clip((obs_precision / 12.0) * k, 0.0, 0.88)
        if sample.kind == SensorKind.MOISTURE:
            self.moisture = np.clip((1.0 - alpha) * self.moisture + alpha * sample.value, 0.02, 0.98)
            self.precision = self.precision + 0.45 * obs_precision * k
        elif sample.kind == SensorKind.FIRMNESS:
            self.firmness = np.clip((1.0 - alpha) * self.firmness + alpha * sample.value, 0.02, 0.98)
            self.precision = self.precision + 0.45 * obs_precision * k
        elif sample.kind == SensorKind.STAND_GAP:
            self.stand_gap = np.clip((1.0 - alpha) * self.stand_gap + alpha * sample.value, 0.02, 0.98)
            self.plant_density = np.clip(1.0 - self.stand_gap, 0.02, 0.98)
            self.precision = self.precision + 0.35 * obs_precision * k

        self.uncertainty = np.clip(1.0 / np.sqrt(1.0 + 0.85 * self.precision), 0.04, 1.0)
        self.version += 1

    def merge_peer_grid(self, peer_grid: ContinuousBeliefGrid, blend_weight: float = 0.45) -> bool:
        """Blend a peer's continuous belief grid when a delayed gossip packet arrives."""
        if peer_grid.precision.shape != self.precision.shape:
            return False
        gain_mask = peer_grid.precision > (self.precision + 0.15)
        if not np.any(gain_mask):
            return False
        w = float(max(0.05, min(0.85, blend_weight)))
        self.moisture = np.where(gain_mask, (1.0 - w) * self.moisture + w * peer_grid.moisture, self.moisture)
        self.firmness = np.where(gain_mask, (1.0 - w) * self.firmness + w * peer_grid.firmness, self.firmness)
        self.stand_gap = np.where(gain_mask, (1.0 - w) * self.stand_gap + w * peer_grid.stand_gap, self.stand_gap)
        self.plant_density = np.clip(1.0 - self.stand_gap, 0.02, 0.98)
        self.precision = np.where(
            gain_mask,
            np.maximum(self.precision, (1.0 - w) * self.precision + w * peer_grid.precision),
            self.precision,
        )
        self.uncertainty = np.clip(1.0 / np.sqrt(1.0 + 0.85 * self.precision), 0.04, 1.0)
        self.version += 1
        return True

    def __getitem__(self, key: str) -> np.ndarray:
        return getattr(self, key)

    def to_dict(self) -> dict[str, list[list[float]]]:
        return {
            "moisture": np.round(self.moisture, 4).tolist(),
            "firmness": np.round(self.firmness, 4).tolist(),
            "stand_gap": np.round(self.stand_gap, 4).tolist(),
            "plant_density": np.round(self.plant_density, 4).tolist(),
            "uncertainty": np.round(self.uncertainty, 4).tolist(),
            "precision": np.round(self.precision, 4).tolist(),
        }


class LocalMycelialMap:
    """Private belief map owned by a single robot (NO shared global memory with other robots)."""

    def __init__(
        self,
        owner_robot_id: str,
        sites: tuple[Site, ...],
        probe_radius_m: float = 0.28,
        camera_fov_rad: float = 1.08,
        camera_range_m: float = 0.95,
    ) -> None:
        self.owner_robot_id = owner_robot_id
        self.sites = {s.site_id: s for s in sites}
        self.probe_radius_m = probe_radius_m
        self.camera_fov_rad = camera_fov_rad
        self.camera_range_m = camera_range_m
        self.slam = LocalSlamGrid(resolution_m=0.15)
        self.continuous_grid = ContinuousBeliefGrid()
        self.seen_sample_ids: set[str] = set()
        self.all_samples: dict[str, SoilSample] = {}
        self.discovered_sites: set[str] = set()
        self.probed_sites: set[str] = set()
        self.visual_priors: dict[str, dict[SensorKind, GaussianChannelBelief]] = {
            s.site_id: {
                SensorKind.MOISTURE: GaussianChannelBelief(),
                SensorKind.FIRMNESS: GaussianChannelBelief(),
                SensorKind.STAND_GAP: GaussianChannelBelief(),
            }
            for s in sites
        }
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
        self._sample_version: int = 0
        self._grid_cache: dict[tuple[Any, ...], dict[str, Any]] = {}

    def reset(self) -> None:
        """Reset all per-site beliefs, camera discovery sets, SLAM cells, and continuous belief grid."""
        self.slam = LocalSlamGrid(resolution_m=0.15)
        self.continuous_grid.reset()
        self.seen_sample_ids.clear()
        self.all_samples.clear()
        self.discovered_sites.clear()
        self.probed_sites.clear()
        for sid in self.sites:
            self.visual_priors[sid] = {
                SensorKind.MOISTURE: GaussianChannelBelief(),
                SensorKind.FIRMNESS: GaussianChannelBelief(),
                SensorKind.STAND_GAP: GaussianChannelBelief(),
            }
            self._beliefs[sid] = {
                SensorKind.MOISTURE: GaussianChannelBelief(),
                SensorKind.FIRMNESS: GaussianChannelBelief(),
                SensorKind.STAND_GAP: GaussianChannelBelief(),
            }
            self._sources[sid].clear()
            self._last_updated_s[sid] = 0.0
        self.planted_sites.clear()
        self.attempted_sites.clear()
        self._sample_version += 1
        self._grid_cache.clear()

    @property
    def state_version(self) -> tuple[int, int, int]:
        """Monotonic version tuple `(sample_version, slam_version, continuous_grid_version)` for belief grid caching."""
        return (self._sample_version, self.slam.version, self.continuous_grid.version)

    def scan_camera_fov(
        self,
        pose: Pose2D,
        time_s: float,
        camera_fov_rad: float | None = None,
        camera_range_m: float | None = None,
        front_range_m: float | None = None,
    ) -> set[str]:
        """Tier-1 continuous front-facing camera / Edge CV cone scan (`camera_fov_rad=1.08`, `camera_range_m=0.95`).

        Marks anchored candidate sites inside the forward camera cone as `visually_discovered` in
        `self.discovered_sites`, records moderate-precision (`tau_vis = 2.0 / (1.0 + dist^2)`) visual
        surface priors for surface moisture sheen and canopy stand-gap, and updates `self.continuous_grid`
        along the camera frustum rays.
        """
        fov = self.camera_fov_rad if camera_fov_rad is None else float(camera_fov_rad)
        rng = self.camera_range_m if camera_range_m is None else float(camera_range_m)
        half_fov = 0.5 * fov
        newly_discovered: set[str] = set()

        for site_id, site in self.sites.items():
            dx = site.center.x_m - pose.x_m
            dy = site.center.y_m - pose.y_m
            dist = hypot(dx, dy)
            if dist > rng:
                continue
            bearing_err = _wrap_angle(atan2(dy, dx) - pose.yaw_rad)
            if abs(bearing_err) > half_fov:
                continue
            if front_range_m is not None and front_range_m < dist - 0.14 and abs(bearing_err) <= 0.22:
                continue

            if site_id not in self.discovered_sites:
                newly_discovered.add(site_id)
            self.discovered_sites.add(site_id)

            tau_vis = round(2.0 / (1.0 + dist * dist), 4)
            vis_ch = self.visual_priors[site_id]
            # Cap accumulated Tier-1 visual precision at 1.5 so total precision tau_0 + tau_vis <= 2.5
            rem_g = max(0.0, 1.5 - vis_ch[SensorKind.STAND_GAP].precision)
            if rem_g > 1e-4:
                gap_prior = (
                    0.05
                    if not site.permitted
                    else (
                        self._beliefs[site_id][SensorKind.STAND_GAP].mean
                        if self._beliefs[site_id][SensorKind.STAND_GAP].count > 0
                        else 0.78
                    )
                )
                vis_ch[SensorKind.STAND_GAP].fuse(gap_prior, obs_precision=min(tau_vis, rem_g))
            rem_m = max(0.0, 1.5 - vis_ch[SensorKind.MOISTURE].precision)
            if rem_m > 1e-4:
                moist_prior = (
                    0.58
                    if not site.permitted
                    else (
                        self._beliefs[site_id][SensorKind.MOISTURE].mean
                        if self._beliefs[site_id][SensorKind.MOISTURE].count > 0
                        else 0.56
                    )
                )
                vis_ch[SensorKind.MOISTURE].fuse(moist_prior, obs_precision=min(0.85 * tau_vis, rem_m))
            self._last_updated_s[site_id] = max(self._last_updated_s[site_id], time_s)

        self.continuous_grid.integrate_camera_fov(
            pose=pose,
            camera_fov_rad=fov,
            camera_range_m=rng,
            sites=self.sites,
            front_range_m=front_range_m,
        )
        return newly_discovered

    def ingest_sample(self, sample: SoilSample, obs_precision: float = 10.0) -> bool:
        """Fuse a Tier-2 physical penetrometer or peer-gossiped `SoilSample` if not seen yet."""
        if not sample.valid or sample.sample_id in self.seen_sample_ids:
            return False
        site = self.sites.get(sample.site_id)
        if site is None or not site.permitted:
            return False
        if hypot(sample.location.x_m - site.center.x_m, sample.location.y_m - site.center.y_m) > max(self.probe_radius_m + 1e-4, 0.60):
            return False

        self.seen_sample_ids.add(sample.sample_id)
        self.all_samples[sample.sample_id] = sample
        self.discovered_sites.add(sample.site_id)
        self.probed_sites.add(sample.site_id)
        self._beliefs[sample.site_id][sample.kind].fuse(sample.value, obs_precision=obs_precision)
        self._sources[sample.site_id].add(sample.robot_id)
        self._last_updated_s[sample.site_id] = max(
            self._last_updated_s[sample.site_id], sample.sampled_at_s
        )
        self.continuous_grid.integrate_sample(sample, obs_precision=obs_precision)
        self._sample_version += 1
        return True

    def channel_precision(self, site_id: str, kind: SensorKind) -> float:
        """Return total posterior precision `tau_{i,s,k}` combining prior (`1.0`), Tier-1 vision, and Tier-2 probe."""
        vis_p = self.visual_priors[site_id][kind].precision if site_id in self.visual_priors else 0.0
        probe_p = self._beliefs[site_id][kind].precision if site_id in self._beliefs else 0.0
        return round(1.0 + vis_p + probe_p, 4)

    def expected_information_gain(
        self,
        site_id: str,
        tau_sensor: float = 10.0,
        w_spatial: float = 0.25,
    ) -> float:
        """Compute Expected Information Gain `EIG(s)` for physical stop-and-probe at `site_id`."""
        site = self.sites.get(site_id)
        if site is None:
            return 0.0
        shannon_sum = 0.0
        for kind in (SensorKind.MOISTURE, SensorKind.FIRMNESS, SensorKind.STAND_GAP):
            tau_k = max(1.0, self.channel_precision(site_id, kind))
            shannon_sum += 0.5 * log(1.0 + (tau_sensor / tau_k))
        sigma_neigh = self.continuous_grid.uncertainty_at(site.center.x_m, site.center.y_m)
        eig = shannon_sum + w_spatial * (sigma_neigh * sigma_neigh)
        return round(eig, 4)

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
        if total_count > 0:
            total_prec = sum(b.precision for b in ch.values())
            return round(1.0 / (1.0 + 0.45 * total_prec), 4)
        if site_id in self.discovered_sites and site_id in self.visual_priors:
            vis_prec = 0.5 * sum(b.precision for b in self.visual_priors[site_id].values())
            if vis_prec > 0.0:
                return round(1.0 / sqrt(1.0 + vis_prec), 4)
        return 1.0

    def nearest_site_id(self, pose: Pose2D) -> str | None:
        if not self.sites:
            return None
        return min(self.sites.keys(), key=lambda sid: hypot(self.sites[sid].center.x_m - pose.x_m, self.sites[sid].center.y_m - pose.y_m))

    def estimates(self) -> dict[str, SiteEstimate]:
        return {site_id: self.estimate(site_id) for site_id in self.sites}

    def needs_remeasurement(self, site_id: str, envelope: SoilSafetyEnvelope) -> bool:
        """Return True if a measured site has borderline soil confidence and only 1 OTHER robot has sampled it."""
        est = self.estimate(site_id)
        if est.sample_count == 0 or len(est.source_robot_ids) >= 2:
            return False
        if self.owner_robot_id in est.source_robot_ids:
            return False
        if est.moisture is None or est.firmness is None:
            return True
        near_moisture_limit = abs(est.moisture - envelope.moisture_range[1]) <= 0.05
        near_firmness_limit = abs(est.firmness - envelope.firmness_range[0]) <= 0.05
        return near_moisture_limit or near_firmness_limit


# Alias requested by multi-modal sensing contract
LocalBayesianFieldMap = LocalMycelialMap


class DecentralizedRobotNode:
    """Autonomous onboard intelligence for a single rover (local SLAM + belief fusion + task bidding)."""

    def __init__(
        self,
        capability: RobotCapability,
        sites: tuple[Site, ...],
        envelope: SoilSafetyEnvelope = SoilSafetyEnvelope(),
        limits: ControlLimits = ControlLimits(speed_m_s=0.38, turn_rad_s=1.15, stop_range_m=0.32, site_reach_m=0.22),
        advisor: SupervisoryAdvisor | None = None,
        camera_fov_rad: float = 1.08,
        camera_range_m: float = 0.95,
        probe_radius_m: float = 0.28,
    ) -> None:
        self.capability = capability
        self.robot_id = capability.robot_id
        self.envelope = envelope
        self.limits = limits
        self.camera_fov_rad = camera_fov_rad
        self.camera_range_m = camera_range_m
        self.probe_radius_m = probe_radius_m
        self.local_map = LocalMycelialMap(
            owner_robot_id=capability.robot_id,
            sites=sites,
            probe_radius_m=probe_radius_m,
            camera_fov_rad=camera_fov_rad,
            camera_range_m=camera_range_m,
        )
        self.last_pose: Pose2D | None = None
        self.last_time_s: float | None = None
        self.last_commanded_linear_m_s: float = 0.0
        self.pending_plant_task: tuple[str, str, float] | None = None
        self.active_distress: DistressSignal | None = None
        self.peer_distress_signals: dict[str, DistressSignal] = {}
        self.rescue_engine: RescueEngine = RescueEngine()

    def reset(self) -> None:
        """Reset node-local belief map and internal state for a fresh episode."""
        self.local_map.reset()
        self.last_pose = None
        self.last_time_s = None
        self.last_commanded_linear_m_s = 0.0
        self.pending_plant_task = None
        self.active_distress = None
        self.peer_distress_signals.clear()
        self.rescue_engine = RescueEngine()

    def process_local_sensors(
        self,
        obs: RobotObservation,
        peer_poses: dict[str, Pose2D],
    ) -> tuple[OnboardSensorTelemetry, int, list[SlamHazardCell]]:
        """Process raw onboard sensor readings into local SLAM & two-tier soil belief updates."""
        v_true = 0.0
        if self.last_pose is not None and self.last_time_s is not None and obs.time_s > self.last_time_s:
            dt = obs.time_s - self.last_time_s
            v_true = hypot(obs.pose.x_m - self.last_pose.x_m, obs.pose.y_m - self.last_pose.y_m) / dt

        v_wheel = abs(self.last_commanded_linear_m_s)
        raw_slip = 0.0
        if v_wheel > 0.05:
            raw_slip = max(0.0, min(1.0, 1.0 - (v_true / max(v_wheel, 1e-3))))

        # Tier 1: Continuous front-facing camera / edge CV cone (~62 deg, 0.95m range)
        self.local_map.scan_camera_fov(
            obs.pose,
            obs.time_s,
            camera_fov_rad=self.camera_fov_rad,
            camera_range_m=self.camera_range_m,
            front_range_m=obs.front_range_m,
        )

        # Tier 2: Physical stop-and-probe penetrometer / soil probe confirmation (tau_probe = 10.0)
        ingested = 0
        for sample in obs.samples:
            if self.local_map.ingest_sample(sample, obs_precision=10.0):
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

        nearest_sid = self.local_map.nearest_site_id(obs.pose) if hasattr(self.local_map, "nearest_site_id") else None
        local_est = self.local_map.estimate(nearest_sid) if nearest_sid else None
        effective_slip = raw_slip if raw_slip > 0.0 else (0.85 if obs.is_bogged else 0.08)
        self.active_distress = self.rescue_engine.build_distress_signal(
            obs,
            slip_ratio=effective_slip,
            local_moisture=local_est.moisture if local_est else None,
            local_firmness=local_est.firmness if local_est else None,
        )

        return telemetry, ingested, new_hazards

    def compute_task_bids(self, obs: RobotObservation) -> list[TaskBidProposal]:
        """Compute capability-aware local auction bids using Expected Information Gain (EIG) and local belief."""
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
            eig = self.local_map.expected_information_gain(site_id)
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

            # 2. Unmeasured or borderline site -> Expected Information Gain (EIG) Active Sensing bid
            if self.capability.can_sample and (
                not est.complete or self.local_map.needs_remeasurement(site_id, self.envelope)
            ):
                # Identity-neutral task bidding: spatial separation emerges from each
                # rover's own estimated travel cost, information gain and the
                # cross-fleet conflict auction below. Do not encode permanent
                # planter/scout zones in robot names.
                sector_bonus = 0.0

                # Normalize Shannon/Gaussian EIG (unvisited site EIG_0 ≈ 3.8468 -> eig_norm ≈ 1.0)
                eig_norm = min(1.15, eig / 3.8468)
                eig_bonus = 0.80 * eig_norm

                # High Value-of-Information (VoI) physical stop-and-probe bonus when a site in this
                # robot's sector or immediate approach has been visually discovered with a promising
                # stand-gap, ensuring the rover completes physical subsurface penetrometer probing.
                visual_voi_bonus = 0.0
                if (
                    site_id in self.local_map.discovered_sites
                    and site_id not in self.local_map.probed_sites
                    and (sector_bonus > 0.0 or dist <= 0.62)
                ):
                    vis_gap_ch = self.local_map.visual_priors.get(site_id, {}).get(SensorKind.STAND_GAP)
                    vis_gap = vis_gap_ch.mean if (vis_gap_ch is not None and vis_gap_ch.count > 0) else 0.72
                    if vis_gap >= self.envelope.min_gap:
                        visual_voi_bonus = max(0.0, 0.80 - eig_bonus)

                bid_val = round(1.2 + sector_bonus + eig_bonus + visual_voi_bonus - 0.45 * dist, 4)
                if est.sample_count > 0:
                    reason_tag = "remeasure borderline soil before planting"
                elif site_id in self.local_map.discovered_sites:
                    reason_tag = f"confirm visually discovered gap via stop-and-probe (EIG={eig:.2f})"
                else:
                    reason_tag = f"explore high-EIG candidate site (EIG={eig:.2f})"
                bids.append(
                    TaskBidProposal(
                        robot_id=self.robot_id,
                        site_id=site_id,
                        mode=TaskMode.SAMPLE,
                        bid_score=bid_val,
                        eta_s=eta_s,
                        confidence=conf,
                        proposal_text=f"{site_id} uncertainty={unc:.2f}, EIG={eig:.2f}, ETA {eta_s:.1f}s -> Proposal: {reason_tag}",
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
        if intent.mode is TaskMode.RESCUE and intent.target_robot_id is not None:
            peer_pose = peer_poses.get(intent.target_robot_id, target_site.center)
            dx_r = peer_pose.x_m - obs.pose.x_m
            dy_r = peer_pose.y_m - obs.pose.y_m
            dist_r = max(1e-4, hypot(dx_r, dy_r))
            heading_err_r = _wrap_angle(atan2(dy_r, dx_r) - obs.pose.yaw_rad)
            angular_r = max(-self.limits.turn_rad_s, min(self.limits.turn_rad_s, heading_err_r * 1.55))
            linear_r = min(self.limits.speed_m_s, dist_r * 0.65) if abs(heading_err_r) < 0.65 else 0.0
            self.last_commanded_linear_m_s = round(linear_r, 3)
            return RobotCommand(
                robot_id=obs.robot_id,
                issued_at_s=issued_at_s,
                expires_at_s=expires_at_s,
                task_id=intent.task_id,
                linear_m_s=round(linear_r, 3),
                angular_rad_s=round(angular_r, 3),
                target_site_id=target_site.site_id,
                recovery_target_id=intent.target_robot_id,
                dock_pitch_rad=0.08,
                latch_rad=0.58,
                spade_rad=0.25,
            )
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
        wp = self._select_waypoint_around_slam_obstacles(obs.pose, target_site.center)
        wp_dx = wp.x_m - obs.pose.x_m
        wp_dy = wp.y_m - obs.pose.y_m
        wp_dist = max(1e-4, hypot(wp_dx, wp_dy))
        goal_ux = wp_dx / wp_dist
        goal_uy = wp_dy / wp_dist
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
                rep_x += (ox / d_cell) * strength * 0.45 + (oy / d_cell) * strength * 1.15
                rep_y += (oy / d_cell) * strength * 0.45 - (ox / d_cell) * strength * 1.15

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
        advisor: SupervisoryAdvisor | None = None,
        orchestrator: SwarmOrchestratorAgent | None = None,
        operator_policy: OperatorPolicy | None = None,
        rescue_policy: str = "react_orchestrator",
        gossip_delay_s: float = 0.4,
    ) -> None:
        self._sites_tuple = sites
        self._capabilities_tuple = capabilities
        self.sites = {s.site_id: s for s in sites}
        self.capabilities = {c.robot_id: c for c in capabilities}
        self.comm_range_m = comm_range_m
        self.comm_mode = comm_mode
        self.envelope = envelope
        self.limits = limits
        self.orchestrator = orchestrator or SwarmOrchestratorAgent()
        self.operator_policy = operator_policy or OperatorPolicy()
        self.rescue_policy = rescue_policy
        self.gossip_delay_s = float(gossip_delay_s)
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
        self.advisor = advisor
        self._bogged_since_s: dict[str, float] = {}
        self._stuck_counts: dict[str, int] = {}
        self.last_fault_evidence: dict[str, FaultEvidence] = {}
        self.last_rescue_assessments: dict[str, RescueAssessment] = {}
        self.last_orchestrator_decisions: dict[str, OrchestratorDecision] = {}
        self.recovery_monitoring_history: list[dict[str, object]] = []
        self.last_advisor_recommendation: str | None = None
        self._sos_broadcast_sent: set[str] = set()
        self._in_flight_gossip: list[tuple[float, str, MycelialGossipPacket]] = []
        self._in_flight_meta: dict[str, dict[str, Any]] = {}
        self._sent_sample_ids: dict[tuple[str, str], set[str]] = {}
        self._sent_hazard_keys: dict[tuple[str, str], set[tuple[int, int]]] = {}
        self._sent_planted_sites: dict[tuple[str, str], set[str]] = {}

    def reset(self) -> None:
        """Reset all robot nodes, local maps, in-flight gossip queues, and fault monitors."""
        for node in self.nodes.values():
            node.reset()
        self._last_time_s = None
        self._last_episode_id = None
        self._negotiated_conflicts.clear()
        self.gossip_history.clear()
        self.latest_gossip_packets = ()
        self.latest_sensor_telemetry.clear()
        self.total_gossip_bytes = 0
        self.total_gossip_packets = 0
        self._bogged_since_s.clear()
        self._stuck_counts.clear()
        self.last_fault_evidence.clear()
        self.last_rescue_assessments.clear()
        self.last_orchestrator_decisions.clear()
        self.recovery_monitoring_history.clear()
        self.last_advisor_recommendation = None
        self._sos_broadcast_sent.clear()
        self._in_flight_gossip.clear()
        self._in_flight_meta.clear()
        self._sent_sample_ids.clear()
        self._sent_hazard_keys.clear()
        self._sent_planted_sites.clear()
        if hasattr(self.orchestrator, "aborted_techniques_by_robot"):
            self.orchestrator.aborted_techniques_by_robot.clear()

    def _deliver_gossip_packet(
        self,
        receiver_id: str,
        pkt: MycelialGossipPacket,
    ) -> None:
        """Deliver an arriving gossip packet into `receiver_node.local_map`."""
        receiver_node = self.nodes.get(receiver_id)
        if receiver_node is None:
            return
        for s in pkt.samples:
            receiver_node.local_map.ingest_sample(s, obs_precision=3.5)
        for cell in pkt.slam_hazards:
            receiver_node.local_map.slam.merge_peer_cell(cell)
        meta = self._in_flight_meta.pop(pkt.packet_id, None)
        if meta is not None:
            for planted_site in meta.get("planted_sites", ()):
                receiver_node.local_map.planted_sites.add(planted_site)
                receiver_node.local_map.attempted_sites.add(planted_site)
            for disc_site in meta.get("discovered_sites", ()):
                receiver_node.local_map.discovered_sites.add(disc_site)
            for probed_site in meta.get("probed_sites", ()):
                receiver_node.local_map.probed_sites.add(probed_site)
                receiver_node.local_map.discovered_sites.add(probed_site)
            for sid, vpriors in meta.get("visual_priors", {}).items():
                if sid not in receiver_node.local_map.probed_sites:
                    cur = receiver_node.local_map.visual_priors.setdefault(sid, {})
                    for k, val in vpriors.items():
                        if isinstance(val, GaussianChannelBelief):
                            if val.count > 0 and (k not in cur or val.precision > cur[k].precision):
                                cur[k] = GaussianChannelBelief(
                                    mean=val.mean,
                                    precision=val.precision,
                                    count=val.count,
                                )
                        elif k not in cur:
                            cur[k] = val
            peer_grid: ContinuousBeliefGrid | None = meta.get("continuous_grid")
            if peer_grid is not None:
                receiver_node.local_map.continuous_grid.merge_peer_grid(peer_grid, blend_weight=0.55)
            else:
                receiver_node.local_map.continuous_grid.version += 1

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
        robot_ids = sorted(obs_by_robot.keys())

        # Compute link-gated peer visibility per robot (zero peer pose leakage during blackout/out-of-range)
        visible_peer_poses_for_robot: dict[str, dict[str, Pose2D]] = {}
        reachable_peers: dict[str, set[str]] = {rid: {rid} for rid in robot_ids}
        for rid_a in robot_ids:
            obs_a = obs_by_robot[rid_a]
            vis_poses: dict[str, Pose2D] = {rid_a: obs_a.pose}
            for rid_b in robot_ids:
                if rid_a == rid_b:
                    continue
                obs_b = obs_by_robot[rid_b]
                d_ab = hypot(obs_b.pose.x_m - obs_a.pose.x_m, obs_b.pose.y_m - obs_a.pose.y_m)
                if obs_a.link_ok and obs_b.link_ok and self._is_link_open(frame.time_s, d_ab):
                    vis_poses[rid_b] = obs_b.pose
                    reachable_peers[rid_a].add(rid_b)
            visible_peer_poses_for_robot[rid_a] = vis_poses

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
        for rf in frame.recovery_feedback:
            mon = self.orchestrator.record_recovery_feedback(rf)
            self.recovery_monitoring_history.append(mon)
            if rf.extracted:
                self._stuck_counts[rf.target_robot_id] = 0
        total_ingested = 0
        self.latest_sensor_telemetry = {}
        self.last_fault_evidence = {}
        for robot_id, obs in obs_by_robot.items():
            node = self.nodes[robot_id]
            prev_cmd_lin = node.last_commanded_linear_m_s
            telemetry, count, _ = node.process_local_sensors(
                obs, visible_peer_poses_for_robot[robot_id]
            )
            self.latest_sensor_telemetry[robot_id] = telemetry
            total_ingested += count
            actively_cmd = abs(prev_cmd_lin) >= 0.04
            clear_rng = obs.front_range_m is not None and obs.front_range_m > self.limits.stop_range_m
            is_stall = bool(actively_cmd and clear_rng and obs.link_ok and telemetry.true_ground_speed_m_s < 0.025)
            extracted_now = any(
                rf.target_robot_id == robot_id and rf.extracted
                for rf in frame.recovery_feedback
            )
            being_pulled_now = any(
                rf.target_robot_id == robot_id and rf.latched and not rf.extracted
                for rf in frame.recovery_feedback
            )
            if extracted_now or (not being_pulled_now and telemetry.true_ground_speed_m_s >= 0.03):
                self._stuck_counts[robot_id] = 0
            elif is_stall:
                self._stuck_counts[robot_id] = self._stuck_counts.get(robot_id, 0) + 1
            elif self._stuck_counts.get(robot_id, 0) < 2:
                self._stuck_counts[robot_id] = 0
            stalled_ticks_now = self._stuck_counts.get(robot_id, 0)
            self.last_fault_evidence[robot_id] = FaultEvidence(
                robot_id=robot_id,
                observed_at_s=obs.time_s,
                commanded_linear_m_s=prev_cmd_lin,
                commanded_angular_rad_s=0.0,
                observed_displacement_m=round(telemetry.true_ground_speed_m_s * 0.1, 5),
                observed_yaw_delta_rad=0.0,
                observed_speed_m_s=obs.true_speed_m_s,
                slip_ratio=0.90 if stalled_ticks_now >= 2 else telemetry.slip_ratio,
                pitch_rad=obs.pitch_rad,
                roll_rad=obs.roll_rad,
                front_range_m=obs.front_range_m,
                link_ok=obs.link_ok,
                stalled_ticks=stalled_ticks_now,
                is_commanded_motion_stall=is_stall,
                confidence=0.95 if stalled_ticks_now >= 2 else 0.85,
            )

        # 3. Each robot independently computes its initial local task bids before peer negotiation
        local_bids_by_robot: dict[str, list[TaskBidProposal]] = {
            rid: self.nodes[rid].compute_task_bids(obs) for rid, obs in obs_by_robot.items()
        }

        # 4. Event-Triggered Peer-to-Peer Mycelial Gossip (with realistic gossip_delay_s transit queue)
        if self._in_flight_gossip:
            still_in_flight: list[tuple[float, str, MycelialGossipPacket]] = []
            delivered_receivers: set[str] = set()
            for deliver_at_s, receiver_id, queued_pkt in self._in_flight_gossip:
                if deliver_at_s <= frame.time_s + 1e-6:
                    self._deliver_gossip_packet(receiver_id, queued_pkt)
                    delivered_receivers.add(receiver_id)
                else:
                    still_in_flight.append((deliver_at_s, receiver_id, queued_pkt))
            self._in_flight_gossip = still_in_flight
            for rec_id in delivered_receivers:
                if rec_id in obs_by_robot:
                    local_bids_by_robot[rec_id] = self.nodes[rec_id].compute_task_bids(obs_by_robot[rec_id])

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

                pair_key = (sender_id, receiver_id)
                already_sent_samples = self._sent_sample_ids.setdefault(pair_key, set())
                already_sent_hazards = self._sent_hazard_keys.setdefault(pair_key, set())
                already_sent_planted = self._sent_planted_sites.setdefault(pair_key, set())

                unseen_samples = tuple(
                    s
                    for sid, s in sender_node.local_map.all_samples.items()
                    if sid not in receiver_node.local_map.seen_sample_ids
                    and sid not in already_sent_samples
                )
                unseen_hazards = tuple(
                    cell
                    for key, cell in sender_node.local_map.slam.cells.items()
                    if key not in receiver_node.local_map.slam.cells
                    and key not in already_sent_hazards
                )
                unseen_planted = (
                    sender_node.local_map.planted_sites
                    - receiver_node.local_map.planted_sites
                    - already_sent_planted
                )

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

                has_new_distress = (
                    sender_node.active_distress is not None
                    and receiver_node.peer_distress_signals.get(sender_id) != sender_node.active_distress
                )

                if not unseen_samples and not unseen_hazards and not unseen_planted and not new_bid_conflict and not has_new_distress:
                    continue

                dialogue: list[str] = []
                # Emergency SOS distress broadcasts are always delivered immediately so rescue latency is unaffected
                if has_new_distress and sender_node.active_distress is not None:
                    receiver_node.peer_distress_signals[sender_id] = sender_node.active_distress
                    if sender_node.active_distress.residual_weights:
                        receiver_node.rescue_engine.learner.import_weights_flat(
                            sender_node.active_distress.residual_weights
                        )
                    dialogue.append(f"{sender_id}->{receiver_id}: {sender_node.active_distress.summary_text}")

                for s in unseen_samples:
                    already_sent_samples.add(s.sample_id)
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

                for cell in unseen_hazards:
                    already_sent_hazards.add((cell.cell_ix, cell.cell_iy))
                    dialogue.append(
                        f"{sender_id}->{receiver_id}: SLAM {cell.kind} mapped at ({cell.x_m:.2f}, {cell.y_m:.2f})"
                    )

                for planted_site in unseen_planted:
                    already_sent_planted.add(planted_site)
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
                    distress_signal=sender_node.active_distress,
                )

                has_fieldwork_payload = bool(
                    unseen_samples or unseen_hazards or unseen_planted or new_bid_conflict
                )
                if has_fieldwork_payload:
                    self._in_flight_meta[pkt.packet_id] = {
                        "planted_sites": tuple(unseen_planted),
                        "discovered_sites": tuple(sender_node.local_map.discovered_sites),
                        "probed_sites": tuple(sender_node.local_map.probed_sites),
                        "visual_priors": {
                            k: dict(v) for k, v in sender_node.local_map.visual_priors.items()
                        },
                        "continuous_grid": sender_node.local_map.continuous_grid.copy(),
                    }
                    if self.gossip_delay_s > 0.0:
                        deliver_at_s = round(frame.time_s + self.gossip_delay_s, 4)
                        self._in_flight_gossip.append((deliver_at_s, receiver_id, pkt))
                    else:
                        self._deliver_gossip_packet(receiver_id, pkt)

                cycle_packets.append(pkt)
                self.gossip_history.append(pkt)
                self.total_gossip_bytes += byte_size
                self.total_gossip_packets += 1

        self.latest_gossip_packets = tuple(cycle_packets)

        # 5. Recompute bids after gossip education & resolve peer task ownership via Consensus Bundle Auction
        updated_bids: dict[str, list[TaskBidProposal]] = {
            rid: self.nodes[rid].compute_task_bids(obs) for rid, obs in obs_by_robot.items()
        }

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

        # 6. Autonomous Peer Rescue Triage (RESCUE_NOW vs DEFER_RESCUE vs ABANDON_ESCALATE) & System-2 Advisor
        # Uses sensor-derived commanded-versus-observed motion stall evidence (_stuck_counts >= 2), not simulator truth.
        self.last_rescue_assessments = {}
        rescue_intent_by_robot: dict[str, TaskIntent] = {}
        stranded_ids = {rid for rid in robot_ids if self._stuck_counts.get(rid, 0) >= 2}
        for rid in robot_ids:
            if rid in stranded_ids:
                self._bogged_since_s.setdefault(rid, frame.time_s)
            else:
                self._bogged_since_s.pop(rid, None)
                self._sos_broadcast_sent.discard(rid)

        for stranded_id, stranded_obs in obs_by_robot.items():
            if stranded_id not in stranded_ids:
                continue
            nearest_site_id = min(
                (s.site_id for s in self.sites.values() if s.permitted),
                key=lambda sid: hypot(
                    self.sites[sid].center.x_m - stranded_obs.pose.x_m,
                    self.sites[sid].center.y_m - stranded_obs.pose.y_m,
                ),
            )
            rescue_intent_by_robot[stranded_id] = TaskIntent(
                task_id=f"{frame.episode_id}:{frame.time_s:.3f}:{stranded_id}:hold",
                robot_id=stranded_id,
                mode=TaskMode.HOLD,
                issued_at_s=frame.time_s,
                expires_at_s=frame.time_s + 1.0,
                target_site_id=None,
                reason=f"stalled at {nearest_site_id}; holding for peer rigid-dock extraction",
            )
            candidates = [
                r
                for r in frame.robots
                if r.robot_id != stranded_id
                and r.robot_id not in stranded_ids
                and r.link_ok
                and r.robot_id in reachable_peers.get(stranded_id, set())
            ]
            if not candidates:
                continue
            rescuer = min(
                candidates,
                key=lambda r: hypot(
                    r.pose.x_m - stranded_obs.pose.x_m,
                    r.pose.y_m - stranded_obs.pose.y_m,
                ),
            )
            dist_m = max(0.05, hypot(rescuer.pose.x_m - stranded_obs.pose.x_m, rescuer.pose.y_m - stranded_obs.pose.y_m))
            eta_s = dist_m / 0.35
            saved_benefit = round((1.80 if self.capabilities[stranded_id].can_plant else 0.70) + 1.35 * stranded_obs.seeds_remaining, 4)
            opp_cost = round(0.18 * eta_s, 4)
            net_utility = round(saved_benefit - opp_cost, 4)

            est_near = self.nodes[rescuer.robot_id].local_map.estimate(nearest_site_id)
            local_m = est_near.moisture if est_near.moisture is not None else 0.88
            local_f = est_near.firmness if est_near.firmness is not None else 0.19
            incident = IncidentReport(
                incident_id=f"inc-{stranded_id}-{frame.time_s:.2f}",
                time_s=frame.time_s,
                stranded_robot_id=stranded_id,
                failure_mode=FailureMode.CONTAGIOUS_MUD_TRAP if local_f < 0.25 else FailureMode.ASYMMETRIC_SLIP_OR_SUCTION,
                stalled_window_s=max(0.2, frame.time_s - self._bogged_since_s.get(stranded_id, frame.time_s)),
                slip_ratio=self.last_fault_evidence[stranded_id].slip_ratio,
                imu_pitch_deg=self.latest_sensor_telemetry[stranded_id].pitch_deg,
                imu_roll_deg=self.latest_sensor_telemetry[stranded_id].roll_deg,
                front_range_m=stranded_obs.front_range_m,
                local_moisture=local_m,
                local_firmness=local_f,
                seeds_remaining=stranded_obs.seeds_remaining,
                available_helpers=tuple(
                    HelperStatus(
                        robot_id=c.robot_id,
                        pose=c.pose,
                        seeds_remaining=c.seeds_remaining,
                        battery_ratio=0.95,
                        is_operational=c.link_ok,
                    )
                    for c in candidates
                ),
                operator_policy=self.operator_policy,
                commanded_speed_m_s=max(0.20, abs(self.nodes[stranded_id].last_commanded_linear_m_s)),
                measured_speed_m_s=stranded_obs.true_speed_m_s or 0.0,
                nearest_site_id=nearest_site_id,
                rescue_benefit=saved_benefit,
                opportunity_cost=opp_cost,
                hazard_risk=0.0,
                net_rescue_value=net_utility,
                helper_can_plant_now=False,
            )
            if self.rescue_policy == "react_orchestrator":
                orch_dec = self.orchestrator.orchestrate(incident)
                self.last_orchestrator_decisions[stranded_id] = orch_dec
                assessment = self.orchestrator.to_rescue_assessment(incident, orch_dec)
                verdict = assessment.verdict
                chosen_tech_val = orch_dec.chosen_technique.value
            elif self.rescue_policy == "never_rescue":
                verdict = RescueVerdict.ABANDON_ESCALATE
                chosen_tech_val = "none"
                assessment = RescueAssessment(
                    stranded_robot_id=stranded_id,
                    rescuer_robot_id=None,
                    nearest_site_id=nearest_site_id,
                    expected_saved_benefit=saved_benefit,
                    rescue_opportunity_cost=opp_cost,
                    net_rescue_utility=net_utility,
                    verdict=verdict,
                    rationale="baseline(never_rescue): never dispatch peer rescue",
                )
            else:
                verdict = RescueVerdict.RESCUE_NOW if net_utility >= 0.0 else RescueVerdict.DEFER_RESCUE
                chosen_tech_val = "direct_pry_and_pull"
                assessment = RescueAssessment(
                    stranded_robot_id=stranded_id,
                    rescuer_robot_id=rescuer.robot_id,
                    nearest_site_id=nearest_site_id,
                    expected_saved_benefit=saved_benefit,
                    rescue_opportunity_cost=opp_cost,
                    net_rescue_utility=net_utility,
                    verdict=verdict,
                    rationale=(
                        f"Peer rescue saves {saved_benefit:.2f} expected utility at cost {opp_cost:.2f} "
                        f"(net={net_utility:+.2f}, ETA={eta_s:.1f}s) -> {verdict.value}"
                    ),
                )
            self.last_rescue_assessments[stranded_id] = assessment
            if verdict == RescueVerdict.RESCUE_NOW:
                rescue_intent_by_robot[rescuer.robot_id] = TaskIntent(
                    task_id=f"{frame.episode_id}:{frame.time_s:.3f}:{rescuer.robot_id}:rescue",
                    robot_id=rescuer.robot_id,
                    mode=TaskMode.RESCUE,
                    issued_at_s=frame.time_s,
                    expires_at_s=frame.time_s + 1.0,
                    target_site_id=nearest_site_id,
                    target_robot_id=stranded_id,
                    reason=f"rescue triage={verdict.value} for {stranded_id} at {nearest_site_id} ({assessment.rationale})",
                    recovery_technique=chosen_tech_val if chosen_tech_val != "none" else "direct_pry_and_pull",
                )
                if stranded_id not in self._sos_broadcast_sent:
                    self._sos_broadcast_sent.add(stranded_id)
                    sos_pkt = MycelialGossipPacket(
                        packet_id=f"pkt:{frame.time_s:.2f}:{stranded_id}->{rescuer.robot_id}:sos",
                        sender_id=stranded_id,
                        receiver_id=rescuer.robot_id,
                        sent_at_s=frame.time_s,
                        uwb_distance_m=round(dist_m, 4),
                        uwb_aoa_rad=0.0,
                        samples=(),
                        slam_hazards=(),
                        task_bid=None,
                        byte_size=64,
                        dialogue_summary=(
                            f"{stranded_id}->{rescuer.robot_id}: SOS BOGGED at {nearest_site_id}; "
                            f"{verdict.value} net={net_utility:+.2f} -> rigid-dock rescue dispatched",
                        ),
                    )
                    self.gossip_history.append(sos_pkt)
                    self.latest_gossip_packets = (*self.latest_gossip_packets, sos_pkt)
                    self.total_gossip_bytes += 64
                    self.total_gossip_packets += 1

        if self.advisor is not None and not rescue_intent_by_robot:
            try:
                rec_map = self.advisor.recommend(
                    frame,
                    self.sites,
                    self.nodes[robot_ids[0]].local_map.estimates() if robot_ids else {},
                    self.nodes[robot_ids[0]].local_map.planted_sites if robot_ids else set(),
                    [],
                )
                if rec_map:
                    self.last_advisor_recommendation = ", ".join(f"{k}->{v}" for k, v in rec_map.items())
            except Exception:
                pass

        for rid in robot_ids:
            node = self.nodes[rid]
            obs = obs_by_robot[rid]
            winning_bid = assigned_bid_for_robot.get(rid)
            if rid in rescue_intent_by_robot:
                intent = rescue_intent_by_robot[rid]
            elif winning_bid is None:
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
            cmd = node.compute_local_command(obs, intent, visible_peer_poses_for_robot[rid])
            intents.append(intent)
            commands.append(cmd)

        return Decision(
            intents=tuple(intents),
            commands=tuple(commands),
            ingested_samples=total_ingested,
        )


def evaluate_continuous_belief_field(
    local_map: LocalMycelialMap,
    x_m: float,
    y_m: float,
    length_scale_m: float = 0.52,
) -> dict[str, float]:
    """Evaluate a single robot's continuous 2D Gaussian-Process / RBF belief field at `(x_m, y_m)`.

    Transforms the robot's local observations (`local_map.all_samples` and `local_map.slam.cells`)
    into continuous spatial surfaces for:
    - `moisture`: Posterior mean moisture $\\hat{M}_i(x, y) \\in [0, 1]$
    - `firmness`: Posterior mean cone-index firmness $\\hat{F}_i(x, y) \\in [0, 1]$
    - `stand_gap`: Posterior mean stand gap $\\hat{G}_i(x, y) \\in [0, 1]$
    - `plant_density`: Estimated living plant density $\\hat{\\rho}_{\\text{plant},i}(x, y) = 1 - \\hat{G}_i(x, y)$
    - `uncertainty`: Continuous epistemic uncertainty $\\sigma_i(x, y) \\in [0, 1]$ (high in unexplored regions,
      collapses toward 0 around directly sampled or peer-gossiped locations)
    - `traversability_risk`: Combined SLAM obstacle + mud-sinkage risk $\\mathcal{R}_i(x, y) \\in [0, 1]$
    """
    from math import exp

    prior_weight = 0.22
    l2 = 2.0 * length_scale_m * length_scale_m

    w_m = prior_weight
    w_f = prior_weight
    w_g = prior_weight
    acc_m = prior_weight * 0.50
    acc_f = prior_weight * 0.50
    acc_g = prior_weight * 0.50
    info_density = 0.0

    # 1. Publicly known protected living rice stands (crop_nw, crop_ne) have known high plant density
    for site in local_map.sites.values():
        if not site.permitted:
            dx = x_m - site.center.x_m
            dy = y_m - site.center.y_m
            k = 3.2 * exp(-(dx * dx + dy * dy) / (2.0 * 0.34 * 0.34))
            acc_g += k * 0.05
            w_g += k
            info_density += 0.5 * k

    # 2. Fuse all samples currently stored in THIS robot's private LocalMycelialMap
    for sample in local_map.all_samples.values():
        dx = x_m - sample.location.x_m
        dy = y_m - sample.location.y_m
        k = 2.8 * exp(-(dx * dx + dy * dy) / l2)
        if sample.kind == SensorKind.MOISTURE:
            acc_m += k * sample.value
            w_m += k
            info_density += k
        elif sample.kind == SensorKind.FIRMNESS:
            acc_f += k * sample.value
            w_f += k
        elif sample.kind == SensorKind.STAND_GAP:
            acc_g += k * sample.value
            w_g += k

    # 2b. Fuse Tier-1 camera FOV cone precision & visual surface priors from local_map.continuous_grid
    if hasattr(local_map, "continuous_grid") and local_map.continuous_grid is not None:
        cg = local_map.continuous_grid
        gx, gy = cg.world_to_index(x_m, y_m)
        cam_prec = float(cg.precision[gy, gx]) - cg.prior_precision
        if cam_prec > 1e-3:
            w_cam = min(1.8, 0.35 * cam_prec)
            acc_m += w_cam * float(cg.moisture[gy, gx])
            w_m += w_cam
            acc_g += w_cam * float(cg.stand_gap[gy, gx])
            w_g += w_cam
            info_density += 0.45 * cam_prec

    mean_m = max(0.02, min(0.98, acc_m / w_m))
    mean_f = max(0.02, min(0.98, acc_f / w_f))
    mean_g = max(0.02, min(0.98, acc_g / w_g))
    plant_density = max(0.02, min(0.98, 1.0 - mean_g))
    uncertainty = max(0.04, min(1.0, 1.0 / (1.0 + 1.35 * info_density)))

    # 3. Continuous SLAM + mud-sinkage traversability risk
    slam_risk = 0.0
    for cell in local_map.slam.cells.values():
        dx = x_m - cell.x_m
        dy = y_m - cell.y_m
        k_slam = exp(-(dx * dx + dy * dy) / (2.0 * 0.24 * 0.24))
        weight = 0.95 if cell.kind == "obstacle" else 0.80
        slam_risk = max(slam_risk, weight * k_slam)

    soil_mud_risk = max(0.0, (mean_m - 0.55) * 1.4) * max(0.0, (0.55 - mean_f) * 1.4) * (1.0 - 0.5 * uncertainty)
    traversability_risk = max(0.02, min(0.99, max(slam_risk, soil_mud_risk)))

    # 4. Continuous Rice-Planting Agronomic Fitness belief Φ_hat(x, y)
    s_moisture = exp(-((mean_m - 0.56) ** 2) / (2.0 * 0.14 * 0.14))
    s_firmness = exp(-((mean_f - 0.65) ** 2) / (2.0 * 0.16 * 0.16))
    s_gap = max(0.0, min(1.0, (mean_g - 0.22) / 0.66)) ** 1.25
    planting_fitness = max(0.0, min(1.0, s_gap * (0.52 * s_moisture + 0.48 * s_firmness) * (1.0 - 0.35 * slam_risk)))

    return {
        "moisture": round(mean_m, 4),
        "firmness": round(mean_f, 4),
        "stand_gap": round(mean_g, 4),
        "plant_density": round(plant_density, 4),
        "uncertainty": round(uncertainty, 4),
        "traversability_risk": round(traversability_risk, 4),
        "planting_fitness": round(planting_fitness, 4),
    }


def sample_continuous_belief_grid(
    local_map: LocalMycelialMap,
    nx: int = 36,
    ny: int = 24,
    x_bounds: tuple[float, float] = (-0.25, 2.95),
    y_bounds: tuple[float, float] = (-0.90, 0.90),
    length_scale_m: float = 0.52,
) -> dict[str, list[list[float]]]:
    """Sample a robot's continuous 2D belief field across a `(ny x nx)` grid using vectorized 2D NumPy kernels + version caching."""
    cache_key = (local_map.state_version, nx, ny, x_bounds, y_bounds, length_scale_m)
    cached = local_map._grid_cache.get(cache_key)
    if cached is not None:
        return cached

    xs_arr = np.linspace(x_bounds[0], x_bounds[1], nx, dtype=np.float64)
    ys_arr = np.linspace(y_bounds[0], y_bounds[1], ny, dtype=np.float64)
    X, Y = np.meshgrid(xs_arr, ys_arr)

    prior_weight = 0.22
    l2 = 2.0 * length_scale_m * length_scale_m

    w_m = np.full((ny, nx), prior_weight, dtype=np.float64)
    w_f = np.full((ny, nx), prior_weight, dtype=np.float64)
    w_g = np.full((ny, nx), prior_weight, dtype=np.float64)
    acc_m = np.full((ny, nx), prior_weight * 0.50, dtype=np.float64)
    acc_f = np.full((ny, nx), prior_weight * 0.50, dtype=np.float64)
    acc_g = np.full((ny, nx), prior_weight * 0.50, dtype=np.float64)
    info_density = np.zeros((ny, nx), dtype=np.float64)

    # 1. Publicly known protected living rice stands (crop_nw, crop_ne)
    inv_2s2_crop = 1.0 / (2.0 * 0.34 * 0.34)
    for site in local_map.sites.values():
        if not site.permitted:
            dx = X - site.center.x_m
            dy = Y - site.center.y_m
            k = 3.2 * np.exp(-(dx * dx + dy * dy) * inv_2s2_crop)
            acc_g += k * 0.05
            w_g += k
            info_density += 0.5 * k

    # 2. Vectorized fusion across all samples in THIS robot's private LocalMycelialMap
    inv_l2 = 1.0 / l2
    for sample in local_map.all_samples.values():
        dx = X - sample.location.x_m
        dy = Y - sample.location.y_m
        k = 2.8 * np.exp(-(dx * dx + dy * dy) * inv_l2)
        if sample.kind == SensorKind.MOISTURE:
            acc_m += k * sample.value
            w_m += k
            info_density += k
        elif sample.kind == SensorKind.FIRMNESS:
            acc_f += k * sample.value
            w_f += k
        elif sample.kind == SensorKind.STAND_GAP:
            acc_g += k * sample.value
            w_g += k

    # 2b. Fuse Tier-1 camera FOV cone precision & visual surface priors from local_map.continuous_grid
    if hasattr(local_map, "continuous_grid") and local_map.continuous_grid is not None:
        cg = local_map.continuous_grid
        if cg.nx == nx and cg.ny == ny:
            cam_prec = np.maximum(0.0, cg.precision - cg.prior_precision)
            w_cam = np.minimum(1.8, 0.35 * cam_prec)
            acc_m += w_cam * cg.moisture
            w_m += w_cam
            acc_g += w_cam * cg.stand_gap
            w_g += w_cam
            info_density += 0.45 * cam_prec
        else:
            gx_idx = np.clip(
                np.round((X - cg.x_bounds[0]) / max(1e-6, cg.x_bounds[1] - cg.x_bounds[0]) * (cg.nx - 1)).astype(int),
                0,
                cg.nx - 1,
            )
            gy_idx = np.clip(
                np.round((Y - cg.y_bounds[0]) / max(1e-6, cg.y_bounds[1] - cg.y_bounds[0]) * (cg.ny - 1)).astype(int),
                0,
                cg.ny - 1,
            )
            cam_prec = np.maximum(0.0, cg.precision[gy_idx, gx_idx] - cg.prior_precision)
            w_cam = np.minimum(1.8, 0.35 * cam_prec)
            acc_m += w_cam * cg.moisture[gy_idx, gx_idx]
            w_m += w_cam
            acc_g += w_cam * cg.stand_gap[gy_idx, gx_idx]
            w_g += w_cam
            info_density += 0.45 * cam_prec

    mean_m = np.clip(acc_m / w_m, 0.02, 0.98)
    mean_f = np.clip(acc_f / w_f, 0.02, 0.98)
    mean_g = np.clip(acc_g / w_g, 0.02, 0.98)
    plant_density = np.clip(1.0 - mean_g, 0.02, 0.98)
    uncertainty = np.clip(1.0 / (1.0 + 1.35 * info_density), 0.04, 1.0)

    # 3. Continuous SLAM + mud-sinkage traversability risk
    slam_risk = np.zeros((ny, nx), dtype=np.float64)
    inv_2s2_slam = 1.0 / (2.0 * 0.24 * 0.24)
    for cell in local_map.slam.cells.values():
        dx = X - cell.x_m
        dy = Y - cell.y_m
        k_slam = np.exp(-(dx * dx + dy * dy) * inv_2s2_slam)
        weight = 0.95 if cell.kind == "obstacle" else 0.80
        np.maximum(slam_risk, weight * k_slam, out=slam_risk)

    soil_mud_risk = (
        np.maximum(0.0, (mean_m - 0.55) * 1.4)
        * np.maximum(0.0, (0.55 - mean_f) * 1.4)
        * (1.0 - 0.5 * uncertainty)
    )
    traversability_risk = np.clip(np.maximum(slam_risk, soil_mud_risk), 0.02, 0.99)

    # 4. Continuous Rice-Planting Agronomic Fitness belief Φ_hat(x, y)
    s_moisture = np.exp(-((mean_m - 0.56) ** 2) / (2.0 * 0.14 * 0.14))
    s_firmness = np.exp(-((mean_f - 0.65) ** 2) / (2.0 * 0.16 * 0.16))
    s_gap = np.clip((mean_g - 0.22) / 0.66, 0.0, 1.0) ** 1.25
    planting_fitness = np.clip(
        s_gap * (0.52 * s_moisture + 0.48 * s_firmness) * (1.0 - 0.35 * slam_risk),
        0.0,
        1.0,
    )

    layers_np = {
        "moisture": np.round(mean_m, 4),
        "firmness": np.round(mean_f, 4),
        "stand_gap": np.round(mean_g, 4),
        "plant_density": np.round(plant_density, 4),
        "uncertainty": np.round(uncertainty, 4),
        "traversability_risk": np.round(traversability_risk, 4),
        "planting_fitness": np.round(planting_fitness, 4),
    }
    result: dict[str, list[list[float]]] = {k: v.tolist() for k, v in layers_np.items()}
    # Keep only the latest 4 cache entries per robot to prevent unbounded memory growth
    if len(local_map._grid_cache) >= 4:
        local_map._grid_cache.clear()
    local_map._grid_cache[cache_key] = result
    return result
