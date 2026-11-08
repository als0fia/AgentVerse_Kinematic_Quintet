"""World, level rice-field geometry, and local sensor host for PaddyPatch (`src/ricebot/world/`).

Strict separation per `docs/COMPONENTS.md` and `docs/FIVE-AGENT-RECONCILIATION.md`:
- Owns field sites (`DEFAULT_SITES`), public traversability/no-go flags, hidden synthetic
  spatial soil/stand distribution, continuous 2D level-paddy terrain functions
  (`ContinuousPaddyTerrain`), location-gated soil probes with configurable uncertainty
  and dropout windows, sensor-derived `FaultEvidence`, and realistic rover-scale
  rice-field MJCF geometry (`build_world_terrain_xml`).
- Abandonment of stepped terraces: represents ONE largely level, traversable operational
  paddy plot (`z ≈ 0.0 m` with `±4–24 mm` micro-topography and low `8.5 cm` perimeter bunds)
  scaled around Agent B's real `27.0 cm × 27.6 cm` Paddy Ant rover dimensions.
- NEVER owns rover chassis, wheel joints, skid-steer kinematics, seed hopper geometry,
  or mission planning.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import cos, exp, hypot, isfinite, pi, sin
import random
from typing import Any

import numpy as np

from ..contracts import Pose2D, SensorKind, Site, SoilSample

try:
    from ..contracts import FaultEvidence
except ImportError:

    def _finite(*numbers: float) -> None:
        if not all(isfinite(n) for n in numbers):
            raise ValueError("non-finite physical quantity")

    @dataclass(frozen=True)
    class FaultEvidence:  # type: ignore[no-redef]
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
            if (
                self.observed_at_s < 0
                or self.observed_displacement_m < 0
                or self.observed_yaw_delta_rad < 0
            ):
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
                abs(self.commanded_linear_m_s) >= 0.04
                or abs(self.commanded_angular_rad_s) >= 0.10
            )
            if self.is_commanded_motion_stall:
                if not actively_commanded:
                    raise ValueError(
                        "cannot claim commanded-motion stall without active drive command"
                    )
                if self.front_range_m is None:
                    raise ValueError(
                        "missing rangefinder (None) is sensor invalidity, not motion stall"
                    )


def derive_fault_evidence(
    *,
    robot_id: str,
    observed_at_s: float,
    commanded_linear_m_s: float,
    commanded_angular_rad_s: float,
    observed_displacement_m: float,
    observed_yaw_delta_rad: float,
    observed_speed_m_s: float | None,
    slip_ratio: float,
    pitch_rad: float,
    roll_rad: float,
    front_range_m: float | None,
    link_ok: bool,
    previous_stalled_ticks: int = 0,
    stop_range_m: float = 0.32,
    stall_ticks_threshold: int = 1,
) -> FaultEvidence:
    """Derive per-robot `FaultEvidence` strictly from onboard command-vs-motion & sensor validity.

    Negative controls enforced:
    1. `HOLD` or zero wheel command (`abs(lin) < 0.04 and abs(ang) < 0.10`) -> `stalled_ticks = 0`, `is_commanded_motion_stall = False`.
    2. Missing/invalid rangefinder (`front_range_m is None`) -> `stalled_ticks = 0`, `is_commanded_motion_stall = False`.
    3. Obstacle safety stop (`front_range_m <= stop_range_m`) -> `stalled_ticks = 0`, `is_commanded_motion_stall = False`.
    """
    actively_commanded = (
        abs(commanded_linear_m_s) >= 0.04 or abs(commanded_angular_rad_s) >= 0.10
    )
    clear_range = front_range_m is not None and front_range_m > stop_range_m
    speed_stalled = observed_speed_m_s is None or observed_speed_m_s < 0.035
    pose_stalled = observed_displacement_m < 0.008 and observed_yaw_delta_rad < 0.025

    if actively_commanded and clear_range and link_ok and speed_stalled and pose_stalled:
        stalled_ticks = previous_stalled_ticks + 1
    else:
        stalled_ticks = 0

    is_stall = bool(
        actively_commanded
        and front_range_m is not None
        and clear_range
        and link_ok
        and stalled_ticks >= max(1, stall_ticks_threshold)
    )
    conf = 0.35 if front_range_m is None else (0.92 if is_stall else 0.88)
    return FaultEvidence(
        robot_id=robot_id,
        observed_at_s=round(observed_at_s, 4),
        commanded_linear_m_s=round(commanded_linear_m_s, 4),
        commanded_angular_rad_s=round(commanded_angular_rad_s, 4),
        observed_displacement_m=round(max(0.0, observed_displacement_m), 4),
        observed_yaw_delta_rad=round(max(0.0, observed_yaw_delta_rad), 4),
        observed_speed_m_s=round(observed_speed_m_s, 4) if observed_speed_m_s is not None else None,
        slip_ratio=round(max(0.0, min(1.0, slip_ratio)), 4),
        pitch_rad=round(pitch_rad, 4),
        roll_rad=round(roll_rad, 4),
        front_range_m=round(front_range_m, 4) if front_range_m is not None else None,
        link_ok=bool(link_ok),
        stalled_ticks=stalled_ticks,
        is_commanded_motion_stall=is_stall,
        confidence=round(conf, 4),
    )


# 6-Patch Paddy Field Layout (2 rows x 3 columns inside a level 3.4m x 2.1m bunded paddy plot)
# - patch_a (0.85, -0.50): Waterlogged Lannitian mud pan (moisture=0.92, firmness=0.16) -> UNSUITABLE
# - patch_b (1.60, -0.50): Viable stand gap (moisture=0.54, firmness=0.68, stand_gap=0.90) -> VIABLE #1
# - patch_c (2.35, -0.50): Viable stand gap (moisture=0.50, firmness=0.72, stand_gap=0.84) -> VIABLE #2
# - patch_d (1.60,  0.45): Waterlogged mud pan (moisture=0.89, firmness=0.19, stand_gap=0.78) -> UNSUITABLE
# - crop_nw (0.85,  0.45): Protected living rice stand (permitted=False)
# - crop_ne (2.35,  0.45): Protected living rice stand (permitted=False)
DEFAULT_SITES: tuple[Site, ...] = (
    Site("patch_a", Pose2D(0.85, -0.50, 0.0), permitted=True, no_go_radius_m=0.26),
    Site("patch_b", Pose2D(1.60, -0.50, 0.0), permitted=True, no_go_radius_m=0.26),
    Site("patch_c", Pose2D(2.35, -0.50, 0.0), permitted=True, no_go_radius_m=0.26),
    Site("patch_d", Pose2D(1.60, 0.45, 0.0), permitted=True, no_go_radius_m=0.26),
    Site("crop_nw", Pose2D(0.85, 0.45, 0.0), permitted=False, no_go_radius_m=0.28),
    Site("crop_ne", Pose2D(2.35, 0.45, 0.0), permitted=False, no_go_radius_m=0.28),
)


DEFAULT_HIDDEN_SOIL_TRUTH: dict[str, dict[SensorKind, float]] = {
    "patch_a": {
        SensorKind.MOISTURE: 0.92,
        SensorKind.FIRMNESS: 0.16,
        SensorKind.STAND_GAP: 0.85,
    },
    "patch_b": {
        SensorKind.MOISTURE: 0.54,
        SensorKind.FIRMNESS: 0.68,
        SensorKind.STAND_GAP: 0.90,
    },
    "patch_c": {
        SensorKind.MOISTURE: 0.50,
        SensorKind.FIRMNESS: 0.72,
        SensorKind.STAND_GAP: 0.84,
    },
    "patch_d": {
        SensorKind.MOISTURE: 0.89,
        SensorKind.FIRMNESS: 0.19,
        SensorKind.STAND_GAP: 0.78,
    },
}

# Unexpected bad-soil scenario truth: patch_b is waterlogged mud while patch_d (north row) is viable loam.
# Tests whether Scout's discovery propagates over peer-to-peer gossip to alter Planter's planting choice.
UNEXPECTED_BAD_SOIL_TRUTH: dict[str, dict[SensorKind, float]] = {
    "patch_a": {
        SensorKind.MOISTURE: 0.92,
        SensorKind.FIRMNESS: 0.16,
        SensorKind.STAND_GAP: 0.85,
    },
    "patch_b": {
        SensorKind.MOISTURE: 0.91,
        SensorKind.FIRMNESS: 0.17,
        SensorKind.STAND_GAP: 0.82,
    },
    "patch_c": {
        SensorKind.MOISTURE: 0.50,
        SensorKind.FIRMNESS: 0.72,
        SensorKind.STAND_GAP: 0.84,
    },
    "patch_d": {
        SensorKind.MOISTURE: 0.52,
        SensorKind.FIRMNESS: 0.70,
        SensorKind.STAND_GAP: 0.88,
    },
}


def generate_hidden_soil_truth(
    scenario: str,
    seed: int,
) -> dict[str, dict[SensorKind, float]]:
    """Return evaluator-private hidden soil truth for a scenario and seed.

    - `nominal` / `obstacle` / `sensor_degraded`: uses `DEFAULT_HIDDEN_SOIL_TRUTH`
      (`patch_a`, `patch_d` are mud hazards; `patch_b`, `patch_c` are viable loam).
    - `bad_soil`: uses `UNEXPECTED_BAD_SOIL_TRUTH` (`patch_b` is mud; `patch_c`, `patch_d` viable).
    - `shifted_moisture`: deterministically permutes which 2 of the 4 permitted patches
      (`patch_a..patch_d`) are viable stand gaps vs. waterlogged *Lannitian* mud hazards
      using `random.Random(seed)`.
    """
    if scenario == "bad_soil":
        return {k: dict(v) for k, v in UNEXPECTED_BAD_SOIL_TRUTH.items()}
    if scenario != "shifted_moisture":
        return {k: dict(v) for k, v in DEFAULT_HIDDEN_SOIL_TRUTH.items()}

    rng = random.Random(seed)
    patch_ids = ["patch_a", "patch_b", "patch_c", "patch_d"]
    viable_pair = set(rng.sample(patch_ids, 2))
    truth: dict[str, dict[SensorKind, float]] = {}
    for pid in patch_ids:
        if pid in viable_pair:
            truth[pid] = {
                SensorKind.MOISTURE: round(rng.uniform(0.46, 0.62), 2),
                SensorKind.FIRMNESS: round(rng.uniform(0.64, 0.78), 2),
                SensorKind.STAND_GAP: round(rng.uniform(0.80, 0.92), 2),
            }
        else:
            truth[pid] = {
                SensorKind.MOISTURE: round(rng.uniform(0.88, 0.95), 2),
                SensorKind.FIRMNESS: round(rng.uniform(0.14, 0.19), 2),
                SensorKind.STAND_GAP: round(rng.uniform(0.76, 0.88), 2),
            }
    return truth


def compute_rice_planting_fitness(
    moisture: float,
    firmness: float,
    stand_gap: float,
    elevation_m: float = 0.0,
) -> float:
    """Agronomic Wet Direct-Seeded Rice (WDSR) fitness index Φ(x, y) in [0.0, 1.0]."""
    s_moisture = exp(-((moisture - 0.56) ** 2) / (2.0 * 0.14 * 0.14))
    s_firmness = exp(-((firmness - 0.65) ** 2) / (2.0 * 0.16 * 0.16))
    s_micro_topo = exp(-((min(0.0, elevation_m + 0.008)) ** 2) / (2.0 * 0.018 * 0.018))
    s_gap = max(0.0, min(1.0, (stand_gap - 0.22) / 0.66)) ** 1.25
    fitness = s_gap * (0.46 * s_moisture + 0.42 * s_firmness + 0.12 * s_micro_topo)
    return round(max(0.0, min(1.0, fitness)), 4)


@dataclass(frozen=True)
class ContinuousTerrainPoint:
    """Continuous spatial field evaluation at any `(x_m, y_m)` coordinate in the level paddy plot."""

    x_m: float
    y_m: float
    elevation_m: float
    slope_grad_x: float
    slope_grad_y: float
    slope_magnitude: float
    moisture: float
    firmness: float
    mud_depth_m: float
    plant_density: float
    stand_gap: float
    bekker_kc_kpa: float
    bekker_kphi_kpa: float
    bekker_n: float
    cohesion_c_kpa: float
    friction_phi_deg: float
    planting_fitness: float = 0.0


class ContinuousPaddyTerrain:
    r"""Continuous 2D analytic terrain & soil-mechanics field $f(x, y)$ over the level paddy plot.

    Models a single largely level puddled paddy operational plot (`z ≈ 0.0 m` with gentle
    millimetre-scale drainage micro-relief and depressed waterlogged mud basins) in strict
    agreement with the MuJoCo `field_ground` collision plane (`z = 0.0 m`).
    """

    def __init__(
        self,
        sites: tuple[Site, ...] = DEFAULT_SITES,
        anchor_truth: dict[str, dict[SensorKind, float]] | None = None,
        seed: int = 7,
    ) -> None:
        self._sites = sites
        self._anchors = dict(anchor_truth or DEFAULT_HIDDEN_SOIL_TRUTH)
        self.seed = int(seed)
        self._grid_cache: dict[
            tuple[int, int, tuple[float, float], tuple[float, float], int], dict[str, Any]
        ] = {}

    def elevation_and_gradient(self, x_m: float, y_m: float) -> tuple[float, float, float]:
        """Return `(elevation_m, dz_dx, dz_dy)` at `(x_m, y_m)` in metres (level plot micro-topography)."""
        phase = 0.17 * (self.seed % 11)
        # Gentle natural paddy laser-leveled drainage grade + micro-undulations (±8–14 mm)
        z = (
            0.008 * sin(1.35 * x_m + phase) * cos(1.65 * y_m)
            + 0.005 * cos(2.40 * x_m - 1.90 * y_m)
            - 0.003 * (x_m - 1.50)
        )
        dz_dx = (
            0.008 * 1.35 * cos(1.35 * x_m + phase) * cos(1.65 * y_m)
            - 0.005 * 2.40 * sin(2.40 * x_m - 1.90 * y_m)
            - 0.003
        )
        dz_dy = (
            -0.008 * 1.65 * sin(1.35 * x_m + phase) * sin(1.65 * y_m)
            + 0.005 * 1.90 * sin(2.40 * x_m - 1.90 * y_m)
        )

        # Depressed waterlogged Lannitian mud basins around high-moisture anchors (-24 mm)
        for site in self._sites:
            truth = self._anchors.get(site.site_id)
            if truth and truth[SensorKind.MOISTURE] > 0.80:
                dx = x_m - site.center.x_m
                dy = y_m - site.center.y_m
                r2 = dx * dx + dy * dy
                sigma2 = 0.42 * 0.42
                w = exp(-r2 / (2.0 * sigma2))
                basin_depth = -0.024 * w
                z += basin_depth
                dz_dx += basin_depth * (-dx / sigma2)
                dz_dy += basin_depth * (-dy / sigma2)

        return (round(z, 5), round(dz_dx, 5), round(dz_dy, 5))

    def evaluate(self, x_m: float, y_m: float) -> ContinuousTerrainPoint:
        """Evaluate all continuous terrain, soil, and vegetation fields at `(x_m, y_m)`."""
        z, dz_dx, dz_dy = self.elevation_and_gradient(x_m, y_m)
        slope_mag = hypot(dz_dx, dz_dy)

        phase = 0.13 * (self.seed % 11)
        base_moisture = 0.56 + 0.06 * sin(1.1 * x_m + phase) * cos(1.4 * y_m)
        base_firmness = 0.62 - 0.05 * sin(1.1 * x_m + phase) * cos(1.4 * y_m)
        base_gap = 0.42 + 0.08 * cos(1.8 * x_m) * sin(1.5 * y_m)

        w_sum = 0.35
        moisture_acc = w_sum * base_moisture
        firmness_acc = w_sum * base_firmness
        gap_acc = w_sum * base_gap

        sigma_soil = 0.46
        for site in self._sites:
            dx = x_m - site.center.x_m
            dy = y_m - site.center.y_m
            r2 = dx * dx + dy * dy
            w = 2.4 * exp(-r2 / (2.0 * sigma_soil * sigma_soil))
            if not site.permitted:
                moisture_acc += w * 0.58
                firmness_acc += w * 0.66
                gap_acc += w * 0.04
                w_sum += w
            elif site.site_id in self._anchors:
                truth = self._anchors[site.site_id]
                moisture_acc += w * truth[SensorKind.MOISTURE]
                firmness_acc += w * truth[SensorKind.FIRMNESS]
                gap_acc += w * truth[SensorKind.STAND_GAP]
                w_sum += w

        moisture = max(0.05, min(0.98, moisture_acc / w_sum))
        firmness = max(0.05, min(0.98, firmness_acc / w_sum))
        stand_gap = max(0.02, min(0.98, gap_acc / w_sum))

        # Preserve exact anchor values within the 0.28 m probe radius of test sites
        for site in self._sites:
            d = hypot(x_m - site.center.x_m, y_m - site.center.y_m)
            if d <= 0.28:
                blend = max(0.0, min(1.0, (d / 0.28) ** 2))
                if not site.permitted:
                    stand_gap = (1.0 - blend) * 0.04 + blend * stand_gap
                    break
                if site.site_id in self._anchors:
                    truth = self._anchors[site.site_id]
                    moisture = (1.0 - blend) * truth[SensorKind.MOISTURE] + blend * moisture
                    firmness = (1.0 - blend) * truth[SensorKind.FIRMNESS] + blend * firmness
                    stand_gap = (1.0 - blend) * truth[SensorKind.STAND_GAP] + blend * stand_gap
                    break

        plant_density = max(0.0, min(1.0, 1.0 - stand_gap))
        mud_depth_m = max(
            0.03, min(0.35, 0.04 + 0.30 * (moisture**1.6) * ((1.0 - firmness) ** 1.2))
        )

        bekker_kc = 1.5 + 24.0 * (firmness**1.4) * (1.0 - 0.55 * moisture)
        bekker_kphi = 45.0 + 920.0 * (firmness**1.5) * (1.0 - 0.60 * moisture)
        bekker_n = 0.52 + 0.58 * firmness
        cohesion_c = 1.2 + 14.5 * firmness * (1.0 - 0.45 * moisture)
        friction_phi = 6.0 + 24.0 * firmness * (1.0 - 0.50 * moisture)
        planting_fitness = compute_rice_planting_fitness(moisture, firmness, stand_gap, z)

        return ContinuousTerrainPoint(
            x_m=round(x_m, 4),
            y_m=round(y_m, 4),
            elevation_m=round(z, 4),
            slope_grad_x=round(dz_dx, 4),
            slope_grad_y=round(dz_dy, 4),
            slope_magnitude=round(slope_mag, 4),
            moisture=round(moisture, 4),
            firmness=round(firmness, 4),
            mud_depth_m=round(mud_depth_m, 4),
            plant_density=round(plant_density, 4),
            stand_gap=round(stand_gap, 4),
            bekker_kc_kpa=round(bekker_kc, 2),
            bekker_kphi_kpa=round(bekker_kphi, 2),
            bekker_n=round(bekker_n, 3),
            cohesion_c_kpa=round(cohesion_c, 2),
            friction_phi_deg=round(friction_phi, 2),
            planting_fitness=planting_fitness,
        )

    def sample_grid(
        self,
        nx: int = 36,
        ny: int = 24,
        x_bounds: tuple[float, float] = (-0.25, 2.95),
        y_bounds: tuple[float, float] = (-0.90, 0.90),
    ) -> dict[str, object]:
        """Vectorized NumPy 2D grid evaluation with O(1) cache lookup for dashboard heatmaps."""
        cache_key = (nx, ny, x_bounds, y_bounds, self.seed)
        cached = self._grid_cache.get(cache_key)
        if cached is not None:
            return cached

        xs_np = np.linspace(x_bounds[0], x_bounds[1], nx, dtype=np.float64)
        ys_np = np.linspace(y_bounds[0], y_bounds[1], ny, dtype=np.float64)
        X, Y = np.meshgrid(xs_np, ys_np)

        phase_z = 0.17 * (self.seed % 11)
        Z = (
            0.008 * np.sin(1.35 * X + phase_z) * np.cos(1.65 * Y)
            + 0.005 * np.cos(2.40 * X - 1.90 * Y)
            - 0.003 * (X - 1.50)
        )
        sigma2_basin = 0.42 * 0.42
        for site in self._sites:
            truth = self._anchors.get(site.site_id)
            if truth and truth[SensorKind.MOISTURE] > 0.80:
                r2 = (X - site.center.x_m) ** 2 + (Y - site.center.y_m) ** 2
                Z = Z - 0.024 * np.exp(-r2 / (2.0 * sigma2_basin))

        phase_s = 0.13 * (self.seed % 11)
        base_m = 0.56 + 0.06 * np.sin(1.1 * X + phase_s) * np.cos(1.4 * Y)
        base_f = 0.62 - 0.05 * np.sin(1.1 * X + phase_s) * np.cos(1.4 * Y)
        base_g = 0.42 + 0.08 * np.cos(1.8 * X) * np.sin(1.5 * Y)

        w_sum = np.full_like(X, 0.35)
        m_acc = 0.35 * base_m
        f_acc = 0.35 * base_f
        g_acc = 0.35 * base_g
        sigma2_soil = 0.46 * 0.46

        for site in self._sites:
            r2 = (X - site.center.x_m) ** 2 + (Y - site.center.y_m) ** 2
            w = 2.4 * np.exp(-r2 / (2.0 * sigma2_soil))
            if not site.permitted:
                m_acc = m_acc + w * 0.58
                f_acc = f_acc + w * 0.66
                g_acc = g_acc + w * 0.04
                w_sum = w_sum + w
            elif site.site_id in self._anchors:
                truth = self._anchors[site.site_id]
                m_acc = m_acc + w * truth[SensorKind.MOISTURE]
                f_acc = f_acc + w * truth[SensorKind.FIRMNESS]
                g_acc = g_acc + w * truth[SensorKind.STAND_GAP]
                w_sum = w_sum + w

        M = np.clip(m_acc / w_sum, 0.05, 0.98)
        F = np.clip(f_acc / w_sum, 0.05, 0.98)
        G = np.clip(g_acc / w_sum, 0.02, 0.98)

        for site in self._sites:
            d = np.hypot(X - site.center.x_m, Y - site.center.y_m)
            mask = d <= 0.28
            if np.any(mask):
                blend = np.clip((d / 0.28) ** 2, 0.0, 1.0)
                if not site.permitted:
                    G = np.where(mask, (1.0 - blend) * 0.04 + blend * G, G)
                elif site.site_id in self._anchors:
                    truth = self._anchors[site.site_id]
                    M = np.where(
                        mask, (1.0 - blend) * truth[SensorKind.MOISTURE] + blend * M, M
                    )
                    F = np.where(
                        mask, (1.0 - blend) * truth[SensorKind.FIRMNESS] + blend * F, F
                    )
                    G = np.where(
                        mask, (1.0 - blend) * truth[SensorKind.STAND_GAP] + blend * G, G
                    )

        P = np.clip(1.0 - G, 0.0, 1.0)
        D = np.clip(0.04 + 0.30 * (M**1.6) * ((1.0 - F) ** 1.2), 0.03, 0.35)

        s_moisture = np.exp(-((M - 0.56) ** 2) / (2.0 * 0.14 * 0.14))
        s_firmness = np.exp(-((F - 0.65) ** 2) / (2.0 * 0.16 * 0.16))
        s_micro_topo = np.exp(-((np.minimum(0.0, Z + 0.008)) ** 2) / (2.0 * 0.018 * 0.018))
        s_gap = np.clip((G - 0.22) / 0.66, 0.0, 1.0) ** 1.25
        FIT = np.clip(
            s_gap * (0.46 * s_moisture + 0.42 * s_firmness + 0.12 * s_micro_topo), 0.0, 1.0
        )

        res: dict[str, Any] = {
            "nx": nx,
            "ny": ny,
            "xs": np.round(xs_np, 4).tolist(),
            "ys": np.round(ys_np, 4).tolist(),
            "x_bounds": list(x_bounds),
            "y_bounds": list(y_bounds),
            "layers": {
                "elevation_m": np.round(Z, 4).tolist(),
                "moisture": np.round(M, 4).tolist(),
                "firmness": np.round(F, 4).tolist(),
                "mud_depth_m": np.round(D, 4).tolist(),
                "plant_density": np.round(P, 4).tolist(),
                "stand_gap": np.round(G, 4).tolist(),
                "planting_fitness": np.round(FIT, 4).tolist(),
            },
        }
        self._grid_cache[cache_key] = res
        return res


class PaddyTerraceWorld:
    """Simulator-independent world & location-gated sensor host.

    Maintains evaluator-private soil/stand profiles plus continuous 2D terrain fields
    (`self.continuous_terrain`) and emits local `SoilSample` readings strictly when a
    rover's physical pose enters `probe_radius_m` of a permitted site. Supports
    configurable measurement noise/bias (`probe_noise_std`, `probe_bias`) and sensor
    dropout windows (`sensor_degraded_window_s`) where readings are marked `valid=False`.
    """

    def __init__(
        self,
        sites: tuple[Site, ...] = DEFAULT_SITES,
        probe_radius_m: float = 0.28,
        hidden_truth: dict[str, dict[SensorKind, float]] | None = None,
        probe_noise_std: float = 0.0,
        probe_bias: float = 0.0,
        sensor_degraded_window_s: tuple[float, float] | None = None,
    ) -> None:
        if probe_radius_m <= 0:
            raise ValueError("probe_radius_m must be positive")
        if probe_noise_std < 0:
            raise ValueError("probe_noise_std must be non-negative")
        self.sites = {s.site_id: s for s in sites}
        self.probe_radius_m = probe_radius_m
        self.probe_noise_std = float(probe_noise_std)
        self.probe_bias = float(probe_bias)
        self.sensor_degraded_window_s = sensor_degraded_window_s
        self._hidden_site_truth = dict(hidden_truth or DEFAULT_HIDDEN_SOIL_TRUTH)
        self._seed: int = 7
        self.continuous_terrain = ContinuousPaddyTerrain(
            sites=sites, anchor_truth=self._hidden_site_truth, seed=self._seed
        )
        self._sampled_pairs: set[tuple[str, str]] = set()
        self._invalid_Attempt_counts: dict[tuple[str, str], int] = {}

    def reset(self, seed: int) -> None:
        """Reset episode sampling state and continuous terrain seed."""
        self._seed = int(seed)
        self.continuous_terrain = ContinuousPaddyTerrain(
            sites=tuple(self.sites.values()),
            anchor_truth=self._hidden_site_truth,
            seed=self._seed,
        )
        self._sampled_pairs.clear()
        self._invalid_Attempt_counts.clear()

    def is_sensor_degraded_at(self, time_s: float) -> bool:
        """Return True if local probes/rangefinders are inside a configured sensor dropout window."""
        if self.sensor_degraded_window_s is None:
            return False
        t0, t1 = self.sensor_degraded_window_s
        return bool(t0 <= time_s <= t1)

    def evaluate_continuous_point(self, x_m: float, y_m: float) -> ContinuousTerrainPoint:
        """Evaluate continuous elevation, slope, mud depth, moisture, firmness, and plant density at `(x_m, y_m)`."""
        return self.continuous_terrain.evaluate(x_m, y_m)

    def sample_local_soil(
        self, robot_id: str, pose: Pose2D, time_s: float
    ) -> tuple[SoilSample, ...]:
        """Return local soil/stand samples if and only if `pose` is within `probe_radius_m`.

        - During an active `sensor_degraded_window_s`, emits `valid=False` diagnostic samples
          without locking `_sampled_pairs` so the rover can acquire valid measurements once
          sensor health recovers.
        - When `probe_noise_std > 0` or `probe_bias != 0`, applies deterministic reproducible
          measurement perturbation clamped to `[0.0, 1.0]`.
        """
        samples: list[SoilSample] = []
        degraded = self.is_sensor_degraded_at(time_s)
        for site_id, site in self.sites.items():
            if not site.permitted or site_id not in self._hidden_site_truth:
                continue
            if (robot_id, site_id) in self._sampled_pairs:
                continue
            dist = hypot(pose.x_m - site.center.x_m, pose.y_m - site.center.y_m)
            if dist <= self.probe_radius_m:
                truth = self._hidden_site_truth[site_id]
                if degraded:
                    attempt_idx = self._invalid_Attempt_counts.get((robot_id, site_id), 0) + 1
                    self._invalid_Attempt_counts[(robot_id, site_id)] = attempt_idx
                    if attempt_idx > 1:
                        continue
                    for kind in (SensorKind.MOISTURE, SensorKind.FIRMNESS, SensorKind.STAND_GAP):
                        samples.append(
                            SoilSample(
                                sample_id=f"{robot_id}:{site_id}:{kind.value}:invalid:{attempt_idx}",
                                robot_id=robot_id,
                                site_id=site_id,
                                kind=kind,
                                value=0.5,
                                sampled_at_s=time_s,
                                location=pose,
                                valid=False,
                            )
                        )
                    continue

                self._sampled_pairs.add((robot_id, site_id))
                for kind_idx, kind in enumerate(
                    (SensorKind.MOISTURE, SensorKind.FIRMNESS, SensorKind.STAND_GAP)
                ):
                    raw_val = truth[kind]
                    if self.probe_noise_std > 0.0 or self.probe_bias != 0.0:
                        rng = random.Random(
                            f"{self._seed}:{robot_id}:{site_id}:{kind.value}:{kind_idx}"
                        )
                        noisy = raw_val + self.probe_bias + rng.gauss(0.0, self.probe_noise_std)
                        measured_val = round(max(0.0, min(1.0, noisy)), 4)
                    else:
                        measured_val = raw_val
                    samples.append(
                        SoilSample(
                            sample_id=f"{robot_id}:{site_id}:{kind.value}",
                            robot_id=robot_id,
                            site_id=site_id,
                            kind=kind,
                            value=measured_val,
                            sampled_at_s=time_s,
                            location=pose,
                            valid=True,
                        )
                    )
        return tuple(samples)

    def mud_traction_scale(self, pose: Pose2D) -> float:
        """Return wheel-velocity traction factor in `(0.35, 1.0]` based on continuous local mud softness."""
        pt = self.continuous_terrain.evaluate(pose.x_m, pose.y_m)
        if pt.moisture > 0.80 and pt.firmness < 0.25:
            return 0.36
        scale = (
            1.0
            - 0.45 * max(0.0, pt.moisture - 0.55) * (1.0 - pt.firmness)
            - 0.25 * pt.slope_magnitude
        )
        return round(max(0.36, min(1.0, scale)), 4)


_LEAF_PALETTE: tuple[str, ...] = (
    "0.22 0.56 0.16 1",
    "0.28 0.64 0.19 1",
    "0.17 0.48 0.14 1",
    "0.33 0.68 0.22 1",
    "0.25 0.59 0.17 1",
    "0.38 0.62 0.18 1",
)


def _build_grasslike_rice_tuft_xml(
    name_prefix: str,
    cx: float,
    cy: float,
    *,
    num_blades: int = 8,
    height_m: float = 0.19,
    spread_m: float = 0.055,
    seed_offset: int = 0,
    include_First3_legacy_names: bool = False,
) -> list[str]:
    """Generate a realistic grasslike rice hill (`Oryza sativa` tillering tuft) with thin arching leaves.

    - Scaled to Agent B's real `27.0 cm × 27.6 cm` rover (`z_hull_top = 10.5 cm`, `z_cam = 12.8 cm`):
      rice leaf blades are `2.0–3.0 mm` radius (`4–6 mm` width) and arch naturally to `13–24 cm`
      above the mud/water surface.
    - Uses `contype="0" conaffinity="0" group="2"` so every blade renders in all cameras and viewer
      windows (`MjvOption.geomgroup` enables `0, 1, 2` by default) while NEVER creating invisible
      physical walls or false LiDAR `<rangefinder>` obstacle hits.
    """
    geoms: list[str] = []
    # Subtle muddy root-crown base at soil/water interface
    geoms.append(
        f'<geom name="{name_prefix}_crown" type="cylinder" pos="{cx:.3f} {cy:.3f} 0.005" '
        f'size="0.014 0.004" contype="0" conaffinity="0" group="2" rgba="0.24 0.34 0.16 0.92"/>'
    )
    suffixes = ("a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k")
    for b_idx in range(num_blades):
        sfx = suffixes[b_idx] if b_idx < len(suffixes) else str(b_idx)
        gname = f"{name_prefix}_{sfx}" if (include_First3_legacy_names or b_idx >= 3) else f"{name_prefix}_{sfx}"
        angle = (2.0 * pi * b_idx / max(1, num_blades)) + 0.23 * ((seed_offset + b_idx) % 5)
        ring_frac = 0.55 + 0.45 * ((b_idx + seed_offset) % 3) / 2.0
        bx0 = cx + 0.005 * cos(angle)
        by0 = cy + 0.005 * sin(angle)
        mid_r = spread_m * 0.48 * ring_frac
        tip_r = spread_m * (0.92 + 0.18 * ((b_idx * 3 + seed_offset) % 3))
        bx_mid = cx + mid_r * cos(angle)
        by_mid = cy + mid_r * sin(angle)
        bx_tip = cx + tip_r * cos(angle + 0.12)
        by_tip = cy + tip_r * sin(angle + 0.12)

        h_var = height_m * (0.78 + 0.11 * ((b_idx + seed_offset) % 4))
        z_mid = h_var * 0.62
        # Outer blades arch slightly downward near the tip like real rice leaves
        z_tip = h_var * (0.98 if b_idx % 2 == 0 else 0.86)
        blade_r = 0.0026 if b_idx < 3 else 0.0021
        color = _LEAF_PALETTE[(b_idx + seed_offset) % len(_LEAF_PALETTE)]

        # Lower stem / leaf sheath segment (keeps legacy `rice_nw_1_a..c` geom names for unit tests)
        geoms.append(
            f'<geom name="{gname}" type="capsule" '
            f'fromto="{bx0:.3f} {by0:.3f} 0.006 {bx_mid:.3f} {by_mid:.3f} {z_mid:.3f}" '
            f'size="{blade_r:.4f}" contype="0" conaffinity="0" group="2" rgba="{color}"/>'
        )
        # Upper arching thin leaf blade segment
        tip_color = _LEAF_PALETTE[(b_idx + seed_offset + 1) % len(_LEAF_PALETTE)]
        geoms.append(
            f'<geom name="{gname}_tip" type="capsule" '
            f'fromto="{bx_mid:.3f} {by_mid:.3f} {z_mid:.3f} {bx_tip:.3f} {by_tip:.3f} {z_tip:.3f}" '
            f'size="{blade_r * 0.76:.4f}" contype="0" conaffinity="0" group="2" rgba="{tip_color}"/>'
        )
    return geoms


def _build_seedling_clumps_xml() -> str:
    """Generate realistic grasslike rice clumps across dense stands, irregular crop rows, and surrounding plot context."""
    lines: list[str] = []

    # 1. Dense protected living rice stands: crop_nw (0.85, 0.45) & crop_ne (2.35, 0.45)
    # First 9 hills preserve `rice_nw_1..9_a..c` and `rice_ne_1..9_a..c` names with natural row jitter
    jitter_offsets: tuple[tuple[float, float], ...] = (
        (-0.018, 0.012),
        (0.014, -0.015),
        (-0.012, -0.010),
        (0.019, 0.014),
        (-0.008, 0.016),
        (0.015, -0.012),
        (-0.016, 0.011),
        (0.012, 0.018),
        (0.016, -0.014),
    )
    for prefix, (cx, cy) in (("nw", (0.85, 0.45)), ("ne", (2.35, 0.45))):
        idx = 0
        for dx in (-0.15, 0.0, 0.15):
            for dy in (-0.15, 0.0, 0.15):
                jx, jy = jitter_offsets[idx % len(jitter_offsets)]
                idx += 1
                x, y = cx + dx + jx, cy + dy + jy
                lines.extend(
                    _build_grasslike_rice_tuft_xml(
                        f"rice_{prefix}_{idx}",
                        x,
                        y,
                        num_blades=8,
                        height_m=0.20,
                        spread_m=0.062,
                        seed_offset=idx,
                        include_First3_legacy_names=True,
                    )
                )
        # Additional irregular filler hills inside each dense stand so canopy looks lush & natural
        extra_stand_offsets = (
            (-0.08, -0.08, 0.22),
            (0.08, -0.07, 0.18),
            (-0.07, 0.08, 0.21),
            (0.09, 0.08, 0.19),
            (-0.21, -0.05, 0.17),
            (0.21, 0.05, 0.18),
            (0.02, 0.21, 0.17),
        )
        for e_i, (edx, edy, eh) in enumerate(extra_stand_offsets, start=10):
            lines.extend(
                _build_grasslike_rice_tuft_xml(
                    f"rice_{prefix}_{e_i}",
                    cx + edx,
                    cy + edy,
                    num_blades=7,
                    height_m=eh,
                    spread_m=0.056,
                    seed_offset=e_i + 3,
                )
            )

    # 2. Irregular but recognisable crop rows & sparse surviving tufts inside the working plot
    # (Placed outside rover transit lanes and outside the 0.22m planting drop centers of patch_a..d)
    field_row_tufts: tuple[tuple[str, float, float, float, int], ...] = (
        # Northern paddy margin crop row (y ≈ +0.81 m)
        ("row_n_01", -0.08, 0.81, 0.18, 6),
        ("row_n_02", 0.14, 0.79, 0.19, 6),
        ("row_n_03", 0.36, 0.82, 0.17, 6),
        ("row_n_04", 0.58, 0.80, 0.20, 7),
        ("row_n_05", 1.16, 0.81, 0.18, 6),
        ("row_n_06", 1.38, 0.79, 0.16, 6),
        ("row_n_07", 1.82, 0.80, 0.17, 6),
        ("row_n_08", 2.04, 0.82, 0.19, 6),
        ("row_n_09", 2.64, 0.80, 0.20, 7),
        ("row_n_10", 2.84, 0.78, 0.18, 6),
        # Sparse surviving hills flanking northern stand-gap patch_d (1.60, 0.45)
        ("gap_d_w", 1.24, 0.52, 0.15, 6),
        ("gap_d_n", 1.58, 0.75, 0.14, 5),
        ("gap_d_e", 1.96, 0.50, 0.16, 6),
        # Southern paddy margin crop row (y ≈ -0.82 m, south of patch_a..c)
        ("row_s_01", -0.06, -0.81, 0.18, 6),
        ("row_s_02", 0.18, -0.83, 0.19, 6),
        ("row_s_03", 0.42, -0.80, 0.17, 6),
        ("row_s_04", 0.64, -0.83, 0.16, 6),
        ("row_s_05", 1.08, -0.82, 0.18, 6),
        ("row_s_06", 1.26, -0.80, 0.19, 7),
        ("row_s_07", 1.44, -0.83, 0.15, 5),
        ("row_s_08", 1.78, -0.82, 0.16, 6),
        ("row_s_09", 1.98, -0.80, 0.19, 7),
        ("row_s_10", 2.16, -0.83, 0.17, 6),
        ("row_s_11", 2.58, -0.81, 0.18, 6),
        ("row_s_12", 2.78, -0.82, 0.20, 7),
        # Surviving inter-patch rice hills along the southern crop strip (between patch_a, patch_b, patch_c)
        ("inter_s_01", 0.44, -0.58, 0.16, 6),
        ("inter_s_02", 1.21, -0.62, 0.15, 6),
        ("inter_s_03", 1.97, -0.61, 0.16, 6),
        ("inter_s_04", 2.72, -0.54, 0.18, 7),
        ("inter_s_05", 2.82, -0.34, 0.17, 6),
        # East edge row continuation (x ≈ 2.82 m)
        ("row_e_01", 2.82, 0.12, 0.18, 6),
        ("row_e_02", 2.84, 0.34, 0.19, 6),
        ("row_e_03", 2.80, 0.56, 0.18, 6),
    )
    for t_idx, (tname, tx, ty, th, n_b) in enumerate(field_row_tufts):
        lines.extend(
            _build_grasslike_rice_tuft_xml(
                f"rice_{tname}",
                tx,
                ty,
                num_blades=n_b,
                height_m=th,
                spread_m=0.052,
                seed_offset=t_idx + 11,
            )
        )

    # 3. Adjacent level field plots beyond the low 8.5 cm perimeter bunds
    # Makes the operational plot look like one working section of a larger rice field
    outer_context_tufts: list[tuple[str, float, float, float]] = []
    for i, ox in enumerate((-0.10, 0.35, 0.80, 1.25, 1.70, 2.15, 2.60, 3.05)):
        outer_context_tufts.append((f"outer_n1_{i}", ox + 0.02 * (i % 2), 1.32, 0.20))
        outer_context_tufts.append((f"outer_n2_{i}", ox - 0.02 * (i % 2), 1.58, 0.21))
        outer_context_tufts.append((f"outer_s1_{i}", ox + 0.02 * (i % 2), -1.32, 0.19))
        outer_context_tufts.append((f"outer_s2_{i}", ox - 0.02 * (i % 2), -1.58, 0.21))
    for j, oy in enumerate((-0.75, -0.35, 0.05, 0.45, 0.82)):
        outer_context_tufts.append((f"outer_e1_{j}", 3.42, oy, 0.20))
        outer_context_tufts.append((f"outer_e2_{j}", 3.72, oy + 0.03, 0.21))

    for o_idx, (oname, ox, oy, oh) in enumerate(outer_context_tufts):
        lines.extend(
            _build_grasslike_rice_tuft_xml(
                f"rice_{oname}",
                ox,
                oy,
                num_blades=5,
                height_m=oh,
                spread_m=0.058,
                seed_offset=o_idx + 29,
            )
        )

    return "\n    ".join(lines)


def _build_patch_beds_xml(
    hidden_truth: dict[str, dict[SensorKind, float]] | None = None,
) -> str:
    """Build visual terrain beds for `patch_a..d` aligned with scenario soil truth (Audit N1).

    Note: Visual materials reflect wet mud vs consolidated loam for camera realism only;
    actual traction/bogging is strictly governed by `ContinuousPaddyTerrain` and
    Bekker-Wong terramechanics, never inferred from texture colors.
    """
    truth_map = hidden_truth or DEFAULT_HIDDEN_SOIL_TRUTH
    patch_coords: tuple[tuple[str, float, float], ...] = (
        ("patch_a", 0.85, -0.50),
        ("patch_b", 1.60, -0.50),
        ("patch_c", 2.35, -0.50),
        ("patch_d", 1.60, 0.45),
    )
    lines: list[str] = []
    for pid, px, py in patch_coords:
        site_truth = truth_map.get(pid, DEFAULT_HIDDEN_SOIL_TRUTH[pid])
        is_mud_hazard = (
            site_truth.get(SensorKind.MOISTURE, 0.5) > 0.80
            or site_truth.get(SensorKind.FIRMNESS, 0.6) < 0.25
        )
        if is_mud_hazard:
            lines.append(
                f'<geom name="bed_{pid}" type="cylinder" pos="{px:.2f} {py:.2f} 0.0025" '
                f'size="0.26 0.0025" material="mud_hazard_mat" contype="0" conaffinity="0" group="2"/>'
            )
            lines.append(
                f'<geom name="puddle_{pid}" type="cylinder" pos="{px:.2f} {py:.2f} 0.0045" '
                f'size="0.19 0.0018" material="canal_water_mat" contype="0" conaffinity="0" group="2"/>'
            )
        else:
            lines.append(
                f'<geom name="bed_{pid}" type="cylinder" pos="{px:.2f} {py:.2f} 0.0025" '
                f'size="0.26 0.0025" material="viable_loam_mat" contype="0" conaffinity="0" group="2"/>'
            )
            lines.append(
                f'<site name="ring_{pid}" type="cylinder" pos="{px:.2f} {py:.2f} 0.0055" '
                f'size="0.16 0.0018" rgba="0.78 0.68 0.30 0.45"/>'
            )
    return "\n    ".join(lines)


def build_world_terrain_xml(
    obstacle_xy: tuple[float, float] = (0.75, -0.25),
    hidden_truth: dict[str, dict[SensorKind, float]] | None = None,
) -> str:
    """Return MJCF `<worldbody>` fragment for the level bunded rice field, patches, grasslike crops, and obstacle."""
    ox, oy = obstacle_xy
    patch_beds_xml = _build_patch_beds_xml(hidden_truth=hidden_truth)
    return f"""
    <light name="sun_main" pos="1.4 -1.2 3.8" dir="-0.12 0.22 -1" diffuse="0.88 0.85 0.78" specular="0.30 0.30 0.28" castshadow="true"/>
    <light name="sky_fill" pos="1.5 1.6 3.0" dir="0 -0.28 -1" diffuse="0.38 0.42 0.46" castshadow="false"/>

    <!-- Overview & Rover-Scale Agricultural Cameras -->
    <camera name="terrace_iso" pos="-0.68 -1.92 1.46" xyaxes="0.82 -0.57 0 0.31 0.45 0.84"/>
    <camera name="top_overview" pos="1.35 -0.02 3.05" xyaxes="1 0 0 0 1 0"/>
    <camera name="rover_closeup" pos="-0.24 -0.58 0.25" xyaxes="0.48 -0.88 0 0.19 0.10 0.98"/>
    <camera name="field_eye_level" pos="1.15 -1.28 0.32" xyaxes="0.96 0.28 0 -0.06 0.20 0.98"/>

    <!-- Level Puddled Paddy Ground Plane (z = 0.0 m, matching ContinuousPaddyTerrain) -->
    <geom name="field_ground" type="plane" size="6 6 0.1" material="paddy_soil_mat" contype="1" conaffinity="7" friction="1.2 0.01 0.005"/>

    <!-- Invisible Group-4 Far Horizon Ring (4.5 m radius) so valid clear LiDAR rays return ~4.0 m while invalid rays return < 0 -> None -->
    <geom name="lidar_horizon_ring" type="cylinder" pos="1.35 0.00 0.11" size="4.50 0.09" contype="0" conaffinity="0" group="4" rgba="0 0 0 0"/>

    <!-- Shallow Standing Water Glint Sheets in Low Micro-Basins (visual only, group 2) -->
    <geom name="shallow_water_sw" type="box" pos="0.72 -0.46 0.0015" size="0.46 0.32 0.001" material="shallow_water_mat" contype="0" conaffinity="0" group="2"/>
    <geom name="shallow_water_nc" type="box" pos="1.58 0.44 0.0015" size="0.44 0.30 0.001" material="shallow_water_mat" contype="0" conaffinity="0" group="2"/>
    <geom name="shallow_water_ne_ furrow" type="box" pos="2.10 0.12 0.0012" size="0.62 0.14 0.001" material="shallow_water_mat" contype="0" conaffinity="0" group="2"/>

    <!-- Low Compacted Earthen Paddy Bunds / Field Ridges (8.5 cm total height, scaled to 27 cm rover) -->
    <geom name="bund_north" type="box" pos="1.35 1.05 0.042" size="1.85 0.11 0.042" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="bund_south" type="box" pos="1.35 -1.05 0.042" size="1.85 0.11 0.042" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="bund_west" type="box" pos="-0.42 0.00 0.042" size="0.11 1.16 0.042" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="bund_east" type="box" pos="3.12 0.00 0.042" size="0.11 1.16 0.042" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- Shallow Irrigation Ditch & Adjacent Level Field Continuation Strips -->
    <geom name="irrigation_canal_water" type="box" pos="-0.70 0.00 0.004" size="0.15 1.85 0.004" material="canal_water_mat" contype="0" conaffinity="0" group="2"/>
    <geom name="outer_water_north" type="box" pos="1.45 1.45 0.0015" size="1.80 0.24 0.001" material="shallow_water_mat" contype="0" conaffinity="0" group="2"/>
    <geom name="outer_water_south" type="box" pos="1.45 -1.45 0.0015" size="1.80 0.24 0.001" material="shallow_water_mat" contype="0" conaffinity="0" group="2"/>

    <!-- Subtle Puddled Row Wheel-Track Furrows across the Operational Plot -->
    <geom name="furrow_1" type="box" pos="1.48 -0.74 0.0015" size="1.42 0.014 0.0015" rgba="0.24 0.19 0.13 0.52" contype="0" conaffinity="0" group="2"/>
    <geom name="furrow_2" type="box" pos="1.48 -0.26 0.0015" size="1.42 0.014 0.0015" rgba="0.24 0.19 0.13 0.52" contype="0" conaffinity="0" group="2"/>
    <geom name="furrow_3" type="box" pos="1.48 0.18 0.0015" size="1.42 0.014 0.0015" rgba="0.24 0.19 0.13 0.52" contype="0" conaffinity="0" group="2"/>
    <geom name="furrow_4" type="box" pos="1.48 0.68 0.0015" size="1.42 0.014 0.0015" rgba="0.24 0.19 0.13 0.52" contype="0" conaffinity="0" group="2"/>

    <!-- Field Patch Beds (Mud Hazard vs Viable Stand-Gap Loam vs Living Rice Stand) -->
    {patch_beds_xml}

    <geom name="bed_crop_nw" type="cylinder" pos="0.85 0.45 0.003" size="0.28 0.003" material="crop_bed_mat" contype="0" conaffinity="0" group="2"/>
    <geom name="bed_crop_ne" type="cylinder" pos="2.35 0.45 0.003" size="0.28 0.003" material="crop_bed_mat" contype="0" conaffinity="0" group="2"/>
    {_build_seedling_clumps_xml()}

    <!-- Mid-episode obstacle (fallen irrigation crate parked below ground until surprise injection) -->
    <body name="mid_run_obstacle" mocap="true" pos="{ox:.3f} {oy:.3f} -5.0">
      <geom name="obstacle_geom" type="box" size="0.11 0.14 0.14" contype="5" conaffinity="7" rgba="0.82 0.28 0.16 1"/>
    </body>
"""
