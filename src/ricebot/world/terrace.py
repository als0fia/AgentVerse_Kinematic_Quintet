"""World and local soil-sensor host for PaddyPatch (`src/ricebot/world/`).

Strict separation per `docs/COMPONENTS.md`:
- Owns field sites (`DEFAULT_SITES`), public traversability/no-go flags, hidden synthetic
  spatial soil/stand distribution, continuous 2D terrain functions (`ContinuousPaddyTerrain`),
  location-gated soil probes, and world terrain geometry.
- NEVER owns rover chassis, wheel joints, skid-steer kinematics, seed hopper geometry,
  or mission planning.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import cos, exp, hypot, sin
import random
from typing import Any

import numpy as np

from ..contracts import Pose2D, SensorKind, Site, SoilSample


# 6-Patch Paddy Field Layout (2 rows x 3 columns inside a 3.2m x 2.2m bunded terrace)
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
    """Agronomic Wet Direct-Seeded Rice (WDSR) fitness index Φ(x, y) in [0.0, 1.0].

    Grounded in puddled paddy soil mechanics & WDSR establishment research:
    - Moisture suitability S_M: bell curve centered at glistening saturation M* = 0.56 (σ = 0.14).
      Penalizes dry surface crusting (M < 0.35, radicle dehydration) and deep waterlogged pooling
      (M > 0.80, anaerobic hypoxia / seed rotting).
    - Firmness suitability S_F: bell curve centered at consolidated 'mud-cake' bearing firmness
      F* = 0.65 (σ = 0.16, CI ~ 150-260 kPa). Penalizes soupy fluid mud (F < 0.32, seed burial)
      and over-settled hard crust (F > 0.88, poor root anchoring).
    - Micro-topographic penalty S_z: penalizes deep micro-depressions (z < -15 mm) where water pools.
    - Canopy stand-gap opportunity S_G: requires open canopy gap (G >= 0.55) so rovers never
      over-seed or trample dense established rice stands.
    """
    s_moisture = exp(-((moisture - 0.56) ** 2) / (2.0 * 0.14 * 0.14))
    s_firmness = exp(-((firmness - 0.65) ** 2) / (2.0 * 0.16 * 0.16))
    s_micro_topo = exp(-((min(0.0, elevation_m + 0.008)) ** 2) / (2.0 * 0.018 * 0.018))
    s_gap = max(0.0, min(1.0, (stand_gap - 0.22) / 0.66)) ** 1.25
    fitness = s_gap * (0.46 * s_moisture + 0.42 * s_firmness + 0.12 * s_micro_topo)
    return round(max(0.0, min(1.0, fitness)), 4)


@dataclass(frozen=True)
class ContinuousTerrainPoint:
    """Continuous spatial field evaluation at any `(x_m, y_m)` coordinate in the paddy terrace."""

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
    r"""Continuous 2D analytic terrain & soil-mechanics field $f(x, y)$ over the paddy terrace.

    Replaces purely discrete point lookups with smooth continuous 2D functions (Gaussian RBF kernels
    + sinusoidal terrace micro-topography + perimeter bund basins) for:
    - **Elevation & Hilliness $z(x, y)$** and slope gradient $\nabla z(x, y) = (\partial z/\partial x, \partial z/\partial y)$
    - **Volumetric Soil Moisture $M(x, y) \in [0, 1]$** and **Cone-Index Firmness $F(x, y) \in [0, 1]$**
    - **Soft Mud Layer Depth $h_{\text{mud}}(x, y)$** above the consolidated plow pan (`0.04 m` to `0.34 m`)
    - **Vegetation / Rice Stand Density $\rho_{\text{plant}}(x, y) \in [0, 1]$** and **Stand Gap $G(x, y) = 1 - \rho_{\text{plant}}(x, y)$**
    - **Bekker–Wong Terramechanics Soil Parameters** ($k_c, k_\phi, n, c, \phi$) derived continuously from $(M(x,y), F(x,y))$
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
        self._grid_cache: dict[tuple[int, int, tuple[float, float], tuple[float, float], int], dict[str, Any]] = {}

    def elevation_and_gradient(self, x_m: float, y_m: float) -> tuple[float, float, float]:
        """Return `(elevation_m, dz_dx, dz_dy)` at `(x_m, y_m)` in metres.

        Models stepped Chinese rice terraces with exact 1.0m height difference between levels:
        - Level 0 (Valley):        x < -0.40 -> z ≈ -1.0m
        - Level 1 (Operational):  -0.40 <= x < 3.40 -> z ≈ 0.0m
        - Level 2 (Mid-Mountain):  3.40 <= x < 8.40 -> z ≈ +1.0m
        - Level 3 (Upper-Mountain): 8.40 <= x < 13.40 -> z ≈ +2.0m
        - Tier 4 (High Ridge):    x >= 13.40 -> z ≈ +3.0m

        Transitions between levels occur exclusively via dedicated fixed ramps:
        - Ramp 0: Level 0 -> Level 1 at South corridor (y ≈ -0.70, x in [-2.4, -0.4])
        - Ramp 1: Level 1 -> Level 2 at North corridor (y ≈ +0.70, x in [3.4, 5.4])
        - Ramp 2: Level 2 -> Level 3 at South corridor (y ≈ -0.70, x in [8.4, 10.4])
        - Ramp 3: Level 3 -> Tier 4   at North corridor (y ≈ +0.70, x in [13.4, 15.4])
        Off-ramp boundaries form steep impassable cliff retaining walls.
        """
        # Smooth ramp factor: 1.0 inside corridor, decays sharply outside
        r0 = exp(-(((y_m - (-0.70)) / 0.28) ** 4))
        r1 = exp(-(((y_m - 0.70) / 0.28) ** 4))
        r2 = exp(-(((y_m - (-0.70)) / 0.28) ** 4))
        r3 = exp(-(((y_m - 0.70) / 0.28) ** 4))

        w0 = 0.04 + (0.95 - 0.04) * r0
        w1 = 0.04 + (0.95 - 0.04) * r1
        w2 = 0.04 + (0.95 - 0.04) * r2
        w3 = 0.04 + (0.95 - 0.04) * r3

        u0 = (x_m - (-1.40)) / w0
        u1 = (x_m - 4.40) / w1
        u2 = (x_m - 9.40) / w2
        u3 = (x_m - 14.40) / w3

        s0 = 0.5 * (1.0 + np.tanh(np.clip(u0, -12.0, 12.0)))
        s1 = 0.5 * (1.0 + np.tanh(np.clip(u1, -12.0, 12.0)))
        s2 = 0.5 * (1.0 + np.tanh(np.clip(u2, -12.0, 12.0)))
        s3 = 0.5 * (1.0 + np.tanh(np.clip(u3, -12.0, 12.0)))

        # Staircase baseline: -1.0m -> 0.0m -> +1.0m -> +2.0m -> +3.0m
        z = -1.0 + 1.0 * s0 + 1.0 * s1 + 1.0 * s2 + 1.0 * s3

        # Micro-relief undulations across the terrace ground
        phase = 0.17 * (self.seed % 11)
        z += (
            0.008 * sin(1.35 * x_m + phase) * cos(1.65 * y_m)
            + 0.004 * cos(2.40 * x_m - 1.90 * y_m)
        )

        # Gradient evaluation
        sech0 = 1.0 / np.cosh(np.clip(u0, -12.0, 12.0))
        sech1 = 1.0 / np.cosh(np.clip(u1, -12.0, 12.0))
        sech2 = 1.0 / np.cosh(np.clip(u2, -12.0, 12.0))
        sech3 = 1.0 / np.cosh(np.clip(u3, -12.0, 12.0))
        dz_dx = (
            0.5 * (1.0 / w0) * (sech0 * sech0)
            + 0.5 * (1.0 / w1) * (sech1 * sech1)
            + 0.5 * (1.0 / w2) * (sech2 * sech2)
            + 0.5 * (1.0 / w3) * (sech3 * sech3)
        )
        dz_dy = 0.0

        # Depressed waterlogged Lannitian mud basins around high-moisture anchors
        for site in self._sites:
            truth = self._anchors.get(site.site_id)
            if truth and truth[SensorKind.MOISTURE] > 0.80:
                dx = x_m - site.center.x_m
                dy = y_m - site.center.y_m
                r2 = dx * dx + dy * dy
                sigma2 = 0.42 * 0.42
                w = exp(-r2 / (2.0 * sigma2))
                basin_depth = -0.028 * w
                z += basin_depth
                dz_dx += basin_depth * (-dx / sigma2)
                dz_dy += basin_depth * (-dy / sigma2)

        return (round(float(z), 5), round(float(dz_dx), 5), round(float(dz_dy), 5))

    def evaluate(self, x_m: float, y_m: float) -> ContinuousTerrainPoint:
        """Evaluate all continuous terrain, soil, and vegetation fields at `(x_m, y_m)`."""
        z, dz_dx, dz_dy = self.elevation_and_gradient(x_m, y_m)
        slope_mag = hypot(dz_dx, dz_dy)

        # Baseline background field values
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
                # Protected established rice hill clump: dense plants (low gap), moderate loam moisture
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
        # so all existing deterministic unit tests and contract checks remain 100% compatible.
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
        # Soft mud depth above the hardpan (m): ranges from 0.04m on firm soil up to 0.34m in Lannitian mud traps
        mud_depth_m = max(0.03, min(0.35, 0.04 + 0.30 * (moisture**1.6) * ((1.0 - firmness) ** 1.2)))

        # Bekker-Wong pressure-sinkage & Mohr-Coulomb shear parameters as continuous functions of moisture/firmness
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
            0.012 * np.sin(1.35 * X + phase_z) * np.cos(1.65 * Y)
            + 0.008 * np.cos(2.40 * X - 1.90 * Y)
            - 0.005 * (X - 1.50)
        )
        sigma2_basin = 0.42 * 0.42
        for site in self._sites:
            truth = self._anchors.get(site.site_id)
            if truth and truth[SensorKind.MOISTURE] > 0.80:
                r2 = (X - site.center.x_m) ** 2 + (Y - site.center.y_m) ** 2
                Z = Z - 0.028 * np.exp(-r2 / (2.0 * sigma2_basin))

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
                    M = np.where(mask, (1.0 - blend) * truth[SensorKind.MOISTURE] + blend * M, M)
                    F = np.where(mask, (1.0 - blend) * truth[SensorKind.FIRMNESS] + blend * F, F)
                    G = np.where(mask, (1.0 - blend) * truth[SensorKind.STAND_GAP] + blend * G, G)

        P = np.clip(1.0 - G, 0.0, 1.0)
        D = np.clip(0.04 + 0.30 * (M**1.6) * ((1.0 - F) ** 1.2), 0.03, 0.35)

        s_moisture = np.exp(-((M - 0.56) ** 2) / (2.0 * 0.14 * 0.14))
        s_firmness = np.exp(-((F - 0.65) ** 2) / (2.0 * 0.16 * 0.16))
        s_micro_topo = np.exp(-((np.minimum(0.0, Z + 0.008)) ** 2) / (2.0 * 0.018 * 0.018))
        s_gap = np.clip((G - 0.22) / 0.66, 0.0, 1.0) ** 1.25
        FIT = np.clip(s_gap * (0.46 * s_moisture + 0.42 * s_firmness + 0.12 * s_micro_topo), 0.0, 1.0)

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

    Maintains hidden ground-truth soil/stand profiles plus continuous 2D terrain fields
    (`self.continuous_terrain`) and emits local `SoilSample` readings strictly when a
    rover's physical pose enters `probe_radius_m` of a permitted site. Used identically
    by both MuJoCo and Isaac Sim adapters.
    """

    def __init__(
        self,
        sites: tuple[Site, ...] = DEFAULT_SITES,
        probe_radius_m: float = 0.28,
        hidden_truth: dict[str, dict[SensorKind, float]] | None = None,
    ) -> None:
        if probe_radius_m <= 0:
            raise ValueError("probe_radius_m must be positive")
        self.sites = {s.site_id: s for s in sites}
        self.probe_radius_m = probe_radius_m
        self._hidden_site_truth = dict(hidden_truth or DEFAULT_HIDDEN_SOIL_TRUTH)
        self.continuous_terrain = ContinuousPaddyTerrain(
            sites=sites, anchor_truth=self._hidden_site_truth, seed=7
        )
        self._sampled_pairs: set[tuple[str, str]] = set()

    def reset(self, seed: int) -> None:
        """Reset episode sampling state and continuous terrain seed."""
        self.continuous_terrain = ContinuousPaddyTerrain(
            sites=tuple(self.sites.values()),
            anchor_truth=self._hidden_site_truth,
            seed=seed,
        )
        self._sampled_pairs.clear()

    def evaluate_continuous_point(self, x_m: float, y_m: float) -> ContinuousTerrainPoint:
        """Evaluate continuous elevation, slope, mud depth, moisture, firmness, and plant density at `(x_m, y_m)`."""
        return self.continuous_terrain.evaluate(x_m, y_m)

    def sample_local_soil(
        self,
        robot_id: str,
        pose: Pose2D,
        time_s: float,
        noise_std: float = 0.0,
        scenario: str = "nominal",
    ) -> tuple[SoilSample, ...]:
        """Return local soil/stand samples if and only if `pose` is within `probe_radius_m`.

        Enforces strict sensor locality and epistemic realism:
        - When within probe radius of a permitted site, emits samples.
        - Supports sensor degradation for `sensor_degraded` scenario (Audit N3)
          and measurement noise simulation (Audit M7).
        """
        samples: list[SoilSample] = []
        for site_id, site in self.sites.items():
            if not site.permitted or site_id not in self._hidden_site_truth:
                continue
            if (robot_id, site_id) in self._sampled_pairs:
                continue
            dist = hypot(pose.x_m - site.center.x_m, pose.y_m - site.center.y_m)
            if dist <= self.probe_radius_m:
                self._sampled_pairs.add((robot_id, site_id))
                truth = self._hidden_site_truth[site_id]
                effective_noise = noise_std
                if scenario == "sensor_degraded":
                    effective_noise = max(effective_noise, 0.06)

                rng = random.Random(hash((robot_id, site_id, int(time_s * 100))) & 0xFFFFFFFF)
                for kind in (SensorKind.MOISTURE, SensorKind.FIRMNESS, SensorKind.STAND_GAP):
                    val = truth[kind]
                    if effective_noise > 0.0:
                        val = round(max(0.0, min(1.0, val + rng.gauss(0.0, effective_noise))), 2)
                    samples.append(
                        SoilSample(
                            sample_id=f"{robot_id}:{site_id}:{kind.value}",
                            robot_id=robot_id,
                            site_id=site_id,
                            kind=kind,
                            value=val,
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
        # Smooth continuous traction degradation on wet slopes
        scale = 1.0 - 0.45 * max(0.0, pt.moisture - 0.55) * (1.0 - pt.firmness) - 0.25 * pt.slope_magnitude
        return round(max(0.36, min(1.0, scale)), 4)


def _build_seedling_clumps_xml() -> str:
    """Flush living rice stand markers anchored directly to the paddy bed."""
    return ""


def build_world_terrain_xml(
    obstacle_xy: tuple[float, float] = (0.75, -0.25),
    use_3d_terraces: bool = True,
) -> str:
    """Return MJCF `<worldbody>` fragment for authentic Chinese mountain rice terraces.

    Features:
    - Monumental staircase progression with exact 1.0m height step between consecutive levels
    - Scale disparity: 1/4 m ant-colony swarm rovers vs vast 1.0m terraced mountain steps (4:1)
    - Fixed dedicated ramps ascending from lower level to higher level (e.g. Ramp 0 South, Ramp 1 North, Ramp 2 South)
    - Vertical stacked-stone retaining walls (1.0m drop) and earthen bunds prevent level crossing everywhere except via ramps
    - Fully grounded deep earth foundations down to bedrock floor (z = -2.5m) with zero floating geometry
    - Water mirror reflections, emerald seedling fields, and sinuous golden earthen bunds matching Yuanyang & Longsheng
    """
    ox, oy = obstacle_xy
    return f"""
    <light name="sun_main" pos="6.0 -6.0 12.0" dir="-0.30 0.35 -1.0" diffuse="0.95 0.92 0.84" specular="0.45 0.45 0.40" castshadow="true"/>
    <light name="sky_fill" pos="3.0 4.0 8.0" dir="0 -0.2 -1" diffuse="0.48 0.52 0.58" castshadow="false"/>

    <!-- Wide amphitheatre overview camera capturing ant-scale swarm on vast 3D stepped rice terraces -->
    <camera name="terrace_iso" pos="-2.80 -4.80 4.60" xyaxes="0.86 -0.51 0 0.26 0.44 0.86"/>
    <camera name="staircase_view" pos="-2.20 -4.20 3.80" xyaxes="0.88 -0.47 0 0.22 0.41 0.88"/>
    <camera name="top_overview" pos="3.50 0.00 7.50" xyaxes="1 0 0 0 1 0"/>

    <!-- ========================================================================= -->
    <!-- BEDROCK VALLEY FOUNDATION (Prevents any hollow voids or floating geometry) -->
    <!-- ========================================================================= -->
    <geom name="valley_floor" type="box" pos="8.00 0.00 -2.50" size="18.0 14.0 1.0" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- ========================================================================= -->
    <!-- STAIRCASE RICE TERRACES: EXACT 1.0m HEIGHT DIFFERENCE PER LEVEL            -->
    <!-- Level 0 (z = -1.0m), Level 1 (z = 0.0m), Level 2 (z = +1.0m),              -->
    <!-- Level 3 (z = +2.0m), Tier 4 (z = +3.0m), Tier 5 (z = +4.0m)               -->
    <!-- ========================================================================= -->

    <!-- LEVEL 0: Lower Valley Terrace (Top Surface z = -1.0m) -->
    <geom name="terrace_level0_base" type="box" pos="-3.50 0.00 -1.75" size="1.10 2.20 0.75" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_level0_ground" type="box" pos="-3.50 0.00 -1.01" size="1.10 2.20 0.01" material="paddy_soil_mat" contype="1" conaffinity="7" friction="1.5 0.01 0.005"/>
    <geom name="terrace_level0_water" type="box" pos="-3.50 0.00 -1.005" size="0.95 2.05 0.005" material="canal_water_mat" contype="0" conaffinity="0"/>
    <geom name="terrace_level0_bund_w" type="box" pos="-4.63 0.00 -0.95" size="0.05 2.25 0.06" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_level0_bund_n" type="box" pos="-3.50 2.23 -0.95" size="1.15 0.05 0.06" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_level0_bund_s" type="box" pos="-3.50 -2.23 -0.95" size="1.15 0.05 0.06" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- LEVEL 1: Main Operational Terrace (Top Surface z = 0.0m) -->
    <geom name="terrace_level1_base" type="box" pos="1.50 0.00 -1.25" size="1.90 1.25 1.25" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="field_ground" type="box" pos="1.50 0.00 -0.01" size="1.90 1.25 0.01" material="paddy_soil_mat" contype="1" conaffinity="7" friction="1.5 0.01 0.005"/>
    <!-- Raised Perimeter Earthen Bunds along Level 1 -->
    <geom name="bund_north" type="box" pos="1.50 1.28 0.05" size="1.95 0.05 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="bund_south" type="box" pos="1.50 -1.28 0.05" size="1.95 0.05 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- LEVEL 2: Mid Mountain Terrace (Top Surface z = +1.0m) -->
    <geom name="terrace_level2_base" type="box" pos="6.90 0.00 -0.75" size="1.50 1.35 1.75" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_level2_ground" type="box" pos="6.90 0.00 0.99" size="1.50 1.35 0.01" material="paddy_soil_mat" contype="1" conaffinity="7" friction="1.5 0.01 0.005"/>
    <geom name="terrace_level2_water" type="box" pos="6.90 0.00 0.995" size="1.35 1.20 0.005" material="canal_water_mat" contype="0" conaffinity="0"/>
    <geom name="terrace_level2_bund_n" type="box" pos="6.90 1.38 1.05" size="1.55 0.05 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_level2_bund_s" type="box" pos="6.90 -1.38 1.05" size="1.55 0.05 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- LEVEL 3: Upper Mountain Terrace (Top Surface z = +2.0m) -->
    <geom name="terrace_level3_base" type="box" pos="11.90 0.00 -0.25" size="1.50 1.45 2.25" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_level3_ground" type="box" pos="11.90 0.00 1.99" size="1.50 1.45 0.01" material="paddy_soil_mat" contype="1" conaffinity="7" friction="1.5 0.01 0.005"/>
    <geom name="terrace_level3_water" type="box" pos="11.90 0.00 1.995" size="1.35 1.30 0.005" material="canal_water_mat" contype="0" conaffinity="0"/>
    <geom name="terrace_level3_bund_n" type="box" pos="11.90 1.48 2.05" size="1.55 0.05 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_level3_bund_s" type="box" pos="11.90 -1.48 2.05" size="1.55 0.05 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- TIER 4: High Mountain Ridge Terrace (Top Surface z = +3.0m) -->
    <geom name="terrace_tier4_base" type="box" pos="17.10 0.00 0.25" size="1.70 2.50 2.75" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_tier4_ground" type="box" pos="17.10 0.00 2.99" size="1.70 2.50 0.01" material="paddy_soil_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_tier4_water" type="box" pos="17.10 0.00 2.995" size="1.50 2.20 0.005" material="canal_water_mat" contype="0" conaffinity="0"/>
    <geom name="terrace_tier4_bund_n" type="box" pos="17.10 2.53 3.05" size="1.75 0.05 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_tier4_bund_s" type="box" pos="17.10 -2.53 3.05" size="1.75 0.05 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- TIER 5: Mountain Peak Amphitheatre (Top Surface z = +4.0m) -->
    <geom name="terrace_tier5_wall" type="box" pos="18.80 0.00 3.50" size="0.08 4.00 0.50" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_tier5_base" type="box" pos="21.80 0.00 0.75" size="3.00 4.50 3.25" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_tier5_ground" type="box" pos="21.80 0.00 3.99" size="3.00 4.50 0.01" material="paddy_soil_mat" contype="1" conaffinity="7"/>

    <!-- ========================================================================= -->
    <!-- DEDICATED 40-DEGREE INCLINE FIXED RAMPS BETWEEN LEVELS                    -->
    <!-- Incline angle: 40.0 deg with respect to X-axis, exact 1.0m height rise    -->
    <!-- Half-length along slope = 0.778m (total length 1.556m), half-width = 0.25m-->
    <!-- Switchback path: Ramp 0 (South) -> Ramp 1 (North) -> Ramp 2 (South)       -->
    <!-- ========================================================================= -->

    <!-- RAMP 0: Ascends from Level 0 (z = -1.0m) up to Level 1 (z = 0.0m) at South Corridor (y = -0.70) -->
    <geom name="ramp_0_center" type="box" pos="-1.00 -0.70 -0.50" size="0.778 0.25 0.03" euler="0 -40 0" material="bund_earth_mat" contype="1" conaffinity="7" friction="2.8 0.05 0.01"/>
    <geom name="ramp_0_curb_s" type="box" pos="-1.00 -0.96 -0.46" size="0.785 0.03 0.05" euler="0 -40 0" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="ramp_0_curb_n" type="box" pos="-1.00 -0.44 -0.46" size="0.785 0.03 0.05" euler="0 -40 0" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- RAMP 1: Ascends from Level 1 (z = 0.0m) up to Level 2 (z = +1.0m) at North Corridor (y = +0.70) -->
    <geom name="ramp_1_center" type="box" pos="4.40 0.70 0.50" size="0.778 0.25 0.03" euler="0 -40 0" material="bund_earth_mat" contype="1" conaffinity="7" friction="2.8 0.05 0.01"/>
    <geom name="ramp_1_curb_s" type="box" pos="4.40 0.44 0.54" size="0.785 0.03 0.05" euler="0 -40 0" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="ramp_1_curb_n" type="box" pos="4.40 0.96 0.54" size="0.785 0.03 0.05" euler="0 -40 0" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- RAMP 2: Ascends from Level 2 (z = +1.0m) up to Level 3 (z = +2.0m) at South Corridor (y = -0.70) -->
    <geom name="ramp_2_center" type="box" pos="9.40 -0.70 1.50" size="0.778 0.25 0.03" euler="0 -40 0" material="bund_earth_mat" contype="1" conaffinity="7" friction="2.8 0.05 0.01"/>
    <geom name="ramp_2_curb_s" type="box" pos="9.40 -0.96 1.54" size="0.785 0.03 0.05" euler="0 -40 0" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="ramp_2_curb_n" type="box" pos="9.40 -0.44 1.54" size="0.785 0.03 0.05" euler="0 -40 0" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- RAMP 3: Ascends from Level 3 (z = +2.0m) up to Tier 4 (z = +3.0m) at North Corridor (y = +0.70) -->
    <geom name="ramp_3_center" type="box" pos="14.40 0.70 2.50" size="0.778 0.25 0.03" euler="0 -40 0" material="bund_earth_mat" contype="1" conaffinity="7" friction="2.8 0.05 0.01"/>
    <geom name="ramp_3_curb_s" type="box" pos="14.40 0.44 2.54" size="0.785 0.03 0.05" euler="0 -40 0" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="ramp_3_curb_n" type="box" pos="14.40 0.96 2.54" size="0.785 0.03 0.05" euler="0 -40 0" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- ========================================================================= -->
    <!-- IMPASSABLE VERTICAL CLIFF RETAINING WALLS (1.0m DROP) OFF-RAMP            -->
    <!-- Robot CANNOT cross between levels except via the designated fixed ramps!  -->
    <!-- ========================================================================= -->

    <!-- Level 0 to Level 1 Stone Retaining Walls (Drop 1.0m from -1.0m to 0.0m) -->
    <geom name="retaining_wall_0_main" type="box" pos="-1.40 0.42 -0.50" size="1.00 0.83 0.50" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="retaining_wall_0_flank" type="box" pos="-1.40 -1.12 -0.50" size="1.00 0.13 0.50" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="bund_rim_0_main" type="box" pos="-0.43 0.42 0.05" size="0.05 0.83 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="bund_rim_0_flank" type="box" pos="-0.43 -1.12 0.05" size="0.05 0.13 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- Level 1 to Level 2 Stone Retaining Walls (Drop 1.0m from 0.0m to +1.0m) -->
    <geom name="retaining_wall_1_main" type="box" pos="4.40 -0.42 0.50" size="1.00 0.83 0.50" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="retaining_wall_1_flank" type="box" pos="4.40 1.12 0.50" size="1.00 0.13 0.50" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="bund_rim_1_main" type="box" pos="5.37 -0.42 1.05" size="0.05 0.83 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="bund_rim_1_flank" type="box" pos="5.37 1.12 1.05" size="0.05 0.13 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- Level 2 to Level 3 Stone Retaining Walls (Drop 1.0m from +1.0m to +2.0m) -->
    <geom name="retaining_wall_2_main" type="box" pos="9.40 0.47 1.50" size="1.00 0.88 0.50" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="retaining_wall_2_flank" type="box" pos="9.40 -1.17 1.50" size="1.00 0.18 0.50" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="bund_rim_2_main" type="box" pos="10.37 0.47 2.05" size="0.05 0.88 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="bund_rim_2_flank" type="box" pos="10.37 -1.17 2.05" size="0.05 0.18 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- Level 3 to Tier 4 Stone Retaining Walls (Drop 1.0m from +2.0m to +3.0m) -->
    <geom name="retaining_wall_3_main" type="box" pos="14.40 -0.52 2.50" size="1.00 0.93 0.50" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="retaining_wall_3_flank" type="box" pos="14.40 1.22 2.50" size="1.00 0.23 0.50" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- ========================================================================= -->
    <!-- FLANKING MOUNTAIN AMPHITHEATRE TERRACES (North & South Tiers)               -->
    <!-- All tiers grounded to bedrock at z = -2.5m (No floating geometry!)        -->
    <!-- ========================================================================= -->

    <!-- North Flank Tier 1 (z = +1.0m) -->
    <geom name="flank_n1_base" type="box" pos="4.50 2.70 -0.75" size="6.00 1.30 1.75" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="flank_n1_ground" type="box" pos="4.50 2.70 0.99" size="6.00 1.30 0.01" material="paddy_soil_mat" contype="1" conaffinity="7"/>
    <geom name="flank_n1_water" type="box" pos="4.50 2.70 0.995" size="5.60 1.10 0.005" material="canal_water_mat" contype="0" conaffinity="0"/>
    <geom name="flank_n1_wall" type="box" pos="4.50 1.38 0.50" size="6.00 0.06 0.50" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="flank_n1_bund" type="box" pos="4.50 1.38 1.05" size="6.00 0.06 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- South Flank Tier 1 (z = +1.0m) -->
    <geom name="flank_s1_base" type="box" pos="4.50 -2.70 -0.75" size="6.00 1.30 1.75" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="flank_s1_ground" type="box" pos="4.50 -2.70 0.99" size="6.00 1.30 0.01" material="paddy_soil_mat" contype="1" conaffinity="7"/>
    <geom name="flank_s1_water" type="box" pos="4.50 -2.70 0.995" size="5.60 1.10 0.005" material="canal_water_mat" contype="0" conaffinity="0"/>
    <geom name="flank_s1_wall" type="box" pos="4.50 -1.38 0.50" size="6.00 0.06 0.50" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="flank_s1_bund" type="box" pos="4.50 -1.38 1.05" size="6.00 0.06 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- North Flank Tier 2 (z = +2.0m) -->
    <geom name="flank_n2_base" type="box" pos="6.00 5.75 -0.25" size="8.00 1.75 2.25" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="flank_n2_ground" type="box" pos="6.00 5.75 1.99" size="8.00 1.75 0.01" material="paddy_soil_mat" contype="1" conaffinity="7"/>
    <geom name="flank_n2_wall" type="box" pos="6.00 3.98 1.50" size="8.00 0.06 0.50" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- South Flank Tier 2 (z = +2.0m) -->
    <geom name="flank_s2_base" type="box" pos="6.00 -5.75 -0.25" size="8.00 1.75 2.25" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="flank_s2_ground" type="box" pos="6.00 -5.75 1.99" size="8.00 1.75 0.01" material="paddy_soil_mat" contype="1" conaffinity="7"/>
    <geom name="flank_s2_wall" type="box" pos="6.00 -3.98 1.50" size="8.00 0.06 0.50" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- ========================================================================= -->
    <!-- LEVEL 1 OPERATIONAL PADDY SITES & CROPS                                    -->
    <!-- Flush with ground bed (no floating disks, no hovering rings)              -->
    <!-- ========================================================================= -->
    <!-- Subtle Row Guide Furrows across the Level 1 Plot -->
    <geom name="furrow_1" type="box" pos="1.55 -0.75 0.001" size="1.45 0.015 0.001" rgba="0.26 0.20 0.14 0.6" contype="0" conaffinity="0"/>
    <geom name="furrow_2" type="box" pos="1.55 -0.25 0.001" size="1.45 0.015 0.001" rgba="0.26 0.20 0.14 0.6" contype="0" conaffinity="0"/>
    <geom name="furrow_3" type="box" pos="1.55 0.20 0.001" size="1.45 0.015 0.001" rgba="0.26 0.20 0.14 0.6" contype="0" conaffinity="0"/>
    <geom name="furrow_4" type="box" pos="1.55 0.70 0.001" size="1.45 0.015 0.001" rgba="0.26 0.20 0.14 0.6" contype="0" conaffinity="0"/>

    <!-- 6 Field Patches on Level 1 (Mud Hazard vs Viable Stand-Gap Loam vs Living Rice Stand) -->
    <geom name="bed_patch_a" type="cylinder" pos="0.85 -0.50 0.001" size="0.25 0.001" material="mud_hazard_mat" contype="0" conaffinity="0"/>
    <geom name="puddle_patch_a" type="cylinder" pos="0.85 -0.50 0.002" size="0.18 0.001" material="canal_water_mat" contype="0" conaffinity="0"/>

    <geom name="bed_patch_b" type="cylinder" pos="1.60 -0.50 0.001" size="0.25 0.001" material="viable_loam_mat" contype="0" conaffinity="0"/>

    <geom name="bed_patch_c" type="cylinder" pos="2.35 -0.50 0.001" size="0.25 0.001" material="viable_loam_mat" contype="0" conaffinity="0"/>

    <geom name="bed_patch_d" type="cylinder" pos="1.60 0.45 0.001" size="0.25 0.001" material="mud_hazard_mat" contype="0" conaffinity="0"/>
    <geom name="puddle_patch_d" type="cylinder" pos="1.60 0.45 0.002" size="0.17 0.001" material="canal_water_mat" contype="0" conaffinity="0"/>

    <geom name="bed_crop_nw" type="cylinder" pos="0.85 0.45 0.001" size="0.27 0.001" material="crop_bed_mat" contype="0" conaffinity="0"/>
    <geom name="bed_crop_ne" type="cylinder" pos="2.35 0.45 0.001" size="0.27 0.001" material="crop_bed_mat" contype="0" conaffinity="0"/>

    <!-- Mid-episode obstacle (fallen irrigation crate parked below ground until surprise injection) -->
    <body name="mid_run_obstacle" mocap="true" pos="{ox:.3f} {oy:.3f} -5.0">
      <geom name="obstacle_geom" type="box" size="0.11 0.14 0.14" contype="5" conaffinity="7" rgba="0.88 0.24 0.16 1"/>
    </body>
"""
