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
from math import cos, cosh, exp, hypot, sin, tanh
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
    tier_base = 2.0 if elevation_m > 1.5 else (1.0 if elevation_m > 0.5 else 0.0)
    dz_local = elevation_m - tier_base
    s_micro_topo = exp(-((min(0.0, dz_local + 0.008)) ** 2) / (2.0 * 0.018 * 0.018))
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

        Models stepped Chinese rice terraces with exact 1.0m height difference:
        - Tier 1 (High): x < 1.18m -> z ≈ +2.0m
        - Tier 2 (Mid):  1.18m <= x < 1.98m -> z ≈ +1.0m
        - Tier 3 (Low):  x >= 1.98m -> z ≈ 0.0m
        Transitions between levels occur exclusively via dedicated fixed ramps
        located at South (y = -0.48), Center (y = 0.00), and North (y = +0.48).
        Off-ramp boundaries form steep impassable cliff retaining walls.
        """
        # Fixed ramp corridors at y = -0.48, 0.00, +0.48
        d_ramp = min(abs(y_m - (-0.48)), abs(y_m), abs(y_m - 0.48))
        ramp_factor = exp(-((d_ramp / 0.16) ** 4))
        w = 0.025 + (0.260 - 0.025) * ramp_factor

        u1 = (x_m - 1.18) / w
        s1 = 0.5 * (1.0 - tanh(u1))
        u2 = (x_m - 1.98) / w
        s2 = 0.5 * (1.0 - tanh(u2))

        # Base 1.0m stepped levels: Tier 1 (+2.0m), Tier 2 (+1.0m), Tier 3 (0.0m)
        z = 1.0 * s1 + 1.0 * s2

        phase = 0.17 * (self.seed % 11)
        z_micro = (
            0.008 * sin(1.35 * x_m + phase) * cos(1.65 * y_m)
            + 0.004 * cos(2.40 * x_m - 1.90 * y_m)
        )
        z += z_micro

        grad_scale = 0.072 * ramp_factor + 3.5 * (1.0 - ramp_factor)
        sech1 = 1.0 / cosh(max(-10.0, min(10.0, u1)))
        sech2 = 1.0 / cosh(max(-10.0, min(10.0, u2)))
        dz_dx = -0.5 * (grad_scale / w) * (sech1 * sech1) - 0.5 * (grad_scale / w) * (sech2 * sech2)
        dz_dy = 0.0

        # Depressed waterlogged Lannitian mud basins around high-moisture anchors
        for site in self._sites:
            truth = self._anchors.get(site.site_id)
            if truth and truth[SensorKind.MOISTURE] > 0.80:
                dx = x_m - site.center.x_m
                dy = y_m - site.center.y_m
                r2 = dx * dx + dy * dy
                sigma2 = 0.42 * 0.42
                w_basin = exp(-r2 / (2.0 * sigma2))
                basin_depth = -0.028 * w_basin
                z += basin_depth
                dz_dx += basin_depth * (-dx / sigma2)
                dz_dy += basin_depth * (-dy / sigma2)

        return (round(z, 5), round(dz_dx, 5), round(dz_dy, 5))

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

        # Stepped 1.0m terrace elevation with ramps
        d_ramp = np.minimum(np.abs(Y - (-0.48)), np.minimum(np.abs(Y), np.abs(Y - 0.48)))
        ramp_factor = np.exp(-((d_ramp / 0.16) ** 4))
        W = 0.025 + (0.260 - 0.025) * ramp_factor

        U1 = (X - 1.18) / W
        S1 = 0.5 * (1.0 - np.tanh(U1))
        U2 = (X - 1.98) / W
        S2 = 0.5 * (1.0 - np.tanh(U2))
        Z = 1.0 * S1 + 1.0 * S2

        phase_z = 0.17 * (self.seed % 11)
        Z += (
            0.008 * np.sin(1.35 * X + phase_z) * np.cos(1.65 * Y)
            + 0.004 * np.cos(2.40 * X - 1.90 * Y)
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
        self, robot_id: str, pose: Pose2D, time_s: float
    ) -> tuple[SoilSample, ...]:
        """Return local soil/stand samples if and only if `pose` is within `probe_radius_m`."""
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
                for kind in (SensorKind.MOISTURE, SensorKind.FIRMNESS, SensorKind.STAND_GAP):
                    samples.append(
                        SoilSample(
                            sample_id=f"{robot_id}:{site_id}:{kind.value}",
                            robot_id=robot_id,
                            site_id=site_id,
                            kind=kind,
                            value=truth[kind],
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
    """Generate 3D green rice seedling stalks inside the protected crop patches."""
    lines: list[str] = []
    for prefix, (cx, cy) in (("nw", (0.85, 0.45)), ("ne", (2.35, 0.45))):
        idx = 0
        for dx in (-0.14, 0.0, 0.14):
            for dy in (-0.14, 0.0, 0.14):
                idx += 1
                x, y = cx + dx, cy + dy
                lines.append(
                    f'<geom name="rice_{prefix}_{idx}_a" type="capsule" fromto="{x:.3f} {y:.3f} 0.01 {x-0.025:.3f} {y+0.015:.3f} 0.15" '
                    f'size="0.009" contype="0" conaffinity="0" rgba="0.18 0.68 0.22 1"/>'
                )
                lines.append(
                    f'<geom name="rice_{prefix}_{idx}_b" type="capsule" fromto="{x:.3f} {y:.3f} 0.01 {x+0.025:.3f} {y-0.018:.3f} 0.16" '
                    f'size="0.008" contype="0" conaffinity="0" rgba="0.25 0.76 0.26 1"/>'
                )
                lines.append(
                    f'<geom name="rice_{prefix}_{idx}_c" type="capsule" fromto="{x:.3f} {y:.3f} 0.01 {x+0.005:.3f} {y+0.028:.3f} 0.14" '
                    f'size="0.008" contype="0" conaffinity="0" rgba="0.14 0.58 0.18 1"/>'
                )
    return "\n    ".join(lines)


def build_world_terrain_xml(obstacle_xy: tuple[float, float] = (0.75, -0.25)) -> str:
    """Return MJCF `<worldbody>` fragment for genuine 3D stepped rice terraces with 1m elevation steps and 40 deg ramps."""
    ox, oy = obstacle_xy
    return f"""
    <light name="sun_main" pos="4.5 -5.0 9.0" dir="-0.25 0.35 -1" diffuse="0.92 0.90 0.82" specular="0.35 0.35 0.30" castshadow="true"/>
    <light name="sky_fill" pos="2.5 3.0 7.0" dir="0 -0.2 -1" diffuse="0.45 0.48 0.52" castshadow="false"/>

    <!-- Wide amphitheatre overview camera capturing ant-scale swarm on vast 3D stepped rice terraces -->
    <camera name="terrace_iso" pos="-2.20 -4.60 4.20" xyaxes="0.88 -0.46 0 0.22 0.44 0.87"/>
    <camera name="top_overview" pos="2.40 0.00 5.60" xyaxes="1 0 0 0 1 0"/>

    <!-- ========================================================================= -->
    <!-- BROAD VALLEY MOUNTAIN BASE (Prevents any hollow voids or floating sheets) -->
    <!-- ========================================================================= -->
    <geom name="valley_floor" type="box" pos="4.50 0.00 -2.50" size="14.0 10.0 1.0" material="mountain_rock_mat" contype="1" conaffinity="7"/>

    <!-- ========================================================================= -->
    <!-- TRUE 3D PHYSICAL ELEVATED RICE TERRACES (1.0m Height Difference Per Level)  -->
    <!-- ========================================================================= -->

    <!-- LEVEL 0: Valley Terrace (Top Surface z = -1.0m, Solid Earth Base to -2.0m) -->
    <geom name="terrace_level0_base" type="box" pos="-2.25 0.00 -1.75" size="1.40 1.30 0.75" material="deep_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_level0_ground" type="box" pos="-2.25 0.00 -1.00" size="1.40 1.30 0.02" material="paddy_soil_mat" contype="1" conaffinity="7" friction="1.5 0.01 0.005"/>
    <geom name="terrace_level0_bund_w" type="box" pos="-3.69 0.00 -0.95" size="0.08 1.36 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_level0_bund_n" type="box" pos="-2.25 1.33 -0.95" size="1.44 0.08 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_level0_bund_s" type="box" pos="-2.25 -1.33 -0.95" size="1.44 0.08 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- LEVEL 1: Operational Terrace (Top Surface z = 0.0m, Solid Earth Base to -2.0m) -->
    <geom name="terrace_level1_base" type="box" pos="1.39 0.00 -1.00" size="1.81 1.25 1.00" material="deep_earth_mat" contype="1" conaffinity="7"/>
    <geom name="field_ground" type="box" pos="1.39 0.00 -0.02" size="1.81 1.25 0.02" material="paddy_soil_mat" contype="1" conaffinity="7" friction="1.5 0.01 0.005"/>
    <!-- Raised Perimeter Earthen Bunds around Level 1 -->
    <geom name="bund_north" type="box" pos="1.39 1.25 0.05" size="1.85 0.08 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="bund_south" type="box" pos="1.39 -1.25 0.05" size="1.85 0.08 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="bund_west" type="box" pos="-0.46 0.00 0.05" size="0.08 1.25 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- LEVEL 2: Mid Mountain Terrace (Top Surface z = +1.0m, Solid Earth Base to -2.0m) -->
    <geom name="terrace_level2_base" type="box" pos="5.79 0.00 -0.50" size="1.40 1.25 1.50" material="deep_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_level2_ground" type="box" pos="5.79 0.00 0.98" size="1.40 1.25 0.02" material="paddy_soil_mat" contype="1" conaffinity="7" friction="1.5 0.01 0.005"/>
    <geom name="terrace_level2_bund_n" type="box" pos="5.79 1.25 1.05" size="1.44 0.08 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_level2_bund_s" type="box" pos="5.79 -1.25 1.05" size="1.44 0.08 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_level2_bund_e" type="box" pos="7.23 0.00 1.05" size="0.08 1.25 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- LEVEL 3: Upper Mountain Terrace (Top Surface z = +2.0m, Solid Earth Base to -2.0m) -->
    <geom name="terrace_level3_base" type="box" pos="9.88 0.00 0.00" size="1.50 1.30 2.00" material="deep_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_level3_ground" type="box" pos="9.88 0.00 1.98" size="1.50 1.30 0.02" material="paddy_soil_mat" contype="1" conaffinity="7" friction="1.5 0.01 0.005"/>
    <geom name="terrace_level3_bund_n" type="box" pos="9.88 1.30 2.05" size="1.54 0.08 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_level3_bund_s" type="box" pos="9.88 -1.30 2.05" size="1.54 0.08 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_level3_bund_e" type="box" pos="11.42 0.00 2.05" size="0.08 1.30 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- ========================================================================= -->
    <!-- FIXED RAMPS: EXACTLY 40.0 DEGREE INCLINE WITH RESPECT TO THE X-AXIS       -->
    <!-- Height difference dz = 1.0m. Angle theta = 40.0 deg.                      -->
    <!-- Horizontal run dx = 1.0 / tan(40 deg) = 1.19175m                          -->
    <!-- Slope length L = 1.0 / sin(40 deg) = 1.55572m -> half-length = 0.77786m   -->
    <!-- High friction clay earth, solid collision contype=1 conaffinity=7         -->
    <!-- ========================================================================= -->

    <!-- RAMP 1: Connecting Level 1 (z=0.0m) to Level 2 (z=1.0m) -->
    <geom name="ramp_1_south" type="box" pos="3.79588 -0.55 0.50" size="0.77786 0.20 0.03" euler="0 -40 0" material="bund_earth_mat" contype="1" conaffinity="7" friction="2.8 0.05 0.01"/>
    <geom name="ramp_1_center" type="box" pos="3.79588 0.00 0.50" size="0.77786 0.22 0.03" euler="0 -40 0" material="bund_earth_mat" contype="1" conaffinity="7" friction="2.8 0.05 0.01"/>
    <geom name="ramp_1_north" type="box" pos="3.79588 0.55 0.50" size="0.77786 0.20 0.03" euler="0 -40 0" material="bund_earth_mat" contype="1" conaffinity="7" friction="2.8 0.05 0.01"/>

    <!-- RAMP 2: Connecting Level 2 (z=1.0m) to Level 3 (z=2.0m) -->
    <geom name="ramp_2_south" type="box" pos="7.78588 -0.55 1.50" size="0.77786 0.20 0.03" euler="0 -40 0" material="bund_earth_mat" contype="1" conaffinity="7" friction="2.8 0.05 0.01"/>
    <geom name="ramp_2_center" type="box" pos="7.78588 0.00 1.50" size="0.77786 0.22 0.03" euler="0 -40 0" material="bund_earth_mat" contype="1" conaffinity="7" friction="2.8 0.05 0.01"/>
    <geom name="ramp_2_north" type="box" pos="7.78588 0.55 1.50" size="0.77786 0.20 0.03" euler="0 -40 0" material="bund_earth_mat" contype="1" conaffinity="7" friction="2.8 0.05 0.01"/>

    <!-- RAMP 0: Connecting Level 0 (z=-1.0m) to Level 1 (z=0.0m) -->
    <geom name="ramp_0_center" type="box" pos="-1.05588 0.00 -0.50" size="0.77786 0.22 0.03" euler="0 40 0" material="bund_earth_mat" contype="1" conaffinity="7" friction="2.8 0.05 0.01"/>

    <!-- ========================================================================= -->
    <!-- IMPASSABLE VERTICAL CLIFF RETAINING WALLS (1.0m DROP) OFF-RAMP            -->
    <!-- Robot CANNOT cross between levels except via the designated 40 deg ramps! -->
    <!-- ========================================================================= -->

    <!-- Level 1 to Level 2 Stone Retaining Walls (Drop 1.0m: z=0.0 to z=1.0, center z=0.50, sz=0.50) -->
    <geom name="retaining_wall_1_south_flank" type="box" pos="3.80 -0.95 0.50" size="0.59 0.20 0.50" material="terrace_wall_mat" contype="1" conaffinity="7"/>
    <geom name="retaining_wall_1_mid_s" type="box" pos="3.80 -0.27 0.50" size="0.59 0.08 0.50" material="terrace_wall_mat" contype="1" conaffinity="7"/>
    <geom name="retaining_wall_1_mid_n" type="box" pos="3.80 0.27 0.50" size="0.59 0.08 0.50" material="terrace_wall_mat" contype="1" conaffinity="7"/>
    <geom name="retaining_wall_1_north_flank" type="box" pos="3.80 0.95 0.50" size="0.59 0.20 0.50" material="terrace_wall_mat" contype="1" conaffinity="7"/>

    <!-- Level 2 to Level 3 Stone Retaining Walls (Drop 1.0m: z=1.0 to z=2.0, center z=1.50, sz=0.50) -->
    <geom name="retaining_wall_2_south_flank" type="box" pos="7.79 -0.95 1.50" size="0.59 0.20 0.50" material="terrace_wall_mat" contype="1" conaffinity="7"/>
    <geom name="retaining_wall_2_mid_s" type="box" pos="7.79 -0.27 1.50" size="0.59 0.08 0.50" material="terrace_wall_mat" contype="1" conaffinity="7"/>
    <geom name="retaining_wall_2_mid_n" type="box" pos="7.79 0.27 1.50" size="0.59 0.08 0.50" material="terrace_wall_mat" contype="1" conaffinity="7"/>
    <geom name="retaining_wall_2_north_flank" type="box" pos="7.79 0.95 1.50" size="0.59 0.20 0.50" material="terrace_wall_mat" contype="1" conaffinity="7"/>

    <!-- Level 0 to Level 1 Stone Retaining Walls (Drop 1.0m: z=-1.0 to z=0.0, center z=-0.50, sz=0.50) -->
    <geom name="retaining_wall_0_south_flank" type="box" pos="-1.06 -0.75 -0.50" size="0.59 0.45 0.50" material="terrace_wall_mat" contype="1" conaffinity="7"/>
    <geom name="retaining_wall_0_north_flank" type="box" pos="-1.06 0.75 -0.50" size="0.59 0.45 0.50" material="terrace_wall_mat" contype="1" conaffinity="7"/>

    <!-- Lateral North Flank Tier (Elevated Stepped Terraces with Rice Crops) -->
    <geom name="flank_n1_base" type="box" pos="4.50 3.20 -0.50" size="5.50 1.70 1.50" material="deep_earth_mat" contype="1" conaffinity="7"/>
    <geom name="flank_n1_ground" type="box" pos="4.50 3.20 1.00" size="5.50 1.70 0.02" material="paddy_soil_mat" contype="1" conaffinity="7"/>
    <geom name="flank_n1_wall" type="box" pos="4.50 1.45 0.50" size="5.50 0.08 0.50" material="terrace_wall_mat" contype="1" conaffinity="7"/>
    <geom name="flank_n1_bund" type="box" pos="4.50 1.45 1.06" size="5.50 0.08 0.04" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- Lateral South Flank Tier (Elevated Stepped Terraces with Rice Crops) -->
    <geom name="flank_s1_base" type="box" pos="4.50 -3.20 -0.50" size="5.50 1.70 1.50" material="deep_earth_mat" contype="1" conaffinity="7"/>
    <geom name="flank_s1_ground" type="box" pos="4.50 -3.20 1.00" size="5.50 1.70 0.02" material="paddy_soil_mat" contype="1" conaffinity="7"/>
    <geom name="flank_s1_wall" type="box" pos="4.50 -1.45 0.50" size="5.50 0.08 0.50" material="terrace_wall_mat" contype="1" conaffinity="7"/>
    <geom name="flank_s1_bund" type="box" pos="4.50 -1.45 1.06" size="5.50 0.08 0.04" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- High Amphitheatre Ridge Tier (z = +3.0m) -->
    <geom name="flank_n2_base" type="box" pos="5.00 6.00 0.50" size="6.00 1.50 2.50" material="deep_earth_mat" contype="1" conaffinity="7"/>
    <geom name="flank_n2_ground" type="box" pos="5.00 6.00 2.00" size="6.00 1.50 0.02" material="paddy_soil_mat" contype="1" conaffinity="7"/>
    <geom name="flank_n2_wall" type="box" pos="5.00 4.95 1.50" size="6.00 0.08 0.50" material="terrace_wall_mat" contype="1" conaffinity="7"/>

    <!-- Sweeping Mountain Terraces & Background (Longji mountain amphitheatre) -->
    <!-- Tier 4: Mountain Terrace Ridge (z = +3.0m, Solid Earth Base) -->
    <geom name="terrace_tier4_base" type="box" pos="13.20 0.00 0.50" size="1.80 4.00 2.50" material="deep_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_tier4_ground" type="box" pos="13.20 0.00 2.98" size="1.80 4.00 0.02" material="paddy_soil_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_tier4_wall" type="box" pos="11.42 0.00 2.50" size="0.06 4.00 0.50" material="terrace_wall_mat" contype="1" conaffinity="7"/>

    <!-- Tier 5: Mountain Terrace Ridge (z = +4.0m) -->
    <geom name="terrace_tier5_base" type="box" pos="16.50 0.00 1.00" size="1.60 6.00 3.00" material="deep_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_tier5_ground" type="box" pos="16.50 0.00 3.98" size="1.60 6.00 0.02" material="paddy_soil_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_tier5_wall" type="box" pos="14.98 0.00 3.50" size="0.06 6.00 0.50" material="terrace_wall_mat" contype="1" conaffinity="7"/>

    <!-- Distant Forested Mountain Ridges (Longji / Yuanyang mountain valley) -->
    <geom name="mountain_ridge_east" type="ellipsoid" pos="21.0 0.0 6.5" size="4.5 10.0 5.5" rgba="0.22 0.38 0.25 1" contype="0" conaffinity="0"/>
    <geom name="mountain_peak_north" type="ellipsoid" pos="12.0 12.0 7.5" size="8.0 5.0 5.5" rgba="0.26 0.42 0.28 1" contype="0" conaffinity="0"/>
    <geom name="mountain_peak_south" type="ellipsoid" pos="12.0 -12.0 7.5" size="8.0 5.0 5.5" rgba="0.26 0.42 0.28 1" contype="0" conaffinity="0"/>

    <!-- Row Guide Furrows across the Level 1 Paddy Plot -->
    <geom name="furrow_1" type="box" pos="1.45 -0.75 0.002" size="1.40 0.015 0.002" rgba="0.26 0.20 0.14 0.6" contype="0" conaffinity="0"/>
    <geom name="furrow_2" type="box" pos="1.45 -0.25 0.002" size="1.40 0.015 0.002" rgba="0.26 0.20 0.14 0.6" contype="0" conaffinity="0"/>
    <geom name="furrow_3" type="box" pos="1.45 0.20 0.002" size="1.40 0.015 0.002" rgba="0.26 0.20 0.14 0.6" contype="0" conaffinity="0"/>
    <geom name="furrow_4" type="box" pos="1.45 0.70 0.002" size="1.40 0.015 0.002" rgba="0.26 0.20 0.14 0.6" contype="0" conaffinity="0"/>

    <!-- 6 Field Patches on Level 1 (Mud Hazard vs Viable Stand-Gap Loam vs Living Rice Stand) -->
    <geom name="bed_patch_a" type="cylinder" pos="0.85 -0.50 0.003" size="0.25 0.003" material="mud_hazard_mat" contype="0" conaffinity="0"/>
    <geom name="puddle_patch_a" type="cylinder" pos="0.85 -0.50 0.005" size="0.18 0.002" material="canal_water_mat" contype="0" conaffinity="0"/>

    <geom name="bed_patch_b" type="cylinder" pos="1.60 -0.50 0.003" size="0.25 0.003" material="viable_loam_mat" contype="0" conaffinity="0"/>
    <site name="ring_patch_b" type="cylinder" pos="1.60 -0.50 0.006" size="0.16 0.002" rgba="0.85 0.72 0.25 0.55"/>

    <geom name="bed_patch_c" type="cylinder" pos="2.35 -0.50 0.003" size="0.25 0.003" material="viable_loam_mat" contype="0" conaffinity="0"/>
    <site name="ring_patch_c" type="cylinder" pos="2.35 -0.50 0.006" size="0.16 0.002" rgba="0.85 0.72 0.25 0.55"/>

    <geom name="bed_patch_d" type="cylinder" pos="1.60 0.45 0.003" size="0.25 0.003" material="mud_hazard_mat" contype="0" conaffinity="0"/>
    <geom name="puddle_patch_d" type="cylinder" pos="1.60 0.45 0.005" size="0.17 0.002" material="canal_water_mat" contype="0" conaffinity="0"/>

    <geom name="bed_crop_nw" type="cylinder" pos="0.85 0.45 0.004" size="0.27 0.004" material="crop_bed_mat" contype="0" conaffinity="0"/>
    <geom name="bed_crop_ne" type="cylinder" pos="2.35 0.45 0.004" size="0.27 0.004" material="crop_bed_mat" contype="0" conaffinity="0"/>
    {_build_seedling_clumps_xml()}

    <!-- Mid-episode obstacle (fallen irrigation crate parked below ground until surprise injection) -->
    <body name="mid_run_obstacle" mocap="true" pos="{ox:.3f} {oy:.3f} -5.0">
      <geom name="obstacle_geom" type="box" size="0.11 0.14 0.14" contype="5" conaffinity="7" rgba="0.88 0.24 0.16 1"/>
    </body>
"""
