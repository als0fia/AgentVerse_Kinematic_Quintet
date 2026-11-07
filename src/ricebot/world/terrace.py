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

    def elevation_and_gradient(self, x_m: float, y_m: float) -> tuple[float, float, float]:
        """Return `(elevation_m, dz_dx, dz_dy)` at `(x_m, y_m)` in metres."""
        phase = 0.17 * (self.seed % 11)
        # Gentle natural terrace drainage grade + micro-undulations (hilliness)
        z = (
            0.012 * sin(1.35 * x_m + phase) * cos(1.65 * y_m)
            + 0.008 * cos(2.40 * x_m - 1.90 * y_m)
            - 0.005 * (x_m - 1.50)
        )
        dz_dx = (
            0.012 * 1.35 * cos(1.35 * x_m + phase) * cos(1.65 * y_m)
            - 0.008 * 2.40 * sin(2.40 * x_m - 1.90 * y_m)
            - 0.005
        )
        dz_dy = (
            -0.012 * 1.65 * sin(1.35 * x_m + phase) * sin(1.65 * y_m)
            + 0.008 * 1.90 * sin(2.40 * x_m - 1.90 * y_m)
        )

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
        )

    def sample_grid(
        self,
        nx: int = 36,
        ny: int = 24,
        x_bounds: tuple[float, float] = (-0.25, 2.95),
        y_bounds: tuple[float, float] = (-0.90, 0.90),
    ) -> dict[str, object]:
        """Sample the continuous terrain over a regular 2D grid for dashboard heatmaps."""
        xs = [
            round(x_bounds[0] + i * (x_bounds[1] - x_bounds[0]) / max(1, nx - 1), 4)
            for i in range(nx)
        ]
        ys = [
            round(y_bounds[0] + j * (y_bounds[1] - y_bounds[0]) / max(1, ny - 1), 4)
            for j in range(ny)
        ]
        elevation: list[list[float]] = []
        moisture: list[list[float]] = []
        firmness: list[list[float]] = []
        mud_depth: list[list[float]] = []
        plant_density: list[list[float]] = []
        stand_gap: list[list[float]] = []

        for y in ys:
            row_z, row_m, row_f, row_d, row_p, row_g = [], [], [], [], [], []
            for x in xs:
                pt = self.evaluate(x, y)
                row_z.append(pt.elevation_m)
                row_m.append(pt.moisture)
                row_f.append(pt.firmness)
                row_d.append(pt.mud_depth_m)
                row_p.append(pt.plant_density)
                row_g.append(pt.stand_gap)
            elevation.append(row_z)
            moisture.append(row_m)
            firmness.append(row_f)
            mud_depth.append(row_d)
            plant_density.append(row_p)
            stand_gap.append(row_g)

        return {
            "nx": nx,
            "ny": ny,
            "xs": xs,
            "ys": ys,
            "x_bounds": list(x_bounds),
            "y_bounds": list(y_bounds),
            "layers": {
                "elevation_m": elevation,
                "moisture": moisture,
                "firmness": firmness,
                "mud_depth_m": mud_depth,
                "plant_density": plant_density,
                "stand_gap": stand_gap,
            },
        }


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
    """Return MJCF `<worldbody>` fragment for the bunded rice terrace, patches, crops, and obstacle."""
    ox, oy = obstacle_xy
    return f"""
    <light name="sun_main" pos="1.5 -1.0 4.0" dir="-0.1 0.2 -1" diffuse="0.85 0.83 0.78" specular="0.25 0.25 0.25" castshadow="true"/>
    <light name="sky_fill" pos="1.5 1.5 3.0" dir="0 -0.3 -1" diffuse="0.35 0.38 0.42" castshadow="false"/>

    <camera name="terrace_iso" pos="-0.65 -1.95 1.55" xyaxes="0.82 -0.57 0 0.32 0.46 0.83"/>
    <camera name="top_overview" pos="1.35 -0.02 3.10" xyaxes="1 0 0 0 1 0"/>

    <!-- Main Paddy Field Ground Plane (collision group 1) -->
    <geom name="field_ground" type="plane" size="5 5 0.1" material="paddy_soil_mat" contype="1" conaffinity="7" friction="1.2 0.01 0.005"/>

    <!-- Raised Perimeter Rice Terrace Bunds / Field Ridges (田埂) -->
    <geom name="bund_north" type="box" pos="1.35 1.05 0.05" size="1.85 0.12 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="bund_south" type="box" pos="1.35 -1.05 0.05" size="1.85 0.12 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="bund_west" type="box" pos="-0.42 0.00 0.05" size="0.12 1.17 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="bund_east" type="box" pos="3.12 0.00 0.05" size="0.12 1.17 0.05" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- Irrigation Canal & Water Channel along West Bund -->
    <geom name="irrigation_canal_water" type="box" pos="-0.72 0.00 0.008" size="0.16 1.15 0.008" material="canal_water_mat" contype="0" conaffinity="0"/>

    <!-- Subtle Row Guide Furrows across the Paddy Plot -->
    <geom name="furrow_1" type="box" pos="1.55 -0.75 0.002" size="1.45 0.015 0.002" rgba="0.26 0.20 0.14 0.6" contype="0" conaffinity="0"/>
    <geom name="furrow_2" type="box" pos="1.55 -0.25 0.002" size="1.45 0.015 0.002" rgba="0.26 0.20 0.14 0.6" contype="0" conaffinity="0"/>
    <geom name="furrow_3" type="box" pos="1.55 0.20 0.002" size="1.45 0.015 0.002" rgba="0.26 0.20 0.14 0.6" contype="0" conaffinity="0"/>
    <geom name="furrow_4" type="box" pos="1.55 0.70 0.002" size="1.45 0.015 0.002" rgba="0.26 0.20 0.14 0.6" contype="0" conaffinity="0"/>

    <!-- 6 Field Patches (Visual Terrain Beds: Mud Hazard vs Viable Stand-Gap Loam vs Living Rice Stand) -->
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
