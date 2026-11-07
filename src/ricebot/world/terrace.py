"""World and local soil-sensor host for PaddyPatch (`src/ricebot/world/`).

Strict separation per `docs/COMPONENTS.md`:
- Owns field sites (`DEFAULT_SITES`), public traversability/no-go flags, hidden synthetic
  spatial soil/stand distribution, location-gated soil probes, and world terrain geometry.
- NEVER owns rover chassis, wheel joints, skid-steer kinematics, seed hopper geometry,
  or mission planning.
"""
from __future__ import annotations

from math import hypot

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


class PaddyTerraceWorld:
    """Simulator-independent world & location-gated sensor host.

    Maintains hidden ground-truth soil/stand profiles and emits local `SoilSample`
    readings strictly when a rover's physical pose enters `probe_radius_m` of a
    permitted site. Used identically by both MuJoCo and Isaac Sim adapters.
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
        self._sampled_pairs: set[tuple[str, str]] = set()

    def reset(self, seed: int) -> None:
        """Reset episode sampling state."""
        _ = seed
        self._sampled_pairs.clear()

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
        """Return wheel-velocity traction factor in `(0.35, 1.0]` based on hidden local mud softness."""
        for site_id, site in self.sites.items():
            truth = self._hidden_site_truth.get(site_id)
            if truth is None:
                continue
            if truth[SensorKind.MOISTURE] > 0.80 and truth[SensorKind.FIRMNESS] < 0.25:
                if hypot(pose.x_m - site.center.x_m, pose.y_m - site.center.y_m) <= self.probe_radius_m:
                    return 0.36
        return 1.0


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
