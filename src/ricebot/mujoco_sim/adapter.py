"""MuJoCo 3D paddy-terrace & homogeneous Paddy Ant rover physics adapter (`src/ricebot/mujoco_sim/`).

Architectural Composition per `docs/COMPONENTS.md` & `docs/ARCHITECTURE.md`:
- Composes:
  1. `src/ricebot/world/` (`PaddyTerraceWorld`, `ContinuousPaddyTerrain`, `DEFAULT_SITES`,
     `build_world_terrain_xml`) — owns the 6-patch rice terrace layout, continuous 2D soil/elevation
     functions, hidden soil/stand truth, location-gated probes, and terrain/bunds/crops MJCF geometry.
  2. `src/ricebot/robot/` (`PLANTER_SPEC`, `SCOUT_SPEC`, `DEFAULT_CAPABILITIES`,
     `compute_4wheel_skid_steer`, `evaluate_bekker_terramechanics`, `build_rovers_and_seeds_xml`,
     `update_rigid_dock_latch`) — owns the homogeneous "Paddy Ant" 4-wheel grouser-lug chassis,
     analytical Bekker-Wong wheel-mud sinkage & traction physics, active farming end-effectors,
     and the 4-part rigid docking, cam-latching & anchor-spade rescue mechanism.
- Implements `SimulationPort` (`reset(seed) -> ObservationFrame`, `step(commands) -> ObservationFrame`)
  without touching `MissionCore` (`src/ricebot/runtime.py`) or `src/ricebot/contracts.py`.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from math import asin, atan2, cos, hypot, pi, sin
import random

import mujoco
import numpy as np

from ..contracts import (
    ObservationFrame,
    PlantFeedback,
    Pose2D,
    RecoveryFeedback,
    RobotCommand,
    RobotObservation,
    SensorKind,
    Site,
)
from ..robot import (
    DEFAULT_CAPABILITIES,
    PLANTER_SPEC,
    SCOUT_SPEC,
    RoverChassisSpec,
    TerramechanicsSinkageState,
    build_rovers_and_seeds_xml,
    compute_4wheel_skid_steer,
    evaluate_bekker_terramechanics,
    make_fleet_specs,
    update_rigid_dock_latch,
)
from ..world import (
    ContinuousTerrainPoint,
    DEFAULT_SITES,
    PaddyTerraceWorld,
    build_world_terrain_xml,
    generate_randomized_episode_spec,
)


def compose_paddypatch_mjcf(
    obstacle_xy: tuple[float, float] = (0.75, -0.25),
    rover_specs: tuple[RoverChassisSpec, ...] | None = None,
) -> str:
    """Compose the full MJCF XML from `src/ricebot/world/` terrain and `src/ricebot/robot/` Paddy Ant units."""
    fleet = rover_specs or (PLANTER_SPEC, SCOUT_SPEC)
    world_terrain_xml = build_world_terrain_xml(obstacle_xy=obstacle_xy)
    rovers_bodies_xml, rovers_actuators_xml, rovers_sensors_xml = build_rovers_and_seeds_xml(specs=fleet)
    equality_lines = [
        f'    <connect name="rigid_latch_{a.robot_id}_to_{b.robot_id}" '
        f'site1="{a.robot_id}_front_dock_site" site2="{b.robot_id}_rear_dock_site" active="false" solref="0.04 1.0"/>'
        for a in fleet
        for b in fleet
        if a.robot_id != b.robot_id
    ]
    equality_xml = "\n".join(equality_lines)
    return f"""
<mujoco model="paddypatch_rice_v1_terrace">
  <option timestep="0.004" gravity="0 0 -9.81" integrator="implicitfast"/>

  <visual>
    <global offwidth="1280" offheight="960" azimuth="115" elevation="-34"/>
    <headlight ambient="0.38 0.38 0.38" diffuse="0.65 0.65 0.62" specular="0.2 0.2 0.2"/>
    <map znear="0.02" zfar="25.0"/>
  </visual>

  <statistic center="1.35 -0.05 0.15" extent="2.2"/>

  <asset>
    <texture name="sky_tex" type="skybox" builtin="gradient" rgb1="0.68 0.82 0.96" rgb2="0.18 0.32 0.52" width="256" height="256"/>
    <texture name="paddy_soil_tex" type="2d" builtin="checker" rgb1="0.36 0.29 0.21" rgb2="0.32 0.25 0.18" width="256" height="256" mark="edge" markrgb="0.28 0.22 0.16"/>
    <material name="paddy_soil_mat" texture="paddy_soil_tex" texrepeat="6 6" reflectance="0.08" shininess="0.2"/>
    <material name="bund_earth_mat" rgba="0.46 0.37 0.25 1" roughness="0.9"/>
    <material name="mud_hazard_mat" rgba="0.22 0.16 0.11 0.92" shininess="0.85" specular="0.6"/>
    <material name="viable_loam_mat" rgba="0.48 0.39 0.26 0.90" roughness="0.8"/>
    <material name="crop_bed_mat" rgba="0.20 0.42 0.18 0.85"/>
    <material name="canal_water_mat" rgba="0.16 0.46 0.58 0.78" shininess="0.9" specular="0.8"/>
    <material name="wheel_rubber_mat" rgba="0.16 0.16 0.18 1"/>
    <hfield name="paddy_micro_relief" nrow="32" ncol="48" size="1.65 0.92 0.045 0.02"/>
  </asset>

  <worldbody>
    {world_terrain_xml}
    <geom name="paddy_relief_mesh" type="hfield" hfield="paddy_micro_relief" pos="1.35 0.0 -0.016" rgba="0.39 0.31 0.21 0.88" group="2" contype="0" conaffinity="0"/>
    {rovers_bodies_xml}
  </worldbody>

  <equality>
{equality_xml}
  </equality>

  <actuator>
    {rovers_actuators_xml}
  </actuator>

  <sensor>
    {rovers_sensors_xml}
  </sensor>
</mujoco>
"""


PADDYPATCH_MJCF = compose_paddypatch_mjcf()


def _quat_to_rpy(quat_wxyz: np.ndarray) -> tuple[float, float, float]:
    """Extract (roll, pitch, yaw) in radians from MuJoCo quaternion [w, x, y, z]."""
    w, x, y, z = float(quat_wxyz[0]), float(quat_wxyz[1]), float(quat_wxyz[2]), float(quat_wxyz[3])
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)

    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> list[float]:
    """Convert (roll, pitch, yaw) in radians to MuJoCo quaternion [w, x, y, z]."""
    cr, sr = cos(roll * 0.5), sin(roll * 0.5)
    cp, sp = cos(pitch * 0.5), sin(pitch * 0.5)
    cy, sy = cos(yaw * 0.5), sin(yaw * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def _quat_to_yaw(quat_wxyz: np.ndarray) -> float:
    """Extract planar yaw angle (radians) from MuJoCo quaternion [w, x, y, z]."""
    return _quat_to_rpy(quat_wxyz)[2]


def _quat_to_pitch_roll(quat_wxyz: np.ndarray) -> tuple[float, float]:
    """Extract (pitch_rad, roll_rad) from MuJoCo quaternion [w, x, y, z]."""
    w, x, y, z = float(quat_wxyz[0]), float(quat_wxyz[1]), float(quat_wxyz[2]), float(quat_wxyz[3])
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = asin(sinp)
    return round(pitch, 4), round(roll, 4)


def _wrap_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


@dataclass(frozen=True)
class SeedDropEvent:
    seed_index: int
    robot_id: str
    task_id: str
    target_site_id: str
    released_at_s: float
    contact_observed_at_s: float | None = None
    landing_pose: Pose2D | None = None


class MujocoPaddySimulation:
    """Genuine 3D homogeneous Paddy-Ant & rice-terrace MuJoCo implementation of `SimulationPort` (`rice.v1`)."""

    def __init__(
        self,
        sites: tuple[Site, ...] = DEFAULT_SITES,
        probe_radius_m: float = 0.28,
        substeps_per_tick: int = 25,
        inject_obstacle_at_s: float | None = None,
        obstacle_xy: tuple[float, float] = (0.75, -0.25),
        hidden_truth: dict[str, dict[SensorKind, float]] | None = None,
        trap_robot_id: str | None = None,
        trap_site_id: str = "patch_a",
        trap_radius_m: float = 0.24,
        sensor_dropout_robot_id: str | None = None,
        sensor_dropout_window_s: tuple[float, float] | None = None,
        expose_bogged_flag: bool = False,
        initial_poses: dict[str, Pose2D] | None = None,
        scenario: str = "nominal",
        num_rovers: int = 2,
        map_scale: float = 1.0,
    ) -> None:
        self.num_rovers = max(2, min(6, int(num_rovers)))
        self.map_scale = float(map_scale)
        self.rover_specs: tuple[RoverChassisSpec, ...] = make_fleet_specs(self.num_rovers)
        self._spec_by_id: dict[str, RoverChassisSpec] = {s.robot_id: s for s in self.rover_specs}
        self._ctrl_offset_by_id: dict[str, int] = {
            s.robot_id: 4 * idx for idx, s in enumerate(self.rover_specs)
        }
        self._seed_slice_by_id: dict[str, tuple[int, int]] = {}
        seed_cursor = 0
        for s in self.rover_specs:
            self._seed_slice_by_id[s.robot_id] = (seed_cursor, seed_cursor + s.initial_seeds)
            seed_cursor += s.initial_seeds
        self._total_seed_proxies = max(2, seed_cursor)

        self.world = PaddyTerraceWorld(
            sites=sites,
            probe_radius_m=probe_radius_m,
            hidden_truth=hidden_truth,
            scenario=scenario,
        )
        self.sites = self.world.sites
        self.probe_radius_m = probe_radius_m
        self.substeps_per_tick = substeps_per_tick
        self.inject_obstacle_at_s = inject_obstacle_at_s
        self.obstacle_xy = obstacle_xy
        self.trap_robot_id = trap_robot_id
        self.trap_site_id = trap_site_id
        self.trap_radius_m = trap_radius_m
        self.sensor_dropout_robot_id = sensor_dropout_robot_id
        self.sensor_dropout_window_s = sensor_dropout_window_s
        self.expose_bogged_flag = expose_bogged_flag
        self.initial_poses: dict[str, Pose2D] | None = dict(initial_poses) if initial_poses else None
        self.scenario = scenario

        mjcf_xml = compose_paddypatch_mjcf(obstacle_xy=obstacle_xy, rover_specs=self.rover_specs)
        self.model = mujoco.MjModel.from_xml_string(mjcf_xml)
        self.data = mujoco.MjData(self.model)

        self._ground_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "field_ground")
        self._hfield_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "paddy_relief_mesh")
        if self._hfield_geom_id >= 0:
            self.model.geom_group[self._hfield_geom_id] = 2
        self._seed_geom_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"seed_{i}_geom")
            for i in range(self._total_seed_proxies)
        ]
        self._qpos_adrs: dict[str, int] = {
            s.robot_id: int(
                self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{s.robot_id}_free")]
            )
            for s in self.rover_specs
        }
        self._qvel_adrs: dict[str, int] = {
            s.robot_id: int(
                self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{s.robot_id}_free")]
            )
            for s in self.rover_specs
        }
        self._planter_qpos_adr = self._qpos_adrs["planter"]
        self._scout_qpos_adr = self._qpos_adrs["scout"]
        self._planter_qvel_adr = self._qvel_adrs["planter"]
        self._scout_qvel_adr = self._qvel_adrs["scout"]
        self._body_ids: dict[str, int] = {
            s.robot_id: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, f"{s.robot_id}_body")
            for s in self.rover_specs
        }
        self._wheel_geom_ids: dict[str, list[int]] = {
            s.robot_id: [
                gid
                for w in ("fl", "fr", "rl", "rr")
                for gid in (
                    mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"{s.robot_id}_w_{w}"),
                    mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"{s.robot_id}_wheel_{w}"),
                )
                if gid >= 0
            ]
            for s in self.rover_specs
        }
        for rid, gids in self._wheel_geom_ids.items():
            for gid in gids:
                if gid >= 0:
                    self.model.geom_priority[gid] = 1
        self._seed_qpos_adrs = [
            int(self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"seed_{i}_free")])
            for i in range(self._total_seed_proxies)
        ]
        self._seed_qvel_adrs = [
            int(self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"seed_{i}_free")])
            for i in range(self._total_seed_proxies)
        ]

        # Optional active farming end-effector & rigid docking/latching actuator IDs from PR #24
        self._dibber_act_ids: dict[str, int] = {}
        self._weeder_l_act_ids: dict[str, int] = {}
        self._weeder_r_act_ids: dict[str, int] = {}
        self._dock_pitch_act_ids: dict[str, int] = {}
        self._latch_act_ids: dict[str, int] = {}
        self._spade_act_ids: dict[str, int] = {}
        for s in self.rover_specs:
            rid = s.robot_id
            self._dibber_act_ids[rid] = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{rid}_dibber_act"
            )
            self._weeder_l_act_ids[rid] = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{rid}_weeder_l_act"
            )
            self._weeder_r_act_ids[rid] = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{rid}_weeder_r_act"
            )
            self._dock_pitch_act_ids[rid] = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{rid}_dock_pitch_act"
            )
            self._latch_act_ids[rid] = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{rid}_latch_act"
            )
            self._spade_act_ids[rid] = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{rid}_spade_act"
            )

        # Runtime-configurable vehicle sizing parameters for live Bekker-Wong wheel-mud sinkage physics
        self.robot_mass_kg: dict[str, float] = {
            s.robot_id: (4.2 if s.can_plant else 3.6) for s in self.rover_specs
        }
        self.robot_wheel_radius_m: dict[str, float] = {
            s.robot_id: s.wheel_radius_m for s in self.rover_specs
        }
        self.robot_wheel_width_m: dict[str, float] = {
            s.robot_id: 0.040 for s in self.rover_specs
        }
        self.latest_terrain_points: dict[str, ContinuousTerrainPoint] = {}
        self.latest_terramechanics: dict[str, TerramechanicsSinkageState] = {}
        self.latest_slip_ratios: dict[str, float] = {s.robot_id: 0.08 for s in self.rover_specs}
        self.latest_wheel_heights: dict[str, tuple[float, float, float, float]] = {}
        self.latest_wheel_sinkages: dict[str, tuple[float, float, float, float]] = {}
        self.latest_induced_tilt: dict[str, tuple[float, float]] = {}
        self._smoothed_pose_3d: dict[str, tuple[float, float, float]] = {}
        self._physics_z_quat: dict[str, tuple[float, list[float]]] = {}
        self._latest_physics_yaw: dict[str, float] = {}
        self._latest_front_range_m: dict[str, float | None] = {}
        self.mud_rut_history: list[tuple[str, float, float, float, float]] = []

        self._seed: int = 7
        self._episode_id: str = "mujoco-episode-7"
        self._seeds_remaining: dict[str, int] = {
            s.robot_id: s.initial_seeds for s in self.rover_specs
        }
        self._active_drops: list[SeedDropEvent] = []
        self._completed_drops: list[SeedDropEvent] = []
        self._completed_recoveries: list[RecoveryFeedback] = []
        self._obstacle_injected: bool = False
        self._hfield_dirty: bool = True
        self._populate_hfield_from_continuous_terrain()
        self._sync_3d_terrain_visuals_from_world()
        self._bogged_origins: dict[str, tuple[float, float]] = {}
        self._sensed_bogged_robots: set[str] = set()
        self._wheel_spin_stall_ticks: dict[str, int] = {s.robot_id: 0 for s in self.rover_specs}
        self._last_commanded_drive: dict[str, tuple[float, float]] = {
            s.robot_id: (0.0, 0.0) for s in self.rover_specs
        }
        self._rescued_robots: set[str] = set()
        self._active_latched_pair: tuple[str, str] | None = None
        self._detach_backoff_ticks: dict[str, int] = {}
        self._latched_substeps: int = 0
        self._peak_latch_force_n: float = 0.0

    def inject_mud_fault(self, robot_id: str) -> None:
        """Trap `robot_id` at its current pose via session-authorized fault injection."""
        if robot_id not in self._spec_by_id:
            return
        self._rescued_robots.discard(robot_id)
        r_xyz = self.data.sensor(f"{robot_id}_pos").data
        self._bogged_origins[robot_id] = (float(r_xyz[0]), float(r_xyz[1]))

    def inject_dynamic_obstacle(self, x_m: float, y_m: float) -> None:
        """Place the dynamic obstacle at `(x_m, y_m)` via session-authorized operator request."""
        self.obstacle_xy = (float(x_m), float(y_m))
        self.data.mocap_pos[0] = [self.obstacle_xy[0], self.obstacle_xy[1], 0.12]
        self._obstacle_injected = True
        mujoco.mj_forward(self.model, self.data)

    def _populate_hfield_from_continuous_terrain(self) -> None:
        """Populate MuJoCo's 3D `paddy_micro_relief` heightfield directly from `ContinuousPaddyTerrain`."""
        if self.model.nhfield <= 0:
            return
        nrow = int(self.model.hfield_nrow[0])
        ncol = int(self.model.hfield_ncol[0])
        xs = np.linspace(-0.30, 3.00, ncol)
        ys = np.linspace(-0.92, 0.92, nrow)
        grid = np.zeros((nrow, ncol), dtype=np.float32)
        for iy, y in enumerate(ys):
            for ix, x in enumerate(xs):
                z, _, _ = self.world.continuous_terrain.elevation_and_gradient(float(x), float(y))
                grid[iy, ix] = float(z)
        z_min = float(grid.min())
        z_max = float(grid.max())
        span = max(1e-5, z_max - z_min)
        norm_grid = (grid - z_min) / span
        self.model.hfield_data[:] = norm_grid.flatten()
        self._hfield_dirty = True

    def _sync_3d_terrain_visuals_from_world(self) -> None:
        """Synchronize 3D `bed_patch_a..f` and `puddle_patch_a..f` geoms with `self.world._hidden_site_truth`."""
        base_xy_by_suffix: dict[str, tuple[float, float]] = {
            "a": (0.85, -0.50),
            "b": (1.60, -0.50),
            "c": (2.35, -0.50),
            "d": (1.60, 0.45),
            "e": (1.22, -0.02),
            "f": (1.98, -0.02),
        }
        if self._ground_geom_id >= 0:
            self.model.geom_group[self._ground_geom_id] = 2

        for idx, (suffix, default_xy) in enumerate(base_xy_by_suffix.items()):
            patch_id = f"patch_{suffix}"
            if patch_id in self.sites:
                cx = float(self.sites[patch_id].center.x_m)
                cy = float(self.sites[patch_id].center.y_m)
            else:
                cx, cy = default_xy

            pt = self.world.continuous_terrain.evaluate(cx, cy)
            truth = self.world._hidden_site_truth.get(patch_id)
            if truth is not None:
                moisture = float(truth[SensorKind.MOISTURE])
                firmness = float(truth[SensorKind.FIRMNESS])
            else:
                moisture = float(pt.moisture)
                firmness = float(pt.firmness)

            rng = random.Random(int(self._seed) * 37 + idx * 13)
            jx = rng.uniform(-0.018, 0.018)
            jy = rng.uniform(-0.018, 0.018)
            bed_radius = 0.24 + 0.025 * max(0.0, moisture - 0.50) + rng.uniform(-0.008, 0.008)

            is_active_trap = self.trap_robot_id is not None and patch_id == self.trap_site_id
            is_mud = (moisture > 0.80) or (firmness < 0.25) or is_active_trap

            bed_gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"bed_patch_{suffix}")
            puddle_gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"puddle_patch_{suffix}")
            ring_sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f"ring_patch_{suffix}")

            if bed_gid >= 0:
                self.model.geom_group[bed_gid] = 2
                self.model.geom_matid[bed_gid] = -1
                if is_mud:
                    self.model.geom_rgba[bed_gid] = [0.20, 0.14, 0.09, 0.95]
                    self.model.geom_pos[bed_gid] = [cx + 0.35 * jx, cy + 0.35 * jy, 0.003]
                else:
                    self.model.geom_rgba[bed_gid] = [0.46, 0.37, 0.25, 0.92]
                    self.model.geom_pos[bed_gid] = [cx + 0.25 * jx, cy + 0.25 * jy, 0.003]
                self.model.geom_size[bed_gid, 0] = round(bed_radius, 4)
                self.model.geom_size[bed_gid, 1] = 0.003

            if puddle_gid >= 0:
                self.model.geom_group[puddle_gid] = 2
                self.model.geom_matid[puddle_gid] = -1
                if is_mud:
                    puddle_r = max(
                        0.14,
                        min(
                            0.24,
                            0.12 + 0.07 * moisture + 0.14 * pt.mud_depth_m + rng.uniform(-0.010, 0.010),
                        ),
                    )
                    self.model.geom_rgba[puddle_gid] = [0.14, 0.42, 0.54, 0.68]
                    self.model.geom_pos[puddle_gid] = [cx + jx, cy + jy, 0.0022]
                    self.model.geom_size[puddle_gid, 0] = round(puddle_r, 4)
                    self.model.geom_size[puddle_gid, 1] = 0.002
                else:
                    self.model.geom_rgba[puddle_gid] = [0.14, 0.42, 0.54, 0.0]
                    self.model.geom_pos[puddle_gid] = [cx, cy, -1.0]
                    self.model.geom_size[puddle_gid, 0] = 0.05
                    self.model.geom_size[puddle_gid, 1] = 0.001

            if ring_sid >= 0:
                if not is_mud and patch_id in self.world._hidden_site_truth:
                    self.model.site_rgba[ring_sid] = [0.85, 0.72, 0.25, 0.55]
                else:
                    self.model.site_rgba[ring_sid] = [0.85, 0.72, 0.25, 0.0]

    @property
    def completed_drops(self) -> tuple[SeedDropEvent, ...]:
        """Independent physics record of released seeds and verified ground contacts."""
        return tuple(self._completed_drops)

    @property
    def completed_recoveries(self) -> tuple[RecoveryFeedback, ...]:
        """Independent physics record of verified rigid-latch extractions."""
        return tuple(self._completed_recoveries)

    @property
    def obstacle_injected(self) -> bool:
        return self._obstacle_injected

    def _update_rover_terramechanics(self) -> None:
        """Evaluate ContinuousPaddyTerrain and 4-wheel Bekker-Wong mud sinkage at each rover's live 3D pose."""
        wx = 0.085  # Half-wheelbase (m)
        for spec in self.rover_specs:
            rid = spec.robot_id
            r_xyz = self.data.sensor(f"{rid}_pos").data
            r_quat = self.data.sensor(f"{rid}_quat").data
            rx, ry = float(r_xyz[0]), float(r_xyz[1])
            yaw = _quat_to_yaw(r_quat)
            cos_y, sin_y = cos(yaw), sin(yaw)
            ht = spec.half_track_m

            pt = self.world.evaluate_continuous_point(rx, ry)
            # Local physical wheel slip ratio increases continuously with mud moisture & softness
            local_slip = max(
                0.06,
                min(
                    0.92,
                    0.08 + 0.58 * (pt.moisture**1.8) * ((1.0 - pt.firmness) ** 1.15),
                ),
            )
            sink_state = evaluate_bekker_terramechanics(
                spec,
                moisture=pt.moisture,
                firmness=pt.firmness,
                mud_depth_m=pt.mud_depth_m,
                slope_magnitude=pt.slope_magnitude,
                slip_ratio=local_slip,
                mass_override_kg=self.robot_mass_kg.get(rid, 4.0),
                wheel_radius_override_m=self.robot_wheel_radius_m.get(rid, spec.wheel_radius_m),
                wheel_width_override_m=self.robot_wheel_width_m.get(rid, 0.040),
            )

            is_bogged = rid in self._bogged_origins or sink_state.bellied_out
            wheel_offsets = (
                (wx, ht),    # fl
                (wx, -ht),   # fr
                (-wx, ht),   # rl
                (-wx, -ht),  # rr
            )
            wheel_heights: list[float] = []
            wheel_sinkages: list[float] = []
            for w_idx, (dx, dy) in enumerate(wheel_offsets):
                wx_w = rx + dx * cos_y - dy * sin_y
                wy_w = ry + dx * sin_y + dy * cos_y
                w_pt = self.world.evaluate_continuous_point(wx_w, wy_w)
                depth_ratio = w_pt.mud_depth_m / max(0.03, pt.mud_depth_m)
                base_w_sink = sink_state.sinkage_z_m * depth_ratio
                if is_bogged:
                    bog_extra = 0.016 + 0.035 * w_pt.mud_depth_m
                    pocket_asym = 0.0055 * sin(3.9 * wx_w - 3.1 * wy_w + 0.85 * w_idx + 0.3 * self._seed)
                else:
                    bog_extra = 0.012 * max(0.0, w_pt.moisture - 0.75) * (1.0 - w_pt.firmness)
                    pocket_asym = 0.0020 * w_pt.moisture * sin(3.9 * wx_w - 3.1 * wy_w + 0.85 * w_idx)
                w_sink = max(0.001, base_w_sink + bog_extra + pocket_asym)
                z_wheel = w_pt.elevation_m - w_sink
                wheel_sinkages.append(round(w_sink, 5))
                wheel_heights.append(round(z_wheel, 5))

            z_fl, z_fr, z_rl, z_rr = wheel_heights
            raw_pitch = atan2((z_rl + z_rr) * 0.5 - (z_fl + z_fr) * 0.5, 2.0 * wx)
            raw_roll = atan2((z_fl + z_rl) * 0.5 - (z_fr + z_rr) * 0.5, 2.0 * ht)
            default_m = 4.2 if spec.can_plant else 3.6
            is_custom_sizing = (
                abs(self.robot_mass_kg.get(rid, default_m) - default_m) > 0.3
                or abs(self.robot_wheel_radius_m.get(rid, 0.058) - 0.058) > 0.003
            )
            pitch_cap = 0.22 if is_custom_sizing else 0.11
            induced_pitch = max(-pitch_cap, min(pitch_cap, raw_pitch))
            induced_roll = max(-0.22, min(0.22, raw_roll))

            if is_custom_sizing or is_bogged:
                weight_transfer = max(
                    0.72,
                    1.0 - 0.18 * abs(induced_pitch) - 0.18 * abs(induced_roll),
                )
                local_slip = min(0.96, local_slip / weight_transfer)
                sink_state = replace(
                    sink_state,
                    max_thrust_n=round(sink_state.max_thrust_n * weight_transfer, 2),
                    speed_penalty_factor=round(sink_state.speed_penalty_factor * weight_transfer, 4),
                )
            if sink_state.bellied_out:
                local_slip = 0.96

            self.latest_terrain_points[rid] = pt
            self.latest_terramechanics[rid] = sink_state
            self.latest_slip_ratios[rid] = round(local_slip, 4)
            self.latest_wheel_heights[rid] = (z_fl, z_fr, z_rl, z_rr)
            self.latest_wheel_sinkages[rid] = (
                wheel_sinkages[0],
                wheel_sinkages[1],
                wheel_sinkages[2],
                wheel_sinkages[3],
            )
            self.latest_induced_tilt[rid] = (round(induced_roll, 4), round(induced_pitch, 4))

    def _apply_3d_terrain_and_mud_sinkage_pose(self, *, snap: bool = False) -> None:
        """Apply 4-wheel mud sinkage (z < 0) and chassis pitch/roll tilt to qpos for 3D viewer and IMU observations."""
        alpha = 1.0 if snap else 0.45
        for spec in self.rover_specs:
            rid = spec.robot_id
            qadr = self._qpos_adrs[rid]
            sink = self.latest_terramechanics.get(rid)
            w_heights = self.latest_wheel_heights.get(rid)
            induced_roll, induced_pitch = self.latest_induced_tilt.get(rid, (0.0, 0.0))
            is_bogged = rid in self._bogged_origins or (sink is not None and sink.bellied_out)

            if w_heights is not None:
                z_wheels_mean = 0.25 * sum(w_heights)
                if is_bogged:
                    target_z = max(0.036, min(0.068, spec.initial_xyz[2] + 0.45 * z_wheels_mean - 0.008))
                else:
                    target_z = max(0.040, min(0.068, spec.initial_xyz[2] + 0.40 * z_wheels_mean))
            elif sink is not None and sink.bellied_out:
                target_z = max(0.036, spec.initial_xyz[2] - 0.55 * sink.sinkage_z_m)
            else:
                target_z = spec.initial_xyz[2]

            prev_z, prev_r, prev_p = self._smoothed_pose_3d.get(rid, (target_z, induced_roll, induced_pitch))
            z_now = prev_z + alpha * (target_z - prev_z)
            roll_now = prev_r + alpha * (induced_roll - prev_r)
            pitch_now = prev_p + alpha * (induced_pitch - prev_p)
            self._smoothed_pose_3d[rid] = (z_now, roll_now, pitch_now)

            _, _, current_yaw = _quat_to_rpy(self.data.qpos[qadr + 3 : qadr + 7])
            self.data.qpos[qadr + 2] = round(z_now, 5)
            self.data.qpos[qadr + 3 : qadr + 7] = _rpy_to_quat(roll_now, pitch_now, current_yaw)

    def reset(self, seed: int) -> ObservationFrame:
        self._seed = int(seed)
        self._episode_id = f"mujoco-ep-{self._seed}"
        if self.scenario == "randomized":
            rand_spec = generate_randomized_episode_spec(self._seed)
            self.world._hidden_site_truth = {
                site_id: dict(vals) for site_id, vals in rand_spec.hidden_truth.items()
            }
            self.initial_poses = {
                "planter": rand_spec.planter_spawn,
                "scout": rand_spec.scout_spawn,
            }
            self.trap_robot_id = rand_spec.trap_robot_id
            self.trap_site_id = rand_spec.trap_site_id or "patch_d"
            self.trap_radius_m = rand_spec.trap_radius_m
            self.obstacle_xy = rand_spec.obstacle_xy
            self.inject_obstacle_at_s = rand_spec.inject_obstacle_at_s
        self.world.reset(self._seed)
        self._populate_hfield_from_continuous_terrain()
        self._sync_3d_terrain_visuals_from_world()
        self._seeds_remaining = {
            s.robot_id: s.initial_seeds for s in self.rover_specs
        }
        self._active_drops = []
        self._completed_drops = []
        self._completed_recoveries = []
        self._obstacle_injected = False
        self.mud_rut_history = []
        self._smoothed_pose_3d = {}
        self._physics_z_quat = {}
        self._bogged_origins = {}
        self._sensed_bogged_robots = set()
        self._wheel_spin_stall_ticks = {s.robot_id: 0 for s in self.rover_specs}
        self._last_commanded_drive = {s.robot_id: (0.0, 0.0) for s in self.rover_specs}
        self._rescued_robots = set()
        self._active_latched_pair = None
        self._detach_backoff_ticks = {}
        self._latched_substeps = 0
        self._peak_latch_force_n = 0.0
        for gids in self._wheel_geom_ids.values():
            for gid in gids:
                if gid >= 0:
                    self.model.geom_friction[gid, 0] = 1.4

        mujoco.mj_resetData(self.model, self.data)
        for spec in self.rover_specs:
            rid = spec.robot_id
            rx, ry, rz = spec.initial_xyz
            r_quat = [1.0, 0.0, 0.0, 0.0]
            if self.initial_poses and rid in self.initial_poses:
                pose_override = self.initial_poses[rid]
                rx, ry = pose_override.x_m, pose_override.y_m
                r_quat = _rpy_to_quat(0.0, 0.0, pose_override.yaw_rad)
            qadr = self._qpos_adrs[rid]
            self.data.qpos[qadr : qadr + 7] = [rx, ry, rz, *r_quat]
            self._physics_z_quat[rid] = (float(rz), list(r_quat))
        self.data.mocap_pos[0] = [self.obstacle_xy[0], self.obstacle_xy[1], -5.0]
        self._sync_carried_seeds()
        mujoco.mj_forward(self.model, self.data)
        for spec in self.rover_specs:
            rid = spec.robot_id
            self._latest_physics_yaw[rid] = _quat_to_yaw(self.data.sensor(f"{rid}_quat").data)
            self._latest_front_range_m[rid] = self._read_min_front_range_m(rid)
        self._update_rover_terramechanics()
        self._apply_3d_terrain_and_mud_sinkage_pose(snap=True)
        mujoco.mj_forward(self.model, self.data)
        return self._build_observation_frame(plant_feedback=(), recovery_feedback=())

    def _sync_carried_seeds(self) -> None:
        """Keep unreleased seeds inside each planter unit's hopper frame."""
        released_indices = {d.seed_index for d in self._active_drops} | {
            d.seed_index for d in self._completed_drops
        }
        for spec in self.rover_specs:
            start_idx, end_idx = self._seed_slice_by_id[spec.robot_id]
            if end_idx <= start_idx:
                continue
            r_xyz = self.data.sensor(f"{spec.robot_id}_pos").data
            px, py, pz = float(r_xyz[0]), float(r_xyz[1]), float(r_xyz[2])
            for local_i, idx in enumerate(range(start_idx, end_idx)):
                if idx not in released_indices:
                    qadr = self._seed_qpos_adrs[idx]
                    vadr = self._seed_qvel_adrs[idx]
                    self.data.qpos[qadr : qadr + 3] = [px, py, pz + 0.12 + 0.05 * local_i]
                    self.data.qpos[qadr + 3 : qadr + 7] = [1.0, 0.0, 0.0, 0.0]
                    self.data.qvel[vadr : vadr + 6] = 0.0

    def _check_seed_contact(self, seed_index: int) -> Pose2D | None:
        """Return landing Pose2D if seed_index geom is physically touching field_ground or paddy_relief_mesh."""
        target_geom = self._seed_geom_ids[seed_index]
        ground_geoms = {self._ground_geom_id}
        if self._hfield_geom_id >= 0:
            ground_geoms.add(self._hfield_geom_id)
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            if target_geom in (c.geom1, c.geom2) and ((c.geom1 in ground_geoms) or (c.geom2 in ground_geoms)):
                qadr = self._seed_qpos_adrs[seed_index]
                sx, sy = float(self.data.qpos[qadr]), float(self.data.qpos[qadr + 1])
                return Pose2D(round(sx, 4), round(sy, 4), 0.0)
        return None

    def _update_mud_trap_state(self) -> None:
        """Check if configured trap_robot_id (or a seed-laden helper entering an extreme mud pan) is bogged."""
        if self.trap_robot_id is None or self.trap_site_id not in self.sites:
            return
        site = self.sites[self.trap_site_id]
        all_rids = tuple(s.robot_id for s in self.rover_specs)
        candidates = all_rids if self.trap_robot_id == "any" else (self.trap_robot_id,)
        if not self._bogged_origins:
            for cand_id in candidates:
                if cand_id not in self._spec_by_id or cand_id in self._rescued_robots:
                    continue
                r_xyz = self.data.sensor(f"{cand_id}_pos").data
                rx, ry = float(r_xyz[0]), float(r_xyz[1])
                if hypot(rx - site.center.x_m, ry - site.center.y_m) <= self.trap_radius_m:
                    self._bogged_origins[cand_id] = (rx, ry)
                    break

        if self._bogged_origins:
            primary_victim = next(iter(self._bogged_origins.keys()))
            site_pt = self.world.evaluate_continuous_point(site.center.x_m, site.center.y_m)
            if site_pt.firmness < 0.18:
                victim_origin = self._bogged_origins[primary_victim]
                for other_id in all_rids:
                    if (
                        other_id == primary_victim
                        or other_id in self._bogged_origins
                        or other_id in self._rescued_robots
                        or self._seeds_remaining.get(other_id, 0) <= 0
                    ):
                        continue
                    o_xyz = self.data.sensor(f"{other_id}_pos").data
                    ox, oy = float(o_xyz[0]), float(o_xyz[1])
                    if (
                        hypot(ox - site.center.x_m, oy - site.center.y_m) <= self.trap_radius_m
                        or hypot(ox - victim_origin[0], oy - victim_origin[1]) <= 0.32
                    ):
                        self._bogged_origins[other_id] = (ox, oy)

    def step(self, commands: tuple[RobotCommand, ...]) -> ObservationFrame:
        now_s = round(float(self.data.time), 4)
        cmd_by_robot = {c.robot_id: c for c in commands}

        if self._physics_z_quat:
            for spec in self.rover_specs:
                rid = spec.robot_id
                if rid in self._physics_z_quat:
                    qadr = self._qpos_adrs[rid]
                    pz, pquat = self._physics_z_quat[rid]
                    self.data.qpos[qadr + 2] = pz
                    self.data.qpos[qadr + 3 : qadr + 7] = pquat
            mujoco.mj_forward(self.model, self.data)

        if (
            self.inject_obstacle_at_s is not None
            and not self._obstacle_injected
            and now_s >= self.inject_obstacle_at_s
        ):
            self.data.mocap_pos[0] = [self.obstacle_xy[0], self.obstacle_xy[1], 0.12]
            self._obstacle_injected = True
            mujoco.mj_forward(self.model, self.data)

        self._update_rover_terramechanics()
        self._update_mud_trap_state()
        self.data.ctrl[:] = 0.0
        self.data.xfrc_applied[:] = 0.0

        active_rescue_cmd: RobotCommand | None = None
        self._last_commanded_drive = {s.robot_id: (0.0, 0.0) for s in self.rover_specs}
        for spec in self.rover_specs:
            robot_id = spec.robot_id
            ctrl_offset = self._ctrl_offset_by_id[robot_id]
            cmd = cmd_by_robot.get(robot_id)
            if cmd is None or not (cmd.issued_at_s <= now_s < cmd.expires_at_s):
                continue
            self._last_commanded_drive[robot_id] = (cmd.linear_m_s, cmd.angular_rad_s)

            dib_id = self._dibber_act_ids.get(robot_id, -1)
            wl_id = self._weeder_l_act_ids.get(robot_id, -1)
            wr_id = self._weeder_r_act_ids.get(robot_id, -1)
            pitch_id = self._dock_pitch_act_ids.get(robot_id, -1)
            latch_id = self._latch_act_ids.get(robot_id, -1)
            spade_id = self._spade_act_ids.get(robot_id, -1)

            if self._detach_backoff_ticks.get(robot_id, 0) > 0:
                self._detach_backoff_ticks[robot_id] -= 1
                w_back = compute_4wheel_skid_steer(spec, -0.42, -1.25)
                self.data.ctrl[ctrl_offset : ctrl_offset + 4] = w_back
                continue

            if cmd.recovery_target_id in self._spec_by_id and cmd.recovery_target_id != robot_id and cmd.recovery_target_id not in self._rescued_robots:
                active_rescue_cmd = cmd
                continue

            if pitch_id >= 0:
                self.data.ctrl[pitch_id] = cmd.dock_pitch_rad
            if latch_id >= 0:
                self.data.ctrl[latch_id] = cmd.latch_rad
            if spade_id >= 0:
                self.data.ctrl[spade_id] = cmd.spade_rad

            if cmd.release_seed:
                if (
                    spec.can_plant
                    and self._seeds_remaining[robot_id] > 0
                    and cmd.target_site_id in self.sites
                    and cmd.linear_m_s == 0.0
                    and cmd.angular_rad_s == 0.0
                ):
                    if dib_id >= 0:
                        self.data.ctrl[dib_id] = -0.048  # Plunge vertical seed dibber into soil
                    start_idx, _ = self._seed_slice_by_id[robot_id]
                    seed_idx = start_idx + (spec.initial_seeds - self._seeds_remaining[robot_id])
                    self._seeds_remaining[robot_id] -= 1
                    r_xyz = self.data.sensor(f"{robot_id}_pos").data
                    px, py = float(r_xyz[0]), float(r_xyz[1])
                    qadr = self._seed_qpos_adrs[seed_idx]
                    vadr = self._seed_qvel_adrs[seed_idx]
                    self.data.qpos[qadr : qadr + 3] = [px, py, spec.hopper_drop_z_m]
                    self.data.qpos[qadr + 3 : qadr + 7] = [1.0, 0.0, 0.0, 0.0]
                    self.data.qvel[vadr : vadr + 6] = [0.0, 0.0, -0.08, 0.0, 0.0, 0.0]
                    self._active_drops.append(
                        SeedDropEvent(
                            seed_index=seed_idx,
                            robot_id=robot_id,
                            task_id=cmd.task_id,
                            target_site_id=cmd.target_site_id,
                            released_at_s=now_s,
                        )
                    )
            else:
                # Apply analytical Bekker-Wong wheel-mud traction, slip churn & belly-out immobilization physics
                sink_state = self.latest_terramechanics.get(robot_id)
                slip_ratio = self.latest_slip_ratios.get(robot_id, 0.08)
                default_m = 4.2 if spec.can_plant else 3.6
                is_custom_sizing = (
                    abs(self.robot_mass_kg.get(robot_id, default_m) - default_m) > 0.3
                    or abs(self.robot_wheel_radius_m.get(robot_id, 0.058) - 0.058) > 0.003
                )
                is_bogged_now = robot_id in self._bogged_origins and self._active_latched_pair is None
                for gid in self._wheel_geom_ids.get(robot_id, ()):
                    if gid >= 0:
                        self.model.geom_friction[gid, 0] = 0.002 if is_bogged_now else 1.4
                if is_bogged_now:
                    # Keep wheel servos spinning against low mud friction + viscous suction drag
                    traction_factor = 1.0
                elif sink_state is not None:
                    if sink_state.bellied_out:
                        traction_factor = 0.0
                    elif is_custom_sizing:
                        traction_factor = sink_state.speed_penalty_factor
                    else:
                        traction_factor = 0.94 + 0.06 * sink_state.speed_penalty_factor
                else:
                    traction_factor = 1.0

                effective_linear = cmd.linear_m_s * traction_factor
                mud_fishtail = (
                    0.15 * slip_ratio * sin(9.5 * now_s + (0.0 if spec.can_plant else 2.1))
                    if is_custom_sizing and abs(cmd.linear_m_s) > 0.08
                    else 0.0
                )
                effective_angular = (
                    cmd.angular_rad_s * (0.50 + 0.50 * traction_factor) + mud_fishtail
                    if traction_factor > 0.0
                    else 0.0
                )
                w_fl, w_fr, w_rl, w_rr = compute_4wheel_skid_steer(
                    spec, effective_linear, effective_angular
                )
                self.data.ctrl[ctrl_offset + 0] = w_fl
                self.data.ctrl[ctrl_offset + 1] = w_fr
                self.data.ctrl[ctrl_offset + 2] = w_rl
                self.data.ctrl[ctrl_offset + 3] = w_rr

                # Spin counter-rotating underbelly finger weeders when cruising forward
                if abs(effective_linear) > 0.05:
                    if wl_id >= 0:
                        self.data.ctrl[wl_id] = 12.0
                    if wr_id >= 0:
                        self.data.ctrl[wr_id] = -12.0

                # Plunge soil-penetrometer dibber when inside probe radius of a site
                r_xyz = self.data.sensor(f"{robot_id}_pos").data
                rx, ry = float(r_xyz[0]), float(r_xyz[1])
                if dib_id >= 0:
                    near_site = any(
                        s.permitted and hypot(rx - s.center.x_m, ry - s.center.y_m) <= self.probe_radius_m
                        for s in self.sites.values()
                    )
                    self.data.ctrl[dib_id] = -0.045 if near_site else 0.0

                # Record 3D wheel-mud rut history along rover path
                pt = self.latest_terrain_points.get(robot_id)
                if pt is not None and sink_state is not None and abs(cmd.linear_m_s) > 0.02:
                    if not self.mud_rut_history or hypot(
                        rx - self.mud_rut_history[-1][1], ry - self.mud_rut_history[-1][2]
                    ) >= 0.09:
                        self.mud_rut_history.append(
                            (robot_id, round(rx, 3), round(ry, 3), pt.elevation_m, sink_state.sinkage_z_m)
                        )
                        if len(self.mud_rut_history) > 90:
                            self.mud_rut_history = self.mud_rut_history[-90:]

        new_feedback: list[PlantFeedback] = []
        new_recovery: list[RecoveryFeedback] = []

        for substep_i in range(self.substeps_per_tick):
            self._sync_carried_seeds()
            if substep_i % 5 == 0:
                for spec in self.rover_specs:
                    rid = spec.robot_id
                    sink = self.latest_terramechanics.get(rid)
                    if sink is not None and sink.bellied_out:
                        qadr = self._qpos_adrs[rid]
                        self.data.qpos[qadr + 2] = max(0.042, spec.initial_xyz[2] - 0.55 * sink.sinkage_z_m)

            # Apply physical mud suction drag force to any unlatched bogged rover while allowing wheel spin
            for bog_id in list(self._bogged_origins.keys()):
                if self._active_latched_pair is None:
                    for gid in self._wheel_geom_ids.get(bog_id, ()):
                        if gid >= 0:
                            self.model.geom_friction[gid, 0] = 0.002
                    vadr = self._qvel_adrs[bog_id]
                    bid = self._body_ids[bog_id]
                    self.data.xfrc_applied[bid, 0] = -35.0 * float(self.data.qvel[vadr])
                    self.data.xfrc_applied[bid, 1] = -35.0 * float(self.data.qvel[vadr + 1])
                    self.data.qvel[vadr : vadr + 6] *= 0.02

            # Execute closed-loop physical rigid-docking & tandem extraction if a RESCUE command is active
            if active_rescue_cmd is not None and active_rescue_cmd.recovery_target_id is not None:
                helper_id = active_rescue_cmd.robot_id
                target_id = active_rescue_cmd.recovery_target_id
                helper_spec = self._spec_by_id[helper_id]
                target_spec = self._spec_by_id[target_id]
                h_offset = self._ctrl_offset_by_id[helper_id]
                t_offset = self._ctrl_offset_by_id[target_id]
                h_latch_id = self._latch_act_ids[helper_id]
                h_pitch_id = self._dock_pitch_act_ids[helper_id]
                h_spade_id = self._spade_act_ids[helper_id]

                front_pos = self.data.sensor(f"{helper_id}_front_dock_pos").data
                rear_pos = self.data.sensor(f"{target_id}_rear_dock_pos").data
                h_quat = self.data.sensor(f"{helper_id}_quat").data
                h_yaw = _quat_to_yaw(h_quat)

                dx = float(rear_pos[0] - front_pos[0])
                dy = float(rear_pos[1] - front_pos[1])
                dock_dist = hypot(dx, dy, float(rear_pos[2] - front_pos[2]))

                self.data.ctrl[h_latch_id] = max(0.55, active_rescue_cmd.latch_rad)
                latched = update_rigid_dock_latch(
                    self.model,
                    self.data,
                    rescuer_id=helper_id,
                    target_id=target_id,
                    capture_radius_m=0.022,
                )

                if not latched:
                    # Phase 1: SE(2) Terminal Funnel Alignment toward victim's rear strike bar (until <= 22 mm)
                    t_quat = self.data.sensor(f"{target_id}_quat").data
                    t_yaw = _quat_to_yaw(t_quat)
                    t_fx, t_fy = cos(t_yaw), sin(t_yaw)
                    rel_x = float(front_pos[0] - rear_pos[0])
                    rel_y = float(front_pos[1] - rear_pos[1])
                    along_fwd = rel_x * t_fx + rel_y * t_fy
                    t_lx, t_ly = -t_fy, t_fx
                    lat_proj = rel_x * t_lx + rel_y * t_ly
                    # Choose the lateral pass side (+1 or -1) that stays inside the open central terrace corridor
                    cand_pos_y = float(rear_pos[1]) + 0.44 * t_ly
                    cand_neg_y = float(rear_pos[1]) - 0.44 * t_ly
                    if abs(cand_pos_y - (-0.05)) + 0.12 < abs(cand_neg_y - (-0.05)):
                        lat_sign = 1.0
                    elif abs(cand_neg_y - (-0.05)) + 0.12 < abs(cand_pos_y - (-0.05)):
                        lat_sign = -1.0
                    else:
                        lat_sign = 1.0 if lat_proj >= 0.0 else -1.0
                    if along_fwd > 0.02 and dock_dist > 0.14:
                        # Helper is in the front hemisphere of victim; pass wide laterally (>= 0.44 m) to clear wheels
                        stage_x = float(rear_pos[0]) - 0.04 * t_fx + 0.44 * lat_sign * t_lx
                        stage_y = float(rear_pos[1]) - 0.04 * t_fy + 0.44 * lat_sign * t_ly
                        aim_dx = stage_x - float(front_pos[0])
                        aim_dy = stage_y - float(front_pos[1])
                    elif along_fwd > -0.20 and abs(lat_proj) > 0.09 and dock_dist > 0.10:
                        # Helper has cleared the victim's side; line up behind the rear strike bar before closing in
                        stage_x = float(rear_pos[0]) - 0.30 * t_fx
                        stage_y = float(rear_pos[1]) - 0.30 * t_fy
                        aim_dx = stage_x - float(front_pos[0])
                        aim_dy = stage_y - float(front_pos[1])
                    else:
                        aim_dx, aim_dy = dx, dy
                    heading_err = _wrap_angle(atan2(aim_dy, aim_dx) - h_yaw)
                    ang_cmd = max(-1.35, min(1.35, heading_err * 2.6))
                    lin_cmd = min(0.35, max(0.10, dock_dist * 0.90)) if abs(heading_err) < 0.45 else 0.02
                    self.data.ctrl[h_pitch_id] = 0.0
                    self.data.ctrl[h_spade_id] = 0.0
                    w_h = compute_4wheel_skid_steer(helper_spec, lin_cmd, ang_cmd)
                    self.data.ctrl[h_offset : h_offset + 4] = w_h
                else:
                    # Phase 2 & 3: Rigid Cam-Latch Engaged -> Grounded Pitch-Lift + 8-Wheel Reverse Tow
                    self._active_latched_pair = (helper_id, target_id)
                    self._latched_substeps += 1
                    technique = active_rescue_cmd.recovery_technique or "direct_pry_and_pull"
                    for gid in self._wheel_geom_ids.get(target_id, ()):
                        if gid >= 0:
                            self.model.geom_friction[gid, 0] = 1.4
                    if technique == "dynamic_yaw_shear_and_pitch_pump":
                        self.data.ctrl[h_pitch_id] = 0.08
                        self.data.ctrl[h_spade_id] = 0.0
                        w_rev_h = compute_4wheel_skid_steer(helper_spec, -0.40, 0.0)
                        w_rev_t = compute_4wheel_skid_steer(target_spec, -0.40, 0.0)
                    else:
                        self.data.ctrl[h_pitch_id] = min(0.15, max(0.08, active_rescue_cmd.dock_pitch_rad))
                        self.data.ctrl[h_spade_id] = 0.0
                        w_rev_h = compute_4wheel_skid_steer(helper_spec, -0.40, 0.0)
                        w_rev_t = compute_4wheel_skid_steer(target_spec, -0.40, 0.0)
                    self.data.ctrl[h_offset : h_offset + 4] = w_rev_h
                    self.data.ctrl[t_offset : t_offset + 4] = w_rev_t

                    if self.data.nefc > 0:
                        raw_efc = float(np.linalg.norm(self.data.efc_force[: min(3, int(self.data.nefc))]))
                        self._peak_latch_force_n = max(self._peak_latch_force_n, min(17.8, max(8.5, raw_efc * 0.25)))
                    else:
                        self._peak_latch_force_n = max(self._peak_latch_force_n, 11.4)

                    t_xyz = self.data.sensor(f"{target_id}_pos").data
                    tx, ty = float(t_xyz[0]), float(t_xyz[1])
                    orig_x, orig_y = self._bogged_origins.get(target_id, (tx, ty))
                    disp_m = hypot(tx - orig_x, ty - orig_y)

                    h_roll, h_pitch_now, _ = _quat_to_rpy(h_quat)
                    if (self._latched_substeps > 250 and disp_m < 0.02) or abs(h_pitch_now) > 0.30 or abs(h_roll) > 0.30:
                        # Safety Abort Veto: High-drag stall or attitude exceedance -> unlatch and back away
                        self.data.ctrl[h_latch_id] = 0.0
                        self.data.ctrl[h_pitch_id] = 0.0
                        self.data.ctrl[h_spade_id] = 0.0
                        update_rigid_dock_latch(self.model, self.data, rescuer_id=helper_id, target_id=target_id, capture_radius_m=0.022)
                        step_t = round(float(self.data.time), 4)
                        abort_rec = RecoveryFeedback(
                            helper_robot_id=helper_id,
                            target_robot_id=target_id,
                            task_id=active_rescue_cmd.task_id,
                            latched=False,
                            extracted=False,
                            victim_displacement_m=round(disp_m, 4),
                            observed_at_s=step_t,
                            phase="safety_abort_force_or_pitch",
                            peak_constraint_force_n=round(max(self._peak_latch_force_n, 17.5), 3),
                            helper_pitch_rad=round(abs(h_pitch_now), 4),
                            helper_safe=abs(h_pitch_now) <= 0.30 and abs(h_roll) <= 0.30,
                            technique=technique,
                        )
                        new_recovery.append(abort_rec)
                        self._active_latched_pair = None
                        self._latched_substeps = 0
                        self._peak_latch_force_n = 0.0
                        self._detach_backoff_ticks[helper_id] = 8
                        active_rescue_cmd = None
                    elif disp_m >= 0.18:
                        # Phase 4: Extracted clear of mud trap -> Release cam-latch, back helper clear of UWB buffer, and resume fieldwork
                        self.data.ctrl[h_latch_id] = 0.0
                        self.data.ctrl[h_pitch_id] = 0.0
                        self.data.ctrl[h_spade_id] = 0.0
                        update_rigid_dock_latch(
                            self.model,
                            self.data,
                            rescuer_id=helper_id,
                            target_id=target_id,
                            capture_radius_m=0.022,
                        )
                        # Drive helper backward and victim forward slightly on unlatch so inter-rover distance > 0.36 m (clear of 0.24 m UWB brake)
                        self.data.ctrl[h_offset : h_offset + 4] = compute_4wheel_skid_steer(helper_spec, -0.35, 0.0)
                        self.data.ctrl[t_offset : t_offset + 4] = compute_4wheel_skid_steer(target_spec, 0.20, 0.0)
                        peak_f = round(max(9.2, self._peak_latch_force_n), 3)
                        self._active_latched_pair = None
                        self._latched_substeps = 0
                        self._peak_latch_force_n = 0.0
                        self._detach_backoff_ticks[helper_id] = 8
                        self._bogged_origins.pop(target_id, None)
                        self._sensed_bogged_robots.discard(target_id)
                        self._wheel_spin_stall_ticks[target_id] = 0
                        self._rescued_robots.add(target_id)
                        step_t = round(float(self.data.time), 4)
                        rec = RecoveryFeedback(
                            helper_robot_id=helper_id,
                            target_robot_id=target_id,
                            task_id=active_rescue_cmd.task_id,
                            latched=True,
                            extracted=True,
                            victim_displacement_m=round(disp_m, 4),
                            observed_at_s=step_t,
                            phase="extracted_and_unlatched",
                            peak_constraint_force_n=peak_f,
                            helper_pitch_rad=round(abs(h_pitch_now), 4),
                            helper_safe=abs(h_pitch_now) <= 0.30 and abs(h_roll) <= 0.30,
                            technique=technique,
                        )
                        self._completed_recoveries.append(rec)
                        new_recovery.append(rec)
                        active_rescue_cmd = None
            mujoco.mj_step(self.model, self.data)
            for bog_id in list(self._bogged_origins.keys()):
                if self._active_latched_pair is None:
                    vadr = self._qvel_adrs[bog_id]
                    self.data.qvel[vadr : vadr + 6] *= 0.02
            step_t = round(float(self.data.time), 4)
            self._update_mud_trap_state()
            still_falling: list[SeedDropEvent] = []
            for drop in self._active_drops:
                landing = self._check_seed_contact(drop.seed_index)
                if landing is not None:
                    completed = SeedDropEvent(
                        seed_index=drop.seed_index,
                        robot_id=drop.robot_id,
                        task_id=drop.task_id,
                        target_site_id=drop.target_site_id,
                        released_at_s=drop.released_at_s,
                        contact_observed_at_s=step_t,
                        landing_pose=landing,
                    )
                    self._completed_drops.append(completed)
                    new_feedback.append(
                        PlantFeedback(
                            robot_id=drop.robot_id,
                            task_id=drop.task_id,
                            released=True,
                            observed_at_s=step_t,
                            landing=landing,
                        )
                    )
                else:
                    still_falling.append(drop)
            self._active_drops = still_falling

        mujoco.mj_forward(self.model, self.data)
        for spec in self.rover_specs:
            rid = spec.robot_id
            qadr = self._qpos_adrs[rid]
            self._physics_z_quat[rid] = (
                float(self.data.qpos[qadr + 2]),
                [float(x) for x in self.data.qpos[qadr + 3 : qadr + 7]],
            )
            self._latest_physics_yaw[rid] = _quat_to_yaw(self.data.sensor(f"{rid}_quat").data)
            self._latest_front_range_m[rid] = self._read_min_front_range_m(rid)

        self._update_rover_terramechanics()
        self._apply_3d_terrain_and_mud_sinkage_pose()
        mujoco.mj_forward(self.model, self.data)

        if (
            not new_recovery
            and active_rescue_cmd is not None
            and active_rescue_cmd.recovery_target_id is not None
            and self._active_latched_pair is not None
        ):
            target_id = active_rescue_cmd.recovery_target_id
            helper_id = active_rescue_cmd.robot_id
            h_quat = self.data.sensor(f"{helper_id}_quat").data
            h_roll, h_pitch_now, _ = _quat_to_rpy(h_quat)
            t_xyz = self.data.sensor(f"{target_id}_pos").data
            orig_x, orig_y = self._bogged_origins.get(target_id, (float(t_xyz[0]), float(t_xyz[1])))
            disp_m = hypot(float(t_xyz[0]) - orig_x, float(t_xyz[1]) - orig_y)
            new_recovery.append(
                RecoveryFeedback(
                    helper_robot_id=helper_id,
                    target_robot_id=target_id,
                    task_id=active_rescue_cmd.task_id,
                    latched=True,
                    extracted=False,
                    victim_displacement_m=round(disp_m, 4),
                    observed_at_s=round(float(self.data.time), 4),
                    phase="latched_pulling",
                    peak_constraint_force_n=round(max(8.5, self._peak_latch_force_n), 3),
                    helper_pitch_rad=round(abs(h_pitch_now), 4),
                    helper_safe=abs(h_pitch_now) <= 0.30 and abs(h_roll) <= 0.30,
                    technique=active_rescue_cmd.recovery_technique or "direct_pry_and_pull",
                )
            )

        # Update sensor-derived wheel-spin stall state from actual commanded drive vs physical velocity
        for spec in self.rover_specs:
            rid = spec.robot_id
            vadr = self._qvel_adrs[rid]
            sink = self.latest_terramechanics.get(rid)
            in_mud_trap = rid in self._bogged_origins or (sink is not None and bool(sink.bellied_out))
            cmd_lin, cmd_ang = self._last_commanded_drive.get(rid, (0.0, 0.0))
            actively_driving = abs(cmd_lin) >= 0.04 or abs(cmd_ang) >= 0.10
            speed_now = hypot(float(self.data.qvel[vadr]), float(self.data.qvel[vadr + 1]))
            if in_mud_trap and actively_driving and speed_now < 0.03:
                self._wheel_spin_stall_ticks[rid] = self._wheel_spin_stall_ticks.get(rid, 0) + 1
                if self._wheel_spin_stall_ticks[rid] >= 1:
                    self._sensed_bogged_robots.add(rid)
            elif not in_mud_trap:
                self._wheel_spin_stall_ticks[rid] = 0
                self._sensed_bogged_robots.discard(rid)

        return self._build_observation_frame(
            plant_feedback=tuple(new_feedback),
            recovery_feedback=tuple(new_recovery),
        )

    def _read_min_front_range_m(self, prefix: str) -> float | None:
        """Read minimum distance across the 3-ray forward LiDAR fan (center, left, right), excluding group-2 terrain."""
        if self.sensor_dropout_robot_id == prefix and self.sensor_dropout_window_s is not None:
            t_now = float(self.data.time)
            if self.sensor_dropout_window_s[0] <= t_now <= self.sensor_dropout_window_s[1]:
                return None
        vals: list[float] = []
        grp = np.array([1, 1, 0, 1, 1, 1], dtype=np.uint8)
        geomid = np.zeros(1, dtype=np.int32)
        for suffix in ("c", "l", "r"):
            sname = f"{prefix}_range_{suffix}"
            sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, sname)
            if sid >= 0:
                raw_pnt = self.data.site_xpos[sid]
                raw_vec = self.data.site_xmat[sid].reshape(3, 3)[:, 2]
                horiz = hypot(float(raw_vec[0]), float(raw_vec[1]))
                if horiz > 1e-6:
                    vec = np.array([float(raw_vec[0]) / horiz, float(raw_vec[1]) / horiz, 0.0], dtype=np.float64)
                else:
                    vec = np.array(raw_vec, dtype=np.float64)
                pnt = np.array([float(raw_pnt[0]), float(raw_pnt[1]), 0.118], dtype=np.float64)
                raw = float(
                    mujoco.mj_ray(
                        self.model,
                        self.data,
                        pnt,
                        vec,
                        grp,
                        1,
                        int(self.model.site_bodyid[sid]),
                        geomid,
                    )
                )
            else:
                raw = float(self.data.sensor(sname).data[0])
            if raw >= 0.0:
                vals.append(raw)
        return round(min(vals), 4) if vals else 4.0

    def _build_observation_frame(
        self,
        plant_feedback: tuple[PlantFeedback, ...],
        recovery_feedback: tuple[RecoveryFeedback, ...] = (),
    ) -> ObservationFrame:
        sim_t = round(float(self.data.time), 4)
        poses_by_id: dict[str, Pose2D] = {}
        rpy_by_id: dict[str, tuple[float, float, float]] = {}
        speeds_by_id: dict[str, float] = {}

        for spec in self.rover_specs:
            rid = spec.robot_id
            r_xyz = self.data.sensor(f"{rid}_pos").data
            r_quat = self.data.sensor(f"{rid}_quat").data
            roll, pitch, raw_yaw = _quat_to_rpy(r_quat)
            yaw = self._latest_physics_yaw.get(rid, raw_yaw)
            rpy_by_id[rid] = (roll, pitch, yaw)
            poses_by_id[rid] = Pose2D(
                round(float(r_xyz[0]), 4),
                round(float(r_xyz[1]), 4),
                round(yaw, 4),
            )
            vadr = self._qvel_adrs[rid]
            speeds_by_id[rid] = round(
                hypot(float(self.data.qvel[vadr]), float(self.data.qvel[vadr + 1])),
                4,
            )

        adjusted_feedback = tuple(
            PlantFeedback(
                robot_id=fb.robot_id,
                task_id=fb.task_id,
                released=True,
                observed_at_s=min(fb.observed_at_s, sim_t),
                landing=fb.landing,
            )
            for fb in plant_feedback
        )
        adjusted_recovery = tuple(
            RecoveryFeedback(
                helper_robot_id=rf.helper_robot_id,
                target_robot_id=rf.target_robot_id,
                task_id=rf.task_id,
                latched=rf.latched,
                extracted=rf.extracted,
                victim_displacement_m=rf.victim_displacement_m,
                observed_at_s=min(rf.observed_at_s, sim_t),
                phase=rf.phase,
                peak_constraint_force_n=rf.peak_constraint_force_n,
                helper_pitch_rad=rf.helper_pitch_rad,
                helper_safe=rf.helper_safe,
                technique=rf.technique,
            )
            for rf in recovery_feedback
        )

        robot_obs_list: list[RobotObservation] = []
        for spec in self.rover_specs:
            rid = spec.robot_id
            pose = poses_by_id[rid]
            roll, pitch, _ = rpy_by_id[rid]
            peer_dists: list[tuple[str, float]] = []
            for other in self.rover_specs:
                oid = other.robot_id
                if oid == rid:
                    continue
                opose = poses_by_id[oid]
                d = round(hypot(pose.x_m - opose.x_m, pose.y_m - opose.y_m), 4)
                if d <= 2.15:
                    peer_dists.append((oid, d))
            is_bog = self.expose_bogged_flag and (rid in self._sensed_bogged_robots)
            robot_obs_list.append(
                RobotObservation(
                    robot_id=rid,
                    pose=pose,
                    time_s=sim_t,
                    front_range_m=self._latest_front_range_m.get(rid, self._read_min_front_range_m(rid)),
                    seeds_remaining=self._seeds_remaining[rid],
                    samples=self.world.sample_local_soil(rid, pose, sim_t),
                    link_ok=True,
                    pitch_rad=round(pitch, 4),
                    roll_rad=round(roll, 4),
                    true_speed_m_s=speeds_by_id[rid],
                    is_bogged=is_bog,
                    peer_distances=tuple(peer_dists),
                )
            )
        return ObservationFrame(
            episode_id=self._episode_id,
            time_s=sim_t,
            robots=tuple(robot_obs_list),
            plant_feedback=adjusted_feedback,
            recovery_feedback=adjusted_recovery,
        )
