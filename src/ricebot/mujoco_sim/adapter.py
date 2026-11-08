"""MuJoCo 3D level paddy-field & homogeneous Paddy Ant rover physics adapter (`src/ricebot/mujoco_sim/`).

Architectural Composition per `docs/COMPONENTS.md` & `docs/FIVE-AGENT-RECONCILIATION.md`:
- Composes:
  1. `src/ricebot/world/` (`PaddyTerraceWorld`, `ContinuousPaddyTerrain`, `DEFAULT_SITES`,
     `build_world_terrain_xml`, `build_world_texture_assets_xml`, `FaultEvidence`,
     `derive_fault_evidence`) — owns the 6-patch level rice-field layout, continuous 2D
     soil/elevation functions, hidden soil/stand truth, location-gated probes with sensor
     validity/uncertainty, sensor-derived fault evidence, and grasslike rice-field MJCF geometry.
  2. `src/ricebot/robot/` (`PLANTER_SPEC`, `SCOUT_SPEC`, `DEFAULT_CAPABILITIES`,
     `compute_4wheel_skid_steer`, `evaluate_bekker_terramechanics`, `build_rovers_and_seeds_xml`,
     `update_rigid_dock_latch`) — owns the homogeneous "Paddy Ant" 4-wheel grouser-lug chassis,
     analytical Bekker-Wong wheel-mud sinkage & traction physics, active farming end-effectors,
     and the 4-part rigid docking, cam-latching & anchor-spade rescue mechanism.
- Implements `SimulationPort` (`reset(seed) -> ObservationFrame`, `step(commands) -> ObservationFrame`).
"""
from __future__ import annotations

from dataclasses import dataclass
from math import asin, atan2, cos, hypot, pi, sin

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
    TerramechanicsSinkageState,
    build_rovers_and_seeds_xml,
    compute_4wheel_skid_steer,
    evaluate_bekker_terramechanics,
    update_rigid_dock_latch,
)
from ..world import (
    ContinuousTerrainPoint,
    DEFAULT_SITES,
    FaultEvidence,
    PaddyTerraceWorld,
    build_world_terrain_xml,
    build_world_texture_assets_xml,
    derive_fault_evidence,
)

# Solid obstacles/bunds/rovers are in group 0; decorative grasslike rice leaves & visual water sheets
# are in group 2 (skipped by LiDAR raycast so decorative plants never create invisible walls).
_LIDAR_SOLID_GEOMGROUP = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)


def compose_paddypatch_mjcf(
    obstacle_xy: tuple[float, float] = (0.75, -0.25),
    hidden_truth: dict[str, dict[SensorKind, float]] | None = None,
) -> str:
    """Compose the full MJCF XML from `src/ricebot/world/` terrain and `src/ricebot/robot/` Paddy Ant units."""
    texture_assets_xml = build_world_texture_assets_xml()
    world_terrain_xml = build_world_terrain_xml(
        obstacle_xy=obstacle_xy, hidden_truth=hidden_truth
    )
    rovers_bodies_xml, rovers_actuators_xml, rovers_sensors_xml = build_rovers_and_seeds_xml()
    return f"""
<mujoco model="paddypatch_rice_v1_terrace">
  <option timestep="0.004" gravity="0 0 -9.81" integrator="implicitfast"/>

  <visual>
    <global offwidth="1280" offheight="960" azimuth="115" elevation="-32"/>
    <headlight ambient="0.40 0.40 0.38" diffuse="0.66 0.65 0.60" specular="0.22 0.22 0.20"/>
    <map znear="0.02" zfar="25.0"/>
  </visual>

  <statistic center="1.35 -0.05 0.15" extent="2.2"/>

  <asset>
    {texture_assets_xml}
  </asset>

  <worldbody>
    {world_terrain_xml}
    <geom name="paddy_relief_mesh" type="hfield" hfield="paddy_micro_relief" pos="1.35 0.0 -0.016" rgba="0.38 0.31 0.21 0.38" contype="0" conaffinity="0" group="2"/>
    {rovers_bodies_xml}
  </worldbody>

  <equality>
    <connect name="rigid_latch_planter_to_scout" site1="planter_front_dock_site" site2="scout_rear_dock_site" active="false" solref="0.04 1.0"/>
    <connect name="rigid_latch_scout_to_planter" site1="scout_front_dock_site" site2="planter_rear_dock_site" active="false" solref="0.04 1.0"/>
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
    w, x, y, z = (
        float(quat_wxyz[0]),
        float(quat_wxyz[1]),
        float(quat_wxyz[2]),
        float(quat_wxyz[3]),
    )
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
    w, x, y, z = (
        float(quat_wxyz[0]),
        float(quat_wxyz[1]),
        float(quat_wxyz[2]),
        float(quat_wxyz[3]),
    )
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
    """Genuine 3D homogeneous Paddy-Ant & level rice-field MuJoCo implementation of `SimulationPort` (`rice.v1`)."""

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
        probe_noise_std: float = 0.0,
        probe_bias: float = 0.0,
        sensor_degraded_window_s: tuple[float, float] | None = None,
        sensor_degraded_robot_id: str = "planter",
    ) -> None:
        self.world = PaddyTerraceWorld(
            sites=sites,
            probe_radius_m=probe_radius_m,
            hidden_truth=hidden_truth,
            probe_noise_std=probe_noise_std,
            probe_bias=probe_bias,
            sensor_degraded_window_s=sensor_degraded_window_s,
        )
        self.sites = self.world.sites
        self.probe_radius_m = probe_radius_m
        self.substeps_per_tick = substeps_per_tick
        self.inject_obstacle_at_s = inject_obstacle_at_s
        self.obstacle_xy = obstacle_xy
        self.trap_robot_id = trap_robot_id
        self.trap_site_id = trap_site_id
        self.trap_radius_m = trap_radius_m
        self.sensor_degraded_window_s = sensor_degraded_window_s
        self.sensor_degraded_robot_id = sensor_degraded_robot_id

        mjcf_xml = compose_paddypatch_mjcf(
            obstacle_xy=obstacle_xy, hidden_truth=self.world._hidden_site_truth
        )
        self.model = mujoco.MjModel.from_xml_string(mjcf_xml)
        self.data = mujoco.MjData(self.model)

        self._ground_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "field_ground"
        )
        self._seed_geom_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"seed_{i}_geom")
            for i in range(PLANTER_SPEC.initial_seeds)
        ]
        self._planter_qpos_adr = int(
            self.model.jnt_qposadr[
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "planter_free")
            ]
        )
        self._scout_qpos_adr = int(
            self.model.jnt_qposadr[
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "scout_free")
            ]
        )
        self._planter_qvel_adr = int(
            self.model.jnt_dofadr[
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "planter_free")
            ]
        )
        self._scout_qvel_adr = int(
            self.model.jnt_dofadr[
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "scout_free")
            ]
        )
        self._body_ids: dict[str, int] = {
            "planter": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "planter_body"),
            "scout": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "scout_body"),
        }
        self._range_site_ids: dict[tuple[str, str], int] = {
            (rid, sfx): mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_SITE, f"{rid}_range_{sfx}"
            )
            for rid in ("planter", "scout")
            for sfx in ("c", "l", "r")
        }
        self._wheel_geom_ids: dict[str, list[int]] = {
            rid: [
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"{rid}_w_{w}")
                for w in ("fl", "fr", "rl", "rr")
            ]
            for rid in ("planter", "scout")
        }
        for rid, gids in self._wheel_geom_ids.items():
            for gid in gids:
                if gid >= 0:
                    self.model.geom_priority[gid] = 1
        self._seed_qpos_adrs = [
            int(
                self.model.jnt_qposadr[
                    mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"seed_{i}_free")
                ]
            )
            for i in range(PLANTER_SPEC.initial_seeds)
        ]
        self._seed_qvel_adrs = [
            int(
                self.model.jnt_dofadr[
                    mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"seed_{i}_free")
                ]
            )
            for i in range(PLANTER_SPEC.initial_seeds)
        ]

        self._dibber_act_ids: dict[str, int] = {}
        self._weeder_l_act_ids: dict[str, int] = {}
        self._weeder_r_act_ids: dict[str, int] = {}
        self._dock_pitch_act_ids: dict[str, int] = {}
        self._latch_act_ids: dict[str, int] = {}
        self._spade_act_ids: dict[str, int] = {}
        for rid in ("planter", "scout"):
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

        self.robot_mass_kg: dict[str, float] = {
            PLANTER_SPEC.robot_id: 4.2,
            SCOUT_SPEC.robot_id: 3.6,
        }
        self.robot_wheel_radius_m: dict[str, float] = {
            PLANTER_SPEC.robot_id: PLANTER_SPEC.wheel_radius_m,
            SCOUT_SPEC.robot_id: SCOUT_SPEC.wheel_radius_m,
        }
        self.robot_wheel_width_m: dict[str, float] = {
            PLANTER_SPEC.robot_id: 0.040,
            SCOUT_SPEC.robot_id: 0.040,
        }
        self.latest_terrain_points: dict[str, ContinuousTerrainPoint] = {}
        self.latest_terramechanics: dict[str, TerramechanicsSinkageState] = {}
        self.latest_slip_ratios: dict[str, float] = {"planter": 0.08, "scout": 0.08}
        self.latest_fault_evidence: dict[str, FaultEvidence] = {}
        self.mud_rut_history: list[tuple[str, float, float, float, float]] = []

        self._seed: int = 7
        self._episode_id: str = "mujoco-episode-7"
        self._seeds_remaining: dict[str, int] = {
            PLANTER_SPEC.robot_id: PLANTER_SPEC.initial_seeds,
            SCOUT_SPEC.robot_id: SCOUT_SPEC.initial_seeds,
        }
        self._active_drops: list[SeedDropEvent] = []
        self._completed_drops: list[SeedDropEvent] = []
        self._completed_recoveries: list[RecoveryFeedback] = []
        self._obstacle_injected: bool = False
        self._populate_hfield_from_continuous_terrain()
        self._bogged_origins: dict[str, tuple[float, float]] = {}
        self._confirmed_bogged_robots: set[str] = set()
        self._rescued_robots: set[str] = set()
        self._active_latched_pair: tuple[str, str] | None = None
        self._detach_backoff_ticks: dict[str, int] = {}
        self._latched_substeps: int = 0
        self._last_issued_commands: dict[str, RobotCommand] = {}
        self._prev_obs_poses: dict[str, Pose2D] = {}
        self._stalled_tick_counts: dict[str, int] = {"planter": 0, "scout": 0}
        self._rangefinder_dropout_robots: set[str] = set()

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

    def set_rangefinder_dropout(self, robot_id: str, dropped: bool = True) -> None:
        """Explicitly enable/disable rangefinder sensor dropout (`front_range_m = None`) for `robot_id`."""
        if dropped:
            self._rangefinder_dropout_robots.add(robot_id)
        else:
            self._rangefinder_dropout_robots.discard(robot_id)

    def _update_rover_terramechanics(self) -> None:
        """Evaluate ContinuousPaddyTerrain and Bekker-Wong wheel-mud sinkage at each rover's live 3D pose."""
        for spec in (PLANTER_SPEC, SCOUT_SPEC):
            rid = spec.robot_id
            r_xyz = self.data.sensor(f"{rid}_pos").data
            rx, ry = float(r_xyz[0]), float(r_xyz[1])
            pt = self.world.evaluate_continuous_point(rx, ry)
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
            if sink_state.bellied_out or rid in self._bogged_origins:
                local_slip = 0.96
            self.latest_terrain_points[rid] = pt
            self.latest_terramechanics[rid] = sink_state
            self.latest_slip_ratios[rid] = round(local_slip, 4)

    def _apply_3d_terrain_and_mud_sinkage_pose(self) -> None:
        """No-op hook retained for API compatibility: rover Z-height is purely governed by MuJoCo contact physics (Audit M3)."""
        return None

    def reset(self, seed: int) -> ObservationFrame:
        self._seed = int(seed)
        self._episode_id = f"mujoco-ep-{self._seed}"
        self.world.reset(self._seed)
        self._populate_hfield_from_continuous_terrain()
        self._seeds_remaining = {
            PLANTER_SPEC.robot_id: PLANTER_SPEC.initial_seeds,
            SCOUT_SPEC.robot_id: SCOUT_SPEC.initial_seeds,
        }
        self._active_drops = []
        self._completed_drops = []
        self._completed_recoveries = []
        self._obstacle_injected = False
        self.mud_rut_history = []
        self._bogged_origins = {}
        self._confirmed_bogged_robots = set()
        self._rescued_robots = set()
        self._active_latched_pair = None
        self._detach_backoff_ticks = {}
        self._latched_substeps = 0
        self._last_issued_commands = {}
        self._prev_obs_poses = {}
        self._stalled_tick_counts = {"planter": 0, "scout": 0}
        self._rangefinder_dropout_robots = set()
        self.latest_fault_evidence = {}
        for gids in self._wheel_geom_ids.values():
            for gid in gids:
                if gid >= 0:
                    self.model.geom_friction[gid, 0] = 1.4

        mujoco.mj_resetData(self.model, self.data)
        px, py, pz = PLANTER_SPEC.initial_xyz
        sx, sy, sz = SCOUT_SPEC.initial_xyz
        self.data.qpos[self._planter_qpos_adr : self._planter_qpos_adr + 7] = [
            px,
            py,
            pz,
            1.0,
            0.0,
            0.0,
            0.0,
        ]
        self.data.qpos[self._scout_qpos_adr : self._scout_qpos_adr + 7] = [
            sx,
            sy,
            sz,
            1.0,
            0.0,
            0.0,
            0.0,
        ]
        self.data.mocap_pos[0] = [self.obstacle_xy[0], self.obstacle_xy[1], -5.0]
        self._sync_carried_seeds()
        mujoco.mj_forward(self.model, self.data)
        self._refresh_solid_rangefinder_sensors()
        self._update_rover_terramechanics()
        return self._build_observation_frame(plant_feedback=(), recovery_feedback=())

    def _sync_carried_seeds(self) -> None:
        """Keep unreleased seeds inside the planter's hopper frame."""
        released_indices = {d.seed_index for d in self._active_drops} | {
            d.seed_index for d in self._completed_drops
        }
        planter_xyz = self.data.sensor("planter_pos").data
        px, py, pz = float(planter_xyz[0]), float(planter_xyz[1]), float(planter_xyz[2])
        for idx in range(PLANTER_SPEC.initial_seeds):
            if idx not in released_indices:
                qadr = self._seed_qpos_adrs[idx]
                vadr = self._seed_qvel_adrs[idx]
                self.data.qpos[qadr : qadr + 3] = [px, py, pz + 0.12 + 0.05 * idx]
                self.data.qpos[qadr + 3 : qadr + 7] = [1.0, 0.0, 0.0, 0.0]
                self.data.qvel[vadr : vadr + 6] = 0.0

    def _check_seed_contact(self, seed_index: int) -> Pose2D | None:
        """Return landing Pose2D if seed_index geom is physically touching field_ground."""
        target_geom = self._seed_geom_ids[seed_index]
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            if {c.geom1, c.geom2} == {target_geom, self._ground_geom_id}:
                qadr = self._seed_qpos_adrs[seed_index]
                sx, sy = float(self.data.qpos[qadr]), float(self.data.qpos[qadr + 1])
                return Pose2D(round(sx, 4), round(sy, 4), 0.0)
        return None

    def _update_mud_trap_state(self) -> None:
        """Check if configured trap_robot_id has entered the deep mud basin at trap_site_id."""
        if (
            self.trap_robot_id is None
            or self.trap_robot_id in self._bogged_origins
            or self.trap_robot_id in self._rescued_robots
            or self.trap_site_id not in self.sites
        ):
            return
        site = self.sites[self.trap_site_id]
        r_xyz = self.data.sensor(f"{self.trap_robot_id}_pos").data
        rx, ry = float(r_xyz[0]), float(r_xyz[1])
        if hypot(rx - site.center.x_m, ry - site.center.y_m) <= self.trap_radius_m:
            self._bogged_origins[self.trap_robot_id] = (rx, ry)

    def step(self, commands: tuple[RobotCommand, ...]) -> ObservationFrame:
        now_s = round(float(self.data.time), 4)
        cmd_by_robot = {c.robot_id: c for c in commands}

        # Record start-of-tick poses and valid commands for sensor-derived FaultEvidence (Audit C3)
        for rid in ("planter", "scout"):
            r_xyz = self.data.sensor(f"{rid}_pos").data
            r_quat = self.data.sensor(f"{rid}_quat").data
            self._prev_obs_poses[rid] = Pose2D(
                round(float(r_xyz[0]), 4),
                round(float(r_xyz[1]), 4),
                round(_quat_to_yaw(r_quat), 4),
            )
            cmd = cmd_by_robot.get(rid)
            if cmd is not None and (cmd.issued_at_s <= now_s < cmd.expires_at_s):
                self._last_issued_commands[rid] = cmd
            else:
                self._last_issued_commands.pop(rid, None)

        if (
            self.inject_obstacle_at_s is not None
            and not self._obstacle_injected
            and now_s >= self.inject_obstacle_at_s
        ):
            ox, oy = float(self.obstacle_xy[0]), float(self.obstacle_xy[1])
            # Ensure surprise mocap obstacle spawns ahead of the rover nose rather than overlapping its chassis
            for rid in ("planter", "scout"):
                r_xyz = self.data.sensor(f"{rid}_pos").data
                rx, ry = float(r_xyz[0]), float(r_xyz[1])
                if hypot(rx - ox, ry - oy) < 0.28:
                    ox = round(max(ox, rx + 0.32), 3)
            self.data.mocap_pos[0] = [ox, oy, 0.12]
            self._obstacle_injected = True
            mujoco.mj_forward(self.model, self.data)

        self._update_rover_terramechanics()
        self._update_mud_trap_state()
        self.data.ctrl[:] = 0.0
        self.data.xfrc_applied[:] = 0.0

        active_rescue_cmd: RobotCommand | None = None
        for spec, ctrl_offset in ((PLANTER_SPEC, 0), (SCOUT_SPEC, 4)):
            robot_id = spec.robot_id
            cmd = cmd_by_robot.get(robot_id)
            if cmd is None or not (cmd.issued_at_s <= now_s < cmd.expires_at_s):
                continue

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

            if (
                cmd.recovery_target_id in ("planter", "scout")
                and cmd.recovery_target_id != robot_id
                and cmd.recovery_target_id not in self._rescued_robots
            ):
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
                        self.data.ctrl[dib_id] = -0.048
                    seed_idx = spec.initial_seeds - self._seeds_remaining[robot_id]
                    self._seeds_remaining[robot_id] -= 1
                    planter_xyz = self.data.sensor("planter_pos").data
                    px, py = float(planter_xyz[0]), float(planter_xyz[1])
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
                sink_state = self.latest_terramechanics.get(robot_id)
                slip_ratio = self.latest_slip_ratios.get(robot_id, 0.08)
                is_custom_sizing = (
                    abs(
                        self.robot_mass_kg.get(robot_id, 4.2 if robot_id == "planter" else 3.6)
                        - (4.2 if robot_id == "planter" else 3.6)
                    )
                    > 0.3
                    or abs(self.robot_wheel_radius_m.get(robot_id, 0.058) - 0.058) > 0.003
                )
                is_bogged_now = (
                    robot_id in self._bogged_origins and self._active_latched_pair is None
                )
                for gid in self._wheel_geom_ids.get(robot_id, ()):
                    if gid >= 0:
                        self.model.geom_friction[gid, 0] = 0.002 if is_bogged_now else 1.4
                if is_bogged_now:
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
                    0.15 * slip_ratio * sin(9.5 * now_s + (0.0 if robot_id == "planter" else 2.1))
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

                if abs(effective_linear) > 0.05:
                    if wl_id >= 0:
                        self.data.ctrl[wl_id] = 12.0
                    if wr_id >= 0:
                        self.data.ctrl[wr_id] = -12.0

                r_xyz = self.data.sensor(f"{robot_id}_pos").data
                rx, ry = float(r_xyz[0]), float(r_xyz[1])
                if dib_id >= 0:
                    near_site = any(
                        s.permitted and hypot(rx - s.center.x_m, ry - s.center.y_m) <= self.probe_radius_m
                        for s in self.sites.values()
                    )
                    self.data.ctrl[dib_id] = -0.045 if near_site else 0.0

                pt = self.latest_terrain_points.get(robot_id)
                if pt is not None and sink_state is not None and abs(cmd.linear_m_s) > 0.02:
                    if not self.mud_rut_history or hypot(
                        rx - self.mud_rut_history[-1][1], ry - self.mud_rut_history[-1][2]
                    ) >= 0.09:
                        self.mud_rut_history.append(
                            (
                                robot_id,
                                round(rx, 3),
                                round(ry, 3),
                                pt.elevation_m,
                                sink_state.sinkage_z_m,
                            )
                        )
                        if len(self.mud_rut_history) > 90:
                            self.mud_rut_history = self.mud_rut_history[-90:]

        new_feedback: list[PlantFeedback] = []
        new_recovery: list[RecoveryFeedback] = []

        for substep_i in range(self.substeps_per_tick):
            self._sync_carried_seeds()

            for bog_id in list(self._bogged_origins.keys()):
                if self._active_latched_pair is None:
                    vadr = self._planter_qvel_adr if bog_id == "planter" else self._scout_qvel_adr
                    bid = self._body_ids[bog_id]
                    self.data.xfrc_applied[bid, 0] = -35.0 * float(self.data.qvel[vadr])
                    self.data.xfrc_applied[bid, 1] = -35.0 * float(self.data.qvel[vadr + 1])
                    self.data.qvel[vadr : vadr + 2] *= 0.12

            if active_rescue_cmd is not None and active_rescue_cmd.recovery_target_id is not None:
                helper_id = active_rescue_cmd.robot_id
                target_id = active_rescue_cmd.recovery_target_id
                helper_spec = PLANTER_SPEC if helper_id == "planter" else SCOUT_SPEC
                target_spec = PLANTER_SPEC if target_id == "planter" else SCOUT_SPEC
                h_offset = 0 if helper_id == "planter" else 4
                t_offset = 0 if target_id == "planter" else 4
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
                    heading_err = _wrap_angle(atan2(dy, dx) - h_yaw)
                    ang_cmd = max(-1.35, min(1.35, heading_err * 2.6))
                    lin_cmd = (
                        min(0.35, max(0.10, dock_dist * 0.90))
                        if abs(heading_err) < 0.45
                        else 0.02
                    )
                    self.data.ctrl[h_pitch_id] = 0.0
                    self.data.ctrl[h_spade_id] = 0.0
                    w_h = compute_4wheel_skid_steer(helper_spec, lin_cmd, ang_cmd)
                    self.data.ctrl[h_offset : h_offset + 4] = w_h
                else:
                    self._active_latched_pair = (helper_id, target_id)
                    self._latched_substeps += 1
                    for gid in self._wheel_geom_ids.get(target_id, ()):
                        if gid >= 0:
                            self.model.geom_friction[gid, 0] = 1.4
                    self.data.ctrl[h_pitch_id] = min(
                        0.15, max(0.08, active_rescue_cmd.dock_pitch_rad)
                    )
                    self.data.ctrl[h_spade_id] = 0.0
                    w_rev_h = compute_4wheel_skid_steer(helper_spec, -0.40, 0.0)
                    w_rev_t = compute_4wheel_skid_steer(target_spec, -0.40, 0.0)
                    self.data.ctrl[h_offset : h_offset + 4] = w_rev_h
                    self.data.ctrl[t_offset : t_offset + 4] = w_rev_t

                    t_xyz = self.data.sensor(f"{target_id}_pos").data
                    tx, ty = float(t_xyz[0]), float(t_xyz[1])
                    orig_x, orig_y = self._bogged_origins.get(target_id, (tx, ty))
                    disp_m = hypot(tx - orig_x, ty - orig_y)

                    h_roll, h_pitch_now, _ = _quat_to_rpy(h_quat)
                    if (
                        (self._latched_substeps > 250 and disp_m < 0.02)
                        or abs(h_pitch_now) > 0.30
                        or abs(h_roll) > 0.30
                    ):
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
                        self._active_latched_pair = None
                        self._latched_substeps = 0
                        self._detach_backoff_ticks[helper_id] = 8
                        active_rescue_cmd = None
                    elif disp_m >= 0.18:
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
                        self.data.ctrl[h_offset : h_offset + 4] = compute_4wheel_skid_steer(
                            helper_spec, -0.35, 0.0
                        )
                        self.data.ctrl[t_offset : t_offset + 4] = compute_4wheel_skid_steer(
                            target_spec, 0.20, 0.0
                        )
                        self._active_latched_pair = None
                        self._latched_substeps = 0
                        self._detach_backoff_ticks[helper_id] = 8
                        self._bogged_origins.pop(target_id, None)
                        self._confirmed_bogged_robots.discard(target_id)
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
                        )
                        self._completed_recoveries.append(rec)
                        new_recovery.append(rec)
                        active_rescue_cmd = None
            mujoco.mj_step(self.model, self.data)
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

        self._update_rover_terramechanics()
        mujoco.mj_forward(self.model, self.data)
        self._refresh_solid_rangefinder_sensors()

        if (
            not new_recovery
            and active_rescue_cmd is not None
            and active_rescue_cmd.recovery_target_id is not None
            and self._active_latched_pair is not None
        ):
            target_id = active_rescue_cmd.recovery_target_id
            t_xyz = self.data.sensor(f"{target_id}_pos").data
            orig_x, orig_y = self._bogged_origins.get(
                target_id, (float(t_xyz[0]), float(t_xyz[1]))
            )
            disp_m = hypot(float(t_xyz[0]) - orig_x, float(t_xyz[1]) - orig_y)
            new_recovery.append(
                RecoveryFeedback(
                    helper_robot_id=active_rescue_cmd.robot_id,
                    target_robot_id=target_id,
                    task_id=active_rescue_cmd.task_id,
                    latched=True,
                    extracted=False,
                    victim_displacement_m=round(disp_m, 4),
                    observed_at_s=round(float(self.data.time), 4),
                    phase="latched_pulling",
                )
            )

        return self._build_observation_frame(
            plant_feedback=tuple(new_feedback),
            recovery_feedback=tuple(new_recovery),
        )

    def _refresh_solid_rangefinder_sensors(self) -> None:
        """Compute forward 3-ray LiDAR distances against solid group-0 geometry only (skipping group-2 decorative rice leaves and self child bodies)."""
        geomid_out = np.zeros(1, dtype=np.int32)
        for prefix in ("planter", "scout"):
            root_body = self._body_ids.get(prefix, -1)
            for suffix in ("c", "l", "r"):
                site_id = self._range_site_ids.get((prefix, suffix), -1)
                if site_id < 0:
                    continue
                vec = self.data.site_xmat[site_id].reshape(3, 3)[:, 2]
                pnt0 = self.data.site_xpos[site_id]
                advance_m = 0.0
                hit_dist = -1.0
                for _ in range(6):
                    pnt = pnt0 + advance_m * vec
                    d = float(
                        mujoco.mj_ray(
                            self.model,
                            self.data,
                            pnt,
                            vec,
                            _LIDAR_SOLID_GEOMGROUP,
                            1,
                            root_body,
                            geomid_out,
                        )
                    )
                    gid = int(geomid_out[0])
                    if d < 0.0 or gid < 0:
                        hit_dist = -1.0
                        break
                    hit_body = int(self.model.geom_bodyid[gid])
                    if int(self.model.body_rootid[hit_body]) == root_body:
                        advance_m += d + 0.015
                        continue
                    hit_dist = advance_m + d
                    break
                if hit_dist >= 0.0:
                    self.data.sensor(f"{prefix}_range_{suffix}").data[0] = min(hit_dist, 4.0)
                else:
                    self.data.sensor(f"{prefix}_range_{suffix}").data[0] = 4.0

    def _read_min_front_range_m(self, prefix: str) -> float | None:
        """Read minimum distance across the 3-ray forward LiDAR fan (`c`, `l`, `r`).

        - Returns `None` when the sensor is in dropout (`sensor_degraded_window_s` or
          `set_rangefinder_dropout`) or when all 3 rays report `< 0.0` (Audit M5 & N3).
        """
        sim_t = round(float(self.data.time), 4)
        if prefix in self._rangefinder_dropout_robots:
            return None
        if self.world.is_sensor_degraded_at(sim_t) and prefix == self.sensor_degraded_robot_id:
            return None

        vals: list[float] = []
        for suffix in ("c", "l", "r"):
            raw = float(self.data.sensor(f"{prefix}_range_{suffix}").data[0])
            if raw >= 0.0:
                vals.append(min(raw, 4.0))
        if not vals:
            return None
        return round(min(vals), 4)

    def _compute_robot_fault_and_bogged(
        self,
        robot_id: str,
        pose: Pose2D,
        sim_t: float,
        front_range_m: float | None,
        link_ok: bool,
        pitch_rad: float,
        roll_rad: float,
        speed_m_s: float,
    ) -> tuple[FaultEvidence, bool]:
        """Derive per-robot `FaultEvidence` and non-oracle `is_bogged` from command-vs-motion stall (Audit C3)."""
        prev_pose = self._prev_obs_poses.get(robot_id)
        last_cmd = self._last_issued_commands.get(robot_id)
        cmd_lin = last_cmd.linear_m_s if last_cmd is not None else 0.0
        cmd_ang = last_cmd.angular_rad_s if last_cmd is not None else 0.0
        disp_m = (
            hypot(pose.x_m - prev_pose.x_m, pose.y_m - prev_pose.y_m)
            if prev_pose is not None
            else 0.0
        )
        yaw_delta = (
            abs(_wrap_angle(pose.yaw_rad - prev_pose.yaw_rad))
            if prev_pose is not None
            else 0.0
        )
        slip_ratio = self.latest_slip_ratios.get(robot_id, 0.08)

        evidence = derive_fault_evidence(
            robot_id=robot_id,
            observed_at_s=sim_t,
            commanded_linear_m_s=cmd_lin,
            commanded_angular_rad_s=cmd_ang,
            observed_displacement_m=disp_m,
            observed_yaw_delta_rad=yaw_delta,
            observed_speed_m_s=speed_m_s,
            slip_ratio=slip_ratio,
            pitch_rad=pitch_rad,
            roll_rad=roll_rad,
            front_range_m=front_range_m,
            link_ok=link_ok,
            previous_stalled_ticks=self._stalled_tick_counts.get(robot_id, 0),
        )
        self._stalled_tick_counts[robot_id] = evidence.stalled_ticks
        self.latest_fault_evidence[robot_id] = evidence

        sink = self.latest_terramechanics.get(robot_id)
        physically_trapped = robot_id in self._bogged_origins or (
            sink is not None and bool(sink.bellied_out)
        )
        if physically_trapped and evidence.is_commanded_motion_stall:
            self._confirmed_bogged_robots.add(robot_id)

        # Negative controls:
        # 1. Missing rangefinder (`front_range_m is None`) can NEVER report `is_bogged=True`.
        # 2. A robot that has only ever been on HOLD (never stalled under an active drive command)
        #    cannot report `is_bogged=True`.
        is_bogged = bool(
            front_range_m is not None
            and physically_trapped
            and robot_id in self._confirmed_bogged_robots
        )
        return evidence, is_bogged

    def _build_observation_frame(
        self,
        plant_feedback: tuple[PlantFeedback, ...],
        recovery_feedback: tuple[RecoveryFeedback, ...] = (),
    ) -> ObservationFrame:
        sim_t = round(float(self.data.time), 4)
        planter_xyz = self.data.sensor("planter_pos").data
        planter_quat = self.data.sensor("planter_quat").data
        scout_xyz = self.data.sensor("scout_pos").data
        scout_quat = self.data.sensor("scout_quat").data

        p_roll, p_pitch, p_yaw = _quat_to_rpy(planter_quat)
        s_roll, s_pitch, s_yaw = _quat_to_rpy(scout_quat)

        planter_pose = Pose2D(
            round(float(planter_xyz[0]), 4),
            round(float(planter_xyz[1]), 4),
            round(p_yaw, 4),
        )
        scout_pose = Pose2D(
            round(float(scout_xyz[0]), 4),
            round(float(scout_xyz[1]), 4),
            round(s_yaw, 4),
        )

        p_speed = round(
            hypot(
                float(self.data.qvel[self._planter_qvel_adr]),
                float(self.data.qvel[self._planter_qvel_adr + 1]),
            ),
            4,
        )
        s_speed = round(
            hypot(
                float(self.data.qvel[self._scout_qvel_adr]),
                float(self.data.qvel[self._scout_qvel_adr + 1]),
            ),
            4,
        )
        inter_dist = round(
            hypot(planter_pose.x_m - scout_pose.x_m, planter_pose.y_m - scout_pose.y_m),
            4,
        )

        p_range = self._read_min_front_range_m("planter")
        s_range = self._read_min_front_range_m("scout")

        _, p_is_bogged = self._compute_robot_fault_and_bogged(
            "planter",
            planter_pose,
            sim_t,
            p_range,
            True,
            round(p_pitch, 4),
            round(p_roll, 4),
            p_speed,
        )
        _, s_is_bogged = self._compute_robot_fault_and_bogged(
            "scout",
            scout_pose,
            sim_t,
            s_range,
            True,
            round(s_pitch, 4),
            round(s_roll, 4),
            s_speed,
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
            )
            for rf in recovery_feedback
        )

        robots = (
            RobotObservation(
                robot_id="planter",
                pose=planter_pose,
                time_s=sim_t,
                front_range_m=p_range,
                seeds_remaining=self._seeds_remaining["planter"],
                samples=self.world.sample_local_soil("planter", planter_pose, sim_t),
                link_ok=True,
                pitch_rad=round(p_pitch, 4),
                roll_rad=round(p_roll, 4),
                true_speed_m_s=p_speed,
                is_bogged=p_is_bogged,
                peer_distances=(("scout", inter_dist),),
            ),
            RobotObservation(
                robot_id="scout",
                pose=scout_pose,
                time_s=sim_t,
                front_range_m=s_range,
                seeds_remaining=self._seeds_remaining["scout"],
                samples=self.world.sample_local_soil("scout", scout_pose, sim_t),
                link_ok=True,
                pitch_rad=round(s_pitch, 4),
                roll_rad=round(s_roll, 4),
                true_speed_m_s=s_speed,
                is_bogged=s_is_bogged,
                peer_distances=(("planter", inter_dist),),
            ),
        )
        return ObservationFrame(
            episode_id=self._episode_id,
            time_s=sim_t,
            robots=robots,
            plant_feedback=adjusted_feedback,
            recovery_feedback=adjusted_recovery,
        )
