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

from dataclasses import dataclass
from math import asin, atan2, cos, hypot, sin

import mujoco
import numpy as np

from ..contracts import (
    ObservationFrame,
    PlantFeedback,
    Pose2D,
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
)
from ..world import (
    ContinuousTerrainPoint,
    DEFAULT_SITES,
    PaddyTerraceWorld,
    build_world_terrain_xml,
)


def compose_paddypatch_mjcf(obstacle_xy: tuple[float, float] = (0.75, -0.25)) -> str:
    """Compose the full MJCF XML from `src/ricebot/world/` terrain and `src/ricebot/robot/` Paddy Ant units."""
    world_terrain_xml = build_world_terrain_xml(obstacle_xy=obstacle_xy)
    rovers_bodies_xml, rovers_actuators_xml, rovers_sensors_xml = build_rovers_and_seeds_xml()
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
    <geom name="paddy_relief_mesh" type="hfield" hfield="paddy_micro_relief" pos="1.35 0.0 -0.016" rgba="0.41 0.33 0.22 0.46" contype="0" conaffinity="0"/>
    {rovers_bodies_xml}
  </worldbody>

  <equality>
    <connect name="rigid_latch_planter_to_scout" site1="planter_front_dock_site" site2="scout_rear_dock_site" active="false" solref="0.008 1.0"/>
    <connect name="rigid_latch_scout_to_planter" site1="scout_front_dock_site" site2="planter_rear_dock_site" active="false" solref="0.008 1.0"/>
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
    ) -> None:
        self.world = PaddyTerraceWorld(
            sites=sites, probe_radius_m=probe_radius_m, hidden_truth=hidden_truth
        )
        self.sites = self.world.sites
        self.probe_radius_m = probe_radius_m
        self.substeps_per_tick = substeps_per_tick
        self.inject_obstacle_at_s = inject_obstacle_at_s
        self.obstacle_xy = obstacle_xy

        mjcf_xml = compose_paddypatch_mjcf(obstacle_xy=obstacle_xy)
        self.model = mujoco.MjModel.from_xml_string(mjcf_xml)
        self.data = mujoco.MjData(self.model)

        self._ground_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "field_ground")
        self._seed_geom_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"seed_{i}_geom")
            for i in range(PLANTER_SPEC.initial_seeds)
        ]
        self._planter_qpos_adr = int(
            self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "planter_free")]
        )
        self._scout_qpos_adr = int(
            self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "scout_free")]
        )
        self._seed_qpos_adrs = [
            int(self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"seed_{i}_free")])
            for i in range(PLANTER_SPEC.initial_seeds)
        ]
        self._seed_qvel_adrs = [
            int(self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"seed_{i}_free")])
            for i in range(PLANTER_SPEC.initial_seeds)
        ]

        # Optional active farming end-effector actuator IDs from PR #24 (dibber plunger + finger weeders)
        self._dibber_act_ids: dict[str, int] = {}
        self._weeder_l_act_ids: dict[str, int] = {}
        self._weeder_r_act_ids: dict[str, int] = {}
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

        # Runtime-configurable vehicle sizing parameters for live Bekker-Wong wheel-mud sinkage physics
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
        self.mud_rut_history: list[tuple[str, float, float, float, float]] = []

        self._seed: int = 7
        self._episode_id: str = "mujoco-episode-7"
        self._seeds_remaining: dict[str, int] = {
            PLANTER_SPEC.robot_id: PLANTER_SPEC.initial_seeds,
            SCOUT_SPEC.robot_id: SCOUT_SPEC.initial_seeds,
        }
        self._active_drops: list[SeedDropEvent] = []
        self._completed_drops: list[SeedDropEvent] = []
        self._obstacle_injected: bool = False
        self._populate_hfield_from_continuous_terrain()

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
    def obstacle_injected(self) -> bool:
        return self._obstacle_injected

    def _update_rover_terramechanics(self) -> None:
        """Evaluate ContinuousPaddyTerrain and Bekker-Wong wheel-mud sinkage at each rover's live 3D pose."""
        for spec in (PLANTER_SPEC, SCOUT_SPEC):
            rid = spec.robot_id
            r_xyz = self.data.sensor(f"{rid}_pos").data
            rx, ry = float(r_xyz[0]), float(r_xyz[1])
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
            if sink_state.bellied_out:
                local_slip = 0.96
            self.latest_terrain_points[rid] = pt
            self.latest_terramechanics[rid] = sink_state
            self.latest_slip_ratios[rid] = round(local_slip, 4)

    def _apply_3d_terrain_and_mud_sinkage_pose(self) -> None:
        """Keep physics qpos stable for horizontal LiDAR rays unless rover is bellied out in deep mud."""
        for spec, qadr in ((PLANTER_SPEC, self._planter_qpos_adr), (SCOUT_SPEC, self._scout_qpos_adr)):
            rid = spec.robot_id
            sink = self.latest_terramechanics.get(rid)
            if sink is not None and sink.bellied_out:
                # Visually sink chassis down into the mud trap when bellied out (S1 immobilized)
                self.data.qpos[qadr + 2] = max(0.042, spec.initial_xyz[2] - 0.55 * sink.sinkage_z_m)

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
        self._obstacle_injected = False
        self.mud_rut_history = []

        mujoco.mj_resetData(self.model, self.data)
        px, py, pz = PLANTER_SPEC.initial_xyz
        sx, sy, sz = SCOUT_SPEC.initial_xyz
        self.data.qpos[self._planter_qpos_adr : self._planter_qpos_adr + 7] = [
            px, py, pz, 1.0, 0.0, 0.0, 0.0
        ]
        self.data.qpos[self._scout_qpos_adr : self._scout_qpos_adr + 7] = [
            sx, sy, sz, 1.0, 0.0, 0.0, 0.0
        ]
        self.data.mocap_pos[0] = [self.obstacle_xy[0], self.obstacle_xy[1], -5.0]
        self._sync_carried_seeds()
        mujoco.mj_forward(self.model, self.data)
        self._update_rover_terramechanics()
        self._apply_3d_terrain_and_mud_sinkage_pose()
        mujoco.mj_forward(self.model, self.data)
        return self._build_observation_frame(plant_feedback=())

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

    def step(self, commands: tuple[RobotCommand, ...]) -> ObservationFrame:
        now_s = round(float(self.data.time), 4)
        cmd_by_robot = {c.robot_id: c for c in commands}

        if (
            self.inject_obstacle_at_s is not None
            and not self._obstacle_injected
            and now_s >= self.inject_obstacle_at_s
        ):
            self.data.mocap_pos[0] = [self.obstacle_xy[0], self.obstacle_xy[1], 0.12]
            self._obstacle_injected = True
            mujoco.mj_forward(self.model, self.data)

        self._update_rover_terramechanics()
        self.data.ctrl[:] = 0.0
        for spec, ctrl_offset in ((PLANTER_SPEC, 0), (SCOUT_SPEC, 4)):
            robot_id = spec.robot_id
            cmd = cmd_by_robot.get(robot_id)
            if cmd is None or not (cmd.issued_at_s <= now_s < cmd.expires_at_s):
                continue

            dib_id = self._dibber_act_ids.get(robot_id, -1)
            wl_id = self._weeder_l_act_ids.get(robot_id, -1)
            wr_id = self._weeder_r_act_ids.get(robot_id, -1)

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
                # Apply analytical Bekker-Wong wheel-mud traction, slip churn & belly-out immobilization physics
                sink_state = self.latest_terramechanics.get(robot_id)
                slip_ratio = self.latest_slip_ratios.get(robot_id, 0.08)
                is_custom_sizing = (
                    abs(self.robot_mass_kg.get(robot_id, 4.2 if robot_id == "planter" else 3.6) - (4.2 if robot_id == "planter" else 3.6)) > 0.3
                    or abs(self.robot_wheel_radius_m.get(robot_id, 0.058) - 0.058) > 0.003
                )
                if sink_state is not None:
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
        for substep_i in range(self.substeps_per_tick):
            self._sync_carried_seeds()
            if substep_i % 5 == 0:
                self._apply_3d_terrain_and_mud_sinkage_pose()
            mujoco.mj_step(self.model, self.data)
            step_t = round(float(self.data.time), 4)
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
        self._apply_3d_terrain_and_mud_sinkage_pose()
        mujoco.mj_forward(self.model, self.data)
        return self._build_observation_frame(plant_feedback=tuple(new_feedback))

    def _read_min_front_range_m(self, prefix: str) -> float:
        """Read minimum distance across the 3-ray forward LiDAR fan (center, left, right)."""
        vals: list[float] = []
        for suffix in ("c", "l", "r"):
            raw = float(self.data.sensor(f"{prefix}_range_{suffix}").data[0])
            if raw >= 0.0:
                vals.append(raw)
        return round(min(vals), 4) if vals else 4.0

    def _build_observation_frame(self, plant_feedback: tuple[PlantFeedback, ...]) -> ObservationFrame:
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

        p_sink = self.latest_terramechanics.get("planter")
        s_sink = self.latest_terramechanics.get("scout")

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

        robots = (
            RobotObservation(
                robot_id="planter",
                pose=planter_pose,
                time_s=sim_t,
                front_range_m=self._read_min_front_range_m("planter"),
                seeds_remaining=self._seeds_remaining["planter"],
                samples=self.world.sample_local_soil("planter", planter_pose, sim_t),
                link_ok=True,
                pitch_rad=round(p_pitch, 4),
                roll_rad=round(p_roll, 4),
                is_bogged=bool(p_sink.bellied_out) if p_sink is not None else False,
            ),
            RobotObservation(
                robot_id="scout",
                pose=scout_pose,
                time_s=sim_t,
                front_range_m=self._read_min_front_range_m("scout"),
                seeds_remaining=self._seeds_remaining["scout"],
                samples=self.world.sample_local_soil("scout", scout_pose, sim_t),
                link_ok=True,
                pitch_rad=round(s_pitch, 4),
                roll_rad=round(s_roll, 4),
                is_bogged=bool(s_sink.bellied_out) if s_sink is not None else False,
            ),
        )
        return ObservationFrame(
            episode_id=self._episode_id,
            time_s=sim_t,
            robots=robots,
            plant_feedback=adjusted_feedback,
        )
