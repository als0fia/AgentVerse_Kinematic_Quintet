"""MuJoCo 3D paddy-terrace & wheeled-rover physics adapter implementing `SimulationPort` (`rice.v1`).

Architectural Composition per `docs/COMPONENTS.md` & `docs/ARCHITECTURE.md`:
- Composes:
  1. `src/ricebot/world/` (`PaddyTerraceWorld`, `DEFAULT_SITES`, `build_world_terrain_xml`) — owns
     the 6-patch rice terrace layout, hidden soil/stand truth, location-gated soil probes, and
     terrain/bunds/crops MJCF geometry.
  2. `src/ricebot/robot/` (`PLANTER_SPEC`, `SCOUT_SPEC`, `DEFAULT_CAPABILITIES`,
     `compute_4wheel_skid_steer`, `build_rovers_and_seeds_xml`) — owns the 4-wheel differential
     rover chassis, skid-steer kinematics, rangefinder/odometry mount frames, and seed-hopper
     embodiment.
- Implements `SimulationPort` (`reset(seed) -> ObservationFrame`, `step(commands) -> ObservationFrame`)
  without touching `MissionCore` (`src/ricebot/mission.py`) or `src/ricebot/contracts.py`.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import atan2

import mujoco
import numpy as np

from ..contracts import (
    ObservationFrame,
    PlantFeedback,
    Pose2D,
    RobotCommand,
    RobotObservation,
    Site,
)
from ..robot import (
    DEFAULT_CAPABILITIES,
    PLANTER_SPEC,
    SCOUT_SPEC,
    build_rovers_and_seeds_xml,
    compute_4wheel_skid_steer,
)
from ..world import (
    DEFAULT_SITES,
    PaddyTerraceWorld,
    build_world_terrain_xml,
)


def compose_paddypatch_mjcf(obstacle_xy: tuple[float, float] = (0.75, -0.25)) -> str:
    """Compose the full MJCF XML from `src/ricebot/world/` terrain and `src/ricebot/robot/` rovers."""
    world_terrain_xml = build_world_terrain_xml(obstacle_xy=obstacle_xy)
    rovers_bodies_xml, rovers_actuators_xml, rovers_sensors_xml = build_rovers_and_seeds_xml()
    return f"""
<mujoco model="paddypatch_rice_v1_terrace">
  <option timestep="0.004" gravity="0 0 -9.81" integrator="implicitfast"/>

  <visual>
    <global offwidth="960" offheight="720" azimuth="115" elevation="-34"/>
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
  </asset>

  <worldbody>
    {world_terrain_xml}
    {rovers_bodies_xml}
  </worldbody>

  <actuator>
    {rovers_actuators_xml}
  </actuator>

  <sensor>
    {rovers_sensors_xml}
  </sensor>
</mujoco>
"""


ANTENNA_MJCF = compose_paddypatch_mjcf()


def _quat_to_yaw(quat_wxyz: np.ndarray) -> float:
    """Extract planar yaw angle (radians) from MuJoCo quaternion [w, x, y, z]."""
    w, x, y, z = float(quat_wxyz[0]), float(quat_wxyz[1]), float(quat_wxyz[2]), float(quat_wxyz[3])
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return atan2(siny_cosp, cosy_cosp)


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
    """Genuine 3D wheeled-rover & rice-terrace MuJoCo implementation of `SimulationPort` (`rice.v1`)."""

    def __init__(
        self,
        sites: tuple[Site, ...] = DEFAULT_SITES,
        probe_radius_m: float = 0.28,
        substeps_per_tick: int = 25,
        inject_obstacle_at_s: float | None = None,
        obstacle_xy: tuple[float, float] = (0.75, -0.25),
    ) -> None:
        self.world = PaddyTerraceWorld(sites=sites, probe_radius_m=probe_radius_m)
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

        self._seed: int = 7
        self._episode_id: str = "mujoco-episode-7"
        self._seeds_remaining: dict[str, int] = {
            PLANTER_SPEC.robot_id: PLANTER_SPEC.initial_seeds,
            SCOUT_SPEC.robot_id: SCOUT_SPEC.initial_seeds,
        }
        self._active_drops: list[SeedDropEvent] = []
        self._completed_drops: list[SeedDropEvent] = []
        self._obstacle_injected: bool = False

    @property
    def completed_drops(self) -> tuple[SeedDropEvent, ...]:
        """Independent physics record of released seeds and verified ground contacts."""
        return tuple(self._completed_drops)

    @property
    def obstacle_injected(self) -> bool:
        return self._obstacle_injected

    def reset(self, seed: int) -> ObservationFrame:
        self._seed = int(seed)
        self._episode_id = f"mujoco-ep-{self._seed}"
        self.world.reset(self._seed)
        self._seeds_remaining = {
            PLANTER_SPEC.robot_id: PLANTER_SPEC.initial_seeds,
            SCOUT_SPEC.robot_id: SCOUT_SPEC.initial_seeds,
        }
        self._active_drops = []
        self._completed_drops = []
        self._obstacle_injected = False

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

        # Apply mid-episode obstacle injection if configured
        if (
            self.inject_obstacle_at_s is not None
            and not self._obstacle_injected
            and now_s >= self.inject_obstacle_at_s
        ):
            self.data.mocap_pos[0] = [self.obstacle_xy[0], self.obstacle_xy[1], 0.12]
            self._obstacle_injected = True
            mujoco.mj_forward(self.model, self.data)

        # Process commands for planter (ctrl 0..3) and scout (ctrl 4..7) via src/ricebot/robot/
        self.data.ctrl[:] = 0.0
        for spec, ctrl_offset in ((PLANTER_SPEC, 0), (SCOUT_SPEC, 4)):
            robot_id = spec.robot_id
            cmd = cmd_by_robot.get(robot_id)
            if cmd is None or not (cmd.issued_at_s <= now_s < cmd.expires_at_s):
                continue

            if cmd.release_seed:
                if (
                    spec.can_plant
                    and self._seeds_remaining[robot_id] > 0
                    and cmd.target_site_id in self.sites
                    and cmd.linear_m_s == 0.0
                    and cmd.angular_rad_s == 0.0
                ):
                    seed_idx = spec.initial_seeds - self._seeds_remaining[robot_id]
                    self._seeds_remaining[robot_id] -= 1
                    planter_xyz = self.data.sensor("planter_pos").data
                    px, py = float(planter_xyz[0]), float(planter_xyz[1])
                    qadr = self._seed_qpos_adrs[seed_idx]
                    vadr = self._seed_qvel_adrs[seed_idx]
                    # Drop from bottom hopper chute at spec.hopper_drop_z_m so gravity lands it on the soil bed
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
                w_fl, w_fr, w_rl, w_rr = compute_4wheel_skid_steer(
                    spec, cmd.linear_m_s, cmd.angular_rad_s
                )
                self.data.ctrl[ctrl_offset + 0] = w_fl
                self.data.ctrl[ctrl_offset + 1] = w_fr
                self.data.ctrl[ctrl_offset + 2] = w_rl
                self.data.ctrl[ctrl_offset + 3] = w_rr

        # Advance MuJoCo physics substeps and detect physical seed-ground contact
        new_feedback: list[PlantFeedback] = []
        for _ in range(self.substeps_per_tick):
            self._sync_carried_seeds()
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

        planter_pose = Pose2D(
            round(float(planter_xyz[0]), 4),
            round(float(planter_xyz[1]), 4),
            round(_quat_to_yaw(planter_quat), 4),
        )
        scout_pose = Pose2D(
            round(float(scout_xyz[0]), 4),
            round(float(scout_xyz[1]), 4),
            round(_quat_to_yaw(scout_quat), 4),
        )

        adjusted_feedback = tuple(
            PlantFeedback(
                robot_id=fb.robot_id,
                task_id=fb.task_id,
                released=fb.released,
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
            ),
            RobotObservation(
                robot_id="scout",
                pose=scout_pose,
                time_s=sim_t,
                front_range_m=self._read_min_front_range_m("scout"),
                seeds_remaining=self._seeds_remaining["scout"],
                samples=self.world.sample_local_soil("scout", scout_pose, sim_t),
                link_ok=True,
            ),
        )
        return ObservationFrame(
            episode_id=self._episode_id,
            time_s=sim_t,
            robots=robots,
            plant_feedback=adjusted_feedback,
        )
