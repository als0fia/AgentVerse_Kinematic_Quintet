"""MuJoCo rapid-prototyping implementation of PaddyPatch SimulationPort (rice.v1).

Purpose:
- Implements the exact `SimulationPort` protocol (`reset(seed)` and `step(commands)`)
  from `src/ricebot/runtime.py` using genuine MuJoCo 4.0.0 rigid-body physics,
  rangefinders, and contact detection.
- Zero Isaac Sim imports: designed for rapid local prototyping on 8 GB VRAM laptops
  and clean drop-in translation to a teammate's Isaac Sim machine later behind the
  identical `SimulationPort` seam.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, sin
from typing import Any

import mujoco
import numpy as np

from ..contracts import (
    ObservationFrame,
    PlantFeedback,
    Pose2D,
    RobotCapability,
    RobotCommand,
    RobotObservation,
    SensorKind,
    Site,
    SoilSample,
)


DEFAULT_SITES: tuple[Site, ...] = (
    Site("patch_a", Pose2D(0.90, -0.45, 0.0), permitted=True, no_go_radius_m=0.25),
    Site("patch_b", Pose2D(1.55, 0.00, 0.0), permitted=True, no_go_radius_m=0.25),
    Site("patch_c", Pose2D(1.55, 0.50, 0.0), permitted=True, no_go_radius_m=0.25),
    Site("protected_crop", Pose2D(0.75, 0.55, 0.0), permitted=False, no_go_radius_m=0.25),
)

DEFAULT_CAPABILITIES: tuple[RobotCapability, ...] = (
    RobotCapability("planter", can_sample=True, can_plant=True),
    RobotCapability("scout", can_sample=True, can_plant=False),
)


PADDYPATCH_MJCF = """
<mujoco model="paddypatch_rice_v1_mujoco">
  <option timestep="0.005" gravity="0 0 -9.81" integrator="RK4"/>

  <visual>
    <global offwidth="640" offheight="480"/>
  </visual>

  <worldbody>
    <light name="sun" pos="0.8 0 3.5" dir="0 0 -1" diffuse="0.85 0.85 0.85"/>
    <camera name="top_overview" pos="0.85 0.0 2.6" xyaxes="1 0 0 0 1 0"/>
    <camera name="iso_view" pos="-0.6 -1.5 1.4" xyaxes="0.82 -0.57 0 0.35 0.50 0.79"/>
    <geom name="field_ground" type="plane" size="4 4 0.1" rgba="0.52 0.45 0.36 1"/>

    <!-- Site markers (visual only, non-colliding) -->
    <site name="site_patch_a" pos="0.90 -0.45 0.005" type="cylinder" size="0.16 0.005" rgba="0.78 0.35 0.22 0.55"/>
    <site name="site_patch_b" pos="1.55 0.00 0.005" type="cylinder" size="0.16 0.005" rgba="0.22 0.78 0.35 0.55"/>
    <site name="site_patch_c" pos="1.55 0.50 0.005" type="cylinder" size="0.16 0.005" rgba="0.22 0.65 0.82 0.55"/>
    <site name="site_protected_crop" pos="0.75 0.55 0.02" type="cylinder" size="0.25 0.02" rgba="0.15 0.55 0.18 0.75"/>

    <!-- Planter Rover (blue chassis with front rangefinder and physical seed hopper) -->
    <body name="planter_body" pos="0.10 0.00 0.07">
      <joint name="planter_x" type="slide" axis="1 0 0" damping="12.0"/>
      <joint name="planter_y" type="slide" axis="0 1 0" damping="12.0"/>
      <joint name="planter_yaw" type="hinge" axis="0 0 1" damping="0.8" armature="0.05"/>
      <geom name="planter_chassis" type="box" size="0.11 0.08 0.04" mass="2.2" contype="2" conaffinity="2" rgba="0.18 0.48 0.86 1"/>
      <geom name="planter_hopper_ring" type="cylinder" pos="0 0 0.05" size="0.035 0.015" mass="0.1" contype="0" conaffinity="0" rgba="0.9 0.75 0.15 1"/>
      <site name="planter_range_site" pos="0.115 0 0" zaxis="1 0 0" size="0.01" rgba="1 0.2 0.2 1"/>
      <site name="planter_odom_site" pos="0 0 0" size="0.01" rgba="1 1 1 1"/>
    </body>

    <!-- Scout Rover (green chassis with front rangefinder and soil probe) -->
    <body name="scout_body" pos="0.10 -0.45 0.07">
      <joint name="scout_x" type="slide" axis="1 0 0" damping="12.0"/>
      <joint name="scout_y" type="slide" axis="0 1 0" damping="12.0"/>
      <joint name="scout_yaw" type="hinge" axis="0 0 1" damping="0.8" armature="0.05"/>
      <geom name="scout_chassis" type="box" size="0.10 0.07 0.035" mass="1.6" contype="4" conaffinity="4" rgba="0.20 0.70 0.38 1"/>
      <site name="scout_range_site" pos="0.105 0 0" zaxis="1 0 0" size="0.01" rgba="1 0.2 0.2 1"/>
      <site name="scout_odom_site" pos="0 0 0" size="0.01" rgba="1 1 1 1"/>
    </body>

    <!-- Rigid-body Seed Proxy 0 (6-DOF freejoint, carried in planter hopper until released) -->
    <body name="seed_proxy_0" pos="0.10 0.00 0.14">
      <freejoint name="seed_0_free"/>
      <geom name="seed_0_geom" type="sphere" size="0.018" mass="0.025" contype="1" conaffinity="1" rgba="0.96 0.84 0.14 1"/>
    </body>

    <!-- Rigid-body Seed Proxy 1 -->
    <body name="seed_proxy_1" pos="0.10 0.00 0.18">
      <freejoint name="seed_1_free"/>
      <geom name="seed_1_geom" type="sphere" size="0.018" mass="0.025" contype="1" conaffinity="1" rgba="0.96 0.74 0.10 1"/>
    </body>

    <!-- Mid-episode obstacle (parked below ground at z=-5.0 until surprise injection) -->
    <body name="mid_run_obstacle" mocap="true" pos="0.95 0.00 -5.0">
      <geom name="obstacle_geom" type="box" size="0.10 0.12 0.14" contype="7" conaffinity="7" rgba="0.86 0.24 0.16 1"/>
    </body>
  </worldbody>

  <actuator>
    <motor name="planter_vx" joint="planter_x" gear="12.0" ctrllimited="true" ctrlrange="-0.5 0.5"/>
    <motor name="planter_vy" joint="planter_y" gear="12.0" ctrllimited="true" ctrlrange="-0.5 0.5"/>
    <motor name="planter_wz" joint="planter_yaw" gear="0.8" ctrllimited="true" ctrlrange="-1.5 1.5"/>
    <motor name="scout_vx" joint="scout_x" gear="12.0" ctrllimited="true" ctrlrange="-0.5 0.5"/>
    <motor name="scout_vy" joint="scout_y" gear="12.0" ctrllimited="true" ctrlrange="-0.5 0.5"/>
    <motor name="scout_wz" joint="scout_yaw" gear="0.8" ctrllimited="true" ctrlrange="-1.5 1.5"/>
  </actuator>

  <sensor>
    <rangefinder name="planter_front_range" site="planter_range_site"/>
    <rangefinder name="scout_front_range" site="scout_range_site"/>
    <framepos name="planter_pos" objtype="site" objname="planter_odom_site"/>
    <framepos name="scout_pos" objtype="site" objname="scout_odom_site"/>
  </sensor>
</mujoco>
"""


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
    """Genuine MuJoCo physics implementation of `SimulationPort` (`rice.v1`).

    Hidden ground-truth soil/stand profiles live strictly inside `_hidden_site_truth`
    for independent grading and are only exposed to robots through noisy,
    location-gated `SoilSample` measurements when a robot enters `probe_radius_m`.
    """

    def __init__(
        self,
        sites: tuple[Site, ...] = DEFAULT_SITES,
        probe_radius_m: float = 0.22,
        substeps_per_tick: int = 20,
        inject_obstacle_at_s: float | None = None,
        obstacle_xy: tuple[float, float] = (0.95, 0.00),
    ) -> None:
        self.sites = {s.site_id: s for s in sites}
        self.probe_radius_m = probe_radius_m
        self.substeps_per_tick = substeps_per_tick
        self.inject_obstacle_at_s = inject_obstacle_at_s
        self.obstacle_xy = obstacle_xy

        self.model = mujoco.MjModel.from_xml_string(PADDYPATCH_MJCF)
        self.data = mujoco.MjData(self.model)

        self._ground_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "field_ground")
        self._seed_geom_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"seed_{i}_geom")
            for i in range(2)
        ]
        self._seed_qpos_adrs = [
            int(self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"seed_{i}_free")])
            for i in range(2)
        ]
        self._seed_qvel_adrs = [
            int(self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"seed_{i}_free")])
            for i in range(2)
        ]

        # Hidden synthetic soil & stand-gap truth (NEVER exposed directly to planner)
        # patch_a: waterlogged/soft (moisture=0.91 > 0.80 max), unsuitable for planting
        # patch_b: viable stand gap (gap=0.88, moisture=0.55, firmness=0.62)
        # patch_c: viable stand gap (gap=0.82, moisture=0.52, firmness=0.66)
        self._hidden_site_truth: dict[str, dict[SensorKind, float]] = {
            "patch_a": {
                SensorKind.MOISTURE: 0.91,
                SensorKind.FIRMNESS: 0.18,
                SensorKind.STAND_GAP: 0.85,
            },
            "patch_b": {
                SensorKind.MOISTURE: 0.55,
                SensorKind.FIRMNESS: 0.62,
                SensorKind.STAND_GAP: 0.88,
            },
            "patch_c": {
                SensorKind.MOISTURE: 0.52,
                SensorKind.FIRMNESS: 0.66,
                SensorKind.STAND_GAP: 0.82,
            },
        }

        self._seed: int = 7
        self._episode_id: str = "mujoco-episode-7"
        self._seeds_remaining: dict[str, int] = {"planter": 2, "scout": 0}
        self._active_drops: list[SeedDropEvent] = []
        self._completed_drops: list[SeedDropEvent] = []
        self._sampled_pairs: set[tuple[str, str]] = set()
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
        self._seeds_remaining = {"planter": 2, "scout": 0}
        self._active_drops = []
        self._completed_drops = []
        self._sampled_pairs = set()
        self._obstacle_injected = False

        mujoco.mj_resetData(self.model, self.data)
        # Initial poses: planter at (0.10, 0.00, 0.0), scout at (0.10, -0.45, 0.0)
        self.data.qpos[0:3] = [0.10, 0.00, 0.0]
        self.data.qpos[3:6] = [0.10, -0.45, 0.0]
        self.data.mocap_pos[0] = [self.obstacle_xy[0], self.obstacle_xy[1], -5.0]
        self._sync_carried_seeds()
        mujoco.mj_forward(self.model, self.data)
        return self._build_observation_frame(plant_feedback=())

    def _sync_carried_seeds(self) -> None:
        """Keep unreleased seeds inside the planter's hopper frame."""
        released_indices = {d.seed_index for d in self._active_drops} | {
            d.seed_index for d in self._completed_drops
        }
        px, py = float(self.data.qpos[0]), float(self.data.qpos[1])
        for idx in range(2):
            if idx not in released_indices:
                qadr = self._seed_qpos_adrs[idx]
                vadr = self._seed_qvel_adrs[idx]
                self.data.qpos[qadr : qadr + 3] = [px, py, 0.14 + 0.04 * idx]
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
            self.data.mocap_pos[0] = [self.obstacle_xy[0], self.obstacle_xy[1], 0.08]
            self._obstacle_injected = True
            mujoco.mj_forward(self.model, self.data)

        # Process commands for planter and scout (reject stale/expired commands independently)
        self.data.ctrl[:] = 0.0
        for robot_id, qpos_offset, ctrl_offset in (("planter", 0, 0), ("scout", 3, 3)):
            cmd = cmd_by_robot.get(robot_id)
            if cmd is None or not (cmd.issued_at_s <= now_s < cmd.expires_at_s):
                continue

            if cmd.release_seed:
                # Only planter with remaining seeds and zero drive can physically release a seed
                if (
                    robot_id == "planter"
                    and self._seeds_remaining["planter"] > 0
                    and cmd.target_site_id in self.sites
                    and cmd.linear_m_s == 0.0
                    and cmd.angular_rad_s == 0.0
                ):
                    seed_idx = 2 - self._seeds_remaining["planter"]
                    self._seeds_remaining["planter"] -= 1
                    px, py = float(self.data.qpos[qpos_offset]), float(self.data.qpos[qpos_offset + 1])
                    qadr = self._seed_qpos_adrs[seed_idx]
                    vadr = self._seed_qvel_adrs[seed_idx]
                    # Release from hopper nozzle at z=0.06m so gravity drops it onto field_ground
                    self.data.qpos[qadr : qadr + 3] = [px, py, 0.06]
                    self.data.qpos[qadr + 3 : qadr + 7] = [1.0, 0.0, 0.0, 0.0]
                    self.data.qvel[vadr : vadr + 6] = [0.0, 0.0, -0.05, 0.0, 0.0, 0.0]
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
                yaw = float(self.data.qpos[qpos_offset + 2])
                lin = float(np.clip(cmd.linear_m_s, -0.40, 0.40))
                ang = float(np.clip(cmd.angular_rad_s, -1.20, 1.20))
                self.data.ctrl[ctrl_offset + 0] = lin * cos(yaw)
                self.data.ctrl[ctrl_offset + 1] = lin * sin(yaw)
                self.data.ctrl[ctrl_offset + 2] = ang

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

    def _read_range_m(self, sensor_name: str) -> float:
        raw = float(self.data.sensor(sensor_name).data[0])
        # MuJoCo rangefinder returns -1.0 when no geom is hit within max range
        if raw < 0.0:
            return 4.0
        return round(raw, 4)

    def _local_samples_for_robot(self, robot_id: str, pose: Pose2D, time_s: float) -> tuple[SoilSample, ...]:
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

    def _build_observation_frame(self, plant_feedback: tuple[PlantFeedback, ...]) -> ObservationFrame:
        sim_t = round(float(self.data.time), 4)
        planter_xyz = self.data.sensor("planter_pos").data
        scout_xyz = self.data.sensor("scout_pos").data

        planter_pose = Pose2D(
            round(float(planter_xyz[0]), 4),
            round(float(planter_xyz[1]), 4),
            round(float(self.data.qpos[2]), 4),
        )
        scout_pose = Pose2D(
            round(float(scout_xyz[0]), 4),
            round(float(scout_xyz[1]), 4),
            round(float(self.data.qpos[5]), 4),
        )

        # Ensure any PlantFeedback timestamp matches or is <= frame time_s per ObservationFrame invariant
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
                front_range_m=self._read_range_m("planter_front_range"),
                seeds_remaining=self._seeds_remaining["planter"],
                samples=self._local_samples_for_robot("planter", planter_pose, sim_t),
                link_ok=True,
            ),
            RobotObservation(
                robot_id="scout",
                pose=scout_pose,
                time_s=sim_t,
                front_range_m=self._read_range_m("scout_front_range"),
                seeds_remaining=self._seeds_remaining["scout"],
                samples=self._local_samples_for_robot("scout", scout_pose, sim_t),
                link_ok=True,
            ),
        )
        return ObservationFrame(
            episode_id=self._episode_id,
            time_s=sim_t,
            robots=robots,
            plant_feedback=adjusted_feedback,
        )
