"""Wheeled rover chassis, 4-wheel skid-steer kinematics, sensors, and seed-hopper embodiment (`src/ricebot/robot/`).

Strict separation per `docs/COMPONENTS.md`:
- Owns rover chassis specs (`RoverChassisSpec`), 4-wheel differential skid-steer kinematics
  (`compute_4wheel_skid_steer`), forward rangefinder ray fan & odometry mount frames,
  seed-hopper release nozzle geometry, and robot-only MJCF body/actuator/sensor builders.
- NEVER owns hidden soil truth, field site coordinates, or mission planning.
"""
from __future__ import annotations

from dataclasses import dataclass

try:
    import numpy as np

    def _clip(val: float, low: float, high: float) -> float:
        return float(np.clip(val, low, high))
except ImportError:
    def _clip(val: float, low: float, high: float) -> float:
        return float(max(low, min(high, val)))

from ..contracts import RobotCapability


@dataclass(frozen=True)
class RoverChassisSpec:
    """Physical and kinematic parameters for a 4-wheel differential-drive paddy rover."""

    robot_id: str
    can_sample: bool
    can_plant: bool
    wheel_radius_m: float
    half_track_m: float
    max_wheel_rad_s: float
    max_linear_m_s: float
    max_angular_rad_s: float
    initial_xyz: tuple[float, float, float]
    hopper_drop_z_m: float
    initial_seeds: int

    @property
    def capability(self) -> RobotCapability:
        return RobotCapability(
            robot_id=self.robot_id,
            can_sample=self.can_sample,
            can_plant=self.can_plant,
        )


PLANTER_SPEC = RoverChassisSpec(
    robot_id="planter",
    can_sample=True,
    can_plant=True,
    wheel_radius_m=0.058,
    half_track_m=0.118,
    max_wheel_rad_s=14.0,
    max_linear_m_s=0.45,
    max_angular_rad_s=1.40,
    initial_xyz=(0.10, -0.05, 0.058),
    hopper_drop_z_m=0.032,
    initial_seeds=2,
)

SCOUT_SPEC = RoverChassisSpec(
    robot_id="scout",
    can_sample=True,
    can_plant=False,
    wheel_radius_m=0.054,
    half_track_m=0.105,
    max_wheel_rad_s=14.0,
    max_linear_m_s=0.45,
    max_angular_rad_s=1.40,
    initial_xyz=(0.10, -0.55, 0.054),
    hopper_drop_z_m=0.0,
    initial_seeds=0,
)

DEFAULT_CAPABILITIES: tuple[RobotCapability, ...] = (
    PLANTER_SPEC.capability,
    SCOUT_SPEC.capability,
)


def compute_4wheel_skid_steer(
    spec: RoverChassisSpec,
    linear_m_s: float,
    angular_rad_s: float,
) -> tuple[float, float, float, float]:
    """Convert bounded `(linear_m_s, angular_rad_s)` into `(w_fl, w_fr, w_rl, w_rr)` in `rad/s`.

    Uses a 1.85x differential skid-steer slip factor tuned for 4-wheel rubber contact on paddy soil.
    This identical function drives MuJoCo `<velocity>` wheel servos and Isaac Sim `ArticulationAction`
    joint velocity targets.
    """
    lin = _clip(linear_m_s, -spec.max_linear_m_s, spec.max_linear_m_s)
    ang = _clip(angular_rad_s, -spec.max_angular_rad_s, spec.max_angular_rad_s)
    v_left = lin - ang * spec.half_track_m * 1.85
    v_right = lin + ang * spec.half_track_m * 1.85
    w_left = _clip(v_left / spec.wheel_radius_m, -spec.max_wheel_rad_s, spec.max_wheel_rad_s)
    w_right = _clip(v_right / spec.wheel_radius_m, -spec.max_wheel_rad_s, spec.max_wheel_rad_s)
    return (w_left, w_right, w_left, w_right)


def build_rovers_and_seeds_xml() -> tuple[str, str, str]:
    """Return `(bodies_xml, actuators_xml, sensors_xml)` for the `planter`, `scout`, and 2 seed proxies."""
    bodies_xml = """
    <!-- ========================================================================= -->
    <!-- ROVER 1: PLANTER ROVER (4-Wheel Articulated Chassis + Seed Hopper + LiDAR) -->
    <!-- ========================================================================= -->
    <body name="planter_body" pos="0.10 -0.05 0.068">
      <freejoint name="planter_free"/>
      <!-- Main waterproof chassis -->
      <geom name="planter_chassis" type="box" pos="0 0 0.015" size="0.135 0.095 0.032" mass="3.2" contype="2" conaffinity="5" rgba="0.16 0.46 0.86 1"/>
      <!-- Top electronics deck & BeiDou RTK mast -->
      <geom name="planter_deck" type="box" pos="-0.02 0 0.055" size="0.08 0.07 0.012" mass="0.2" contype="0" conaffinity="0" rgba="0.92 0.94 0.96 1"/>
      <geom name="planter_rtk_mast" type="cylinder" pos="-0.07 0.05 0.095" size="0.008 0.035" mass="0.05" contype="0" conaffinity="0" rgba="0.85 0.85 0.85 1"/>
      <geom name="planter_rtk_dome" type="sphere" pos="-0.07 0.05 0.135" size="0.018" mass="0.02" contype="0" conaffinity="0" rgba="0.96 0.96 0.98 1"/>
      <!-- Seed Hopper Funnel & Drop Chute -->
      <geom name="planter_hopper_outer" type="cylinder" pos="0.03 0 0.075" size="0.042 0.028" mass="0.15" contype="0" conaffinity="0" rgba="0.95 0.76 0.14 1"/>
      <geom name="planter_hopper_inner" type="cylinder" pos="0.03 0 0.104" size="0.032 0.004" mass="0.01" contype="0" conaffinity="0" rgba="0.22 0.22 0.24 1"/>
      <!-- Front LiDAR / Rangefinder Turret (3-ray fan: center, +18 deg left, -18 deg right) -->
      <geom name="planter_lidar_pod" type="cylinder" pos="0.115 0 0.055" size="0.022 0.014" mass="0.05" contype="0" conaffinity="0" rgba="0.18 0.18 0.20 1"/>
      <site name="planter_range_c" pos="0.140 0.00 0.02" zaxis="1 0 0" size="0.008" rgba="1 0.2 0.2 1"/>
      <site name="planter_range_l" pos="0.138 0.04 0.02" zaxis="0.951 0.309 0" size="0.006" rgba="1 0.4 0.2 1"/>
      <site name="planter_range_r" pos="0.138 -0.04 0.02" zaxis="0.951 -0.309 0" size="0.006" rgba="1 0.4 0.2 1"/>
      <site name="planter_odom_site" pos="0 0 0" size="0.01" rgba="1 1 1 1"/>

      <!-- 4 Revolute Paddy Wheels (radius=0.058m, half-width=0.020m) -->
      <body name="planter_wheel_fl" pos="0.085 0.118 0.0">
        <joint name="planter_j_fl" type="hinge" axis="0 1 0" damping="0.08" armature="0.005"/>
        <geom name="planter_w_fl" type="cylinder" size="0.058 0.020" zaxis="0 1 0" mass="0.35" material="wheel_rubber_mat" contype="2" conaffinity="1" friction="1.4 0.01 0.005"/>
        <geom name="planter_hub_fl" type="box" pos="0 0.021 0" size="0.038 0.002 0.010" contype="0" conaffinity="0" rgba="0.90 0.75 0.15 1"/>
      </body>
      <body name="planter_wheel_fr" pos="0.085 -0.118 0.0">
        <joint name="planter_j_fr" type="hinge" axis="0 1 0" damping="0.08" armature="0.005"/>
        <geom name="planter_w_fr" type="cylinder" size="0.058 0.020" zaxis="0 1 0" mass="0.35" material="wheel_rubber_mat" contype="2" conaffinity="1" friction="1.4 0.01 0.005"/>
        <geom name="planter_hub_fr" type="box" pos="0 -0.021 0" size="0.038 0.002 0.010" contype="0" conaffinity="0" rgba="0.90 0.75 0.15 1"/>
      </body>
      <body name="planter_wheel_rl" pos="-0.085 0.118 0.0">
        <joint name="planter_j_rl" type="hinge" axis="0 1 0" damping="0.08" armature="0.005"/>
        <geom name="planter_w_rl" type="cylinder" size="0.058 0.020" zaxis="0 1 0" mass="0.35" material="wheel_rubber_mat" contype="2" conaffinity="1" friction="1.4 0.01 0.005"/>
        <geom name="planter_hub_rl" type="box" pos="0 0.021 0" size="0.038 0.002 0.010" contype="0" conaffinity="0" rgba="0.90 0.75 0.15 1"/>
      </body>
      <body name="planter_wheel_rr" pos="-0.085 -0.118 0.0">
        <joint name="planter_j_rr" type="hinge" axis="0 1 0" damping="0.08" armature="0.005"/>
        <geom name="planter_w_rr" type="cylinder" size="0.058 0.020" zaxis="0 1 0" mass="0.35" material="wheel_rubber_mat" contype="2" conaffinity="1" friction="1.4 0.01 0.005"/>
        <geom name="planter_hub_rr" type="box" pos="0 -0.021 0" size="0.038 0.002 0.010" contype="0" conaffinity="0" rgba="0.90 0.75 0.15 1"/>
      </body>
    </body>

    <!-- ========================================================================= -->
    <!-- ROVER 2: SCOUT ROVER (4-Wheel Agile Soil-Profiling & Stand-Mapping Rover) -->
    <!-- ========================================================================= -->
    <body name="scout_body" pos="0.10 -0.55 0.064">
      <freejoint name="scout_free"/>
      <!-- Agile lightweight green chassis -->
      <geom name="scout_chassis" type="box" pos="0 0 0.012" size="0.120 0.085 0.028" mass="2.4" contype="4" conaffinity="5" rgba="0.18 0.72 0.36 1"/>
      <!-- Soil Cone-Index / Moisture Probe Mast & Sensor Pod -->
      <geom name="scout_deck" type="box" pos="0 0 0.048" size="0.075 0.065 0.010" mass="0.15" contype="0" conaffinity="0" rgba="0.90 0.95 0.90 1"/>
      <geom name="scout_probe_boom" type="cylinder" pos="-0.04 0 0.085" size="0.012 0.030" mass="0.05" contype="0" conaffinity="0" rgba="0.25 0.85 0.92 1"/>
      <geom name="scout_lidar_pod" type="cylinder" pos="0.095 0 0.048" size="0.020 0.012" mass="0.04" contype="0" conaffinity="0" rgba="0.18 0.18 0.20 1"/>
      <site name="scout_range_c" pos="0.125 0.00 0.02" zaxis="1 0 0" size="0.008" rgba="0.2 1.0 0.4 1"/>
      <site name="scout_range_l" pos="0.122 0.04 0.02" zaxis="0.951 0.309 0" size="0.006" rgba="0.2 1.0 0.4 1"/>
      <site name="scout_range_r" pos="0.122 -0.04 0.02" zaxis="0.951 -0.309 0" size="0.006" rgba="0.2 1.0 0.4 1"/>
      <site name="scout_odom_site" pos="0 0 0" size="0.01" rgba="1 1 1 1"/>

      <!-- 4 Revolute Paddy Wheels (radius=0.054m, half-width=0.018m) -->
      <body name="scout_wheel_fl" pos="0.075 0.105 0.0">
        <joint name="scout_j_fl" type="hinge" axis="0 1 0" damping="0.06" armature="0.004"/>
        <geom name="scout_w_fl" type="cylinder" size="0.054 0.018" zaxis="0 1 0" mass="0.28" material="wheel_rubber_mat" contype="4" conaffinity="1" friction="1.4 0.01 0.005"/>
        <geom name="scout_hub_fl" type="box" pos="0 0.019 0" size="0.034 0.002 0.009" contype="0" conaffinity="0" rgba="0.25 0.90 0.50 1"/>
      </body>
      <body name="scout_wheel_fr" pos="0.075 -0.105 0.0">
        <joint name="scout_j_fr" type="hinge" axis="0 1 0" damping="0.06" armature="0.004"/>
        <geom name="scout_w_fr" type="cylinder" size="0.054 0.018" zaxis="0 1 0" mass="0.28" material="wheel_rubber_mat" contype="4" conaffinity="1" friction="1.4 0.01 0.005"/>
        <geom name="scout_hub_fr" type="box" pos="0 -0.019 0" size="0.034 0.002 0.009" contype="0" conaffinity="0" rgba="0.25 0.90 0.50 1"/>
      </body>
      <body name="scout_wheel_rl" pos="-0.075 0.105 0.0">
        <joint name="scout_j_rl" type="hinge" axis="0 1 0" damping="0.06" armature="0.004"/>
        <geom name="scout_w_rl" type="cylinder" size="0.054 0.018" zaxis="0 1 0" mass="0.28" material="wheel_rubber_mat" contype="4" conaffinity="1" friction="1.4 0.01 0.005"/>
        <geom name="scout_hub_rl" type="box" pos="0 0.019 0" size="0.034 0.002 0.009" contype="0" conaffinity="0" rgba="0.25 0.90 0.50 1"/>
      </body>
      <body name="scout_wheel_rr" pos="-0.075 -0.105 0.0">
        <joint name="scout_j_rr" type="hinge" axis="0 1 0" damping="0.06" armature="0.004"/>
        <geom name="scout_w_rr" type="cylinder" size="0.054 0.018" zaxis="0 1 0" mass="0.28" material="wheel_rubber_mat" contype="4" conaffinity="1" friction="1.4 0.01 0.005"/>
        <geom name="scout_hub_rr" type="box" pos="0 -0.019 0" size="0.034 0.002 0.009" contype="0" conaffinity="0" rgba="0.25 0.90 0.50 1"/>
      </body>
    </body>

    <!-- Rigid-body Seed Proxy 0 (6-DOF freejoint, carried in planter hopper until released) -->
    <body name="seed_proxy_0" pos="0.13 -0.05 0.18">
      <freejoint name="seed_0_free"/>
      <geom name="seed_0_geom" type="sphere" size="0.022" mass="0.03" contype="1" conaffinity="1" rgba="0.98 0.86 0.12 1"/>
    </body>

    <!-- Rigid-body Seed Proxy 1 -->
    <body name="seed_proxy_1" pos="0.13 -0.05 0.23">
      <freejoint name="seed_1_free"/>
      <geom name="seed_1_geom" type="sphere" size="0.022" mass="0.03" contype="1" conaffinity="1" rgba="0.98 0.76 0.08 1"/>
    </body>
"""
    actuators_xml = """
    <!-- Planter 4-Wheel Velocity Servos (rad/s) -->
    <velocity name="planter_act_fl" joint="planter_j_fl" kv="1.2" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <velocity name="planter_act_fr" joint="planter_j_fr" kv="1.2" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <velocity name="planter_act_rl" joint="planter_j_rl" kv="1.2" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <velocity name="planter_act_rr" joint="planter_j_rr" kv="1.2" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <!-- Scout 4-Wheel Velocity Servos (rad/s) -->
    <velocity name="scout_act_fl" joint="scout_j_fl" kv="1.0" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <velocity name="scout_act_fr" joint="scout_j_fr" kv="1.0" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <velocity name="scout_act_rl" joint="scout_j_rl" kv="1.0" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <velocity name="scout_act_rr" joint="scout_j_rr" kv="1.0" ctrllimited="true" ctrlrange="-14.0 14.0"/>
"""
    sensors_xml = """
    <rangefinder name="planter_range_c" site="planter_range_c"/>
    <rangefinder name="planter_range_l" site="planter_range_l"/>
    <rangefinder name="planter_range_r" site="planter_range_r"/>
    <rangefinder name="scout_range_c" site="scout_range_c"/>
    <rangefinder name="scout_range_l" site="scout_range_l"/>
    <rangefinder name="scout_range_r" site="scout_range_r"/>
    <framepos name="planter_pos" objtype="site" objname="planter_odom_site"/>
    <framequat name="planter_quat" objtype="site" objname="planter_odom_site"/>
    <framepos name="scout_pos" objtype="site" objname="scout_odom_site"/>
    <framequat name="scout_quat" objtype="site" objname="scout_odom_site"/>
"""
    return bodies_xml, actuators_xml, sensors_xml
