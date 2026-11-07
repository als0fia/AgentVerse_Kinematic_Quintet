"""MuJoCo 3D paddy-terrace & wheeled-rover physics backend implementing SimulationPort (rice.v1).

Purpose:
- Implements a genuine 3D Southern China / Antenna rice terrace in MuJoCo 4.0.0:
  - Raised perimeter soil bunds / field ridges (田埂), irrigation canal & drainage sluice,
    and a 6-patch rice plot with:
    - Living green rice seedling stands (protected no-go zones with 3D plant stalks)
    - Deep "Lannitian" (烂泥田) waterlogged mud hazard pans (high moisture, low firmness)
    - Viable missing-stand soil patches (firm drained loam ready for reseeding)
  - Two 4-wheel differential-drive rovers (`planter` and `scout`) with 6-DOF `freejoint`
    chassis, 4 revolute wheel joints (`_fl`, `_fr`, `_rl`, `_rr`) rolling via ground
    friction on the paddy soil, forward raycast rangefinder arrays, and soil probe booms.
  - Physical seed hopper on `planter` carrying 6-DOF rigid-body seed spheres (`seed_proxy_0`,
    `seed_proxy_1`) that drop through the bottom chute under gravity and register physical
    ground contact (`mjData.contact`) against the paddy soil.
  - Real-time 3D visual overlays (`mjvScene` user geoms) in the interactive MuJoCo viewer
    showing live rangefinder laser beams, soil probe rings, shared belief map status
    (unknown vs rejected mud hazard vs viable stand gap vs planted), and active target lines.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import atan2, hypot
import xml.etree.ElementTree as ET

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

DEFAULT_CAPABILITIES: tuple[RobotCapability, ...] = (
    RobotCapability("planter", can_sample=True, can_plant=True),
    RobotCapability("scout", can_sample=True, can_plant=False),
)


def _build_seedling_clumps_xml() -> str:
    """Generate 3D green rice seedling stalks inside the protected crop patches and terrace border."""
    lines: list[str] = []
    # Protected crop patches at (0.85, 0.45) and (2.35, 0.45)
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


ANTENNA_MJCF = f"""
<mujoco model="antenna_rice_v1_terrace">
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
    <!-- patch_a (0.85, -0.50): Deep Lannitian Mud Hazard Pan (glossy dark waterlogged mud + shallow puddle) -->
    <geom name="bed_patch_a" type="cylinder" pos="0.85 -0.50 0.003" size="0.25 0.003" material="mud_hazard_mat" contype="0" conaffinity="0"/>
    <geom name="puddle_patch_a" type="cylinder" pos="0.85 -0.50 0.005" size="0.18 0.002" material="canal_water_mat" contype="0" conaffinity="0"/>

    <!-- patch_b (1.60, -0.50): Viable Missing-Stand Loam Patch #1 (prepared soil bed + target ring) -->
    <geom name="bed_patch_b" type="cylinder" pos="1.60 -0.50 0.003" size="0.25 0.003" material="viable_loam_mat" contype="0" conaffinity="0"/>
    <site name="ring_patch_b" type="cylinder" pos="1.60 -0.50 0.006" size="0.16 0.002" rgba="0.85 0.72 0.25 0.55"/>

    <!-- patch_c (2.35, -0.50): Viable Missing-Stand Loam Patch #2 -->
    <geom name="bed_patch_c" type="cylinder" pos="2.35 -0.50 0.003" size="0.25 0.003" material="viable_loam_mat" contype="0" conaffinity="0"/>
    <site name="ring_patch_c" type="cylinder" pos="2.35 -0.50 0.006" size="0.16 0.002" rgba="0.85 0.72 0.25 0.55"/>

    <!-- patch_d (1.60, 0.45): Waterlogged Lannitian Mud Hazard Pan #2 -->
    <geom name="bed_patch_d" type="cylinder" pos="1.60 0.45 0.003" size="0.25 0.003" material="mud_hazard_mat" contype="0" conaffinity="0"/>
    <geom name="puddle_patch_d" type="cylinder" pos="1.60 0.45 0.005" size="0.17 0.002" material="canal_water_mat" contype="0" conaffinity="0"/>

    <!-- Protected Crop Patches (crop_nw at 0.85, 0.45 and crop_ne at 2.35, 0.45) with 3D Rice Seedlings -->
    <geom name="bed_crop_nw" type="cylinder" pos="0.85 0.45 0.004" size="0.27 0.004" material="crop_bed_mat" contype="0" conaffinity="0"/>
    <geom name="bed_crop_ne" type="cylinder" pos="2.35 0.45 0.004" size="0.27 0.004" material="crop_bed_mat" contype="0" conaffinity="0"/>
    {_build_seedling_clumps_xml()}

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

    <!-- Mid-episode obstacle (fallen irrigation crate/boulder parked below ground until surprise injection) -->
    <body name="mid_run_obstacle" mocap="true" pos="0.95 -0.05 -5.0">
      <geom name="obstacle_geom" type="box" size="0.11 0.14 0.14" contype="5" conaffinity="7" rgba="0.88 0.24 0.16 1"/>
    </body>
  </worldbody>

  <actuator>
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
  </actuator>

  <sensor>
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
  </sensor>
</mujoco>
"""


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
        self.sites = {s.site_id: s for s in sites}
        self.probe_radius_m = probe_radius_m
        self.substeps_per_tick = substeps_per_tick
        self.inject_obstacle_at_s = inject_obstacle_at_s
        self.obstacle_xy = obstacle_xy

        self.model = mujoco.MjModel.from_xml_string(ANTENNA_MJCF)
        self.data = mujoco.MjData(self.model)

        self._ground_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "field_ground")
        self._seed_geom_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"seed_{i}_geom")
            for i in range(2)
        ]
        self._planter_qpos_adr = int(
            self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "planter_free")]
        )
        self._scout_qpos_adr = int(
            self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "scout_free")]
        )
        self._seed_qpos_adrs = [
            int(self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"seed_{i}_free")])
            for i in range(2)
        ]
        self._seed_qvel_adrs = [
            int(self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"seed_{i}_free")])
            for i in range(2)
        ]

        # Hidden synthetic soil & stand-gap truth (NEVER exposed directly to planner)
        # - patch_a & patch_d: "Lannitian" waterlogged mud hazards (moisture > 0.80, firmness < 0.20)
        # - patch_b & patch_c: Viable missing-stand rice patches ready for planting
        self._hidden_site_truth: dict[str, dict[SensorKind, float]] = {
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
        # Planter starts at (0.10, -0.05, 0.058), Scout starts at (0.10, -0.55, 0.054)
        self.data.qpos[self._planter_qpos_adr : self._planter_qpos_adr + 7] = [
            0.10, -0.05, 0.058, 1.0, 0.0, 0.0, 0.0
        ]
        self.data.qpos[self._scout_qpos_adr : self._scout_qpos_adr + 7] = [
            0.10, -0.55, 0.054, 1.0, 0.0, 0.0, 0.0
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
        for idx in range(2):
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

    def _apply_4wheel_skid_steer(self, ctrl_offset: int, wheel_radius: float, half_track: float, lin: float, ang: float) -> None:
        """Convert bounded (linear_m_s, angular_rad_s) into 4-wheel angular velocity targets (rad/s)."""
        # Differential skid-steer amplification factor (~1.8x for slip on 4-wheel skid turn)
        v_left = lin - ang * half_track * 1.85
        v_right = lin + ang * half_track * 1.85
        w_left = float(np.clip(v_left / wheel_radius, -14.0, 14.0))
        w_right = float(np.clip(v_right / wheel_radius, -14.0, 14.0))
        self.data.ctrl[ctrl_offset + 0] = w_left   # FL
        self.data.ctrl[ctrl_offset + 1] = w_right  # FR
        self.data.ctrl[ctrl_offset + 2] = w_left   # RL
        self.data.ctrl[ctrl_offset + 3] = w_right  # RR

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

        # Process commands for planter (ctrl 0..3) and scout (ctrl 4..7)
        self.data.ctrl[:] = 0.0
        for robot_id, ctrl_offset, wheel_r, half_track in (
            ("planter", 0, 0.058, 0.118),
            ("scout", 4, 0.054, 0.105),
        ):
            cmd = cmd_by_robot.get(robot_id)
            if cmd is None or not (cmd.issued_at_s <= now_s < cmd.expires_at_s):
                continue

            if cmd.release_seed:
                if (
                    robot_id == "planter"
                    and self._seeds_remaining["planter"] > 0
                    and cmd.target_site_id in self.sites
                    and cmd.linear_m_s == 0.0
                    and cmd.angular_rad_s == 0.0
                ):
                    seed_idx = 2 - self._seeds_remaining["planter"]
                    self._seeds_remaining["planter"] -= 1
                    planter_xyz = self.data.sensor("planter_pos").data
                    px, py = float(planter_xyz[0]), float(planter_xyz[1])
                    qadr = self._seed_qpos_adrs[seed_idx]
                    vadr = self._seed_qvel_adrs[seed_idx]
                    # Drop from bottom hopper chute at z=0.032m so gravity lands it on the soil bed
                    self.data.qpos[qadr : qadr + 3] = [px, py, 0.032]
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
                lin = float(np.clip(cmd.linear_m_s, -0.45, 0.45))
                ang = float(np.clip(cmd.angular_rad_s, -1.40, 1.40))
                self._apply_4wheel_skid_steer(ctrl_offset, wheel_r, half_track, lin, ang)

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
                samples=self._local_samples_for_robot("planter", planter_pose, sim_t),
                link_ok=True,
            ),
            RobotObservation(
                robot_id="scout",
                pose=scout_pose,
                time_s=sim_t,
                front_range_m=self._read_min_front_range_m("scout"),
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
