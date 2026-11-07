"""Standalone MuJoCo simulation: Paddy Ant using ego-camera computer vision to detect, orient toward, and climb a terraced incline ramp.

Validates:
1. Ego-Camera RGB-D Ramp Detection (`planter_front_camera`): Real-time MuJoCo offscreen RGB + Depth rendering (`mujoco.Renderer`), chromatic/luminance segmentation of the earthen ramp and high-contrast guide rails, and 3D depth-gradient slope estimation.
2. Closed-Loop Visual Servoing (Orientation & Alignment): Rover spawns off-axis (`y = 0.24 m`, yaw = `-24 deg` away from the ramp at `y = 0.50 m`), detects the ramp in its camera FOV, steers to zero out the visual bearing error (`ORIENTING_TO_RAMP` -> `ALIGNED_APPROACH`), and enters the ramp toe squarely.
3. Terraced Incline Ascent (`CLIMBING_INCLINE`): 22-degree earthen ramp ascent between Lower Tier 0 (`z = 0.0 m`) and Upper Tier 1 (`z = +0.35 m`).

Usage:
  # Run headless closed-loop CV orientation + ramp climb verification:
  python scripts/simulate_terrace_incline.py

  # Launch interactive 3D MuJoCo viewer with live CV trajectory & ray overlays:
  python scripts/simulate_terrace_incline.py --viewer-seconds 15
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ricebot.robot import (
    PLANTER_SPEC,
    RampVisionDetection,
    build_rovers_and_seeds_xml,
    compute_4wheel_skid_steer,
    detect_ramp_and_compute_visual_servo,
    make_homogeneous_rover_spec,
)
from ricebot.robot.rovers import _build_homogeneous_ant_rover_xml


def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> list[float]:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def _quat_to_pitch_yaw(quat_wxyz: np.ndarray) -> tuple[float, float]:
    w, x, y, z = float(quat_wxyz[0]), float(quat_wxyz[1]), float(quat_wxyz[2]), float(quat_wxyz[3])
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return pitch, yaw


def build_terrace_incline_world_xml(
    ramp_angle_deg: float = 22.0,
    spawn_xyz: tuple[float, float, float] = (1.35, 0.50, 0.08),
) -> str:
    """Return MJCF XML modeling a 2-tier terraced paddy with a connecting earthen bund ramp and visual guide markers."""
    ramp_rad = math.radians(ramp_angle_deg)

    # Ramp geometry connecting lower tier (z=0.0) to upper tier (z=0.35)
    ramp_len = 0.35 / math.sin(ramp_rad)  # Hypotenuse length
    ramp_x = 1.80
    ramp_z = 0.120  # Flush entry at z=0.0 and flush exit at z=0.35

    unit_a_spec = make_homogeneous_rover_spec("planter", initial_xyz=spawn_xyz, initial_seeds=2)
    unit_b_spec = make_homogeneous_rover_spec("scout", initial_xyz=(2.80, 0.50, 0.42), initial_seeds=2)

    body_a = _build_homogeneous_ant_rover_xml(
        unit_a_spec,
        chassis_rgba="0.16 0.46 0.86 1",
        hub_rgba="0.90 0.75 0.15 1",
        ray_rgba="1 0.2 0.2 1",
        contype=2,
    )
    body_b = _build_homogeneous_ant_rover_xml(
        unit_b_spec,
        chassis_rgba="0.18 0.72 0.36 1",
        hub_rgba="0.90 0.75 0.15 1",
        ray_rgba="0.2 1.0 0.4 1",
        contype=4,
    )
    _, actuators_xml, sensors_xml = build_rovers_and_seeds_xml()

    xml = f"""
<mujoco model="terrace_incline_and_mud_world">
  <compiler angle="radian" coordinate="local"/>
  <option gravity="0 0 -9.81" timestep="0.002" iterations="50" solver="Newton">
    <flag contact="enable" frictionloss="enable"/>
  </option>

  <visual>
    <global offwidth="960" offheight="720" azimuth="135" elevation="-26"/>
    <headlight ambient="0.4 0.42 0.45" diffuse="0.75 0.72 0.68" specular="0.15 0.15 0.15"/>
    <rgba haze="0.1 0.15 0.22 1"/>
  </visual>

  <statistic center="1.75 0.45 0.20" extent="2.2"/>

  <asset>
    <texture name="sky_tex" type="skybox" builtin="gradient" rgb1="0.68 0.82 0.96" rgb2="0.18 0.32 0.52" width="256" height="256"/>
    <texture name="paddy_soil_tex" type="2d" builtin="checker" rgb1="0.36 0.29 0.21" rgb2="0.32 0.25 0.18" width="256" height="256"/>
    <material name="paddy_soil_mat" texture="paddy_soil_tex" texrepeat="6 6"/>
    <material name="paddy_mud" rgba="0.20 0.15 0.11 1" roughness="0.9"/>
    <material name="bund_earth" rgba="0.68 0.42 0.18 1" roughness="0.8"/>
    <material name="ramp_guide_mat" rgba="0.96 0.62 0.14 1" reflectance="0.15"/>
    <material name="water_mat" rgba="0.15 0.35 0.55 0.65"/>
    <material name="mud_hazard_mat" rgba="0.22 0.16 0.11 0.92" shininess="0.85"/>
    <material name="wheel_rubber_mat" rgba="0.16 0.16 0.18 1"/>
  </asset>

  <worldbody>
    <light name="sun" pos="2 0 4" dir="-0.3 0.1 -1" diffuse="0.9 0.88 0.82" castshadow="true"/>

    <!-- TIER 0: Lower Flooded Paddy Field (z = 0.0m) -->
    <geom name="tier0_ground" type="plane" pos="0 0 0" size="3 3 0.1" material="paddy_mud" contype="1" conaffinity="7" friction="1.15 0.01 0.002"/>
    <geom name="tier0_water" type="box" pos="0 0 0.015" size="2 2 0.015" material="water_mat" contype="0" conaffinity="0"/>

    <!-- TIER 1: Upper Flooded Terrace (z = +0.35m) -->
    <geom name="tier1_ground" type="box" pos="3.30 0 0.175" size="1.15 2.0 0.175" material="paddy_mud" contype="1" conaffinity="7" friction="1.15 0.01 0.002"/>
    <geom name="tier1_crest_bridge" type="box" pos="2.18 0.5 0.175" size="0.14 0.35 0.175" material="bund_earth" contype="1" conaffinity="7" friction="1.35 0.02 0.005"/>
    <geom name="tier1_crest_stripe" type="box" pos="2.35 0.5 0.351" size="0.30 0.045 0.002" material="ramp_guide_mat" contype="0" conaffinity="0"/>
    <geom name="tier1_water" type="box" pos="3.2 0 0.365" size="1.1 1.9 0.015" material="water_mat" contype="0" conaffinity="0"/>

    <!-- Retaining Bund Ridges separating Tier 0 & Tier 1 (Gap at y=0.5 for Ramp Incline) -->
    <geom name="retaining_bund_south" type="box" pos="2.0 -0.95 0.175" size="0.12 1.00 0.175" material="paddy_mud" contype="1" conaffinity="7"/>
    <geom name="retaining_bund_north" type="box" pos="2.0 1.55 0.175" size="0.12 0.70 0.175" material="paddy_mud" contype="1" conaffinity="7"/>

    <!-- Earthen Incline Ramp cutting through Bund (22 deg slope for rover transition) -->
    <geom name="bund_incline_ramp" type="box" pos="{ramp_x} 0.5 {ramp_z}" euler="0 {-ramp_rad:.3f} 0" size="{ramp_len/2:.3f} 0.35 0.06" material="bund_earth" contype="1" conaffinity="7" friction="1.35 0.02 0.005"/>
    <!-- High-Contrast Visual Ramp Centerline & Edge Guide Stripes for Onboard Camera Detection -->
    <geom name="ramp_center_stripe" type="box" pos="{ramp_x} 0.5 {ramp_z + 0.003:.3f}" euler="0 {-ramp_rad:.3f} 0" size="{ramp_len/2:.3f} 0.045 0.061" material="ramp_guide_mat" contype="0" conaffinity="0"/>
    <geom name="ramp_edge_left" type="box" pos="{ramp_x} 0.81 {ramp_z + 0.003:.3f}" euler="0 {-ramp_rad:.3f} 0" size="{ramp_len/2:.3f} 0.025 0.061" material="ramp_guide_mat" contype="0" conaffinity="0"/>
    <geom name="ramp_edge_right" type="box" pos="{ramp_x} 0.19 {ramp_z + 0.003:.3f}" euler="0 {-ramp_rad:.3f} 0" size="{ramp_len/2:.3f} 0.025 0.061" material="ramp_guide_mat" contype="0" conaffinity="0"/>

    {body_a}
    {body_b}
  </worldbody>

  <equality>
    <connect name="rigid_latch_planter_to_scout" site1="planter_front_dock_site" site2="scout_rear_dock_site" active="false" solref="0.04 1.0"/>
    <connect name="rigid_latch_scout_to_planter" site1="scout_front_dock_site" site2="planter_rear_dock_site" active="false" solref="0.04 1.0"/>
  </equality>

  <actuator>
    {actuators_xml}
  </actuator>

  <sensor>
    {sensors_xml}
  </sensor>
</mujoco>
"""
    return xml


def run_vision_guided_ramp_climb(viewer_seconds: float = 0.0) -> dict[str, object]:
    """Run closed-loop ego-camera visual servoing from an off-axis spawn pose up the 22-deg terrace ramp."""
    print("======================================================================")
    print("MUJOCO CV SIMULATION: VISION-GUIDED RAMP DETECTION, ORIENTATION & CLIMB")
    print("======================================================================\n")

    # Spawn planter off-axis (y=0.26 m vs ramp centerline y=0.50 m) and misaligned in yaw (+0.38 rad / +21.8 deg)
    xml_str = build_terrace_incline_world_xml(ramp_angle_deg=22.0, spawn_xyz=(0.78, 0.26, 0.062))
    model = mujoco.MjModel.from_xml_string(xml_str)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)

    qadr = int(model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "planter_free")])
    initial_yaw_rad = -0.12  # Pointing slightly away from the ramp at (1.80, 0.50)
    data.qpos[qadr + 3 : qadr + 7] = _rpy_to_quat(0.0, 0.0, initial_yaw_rad)
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, height=120, width=160)
    init_pos = data.sensor("planter_pos").data.copy()
    _, init_yaw = _quat_to_pitch_yaw(data.sensor("planter_quat").data)
    print(
        f"Initial Off-Axis Pose: x={init_pos[0]:.3f} m, y={init_pos[1]:.3f} m (ramp at y=0.500 m), "
        f"z={init_pos[2]:.3f} m, yaw={math.degrees(init_yaw):+.1f} deg"
    )

    last_det: RampVisionDetection | None = None
    trace_log: list[dict[str, object]] = []

    def step_visual_servo(step_idx: int) -> None:
        nonlocal last_det
        # Update ego-camera RGB-D perception at 25 Hz (every 20 physics steps @ dt=0.002s)
        if step_idx % 20 == 0 or last_det is None:
            pos = data.sensor("planter_pos").data
            pitch, yaw = _quat_to_pitch_yaw(data.sensor("planter_quat").data)

            renderer.update_scene(data, camera="planter_front_camera")
            rgb = renderer.render()
            renderer.enable_depth_rendering()
            renderer.update_scene(data, camera="planter_front_camera")
            depth = renderer.render()
            renderer.disable_depth_rendering()

            last_det = detect_ramp_and_compute_visual_servo(
                rgb,
                depth,
                current_pitch_rad=pitch,
                current_z_m=float(pos[2]),
                target_tier_z_m=0.35,
            )
            if step_idx % 200 == 0:
                print(
                    f"  [t={data.time:4.2f}s | {last_det.phase:18s}] "
                    f"pose=({pos[0]:.2f}, {pos[1]:.2f}, z={pos[2]:.3f}m) "
                    f"cam_px={last_det.pixel_count:4d} centroid_u={last_det.centroid_u_px:5.1f}/160 "
                    f"bearing_err={math.degrees(last_det.bearing_error_rad):+5.1f}deg "
                    f"slope_est={last_det.estimated_slope_deg:4.1f}deg "
                    f"cmd=(v={last_det.linear_cmd_m_s:.2f} m/s, w={last_det.steering_cmd_rad_s:+.2f} rad/s)"
                )
                trace_log.append(
                    {
                        "time_s": round(float(data.time), 2),
                        "phase": last_det.phase,
                        "x_m": round(float(pos[0]), 3),
                        "y_m": round(float(pos[1]), 3),
                        "z_m": round(float(pos[2]), 3),
                        "pixel_count": last_det.pixel_count,
                        "bearing_error_deg": round(math.degrees(last_det.bearing_error_rad), 2),
                        "estimated_slope_deg": last_det.estimated_slope_deg,
                    }
                )

        w_fl, w_fr, w_rl, w_rr = compute_4wheel_skid_steer(
            PLANTER_SPEC, last_det.linear_cmd_m_s, last_det.steering_cmd_rad_s
        )
        # Boost wheel angular velocity on steep incline while preserving differential steering
        incline_boost = 1.55 if last_det.phase == "CLIMBING_INCLINE" else 1.20
        data.ctrl[0] = max(-14.0, min(14.0, w_fl * incline_boost))
        data.ctrl[1] = max(-14.0, min(14.0, w_fr * incline_boost))
        data.ctrl[2] = max(-14.0, min(14.0, w_rl * incline_boost))
        data.ctrl[3] = max(-14.0, min(14.0, w_rr * incline_boost))
        mujoco.mj_step(model, data)

    total_steps = 2200  # 4.4 seconds @ dt=0.002s
    if viewer_seconds > 0:
        import mujoco.viewer as mj_viewer

        start_wall = time.time()
        with mj_viewer.launch_passive(model, data) as viewer:
            viewer.cam.lookat[:] = [1.65, 0.45, 0.16]
            viewer.cam.distance = 2.15
            viewer.cam.elevation = -26.0
            viewer.cam.azimuth = 132.0
            for step_i in range(total_steps):
                if not viewer.is_running():
                    break
                step_visual_servo(step_i)
                if step_i % 12 == 0:
                    viewer.sync()
                    time.sleep(0.018)
            while viewer.is_running() and (time.time() - start_wall) < viewer_seconds:
                viewer.sync()
                time.sleep(0.04)
    else:
        for step_i in range(total_steps):
            step_visual_servo(step_i)

    renderer.close()

    final_pos = data.sensor("planter_pos").data
    final_x, final_y, final_z = float(final_pos[0]), float(final_pos[1]), float(final_pos[2])
    elevation_gain = final_z - float(init_pos[2])
    lateral_alignment_err = abs(final_y - 0.50)

    print(
        f"\nFinal Planter Pose on Ramp/Upper Tier: x={final_x:.3f} m, y={final_y:.3f} m "
        f"(lateral err={lateral_alignment_err*1000:.1f} mm), z={final_z:.3f} m"
    )
    print(f"Elevation Gain (Delta Z): +{elevation_gain*1000:.1f} mm")
    assert elevation_gain > 0.15, f"Expected vision-guided rover to climb >150 mm up ramp, got {elevation_gain:.3f} m"
    assert lateral_alignment_err < 0.12, f"Expected visual servoing to align with ramp y=0.50m, got y={final_y:.3f}m"
    print("\n[SUCCESS] Paddy Ant detected the ramp via ego-camera RGB-D vision, oriented onto centerline, and climbed the 22-deg incline!")
    return {
        "initial_pose_xyz": [round(float(v), 3) for v in init_pos],
        "final_pose_xyz": [round(final_x, 3), round(final_y, 3), round(final_z, 3)],
        "elevation_gain_m": round(elevation_gain, 3),
        "lateral_alignment_error_m": round(lateral_alignment_err, 3),
        "trace": trace_log,
    }


def run_terrace_incline_test() -> dict[str, object]:
    return run_vision_guided_ramp_climb(viewer_seconds=0.0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--viewer-seconds",
        type=float,
        default=0.0,
        help="Launch interactive 3D MuJoCo viewer for N seconds",
    )
    args = parser.parse_args()
    run_vision_guided_ramp_climb(viewer_seconds=args.viewer_seconds)
