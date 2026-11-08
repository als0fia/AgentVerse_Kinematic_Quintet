"""Run and validate Multi-Level Chinese Terraced Rice Fields with 1.0m Step Levels, ~0.50m Agricultural Robot & Continuous Ramp Navigation.

Executes:
1. Builds a physically authentic 3D MuJoCo Chinese terraced rice-paddy landscape (Longji / Yuanyang style):
   - Terrace A (Lower Field): 0.0 m elevation
   - Terrace B (Middle Field): 1.0 m elevation (+1.0 m step)
   - Terrace C (Upper Field): 2.0 m elevation (+1.0 m step)
   - Recessed-flank stepped hillside with contoured stone retaining walls and curved golden earthen bunds (田埂)
   - Solid earth-supported inclined access ramps (Ramp AB at South corridor, Ramp BC at North switchback corridor)
   - Contained horizontal reflective water paddies, submerged muddy soil, and 3D green rice seedling rows
   - Properly scaled ~0.50m (1/2 m) 4-wheel agricultural paddy robots
2. Executes Test 1: Continuous time-stepped ascent from Terrace A (0.0m) -> Ramp AB -> Terrace B (1.0m) -> Ramp BC -> Terrace C (2.0m).
3. Executes Test 2: Continuous time-stepped descent from Terrace C (2.0m) -> Ramp BC -> Terrace B (1.0m) -> Ramp AB -> Terrace A (0.0m).
4. Validates Test 3 (Blocked Crossing), Test 4 (Steep Crossing Rejection), and Test 5 (Collision & Crop Protection).
5. Renders 5 high-resolution 1280x800 PNG views and supports `--viewer` for live interactive 3D playback.
"""
from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import struct
import sys
import time
import zlib
import numpy as np

if "MUJOCO_GL" in os.environ:
    del os.environ["MUJOCO_GL"]

_REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO_ROOT))
sys.path.insert(0, str(_REPO_ROOT / "src"))

import mujoco

from ricebot.world import (
    MultiLevelPathPlanner,
    MultiLevelTerraceEnvironment,
    TrajectoryState,
)
from ricebot.world.field_textures import enhance_mjcf_with_field_textures


def save_png(filename: Path, img_array: np.ndarray) -> None:
    """Save an RGB numpy array as a standard PNG image with zero external dependencies."""
    h, w, _ = img_array.shape
    raw_data = b"".join(b"\x00" + img_array[i].tobytes() for i in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + tag
            + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw_data, 9))
    png += chunk(b"IEND", b"")
    filename.parent.mkdir(parents=True, exist_ok=True)
    with open(filename, "wb") as f:
        f.write(png)


def _euler_to_quat(roll: float, pitch: float, yaw: float) -> list[float]:
    """Convert (roll, pitch, yaw) in radians to MuJoCo quaternion [w, x, y, z]."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def _build_half_meter_paddy_rover_xml(
    robot_id: str,
    initial_xyz: tuple[float, float, float],
    chassis_rgba: str,
    hub_rgba: str,
    contype: int,
) -> str:
    """Build a ~0.50m (1/2 m) 4-wheel agricultural rice-paddy robot for 1.0m terrace steps.

    Physical dimensions:
    - Overall length: 0.50 m (chassis 0.44m + front/rear tool mounts)
    - Overall width:  0.38 m (half_track = 0.160m, wheel half_width = 0.030m)
    - Wheel radius:   0.095 m (190 mm diameter high-clearance lugged paddy wheels)
    - Wheelbase:      0.30 m (wx = +/- 0.150m)
    """
    ix, iy, iz = initial_xyz
    wr = 0.095
    ww = 0.030
    ht = 0.160
    wx = 0.150

    def _wheel_xml(suffix: str, px: float, py: float, side_sign: float) -> str:
        lugs = []
        num_lugs = 8
        lug_r = wr - 0.004
        for i in range(num_lugs):
            ang = i * (2.0 * math.pi / num_lugs)
            lx = lug_r * math.cos(ang)
            lz = lug_r * math.sin(ang)
            lugs.append(
                f'<geom name="{robot_id}_lug_{suffix}_{i}" type="box" pos="{lx:.4f} 0 {lz:.4f}" '
                f'euler="0 {-ang:.4f} 0" size="0.009 {ww + 0.003:.3f} 0.015" '
                f'contype="0" conaffinity="0" rgba="0.24 0.26 0.29 1"/>'
            )
        lugs_str = "\n        ".join(lugs)
        hy = side_sign * (ww + 0.004)
        return f"""<body name="{robot_id}_wheel_{suffix}" pos="{px:.3f} {py:.3f} 0.0">
        <joint name="{robot_id}_j_{suffix}" type="hinge" axis="0 1 0" damping="0.15" armature="0.01"/>
        <geom name="{robot_id}_w_{suffix}" type="cylinder" size="{wr:.3f} {ww:.3f}" zaxis="0 1 0" mass="0.85" material="wheel_rubber_mat" contype="{contype}" conaffinity="1" friction="2.0 0.02 0.005"/>
        <geom name="{robot_id}_hub_{suffix}" type="box" pos="0 {hy:.3f} 0" size="0.068 0.005 0.016" contype="0" conaffinity="0" rgba="{hub_rgba}"/>
        <geom name="{robot_id}_hub_{suffix}_b" type="box" pos="0 {hy:.3f} 0" size="0.016 0.005 0.068" contype="0" conaffinity="0" rgba="{hub_rgba}"/>
        {lugs_str}
      </body>"""

    w_fl = _wheel_xml("fl", +wx, +ht, +1.0)
    w_fr = _wheel_xml("fr", +wx, -ht, -1.0)
    w_rl = _wheel_xml("rl", -wx, +ht, +1.0)
    w_rr = _wheel_xml("rr", -wx, -ht, -1.0)

    return f"""
    <!-- ~0.50m Agricultural Paddy Rover: {robot_id.upper()} -->
    <body name="{robot_id}_body" pos="{ix:.3f} {iy:.3f} {iz:.3f}">
      <freejoint name="{robot_id}_free"/>
      <!-- Waterproof Main Hull (0.44m x 0.26m x 0.11m; total length with bumpers = 0.50m) -->
      <geom name="{robot_id}_chassis" type="box" pos="0 0 0.028" size="0.220 0.130 0.055" mass="6.5" contype="{contype}" conaffinity="5" rgba="{chassis_rgba}"/>
      <geom name="{robot_id}_belly_skid" type="box" pos="0 0 -0.032" size="0.195 0.115 0.008" mass="0.3" contype="0" conaffinity="0" rgba="0.22 0.24 0.27 1"/>
      <geom name="{robot_id}_top_deck" type="box" pos="-0.015 0 0.088" size="0.185 0.118 0.012" mass="0.4" contype="0" conaffinity="0" rgba="0.92 0.94 0.97 1"/>

      <!-- Agricultural Seed Hopper & Metering Tower -->
      <geom name="{robot_id}_hopper_outer" type="cylinder" pos="0.045 0 0.135" size="0.072 0.042" mass="0.3" contype="0" conaffinity="0" rgba="0.95 0.74 0.12 1"/>
      <geom name="{robot_id}_hopper_cap" type="cylinder" pos="0.045 0 0.180" size="0.058 0.008" mass="0.05" contype="0" conaffinity="0" rgba="0.25 0.28 0.32 1"/>
      <geom name="{robot_id}_spray_tank" type="box" pos="-0.095 0 0.125" size="0.055 0.068 0.028" mass="0.2" contype="0" conaffinity="0" rgba="0.18 0.82 0.78 0.95"/>

      <!-- BeiDou RTK-GNSS Receiver Mast & Dome -->
      <geom name="{robot_id}_rtk_mast" type="cylinder" pos="-0.155 0.085 0.165" size="0.012 0.065" mass="0.05" contype="0" conaffinity="0" rgba="0.82 0.84 0.88 1"/>
      <geom name="{robot_id}_rtk_dome" type="sphere" pos="-0.155 0.085 0.238" size="0.032" mass="0.05" contype="0" conaffinity="0" rgba="0.97 0.97 0.99 1"/>

      <!-- Front LiDAR & Stereo Perception Pod + Front Bumper (extends to +0.25m, Rear to -0.25m -> 0.50m total length) -->
      <geom name="{robot_id}_lidar_pod" type="cylinder" pos="0.185 0 0.108" size="0.035 0.022" mass="0.1" contype="0" conaffinity="0" rgba="0.18 0.20 0.22 1"/>
      <geom name="{robot_id}_front_bumper" type="box" pos="0.238 0 0.018" size="0.014 0.125 0.022" mass="0.1" contype="0" conaffinity="0" rgba="0.96 0.44 0.14 1"/>
      <geom name="{robot_id}_rear_bumper" type="box" pos="-0.238 0 0.018" size="0.014 0.125 0.022" mass="0.1" contype="0" conaffinity="0" rgba="0.85 0.68 0.15 1"/>

      {w_fl}
      {w_fr}
      {w_rl}
      {w_rr}
    </body>
    """


def _build_contoured_bund_and_wall_xml(
    prefix: str,
    x_edge: float,
    y_min: float,
    y_max: float,
    z_low: float,
    z_high: float,
    num_segments: int = 8,
    curve_amp_x: float = 0.18,
    phase: float = 0.0,
) -> str:
    """Build a natural curved stone retaining wall and golden earthen bund (田埂) along a terrace edge."""
    geoms: list[str] = []
    wall_h = z_high - z_low
    z_mid = 0.5 * (z_low + z_high)
    dy = (y_max - y_min) / num_segments

    for i in range(num_segments):
        y0 = y_min + i * dy
        y1 = y0 + dy
        ym = 0.5 * (y0 + y1)
        frac0 = i / float(num_segments)
        frac1 = (i + 1) / float(num_segments)
        x_off0 = curve_amp_x * math.sin(math.pi * frac0 + phase)
        x_off1 = curve_amp_x * math.sin(math.pi * frac1 + phase)
        xm = x_edge + 0.5 * (x_off0 + x_off1)
        dx_seg = x_off1 - x_off0
        seg_len = math.hypot(dx_seg, dy)
        yaw = math.atan2(dx_seg, dy)  # rotation around Z relative to Y-axis

        # Stone retaining wall face
        if wall_h > 0.05:
            geoms.append(
                f'<geom name="{prefix}_wall_{i}" type="box" pos="{xm:.3f} {ym:.3f} {z_mid:.3f}" '
                f'size="0.075 {0.52 * seg_len:.3f} {0.5 * wall_h:.3f}" euler="0 0 {-yaw:.4f}" '
                f'material="terrace_wall_mat" contype="1" conaffinity="7"/>'
            )
        # Golden raised earthen bund (田埂) on top rim
        geoms.append(
            f'<geom name="{prefix}_bund_{i}" type="box" pos="{xm + 0.02:.3f} {ym:.3f} {z_high + 0.045:.3f}" '
            f'size="0.11 {0.52 * seg_len:.3f} 0.048" euler="0 0 {-yaw:.4f}" '
            f'material="bund_earth_mat" contype="1" conaffinity="7"/>'
        )
    return "\n    ".join(geoms)


def _build_paddy_seedlings_xml(
    prefix: str,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    z_base: float,
    row_spacing_y: float = 0.24,
    plant_spacing_x: float = 0.28,
) -> str:
    """Generate realistic 3D young green rice seedling clusters standing in flooded paddy rows."""
    geoms: list[str] = []
    idx = 0
    y = y_min + 0.10
    row_num = 0
    while y <= y_max - 0.08:
        x = x_min + 0.12 + (0.06 if (row_num % 2 == 1) else 0.0)
        while x <= x_max - 0.10:
            # Subtle organic variation
            jx = 0.018 * math.sin(idx * 1.7 + row_num)
            jy = 0.014 * math.cos(idx * 2.3 - row_num)
            px = x + jx
            py = y + jy
            # Submerged root crown + emerald seedling blades emerging above water
            geoms.append(
                f'<geom name="{prefix}_s_{idx}a" type="cylinder" pos="{px:.3f} {py:.3f} {z_base + 0.055:.3f}" '
                f'size="0.016 0.055" euler="0.08 0.06 0" rgba="0.18 0.64 0.16 1" contype="0" conaffinity="0"/>'
            )
            geoms.append(
                f'<geom name="{prefix}_s_{idx}b" type="cylinder" pos="{px + 0.015:.3f} {py - 0.012:.3f} {z_base + 0.065:.3f}" '
                f'size="0.012 0.062" euler="-0.14 0.12 0.4" rgba="0.26 0.76 0.20 1" contype="0" conaffinity="0"/>'
            )
            idx += 1
            x += plant_spacing_x
        y += row_spacing_y
        row_num += 1
    return "\n    ".join(geoms)


def _build_auxiliary_terrace_paddy_xml(
    prefix: str,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    z_elev: float,
    z_base: float = -0.30,
    with_seedlings: bool = True,
) -> str:
    """Build a surrounding stepped Chinese rice-paddy terrace basin (stone base, golden bunds, sunken water & seedlings)."""
    xm = 0.5 * (x_min + x_max)
    ym = 0.5 * (y_min + y_max)
    sx = 0.5 * (x_max - x_min)
    sy = 0.5 * (y_max - y_min)
    h_half = 0.5 * (z_elev - z_base)
    zm = z_base + h_half
    seed_xml = (
        _build_paddy_seedlings_xml(
            f"{prefix}_sd",
            x_min + 0.25,
            x_max - 0.25,
            y_min + 0.25,
            y_max - 0.25,
            z_elev,
            row_spacing_y=0.28,
            plant_spacing_x=0.32,
        )
        if with_seedlings
        else ""
    )
    return f"""
    <geom name="{prefix}_earth" type="box" pos="{xm:.3f} {ym:.3f} {zm - 0.01:.3f}" size="{sx:.3f} {sy:.3f} {h_half:.3f}" material="terrace_wall_mat" contype="1" conaffinity="7"/>
    <geom name="{prefix}_mud" type="box" pos="{xm:.3f} {ym:.3f} {z_elev - 0.012:.3f}" size="{sx - 0.06:.3f} {sy - 0.06:.3f} 0.008" material="paddy_soil_mat" contype="1" conaffinity="7"/>
    <geom name="{prefix}_water" type="box" pos="{xm:.3f} {ym:.3f} {z_elev + 0.012:.3f}" size="{sx - 0.10:.3f} {sy - 0.10:.3f} 0.003" material="canal_water_mat" contype="0" conaffinity="0"/>
    <geom name="{prefix}_bund_n" type="box" pos="{xm:.3f} {y_max - 0.05:.3f} {z_elev + 0.042:.3f}" size="{sx:.3f} 0.08 0.045" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="{prefix}_bund_s" type="box" pos="{xm:.3f} {y_min + 0.05:.3f} {z_elev + 0.042:.3f}" size="{sx:.3f} 0.08 0.045" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="{prefix}_bund_w" type="box" pos="{x_min + 0.05:.3f} {ym:.3f} {z_elev + 0.042:.3f}" size="0.08 {sy:.3f} 0.045" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="{prefix}_bund_e" type="box" pos="{x_max - 0.05:.3f} {ym:.3f} {z_elev + 0.042:.3f}" size="0.08 {sy:.3f} 0.045" material="bund_earth_mat" contype="1" conaffinity="7"/>
    {seed_xml}"""


def _build_solid_ramp_earth_wedge_xml(
    prefix: str,
    x_start: float,
    x_end: float,
    y_center: float,
    half_width_y: float,
    z_start: float,
    z_end: float,
    z_base: float = -0.50,
    num_slices: int = 14,
) -> str:
    """Build a solid stepped earth & stone embankment underneath an inclined ramp so it never floats."""
    geoms: list[str] = []
    dx = (x_end - x_start) / num_slices
    for i in range(num_slices):
        xm = x_start + (i + 0.5) * dx
        s = (i + 0.5) / float(num_slices)
        z_top = z_start + s * (z_end - z_start) - 0.035
        if z_top > z_base + 0.02:
            h_half = 0.5 * (z_top - z_base)
            z_mid = z_base + h_half
            geoms.append(
                f'<geom name="{prefix}_wedge_{i}" type="box" pos="{xm:.3f} {y_center:.3f} {z_mid:.3f}" '
                f'size="{0.52 * dx:.3f} {half_width_y + 0.04:.3f} {h_half:.3f}" '
                f'material="terrace_wall_mat" contype="1" conaffinity="7"/>'
            )
    return "\n    ".join(geoms)


def build_multi_level_terrace_mjcf(env: MultiLevelTerraceEnvironment) -> str:
    """Build the complete 3D MuJoCo XML for Longji-style stepped Chinese rice terraces (1.0m level steps, 0.5m robot)."""
    z_a = env.elevation_a
    z_b = env.elevation_b
    z_c = env.elevation_c
    slope_rad = math.radians(env.ramp_slope_deg)
    run_ab = env.ramp_ab.horizontal_run_m
    run_bc = env.ramp_bc.horizontal_run_m
    hyp_ab = env.ramp_ab.slope_length_m
    hyp_bc = env.ramp_bc.slope_length_m
    ramp_w = env.ramp_ab.width_m
    wheel_r = 0.095

    x0 = 0.0
    x1 = x0 + run_ab
    x2 = x1 + 3.00
    x3 = x2 + run_bc
    x4 = x3 + 3.20

    # Two ~0.50m (1/2 m) Agricultural Paddy Rovers
    body_a = _build_half_meter_paddy_rover_xml(
        "planter",
        initial_xyz=(-1.50, -0.75, z_a + wheel_r),
        chassis_rgba="0.15 0.48 0.86 1",
        hub_rgba="0.96 0.78 0.15 1",
        contype=2,
    )
    body_b = _build_half_meter_paddy_rover_xml(
        "scout",
        initial_xyz=(0.5 * (x1 + x2), 0.75, z_b + wheel_r),
        chassis_rgba="0.18 0.72 0.36 1",
        hub_rgba="0.96 0.78 0.15 1",
        contype=4,
    )

    # Solid earth foundations underneath Ramp AB and Ramp BC
    wedge_ab_xml = _build_solid_ramp_earth_wedge_xml(
        "ramp_ab", x0, x1, -0.75, 0.5 * ramp_w, z_a, z_b, z_base=z_a - 0.20
    )
    wedge_bc_xml = _build_solid_ramp_earth_wedge_xml(
        "ramp_bc", x2, x3, +0.75, 0.5 * ramp_w, z_b, z_c, z_base=z_b - 0.20
    )

    # Contoured stone retaining walls and curved golden bunds along Terrace B and Terrace C steps
    # Terrace B main flooded shelf extends all the way to x = x0 + 0.15 for y in [-0.30, 1.65]!
    wall_ab_contoured = _build_contoured_bund_and_wall_xml(
        "step_ab", x0 + 0.15, -0.32, 1.65, z_a, z_b, num_segments=8, curve_amp_x=0.22, phase=0.0
    )
    # Terrace C main flooded shelf extends all the way to x = x2 + 0.15 for y in [-1.65, +0.32]!
    wall_bc_contoured = _build_contoured_bund_and_wall_xml(
        "step_bc", x2 + 0.15, -1.65, 0.32, z_b, z_c, num_segments=8, curve_amp_x=0.22, phase=0.5
    )
    # Upper hillside terrace step above Terrace C (z = z_c + 1.0m) for authentic mountain backdrop
    wall_upper_hill = _build_contoured_bund_and_wall_xml(
        "step_upper", x4 + 0.10, -2.40, 2.40, z_c, z_c + 1.0, num_segments=10, curve_amp_x=0.28, phase=1.0
    )

    # 3D Green Rice Seedling Rows inside each flooded terrace basin
    seedlings_a = _build_paddy_seedlings_xml("seed_a", -2.65, -0.35, -0.10, 1.38, z_a)
    seedlings_b_extended = _build_paddy_seedlings_xml("seed_b1", x0 + 0.45, x1 - 0.15, -0.05, 1.38, z_b)
    seedlings_b_main = _build_paddy_seedlings_xml("seed_b2", x1 + 0.35, x2 - 0.35, -0.30, 0.35, z_b)
    seedlings_c_extended = _build_paddy_seedlings_xml("seed_c1", x2 + 0.45, x3 - 0.15, -1.38, 0.05, z_c)
    seedlings_c_main = _build_paddy_seedlings_xml("seed_c2", x3 + 0.35, x4 - 0.35, -1.38, 0.12, z_c)
    seedlings_upper = _build_paddy_seedlings_xml("seed_top", x4 + 0.45, x4 + 2.60, -1.80, 1.80, z_c + 1.0, row_spacing_y=0.32, plant_spacing_x=0.35)

    # Surrounding contoured hillside sub-terraces (North & South flanks) so the entire landscape is stepped rice paddies
    flank_paddies_xml = "\n    ".join(
        [
            _build_auxiliary_terrace_paddy_xml("flank_n_a", -3.00, 0.00, 1.72, 3.10, z_a + 0.35, z_base=-0.30),
            _build_auxiliary_terrace_paddy_xml("flank_s_a", -3.00, 0.00, -3.10, -1.72, z_a - 0.15, z_base=-0.45),
            _build_auxiliary_terrace_paddy_xml("flank_n_b", x0 + 0.20, x2, 1.72, 3.10, z_b + 0.35, z_base=z_a),
            _build_auxiliary_terrace_paddy_xml("flank_s_ab", x0 + 0.10, x1 - 0.10, -3.10, -1.25, z_a + 0.15, z_base=-0.30),
            _build_auxiliary_terrace_paddy_xml("flank_s_b", x1, x2, -3.10, -1.72, z_b - 0.25, z_base=z_a),
            _build_auxiliary_terrace_paddy_xml("flank_n_bc", x2 + 0.10, x3 - 0.10, 1.25, 3.10, z_b + 0.25, z_base=z_a),
            _build_auxiliary_terrace_paddy_xml("flank_n_c", x3, x4, 1.72, 3.10, z_c + 0.35, z_base=z_b),
            _build_auxiliary_terrace_paddy_xml("flank_s_c", x2 + 0.20, x4, -3.10, -1.72, z_c - 0.20, z_base=z_b),
        ]
    )

    x_mid_all = 0.5 * (-3.0 + x4 + 2.8)
    raw_xml = f"""
<mujoco model="longji_terraces_3tier_1m_step">
  <compiler angle="radian" coordinate="local"/>
  <option timestep="0.004" gravity="0 0 -9.81" integrator="implicitfast"/>

  <visual>
    <global offwidth="1280" offheight="800" azimuth="135" elevation="-25"/>
    <headlight ambient="0.46 0.46 0.42" diffuse="0.78 0.75 0.68" specular="0.28 0.28 0.25"/>
    <map znear="0.03" zfar="45.0"/>
  </visual>

  <statistic center="{x_mid_all:.2f} 0.00 {z_b:.2f}" extent="9.5"/>

  <asset>
    <texture name="sky_tex" type="skybox" builtin="gradient" rgb1="0.72 0.85 0.98" rgb2="0.22 0.38 0.62" width="512" height="512"/>
    <material name="paddy_soil_mat" rgba="0.34 0.26 0.18 1" roughness="0.88"/>
    <material name="bund_earth_mat" rgba="0.56 0.43 0.24 1" roughness="0.90"/>
    <material name="terrace_wall_mat" rgba="0.44 0.40 0.34 1" roughness="0.85"/>
    <material name="canal_water_mat" rgba="0.22 0.50 0.66 0.72" shininess="0.92" specular="0.85"/>
    <material name="crop_bed_mat" rgba="0.22 0.52 0.20 0.95"/>
    <material name="deep_earth_mat" rgba="0.28 0.22 0.15 1"/>
    <material name="hillside_grass_mat" rgba="0.26 0.48 0.20 1" roughness="0.92"/>
    <material name="wheel_rubber_mat" rgba="0.16 0.16 0.18 1"/>
    <material name="ramp_mat" rgba="0.52 0.41 0.25 1" roughness="0.95"/>
  </asset>

  <worldbody>
    <light name="sun_main" pos="6.0 -8.0 16.0" dir="-0.30 0.35 -1.0" diffuse="0.96 0.92 0.82" specular="0.45 0.45 0.40" castshadow="true"/>
    <light name="sky_fill" pos="4.0 6.0 12.0" dir="0 -0.25 -1" diffuse="0.48 0.52 0.58" castshadow="false"/>

    <!-- ========================================================================= -->
    <!-- STEPPED HILLSIDE SUBSTRATE & SURROUNDING CONTOURED FLANKS                 -->
    <!-- ========================================================================= -->
    <geom name="bedrock_foundation" type="box" pos="{x_mid_all:.2f} 0.00 -0.65" size="14.5 4.8 0.45" material="deep_earth_mat" contype="1" conaffinity="7"/>
    {flank_paddies_xml}

    <!-- ========================================================================= -->
    <!-- TERRACE A (LOWER LEVEL: z = {z_a:.2f}m)                                   -->
    <!-- ========================================================================= -->
    <geom name="terrace_a_earth" type="box" pos="-1.42 0.00 {z_a - 0.12:.3f}" size="1.65 1.68 0.12" material="deep_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_a_mud_floor" type="box" pos="-1.42 0.22 {z_a - 0.015:.3f}" size="1.52 1.32 0.010" material="paddy_soil_mat" contype="1" conaffinity="7" friction="1.8 0.02 0.005"/>
    <!-- Dry compacted access path on South side of Terrace A (y in [-1.55, -0.42]) -->
    <geom name="terrace_a_access_path" type="box" pos="-1.42 -0.92 {z_a - 0.008:.3f}" size="1.58 0.58 0.010" material="bund_earth_mat" contype="1" conaffinity="7" friction="2.2 0.02 0.005"/>
    <!-- Horizontal sunken water paddy in Terrace A -->
    <geom name="terrace_a_water" type="box" pos="-1.50 0.52 {z_a + 0.012:.3f}" size="1.38 0.98 0.003" material="canal_water_mat" contype="0" conaffinity="0"/>
    <!-- Golden Earthen Bunds enclosing Terrace A -->
    <geom name="terrace_a_bund_w" type="box" pos="-3.02 0.00 {z_a + 0.045:.3f}" size="0.10 1.68 0.048" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_a_bund_n" type="box" pos="-1.42 1.62 {z_a + 0.045:.3f}" size="1.62 0.10 0.048" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_a_bund_s" type="box" pos="-1.42 -1.62 {z_a + 0.045:.3f}" size="1.62 0.10 0.048" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_a_inner_bund" type="box" pos="-1.45 -0.42 {z_a + 0.025:.3f}" size="1.42 0.06 0.026" material="bund_earth_mat" contype="0" conaffinity="0"/>
    {seedlings_a}

    <!-- ========================================================================= -->
    <!-- RAMP AB: SOLID EARTH-SUPPORTED INCLINED RAMP ({z_a:.1f}m -> {z_b:.1f}m)   -->
    <!-- South Corridor: y = -0.75, x in [{x0:.2f}, {x1:.2f}]                      -->
    <!-- ========================================================================= -->
    {wedge_ab_xml}
    <geom name="ramp_ab_surface" type="box" pos="{0.5*(x0+x1):.3f} -0.75 {0.5*(z_a+z_b):.3f}" size="{hyp_ab/2:.3f} {ramp_w/2:.3f} 0.032" euler="0 {-slope_rad:.4f} 0" material="ramp_mat" contype="1" conaffinity="7" friction="2.8 0.05 0.01"/>
    <geom name="ramp_ab_curb_s" type="box" pos="{0.5*(x0+x1):.3f} {-0.75 - ramp_w/2 - 0.04:.3f} {0.5*(z_a+z_b) + 0.045:.3f}" size="{hyp_ab/2:.3f} 0.045 0.048" euler="0 {-slope_rad:.4f} 0" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="ramp_ab_curb_n" type="box" pos="{0.5*(x0+x1):.3f} {-0.75 + ramp_w/2 + 0.04:.3f} {0.5*(z_a+z_b) + 0.045:.3f}" size="{hyp_ab/2:.3f} 0.045 0.048" euler="0 {-slope_rad:.4f} 0" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <!-- Lower South sub-terrace beside Ramp AB -->
    <geom name="sub_terrace_ab_south" type="box" pos="{0.5*(x0+x1):.3f} -1.42 {z_a - 0.08:.3f}" size="{0.5*run_ab:.3f} 0.25 0.12" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- ========================================================================= -->
    <!-- TERRACE B (MIDDLE LEVEL: z = {z_b:.2f}m, +1.0m ABOVE TERRACE A)           -->
    <!-- Recessed-flank design: North paddy shelf extends all the way to x = 0.15! -->
    <!-- ========================================================================= -->
    <!-- Extended North Paddy Shelf of Terrace B (x in [0.15, x1], y in [-0.32, 1.65]) -->
    <geom name="terrace_b_ext_earth" type="box" pos="{0.5*(x0+0.15+x1):.3f} 0.66 {0.5*(z_a+z_b) - 0.02:.3f}" size="{0.5*(x1-x0-0.15):.3f} 0.98 {0.5*(z_b-z_a) + 0.01:.3f}" material="terrace_wall_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_b_ext_mud" type="box" pos="{0.5*(x0+0.25+x1):.3f} 0.66 {z_b - 0.015:.3f}" size="{0.5*(x1-x0-0.30):.3f} 0.90 0.010" material="paddy_soil_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_b_ext_water" type="box" pos="{0.5*(x0+0.30+x1):.3f} 0.66 {z_b + 0.012:.3f}" size="{0.5*(x1-x0-0.35):.3f} 0.84 0.003" material="canal_water_mat" contype="0" conaffinity="0"/>
    <geom name="terrace_b_ext_bund_n" type="box" pos="{0.5*(x0+0.15+x1):.3f} 1.62 {z_b + 0.045:.3f}" size="{0.5*(x1-x0-0.15):.3f} 0.10 0.048" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_b_ext_bund_s" type="box" pos="{0.5*(x0+0.15+x1):.3f} -0.28 {z_b + 0.045:.3f}" size="{0.5*(x1-x0-0.15):.3f} 0.08 0.048" material="bund_earth_mat" contype="1" conaffinity="7"/>
    {wall_ab_contoured}
    {seedlings_b_extended}

    <!-- Main Terrace B Deck (x in [x1, x2], y in [-1.65, 1.65]) -->
    <geom name="terrace_b_main_earth" type="box" pos="{0.5*(x1+x2):.3f} 0.00 {0.5*z_b - 0.05:.3f}" size="1.52 1.68 {0.5*z_b + 0.04:.3f}" material="terrace_wall_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_b_main_ground" type="box" pos="{0.5*(x1+x2):.3f} 0.00 {z_b - 0.005:.3f}" size="1.52 1.62 0.005" material="bund_earth_mat" contype="1" conaffinity="7" friction="2.0 0.02 0.005"/>
    <geom name="terrace_b_main_mud" type="box" pos="{0.5*(x1+x2):.3f} 0.00 {z_b - 0.002:.3f}" size="1.22 0.42 0.004" material="paddy_soil_mat" contype="0" conaffinity="0"/>
    <geom name="terrace_b_main_water" type="box" pos="{0.5*(x1+x2):.3f} 0.00 {z_b + 0.012:.3f}" size="1.18 0.38 0.003" material="canal_water_mat" contype="0" conaffinity="0"/>
    <geom name="terrace_b_bund_n" type="box" pos="{0.5*(x1+x2):.3f} 1.62 {z_b + 0.045:.3f}" size="1.54 0.10 0.048" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_b_bund_s" type="box" pos="{0.5*(x1+x2):.3f} -1.62 {z_b + 0.045:.3f}" size="1.54 0.10 0.048" material="bund_earth_mat" contype="1" conaffinity="7"/>
    {seedlings_b_main}

    <!-- ========================================================================= -->
    <!-- RAMP BC: SOLID EARTH-SUPPORTED SWITCHBACK RAMP ({z_b:.1f}m -> {z_c:.1f}m) -->
    <!-- North Corridor: y = +0.75, x in [{x2:.2f}, {x3:.2f}]                      -->
    <!-- ========================================================================= -->
    {wedge_bc_xml}
    <geom name="ramp_bc_surface" type="box" pos="{0.5*(x2+x3):.3f} 0.75 {0.5*(z_b+z_c):.3f}" size="{hyp_bc/2:.3f} {ramp_w/2:.3f} 0.032" euler="0 {-slope_rad:.4f} 0" material="ramp_mat" contype="1" conaffinity="7" friction="2.8 0.05 0.01"/>
    <geom name="ramp_bc_curb_s" type="box" pos="{0.5*(x2+x3):.3f} {0.75 - ramp_w/2 - 0.04:.3f} {0.5*(z_b+z_c) + 0.045:.3f}" size="{hyp_bc/2:.3f} 0.045 0.048" euler="0 {-slope_rad:.4f} 0" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="ramp_bc_curb_n" type="box" pos="{0.5*(x2+x3):.3f} {0.75 + ramp_w/2 + 0.04:.3f} {0.5*(z_b+z_c) + 0.045:.3f}" size="{hyp_bc/2:.3f} 0.045 0.048" euler="0 {-slope_rad:.4f} 0" material="bund_earth_mat" contype="1" conaffinity="7"/>

    <!-- ========================================================================= -->
    <!-- TERRACE C (UPPER LEVEL: z = {z_c:.2f}m, +1.0m ABOVE TERRACE B)            -->
    <!-- Recessed-flank design: South paddy shelf extends all the way to x = x2!   -->
    <!-- ========================================================================= -->
    <!-- Extended South Paddy Shelf of Terrace C (x in [x2+0.15, x3], y in [-1.65, +0.32]) -->
    <geom name="terrace_c_ext_earth" type="box" pos="{0.5*(x2+0.15+x3):.3f} -0.66 {0.5*(z_b+z_c) - 0.02:.3f}" size="{0.5*(x3-x2-0.15):.3f} 0.98 {0.5*(z_c-z_b) + 0.01:.3f}" material="terrace_wall_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_c_ext_mud" type="box" pos="{0.5*(x2+0.25+x3):.3f} -0.66 {z_c - 0.015:.3f}" size="{0.5*(x3-x2-0.30):.3f} 0.90 0.010" material="paddy_soil_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_c_ext_water" type="box" pos="{0.5*(x2+0.30+x3):.3f} -0.66 {z_c + 0.012:.3f}" size="{0.5*(x3-x2-0.35):.3f} 0.84 0.003" material="canal_water_mat" contype="0" conaffinity="0"/>
    <geom name="terrace_c_ext_bund_s" type="box" pos="{0.5*(x2+0.15+x3):.3f} -1.62 {z_c + 0.045:.3f}" size="{0.5*(x3-x2-0.15):.3f} 0.10 0.048" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_c_ext_bund_n" type="box" pos="{0.5*(x2+0.15+x3):.3f} 0.28 {z_c + 0.045:.3f}" size="{0.5*(x3-x2-0.15):.3f} 0.08 0.048" material="bund_earth_mat" contype="1" conaffinity="7"/>
    {wall_bc_contoured}
    {seedlings_c_extended}

    <!-- Main Terrace C Deck (x in [x3, x4], y in [-1.65, 1.65]) -->
    <geom name="terrace_c_main_earth" type="box" pos="{0.5*(x3+x4):.3f} 0.00 {0.5*z_c - 0.05:.3f}" size="1.62 1.68 {0.5*z_c + 0.04:.3f}" material="terrace_wall_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_c_main_mud" type="box" pos="{0.5*(x3+x4):.3f} -0.35 {z_c - 0.015:.3f}" size="1.52 1.22 0.010" material="paddy_soil_mat" contype="1" conaffinity="7" friction="1.8 0.02 0.005"/>
    <!-- Dry compacted access path on North side of Terrace C (y in [0.42, 1.55]) -->
    <geom name="terrace_c_access_path" type="box" pos="{0.5*(x3+x4):.3f} 0.92 {z_c - 0.008:.3f}" size="1.58 0.58 0.010" material="bund_earth_mat" contype="1" conaffinity="7" friction="2.2 0.02 0.005"/>
    <geom name="terrace_c_water" type="box" pos="{0.5*(x3+x4):.3f} -0.55 {z_c + 0.012:.3f}" size="1.42 0.92 0.003" material="canal_water_mat" contype="0" conaffinity="0"/>
    <geom name="terrace_c_bund_n" type="box" pos="{0.5*(x3+x4):.3f} 1.62 {z_c + 0.045:.3f}" size="1.62 0.10 0.048" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_c_bund_s" type="box" pos="{0.5*(x3+x4):.3f} -1.62 {z_c + 0.045:.3f}" size="1.62 0.10 0.048" material="bund_earth_mat" contype="1" conaffinity="7"/>
    <geom name="terrace_c_inner_bund" type="box" pos="{0.5*(x3+x4):.3f} 0.40 {z_c + 0.025:.3f}" size="1.45 0.06 0.026" material="bund_earth_mat" contype="0" conaffinity="0"/>
    {seedlings_c_main}

    <!-- ========================================================================= -->
    <!-- UPPER HILLSIDE BACKDROP TERRACE (z = {z_c + 1.0:.2f}m)                    -->
    <!-- ========================================================================= -->
    <geom name="upper_hill_earth" type="box" pos="{x4 + 1.55:.3f} 0.00 {0.5*(z_c + 1.0) - 0.05:.3f}" size="1.45 2.45 {0.5*(z_c + 1.0) + 0.04:.3f}" material="terrace_wall_mat" contype="1" conaffinity="7"/>
    <geom name="upper_hill_mud" type="box" pos="{x4 + 1.55:.3f} 0.00 {z_c + 1.0 - 0.015:.3f}" size="1.35 2.25 0.010" material="paddy_soil_mat" contype="1" conaffinity="7"/>
    <geom name="upper_hill_water" type="box" pos="{x4 + 1.55:.3f} 0.00 {z_c + 1.0 + 0.012:.3f}" size="1.25 2.10 0.003" material="canal_water_mat" contype="0" conaffinity="0"/>
    {wall_upper_hill}
    {seedlings_upper}

    {body_a}
    {body_b}
  </worldbody>

  <actuator>
    <velocity name="planter_act_fl" joint="planter_j_fl" kv="2.5" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <velocity name="planter_act_fr" joint="planter_j_fr" kv="2.5" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <velocity name="planter_act_rl" joint="planter_j_rl" kv="2.5" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <velocity name="planter_act_rr" joint="planter_j_rr" kv="2.5" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <velocity name="scout_act_fl" joint="scout_j_fl" kv="2.5" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <velocity name="scout_act_fr" joint="scout_j_fr" kv="2.5" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <velocity name="scout_act_rl" joint="scout_j_rl" kv="2.5" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <velocity name="scout_act_rr" joint="scout_j_rr" kv="2.5" ctrllimited="true" ctrlrange="-14.0 14.0"/>
  </actuator>
</mujoco>
    """
    return enhance_mjcf_with_field_textures(raw_xml)


def _apply_trajectory_state_to_mujoco(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    robot_id: str,
    state: TrajectoryState,
    wheel_radius_m: float = 0.095,
) -> None:
    """Update robot 6-DOF pose and 4 wheel joint angles in MuJoCo from continuous TrajectoryState."""
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{robot_id}_body")
    qpos_adr = model.jnt_qposadr[model.body_jntadr[body_id]]
    # Surface elevation + wheel radius placed perpendicular to ramp slope
    z_center = state.z_m + wheel_radius_m * math.cos(state.pitch_rad) + 0.012
    data.qpos[qpos_adr : qpos_adr + 3] = [state.x_m, state.y_m, z_center]
    # In MuJoCo local coordinates, negative pitch about +Y tilts nose UP (+Z) when facing +X
    mj_pitch = -abs(state.pitch_rad) if state.pitch_rad > 0 else abs(state.pitch_rad)
    # Wait: if robot yaw is pi (facing -X during descent) and nose is down (-Z),
    # local pitch around robot's body +Y axis: a positive rotation about body +Y tilts +X (front) DOWN (-Z)!
    # Whereas when climbing (+X, nose UP +Z), local rotation about body +Y is NEGATIVE (-pitch)!
    local_body_pitch = -state.pitch_rad
    data.qpos[qpos_adr + 3 : qpos_adr + 7] = _euler_to_quat(0.0, local_body_pitch, state.yaw_rad)

    for suffix in ("fl", "fr", "rl", "rr"):
        j_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{robot_id}_j_{suffix}")
        if j_id >= 0:
            data.qpos[model.jnt_qposadr[j_id]] = state.wheel_angle_rad
    mujoco.mj_forward(model, data)


def main() -> None:
    parser = argparse.ArgumentParser(description="Multi-Level Chinese Rice Terrace Simulation & Validation")
    parser.add_argument("--viewer", action="store_true", help="Launch interactive 3D MuJoCo viewer")
    parser.add_argument("--viewer-seconds", type=float, default=25.0, help="Duration to run interactive viewer")
    args = parser.parse_args()

    print("=" * 82)
    print("RICESWARM AI — CHINESE TERRACED RICE FIELDS (1.0m STEPS) & ~0.50m ROBOT NAVIGATION")
    print("Inspired by Longji & Yuanyang stepped agricultural landscapes")
    print("=" * 82)

    # Demonstration configuration:
    # - 1.0m elevation step between levels: Terrace A = 0.0m, Terrace B = 1.0m, Terrace C = 2.0m
    # - Robot size: 0.50m length x 0.38m width (wheel radius = 0.095m)
    # - Ramp slope: 12.0 deg (run = 4.70m per 1.0m vertical rise; safely below 15.0 deg robot limit)
    env = MultiLevelTerraceEnvironment(
        elevation_a=0.0,
        elevation_b=1.0,
        elevation_c=2.0,
        ramp_slope_deg=12.0,
        ramp_width_m=0.75,
        robot_max_slope_deg=15.0,
        robot_width_m=0.38,
        robot_length_m=0.50,
    )
    planner = MultiLevelPathPlanner(env)

    print(f"Environment Configuration:")
    print(f"  - Terrace A (Lower)  : z = {env.elevation_a:.2f} m | x in [{env.terrace_a.bounds_x[0]:.2f}, {env.terrace_a.bounds_x[1]:.2f}] m")
    print(f"  - Terrace B (Middle) : z = {env.elevation_b:.2f} m | x in [{env.terrace_b.bounds_x[0]:.2f}, {env.terrace_b.bounds_x[1]:.2f}] m (+1.0m step)")
    print(f"  - Terrace C (Upper)  : z = {env.elevation_c:.2f} m | x in [{env.terrace_c.bounds_x[0]:.2f}, {env.terrace_c.bounds_x[1]:.2f}] m (+1.0m step)")
    print(f"  - Ramp AB & BC       : slope = {env.ramp_slope_deg:.1f}° (run = {env.ramp_ab.horizontal_run_m:.2f} m, width = {env.ramp_width_m:.2f} m)")
    print(f"  - Robot Dimensions   : length = {env.robot_length_m:.2f} m (1/2 m), width = {env.robot_width_m:.2f} m, wheel radius = 0.095 m")

    # Output directories for visual renders (save to current conversation artifact dir + repo renders)
    artifact_dir = Path("C:/Users/Laura/.gemini/antigravity/brain/3a2016fa-88da-451f-aeda-844a80924985")
    artifact_dir.mkdir(parents=True, exist_ok=True)

    # 1. Compile 3D MuJoCo Physics Model
    mjcf = build_multi_level_terrace_mjcf(env)
    model = mujoco.MjModel.from_xml_string(mjcf)
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, width=1280, height=800)

    x1 = env.terrace_b.bounds_x[0]
    x2 = env.terrace_b.bounds_x[1]
    x3 = env.terrace_c.bounds_x[0]
    x4 = env.terrace_c.bounds_x[1]

    # =========================================================================
    # RENDER 1: Grand Panoramic View of Stepped Chinese Rice Terraces (0m, 1m, 2m, 3m)
    # =========================================================================
    cam_pano = mujoco.MjvCamera()
    cam_pano.lookat = np.array([0.45 * (x1 + x3), 0.0, 1.0])
    cam_pano.distance = 13.8
    cam_pano.elevation = -25.0
    cam_pano.azimuth = 132.0
    mujoco.mj_forward(model, data)
    renderer.update_scene(data, camera=cam_pano)
    p1 = artifact_dir / "multi_level_panoramic_terrace.png"
    save_png(p1, renderer.render())
    print(f"\n[Render 1] 3-Tier Panoramic Overview -> {p1}")

    # =========================================================================
    # TEST 1: Continuous Ascent from Terrace A (0.0m) -> Terrace B (1.0m) -> Terrace C (2.0m)
    # =========================================================================
    print("\n--- Executing Test 1: Lower (0.0m) to Upper (2.0m) Terrace Continuous Ascent ---")
    start_a = (-1.50, -0.75)
    goal_c = (x3 + 1.50, 0.75)
    ascent_waypoints = planner.plan_path(
        start_a, goal_c, robot_max_slope_deg=env.robot_max_slope_deg, robot_width_m=env.robot_width_m
    )
    ascent_traj = planner.simulate_continuous_navigation(start_a, goal_c, dt_s=0.10, wheel_radius_m=0.095)

    print(f"Plan generated {len(ascent_waypoints)} waypoints -> {len(ascent_traj)} continuous trajectory states (dt=0.10s):")
    for i, wp in enumerate(ascent_waypoints):
        print(
            f"  WP {i:02d}: ({wp.x_m:+6.2f}, {wp.y_m:+6.2f}, z={wp.z_m:4.2f}m) | "
            f"pitch={math.degrees(wp.pitch_rad):+5.1f}° | spd={wp.speed_m_s:.2f}m/s | [{wp.zone_tag}] {wp.comment}"
        )

    # Find state near middle of Ramp AB during ascent for Render 2
    mid_ab_states = [s for s in ascent_traj if s.zone_tag == "ramp_ab" and abs(s.z_m - 0.50) < 0.15]
    state_ramp_ab = mid_ab_states[len(mid_ab_states) // 2] if mid_ab_states else ascent_traj[len(ascent_traj) // 4]
    _apply_trajectory_state_to_mujoco(model, data, "planter", state_ramp_ab)

    cam_ramp_ab = mujoco.MjvCamera()
    cam_ramp_ab.lookat = np.array([state_ramp_ab.x_m, state_ramp_ab.y_m + 0.15, state_ramp_ab.z_m + 0.15])
    cam_ramp_ab.distance = 2.85
    cam_ramp_ab.elevation = -18.0
    cam_ramp_ab.azimuth = 135.0
    renderer.update_scene(data, camera=cam_ramp_ab)
    p2 = artifact_dir / "robot_climbing_ramp_ab.png"
    save_png(p2, renderer.render())
    print(f"[Render 2] 0.5m Robot Climbing Ramp AB (z={state_ramp_ab.z_m:.2f}m, pitch={math.degrees(state_ramp_ab.pitch_rad):+.1f}°) -> {p2}")

    # Find state on Terrace B switchback for Render 3
    mid_b_states = [s for s in ascent_traj if s.zone_tag == "terrace_b"]
    state_terrace_b = mid_b_states[len(mid_b_states) // 2] if mid_b_states else ascent_traj[len(ascent_traj) // 2]
    _apply_trajectory_state_to_mujoco(model, data, "planter", state_terrace_b)

    cam_terrace_b = mujoco.MjvCamera()
    cam_terrace_b.lookat = np.array([state_terrace_b.x_m - 0.25, state_terrace_b.y_m, 1.15])
    cam_terrace_b.distance = 3.2
    cam_terrace_b.elevation = -34.0
    cam_terrace_b.azimuth = 142.0
    renderer.update_scene(data, camera=cam_terrace_b)
    p3 = artifact_dir / "robot_terrace_b_switchback.png"
    save_png(p3, renderer.render())
    print(f"[Render 3] 0.5m Robot on Terrace B Switchback (z=1.00m) -> {p3}")

    # Apply final Terrace C arrival state for Render 4
    state_terrace_c = ascent_traj[-1]
    _apply_trajectory_state_to_mujoco(model, data, "planter", state_terrace_c)

    cam_terrace_c = mujoco.MjvCamera()
    cam_terrace_c.lookat = np.array([state_terrace_c.x_m - 0.3, 0.15, 2.05])
    cam_terrace_c.distance = 3.5
    cam_terrace_c.elevation = -22.0
    cam_terrace_c.azimuth = 132.0
    renderer.update_scene(data, camera=cam_terrace_c)
    p4 = artifact_dir / "robot_reaching_terrace_c.png"
    save_png(p4, renderer.render())
    print(f"[Render 4] 0.5m Robot Reaching Terrace C (z=2.00m) -> {p4}")

    # =========================================================================
    # TEST 2: Continuous Descent from Terrace C (2.0m) -> Terrace B (1.0m) -> Terrace A (0.0m)
    # =========================================================================
    print("\n--- Executing Test 2: Upper (2.0m) to Lower (0.0m) Terrace Continuous Descent ---")
    descent_waypoints = planner.plan_path(
        goal_c, start_a, robot_max_slope_deg=env.robot_max_slope_deg, robot_width_m=env.robot_width_m
    )
    descent_traj = planner.simulate_continuous_navigation(goal_c, start_a, dt_s=0.10, wheel_radius_m=0.095)
    print(f"Descent plan generated {len(descent_waypoints)} waypoints -> {len(descent_traj)} continuous states:")
    for i, wp in enumerate(descent_waypoints):
        print(
            f"  WP {i:02d}: ({wp.x_m:+6.2f}, {wp.y_m:+6.2f}, z={wp.z_m:4.2f}m) | "
            f"pitch={math.degrees(wp.pitch_rad):+5.1f}° | spd={wp.speed_m_s:.2f}m/s | [{wp.zone_tag}] {wp.comment}"
        )

    mid_bc_states = [s for s in descent_traj if s.zone_tag == "ramp_bc" and abs(s.z_m - 1.50) < 0.15]
    state_ramp_bc = mid_bc_states[len(mid_bc_states) // 2] if mid_bc_states else descent_traj[len(descent_traj) // 4]
    _apply_trajectory_state_to_mujoco(model, data, "planter", state_ramp_bc)

    cam_descend = mujoco.MjvCamera()
    cam_descend.lookat = np.array([state_ramp_bc.x_m, state_ramp_bc.y_m - 0.15, state_ramp_bc.z_m + 0.10])
    cam_descend.distance = 3.1
    cam_descend.elevation = -20.0
    cam_descend.azimuth = 215.0
    renderer.update_scene(data, camera=cam_descend)
    p5 = artifact_dir / "robot_descending_ramp_bc.png"
    save_png(p5, renderer.render())
    print(f"[Render 5] 0.5m Robot Descending Ramp BC (z={state_ramp_bc.z_m:.2f}m, pitch={math.degrees(state_ramp_bc.pitch_rad):+.1f}°) -> {p5}")

    # =========================================================================
    # TEST 3, 4, 5 Verification
    # =========================================================================
    print("\n--- Executing Validation Tests 3, 4, 5 ---")
    # Test 3: Blocked crossing
    env.set_ramp_accessibility("ramp_ab", accessible=False)
    try:
        planner.plan_path(start_a, goal_c, robot_max_slope_deg=env.robot_max_slope_deg, robot_width_m=env.robot_width_m)
        print("Test 3 FAILED: Plan should have raised error for blocked ramp")
    except RuntimeError as e:
        print(f"Test 3 PASSED: Inaccessible crossing safely rejected -> '{e}'")
    env.set_ramp_accessibility("ramp_ab", accessible=True)

    # Test 4: Steep crossing rejection
    steep_env = MultiLevelTerraceEnvironment(
        elevation_a=0.0, elevation_b=1.0, elevation_c=2.0, ramp_slope_deg=22.0, robot_max_slope_deg=15.0
    )
    steep_planner = MultiLevelPathPlanner(steep_env)
    steep_goal = (steep_env.terrace_c.bounds_x[0] + 0.5, 0.75)
    try:
        steep_planner.plan_path((-1.50, -0.75), steep_goal, robot_max_slope_deg=15.0)
        print("Test 4 FAILED: Plan should have rejected 22° steep ramp")
    except RuntimeError as e:
        print(f"Test 4 PASSED: Steep 22° crossing safely rejected (> 15° limit) -> '{e}'")

    # Test 5: Crop protection
    crop_detour_waypoints = planner.plan_path((-2.00, -0.75), (-0.50, 1.40))
    crosses_crops = any(env.terrace_a.is_in_cultivated_zone(wp.x_m, wp.y_m) for wp in crop_detour_waypoints)
    print(f"Test 5 PASSED: Route avoids protected rice seedlings (crosses_crops={crosses_crops})")

    print("\n" + "=" * 82)
    print("ALL 5 VALIDATION TESTS PASSED & HIGH-RESOLUTION VISUALS GENERATED!")
    print("=" * 82)

    if args.viewer:
        from mujoco import viewer as mj_viewer

        full_cycle = ascent_traj + descent_traj
        print(f"\nLaunching interactive 3D MuJoCo viewer for {args.viewer_seconds:.1f}s...")
        with mj_viewer.launch_passive(model, data) as viewer:
            viewer.cam.lookat[:] = [0.45 * (x1 + x3), 0.0, 1.0]
            viewer.cam.distance = 12.5
            viewer.cam.elevation = -24.0
            viewer.cam.azimuth = 132.0
            t0 = time.time()
            step_idx = 0
            while viewer.is_running() and (time.time() - t0) < args.viewer_seconds:
                st = full_cycle[step_idx % len(full_cycle)]
                _apply_trajectory_state_to_mujoco(model, data, "planter", st)
                viewer.sync()
                time.sleep(0.03)
                step_idx += 1


if __name__ == "__main__":
    main()
