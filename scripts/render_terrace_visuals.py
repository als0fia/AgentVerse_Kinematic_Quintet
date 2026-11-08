"""Render photorealistic 3D Chinese rice terrace views with MuJoCo offscreen renderer.

Generates 4 views of the 1.0m-stepped Longji/Yuanyang Chinese terraced rice fields and ~0.50m agricultural rovers:
1. Grand Amphitheatre Overview: All stepped levels (0.0m, 1.0m, 2.0m, 3.0m), earth-supported ramps, reflective sky water.
2. Ramp Incline Detail: 0.50m rover climbing Ramp AB (0.0m -> 1.0m) beside contoured stone retaining wall.
3. Rover Scale Comparison: 0.50m (1/2 m) Paddy Rover next to 1.0m stepped terrace wall and rice seedlings.
4. Longsheng Mountain Cascade: Panoramic perspective overlooking the stepped water paddies and curved bunds.
"""
from __future__ import annotations

import math
import os
import struct
import sys
import zlib
from pathlib import Path
import numpy as np

if "MUJOCO_GL" in os.environ:
    del os.environ["MUJOCO_GL"]

_REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO_ROOT))
sys.path.insert(0, str(_REPO_ROOT / "src"))

import mujoco
from ricebot.world import MultiLevelPathPlanner, MultiLevelTerraceEnvironment
from scripts.run_multi_level_terrace_navigation import (
    _apply_trajectory_state_to_mujoco,
    build_multi_level_terrace_mjcf,
    save_png,
)


def render_all_terrace_views(output_dir: Path) -> list[Path]:
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
    mjcf = build_multi_level_terrace_mjcf(env)

    model = mujoco.MjModel.from_xml_string(mjcf)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, width=1280, height=800)
    saved_paths: list[Path] = []

    x1 = env.terrace_b.bounds_x[0]
    x3 = env.terrace_c.bounds_x[0]
    ascent_traj = planner.simulate_continuous_navigation((-1.50, -0.75), (x3 + 1.50, 0.75), dt_s=0.10)

    # 1. Grand Amphitheatre Overview
    cam1 = mujoco.MjvCamera()
    cam1.lookat = np.array([0.45 * (x1 + x3), 0.0, 1.0])
    cam1.distance = 13.8
    cam1.elevation = -25.0
    cam1.azimuth = 132.0
    renderer.update_scene(data, camera=cam1)
    out1 = output_dir / "chinese_rice_terrace_preview.png"
    save_png(out1, renderer.render())
    saved_paths.append(out1)
    print(f"[Render] Grand Panorama -> {out1}")

    # 2. Ramp Incline Detail (0.50m Rover climbing Ramp AB)
    mid_ab = [s for s in ascent_traj if s.zone_tag == "ramp_ab" and abs(s.z_m - 0.50) < 0.15]
    st_ab = mid_ab[len(mid_ab) // 2] if mid_ab else ascent_traj[len(ascent_traj) // 4]
    _apply_trajectory_state_to_mujoco(model, data, "planter", st_ab)

    cam2 = mujoco.MjvCamera()
    cam2.lookat = np.array([st_ab.x_m, st_ab.y_m + 0.15, st_ab.z_m + 0.15])
    cam2.distance = 2.85
    cam2.elevation = -18.0
    cam2.azimuth = 135.0
    renderer.update_scene(data, camera=cam2)
    out2 = output_dir / "ramp_40deg_detail.png"
    save_png(out2, renderer.render())
    saved_paths.append(out2)
    print(f"[Render] Ramp Climb Detail -> {out2}")

    # 3. Rover Scale Comparison (0.50m Rover vs 1.0m Step Wall)
    st_scale = ascent_traj[2]
    _apply_trajectory_state_to_mujoco(model, data, "planter", st_scale)
    cam3 = mujoco.MjvCamera()
    cam3.lookat = np.array([0.2, -0.35, 0.45])
    cam3.distance = 2.9
    cam3.elevation = -16.0
    cam3.azimuth = 142.0
    renderer.update_scene(data, camera=cam3)
    out3 = output_dir / "terrace_scale_comparison.png"
    save_png(out3, renderer.render())
    saved_paths.append(out3)
    print(f"[Render] Scale Comparison (0.5m Robot vs 1.0m Terrace Step) -> {out3}")

    # 4. Longsheng Vista Panorama
    cam4 = mujoco.MjvCamera()
    cam4.lookat = np.array([0.5 * (x1 + x3), 0.0, 1.1])
    cam4.distance = 14.5
    cam4.elevation = -34.0
    cam4.azimuth = 118.0
    renderer.update_scene(data, camera=cam4)
    out4 = output_dir / "terrace_longsheng_vista.png"
    save_png(out4, renderer.render())
    saved_paths.append(out4)
    print(f"[Render] Longsheng Vista -> {out4}")

    return saved_paths


if __name__ == "__main__":
    out_dir = _REPO_ROOT / "docs" / "images" / "terrace_renders"
    if len(sys.argv) > 1:
        out_dir = Path(sys.argv[1])
    render_all_terrace_views(out_dir)
