"""Render photorealistic 3D Chinese rice terrace views with MuJoCo offscreen renderer.

Generates 4 views:
1. Grand Amphitheatre Overview: All stepped levels (-1m to +4m), 40° ramps, reflective sky water.
2. 40° Ramp Incline Detail: Level 1 (0.0m) to Level 2 (1.0m) transition with sheer retaining wall.
3. Swarm Scale Comparison: 25cm Paddy Ant rovers vs massive 1.0m stepped terrain (4:1 scale).
4. Longsheng Mountain Cascade: Panoramic perspective overlooking the terrace ridges.
"""
from __future__ import annotations

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
from ricebot.mujoco_sim.adapter import compose_paddypatch_mjcf
from ricebot.world.field_textures import enhance_mjcf_with_field_textures


def save_png(filename: Path, img_array: np.ndarray) -> None:
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


def render_all_terrace_views(output_dir: Path) -> list[Path]:
    raw_xml = compose_paddypatch_mjcf()
    textured_xml = enhance_mjcf_with_field_textures(raw_xml)

    model = mujoco.MjModel.from_xml_string(textured_xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, width=1280, height=800)
    saved_paths = []

    # 1. Grand Amphitheatre Overview
    cam1 = mujoco.MjvCamera()
    cam1.lookat = np.array([4.2, 0.0, 0.9])
    cam1.distance = 9.5
    cam1.elevation = -26.0
    cam1.azimuth = 132.0
    renderer.update_scene(data, camera=cam1)
    out1 = output_dir / "chinese_rice_terrace_preview.png"
    save_png(out1, renderer.render())
    saved_paths.append(out1)
    print(f"[Render] Grand Panorama -> {out1}")

    # 2. 40° Ramp Incline Detail
    cam2 = mujoco.MjvCamera()
    cam2.lookat = np.array([3.9, 0.5, 0.6])
    cam2.distance = 3.2
    cam2.elevation = -16.0
    cam2.azimuth = 145.0
    renderer.update_scene(data, camera=cam2)
    out2 = output_dir / "ramp_40deg_detail.png"
    save_png(out2, renderer.render())
    saved_paths.append(out2)
    print(f"[Render] 40-deg Ramp Detail -> {out2}")

    # 3. Rover Scale Comparison
    cam3 = mujoco.MjvCamera()
    cam3.lookat = np.array([1.6, -0.4, 0.25])
    cam3.distance = 2.4
    cam3.elevation = -12.0
    cam3.azimuth = 160.0
    renderer.update_scene(data, camera=cam3)
    out3 = output_dir / "terrace_scale_comparison.png"
    save_png(out3, renderer.render())
    saved_paths.append(out3)
    print(f"[Render] Scale Comparison -> {out3}")

    # 4. Longsheng Vista Panorama
    cam4 = mujoco.MjvCamera()
    cam4.lookat = np.array([6.5, 0.0, 1.2])
    cam4.distance = 12.0
    cam4.elevation = -42.0
    cam4.azimuth = 95.0
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
