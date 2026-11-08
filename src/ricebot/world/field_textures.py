"""Procedural agricultural paddy-field texture generator & MJCF material assets (`src/ricebot/world/field_textures.py`).

Generates realistic, seamless 2D PNG textures for the level puddled rice-field plot:
- `rice_field_paddy.png`: Puddled wet silt-loam soil with subtle harrowed row micro-furrows,
  organic soil clod mottling, and shallow water film glints between crop rows.
- `terrace_bund_earth.png`: Compacted earthen paddy bund (田埂) clay-loam with fine root/straw flecks.
- `paddy_mud_hazard.png`: Dark waterlogged *Lannitian* (烂泥田) soupy mud basin texture.
- `paddy_viable_loam.png`: Consolidated moist "mud-cake" silt-loam seedbed texture.
- `paddy_water_mirror.png`: Shallow irrigation canal & puddle surface ripple texture.

Uses only Python standard library (`zlib`, `struct`, `pathlib`) + `numpy` so CI and headless
runs require zero external image libraries.
"""
from __future__ import annotations

from pathlib import Path
import struct
import zlib

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
TEXTURE_DIR = REPO_ROOT / "assets" / "textures"

TEXTURE_FILENAMES: dict[str, str] = {
    "paddy_soil_tex": "rice_field_paddy.png",
    "bund_earth_tex": "terrace_bund_earth.png",
    "mud_hazard_tex": "paddy_mud_hazard.png",
    "viable_loam_tex": "paddy_viable_loam.png",
    "canal_water_tex": "paddy_water_mirror.png",
}


def _write_rgb_png(path: Path, rgb: np.ndarray) -> None:
    """Write an `(H, W, 3)` uint8 array to a standard RGB PNG file using stdlib `zlib`."""
    height, width, channels = rgb.shape
    if channels != 3:
        raise ValueError("Expected (H, W, 3) uint8 RGB array")
    path.parent.mkdir(parents=True, exist_ok=True)

    raw_rows = b"".join(b"\x00" + rgb[y].tobytes() for y in range(height))

    def _chunk(tag: bytes, data: bytes) -> bytes:
        crc = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack("!I", len(data)) + tag + data + struct.pack("!I", crc)

    ihdr = struct.pack("!IIBBBBB", width, height, 8, 2, 0, 0, 0)
    png_bytes = (
        b"\x89PNG\r\n\x1a\n"
        + _chunk(b"IHDR", ihdr)
        + _chunk(b"IDAT", zlib.compress(raw_rows, level=6))
        + _chunk(b"IEND", b"")
    )
    path.write_bytes(png_bytes)


def _fbm_periodic(ny: int, nx: int, seed: int) -> np.ndarray:
    """Deterministic multi-octave periodic value noise in `[-1, 1]` for seamless tiling."""
    rng = np.random.default_rng(seed)
    yy, xx = np.meshgrid(
        np.linspace(0.0, 2.0 * np.pi, ny, endpoint=False),
        np.linspace(0.0, 2.0 * np.pi, nx, endpoint=False),
        indexing="ij",
    )
    acc = np.zeros((ny, nx), dtype=np.float64)
    weight_sum = 0.0
    for octave, freq in enumerate((1, 2, 4, 8, 16)):
        amp = 0.52**octave
        p1, p2, p3, p4 = rng.uniform(0.0, 2.0 * np.pi, size=4)
        wave = (
            np.sin(freq * xx + p1) * np.cos(freq * yy + p2)
            + 0.65 * np.cos(freq * (xx - yy) + p3)
            + 0.45 * np.sin(freq * (xx + 0.5 * yy) + p4)
        )
        acc += amp * wave
        weight_sum += amp * 2.1
    fine_grain = rng.normal(0.0, 0.08, size=(ny, nx))
    return np.clip(acc / max(weight_sum, 1e-6) + fine_grain, -1.0, 1.0)


def generate_paddy_field_textures(force: bool = False) -> dict[str, Path]:
    """Ensure all 5 realistic agricultural paddy-field PNG textures exist on disk."""
    TEXTURE_DIR.mkdir(parents=True, exist_ok=True)
    # Remove legacy stepped-terrace brick retaining wall texture if present
    legacy_wall = TEXTURE_DIR / "terrace_retaining_wall.png"
    if legacy_wall.exists():
        try:
            legacy_wall.unlink()
        except OSError:
            pass

    paths = {key: TEXTURE_DIR / fname for key, fname in TEXTURE_FILENAMES.items()}
    if not force and all(p.exists() and p.stat().st_size > 1024 for p in paths.values()):
        return paths

    ny, nx = 256, 256
    yy, xx = np.meshgrid(
        np.linspace(0.0, 2.0 * np.pi, ny, endpoint=False),
        np.linspace(0.0, 2.0 * np.pi, nx, endpoint=False),
        indexing="ij",
    )

    # 1. Main puddled paddy silt-loam ground with subtle wet water sheen & organic clods
    n_soil = _fbm_periodic(ny, nx, seed=101)
    n_micro = _fbm_periodic(ny, nx, seed=102)
    furrow_ripple = 0.06 * np.sin(6.0 * yy + 0.35 * np.sin(2.0 * xx))
    water_film_mask = np.clip(0.5 + 0.85 * (n_soil - 0.12), 0.0, 1.0)
    r_paddy = 86.0 + 22.0 * n_soil + 10.0 * n_micro - 12.0 * water_film_mask + 14.0 * furrow_ripple
    g_paddy = 72.0 + 18.0 * n_soil + 9.0 * n_micro - 6.0 * water_film_mask + 11.0 * furrow_ripple
    b_paddy = 52.0 + 13.0 * n_soil + 7.0 * n_micro + 6.0 * water_film_mask + 7.0 * furrow_ripple
    rgb_paddy = np.stack(
        [np.clip(r_paddy, 38, 138), np.clip(g_paddy, 34, 118), np.clip(b_paddy, 26, 94)],
        axis=-1,
    ).astype(np.uint8)
    _write_rgb_png(paths["paddy_soil_tex"], rgb_paddy)

    # 2. Compacted earthen bund (low perimeter ridge with dry-top clay & straw flecks)
    n_bund = _fbm_periodic(ny, nx, seed=203)
    n_straw = _fbm_periodic(ny, nx, seed=204)
    straw_mask = (n_straw > 0.32).astype(np.float64)
    r_bund = 114.0 + 24.0 * n_bund + 18.0 * straw_mask
    g_bund = 92.0 + 19.0 * n_bund + 15.0 * straw_mask
    b_bund = 64.0 + 14.0 * n_bund + 6.0 * straw_mask
    rgb_bund = np.stack(
        [np.clip(r_bund, 55, 165), np.clip(g_bund, 45, 140), np.clip(b_bund, 30, 102)],
        axis=-1,
    ).astype(np.uint8)
    _write_rgb_png(paths["bund_earth_tex"], rgb_bund)

    # 3. Dark waterlogged Lannitian mud hazard basin
    n_mud = _fbm_periodic(ny, nx, seed=305)
    r_mud = 52.0 + 14.0 * n_mud
    g_mud = 41.0 + 12.0 * n_mud
    b_mud = 31.0 + 10.0 * n_mud
    rgb_mud = np.stack(
        [np.clip(r_mud, 24, 82), np.clip(g_mud, 20, 68), np.clip(b_mud, 15, 54)],
        axis=-1,
    ).astype(np.uint8)
    _write_rgb_png(paths["mud_hazard_tex"], rgb_mud)

    # 4. Consolidated viable silt-loam planting bed ("mud-cake" aerobic seedbed)
    n_loam = _fbm_periodic(ny, nx, seed=407)
    r_loam = 116.0 + 20.0 * n_loam
    g_loam = 95.0 + 16.0 * n_loam
    b_loam = 66.0 + 12.0 * n_loam
    rgb_loam = np.stack(
        [np.clip(r_loam, 68, 158), np.clip(g_loam, 54, 132), np.clip(b_loam, 38, 96)],
        axis=-1,
    ).astype(np.uint8)
    _write_rgb_png(paths["viable_loam_tex"], rgb_loam)

    # 5. Shallow paddy water & irrigation channel surface texture
    n_water = _fbm_periodic(ny, nx, seed=509)
    ripple = 0.5 * np.sin(5.0 * xx + 3.0 * yy) + 0.5 * np.cos(4.0 * xx - 6.0 * yy)
    r_water = 58.0 + 14.0 * n_water + 8.0 * ripple
    g_water = 88.0 + 16.0 * n_water + 10.0 * ripple
    b_water = 86.0 + 18.0 * n_water + 12.0 * ripple
    rgb_water = np.stack(
        [np.clip(r_water, 32, 108), np.clip(g_water, 54, 134), np.clip(b_water, 52, 138)],
        axis=-1,
    ).astype(np.uint8)
    _write_rgb_png(paths["canal_water_tex"], rgb_water)

    return paths


def build_world_texture_assets_xml() -> str:
    """Return MJCF `<asset>` declarations for realistic paddy soil, mud, loam, bunds, and shallow water."""
    paths = generate_paddy_field_textures(force=False)
    paddy_file = paths["paddy_soil_tex"].as_posix()
    bund_file = paths["bund_earth_tex"].as_posix()
    mud_file = paths["mud_hazard_tex"].as_posix()
    loam_file = paths["viable_loam_tex"].as_posix()
    water_file = paths["canal_water_tex"].as_posix()
    return f"""
    <texture name="sky_tex" type="skybox" builtin="gradient" rgb1="0.74 0.85 0.94" rgb2="0.28 0.42 0.58" width="256" height="256"/>
    <texture name="paddy_soil_tex" type="2d" file="{paddy_file}"/>
    <texture name="bund_earth_tex" type="2d" file="{bund_file}"/>
    <texture name="mud_hazard_tex" type="2d" file="{mud_file}"/>
    <texture name="viable_loam_tex" type="2d" file="{loam_file}"/>
    <texture name="canal_water_tex" type="2d" file="{water_file}"/>

    <material name="paddy_soil_mat" texture="paddy_soil_tex" texrepeat="7 7" reflectance="0.06" shininess="0.38" specular="0.28"/>
    <material name="bund_earth_mat" texture="bund_earth_tex" texrepeat="4 2" reflectance="0.02" shininess="0.15" specular="0.10"/>
    <material name="mud_hazard_mat" texture="mud_hazard_tex" texrepeat="2 2" rgba="0.88 0.84 0.80 0.96" reflectance="0.12" shininess="0.82" specular="0.62"/>
    <material name="viable_loam_mat" texture="viable_loam_tex" texrepeat="2 2" rgba="0.96 0.94 0.90 0.94" reflectance="0.03" shininess="0.24" specular="0.16"/>
    <material name="crop_bed_mat" texture="paddy_soil_tex" texrepeat="2 2" rgba="0.34 0.42 0.26 0.90" shininess="0.32" specular="0.22"/>
    <material name="canal_water_mat" texture="canal_water_tex" texrepeat="3 3" rgba="0.32 0.50 0.48 0.62" reflectance="0.22" shininess="0.92" specular="0.82"/>
    <material name="shallow_water_mat" texture="canal_water_tex" texrepeat="5 5" rgba="0.36 0.48 0.42 0.34" reflectance="0.16" shininess="0.88" specular="0.72"/>
    <material name="wheel_rubber_mat" rgba="0.16 0.16 0.18 1"/>
    <hfield name="paddy_micro_relief" nrow="32" ncol="48" size="1.65 0.92 0.035 0.015"/>
"""
