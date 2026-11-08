r"""Field textures for photorealistic Longsheng & Yuanyang rice paddy terraces.

Provides seamless high-resolution textures matching terraced rice fields in China:
- Lush emerald and lime green young rice seedling rows with wet soil/water furrows
- Sunken terrace water beds mirroring the mountain sky with soft ripples and specular glints
- Sinuous golden-ochre clay bund embankments (田埂) with grass-fringed crests
- Ancient stacked stone & packed clay terrace retaining walls
- Fertile dark organic loam soil and deep saturated Lannitian mud hazard pits

Non-invasive: modifies MJCF XML at runtime without altering git-tracked source files.
"""
from __future__ import annotations

from pathlib import Path
import re
from typing import Callable

# Locate textures relative to the package or repository root
_CURRENT_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _CURRENT_DIR.parents[2]  # from src/ricebot/world -> repo root
TEXTURES_DIR = _REPO_ROOT / "assets" / "textures"

# Fallback paths if run from different root
_FALLBACK_DIRS = [
    Path("C:/Users/Laura/Downloads/AgentVerse_Kinematic_Quintet/assets/textures"),
    Path("C:/Users/Laura/Downloads/riceswarm_feature_source/assets/textures"),
    Path(__file__).resolve().parents[1] / "assets" / "textures",
]


def get_textures_dir() -> Path:
    """Resolve the directory containing texture PNGs."""
    if TEXTURES_DIR.exists():
        return TEXTURES_DIR
    for d in _FALLBACK_DIRS:
        if d.exists():
            return d
    return TEXTURES_DIR


def get_texture_path(filename: str) -> Path:
    """Return the absolute path to a specific texture file."""
    return get_textures_dir() / filename


def build_textured_asset_xml(textures_dir: Path | None = None) -> str:
    """Generate the <asset> XML block with photorealistic rice terrace textures and tuned materials."""
    t_dir = textures_dir or get_textures_dir()

    rice_crop_p = (t_dir / "rice_field_paddy.png").as_posix()
    water_mirror_p = (t_dir / "paddy_water_mirror.png").as_posix()
    bund_earth_p = (t_dir / "terrace_bund_earth.png").as_posix()
    viable_loam_p = (t_dir / "paddy_viable_loam.png").as_posix()
    mud_hazard_p = (t_dir / "paddy_mud_hazard.png").as_posix()
    terrace_wall_p = (t_dir / "terrace_retaining_wall.png").as_posix()

    return f"""  <asset>
    <!-- Misty mountain atmosphere matching Chinese terrace scenery -->
    <texture name="sky_tex" type="skybox" builtin="gradient" rgb1="0.65 0.82 0.95" rgb2="0.25 0.45 0.38" width="512" height="512"/>

    <!-- High-Resolution Photorealistic Rice Terrace Textures -->
    <texture name="rice_paddy_crop_tex" type="2d" file="{rice_crop_p}"/>
    <texture name="paddy_water_mirror_tex" type="2d" file="{water_mirror_p}"/>
    <texture name="terrace_bund_earth_tex" type="2d" file="{bund_earth_p}"/>
    <texture name="paddy_viable_loam_tex" type="2d" file="{viable_loam_p}"/>
    <texture name="paddy_mud_hazard_tex" type="2d" file="{mud_hazard_p}"/>
    <texture name="terrace_wall_tex" type="2d" file="{terrace_wall_p}"/>

    <!-- Tuned Materials: Photorealistic Rice Field Appearance & Reflectance -->
    <!-- 1. Paddy soil & terraced ground: natural moist organic paddy loam -->
    <material name="paddy_soil_mat" texture="paddy_viable_loam_tex" texrepeat="5 5" reflectance="0.06" roughness="0.88"/>

    <!-- 2. Dense seedling clumps and planting beds -->
    <material name="crop_bed_mat" texture="rice_paddy_crop_tex" texrepeat="3 2" reflectance="0.08" shininess="0.2" roughness="0.80"/>

    <!-- 3. Sunken flooded terrace water: specular sky-reflection with ripple shimmer -->
    <material name="canal_water_mat" texture="paddy_water_mirror_tex" texrepeat="4 4" reflectance="0.55" shininess="0.92" specular="0.85" rgba="0.28 0.56 0.72 0.78"/>

    <!-- 4. Sinuous earthen bund ridges (田埂) separating terrace tiers -->
    <material name="bund_earth_mat" texture="terrace_bund_earth_tex" texrepeat="4 2" roughness="0.88"/>

    <!-- 5. Aerated viable loam planting soil -->
    <material name="viable_loam_mat" texture="paddy_viable_loam_tex" texrepeat="3 3" roughness="0.92"/>

    <!-- 6. Saturated deep Lannitian mud hazard pits -->
    <material name="mud_hazard_mat" texture="paddy_mud_hazard_tex" texrepeat="3 3" reflectance="0.25" shininess="0.75" specular="0.60"/>

    <!-- 7. Traditional stacked slate & clay terrace retaining walls -->
    <material name="terrace_wall_mat" texture="terrace_wall_tex" texrepeat="4 2" roughness="0.88"/>
    <material name="mountain_rock_mat" rgba="0.30 0.42 0.32 1" roughness="0.95"/>
    <material name="deep_earth_mat" rgba="0.32 0.25 0.18 1" roughness="0.95"/>
    <material name="hillside_grass_mat" rgba="0.24 0.48 0.20 1" roughness="0.92"/>
    <material name="ramp_mat" texture="terrace_bund_earth_tex" texrepeat="6 2" rgba="0.88 0.76 0.56 1" roughness="0.92"/>
    <material name="access_path_mat" texture="terrace_bund_earth_tex" texrepeat="5 2" rgba="0.78 0.66 0.48 1" roughness="0.92"/>
    <material name="seedling_leaf_mat" rgba="0.24 0.68 0.22 1" shininess="0.3" roughness="0.7"/>

    <!-- 8. Rover rubber tire material -->
    <material name="wheel_rubber_mat" rgba="0.14 0.14 0.16 1" roughness="0.75"/>

    <!-- Micro-relief heightfield -->
    <hfield name="paddy_micro_relief" nrow="32" ncol="48" size="1.65 0.92 0.045 0.02"/>
  </asset>"""


def enhance_mjcf_with_field_textures(mjcf_xml: str, textures_dir: Path | None = None) -> str:
    """Upgrade any PaddyPatch MJCF XML string with photorealistic rice terrace textures."""
    asset_block = build_textured_asset_xml(textures_dir=textures_dir)

    # 1. Replace the existing <asset>...</asset> block
    if "<asset>" in mjcf_xml and "</asset>" in mjcf_xml:
        enhanced = re.sub(r"<asset>.*?</asset>", asset_block, mjcf_xml, flags=re.DOTALL)
    else:
        enhanced = mjcf_xml.replace("<mujoco", f"<mujoco>\n{asset_block}\n", 1)

    # 2. Assign terrace_wall_mat to retaining wall geoms
    enhanced = re.sub(
        r'(<geom\s+[^>]*?name="[^"]*(?:retaining_wall|wall)[^"]*"[^>]*?)(?:rgba="[^"]*"|material="bund_earth_mat")',
        r'\1material="terrace_wall_mat"',
        enhanced,
    )

    # 3. Assign bund_earth_mat to earthen bund embankments / crests with plain rgba
    enhanced = re.sub(
        r'(<geom\s+[^>]*?name="[^"]*bund[^"]*"[^>]*?)rgba="[^"]*"',
        r'\1material="bund_earth_mat"',
        enhanced,
    )

    # 4. Prevent coplanar hfield mesh z-fighting over textured terrace ground
    enhanced = re.sub(
        r'(<geom\s+[^>]*?name="paddy_relief_mesh"[^>]*?)rgba="[^"]*"',
        r'\1rgba="0 0 0 0"',
        enhanced,
    )

    return enhanced


_ORIGINAL_COMPOSE_FN: Callable[..., str] | None = None


def patch_adapter_for_textures() -> None:
    """Non-invasively monkey-patch `ricebot.mujoco_sim.adapter.compose_paddypatch_mjcf`.

    Ensures that any part of the application that loads the simulation automatically
    receives the photorealistic rice field textures without touching adapter.py on disk.
    """
    global _ORIGINAL_COMPOSE_FN
    import ricebot.mujoco_sim.adapter as adapter_mod

    if _ORIGINAL_COMPOSE_FN is None:
        _ORIGINAL_COMPOSE_FN = adapter_mod.compose_paddypatch_mjcf

        def _textured_compose(*args, **kwargs) -> str:
            raw = _ORIGINAL_COMPOSE_FN(*args, **kwargs)
            return enhance_mjcf_with_field_textures(raw)

        adapter_mod.compose_paddypatch_mjcf = _textured_compose
        # Also update the module-level constant if present
        if hasattr(adapter_mod, "PADDYPATCH_MJCF"):
            adapter_mod.PADDYPATCH_MJCF = enhance_mjcf_with_field_textures(adapter_mod.PADDYPATCH_MJCF)


def unpatch_adapter_for_textures() -> None:
    """Restore original compose_paddypatch_mjcf function."""
    global _ORIGINAL_COMPOSE_FN
    if _ORIGINAL_COMPOSE_FN is not None:
        import ricebot.mujoco_sim.adapter as adapter_mod
        adapter_mod.compose_paddypatch_mjcf = _ORIGINAL_COMPOSE_FN
        _ORIGINAL_COMPOSE_FN = None
