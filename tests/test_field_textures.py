from __future__ import annotations

from pathlib import Path
import sys

# Ensure src/ is on sys.path
_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT / "src"))

import mujoco

from ricebot.mujoco_sim.adapter import MujocoPaddySimulation, compose_paddypatch_mjcf
from ricebot.world.field_textures import (
    TEXTURES_DIR,
    build_textured_asset_xml,
    enhance_mjcf_with_field_textures,
    get_texture_path,
    get_textures_dir,
    patch_adapter_for_textures,
    unpatch_adapter_for_textures,
)


def test_texture_files_exist():
    """Verify all 6 core rice terrace textures exist and have valid size."""
    expected_textures = [
        "rice_field_paddy.png",
        "paddy_water_mirror.png",
        "terrace_bund_earth.png",
        "paddy_viable_loam.png",
        "paddy_mud_hazard.png",
        "terrace_retaining_wall.png",
    ]
    t_dir = get_textures_dir()
    assert t_dir.exists(), f"Textures directory does not exist: {t_dir}"

    for tex_name in expected_textures:
        tex_path = get_texture_path(tex_name)
        assert tex_path.exists(), f"Missing texture file: {tex_path}"
        assert tex_path.stat().st_size > 1000, f"Texture file suspiciously small: {tex_path}"


def test_enhance_mjcf_asset_block():
    """Verify XML asset replacement includes all textures and materials."""
    raw_mjcf = compose_paddypatch_mjcf()
    enhanced = enhance_mjcf_with_field_textures(raw_mjcf)

    assert "rice_paddy_crop_tex" in enhanced
    assert "paddy_water_mirror_tex" in enhanced
    assert "terrace_bund_earth_tex" in enhanced
    assert "terrace_wall_mat" in enhanced


def test_mujoco_compilation_with_textures():
    """Verify that MuJoCo compiles the textured MJCF cleanly into MjModel."""
    raw_mjcf = compose_paddypatch_mjcf()
    enhanced = enhance_mjcf_with_field_textures(raw_mjcf)

    model = mujoco.MjModel.from_xml_string(enhanced)
    assert model.ntex >= 6, f"Expected at least 6 textures, got {model.ntex}"
    assert model.nmat >= 7, f"Expected at least 7 materials, got {model.nmat}"

    # Verify key materials are bound to 2D textures (stage index 1 in MuJoCo)
    soil_mat_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MATERIAL, "paddy_soil_mat")
    assert soil_mat_id >= 0
    assert any(tid >= 0 for tid in model.mat_texid[soil_mat_id]), "paddy_soil_mat must have an assigned texture"

    water_mat_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MATERIAL, "canal_water_mat")
    assert water_mat_id >= 0
    assert any(tid >= 0 for tid in model.mat_texid[water_mat_id]), "canal_water_mat must have an assigned texture"

    bund_mat_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MATERIAL, "bund_earth_mat")
    assert bund_mat_id >= 0
    assert any(tid >= 0 for tid in model.mat_texid[bund_mat_id]), "bund_earth_mat must have an assigned texture"


def test_patch_adapter_and_simulation_step():
    """Verify patch_adapter_for_textures applies textures to MujocoPaddySimulation without errors."""
    try:
        patch_adapter_for_textures()
        sim = MujocoPaddySimulation()
        assert sim.model.ntex >= 6
        # Run a single step to verify physics stability
        obs = sim.step(())
        assert obs.time_s >= 0.0
        assert len(obs.robots) >= 2
    finally:
        unpatch_adapter_for_textures()


if __name__ == "__main__":
    test_texture_files_exist()
    print("PASS: test_texture_files_exist")
    test_enhance_mjcf_asset_block()
    print("PASS: test_enhance_mjcf_asset_block")
    test_mujoco_compilation_with_textures()
    print("PASS: test_mujoco_compilation_with_textures")
    test_patch_adapter_and_simulation_step()
    print("PASS: test_patch_adapter_and_simulation_step")
    print("ALL TESTS PASSED SUCCESSFULLY!")
