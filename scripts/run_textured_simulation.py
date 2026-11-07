#!/usr/bin/env python3
"""Run PaddyPatch rice.v1 3D Simulation with Photorealistic Rice Field Textures.

Loads the authentic Longsheng/Yuanyang Chinese terraced rice field textures:
- Vibrant green rice paddy crop seedling rows
- Flooded sunken water mirrors reflecting the sky
- Sinuous golden-ochre clay bunds (田埂) with grass-topped ridges
- Ancient stone & clay terrace retaining walls
- Fertile loam and deep mud hazard pits

Non-interfering: loads textures seamlessly at runtime without altering existing repository code.
Supports all PaddyPatch arguments (--dashboard, --viewer-seconds, --scenario, --policy, etc.).
"""
from __future__ import annotations

from pathlib import Path
import sys

# Ensure src/ and repo root are on sys.path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

# Apply photorealistic field textures before initializing simulation
from ricebot.world.field_textures import patch_adapter_for_textures

patch_adapter_for_textures()

# Run the standard PaddyPatch simulator entry point
try:
    from scripts.run_mujoco_paddypatch import main
except ModuleNotFoundError:
    from run_mujoco_paddypatch import main

if __name__ == "__main__":
    raise SystemExit(main())
