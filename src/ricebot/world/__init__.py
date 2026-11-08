"""World and sensor host module (`src/ricebot/world/`).

Owns:
- Level operational paddy plot layout (6 sites: viable stand gaps, Lannitian mud hazards,
  and protected living rice crop patches).
- Continuous 2D analytic terrain & soil-mechanics field (`ContinuousPaddyTerrain`,
  `ContinuousTerrainPoint`).
- Hidden synthetic spatial soil/stand-gap distribution (never exposed to planner).
- Location-gated local soil probe sampler (`probe_radius_m`) with configurable noise,
  bias, and sensor degradation/dropout windows.
- Sensor-derived `FaultEvidence` (`derive_fault_evidence`).
- World-only MJCF terrain/bunds/crops/obstacle fragment & procedural agricultural textures.
"""
from .field_textures import (
    TEXTURE_DIR,
    TEXTURE_FILENAMES,
    build_world_texture_assets_xml,
    generate_paddy_field_textures,
)
from .terrace import (
    DEFAULT_HIDDEN_SOIL_TRUTH,
    DEFAULT_SITES,
    UNEXPECTED_BAD_SOIL_TRUTH,
    ContinuousPaddyTerrain,
    ContinuousTerrainPoint,
    FaultEvidence,
    PaddyTerraceWorld,
    build_world_terrain_xml,
    compute_rice_planting_fitness,
    derive_fault_evidence,
    generate_hidden_soil_truth,
)

__all__ = [
    "ContinuousPaddyTerrain",
    "ContinuousTerrainPoint",
    "DEFAULT_HIDDEN_SOIL_TRUTH",
    "DEFAULT_SITES",
    "FaultEvidence",
    "PaddyTerraceWorld",
    "TEXTURE_DIR",
    "TEXTURE_FILENAMES",
    "UNEXPECTED_BAD_SOIL_TRUTH",
    "build_world_terrain_xml",
    "build_world_texture_assets_xml",
    "compute_rice_planting_fitness",
    "derive_fault_evidence",
    "generate_hidden_soil_truth",
    "generate_paddy_field_textures",
]
