"""World and sensor host module (`src/ricebot/world/`).

Owns:
- Tiny paddy terrace layout (6 sites: viable stand gaps, Lannitian mud hazards,
  and protected living rice crop patches).
- Continuous 2D analytic terrain & soil-mechanics field (`ContinuousPaddyTerrain`,
  `ContinuousTerrainPoint`).
- Hidden synthetic spatial soil/stand-gap distribution (never exposed to planner).
- Location-gated local soil probe sampler (`probe_radius_m`).
- World-only MJCF terrain/bunds/crops/obstacle fragment (zero robot chassis/wheel/hopper definitions).
"""
from .terrace import (
    DEFAULT_HIDDEN_SOIL_TRUTH,
    DEFAULT_SITES,
    UNEXPECTED_BAD_SOIL_TRUTH,
    ContinuousPaddyTerrain,
    ContinuousTerrainPoint,
    PaddyTerraceWorld,
    RandomizedEpisodeSpec,
    build_world_terrain_xml,
    compute_rice_planting_fitness,
    generate_hidden_soil_truth,
    generate_randomized_episode_spec,
)

__all__ = [
    "ContinuousPaddyTerrain",
    "ContinuousTerrainPoint",
    "DEFAULT_HIDDEN_SOIL_TRUTH",
    "DEFAULT_SITES",
    "UNEXPECTED_BAD_SOIL_TRUTH",
    "PaddyTerraceWorld",
    "RandomizedEpisodeSpec",
    "build_world_terrain_xml",
    "compute_rice_planting_fitness",
    "generate_hidden_soil_truth",
    "generate_randomized_episode_spec",
]
