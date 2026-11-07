"""World and sensor host module (`src/ricebot/world/`).

Owns:
- Tiny paddy terrace layout (6 sites: viable stand gaps, Lannitian mud hazards,
  and protected living rice crop patches).
- Hidden synthetic spatial soil/stand-gap distribution (never exposed to planner).
- Location-gated local soil probe sampler (`probe_radius_m`).
- World-only MJCF terrain/bunds/crops/obstacle fragment (zero robot chassis/wheel/hopper definitions).
"""
from .terrace import (
    DEFAULT_SITES,
    PaddyTerraceWorld,
    build_world_terrain_xml,
)

__all__ = [
    "DEFAULT_SITES",
    "PaddyTerraceWorld",
    "build_world_terrain_xml",
]
