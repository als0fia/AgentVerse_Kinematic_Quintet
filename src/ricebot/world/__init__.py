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
from .comms import (
    ChannelStatus,
    PaddyP2PCommsChannel,
    PeerLinkStatus,
)
from .field_textures import (
    TEXTURES_DIR,
    build_textured_asset_xml,
    enhance_mjcf_with_field_textures,
    get_texture_path,
    get_textures_dir,
    patch_adapter_for_textures,
    unpatch_adapter_for_textures,
)
from .terrace import (
    DEFAULT_HIDDEN_SOIL_TRUTH,
    DEFAULT_SITES,
    UNEXPECTED_BAD_SOIL_TRUTH,
    ContinuousPaddyTerrain,
    ContinuousTerrainPoint,
    PaddyTerraceWorld,
    build_world_terrain_xml,
    compute_rice_planting_fitness,
    generate_hidden_soil_truth,
)
from .terrace_network import (
    MultiLevelPathPlanner,
    MultiLevelTerraceEnvironment,
    NavigationWaypoint,
    RampConnection,
    TerraceLevel,
)

__all__ = [
    "ChannelStatus",
    "ContinuousPaddyTerrain",
    "ContinuousTerrainPoint",
    "DEFAULT_HIDDEN_SOIL_TRUTH",
    "DEFAULT_SITES",
    "MultiLevelPathPlanner",
    "MultiLevelTerraceEnvironment",
    "NavigationWaypoint",
    "PaddyP2PCommsChannel",
    "PaddyTerraceWorld",
    "PeerLinkStatus",
    "RampConnection",
    "TEXTURES_DIR",
    "TerraceLevel",
    "UNEXPECTED_BAD_SOIL_TRUTH",
    "build_textured_asset_xml",
    "build_world_terrain_xml",
    "compute_rice_planting_fitness",
    "enhance_mjcf_with_field_textures",
    "generate_hidden_soil_truth",
    "get_texture_path",
    "get_textures_dir",
    "patch_adapter_for_textures",
    "unpatch_adapter_for_textures",
]
