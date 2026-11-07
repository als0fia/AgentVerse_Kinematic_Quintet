"""Robot embodiment module for Antenna (`src/ricebot/robot/`).

Owns rover chassis definitions, 4-wheel differential skid-steer kinematics,
sensor mount frames, seed-hopper release nozzle geometry, and robot MJCF/articulation
fragments.
"""
from .rovers import (
    DEFAULT_CAPABILITIES,
    PLANTER_SPEC,
    SCOUT_SPEC,
    RoverChassisSpec,
    build_rovers_and_seeds_xml,
    compute_4wheel_skid_steer,
)

__all__ = [
    "DEFAULT_CAPABILITIES",
    "PLANTER_SPEC",
    "SCOUT_SPEC",
    "RoverChassisSpec",
    "build_rovers_and_seeds_xml",
    "compute_4wheel_skid_steer",
]
