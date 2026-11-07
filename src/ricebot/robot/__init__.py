"""Robot embodiment module for PaddyPatch (`src/ricebot/robot/`).

Owns homogeneous ant-colony rover chassis definitions, 4-wheel differential
skid-steer kinematics, Bekker-Wong / Janosi-Hanamoto wheel-mud terramechanics
(`TerramechanicsSinkageState`, `evaluate_bekker_terramechanics`), sensor mount
frames, seed-hopper release nozzle geometry, combined rigid docking/cam-locking
rescue mechanics, and robot MJCF/articulation fragments.
"""
from .rovers import (
    DEFAULT_CAPABILITIES,
    PLANTER_SPEC,
    SCOUT_SPEC,
    RoverChassisSpec,
    RampVisionDetection,
    TerramechanicsSinkageState,
    build_rigid_docking_rescue_mjcf,
    build_rovers_and_seeds_xml,
    compute_4wheel_skid_steer,
    detect_ramp_and_compute_visual_servo,
    evaluate_bekker_terramechanics,
    export_robot_mjcf_assets,
    make_homogeneous_rover_spec,
    update_rigid_dock_latch,
)

__all__ = [
    "DEFAULT_CAPABILITIES",
    "PLANTER_SPEC",
    "SCOUT_SPEC",
    "RoverChassisSpec",
    "RampVisionDetection",
    "TerramechanicsSinkageState",
    "detect_ramp_and_compute_visual_servo",
    "build_rigid_docking_rescue_mjcf",
    "build_rovers_and_seeds_xml",
    "compute_4wheel_skid_steer",
    "evaluate_bekker_terramechanics",
    "export_robot_mjcf_assets",
    "make_homogeneous_rover_spec",
    "update_rigid_dock_latch",
]
