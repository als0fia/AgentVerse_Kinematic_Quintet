"""Homogeneous "Ant-Colony" wheeled rover embodiment, kinematics, farming tools, and rigid docking/locking rescue mechanism (`src/ricebot/robot/`).

Strict separation per `docs/COMPONENTS.md` (Role 2 — Robot Design & Modelling):
- Every robot in the swarm shares an **identical homogeneous physical chassis**:
  1. 4-wheel differential paddy drivetrain (`wheel_radius_m=0.058`, `half_track_m=0.118` -> `276 mm`
     outer width to fit inside standard `300 mm` rice rows) with 6 radial herringbone grouser lugs per wheel.
  2. Full articulated farming & sensing suite on every unit:
     - Conformal seed hopper funnel + bottom seed drop chute (`_hopper_outer`, `_seed_chute_tube`)
     - Articulated vertical seed-planter dibber & soil-penetrometer plunger (`_dibber_arm`, `_dibber_j`)
       that physically plunges into the mud during soil sampling and seed planting
     - Dual counter-rotating underbelly rotary finger weeders (`_weeder_l`, `_weeder_r`, `_weeder_jl/jr`)
       with 6 star tines that spin while driving to uproot inter-row weeds
     - Precision micro-sprayer tank + 4-nozzle rear spray boom + front tactile crop-row feeler whiskers
     - BeiDou RTK dome + forward 3-ray LiDAR rangefinder fan (`_range_c`, `_range_l`, `_range_r`).
  3. **Combined Rigid Docking & Cam-Locking Rescue Mechanism (no slack cable or winch):**
     - Front active pitch-lifting docking cone + rotary cam-latch jaw (`_dock_pitch`, `_latch_j`)
     - Rear passive alignment funnel + rigid strike bar (`_rear_dock_site`)
     - Rear deployable hardpan anchor spades (`_spade_j`)
     Any unit can dynamically act as a worker (weeding/sampling/planting) or dock rigidly bumper-to-bumper
     with a stranded peer, lock the cam-latch, pitch-lift the peer's chassis to break mud suction,
     and push/pull as a rigid 8-wheel tandem train.
- NEVER owns hidden soil truth, field site coordinates, or mission planning.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, pi, sin
from pathlib import Path

try:
    import numpy as np

    def _clip(val: float, low: float, high: float) -> float:
        return float(np.clip(val, low, high))
except ImportError:
    def _clip(val: float, low: float, high: float) -> float:
        return float(max(low, min(high, val)))

from ..contracts import RobotCapability


@dataclass(frozen=True)
class RoverChassisSpec:
    """Physical and kinematic parameters for a homogeneous 4-wheel paddy swarm unit."""

    robot_id: str
    can_sample: bool
    can_plant: bool
    wheel_radius_m: float = 0.058
    half_track_m: float = 0.118
    wheel_half_width_m: float = 0.020
    chassis_half_x_m: float = 0.135
    chassis_half_y_m: float = 0.095
    chassis_half_z_m: float = 0.032
    max_wheel_rad_s: float = 14.0
    max_linear_m_s: float = 0.45
    max_angular_rad_s: float = 1.40
    initial_xyz: tuple[float, float, float] = (0.10, -0.05, 0.058)
    hopper_drop_z_m: float = 0.032
    initial_seeds: int = 2
    max_dock_pitch_rad: float = 0.45
    max_latch_rad: float = 0.65
    max_spade_rad: float = 0.85
    dock_capture_radius_m: float = 0.065

    @property
    def outer_width_m(self) -> float:
        """Total wheel-to-wheel outer width in metres (must be < 0.300 m crop row width)."""
        return 2.0 * (self.half_track_m + self.wheel_half_width_m)

    @property
    def capability(self) -> RobotCapability:
        return RobotCapability(
            robot_id=self.robot_id,
            can_sample=self.can_sample,
            can_plant=self.can_plant,
        )


def make_homogeneous_rover_spec(
    robot_id: str,
    initial_xyz: tuple[float, float, float],
    *,
    can_sample: bool = True,
    can_plant: bool = True,
    initial_seeds: int = 2,
) -> RoverChassisSpec:
    """Instantiate a homogeneous swarm unit with identical physical geometry and mechanics.

    Role differences in a particular scenario (`can_plant` or `initial_seeds`) reflect
    current payload state/mission assignment, NOT different physical hardware.
    """
    return RoverChassisSpec(
        robot_id=robot_id,
        can_sample=can_sample,
        can_plant=can_plant,
        initial_xyz=initial_xyz,
        initial_seeds=initial_seeds,
    )


# Both units share 100% identical physical chassis dimensions, wheels, farming tools, and rigid dock.
# In the P0 rice.v1 baseline fixture, `scout` starts with 0 seeds (`can_plant=False`) so it immediately
# scouts unknown patches while `planter` carries 2 seeds (`can_plant=True`).
PLANTER_SPEC = make_homogeneous_rover_spec(
    "planter",
    initial_xyz=(0.10, -0.05, 0.058),
    can_sample=True,
    can_plant=True,
    initial_seeds=2,
)

SCOUT_SPEC = make_homogeneous_rover_spec(
    "scout",
    initial_xyz=(0.10, -0.55, 0.058),
    can_sample=True,
    can_plant=False,
    initial_seeds=0,
)

DEFAULT_CAPABILITIES: tuple[RobotCapability, ...] = (
    PLANTER_SPEC.capability,
    SCOUT_SPEC.capability,
)


def compute_4wheel_skid_steer(
    spec: RoverChassisSpec,
    linear_m_s: float,
    angular_rad_s: float,
) -> tuple[float, float, float, float]:
    """Convert bounded `(linear_m_s, angular_rad_s)` into `(w_fl, w_fr, w_rl, w_rr)` in `rad/s`.

    Uses a 1.85x differential skid-steer slip factor tuned for 4-wheel rubber contact on paddy soil.
    This identical function drives MuJoCo `<velocity>` wheel servos and Isaac Sim `ArticulationAction`
    joint velocity targets.
    """
    lin = _clip(linear_m_s, -spec.max_linear_m_s, spec.max_linear_m_s)
    ang = _clip(angular_rad_s, -spec.max_angular_rad_s, spec.max_angular_rad_s)
    v_left = lin - ang * spec.half_track_m * 1.85
    v_right = lin + ang * spec.half_track_m * 1.85
    w_left = _clip(v_left / spec.wheel_radius_m, -spec.max_wheel_rad_s, spec.max_wheel_rad_s)
    w_right = _clip(v_right / spec.wheel_radius_m, -spec.max_wheel_rad_s, spec.max_wheel_rad_s)
    return (w_left, w_right, w_left, w_right)


def _build_wheel_with_grousers_xml(
    rid: str,
    suffix: str,
    pos_xyz: str,
    wr: float,
    ww: float,
    side_sign: float,
    contype: int,
    hub_rgba: str,
) -> str:
    """Build a single narrow paddy wheel with 6 radial herringbone grouser lugs and crossed hub bars."""
    lugs_xml = []
    num_lugs = 6
    lug_r = wr - 0.003
    for i in range(num_lugs):
        angle = i * (2.0 * pi / num_lugs)
        lx = lug_r * cos(angle)
        lz = lug_r * sin(angle)
        lugs_xml.append(
            f'<geom name="{rid}_lug_{suffix}_{i}" type="box" pos="{lx:.4f} 0 {lz:.4f}" '
            f'euler="0 {-angle:.4f} 0" size="0.005 {ww + 0.002:.3f} 0.009" '
            f'contype="0" conaffinity="0" rgba="0.28 0.30 0.34 1"/>'
        )
    lugs_str = "\n        ".join(lugs_xml)
    hub_y = side_sign * (ww + 0.002)
    return f"""<body name="{rid}_wheel_{suffix}" pos="{pos_xyz}">
        <joint name="{rid}_j_{suffix}" type="hinge" axis="0 1 0" damping="0.08" armature="0.005"/>
        <geom name="{rid}_w_{suffix}" type="cylinder" size="{wr:.3f} {ww:.3f}" zaxis="0 1 0" mass="0.35" material="wheel_rubber_mat" contype="{contype}" conaffinity="1" friction="1.4 0.01 0.005"/>
        <geom name="{rid}_hub_{suffix}" type="box" pos="0 {hub_y:.3f} 0" size="0.042 0.003 0.009" contype="0" conaffinity="0" rgba="{hub_rgba}"/>
        <geom name="{rid}_hub_{suffix}_b" type="box" pos="0 {hub_y:.3f} 0" size="0.009 0.003 0.042" contype="0" conaffinity="0" rgba="{hub_rgba}"/>
        {lugs_str}
      </body>"""


def _build_homogeneous_ant_rover_xml(
    spec: RoverChassisSpec,
    chassis_rgba: str,
    hub_rgba: str,
    ray_rgba: str,
    contype: int,
) -> str:
    """Build MJCF `<body name="{id}_body">` for a single homogeneous ant-colony rover.

    Every unit includes identical farming, sensing, and rescue hardware:
    1. Waterproof hull + HDPE hydrodynamic belly skid + BeiDou RTK dome
    2. Active Farming End-Effectors:
       - Conformal Seed Hopper Funnel + bottom seed tube (`_hopper_outer`, `_seed_chute_tube`)
       - Articulated Vertical Seed-Dibber & Soil-Penetrometer Plunger (`_dibber_arm`, `_dibber_j`)
         that physically plunges down into the paddy mud during soil probing and seed planting
       - Dual Counter-Rotating Underbelly Finger Weeders (`_weeder_l`, `_weeder_r`, `_weeder_jl/jr`)
         with spinning star tines that uproot inter-row weeds while driving
       - Conformal Bio-Sprayer Tank + Rear 4-Nozzle Micro-Spray Boom + Front Crop-Row Whiskers
    3. 3-ray forward LiDAR fan (`_range_c`, `_range_l`, `_range_r`) + odometry site
    4. 4 revolute open-lug paddy wheels (`_j_fl`, `_j_fr`, `_j_rl`, `_j_rr`)
    5. Combined Rigid Docking & Cam-Locking Rescue Mechanism:
       - Front pitch-lifting docking boom (`_dock_pitch`) with tapered alignment cone and
         rotary cam-locking jaw (`_latch_j`) + `_front_dock_site`
       - Rear passive alignment funnel + rigid locking strike bar + `_rear_dock_site`
       - Rear deployable hardpan anchor spades (`_spade_j`)
    """
    rid = spec.robot_id
    ix, iy, iz = spec.initial_xyz
    wr = spec.wheel_radius_m
    ww = spec.wheel_half_width_m
    ht = spec.half_track_m
    wx = 0.085  # Half-wheelbase (170 mm axle-to-axle)

    wheel_fl = _build_wheel_with_grousers_xml(rid, "fl", f"{wx:.3f} {ht:.3f} 0.0", wr, ww, +1.0, contype, hub_rgba)
    wheel_fr = _build_wheel_with_grousers_xml(rid, "fr", f"{wx:.3f} -{ht:.3f} 0.0", wr, ww, -1.0, contype, hub_rgba)
    wheel_rl = _build_wheel_with_grousers_xml(rid, "rl", f"-{wx:.3f} {ht:.3f} 0.0", wr, ww, +1.0, contype, hub_rgba)
    wheel_rr = _build_wheel_with_grousers_xml(rid, "rr", f"-{wx:.3f} -{ht:.3f} 0.0", wr, ww, -1.0, contype, hub_rgba)

    return f"""
    <!-- ========================================================================= -->
    <!-- HOMOGENEOUS SWARM UNIT: {rid.upper()} (Farming + Sensing + Rigid Dock/Lock Rescue) -->
    <!-- ========================================================================= -->
    <body name="{rid}_body" pos="{ix:.3f} {iy:.3f} {iz + 0.010:.3f}">
      <freejoint name="{rid}_free"/>
      <!-- Sealed IP68 Waterproof Hull (270 x 190 x 64 mm) + Hydrodynamic Belly Skid -->
      <geom name="{rid}_chassis" type="box" pos="0 0 0.015" size="{spec.chassis_half_x_m:.3f} {spec.chassis_half_y_m:.3f} {spec.chassis_half_z_m:.3f}" mass="2.45" contype="{contype}" conaffinity="5" rgba="{chassis_rgba}"/>
      <geom name="{rid}_belly_skid" type="box" pos="0 0 -0.019" size="0.115 0.078 0.005" mass="0.08" contype="0" conaffinity="0" rgba="0.22 0.24 0.27 1"/>
      <geom name="{rid}_deck" type="box" pos="-0.010 0 0.052" size="0.105 0.078 0.008" mass="0.10" contype="0" conaffinity="0" rgba="0.90 0.93 0.96 1"/>

      <!-- BeiDou RTK-GNSS Receiver Mast & Status Beacon -->
      <geom name="{rid}_rtk_mast" type="cylinder" pos="-0.088 0.058 0.095" size="0.007 0.036" mass="0.01" contype="0" conaffinity="0" rgba="0.82 0.84 0.88 1"/>
      <geom name="{rid}_rtk_dome" type="sphere" pos="-0.088 0.058 0.135" size="0.018" mass="0.01" contype="0" conaffinity="0" rgba="0.96 0.96 0.98 1"/>

      <!-- FARMING TOOL 1: Conformal Seed Hopper Funnel + Seed Sight-Glass + Bottom Drop Chute -->
      <geom name="{rid}_hopper_outer" type="cylinder" pos="0.034 0 0.076" size="0.042 0.024" mass="0.06" contype="0" conaffinity="0" rgba="0.95 0.74 0.12 1"/>
      <geom name="{rid}_hopper_inner" type="cylinder" pos="0.034 0 0.102" size="0.034 0.005" mass="0.01" contype="0" conaffinity="0" rgba="0.22 0.24 0.28 1"/>
      <geom name="{rid}_hopper_seeds_vis" type="ellipsoid" pos="0.034 0 0.106" size="0.028 0.028 0.008" mass="0.01" contype="0" conaffinity="0" rgba="0.98 0.86 0.18 1"/>
      <geom name="{rid}_seed_chute_tube" type="cylinder" pos="0.018 0 -0.005" size="0.014 0.026" mass="0.01" contype="0" conaffinity="0" rgba="0.78 0.80 0.84 1"/>

      <!-- FARMING TOOL 2: Articulated Vertical Seed-Planter Dibber & Soil-Probe Plunger -->
      <geom name="{rid}_probe_boom" type="cylinder" pos="-0.015 -0.048 0.082" size="0.012 0.028" mass="0.01" contype="0" conaffinity="0" rgba="0.28 0.32 0.36 1"/>
      <body name="{rid}_dibber_arm" pos="-0.015 -0.048 0.045">
        <joint name="{rid}_dibber_j" type="slide" axis="0 0 1" range="-0.052 0.010" damping="1.5" armature="0.002"/>
        <geom name="{rid}_dibber_rod" type="cylinder" pos="0 0 0.010" size="0.007 0.058" mass="0.01" contype="0" conaffinity="0" rgba="0.20 0.88 0.96 1"/>
        <geom name="{rid}_dibber_collar" type="cylinder" pos="0 0 -0.038" size="0.016 0.005" mass="0.005" contype="0" conaffinity="0" rgba="0.98 0.78 0.12 1"/>
        <geom name="{rid}_dibber_tip" type="sphere" pos="0 0 -0.048" size="0.011" mass="0.005" contype="0" conaffinity="0" rgba="0.98 0.45 0.12 1"/>
      </body>

      <!-- FARMING TOOL 3: Dual Counter-Rotating Underbelly Finger Weeders (Left & Right) -->
      <body name="{rid}_weeder_l" pos="-0.012 0.066 -0.022">
        <joint name="{rid}_weeder_jl" type="hinge" axis="0 0 1" damping="0.02" armature="0.001"/>
        <geom name="{rid}_weeder_hub_l" type="cylinder" size="0.012 0.008" mass="0.005" contype="0" conaffinity="0" rgba="0.95 0.55 0.10 1"/>
        <geom name="{rid}_weeder_tine_l1" type="box" size="0.034 0.004 0.005" contype="0" conaffinity="0" rgba="0.98 0.72 0.12 1"/>
        <geom name="{rid}_weeder_tine_l2" type="box" euler="0 0 1.047" size="0.034 0.004 0.005" contype="0" conaffinity="0" rgba="0.98 0.72 0.12 1"/>
        <geom name="{rid}_weeder_tine_l3" type="box" euler="0 0 2.094" size="0.034 0.004 0.005" contype="0" conaffinity="0" rgba="0.98 0.72 0.12 1"/>
      </body>
      <body name="{rid}_weeder_r" pos="-0.012 -0.066 -0.022">
        <joint name="{rid}_weeder_jr" type="hinge" axis="0 0 1" damping="0.02" armature="0.001"/>
        <geom name="{rid}_weeder_hub_r" type="cylinder" size="0.012 0.008" mass="0.005" contype="0" conaffinity="0" rgba="0.95 0.55 0.10 1"/>
        <geom name="{rid}_weeder_tine_r1" type="box" size="0.034 0.004 0.005" contype="0" conaffinity="0" rgba="0.98 0.72 0.12 1"/>
        <geom name="{rid}_weeder_tine_r2" type="box" euler="0 0 1.047" size="0.034 0.004 0.005" contype="0" conaffinity="0" rgba="0.98 0.72 0.12 1"/>
        <geom name="{rid}_weeder_tine_r3" type="box" euler="0 0 2.094" size="0.034 0.004 0.005" contype="0" conaffinity="0" rgba="0.98 0.72 0.12 1"/>
      </body>

      <!-- FARMING TOOL 4: Conformal Bio-Sprayer Tank, Rear 4-Nozzle Spray Boom & Front Crop Whiskers -->
      <geom name="{rid}_spray_tank" type="box" pos="-0.062 0.005 0.070" size="0.032 0.036 0.014" mass="0.03" contype="0" conaffinity="0" rgba="0.18 0.82 0.78 0.92"/>
      <geom name="{rid}_spray_boom" type="box" pos="-0.128 0 0.038" size="0.006 0.092 0.005" mass="0.01" contype="0" conaffinity="0" rgba="0.22 0.85 0.42 1"/>
      <geom name="{rid}_nozzle_l1" type="cylinder" pos="-0.128 0.075 0.026" size="0.004 0.010" contype="0" conaffinity="0" rgba="0.98 0.88 0.15 1"/>
      <geom name="{rid}_nozzle_l2" type="cylinder" pos="-0.128 0.028 0.026" size="0.004 0.010" contype="0" conaffinity="0" rgba="0.98 0.88 0.15 1"/>
      <geom name="{rid}_nozzle_r1" type="cylinder" pos="-0.128 -0.028 0.026" size="0.004 0.010" contype="0" conaffinity="0" rgba="0.98 0.88 0.15 1"/>
      <geom name="{rid}_nozzle_r2" type="cylinder" pos="-0.128 -0.075 0.026" size="0.004 0.010" contype="0" conaffinity="0" rgba="0.98 0.88 0.15 1"/>
      <!-- Front Tactile Crop-Row Alignment Whiskers -->
      <geom name="{rid}_whisker_l" type="capsule" pos="0.142 0.086 0.018" euler="0 1.57 0.42" size="0.003 0.026" contype="0" conaffinity="0" rgba="0.95 0.85 0.20 1"/>
      <geom name="{rid}_whisker_r" type="capsule" pos="0.142 -0.086 0.018" euler="0 1.57 -0.42" size="0.003 0.026" contype="0" conaffinity="0" rgba="0.95 0.85 0.20 1"/>

      <!-- Front LiDAR / Rangefinder Turret (3-ray fan: center, +18 deg left, -18 deg right) -->
      <geom name="{rid}_lidar_pod" type="cylinder" pos="0.112 0 0.056" size="0.020 0.013" mass="0.02" contype="0" conaffinity="0" rgba="0.18 0.18 0.20 1"/>
      <site name="{rid}_range_c" pos="0.140 0.00 0.050" zaxis="1 0 0" size="0.008" rgba="{ray_rgba}"/>
      <site name="{rid}_range_l" pos="0.138 0.04 0.050" zaxis="0.951 0.309 0" size="0.006" rgba="{ray_rgba}"/>
      <site name="{rid}_range_r" pos="0.138 -0.04 0.050" zaxis="0.951 -0.309 0" size="0.006" rgba="{ray_rgba}"/>
      <site name="{rid}_odom_site" pos="0 0 0" size="0.01" rgba="1 1 1 1"/>

      <!-- COMBINED RIGID DOCKING & CAM-LOCKING RESCUE MECHANISM (FRONT ACTIVE CONE + LATCH) -->
      <body name="{rid}_dock_boom" pos="0.135 0 0.010">
        <!-- Active pitch-lift joint: pries a docked stuck peer upward (+Z) to break mud suction -->
        <joint name="{rid}_dock_pitch" type="hinge" axis="0 -1 0" range="-0.20 0.45" damping="2.5" armature="0.005"/>
        <geom name="{rid}_dock_shank" type="box" pos="0.022 0 0" size="0.022 0.018 0.009" mass="0.01" contype="0" conaffinity="0" rgba="0.82 0.85 0.90 1"/>
        <geom name="{rid}_dock_cone" type="cylinder" pos="0.048 0 0" zaxis="1 0 0" size="0.016 0.015" mass="0.01" contype="0" conaffinity="0" rgba="0.96 0.42 0.14 1"/>
        <site name="{rid}_front_dock_site" pos="0.058 0 0" size="0.010" rgba="1 0.3 0.1 1"/>
        <!-- Rotary Cam-Locking Jaw inside the Docking Cone (grabs peer's rear strike bar rigidly) -->
        <body name="{rid}_latch_jaw" pos="0.048 0 0.012">
          <joint name="{rid}_latch_j" type="hinge" axis="0 1 0" range="0.0 0.65" damping="0.8" armature="0.002"/>
          <geom name="{rid}_latch_arm" type="box" pos="0.012 0 0.002" size="0.014 0.008 0.004" mass="0.003" contype="0" conaffinity="0" rgba="0.98 0.18 0.18 1"/>
          <geom name="{rid}_latch_hook" type="box" pos="0.022 0 -0.008" size="0.005 0.010 0.010" mass="0.003" contype="0" conaffinity="0" rgba="0.98 0.18 0.18 1"/>
        </body>
      </body>

      <!-- REAR PASSIVE ALIGNMENT FUNNEL & RIGID STRIKE BAR (Matches peer's Front Docking Cone) -->
      <geom name="{rid}_rear_funnel_l" type="box" pos="-0.150 0.026 0.010" euler="0 0 -0.38" size="0.020 0.005 0.015" mass="0.005" contype="0" conaffinity="0" rgba="0.78 0.82 0.88 1"/>
      <geom name="{rid}_rear_funnel_r" type="box" pos="-0.150 -0.026 0.010" euler="0 0 0.38" size="0.020 0.005 0.015" mass="0.005" contype="0" conaffinity="0" rgba="0.78 0.82 0.88 1"/>
      <geom name="{rid}_rear_strike_bar" type="cylinder" pos="-0.148 0 0.010" zaxis="0 1 0" size="0.007 0.024" mass="0.005" contype="0" conaffinity="0" rgba="0.98 0.78 0.12 1"/>
      <site name="{rid}_rear_dock_site" pos="-0.148 0 0.010" size="0.010" rgba="0.95 0.85 0.15 1"/>

      <!-- REAR DEPLOYABLE HARDPAN ANCHOR SPADES (Reaction anchor during heavy pry/pull) -->
      <body name="{rid}_spade_arm" pos="-0.132 0 -0.005">
        <joint name="{rid}_spade_j" type="hinge" axis="0 1 0" range="0.0 0.85" damping="1.2" armature="0.003"/>
        <geom name="{rid}_spade_l" type="box" pos="-0.028 0.055 -0.018" euler="0 0.55 0" size="0.028 0.010 0.004" mass="0.005" contype="0" conaffinity="0" rgba="0.90 0.22 0.22 1"/>
        <geom name="{rid}_spade_r" type="box" pos="-0.028 -0.055 -0.018" euler="0 0.55 0" size="0.028 0.010 0.004" mass="0.005" contype="0" conaffinity="0" rgba="0.90 0.22 0.22 1"/>
      </body>

      <!-- 4 Revolute Open-Lug Paddy Wheels (radius={wr:.3f}m, half-width={ww:.3f}m, outer width={spec.outer_width_m*1000:.0f}mm) -->
      {wheel_fl}
      {wheel_fr}
      {wheel_rl}
      {wheel_rr}
    </body>
"""


def build_rovers_and_seeds_xml() -> tuple[str, str, str]:
    """Return `(bodies_xml, actuators_xml, sensors_xml)` for the homogeneous swarm units and seed proxies.

    Note:
    - Wheel velocity actuators for `planter` (indices 0..3) and `scout` (indices 4..7) are placed first
      so `MujocoPaddySimulation` drives them without index shifts.
    - Rigid docking/latching/spade actuators are placed at indices 8..13.
    - Active farming tool actuators (vertical dibber plunger + counter-rotating underbelly weeders)
      are placed at indices 14..19.
    """
    planter_body = _build_homogeneous_ant_rover_xml(
        PLANTER_SPEC,
        chassis_rgba="0.16 0.46 0.86 1",
        hub_rgba="0.90 0.75 0.15 1",
        ray_rgba="1 0.2 0.2 1",
        contype=2,
    )
    scout_body = _build_homogeneous_ant_rover_xml(
        SCOUT_SPEC,
        chassis_rgba="0.18 0.72 0.36 1",
        hub_rgba="0.25 0.90 0.50 1",
        ray_rgba="0.2 1.0 0.4 1",
        contype=4,
    )

    bodies_xml = (
        planter_body
        + scout_body
        + """
    <!-- Rigid-body Seed Proxy 0 (6-DOF freejoint, carried in hopper until released) -->
    <body name="seed_proxy_0" pos="0.13 -0.05 0.18">
      <freejoint name="seed_0_free"/>
      <geom name="seed_0_geom" type="sphere" size="0.022" mass="0.03" contype="1" conaffinity="1" rgba="0.98 0.86 0.12 1"/>
    </body>

    <!-- Rigid-body Seed Proxy 1 -->
    <body name="seed_proxy_1" pos="0.13 -0.05 0.23">
      <freejoint name="seed_1_free"/>
      <geom name="seed_1_geom" type="sphere" size="0.022" mass="0.03" contype="1" conaffinity="1" rgba="0.98 0.76 0.08 1"/>
    </body>
"""
    )

    actuators_xml = """
    <!-- Planter 4-Wheel Velocity Servos (ctrl 0..3, rad/s) -->
    <velocity name="planter_act_fl" joint="planter_j_fl" kv="1.2" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <velocity name="planter_act_fr" joint="planter_j_fr" kv="1.2" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <velocity name="planter_act_rl" joint="planter_j_rl" kv="1.2" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <velocity name="planter_act_rr" joint="planter_j_rr" kv="1.2" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <!-- Scout 4-Wheel Velocity Servos (ctrl 4..7, rad/s) -->
    <velocity name="scout_act_fl" joint="scout_j_fl" kv="1.2" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <velocity name="scout_act_fr" joint="scout_j_fr" kv="1.2" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <velocity name="scout_act_rl" joint="scout_j_rl" kv="1.2" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <velocity name="scout_act_rr" joint="scout_j_rr" kv="1.2" ctrllimited="true" ctrlrange="-14.0 14.0"/>

    <!-- Homogeneous Rigid Docking, Cam-Latching & Anchor Spade Actuators (ctrl 8..13) -->
    <position name="planter_dock_pitch_act" joint="planter_dock_pitch" kp="35.0" ctrllimited="true" ctrlrange="-0.20 0.45"/>
    <position name="planter_latch_act" joint="planter_latch_j" kp="15.0" ctrllimited="true" ctrlrange="0.0 0.65"/>
    <position name="planter_spade_act" joint="planter_spade_j" kp="20.0" ctrllimited="true" ctrlrange="0.0 0.85"/>
    <position name="scout_dock_pitch_act" joint="scout_dock_pitch" kp="35.0" ctrllimited="true" ctrlrange="-0.20 0.45"/>
    <position name="scout_latch_act" joint="scout_latch_j" kp="15.0" ctrllimited="true" ctrlrange="0.0 0.65"/>
    <position name="scout_spade_act" joint="scout_spade_j" kp="20.0" ctrllimited="true" ctrlrange="0.0 0.85"/>

    <!-- Homogeneous Farming Tool Actuators: Vertical Seed/Soil Dibber Plunger & Underbelly Rotary Weeders (ctrl 14..19) -->
    <position name="planter_dibber_act" joint="planter_dibber_j" kp="30.0" ctrllimited="true" ctrlrange="-0.052 0.010"/>
    <velocity name="planter_weeder_l_act" joint="planter_weeder_jl" kv="0.15" ctrllimited="true" ctrlrange="-18.0 18.0"/>
    <velocity name="planter_weeder_r_act" joint="planter_weeder_jr" kv="0.15" ctrllimited="true" ctrlrange="-18.0 18.0"/>
    <position name="scout_dibber_act" joint="scout_dibber_j" kp="30.0" ctrllimited="true" ctrlrange="-0.052 0.010"/>
    <velocity name="scout_weeder_l_act" joint="scout_weeder_jl" kv="0.15" ctrllimited="true" ctrlrange="-18.0 18.0"/>
    <velocity name="scout_weeder_r_act" joint="scout_weeder_jr" kv="0.15" ctrllimited="true" ctrlrange="-18.0 18.0"/>
"""

    sensors_xml = """
    <rangefinder name="planter_range_c" site="planter_range_c"/>
    <rangefinder name="planter_range_l" site="planter_range_l"/>
    <rangefinder name="planter_range_r" site="planter_range_r"/>
    <rangefinder name="scout_range_c" site="scout_range_c"/>
    <rangefinder name="scout_range_l" site="scout_range_l"/>
    <rangefinder name="scout_range_r" site="scout_range_r"/>
    <framepos name="planter_pos" objtype="site" objname="planter_odom_site"/>
    <framequat name="planter_quat" objtype="site" objname="planter_odom_site"/>
    <framepos name="scout_pos" objtype="site" objname="scout_odom_site"/>
    <framequat name="scout_quat" objtype="site" objname="scout_odom_site"/>
    <!-- Docking Cone & Rear Receiver Funnel 3D Site Positions for Rigid Latch Verification -->
    <framepos name="planter_front_dock_pos" objtype="site" objname="planter_front_dock_site"/>
    <framepos name="planter_rear_dock_pos" objtype="site" objname="planter_rear_dock_site"/>
    <framepos name="scout_front_dock_pos" objtype="site" objname="scout_front_dock_site"/>
    <framepos name="scout_rear_dock_pos" objtype="site" objname="scout_rear_dock_site"/>
"""
    return bodies_xml, actuators_xml, sensors_xml


def build_rigid_docking_rescue_mjcf() -> str:
    """Build a standalone MuJoCo XML scene with 2 homogeneous ants and bidirectional rigid latch constraints.

    Includes `<equality><connect ... active="false"/></equality>` constraints between each unit's
    `front_dock_site` and the peer's `rear_dock_site`. When a rescuer drives its front docking cone
    into a stranded peer's rear funnel (`distance <= dock_capture_radius_m`) and closes its cam-latch
    (`latch_act > 0.35`), the rigid mechanical lock engages (`eq_active = 1`), allowing the rescuer
    to pitch-lift the peer's chassis upward (`dock_pitch_act > 0`) and tow/push it as a rigid train.
    """
    unit_a_spec = make_homogeneous_rover_spec("planter", initial_xyz=(0.40, 0.0, 0.058), initial_seeds=2)
    unit_b_spec = make_homogeneous_rover_spec("scout", initial_xyz=(0.82, 0.0, 0.046), initial_seeds=2)

    body_a = _build_homogeneous_ant_rover_xml(
        unit_a_spec,
        chassis_rgba="0.16 0.46 0.86 1",
        hub_rgba="0.90 0.75 0.15 1",
        ray_rgba="1 0.2 0.2 1",
        contype=2,
    )
    body_b = _build_homogeneous_ant_rover_xml(
        unit_b_spec,
        chassis_rgba="0.18 0.72 0.36 1",
        hub_rgba="0.25 0.90 0.50 1",
        ray_rgba="0.2 1.0 0.4 1",
        contype=4,
    )
    _, actuators_xml, sensors_xml = build_rovers_and_seeds_xml()

    return f"""<mujoco model="paddy_ant_rigid_dock_rescue">
  <option timestep="0.004" gravity="0 0 -9.81" integrator="implicitfast"/>
  <visual>
    <global offwidth="960" offheight="720" azimuth="110" elevation="-25"/>
    <headlight ambient="0.40 0.40 0.40" diffuse="0.70 0.70 0.68"/>
  </visual>
  <statistic center="0.60 0.0 0.12" extent="0.95"/>
  <asset>
    <texture name="sky_tex" type="skybox" builtin="gradient" rgb1="0.68 0.82 0.96" rgb2="0.18 0.32 0.52" width="256" height="256"/>
    <texture name="paddy_soil_tex" type="2d" builtin="checker" rgb1="0.36 0.29 0.21" rgb2="0.32 0.25 0.18" width="256" height="256"/>
    <material name="paddy_soil_mat" texture="paddy_soil_tex" texrepeat="6 6"/>
    <material name="mud_hazard_mat" rgba="0.22 0.16 0.11 0.92" shininess="0.85"/>
    <material name="wheel_rubber_mat" rgba="0.16 0.16 0.18 1"/>
  </asset>
  <worldbody>
    <light name="sun" pos="0.6 -0.8 2.5" dir="0 0.2 -1" diffuse="0.85 0.83 0.78"/>
    <geom name="field_ground" type="plane" size="4 4 0.1" material="paddy_soil_mat" contype="1" conaffinity="7" friction="1.4 0.01 0.005"/>
    <!-- 300 mm Rice Row Boundary Markers (Y = -0.15m and Y = +0.15m) -->
    <geom name="row_left" type="box" pos="0.60 0.15 0.004" size="1.20 0.006 0.004" rgba="0.18 0.72 0.24 0.8" contype="0" conaffinity="0"/>
    <geom name="row_right" type="box" pos="0.60 -0.15 0.004" size="1.20 0.006 0.004" rgba="0.18 0.72 0.24 0.8" contype="0" conaffinity="0"/>
    <geom name="mud_trap_pan" type="cylinder" pos="0.85 0.0 0.002" size="0.24 0.002" material="mud_hazard_mat" contype="0" conaffinity="0"/>
    {body_a}
    {body_b}
  </worldbody>
  <equality>
    <!-- Bidirectional Rigid Mechanical Latch Constraints (activated when docking cone locks into rear strike bar) -->
    <connect name="rigid_latch_planter_to_scout" site1="planter_front_dock_site" site2="scout_rear_dock_site" active="false" solref="0.008 1.0"/>
    <connect name="rigid_latch_scout_to_planter" site1="scout_front_dock_site" site2="planter_rear_dock_site" active="false" solref="0.008 1.0"/>
  </equality>
  <actuator>
    {actuators_xml}
  </actuator>
  <sensor>
    {sensors_xml}
  </sensor>
</mujoco>
"""


def update_rigid_dock_latch(
    model: object,
    data: object,
    rescuer_id: str = "planter",
    target_id: str = "scout",
    capture_radius_m: float = 0.065,
) -> bool:
    """Evaluate physical proximity between `rescuer_id` front cone and `target_id` rear funnel.

    If `latch_act` is commanded closed (`> 0.30 rad`) and the 3D Euclidean distance between
    `{rescuer_id}_front_dock_site` and `{target_id}_rear_dock_site` is within `capture_radius_m`,
    engages the MuJoCo rigid equality lock (`rigid_latch_{rescuer_id}_to_{target_id}`).
    Releasing `latch_act <= 0.10` unlocks the rigid coupling.
    """
    import mujoco

    eq_name = f"rigid_latch_{rescuer_id}_to_{target_id}"
    eq_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, eq_name)
    if eq_id < 0:
        return False

    latch_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{rescuer_id}_latch_act")
    latch_cmd = float(data.ctrl[latch_act_id]) if latch_act_id >= 0 else 0.0

    if latch_cmd <= 0.10:
        data.eq_active[eq_id] = 0
        return False

    if int(data.eq_active[eq_id]) == 1:
        return True

    front_pos = data.sensor(f"{rescuer_id}_front_dock_pos").data
    rear_pos = data.sensor(f"{target_id}_rear_dock_pos").data
    dist = hypot(
        float(front_pos[0] - rear_pos[0]),
        float(front_pos[1] - rear_pos[1]),
        float(front_pos[2] - rear_pos[2]),
    )
    if latch_cmd >= 0.30 and dist <= capture_radius_m:
        data.eq_active[eq_id] = 1
        return True
    return False


def export_robot_mjcf_assets(output_dir: str | Path | None = None) -> tuple[Path, Path]:
    """Export standalone MJCF XML files for teammates and downstream agents to load in MuJoCo/Isaac."""
    target_dir = Path(output_dir) if output_dir else Path(__file__).resolve().parent / "assets"
    target_dir.mkdir(parents=True, exist_ok=True)

    rescue_path = target_dir / "paddy_ant_rigid_rescue.xml"
    rescue_path.write_text(build_rigid_docking_rescue_mjcf().strip() + "\n", encoding="utf-8")

    # Also export single standalone homogeneous ant unit
    single_body = _build_homogeneous_ant_rover_xml(
        PLANTER_SPEC,
        chassis_rgba="0.16 0.46 0.86 1",
        hub_rgba="0.90 0.75 0.15 1",
        ray_rgba="1 0.2 0.2 1",
        contype=2,
    )
    single_xml = f"""<mujoco model="paddy_ant_homogeneous_unit">
  <option timestep="0.004" gravity="0 0 -9.81" integrator="implicitfast"/>
  <statistic center="0.10 -0.05 0.10" extent="0.45"/>
  <asset>
    <material name="wheel_rubber_mat" rgba="0.16 0.16 0.18 1"/>
  </asset>
  <worldbody>
    <light name="sun" pos="0.5 -0.5 2.0" dir="0 0 -1"/>
    <geom name="field_ground" type="plane" size="2 2 0.1" rgba="0.34 0.27 0.19 1" contype="1" conaffinity="7" friction="1.4 0.01 0.005"/>
    {single_body}
  </worldbody>
  <actuator>
    <velocity name="planter_act_fl" joint="planter_j_fl" kv="1.2" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <velocity name="planter_act_fr" joint="planter_j_fr" kv="1.2" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <velocity name="planter_act_rl" joint="planter_j_rl" kv="1.2" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <velocity name="planter_act_rr" joint="planter_j_rr" kv="1.2" ctrllimited="true" ctrlrange="-14.0 14.0"/>
    <position name="planter_dock_pitch_act" joint="planter_dock_pitch" kp="35.0" ctrllimited="true" ctrlrange="-0.20 0.45"/>
    <position name="planter_latch_act" joint="planter_latch_j" kp="15.0" ctrllimited="true" ctrlrange="0.0 0.65"/>
    <position name="planter_spade_act" joint="planter_spade_j" kp="20.0" ctrllimited="true" ctrlrange="0.0 0.85"/>
    <position name="planter_dibber_act" joint="planter_dibber_j" kp="30.0" ctrllimited="true" ctrlrange="-0.052 0.010"/>
    <velocity name="planter_weeder_l_act" joint="planter_weeder_jl" kv="0.15" ctrllimited="true" ctrlrange="-18.0 18.0"/>
    <velocity name="planter_weeder_r_act" joint="planter_weeder_jr" kv="0.15" ctrllimited="true" ctrlrange="-18.0 18.0"/>
  </actuator>
  <sensor>
    <rangefinder name="planter_range_c" site="planter_range_c"/>
    <rangefinder name="planter_range_l" site="planter_range_l"/>
    <rangefinder name="planter_range_r" site="planter_range_r"/>
    <framepos name="planter_pos" objtype="site" objname="planter_odom_site"/>
    <framequat name="planter_quat" objtype="site" objname="planter_odom_site"/>
    <framepos name="planter_front_dock_pos" objtype="site" objname="planter_front_dock_site"/>
    <framepos name="planter_rear_dock_pos" objtype="site" objname="planter_rear_dock_site"/>
  </sensor>
</mujoco>
"""
    single_path = target_dir / "paddy_ant_unit.xml"
    single_path.write_text(single_xml, encoding="utf-8")
    return single_path, rescue_path
