"""Multi-Level Terraced Rice-Paddy Environment & Inter-Terrace Navigation Network.

Strict engineering adherence:
1. Environment: Realistic Chinese terraced rice fields (Longji / Yuanyang style).
2. Terrace Geometry: Interconnected stepped terraces with configurable elevations:
   - Terrace A (Lower): 0.0 m
   - Terrace B (Mid):   0.5 m
   - Terrace C (Upper): 1.0 m
3. Physical Connections: Engineered access ramps with conservative 10° slope
   (0.5m rise over ~2.84m horizontal run), suitable width (0.55m for 0.276m rover),
   non-slip surface, and side protective guide curbs.
4. Navigation Network: Connected topological graph where each terrace is a navigable
   region and each ramp is an edge with length, slope, width, and accessibility status.
5. Kinematic & Physical Terrain Following: Continuous elevation z(x, y) and pitch
   orientation θ_pitch(x, y) with smooth entrance/exit transitions and zero wall clipping.
6. Field-Safe Zoning: Separates cultivated rice seedling zones, inter-row corridors,
   access paths, embankments/walls, and ramps.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Sequence


@dataclass(frozen=True)
class TerraceLevel:
    """A distinct, navigable horizontal rice terrace tier."""

    terrace_id: str
    name: str
    elevation_m: float
    bounds_x: tuple[float, float]  # (min_x, max_x)
    bounds_y: tuple[float, float]  # (min_y, max_y)
    water_depth_m: float = 0.03
    cultivated_zones: tuple[tuple[float, float, float, float], ...] = ()  # (min_x, max_x, min_y, max_y)
    inter_row_corridors: tuple[tuple[float, float, float, float], ...] = ()
    access_paths: tuple[tuple[float, float, float, float], ...] = ()
    embankments: tuple[tuple[float, float, float, float], ...] = ()

    def contains(self, x_m: float, y_m: float) -> bool:
        """Check if (x, y) lies inside terrace horizontal bounds."""
        return (
            self.bounds_x[0] <= x_m <= self.bounds_x[1]
            and self.bounds_y[0] <= y_m <= self.bounds_y[1]
        )

    def is_in_cultivated_zone(self, x_m: float, y_m: float, margin_m: float = 0.05) -> bool:
        """Return True if (x, y) is inside any planted rice crop bed."""
        for min_x, max_x, min_y, max_y in self.cultivated_zones:
            if (min_x - margin_m <= x_m <= max_x + margin_m) and (
                min_y - margin_m <= y_m <= max_y + margin_m
            ):
                return True
        return False


@dataclass(frozen=True)
class RampConnection:
    """An engineered, continuous access ramp connecting two adjacent terrace levels."""

    ramp_id: str
    name: str
    from_terrace: str  # Lower terrace id (for ascent)
    to_terrace: str  # Upper terrace id (for ascent)
    start_pos: tuple[float, float, float]  # (x, y, z) at lower entrance
    end_pos: tuple[float, float, float]  # (x, y, z) at upper exit
    width_m: float = 0.55  # Suitable for 0.276m rover with lateral clearance
    slope_deg: float = 10.0  # Conservative illustrative target ~10 degrees
    is_accessible: bool = True  # Can be disabled to simulate blocked crossings
    max_safe_slope_deg: float = 15.0  # Max slope an ant-rover can safely negotiate
    curb_height_m: float = 0.04  # Side protective curbs
    curb_width_m: float = 0.03

    @property
    def elevation_diff_m(self) -> float:
        """Vertical elevation rise from lower entrance to upper exit."""
        return self.end_pos[2] - self.start_pos[2]

    @property
    def horizontal_run_m(self) -> float:
        """Horizontal euclidean distance from start to end."""
        dx = self.end_pos[0] - self.start_pos[0]
        dy = self.end_pos[1] - self.start_pos[1]
        return math.hypot(dx, dy)

    @property
    def slope_length_m(self) -> float:
        """3D hypotenuse length of the inclined ramp surface."""
        return math.hypot(self.horizontal_run_m, self.elevation_diff_m)

    @property
    def heading_rad(self) -> float:
        """Heading angle (yaw) pointing from lower entrance to upper exit."""
        dx = self.end_pos[0] - self.start_pos[0]
        dy = self.end_pos[1] - self.start_pos[1]
        return math.atan2(dy, dx)

    def contains(self, x_m: float, y_m: float, tolerance_m: float = 0.05) -> bool:
        """Check if (x, y) lies along the ramp's corridor footprint."""
        min_x = min(self.start_pos[0], self.end_pos[0]) - tolerance_m
        max_x = max(self.start_pos[0], self.end_pos[0]) + tolerance_m
        center_y = 0.5 * (self.start_pos[1] + self.end_pos[1])
        half_w = 0.5 * self.width_m + tolerance_m
        return (min_x <= x_m <= max_x) and (center_y - half_w <= y_m <= center_y + half_w)

    def elevation_and_pitch_at(self, x_m: float, y_m: float) -> tuple[float, float]:
        """Compute the exact ramp surface elevation (z) and pitch angle (rad) at (x, y).

        Linear interpolation between lower entrance and upper exit with smooth cubic
        blending near endpoints to guarantee zero visual or physical jump.
        """
        run = self.horizontal_run_m
        if run < 1e-6:
            return (self.start_pos[2], 0.0)

        # Vector from start to (x, y) projected along ramp heading
        dx = x_m - self.start_pos[0]
        dy = y_m - self.start_pos[1]
        vx = self.end_pos[0] - self.start_pos[0]
        vy = self.end_pos[1] - self.start_pos[1]
        dot = (dx * vx + dy * vy) / (run * run)
        t = max(0.0, min(1.0, dot))

        # Smooth cubic S-curve Hermite blend: h(t) = 3t^2 - 2t^3
        smooth_t = 3.0 * (t ** 2) - 2.0 * (t ** 3)
        z = self.start_pos[2] + smooth_t * (self.end_pos[2] - self.start_pos[2])

        # Pitch angle (positive when nose-up climbing)
        ramp_pitch = math.atan2(self.elevation_diff_m, run)
        # Taper pitch smoothly to zero at the immediate boundary transitions
        pitch = ramp_pitch * (6.0 * t * (1.0 - t)) if (0.0 < t < 1.0) else 0.0
        return (round(z, 4), round(pitch, 4))


@dataclass(frozen=True)
class NavigationWaypoint:
    """3D navigation waypoint with speed limits and spatial zone tag."""

    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    pitch_rad: float
    speed_m_s: float
    zone_tag: str  # e.g., "terrace_a", "ramp_ab", "crop_corridor"
    comment: str = ""


class MultiLevelTerraceEnvironment:
    """Configurable multi-level Chinese terraced rice field environment.

    Default configuration implements:
    - Terrace A: 0.0 m (Lower)
    - Terrace B: 0.5 m (Mid)
    - Terrace C: 1.0 m (Upper)
    - Ramp AB: Connects A to B at South corridor (y = -0.75m), 10° slope, ~2.84m run
    - Ramp BC: Connects B to C at North corridor (y = +0.75m), 10° slope, ~2.84m run (Switchback)
    - Retaining Walls: Vertical 0.5m stone retaining walls off-ramp
    """

    def __init__(
        self,
        elevation_a: float = 0.0,
        elevation_b: float = 0.5,
        elevation_c: float = 1.0,
        ramp_slope_deg: float = 10.0,
        ramp_width_m: float = 0.55,
        robot_max_slope_deg: float = 15.0,
        robot_width_m: float = 0.276,
    ) -> None:
        self.elevation_a = float(elevation_a)
        self.elevation_b = float(elevation_b)
        self.elevation_c = float(elevation_c)
        self.ramp_slope_deg = float(ramp_slope_deg)
        self.ramp_width_m = float(ramp_width_m)
        self.robot_max_slope_deg = float(robot_max_slope_deg)
        self.robot_width_m = float(robot_width_m)

        # Compute horizontal run for 10-degree ramp: Δx = Δz / tan(slope)
        slope_rad = math.radians(self.ramp_slope_deg)
        dz_ab = self.elevation_b - self.elevation_a
        dz_bc = self.elevation_c - self.elevation_b
        run_ab = dz_ab / math.tan(slope_rad) if math.tan(slope_rad) > 0 else 2.84
        run_bc = dz_bc / math.tan(slope_rad) if math.tan(slope_rad) > 0 else 2.84

        # Layout boundaries along X-axis:
        # Terrace A: [-3.00, 0.00]
        # Ramp AB:   [0.00, run_ab] (e.g. [0.00, 2.84])
        # Terrace B: [run_ab, run_ab + 3.00] (e.g. [2.84, 5.84])
        # Ramp BC:   [run_ab + 3.00, run_ab + 3.00 + run_bc] (e.g. [5.84, 8.68])
        # Terrace C: [run_ab + 3.00 + run_bc, run_ab + 3.00 + run_bc + 3.20] (e.g. [8.68, 11.88])
        x0 = 0.0
        x1 = x0 + run_ab
        x2 = x1 + 3.00
        x3 = x2 + run_bc
        x4 = x3 + 3.20

        # Define Terrace A (Lower)
        self.terrace_a = TerraceLevel(
            terrace_id="terrace_a",
            name="Terrace A (Lower Field)",
            elevation_m=self.elevation_a,
            bounds_x=(-3.00, x0),
            bounds_y=(-1.60, 1.60),
            water_depth_m=0.03,
            cultivated_zones=(
                (-2.60, -0.40, 0.10, 1.30),  # North planting bed
            ),
            inter_row_corridors=(
                (-2.60, -0.40, -0.20, 0.05),
            ),
            access_paths=(
                (-2.80, -0.10, -1.30, -0.50),  # South access lane leading to Ramp AB
            ),
            embankments=(
                (-3.05, -3.00, -1.65, 1.65),  # West bund
                (-3.00, x0, 1.55, 1.65),      # North bund
                (-3.00, x0, -1.65, -1.55),    # South bund
            ),
        )

        # Define Ramp AB (South Corridor, y = -0.75m)
        self.ramp_ab = RampConnection(
            ramp_id="ramp_ab",
            name="Ramp AB (Terrace A -> B)",
            from_terrace="terrace_a",
            to_terrace="terrace_b",
            start_pos=(x0, -0.75, self.elevation_a),
            end_pos=(x1, -0.75, self.elevation_b),
            width_m=self.ramp_width_m,
            slope_deg=self.ramp_slope_deg,
            is_accessible=True,
            max_safe_slope_deg=self.robot_max_slope_deg,
        )

        # Define Terrace B (Mid)
        self.terrace_b = TerraceLevel(
            terrace_id="terrace_b",
            name="Terrace B (Mid Field)",
            elevation_m=self.elevation_b,
            bounds_x=(x1, x2),
            bounds_y=(-1.60, 1.60),
            water_depth_m=0.03,
            cultivated_zones=(
                (x1 + 0.40, x2 - 0.40, -0.30, 0.30),  # Central rice bed
            ),
            inter_row_corridors=(
                (x1 + 0.20, x2 - 0.20, -0.55, -0.35),
                (x1 + 0.20, x2 - 0.20, 0.35, 0.55),
            ),
            access_paths=(
                # Switchback diagonal corridor connecting Ramp AB exit to Ramp BC entrance
                (x1, x2, -1.30, -0.50),  # South lane
                (x1, x2, 0.50, 1.30),    # North lane
                (0.5 * (x1 + x2) - 0.4, 0.5 * (x1 + x2) + 0.4, -0.6, 0.6), # Cross-walk
            ),
            embankments=(
                (x1, x2, 1.55, 1.65),
                (x1, x2, -1.65, -1.55),
            ),
        )

        # Define Ramp BC (North Corridor, y = +0.75m - Switchback design)
        self.ramp_bc = RampConnection(
            ramp_id="ramp_bc",
            name="Ramp BC (Terrace B -> C)",
            from_terrace="terrace_b",
            to_terrace="terrace_c",
            start_pos=(x2, 0.75, self.elevation_b),
            end_pos=(x3, 0.75, self.elevation_c),
            width_m=self.ramp_width_m,
            slope_deg=self.ramp_slope_deg,
            is_accessible=True,
            max_safe_slope_deg=self.robot_max_slope_deg,
        )

        # Define Terrace C (Upper)
        self.terrace_c = TerraceLevel(
            terrace_id="terrace_c",
            name="Terrace C (Upper Field)",
            elevation_m=self.elevation_c,
            bounds_x=(x3, x4),
            bounds_y=(-1.60, 1.60),
            water_depth_m=0.03,
            cultivated_zones=(
                (x3 + 0.40, x4 - 0.40, -1.30, 0.10),  # South planting bed
            ),
            inter_row_corridors=(
                (x3 + 0.30, x4 - 0.30, 0.15, 0.45),
            ),
            access_paths=(
                (x3, x4, 0.50, 1.30),  # North access lane from Ramp BC
            ),
            embankments=(
                (x3, x4, 1.55, 1.65),
                (x3, x4, -1.65, -1.55),
                (x4, x4 + 0.05, -1.65, 1.65),  # East mountain retaining wall
            ),
        )

        self.levels: dict[str, TerraceLevel] = {
            "terrace_a": self.terrace_a,
            "terrace_b": self.terrace_b,
            "terrace_c": self.terrace_c,
        }
        self.ramps: dict[str, RampConnection] = {
            "ramp_ab": self.ramp_ab,
            "ramp_bc": self.ramp_bc,
        }

    def set_ramp_accessibility(self, ramp_id: str, accessible: bool) -> None:
        """Dynamically set ramp accessibility (used to simulate blocked crossings)."""
        if ramp_id not in self.ramps:
            raise KeyError(f"Unknown ramp '{ramp_id}'")
        orig = self.ramps[ramp_id]
        updated = RampConnection(
            ramp_id=orig.ramp_id,
            name=orig.name,
            from_terrace=orig.from_terrace,
            to_terrace=orig.to_terrace,
            start_pos=orig.start_pos,
            end_pos=orig.end_pos,
            width_m=orig.width_m,
            slope_deg=orig.slope_deg,
            is_accessible=bool(accessible),
            max_safe_slope_deg=orig.max_safe_slope_deg,
            curb_height_m=orig.curb_height_m,
            curb_width_m=orig.curb_width_m,
        )
        self.ramps[ramp_id] = updated
        if ramp_id == "ramp_ab":
            self.ramp_ab = updated
        elif ramp_id == "ramp_bc":
            self.ramp_bc = updated

    def locate_region(self, x_m: float, y_m: float) -> str:
        """Identify which terrace level or ramp corridor contains (x, y)."""
        for r_id, ramp in self.ramps.items():
            if ramp.contains(x_m, y_m):
                return r_id
        for l_id, level in self.levels.items():
            if level.contains(x_m, y_m):
                return l_id
        return "out_of_bounds"

    def get_elevation_and_pitch(self, x_m: float, y_m: float) -> tuple[float, float]:
        """Return continuous terrain surface elevation z (m) and pitch angle (rad) at (x, y).

        Guarantees:
        - Exact horizontal elevation on terrace beds
        - Continuous S-curve incline along ramp corridors with matching body pitch
        - Impassable vertical cliff height along retaining walls outside ramp corridors
        """
        # Check active ramp corridors first
        for ramp in self.ramps.values():
            if ramp.contains(x_m, y_m):
                return ramp.elevation_and_pitch_at(x_m, y_m)

        # Check terrace levels
        for level in self.levels.values():
            if level.contains(x_m, y_m):
                return (level.elevation_m, 0.0)

        # Fallback extrapolation
        if x_m < self.terrace_a.bounds_x[0]:
            return (self.terrace_a.elevation_m, 0.0)
        return (self.terrace_c.elevation_m, 0.0)

    def is_line_crossing_wall(self, p1: tuple[float, float], p2: tuple[float, float]) -> bool:
        """Check whether a line segment (p1 -> p2) penetrates retaining walls outside ramps."""
        x1, y1 = p1
        x2, y2 = p2
        # Retaining wall AB is at x = 0.0 to 2.84 outside South corridor
        # Retaining wall BC is at x = 5.84 to 8.68 outside North corridor
        wall_ab_x = self.ramp_ab.start_pos[0]
        wall_bc_x = self.ramp_bc.start_pos[0]

        # Check AB crossing
        if (x1 < wall_ab_x <= x2) or (x2 < wall_ab_x <= x1):
            # Compute intersection y
            t = (wall_ab_x - x1) / (x2 - x1) if abs(x2 - x1) > 1e-6 else 0.5
            y_cross = y1 + t * (y2 - y1)
            # Ramp corridor is at y in [-1.05, -0.45]
            if not (-1.05 <= y_cross <= -0.45):
                return True

        # Check BC crossing
        if (x1 < wall_bc_x <= x2) or (x2 < wall_bc_x <= x1):
            t = (wall_bc_x - x1) / (x2 - x1) if abs(x2 - x1) > 1e-6 else 0.5
            y_cross = y1 + t * (y2 - y1)
            # Ramp corridor is at y in [0.45, 1.05]
            if not (0.45 <= y_cross <= 1.05):
                return True

        return False

    def is_line_crossing_crop(self, p1: tuple[float, float], p2: tuple[float, float]) -> bool:
        """Check whether a line segment crosses any protected rice seedling zone."""
        # Sample segment at 10 points
        for i in range(11):
            t = i / 10.0
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            for level in self.levels.values():
                if level.is_in_cultivated_zone(x, y):
                    return True
        return False


class MultiLevelPathPlanner:
    """Multi-Level Topological & Geometric Path Planner between Chinese Rice Terraces.

    Implements:
    - Connected graph search: Terrace A <-> Ramp AB <-> Terrace B <-> Ramp BC <-> Terrace C
    - Slope safety check: Rejects crossings exceeding robot's maximum traversable slope
    - Width constraint check: Rejects crossings narrower than robot hull width
    - Blocked crossing detection: Recognizes inaccessible ramps and aborts or re-routes
    - Approach & alignment behavior: Slows down and aligns heading before entering ramp
    - Continuous ramp climbing / descending with pitch articulation
    - Crop zone protection: Routes through access lanes and corridors, avoiding crops
    """

    def __init__(self, env: MultiLevelTerraceEnvironment) -> None:
        self.env = env

    def plan_path(
        self,
        start_xy: tuple[float, float],
        goal_xy: tuple[float, float],
        robot_max_slope_deg: float = 15.0,
        robot_width_m: float = 0.276,
    ) -> list[NavigationWaypoint]:
        """Plan a complete, collision-free, multi-level route from start_xy to goal_xy."""
        start_reg = self.env.locate_region(start_xy[0], start_xy[1])
        goal_reg = self.env.locate_region(goal_xy[0], goal_xy[1])

        if start_reg == "out_of_bounds" or goal_reg == "out_of_bounds":
            raise ValueError(f"Start {start_xy} or goal {goal_xy} is out of bounds")

        # Map ramp region to nearest terrace if robot starts on a ramp
        if start_reg.startswith("ramp_"):
            start_level_id = self.env.ramps[start_reg].from_terrace
        else:
            start_level_id = start_reg

        if goal_reg.startswith("ramp_"):
            goal_level_id = self.env.ramps[goal_reg].to_terrace
        else:
            goal_level_id = goal_reg

        # 1. Same-terrace movement:
        if start_level_id == goal_level_id:
            return self._plan_intra_terrace(start_xy, goal_xy, start_level_id)

        # 2. Multi-terrace graph route finding:
        route_ramps = self._find_ramp_sequence(start_level_id, goal_level_id)
        if route_ramps is None:
            raise RuntimeError(f"No valid connected path between {start_level_id} and {goal_level_id}")

        # 3. Validate every ramp along the sequence:
        for ramp, direction in route_ramps:
            # Check accessibility (Test 3)
            if not ramp.is_accessible:
                raise RuntimeError(f"Ramp '{ramp.ramp_id}' is blocked and cannot be traversed")
            # Check slope constraint (Test 4)
            if ramp.slope_deg > robot_max_slope_deg:
                raise RuntimeError(
                    f"Ramp '{ramp.ramp_id}' slope ({ramp.slope_deg}°) exceeds robot limit ({robot_max_slope_deg}°)"
                )
            # Check width constraint
            if ramp.width_m < robot_width_m:
                raise RuntimeError(
                    f"Ramp '{ramp.ramp_id}' width ({ramp.width_m}m) is too narrow for robot ({robot_width_m}m)"
                )

        # 4. Generate sequential waypoints across terraces and ramps:
        waypoints: list[NavigationWaypoint] = []
        curr_xy = start_xy

        for ramp, direction in route_ramps:
            is_uphill = (direction == "ascent")
            ramp_in = ramp.start_pos if is_uphill else ramp.end_pos
            ramp_out = ramp.end_pos if is_uphill else ramp.start_pos

            # Approach point: 0.4m in front of ramp entrance
            dx = ramp_out[0] - ramp_in[0]
            dy = ramp_out[1] - ramp_in[1]
            h_len = math.hypot(dx, dy)
            ux = dx / h_len
            uy = dy / h_len

            approach_xy = (ramp_in[0] - 0.40 * ux, ramp_in[1] - 0.40 * uy)
            approach_yaw = math.atan2(uy, ux)

            # Intra-terrace path from curr_xy to approach_xy
            from_level = ramp.from_terrace if is_uphill else ramp.to_terrace
            sub_path = self._plan_intra_terrace(curr_xy, approach_xy, from_level)
            waypoints.extend(sub_path)

            # 1. Approach Ramp Entrance
            z_in, _ = self.env.get_elevation_and_pitch(approach_xy[0], approach_xy[1])
            waypoints.append(
                NavigationWaypoint(
                    x_m=round(approach_xy[0], 3),
                    y_m=round(approach_xy[1], 3),
                    z_m=z_in,
                    yaw_rad=round(approach_yaw, 3),
                    pitch_rad=0.0,
                    speed_m_s=0.18,  # Reduce speed for ramp approach
                    zone_tag=from_level,
                    comment=f"Approach {ramp.name} entrance",
                )
            )

            # 2. Align with ramp centerline
            waypoints.append(
                NavigationWaypoint(
                    x_m=round(ramp_in[0], 3),
                    y_m=round(ramp_in[1], 3),
                    z_m=round(ramp_in[2], 3),
                    yaw_rad=round(approach_yaw, 3),
                    pitch_rad=0.0,
                    speed_m_s=0.15,
                    zone_tag=ramp.ramp_id,
                    comment=f"Enter {ramp.name}",
                )
            )

            # 3. Continuous ramp climb/descent (sample along slope)
            ramp_pitch = math.atan2(ramp.elevation_diff_m, ramp.horizontal_run_m)
            effective_pitch = ramp_pitch if is_uphill else -ramp_pitch

            num_ramp_steps = max(3, int(ramp.horizontal_run_m / 0.60))
            for i in range(1, num_ramp_steps):
                s = i / float(num_ramp_steps)
                rx = ramp_in[0] + s * (ramp_out[0] - ramp_in[0])
                ry = ramp_in[1] + s * (ramp_out[1] - ramp_in[1])
                rz, _ = ramp.elevation_and_pitch_at(rx, ry)
                waypoints.append(
                    NavigationWaypoint(
                        x_m=round(rx, 3),
                        y_m=round(ry, 3),
                        z_m=round(rz, 3),
                        yaw_rad=round(approach_yaw, 3),
                        pitch_rad=round(effective_pitch, 3),
                        speed_m_s=0.20,
                        zone_tag=ramp.ramp_id,
                        comment=f"{'Climbing' if is_uphill else 'Descending'} {ramp.name}",
                    )
                )

            # 4. Exit Ramp and Transition to Upper/Lower Terrace
            waypoints.append(
                NavigationWaypoint(
                    x_m=round(ramp_out[0], 3),
                    y_m=round(ramp_out[1], 3),
                    z_m=round(ramp_out[2], 3),
                    yaw_rad=round(approach_yaw, 3),
                    pitch_rad=0.0,
                    speed_m_s=0.18,
                    zone_tag=ramp.ramp_id,
                    comment=f"Exit {ramp.name}",
                )
            )

            # Step onto horizontal terrace
            exit_step_xy = (ramp_out[0] + 0.35 * ux, ramp_out[1] + 0.35 * uy)
            to_level = ramp.to_terrace if is_uphill else ramp.from_terrace
            z_out, _ = self.env.get_elevation_and_pitch(exit_step_xy[0], exit_step_xy[1])
            waypoints.append(
                NavigationWaypoint(
                    x_m=round(exit_step_xy[0], 3),
                    y_m=round(exit_step_xy[1], 3),
                    z_m=z_out,
                    yaw_rad=round(approach_yaw, 3),
                    pitch_rad=0.0,
                    speed_m_s=0.22,
                    zone_tag=to_level,
                    comment=f"Transition to {to_level}",
                )
            )
            curr_xy = exit_step_xy

        # 5. Final stretch on destination terrace
        final_stretch = self._plan_intra_terrace(curr_xy, goal_xy, goal_level_id)
        waypoints.extend(final_stretch)

        return waypoints

    def _find_ramp_sequence(
        self, start_level_id: str, goal_level_id: str
    ) -> list[tuple[RampConnection, str]] | None:
        """Find ordered list of (ramp, 'ascent' | 'descent') connecting start and goal."""
        order = ["terrace_a", "terrace_b", "terrace_c"]
        if start_level_id not in order or goal_level_id not in order:
            return None

        i_start = order.index(start_level_id)
        i_goal = order.index(goal_level_id)

        if i_start < i_goal:
            # Ascent (e.g. A -> B -> C)
            result = []
            for i in range(i_start, i_goal):
                lvl_from = order[i]
                lvl_to = order[i + 1]
                ramp_id = f"ramp_{lvl_from[-1]}{lvl_to[-1]}"
                ramp = self.env.ramps.get(ramp_id)
                if ramp is None:
                    return None
                result.append((ramp, "ascent"))
            return result
        elif i_start > i_goal:
            # Descent (e.g. C -> B -> A)
            result = []
            for i in range(i_start, i_goal, -1):
                lvl_from = order[i]
                lvl_to = order[i - 1]
                ramp_id = f"ramp_{lvl_to[-1]}{lvl_from[-1]}"
                ramp = self.env.ramps.get(ramp_id)
                if ramp is None:
                    return None
                result.append((ramp, "descent"))
            return result
        return []

    def _plan_intra_terrace(
        self, start_xy: tuple[float, float], goal_xy: tuple[float, float], terrace_id: str
    ) -> list[NavigationWaypoint]:
        """Plan a field-safe path within a single terrace, respecting crop zones and access corridors."""
        terrace = self.env.levels[terrace_id]
        elev = terrace.elevation_m

        # Direct line check: does start -> goal cross protected crops or retaining walls?
        if not self.env.is_line_crossing_crop(start_xy, goal_xy) and not self.env.is_line_crossing_wall(
            start_xy, goal_xy
        ):
            dy = goal_xy[1] - start_xy[1]
            dx = goal_xy[0] - start_xy[0]
            yaw = math.atan2(dy, dx)
            return [
                NavigationWaypoint(
                    x_m=round(goal_xy[0], 3),
                    y_m=round(goal_xy[1], 3),
                    z_m=elev,
                    yaw_rad=round(yaw, 3),
                    pitch_rad=0.0,
                    speed_m_s=0.25,
                    zone_tag=terrace_id,
                    comment=f"Direct safe corridor on {terrace_id}",
                )
            ]

        # Route around crop beds via designated access lane:
        # For Terrace A: Access lane is at y = -0.75
        # For Terrace B: Access lanes are at y = -0.75 and y = +0.75
        # For Terrace C: Access lane is at y = +0.75
        waypoints: list[NavigationWaypoint] = []
        if terrace_id == "terrace_a":
            safe_y = -0.75
        elif terrace_id == "terrace_b":
            # Pick whichever lane is closer or intermediate traverse
            safe_y = -0.75 if (start_xy[1] < 0) else 0.75
        else:
            safe_y = 0.75

        # Midpoint detour through access lane
        mid1 = (start_xy[0], safe_y)
        mid2 = (goal_xy[0], safe_y)

        yaw1 = math.atan2(mid1[1] - start_xy[1], mid1[0] - start_xy[0]) if (mid1 != start_xy) else 0.0
        yaw2 = math.atan2(mid2[1] - mid1[1], mid2[0] - mid1[0]) if (mid2 != mid1) else yaw1
        yaw3 = math.atan2(goal_xy[1] - mid2[1], goal_xy[0] - mid2[0]) if (goal_xy != mid2) else yaw2

        if math.hypot(mid1[0] - start_xy[0], mid1[1] - start_xy[1]) > 0.05:
            waypoints.append(
                NavigationWaypoint(
                    x_m=round(mid1[0], 3),
                    y_m=round(mid1[1], 3),
                    z_m=elev,
                    yaw_rad=round(yaw1, 3),
                    pitch_rad=0.0,
                    speed_m_s=0.22,
                    zone_tag=terrace_id,
                    comment="Enter access corridor",
                )
            )

        if math.hypot(mid2[0] - mid1[0], mid2[1] - mid1[1]) > 0.05:
            waypoints.append(
                NavigationWaypoint(
                    x_m=round(mid2[0], 3),
                    y_m=round(mid2[1], 3),
                    z_m=elev,
                    yaw_rad=round(yaw2, 3),
                    pitch_rad=0.0,
                    speed_m_s=0.25,
                    zone_tag=terrace_id,
                    comment="Traversing access lane bypass",
                )
            )

        waypoints.append(
            NavigationWaypoint(
                x_m=round(goal_xy[0], 3),
                y_m=round(goal_xy[1], 3),
                z_m=elev,
                yaw_rad=round(yaw3, 3),
                pitch_rad=0.0,
                speed_m_s=0.22,
                zone_tag=terrace_id,
                comment=f"Reach target on {terrace_id}",
            )
        )
        return waypoints
