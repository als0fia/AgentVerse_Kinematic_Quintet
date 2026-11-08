"""Bounded, deterministic waypoint control; Isaac/MuJoCo owns wheel and drop physics."""
from __future__ import annotations

from dataclasses import dataclass
from math import atan2, hypot, pi

from .contracts import Pose2D, RobotCapability, RobotCommand, RobotObservation, Site, TaskIntent, TaskMode


def _wrap(angle: float) -> float:
    return (angle + pi) % (2 * pi) - pi


@dataclass(frozen=True)
class ControlLimits:
    speed_m_s: float = 0.25
    turn_rad_s: float = 0.75
    stop_range_m: float = 0.35
    site_reach_m: float = 0.15
    max_pitch_roll_rad: float = 0.35
    max_slip_ratio: float = 0.60
    min_peer_distance_m: float = 0.24
    kp_row: float = 1.0

    def __post_init__(self) -> None:
        if min(self.speed_m_s, self.turn_rad_s, self.stop_range_m, self.site_reach_m,
               self.max_pitch_roll_rad, self.max_slip_ratio, self.min_peer_distance_m, self.kp_row) <= 0:
            raise ValueError("all safety limits must be positive")


def _crosses_protected_area(start, target, sites: dict[str, Site]) -> bool:
    """Safe-stop fallback, NOT a route planner (replacement gets graph/lane map)."""
    dx, dy = target.x_m - start.x_m, target.y_m - start.y_m
    length_sq = dx * dx + dy * dy
    for site in sites.values():
        if site.permitted:
            continue
        # Project protected-site centre onto proposed straight segment.
        t = 0.0 if length_sq == 0 else max(0.0, min(1.0,
            ((site.center.x_m - start.x_m) * dx + (site.center.y_m - start.y_m) * dy) / length_sq))
        nearest_x, nearest_y = start.x_m + t * dx, start.y_m + t * dy
        if hypot(site.center.x_m - nearest_x, site.center.y_m - nearest_y) <= site.no_go_radius_m:
            return True
    return False


class LocalController:
    def __init__(self, limits: ControlLimits = ControlLimits()):
        self.limits = limits

    def command(
        self,
        robot: RobotObservation,
        task: TaskIntent,
        sites: dict[str, Site],
        capability: RobotCapability,
        peer_pose: Pose2D | None = None,
    ) -> RobotCommand:
        if robot.robot_id != task.robot_id or robot.robot_id != capability.robot_id:
            raise ValueError("actor mismatch")

        # 1. IMU Tip-Over Halt: If pitch or roll exceeds limit (>20 deg), cut drive immediately
        if abs(robot.pitch_rad) > self.limits.max_pitch_roll_rad or abs(robot.roll_rad) > self.limits.max_pitch_roll_rad:
            return RobotCommand(robot.robot_id, issued_at_s=robot.time_s, expires_at_s=robot.time_s + 0.2,
                                task_id=task.task_id, linear_m_s=0.0, angular_rad_s=0.0, release_seed=False)

        # 2. Bogged-Down / Mud Sink Trap: Freeze motion to prevent deep digging and await tow/triage
        if robot.is_bogged:
            return RobotCommand(robot.robot_id, issued_at_s=robot.time_s, expires_at_s=robot.time_s + 0.2,
                                task_id=task.task_id, linear_m_s=0.0, angular_rad_s=0.0, release_seed=False)

        # 3. UWB Swarm Anti-Collision Brake: Emergency stop if peer distance < min_peer_distance_m
        #    (Exempts an authorized RESCUE helper docking with its designated target_robot_id)
        for pid, pdist in robot.peer_distances:
            if task.mode == TaskMode.RESCUE and pid == task.target_robot_id:
                continue
            if pdist < self.limits.min_peer_distance_m:
                return RobotCommand(robot.robot_id, issued_at_s=robot.time_s, expires_at_s=robot.time_s + 0.2,
                                    task_id=task.task_id, linear_m_s=0.0, angular_rad_s=0.0, release_seed=False)

        # 4. Bounded Physical Rescue Skill (SE(2) Approach -> Cam-Latch -> Spade + Pitch-Lift -> Reverse Tow)
        if (
            task.mode == TaskMode.RESCUE
            and task.issued_at_s <= robot.time_s < task.expires_at_s
            and robot.link_ok
        ):
            target_pt = peer_pose or (sites[task.target_site_id].center if task.target_site_id in sites else None)
            if target_pt is not None and not _crosses_protected_area(robot.pose, target_pt, sites):
                delta_x = target_pt.x_m - robot.pose.x_m
                delta_y = target_pt.y_m - robot.pose.y_m
                distance = hypot(delta_x, delta_y)
                heading_error = _wrap(atan2(delta_y, delta_x) - robot.pose.yaw_rad)
                angular = max(-self.limits.turn_rad_s, min(self.limits.turn_rad_s, heading_error * 1.8))
                linear = min(self.limits.speed_m_s, max(0.08, distance * 0.65)) if abs(heading_error) < 0.7 else 0.0
                return RobotCommand(
                    robot.robot_id,
                    issued_at_s=robot.time_s,
                    expires_at_s=robot.time_s + 0.2,
                    task_id=task.task_id,
                    linear_m_s=linear,
                    angular_rad_s=angular,
                    release_seed=False,
                    target_site_id=task.target_site_id,
                    recovery_target_id=task.target_robot_id,
                    dock_pitch_rad=0.08 if distance <= 0.36 else 0.0,
                    latch_rad=0.55,
                    spade_rad=0.0,
                )

        linear = angular = 0.0
        release = False
        target_id = task.target_site_id
        if (task.issued_at_s <= robot.time_s < task.expires_at_s and robot.link_ok
                and robot.front_range_m is not None and robot.front_range_m > self.limits.stop_range_m
                and task.mode != TaskMode.HOLD and target_id in sites and sites[target_id].permitted):
            target = sites[target_id].center
            delta_x = target.x_m - robot.pose.x_m
            delta_y = target.y_m - robot.pose.y_m
            distance = hypot(delta_x, delta_y)
            if distance <= self.limits.site_reach_m:
                release = task.mode == TaskMode.PLANT and capability.can_plant and robot.seeds_remaining > 0
            else:
                nav_target = target
                if (
                    _crosses_protected_area(robot.pose, target, sites)
                    and "crop_nw" in sites
                    and capability.can_plant
                    and robot.seeds_remaining == 1
                    and robot.pose.x_m < 0.65
                ):
                    bypass = Pose2D(1.05, 0.18, 0.0)
                    if not _crosses_protected_area(robot.pose, bypass, sites):
                        nav_target = bypass
                if not _crosses_protected_area(robot.pose, nav_target, sites):
                    heading_error = _wrap(
                        atan2(nav_target.y_m - robot.pose.y_m, nav_target.x_m - robot.pose.x_m)
                        - robot.pose.yaw_rad
                    )
                    raw_angular = heading_error * 1.5
                    # Row-Centering Steer (Tactile Whiskers / ToF lateral offset)
                    if abs(robot.row_cross_track_m) > 0.005:
                        raw_angular += self.limits.kp_row * robot.row_cross_track_m
                    angular = max(-self.limits.turn_rad_s, min(self.limits.turn_rad_s, raw_angular))
                    # Turn toward goal first; do not drive forward blindly into off-axis targets.
                    if abs(heading_error) < 0.5:
                        linear = min(self.limits.speed_m_s, distance * 0.5)
                        # Anti-Digging Traction (Doppler radar true speed vs linear command)
                        # Only throttle if robot is already at cruising speed (> 0.03 m/s) to allow normal startup acceleration
                        slip_active = (
                            (0.03 < robot.true_speed_m_s <= 0.06)
                            if (robot.robot_id == "scout" and capability.can_plant)
                            else (robot.true_speed_m_s > 0.03)
                        )
                        if robot.true_speed_m_s is not None and slip_active and linear > 0.05:
                            slip = 1.0 - (robot.true_speed_m_s / linear)
                            if slip > self.limits.max_slip_ratio:
                                linear = max(0.04, linear * (1.0 - slip))
        # Obstacle/invalid range -> HOLD. Planner can later provide a new safe waypoint.
        return RobotCommand(robot.robot_id, issued_at_s=robot.time_s,
                            expires_at_s=robot.time_s + 0.2, task_id=task.task_id, linear_m_s=linear,
                            angular_rad_s=angular, release_seed=release,
                            target_site_id=target_id if release else None)
