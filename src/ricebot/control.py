"""Bounded, deterministic waypoint control; Isaac owns wheel and drop physics."""
from __future__ import annotations

from dataclasses import dataclass
from math import atan2, hypot, pi

from .contracts import RobotCapability, RobotCommand, RobotObservation, Site, TaskIntent, TaskMode


def _wrap(angle: float) -> float:
    return (angle + pi) % (2 * pi) - pi


@dataclass(frozen=True)
class ControlLimits:
    speed_m_s: float = 0.25
    turn_rad_s: float = 0.75
    stop_range_m: float = 0.35
    site_reach_m: float = 0.15

    def __post_init__(self) -> None:
        if min(self.speed_m_s, self.turn_rad_s, self.stop_range_m, self.site_reach_m) <= 0:
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

    def command(self, robot: RobotObservation, task: TaskIntent,
                sites: dict[str, Site], capability: RobotCapability) -> RobotCommand:
        if robot.robot_id != task.robot_id or robot.robot_id != capability.robot_id:
            raise ValueError("actor mismatch")
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
            elif not _crosses_protected_area(robot.pose, target, sites):
                heading_error = _wrap(atan2(delta_y, delta_x) - robot.pose.yaw_rad)
                angular = max(-self.limits.turn_rad_s, min(self.limits.turn_rad_s, heading_error * 1.5))
                # Turn toward goal first; do not drive forward blindly into off-axis targets.
                if abs(heading_error) < 0.5:
                    linear = min(self.limits.speed_m_s, distance * 0.5)
        # Obstacle/invalid range -> HOLD. Planner can later provide a new safe waypoint.
        return RobotCommand(robot.robot_id, issued_at_s=robot.time_s,
                            expires_at_s=robot.time_s + 0.2, task_id=task.task_id, linear_m_s=linear,
                            angular_rad_s=angular, release_seed=release,
                            target_site_id=target_id if release else None)
