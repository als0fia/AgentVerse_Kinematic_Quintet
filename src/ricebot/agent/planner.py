"""Swarm Agentic Coordination (Task Allocation + Rescue Triage / "Worth Saving" Evaluation).

Completely decoupled from both Mapping (field_map) and Traversal (controller):
- Input: ObservationFrame + teammate's field_map (MeasuredFieldMap)
- Output: tuple[TaskIntent, ...] (one SAMPLE, PLANT, or HOLD intent per robot,
  with rescue/triage reasoning embedded when a peer is stranded)
"""
from __future__ import annotations

import json
import os
import urllib.request
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from math import hypot, pi
from typing import Protocol

from ..contracts import (
    ObservationFrame,
    Pose2D,
    RobotCapability,
    RobotCommand,
    RobotObservation,
    Site,
    TaskIntent,
    TaskMode,
)
from ..mission import MeasuredFieldMap, SimpleMissionPlanner, SiteEstimate

TravelCostFn = Callable[[Pose2D, Site], float | None]


def homogeneous_capabilities(robot_ids: tuple[str, ...] | list[str]) -> tuple[RobotCapability, ...]:
    """Create identical full capabilities (can_sample=True, can_plant=True) for all robots."""
    return tuple(
        RobotCapability(robot_id=rid, can_sample=True, can_plant=True)
        for rid in robot_ids
    )


class RescueVerdict(str, Enum):
    """Agentic triage decision for whether a stranded/faulted robot is worth saving."""

    HEALTHY = "healthy"
    RESCUE_NOW = "rescue_now"  # Peer diverts immediately to save stranded robot
    DEFER_RESCUE = "defer_rescue"  # Peer finishes high-value planting first, then rescues
    ABANDON_ESCALATE = "abandon_escalate"  # Not worth risking/diverting peer; safe HOLD for human


@dataclass(frozen=True)
class RescueAssessment:
    """Cost-benefit evaluation of whether a stranded robot is worth saving."""

    stranded_robot_id: str
    verdict: RescueVerdict
    nearest_site_id: str | None
    rescue_benefit: float
    opportunity_cost: float
    hazard_risk: float
    net_rescue_value: float
    helper_robot_id: str | None = None
    reason: str = ""


@dataclass(frozen=True)
class AgentBid:
    """A single robot agent's autonomous bid for a task at a specific site."""

    robot_id: str
    mode: TaskMode
    site_id: str
    utility: float
    distance_m: float
    reason: str


@dataclass(frozen=True)
class AmbiguityEvent:
    """Discrete surprise trigger passed to an optional System-2 LLM advisor."""

    episode_id: str
    time_s: float
    kind: str  # "quality_overturned", "route_obstructed", "robot_stranded"
    affected_site_ids: tuple[str, ...]
    candidate_site_ids: tuple[str, ...]


class SupervisoryAdvisor(Protocol):
    """Optional System-2 advisor (e.g., Gemini LLM) called on discrete surprise events."""

    def advise(
        self,
        event: AmbiguityEvent,
        frame: ObservationFrame,
        estimates: dict[str, SiteEstimate],
    ) -> dict[str, tuple[TaskMode, str | None, str]] | None: ...


class GeminiSwarmAdvisor:
    """Optional System-2 LLM agent using the Gemini REST API (stdlib-only).

    Triggered ONLY on mid-episode surprises/ambiguities (including when a robot
    becomes stranded and the swarm must decide if it is worth saving).
    """

    def __init__(
        self,
        model: str = "gemini-2.5-flash",
        api_key_env: str = "GEMINI_API_KEY",
        timeout_s: float = 2.5,
    ):
        self.model = model
        self.api_key_env = api_key_env
        self.timeout_s = timeout_s

    def advise(
        self,
        event: AmbiguityEvent,
        frame: ObservationFrame,
        estimates: dict[str, SiteEstimate],
    ) -> dict[str, tuple[TaskMode, str | None, str]] | None:
        api_key = os.environ.get(self.api_key_env)
        if not api_key:
            return None
        state_summary = {
            "event": {
                "kind": event.kind,
                "affected_sites": list(event.affected_site_ids),
                "candidate_sites": list(event.candidate_site_ids),
            },
            "robots": [
                {
                    "robot_id": r.robot_id,
                    "x_m": round(r.pose.x_m, 2),
                    "y_m": round(r.pose.y_m, 2),
                    "seeds_remaining": r.seeds_remaining,
                    "front_range_m": r.front_range_m,
                    "link_ok": r.link_ok,
                }
                for r in frame.robots
            ],
            "sites": {
                sid: {
                    "moisture": est.moisture,
                    "firmness": est.firmness,
                    "stand_gap": est.stand_gap,
                    "complete": est.complete,
                    "sample_count": est.sample_count,
                }
                for sid, est in estimates.items()
                if sid in event.candidate_site_ids
            },
        }
        prompt = (
            "You are the supervisory swarm coordinator for homogeneous agricultural rice-planting robots.\n"
            "All robots can both SAMPLE (measure soil/stand) and PLANT (drop a seed if seeds_remaining > 0 "
            "and the site has complete=true with stand_gap >= 0.6, moisture in [0.25, 0.8], "
            "firmness in [0.2, 0.9]).\n"
            "If a robot is stranded (front_range_m is None or very small), evaluate whether it is worth saving "
            "(e.g. if it carries needed seeds and is near a safe site) or should HOLD for human pickup while "
            "healthy peers finish planting.\n"
            "Assign each robot a non-overlapping target_site_id and mode ('sample', 'plant', or 'hold').\n"
            "Return ONLY valid JSON of the form:\n"
            '{"assignments": {"<robot_id>": {"mode": "sample|plant|hold", "target_site_id": "<site_id>|null", "reason": "<short reason>"}}}\n'
            f"State:\n{json.dumps(state_summary)}"
        )
        url = (
            f"https://generativelanguage.googleapis.com/v1beta/models/{self.model}:generateContent"
            f"?key={api_key}"
        )
        payload = json.dumps(
            {
                "contents": [{"parts": [{"text": prompt}]}],
                "generationConfig": {"responseMimeType": "application/json", "temperature": 0.1},
            }
        ).encode("utf-8")
        req = urllib.request.Request(
            url, data=payload, headers={"Content-Type": "application/json"}, method="POST"
        )
        with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
            body = json.loads(resp.read().decode("utf-8"))
        text = body["candidates"][0]["content"]["parts"][0]["text"]
        parsed = json.loads(text).get("assignments", {})
        result: dict[str, tuple[TaskMode, str | None, str]] = {}
        for rid, item in parsed.items():
            mode = TaskMode(item["mode"])
            target = item.get("target_site_id")
            reason = str(item.get("reason", event.kind))
            result[rid] = (mode, target, reason)
        return result


class SwarmRobotAgent:
    """Autonomous decision agent for one homogeneous robot in the swarm."""

    def __init__(
        self,
        capability: RobotCapability,
        min_gap: float = 0.6,
        moisture_range: tuple[float, float] = (0.25, 0.8),
        firmness_range: tuple[float, float] = (0.2, 0.9),
        travel_cost_weight: float = 0.08,
    ):
        self.capability = capability
        self.min_gap = min_gap
        self.moisture_range = moisture_range
        self.firmness_range = firmness_range
        self.travel_cost_weight = travel_cost_weight

    def eligible_to_plant(self, estimate: SiteEstimate) -> bool:
        return bool(
            estimate.complete
            and estimate.stand_gap is not None
            and estimate.moisture is not None
            and estimate.firmness is not None
            and estimate.stand_gap >= self.min_gap
            and self.moisture_range[0] <= estimate.moisture <= self.moisture_range[1]
            and self.firmness_range[0] <= estimate.firmness <= self.firmness_range[1]
        )

    def generate_bids(
        self,
        obs: RobotObservation,
        sites: dict[str, Site],
        estimates: dict[str, SiteEstimate],
        distances: dict[str, float],
        peer_poses_with_seeds: list[Pose2D],
        replan_prefix: str = "",
    ) -> list[AgentBid]:
        """Return all valid (PLANT or SAMPLE) task bids for this robot, ranked by utility."""
        bids: list[AgentBid] = []
        for site_id, dist_m in distances.items():
            est = estimates[site_id]
            site = sites[site_id]
            confidence = float(getattr(est, "confidence", min(1.0, 0.7 + 0.1 * est.sample_count)))
            uncertainty = float(
                getattr(
                    est,
                    "uncertainty",
                    0.0 if est.complete else max(0.2, 1.0 - 0.3 * est.sample_count),
                )
            )

            # 1. Exploit Bid (PLANT): bid to plant at a confirmed eligible gap
            if (
                self.capability.can_plant
                and obs.seeds_remaining > 0
                and self.eligible_to_plant(est)
            ):
                assert est.stand_gap is not None
                benefit = 1.25 + est.stand_gap * confidence
                utility = benefit - self.travel_cost_weight * dist_m
                bids.append(
                    AgentBid(
                        robot_id=obs.robot_id,
                        mode=TaskMode.PLANT,
                        site_id=site_id,
                        utility=round(utility, 6),
                        distance_m=dist_m,
                        reason=f"{replan_prefix}confirmed eligible gap",
                    )
                )

            # 2. Explore Bid (SAMPLE): bid to sample an incomplete/unverified site
            if self.capability.can_sample and not est.complete:
                partial_gap_hint = (
                    0.35 * est.stand_gap
                    if (est.stand_gap is not None and est.stand_gap >= self.min_gap)
                    else 0.0
                )
                peer_proximity_bonus = 0.0
                if peer_poses_with_seeds:
                    nearest_peer_dist = min(
                        hypot(site.center.x_m - p.x_m, site.center.y_m - p.y_m)
                        for p in peer_poses_with_seeds
                    )
                    peer_proximity_bonus = 0.15 / (1.0 + nearest_peer_dist)
                scout_specialization = 0.15 if obs.seeds_remaining == 0 else 0.0
                raw_voi = (
                    uncertainty
                    + partial_gap_hint
                    + peer_proximity_bonus
                    + scout_specialization
                )
                utility = raw_voi - self.travel_cost_weight * dist_m
                bids.append(
                    AgentBid(
                        robot_id=obs.robot_id,
                        mode=TaskMode.SAMPLE,
                        site_id=site_id,
                        utility=round(utility, 6),
                        distance_m=dist_m,
                        reason=f"{replan_prefix}active VoI sample",
                    )
                )

        bids.sort(key=lambda b: (-b.utility, b.distance_m, b.site_id))
        return bids


class CooperativeSwarmPlanner(SimpleMissionPlanner):
    """Decentralized auction-based Swarm Coordinator + Rescue Worth-Saving Evaluator.

    Drop-in replacement for SimpleMissionPlanner inside MissionCore:
        core = MissionCore(field_map, CooperativeSwarmPlanner(caps), controller)
    """

    def __init__(
        self,
        capabilities: tuple[RobotCapability, ...],
        min_gap: float = 0.6,
        moisture_range: tuple[float, float] = (0.25, 0.8),
        firmness_range: tuple[float, float] = (0.2, 0.9),
        travel_cost_fn: TravelCostFn | None = None,
        advisor: SupervisoryAdvisor | None = None,
        travel_cost_weight: float = 0.08,
        obstacle_trigger_range_m: float = 0.35,
        blocked_cooldown_s: float = 4.0,
        stuck_ticks_threshold: int = 2,
    ):
        super().__init__(
            capabilities=capabilities,
            min_gap=min_gap,
            moisture_range=moisture_range,
            firmness_range=firmness_range,
        )
        if (
            travel_cost_weight < 0
            or obstacle_trigger_range_m <= 0
            or blocked_cooldown_s < 0
            or stuck_ticks_threshold < 1
        ):
            raise ValueError("invalid planner weights")
        self.travel_cost_fn = travel_cost_fn
        self.advisor = advisor
        self.travel_cost_weight = travel_cost_weight
        self.obstacle_trigger_range_m = obstacle_trigger_range_m
        self.blocked_cooldown_s = blocked_cooldown_s
        self.stuck_ticks_threshold = stuck_ticks_threshold
        self.agents: dict[str, SwarmRobotAgent] = {
            cap.robot_id: SwarmRobotAgent(
                capability=cap,
                min_gap=min_gap,
                moisture_range=moisture_range,
                firmness_range=firmness_range,
                travel_cost_weight=travel_cost_weight,
            )
            for cap in capabilities
        }
        self._previous_targets: dict[str, tuple[TaskMode, str | None]] = {}
        self._previous_eligible: set[str] = set()
        self._blocked_approaches: dict[tuple[str, str], float] = {}
        self._last_pose: dict[str, Pose2D] = {}
        self._last_commands: dict[str, RobotCommand] = {}
        self._last_episode: str | None = None
        self._stuck_counts: dict[str, int] = {}
        self.last_rescue_assessments: dict[str, RescueAssessment] = {}
        self.advisor_calls: int = 0
        self.advisor_fallbacks: int = 0

    def _estimate_travel_distance(self, pose: Pose2D, site: Site) -> float | None:
        if self.travel_cost_fn is not None:
            return self.travel_cost_fn(pose, site)
        return hypot(pose.x_m - site.center.x_m, pose.y_m - site.center.y_m)

    def record_commands(self, commands: tuple[RobotCommand, ...]) -> None:
        """Keep *issued* movement evidence for the next observation frame.

        An active SAMPLE/PLANT task does not prove the controller commanded
        motion: obstacle avoidance, seed release, missing sensors and safe HOLD
        can all correctly produce zero wheel commands.
        """
        self._last_commands = {
            command.robot_id: command
            for command in commands
            if command.robot_id in self.capabilities
        }

    def _update_stranded_tracking(self, frame: ObservationFrame) -> set[str]:
        """Conservative incident detection; never treat a missing range as stuck.

        Detect a genuine commanded-motion mismatch over repeated valid frames.
        A physics adapter may also report an explicit synthetic is_bogged
        observation. The high-level mission should still independently test
        actual wheel/pose evidence before claiming physical rescue success.
        """
        if frame.episode_id != self._last_episode:
            self._last_episode = frame.episode_id
            self._last_pose.clear()
            self._stuck_counts.clear()
            self._last_commands.clear()

        stranded: set[str] = set()
        for robot in frame.robots:
            previous_pose = self._last_pose.get(robot.robot_id)
            previous_command = self._last_commands.get(robot.robot_id)
            displacement_m = (
                hypot(
                    robot.pose.x_m - previous_pose.x_m,
                    robot.pose.y_m - previous_pose.y_m,
                )
                if previous_pose is not None else float("inf")
            )
            yaw_delta_rad = (
                abs((robot.pose.yaw_rad - previous_pose.yaw_rad + pi) % (2 * pi) - pi)
                if previous_pose is not None else float("inf")
            )
            actively_commanded = (
                previous_command is not None
                and previous_command.issued_at_s < robot.time_s
                and robot.time_s <= previous_command.expires_at_s + 0.05
                and (
                    abs(previous_command.linear_m_s) >= 0.04
                    or abs(previous_command.angular_rad_s) >= 0.10
                )
            )
            # An obstacle or failed sensor may properly cause a safety stop.
            # The controller must not call that an immobilisation incident.
            clear_range = (
                robot.front_range_m is not None
                and robot.front_range_m > self.obstacle_trigger_range_m
            )
            motion_failed = (
                actively_commanded and clear_range and robot.link_ok
                and displacement_m < 0.005 and yaw_delta_rad < 0.02
            )
            if motion_failed:
                self._stuck_counts[robot.robot_id] = (
                    self._stuck_counts.get(robot.robot_id, 0) + 1
                )
            else:
                self._stuck_counts[robot.robot_id] = 0
            self._last_pose[robot.robot_id] = robot.pose

            if robot.is_bogged or self._stuck_counts[robot.robot_id] >= self.stuck_ticks_threshold:
                stranded.add(robot.robot_id)
        return stranded

    def evaluate_rescue_worth(
        self,
        stranded_robot: RobotObservation,
        healthy_peers: list[RobotObservation],
        sites: dict[str, Site],
        estimates: dict[str, SiteEstimate],
        allowed_site_ids: list[str],
    ) -> RescueAssessment:
        """Agentic triage: calculate whether a stranded robot is WORTH SAVING right now.

        Weighs:
        1. Rescue Benefit:
           - Stranded Seed Value: Does the stranded robot carry seeds that healthy
             peers lack to finish the remaining field gaps?
           - Unshared Data Value: Did it lose comms (link_ok=False) so a physical
             rendezvous recovers local samples?
           - Base Hardware Recovery Value: Base value of assisting a recoverable peer.
        2. Hazard Risk:
           - Is the stranded robot inside/near a protected no-go zone (permitted=False)
             where sending a helper risks losing a second robot?
        3. Opportunity Cost:
           - Can a healthy peer plant a confirmed stand gap right now, plus travel distance
             to the stranded robot?
        """
        # Find nearest permitted site to the stranded robot's location (rendezvous point)
        permitted_sites = [sites[sid] for sid in allowed_site_ids if sid in sites]
        nearest_site = (
            min(
                permitted_sites,
                key=lambda s: hypot(
                    s.center.x_m - stranded_robot.pose.x_m,
                    s.center.y_m - stranded_robot.pose.y_m,
                ),
            )
            if permitted_sites
            else None
        )

        # Check hazard risk (is stranded robot inside or right next to a no-go area?)
        hazard_risk = 0.0
        for s in sites.values():
            if not s.permitted:
                dist_to_hazard = hypot(
                    stranded_robot.pose.x_m - s.center.x_m,
                    stranded_robot.pose.y_m - s.center.y_m,
                )
                if dist_to_hazard <= s.no_go_radius_m + 0.25:
                    hazard_risk = 2.0  # High risk of losing helper robot too

        if not healthy_peers or nearest_site is None or hazard_risk >= 1.5:
            return RescueAssessment(
                stranded_robot_id=stranded_robot.robot_id,
                verdict=RescueVerdict.ABANDON_ESCALATE,
                nearest_site_id=nearest_site.site_id if nearest_site else None,
                rescue_benefit=0.0,
                opportunity_cost=0.0,
                hazard_risk=hazard_risk,
                net_rescue_value=-hazard_risk,
                helper_robot_id=None,
                reason="not worth saving: high hazard risk or no available helper; safe HOLD for human",
            )

        # Pick closest healthy peer as candidate rescuer
        helper = min(
            healthy_peers,
            key=lambda p: hypot(
                p.pose.x_m - stranded_robot.pose.x_m,
                p.pose.y_m - stranded_robot.pose.y_m,
            ),
        )
        rescue_dist = hypot(
            helper.pose.x_m - stranded_robot.pose.x_m,
            helper.pose.y_m - stranded_robot.pose.y_m,
        )

        # Count how many remaining candidate gaps exist vs healthy peer seeds
        healthy_seeds = sum(p.seeds_remaining for p in healthy_peers)
        remaining_work_sites = len(allowed_site_ids)
        confirmed_gaps = sum(1 for sid in allowed_site_ids if self._eligible(estimates[sid]))

        # 1. Rescue Benefit calculation
        seeds_needed_from_stranded = max(
            0, min(stranded_robot.seeds_remaining, remaining_work_sites - healthy_seeds)
        )
        seed_recovery_value = 0.95 * seeds_needed_from_stranded
        offline_data_value = 0.55 if not stranded_robot.link_ok else 0.0
        base_asset_value = 0.35 if stranded_robot.seeds_remaining > 0 else 0.10
        rescue_benefit = round(seed_recovery_value + offline_data_value + base_asset_value, 4)

        # 2. Opportunity Cost calculation (what does the helper give up by diverting now?)
        helper_can_plant_now = helper.seeds_remaining > 0 and confirmed_gaps > 0
        active_planting_cost = 1.45 if helper_can_plant_now else 0.25
        travel_cost = self.travel_cost_weight * rescue_dist
        opportunity_cost = round(active_planting_cost + travel_cost, 4)

        net_value = round(rescue_benefit - opportunity_cost - hazard_risk, 4)

        if net_value > 0.0:
            return RescueAssessment(
                stranded_robot_id=stranded_robot.robot_id,
                verdict=RescueVerdict.RESCUE_NOW,
                nearest_site_id=nearest_site.site_id,
                rescue_benefit=rescue_benefit,
                opportunity_cost=opportunity_cost,
                hazard_risk=hazard_risk,
                net_rescue_value=net_value,
                helper_robot_id=helper.robot_id,
                reason=f"worth saving now (net={net_value:+.2f}): carries needed seeds/data",
            )
        if rescue_benefit >= 0.60 and helper_can_plant_now:
            return RescueAssessment(
                stranded_robot_id=stranded_robot.robot_id,
                verdict=RescueVerdict.DEFER_RESCUE,
                nearest_site_id=nearest_site.site_id,
                rescue_benefit=rescue_benefit,
                opportunity_cost=opportunity_cost,
                hazard_risk=hazard_risk,
                net_rescue_value=net_value,
                helper_robot_id=helper.robot_id,
                reason=f"defer rescue (net={net_value:+.2f}): finish high-value planting first",
            )
        return RescueAssessment(
            stranded_robot_id=stranded_robot.robot_id,
            verdict=RescueVerdict.ABANDON_ESCALATE,
            nearest_site_id=nearest_site.site_id,
            rescue_benefit=rescue_benefit,
            opportunity_cost=opportunity_cost,
            hazard_risk=hazard_risk,
            net_rescue_value=net_value,
            helper_robot_id=None,
            reason=f"not worth saving (net={net_value:+.2f}): healthy peers can complete mission; escalate to human",
        )

    def _detect_surprise(
        self,
        frame: ObservationFrame,
        currently_eligible: set[str],
        newly_blocked_robots: list[str],
        stranded_robots: set[str],
        candidates: list[str],
    ) -> AmbiguityEvent | None:
        overturned = sorted(self._previous_eligible - currently_eligible)
        if overturned:
            return AmbiguityEvent(
                episode_id=frame.episode_id,
                time_s=frame.time_s,
                kind="quality_overturned",
                affected_site_ids=tuple(overturned),
                candidate_site_ids=tuple(candidates),
            )
        if stranded_robots:
            return AmbiguityEvent(
                episode_id=frame.episode_id,
                time_s=frame.time_s,
                kind="robot_stranded",
                affected_site_ids=(),
                candidate_site_ids=tuple(candidates),
            )
        if newly_blocked_robots:
            affected = [
                self._previous_targets[rid][1]
                for rid in newly_blocked_robots
                if rid in self._previous_targets and self._previous_targets[rid][1] is not None
            ]
            return AmbiguityEvent(
                episode_id=frame.episode_id,
                time_s=frame.time_s,
                kind="route_obstructed",
                affected_site_ids=tuple(s for s in affected if s is not None),
                candidate_site_ids=tuple(candidates),
            )
        return None

    def _try_advisor(
        self,
        event: AmbiguityEvent,
        frame: ObservationFrame,
        estimates: dict[str, SiteEstimate],
        allowed_sites: set[str],
    ) -> dict[str, tuple[TaskMode, str | None, str]] | None:
        if self.advisor is None:
            return None
        if any(not r.link_ok for r in frame.robots):
            self.advisor_fallbacks += 1
            return None
        try:
            self.advisor_calls += 1
            proposal = self.advisor.advise(event, frame, estimates)
        except Exception:
            self.advisor_fallbacks += 1
            return None
        if not proposal:
            return None

        validated: dict[str, tuple[TaskMode, str | None, str]] = {}
        used_sites: set[str] = set()
        obs_by_id = {r.robot_id: r for r in frame.robots}
        for robot_id, (mode, target_id, reason) in proposal.items():
            if robot_id not in self.capabilities or robot_id not in obs_by_id:
                self.advisor_fallbacks += 1
                return None
            cap = self.capabilities[robot_id]
            obs = obs_by_id[robot_id]
            if mode == TaskMode.HOLD:
                validated[robot_id] = (TaskMode.HOLD, None, f"advisor:{reason or event.kind}")
                continue
            if (
                target_id is None
                or target_id not in allowed_sites
                or target_id in used_sites
            ):
                self.advisor_fallbacks += 1
                return None
            if mode == TaskMode.PLANT:
                if (
                    not cap.can_plant
                    or obs.seeds_remaining <= 0
                    or not self._eligible(estimates[target_id])
                ):
                    self.advisor_fallbacks += 1
                    return None
            elif mode == TaskMode.SAMPLE and not cap.can_sample:
                self.advisor_fallbacks += 1
                return None
            used_sites.add(target_id)
            validated[robot_id] = (mode, target_id, f"advisor:{reason or event.kind}")
        return validated

    def decide(
        self,
        frame: ObservationFrame,
        field_map: MeasuredFieldMap,
        excluded_sites: frozenset[str] = frozenset(),
    ) -> tuple[TaskIntent, ...]:
        # 1. Detect blocked approaches and stranded robots
        self._blocked_approaches = {
            k: exp_t for k, exp_t in self._blocked_approaches.items() if exp_t > frame.time_s
        }
        newly_blocked_robots: list[str] = []
        for robot in frame.robots:
            if (
                robot.front_range_m is not None
                and robot.front_range_m <= self.obstacle_trigger_range_m
            ):
                newly_blocked_robots.append(robot.robot_id)
                prev = self._previous_targets.get(robot.robot_id)
                if prev is not None and prev[1] is not None:
                    self._blocked_approaches[(robot.robot_id, prev[1])] = (
                        frame.time_s + self.blocked_cooldown_s
                    )
        stranded_ids = self._update_stranded_tracking(frame)

        # 2. Query the mapping teammate's field_map.estimate(site_id)
        estimates: dict[str, SiteEstimate] = {
            site_id: field_map.estimate(site_id) for site_id in field_map.sites
        }
        allowed_site_ids = [
            site_id
            for site_id, site in field_map.sites.items()
            if site.permitted and site_id not in excluded_sites
        ]
        currently_eligible = {
            site_id for site_id in allowed_site_ids if self._eligible(estimates[site_id])
        }

        # 3. Evaluate whether any stranded robot is WORTH SAVING
        robots_sorted: list[RobotObservation] = sorted(frame.robots, key=lambda r: r.robot_id)
        healthy_robots = [r for r in robots_sorted if r.robot_id not in stranded_ids]
        self.last_rescue_assessments = {}
        rescue_assignments: dict[str, AgentBid] = {}
        assigned_sites: set[str] = set()

        for robot in robots_sorted:
            if robot.robot_id in stranded_ids:
                assessment = self.evaluate_rescue_worth(
                    stranded_robot=robot,
                    healthy_peers=healthy_robots,
                    sites=field_map.sites,
                    estimates=estimates,
                    allowed_site_ids=allowed_site_ids,
                )
                self.last_rescue_assessments[robot.robot_id] = assessment
                # Stranded robot itself enters safe HOLD with its triage verdict in reason
                rescue_assignments[robot.robot_id] = AgentBid(
                    robot_id=robot.robot_id,
                    mode=TaskMode.HOLD,
                    site_id="",
                    utility=1000.0,
                    distance_m=0.0,
                    reason=f"stranded({assessment.verdict.value}): {assessment.reason}",
                )
                # If verdict is RESCUE_NOW, dispatch the designated helper robot to the rendezvous site
                if (
                    assessment.verdict == RescueVerdict.RESCUE_NOW
                    and assessment.helper_robot_id is not None
                    and assessment.nearest_site_id is not None
                    and assessment.helper_robot_id not in rescue_assignments
                    and assessment.nearest_site_id not in assigned_sites
                ):
                    rescue_assignments[assessment.helper_robot_id] = AgentBid(
                        robot_id=assessment.helper_robot_id,
                        mode=TaskMode.SAMPLE,
                        site_id=assessment.nearest_site_id,
                        utility=950.0,
                        distance_m=0.0,
                        reason=f"rescue_assist({robot.robot_id}): {assessment.reason}",
                    )
                    assigned_sites.add(assessment.nearest_site_id)

        # 4. Check for mid-episode surprise and optional System-2 LLM advisor
        surprise = self._detect_surprise(
            frame,
            currently_eligible,
            newly_blocked_robots,
            stranded_ids,
            allowed_site_ids,
        )
        advisor_override = (
            self._try_advisor(surprise, frame, estimates, set(allowed_site_ids))
            if surprise is not None
            else None
        )

        # 5. Collect autonomous bids from remaining healthy robots
        peer_poses_with_seeds = [
            r.pose
            for r in healthy_robots
            if self.capabilities[r.robot_id].can_plant and r.seeds_remaining > 0
        ]

        all_bids: list[AgentBid] = []
        winning_bids: dict[str, AgentBid] = dict(rescue_assignments)

        for robot in robots_sorted:
            if advisor_override and robot.robot_id in advisor_override:
                mode, target_id, reason = advisor_override[robot.robot_id]
                if target_id is not None:
                    assigned_sites.add(target_id)
                winning_bids[robot.robot_id] = AgentBid(
                    robot_id=robot.robot_id,
                    mode=mode,
                    site_id=target_id or "",
                    utility=999.0,
                    distance_m=0.0,
                    reason=reason,
                )
                continue
            if robot.robot_id in winning_bids:
                continue

            prev_mode, prev_target = self._previous_targets.get(
                robot.robot_id, (TaskMode.HOLD, None)
            )
            replan_prefix = ""
            if (
                prev_mode == TaskMode.PLANT
                and prev_target is not None
                and prev_target not in currently_eligible
                and prev_target not in excluded_sites
            ):
                replan_prefix = f"replan(site {prev_target} quality overturned): "
            elif robot.robot_id in newly_blocked_robots:
                replan_prefix = "replan(route obstructed): "

            all_distances: dict[str, float] = {}
            unblocked_distances: dict[str, float] = {}
            for site_id in allowed_site_ids:
                dist = self._estimate_travel_distance(robot.pose, field_map.sites[site_id])
                if dist is not None:
                    all_distances[site_id] = dist
                    if (robot.robot_id, site_id) not in self._blocked_approaches:
                        unblocked_distances[site_id] = dist
            distances = unblocked_distances if unblocked_distances else all_distances

            agent = self.agents[robot.robot_id]
            robot_bids = agent.generate_bids(
                obs=robot,
                sites=field_map.sites,
                estimates=estimates,
                distances=distances,
                peer_poses_with_seeds=peer_poses_with_seeds,
                replan_prefix=replan_prefix,
            )
            all_bids.extend(robot_bids)

        # 6. Resolve Decentralized Swarm Auction
        all_bids.sort(key=lambda b: (-b.utility, b.distance_m, b.site_id, b.robot_id))
        for bid in all_bids:
            if bid.robot_id in winning_bids or bid.site_id in assigned_sites:
                continue
            winning_bids[bid.robot_id] = bid
            assigned_sites.add(bid.site_id)

        # 7. Emit final TaskIntents for each robot
        intents: list[TaskIntent] = []
        for robot in robots_sorted:
            win = winning_bids.get(robot.robot_id)
            if win is not None and win.mode != TaskMode.HOLD and win.site_id:
                mode, target_id, reason = win.mode, win.site_id, win.reason
            elif win is not None and win.mode == TaskMode.HOLD:
                mode, target_id, reason = TaskMode.HOLD, None, win.reason
            else:
                prev_mode, prev_target = self._previous_targets.get(
                    robot.robot_id, (TaskMode.HOLD, None)
                )
                replan_prefix = ""
                if (
                    prev_mode == TaskMode.PLANT
                    and prev_target is not None
                    and prev_target not in currently_eligible
                    and prev_target not in excluded_sites
                ):
                    replan_prefix = f"replan(site {prev_target} quality overturned): "
                elif robot.robot_id in newly_blocked_robots:
                    replan_prefix = "replan(route obstructed): "
                mode = TaskMode.HOLD
                target_id = None
                reason = f"{replan_prefix}no validated accessible work"

            self._previous_targets[robot.robot_id] = (mode, target_id)
            intents.append(
                TaskIntent(
                    task_id=f"{frame.episode_id}:{frame.time_s:.3f}:{robot.robot_id}:{mode.value}",
                    robot_id=robot.robot_id,
                    mode=mode,
                    target_site_id=target_id,
                    issued_at_s=frame.time_s,
                    expires_at_s=frame.time_s + 1.0,
                    reason=reason,
                )
            )

        self._previous_eligible = currently_eligible
        return tuple(intents)
