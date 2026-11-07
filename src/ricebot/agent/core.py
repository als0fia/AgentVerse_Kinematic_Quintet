"""Homogeneous Agentic Swarm & Edge Coordination (src/ricebot/agent/).

Tier 1: On-Rover Micro-Edge (SwarmRobotAgent + Decentralized Auction)
- Every rover runs its own SwarmRobotAgent evaluating Explore (SAMPLE) vs Exploit (PLANT)
- Decentralized auction resolves task bids without duplicate sites
- Hard Safety Verifier rejects unsafe AI hallucinations and provides instant fallback
"""
from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Callable, Iterable

from ..contracts import (
    ObservationFrame,
    Pose2D,
    RobotCapability,
    RobotObservation,
    Site,
    TaskIntent,
    TaskMode,
)
from ..mission import MeasuredFieldMap, SiteEstimate
from .advisor import GeminiSwarmAdvisor


def homogeneous_capabilities(robot_ids: Iterable[str]) -> tuple[RobotCapability, ...]:
    """Generates identical hardware capabilities for every rover in the swarm.

    Both soil/stand sensors (can_sample=True) and seed hopper (can_plant=True).
    Nobody is hardcoded as 'Scout' or 'Planter.' Roles emerge dynamically on
    every decision cycle based on each robot's position, remaining seeds, and the shared map.
    """
    ids = list(robot_ids)
    if not ids or len(set(ids)) != len(ids):
        raise ValueError("nonempty unique robot IDs required")
    return tuple(
        RobotCapability(robot_id=rid, can_sample=True, can_plant=True)
        for rid in ids
    )


@dataclass(frozen=True)
class AgentBid:
    """Individual rover task utility bid generated onboard (Tier 1)."""
    robot_id: str
    site_id: str
    mode: TaskMode
    utility: float
    reason: str = ""


class SwarmRobotAgent:
    """Micro-edge agent running onboard every homogeneous rover (Tier 1 System 1).

    Evaluates Explore (SAMPLE) vs Exploit (PLANT) utility bids locally in <1 ms.
    """

    def __init__(
        self,
        capability: RobotCapability,
        min_gap: float = 0.6,
        moisture_range: tuple[float, float] = (0.25, 0.8),
        firmness_range: tuple[float, float] = (0.2, 0.9),
        gap_weight: float = 2.0,
        dist_weight: float = 1.0,
        coop_radius_m: float = 3.0,
    ):
        self.capability = capability
        self.min_gap = min_gap
        self.moisture_range = moisture_range
        self.firmness_range = firmness_range
        self.gap_weight = gap_weight
        self.dist_weight = dist_weight
        self.coop_radius_m = coop_radius_m

    def is_eligible(self, estimate: SiteEstimate) -> bool:
        """Site is confirmed eligible only if completely sampled and within envelope."""
        return bool(
            estimate.complete
            and estimate.stand_gap is not None
            and estimate.stand_gap >= self.min_gap
            and estimate.moisture is not None
            and self.moisture_range[0] <= estimate.moisture <= self.moisture_range[1]
            and estimate.firmness is not None
            and self.firmness_range[0] <= estimate.firmness <= self.firmness_range[1]
        )

    def compute_bids(
        self,
        robot_obs: RobotObservation,
        field_map: MeasuredFieldMap,
        candidate_sites: list[Site],
        teammates: list[RobotObservation],
        travel_cost_fn: Callable[[Pose2D, Site], float | None] | None = None,
    ) -> list[AgentBid]:
        """Computes utility bids for candidate sites.

        Exploit (PLANT): High utility if site is confirmed eligible and rover has seeds.
        Explore (SAMPLE): Value-of-Information (VoI) for unconfirmed sites + cooperative proximity bonus.
        """
        bids: list[AgentBid] = []

        for site in candidate_sites:
            if not site.permitted:
                continue

            estimate = field_map.estimate(site.site_id)

            # Travel distance / cost (uses real lane path distance if provided by traversal teammate)
            cost: float | None = None
            if travel_cost_fn is not None:
                cost = travel_cost_fn(robot_obs.pose, site)
            if cost is None:
                cost = hypot(
                    robot_obs.pose.x_m - site.center.x_m,
                    robot_obs.pose.y_m - site.center.y_m,
                )

            # Exploit (PLANT) bid
            if self.is_eligible(estimate) and self.capability.can_plant and robot_obs.seeds_remaining > 0:
                stand_gap = estimate.stand_gap if estimate.stand_gap is not None else 0.0
                utility = 10.0 + (stand_gap * self.gap_weight) - (cost * 1.5)
                bids.append(
                    AgentBid(
                        robot_id=robot_obs.robot_id,
                        site_id=site.site_id,
                        mode=TaskMode.PLANT,
                        utility=round(utility, 4),
                        reason=f"confirmed eligible gap={stand_gap:.2f} (dist={cost:.2f}m)",
                    )
                )

            # Explore (SAMPLE) bid
            elif not estimate.complete and self.capability.can_sample:
                # Value of Information: completely unknown vs partially known
                voi_base = 4.0 + (2.0 if estimate.sample_count == 0 else 1.0)

                # Seed-depletion transition:
                # If rover has 0 seeds remaining, boost sample utility to become a dedicated scout
                depletion_boost = 3.0 if robot_obs.seeds_remaining == 0 else 0.0

                # Cooperative proximity bonus:
                # Prioritise scouting sites near teammates that still have seeds
                coop_bonus = 0.0
                for peer in teammates:
                    if peer.robot_id != robot_obs.robot_id and peer.seeds_remaining > 0:
                        d_peer = hypot(
                            peer.pose.x_m - site.center.x_m,
                            peer.pose.y_m - site.center.y_m,
                        )
                        if d_peer <= self.coop_radius_m:
                            bonus = 2.0 - (d_peer / self.coop_radius_m) * 1.5
                            if bonus > coop_bonus:
                                coop_bonus = bonus

                utility = voi_base + depletion_boost + coop_bonus - (cost * self.dist_weight)
                bids.append(
                    AgentBid(
                        robot_id=robot_obs.robot_id,
                        site_id=site.site_id,
                        mode=TaskMode.SAMPLE,
                        utility=round(utility, 4),
                        reason=f"sample unconfirmed site (samples={estimate.sample_count}, dist={cost:.2f}m)",
                    )
                )

        # Sort bids descending by utility with deterministic tie-breaking
        bids.sort(key=lambda b: (b.utility, b.site_id), reverse=True)

        if not bids:
            bids.append(
                AgentBid(
                    robot_id=robot_obs.robot_id,
                    site_id="",
                    mode=TaskMode.HOLD,
                    utility=0.0,
                    reason="no accessible work",
                )
            )

        return bids


class CooperativeSwarmPlanner:
    """Decentralized Swarm Planner with Tier-1 Micro-Edge Auction and Tier-2 AI Hub.

    Matches SimpleMissionPlanner's interface for seamless plug-in with MissionCore:
    planner.capabilities, planner.decide(frame, field_map, excluded_sites).
    """

    def __init__(
        self,
        capabilities: tuple[RobotCapability, ...] | dict[str, RobotCapability],
        advisor: GeminiSwarmAdvisor | None = None,
        travel_cost_fn: Callable[[Pose2D, Site], float | None] | None = None,
        min_gap: float = 0.6,
        moisture_range: tuple[float, float] = (0.25, 0.8),
        firmness_range: tuple[float, float] = (0.2, 0.9),
    ):
        if isinstance(capabilities, dict):
            self.capabilities: dict[str, RobotCapability] = dict(capabilities)
        else:
            if not capabilities or len({c.robot_id for c in capabilities}) != len(capabilities):
                raise ValueError("unique nonempty capabilities required")
            self.capabilities = {c.robot_id: c for c in capabilities}

        self.advisor = advisor
        self.travel_cost_fn = travel_cost_fn
        self.min_gap = min_gap
        self.moisture_range = moisture_range
        self.firmness_range = firmness_range

        # Instantiate a micro-edge agent for each rover
        self.agents: dict[str, SwarmRobotAgent] = {
            c.robot_id: SwarmRobotAgent(
                capability=c,
                min_gap=min_gap,
                moisture_range=moisture_range,
                firmness_range=firmness_range,
            )
            for c in self.capabilities.values()
        }
        self._prev_frame: ObservationFrame | None = None

    def is_eligible(self, estimate: SiteEstimate) -> bool:
        return bool(
            estimate.complete
            and estimate.stand_gap is not None
            and estimate.stand_gap >= self.min_gap
            and estimate.moisture is not None
            and self.moisture_range[0] <= estimate.moisture <= self.moisture_range[1]
            and estimate.firmness is not None
            and self.firmness_range[0] <= estimate.firmness <= self.firmness_range[1]
        )

    def _verify_safety(
        self,
        bid: AgentBid,
        robot_obs: RobotObservation,
        field_map: MeasuredFieldMap,
    ) -> AgentBid:
        """Hard Safety Verifier: rejects invalid/hallucinated actions and enforces safe envelopes."""
        if bid.mode == TaskMode.PLANT:
            if not bid.site_id or bid.site_id not in field_map.sites:
                return AgentBid(robot_obs.robot_id, "", TaskMode.HOLD, 0.0, "safety: unknown site")
            site = field_map.sites[bid.site_id]
            if not site.permitted:
                return AgentBid(robot_obs.robot_id, "", TaskMode.HOLD, 0.0, "safety: site not permitted")
            if not self.capabilities[robot_obs.robot_id].can_plant:
                return AgentBid(robot_obs.robot_id, "", TaskMode.HOLD, 0.0, "safety: actor cannot plant")
            if robot_obs.seeds_remaining <= 0:
                return AgentBid(robot_obs.robot_id, "", TaskMode.HOLD, 0.0, "safety: zero seeds remaining")
            est = field_map.estimate(bid.site_id)
            if not est.complete:
                return AgentBid(robot_obs.robot_id, "", TaskMode.HOLD, 0.0, "safety: incomplete site sample")
            if not self.is_eligible(est):
                return AgentBid(robot_obs.robot_id, "", TaskMode.HOLD, 0.0, "safety: site out of agronomic spec")
            return bid

        if bid.mode == TaskMode.SAMPLE:
            if not bid.site_id or bid.site_id not in field_map.sites:
                return AgentBid(robot_obs.robot_id, "", TaskMode.HOLD, 0.0, "safety: unknown site")
            site = field_map.sites[bid.site_id]
            if not site.permitted:
                return AgentBid(robot_obs.robot_id, "", TaskMode.HOLD, 0.0, "safety: site not permitted")
            if not self.capabilities[robot_obs.robot_id].can_sample:
                return AgentBid(robot_obs.robot_id, "", TaskMode.HOLD, 0.0, "safety: actor cannot sample")
            return bid

        return bid

    def _try_advisor(
        self,
        frame: ObservationFrame,
        field_map: MeasuredFieldMap,
        excluded_sites: frozenset[str],
    ) -> tuple[TaskIntent, ...] | None:
        """Attempts Tier-2 AI Supervisor advice; rejects hallucinations and falls back safely."""
        if self.advisor is None:
            return None

        # Check connection link: if link down, instant offline fallback to Tier 1
        if any(not r.link_ok for r in frame.robots):
            return None

        event = self.advisor.detect_event(frame, field_map, self._prev_frame)
        if not event:
            return None

        intents = self.advisor.advise(frame, field_map, self.capabilities, event, excluded_sites)
        if intents is None:
            return None

        # Validate advisor intents through the Hard Safety Verifier
        obs_by_id = {r.robot_id: r for r in frame.robots}
        assigned_sites: set[str] = set(excluded_sites)

        for intent in intents:
            if intent.robot_id not in obs_by_id:
                return None
            robot_obs = obs_by_id[intent.robot_id]

            if intent.mode != TaskMode.HOLD:
                if not intent.target_site_id or intent.target_site_id in assigned_sites:
                    return None
                bid = AgentBid(
                    robot_id=intent.robot_id,
                    site_id=intent.target_site_id,
                    mode=intent.mode,
                    utility=1.0,
                    reason=intent.reason,
                )
                verified = self._verify_safety(bid, robot_obs, field_map)
                if verified.mode != intent.mode:
                    # Advisor suggested unsafe action; reject entire advice and fall back to Tier 1
                    return None
                assigned_sites.add(intent.target_site_id)

        return intents

    def _auction(
        self,
        frame: ObservationFrame,
        field_map: MeasuredFieldMap,
        excluded_sites: frozenset[str],
    ) -> tuple[TaskIntent, ...]:
        """Runs onboard decentralized auction to resolve bids across the homogeneous swarm."""
        candidate_sites = [
            site for site in field_map.sites.values()
            if site.permitted and site.site_id not in excluded_sites
        ]
        teammates = list(frame.robots)

        # 1. Collect utility bids from each rover's tiny AI agent
        bids_by_robot: dict[str, list[AgentBid]] = {}
        all_bids: list[AgentBid] = []
        for robot_obs in frame.robots:
            agent = self.agents[robot_obs.robot_id]
            bids = agent.compute_bids(
                robot_obs=robot_obs,
                field_map=field_map,
                candidate_sites=candidate_sites,
                teammates=teammates,
                travel_cost_fn=self.travel_cost_fn,
            )
            bids_by_robot[robot_obs.robot_id] = bids
            all_bids.extend(bids)

        # 2. Decentralized Auction / Consensus Resolution
        # Sort bids descending by utility (with deterministic tie-breakers)
        all_bids.sort(key=lambda b: (b.utility, b.robot_id, b.site_id), reverse=True)

        assigned_sites: set[str] = set(excluded_sites)
        winning_bids: dict[str, AgentBid] = {}

        # Greedily allocate highest-utility non-conflicting bids
        for bid in all_bids:
            if bid.robot_id in winning_bids:
                continue
            if bid.mode == TaskMode.HOLD:
                continue
            if bid.site_id not in assigned_sites:
                winning_bids[bid.robot_id] = bid
                assigned_sites.add(bid.site_id)

        # For any unassigned rover, pick its next-best non-conflicting bid or HOLD
        for robot_obs in frame.robots:
            rid = robot_obs.robot_id
            if rid not in winning_bids:
                assigned = False
                for b in bids_by_robot[rid]:
                    if b.mode != TaskMode.HOLD and b.site_id and b.site_id not in assigned_sites:
                        winning_bids[rid] = b
                        assigned_sites.add(b.site_id)
                        assigned = True
                        break
                if not assigned:
                    winning_bids[rid] = AgentBid(
                        robot_id=rid,
                        site_id="",
                        mode=TaskMode.HOLD,
                        utility=0.0,
                        reason="no non-conflicting task available",
                    )

        # 3. Apply Hard Safety Verifier and generate standard TaskIntents
        intents: list[TaskIntent] = []
        obs_by_id = {r.robot_id: r for r in frame.robots}

        # Deterministic order by robot_id
        for robot_obs in sorted(frame.robots, key=lambda r: r.robot_id):
            bid = winning_bids[robot_obs.robot_id]
            verified = self._verify_safety(bid, robot_obs, field_map)

            target = verified.site_id if verified.mode != TaskMode.HOLD else None
            intents.append(
                TaskIntent(
                    task_id=f"{frame.episode_id}:{frame.time_s:.3f}:{robot_obs.robot_id}:{verified.mode.value}",
                    robot_id=robot_obs.robot_id,
                    mode=verified.mode,
                    target_site_id=target,
                    issued_at_s=frame.time_s,
                    expires_at_s=frame.time_s + 1.0,
                    reason=verified.reason,
                )
            )

        return tuple(intents)

    def decide(
        self,
        frame: ObservationFrame,
        field_map: MeasuredFieldMap,
        excluded_sites: frozenset[str] = frozenset(),
    ) -> tuple[TaskIntent, ...]:
        """Main planning decision cycle called by MissionCore."""
        # Tier 2 Advisor (if online and event triggered)
        advisor_intents = self._try_advisor(frame, field_map, excluded_sites)
        if advisor_intents is not None:
            self._prev_frame = frame
            return advisor_intents

        # Tier 1 Micro-Edge Auction
        intents = self._auction(frame, field_map, excluded_sites)
        self._prev_frame = frame
        return intents
