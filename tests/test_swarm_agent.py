"""Unit tests for Agentic Swarm & Edge Architecture (src/ricebot/agent/)."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ricebot.agent import (
    AgentBid,
    CooperativeSwarmPlanner,
    GeminiSwarmAdvisor,
    SwarmRobotAgent,
    homogeneous_capabilities,
)
from ricebot.contracts import (
    ObservationFrame,
    PlantFeedback,
    Pose2D,
    RobotCapability,
    RobotObservation,
    SensorKind,
    Site,
    SoilSample,
    TaskIntent,
    TaskMode,
)
from ricebot.control import LocalController
from ricebot.mission import MeasuredFieldMap
from ricebot.runtime import MissionCore

SITES = (
    Site("site-a", Pose2D(0.0, 0.0, 0.0)),
    Site("site-b", Pose2D(2.0, 0.0, 0.0)),
    Site("site-c", Pose2D(0.0, 2.0, 0.0)),
    Site("protected", Pose2D(5.0, 5.0, 0.0), permitted=False),
)


def make_sample(actor: str, site: str, kind: SensorKind, value: float, t: float) -> SoilSample:
    site_obj = next(s for s in SITES if s.site_id == site)
    return SoilSample(
        sample_id=f"{actor}:{site}:{kind.value}:{t}",
        robot_id=actor,
        site_id=site,
        kind=kind,
        value=value,
        sampled_at_s=t,
        location=site_obj.center,
    )


def make_trio(actor: str, site: str, t: float, gap: float = 0.85, moisture: float = 0.5, firmness: float = 0.55):
    return (
        make_sample(actor, site, SensorKind.MOISTURE, moisture, t),
        make_sample(actor, site, SensorKind.FIRMNESS, firmness, t),
        make_sample(actor, site, SensorKind.STAND_GAP, gap, t),
    )


def make_robot(
    actor: str,
    x: float,
    y: float,
    t: float = 0.0,
    seeds: int = 2,
    front_range_m: float | None = 3.0,
    samples=(),
    link_ok: bool = True,
) -> RobotObservation:
    return RobotObservation(
        robot_id=actor,
        pose=Pose2D(x, y, 0.0),
        time_s=t,
        front_range_m=front_range_m,
        seeds_remaining=seeds,
        samples=tuple(samples),
        link_ok=link_ok,
    )


def make_frame(t: float, *robots: RobotObservation, feedback=()) -> ObservationFrame:
    return ObservationFrame(
        episode_id="swarm-ep-1",
        time_s=t,
        robots=tuple(robots),
        plant_feedback=tuple(feedback),
    )


class SwarmAgentTests(unittest.TestCase):
    def test_homogeneous_capabilities(self):
        caps = homogeneous_capabilities(["rover-1", "rover-2", "rover-3"])
        self.assertEqual(len(caps), 3)
        for c in caps:
            self.assertTrue(c.can_sample)
            self.assertTrue(c.can_plant)

        with self.assertRaises(ValueError):
            homogeneous_capabilities([])
        with self.assertRaises(ValueError):
            homogeneous_capabilities(["rover-1", "rover-1"])

    def test_swarm_robot_agent_explore_vs_exploit_bidding(self):
        caps = homogeneous_capabilities(["rover-1"])[0]
        agent = SwarmRobotAgent(caps)
        field = MeasuredFieldMap(SITES)

        # 1. Unconfirmed sites -> bids SAMPLE
        r1 = make_robot("rover-1", 0.0, 0.0, t=1.0, seeds=2)
        bids = agent.compute_bids(r1, field, list(SITES), [r1])
        self.assertTrue(any(b.mode == TaskMode.SAMPLE for b in bids))
        self.assertFalse(any(b.mode == TaskMode.PLANT for b in bids))

        # 2. Add complete eligible samples for site-a -> bids PLANT with higher utility
        samples = make_trio("rover-1", "site-a", t=1.0, gap=0.9, moisture=0.5, firmness=0.5)
        field.update(make_frame(1.0, make_robot("rover-1", 0.0, 0.0, t=1.0, samples=samples)))
        self.assertTrue(field.estimate("site-a").complete)

        bids2 = agent.compute_bids(r1, field, list(SITES), [r1])
        top_bid = bids2[0]
        self.assertEqual(top_bid.mode, TaskMode.PLANT)
        self.assertEqual(top_bid.site_id, "site-a")
        self.assertGreater(top_bid.utility, 8.0)

    def test_seed_depletion_transition(self):
        caps = homogeneous_capabilities(["rover-1"])[0]
        agent = SwarmRobotAgent(caps)
        field = MeasuredFieldMap(SITES)

        # Site A is eligible for planting
        field.update(make_frame(1.0, make_robot("rover-1", 0.0, 0.0, t=1.0, samples=make_trio("rover-1", "site-a", 1.0))))

        # Rover has 0 seeds -> cannot plant, automatically boosts SAMPLE bids
        r_depleted = make_robot("rover-1", 0.0, 0.0, t=2.0, seeds=0)
        bids = agent.compute_bids(r_depleted, field, list(SITES), [r_depleted])
        self.assertFalse(any(b.mode == TaskMode.PLANT for b in bids))
        self.assertTrue(any(b.mode == TaskMode.SAMPLE for b in bids))
        sample_bid = next(b for b in bids if b.mode == TaskMode.SAMPLE)
        # Utility includes depletion boost (+3.0)
        self.assertGreater(sample_bid.utility, 5.0)

    def test_cooperative_proximity_bonus(self):
        caps = homogeneous_capabilities(["rover-1", "rover-2"])
        agent1 = SwarmRobotAgent(caps[0])
        field = MeasuredFieldMap(SITES)

        # Rover 1 has 0 seeds, Rover 2 has 2 seeds located close to site-b (at x=2.0, y=0.0)
        r1 = make_robot("rover-1", 0.0, 0.0, t=1.0, seeds=0)
        r2 = make_robot("rover-2", 1.8, 0.0, t=1.0, seeds=2)

        # Rover 1 evaluates site-b (near Rover 2) vs site-c (far from Rover 2)
        bids = agent1.compute_bids(r1, field, list(SITES), [r1, r2])
        bid_b = next(b for b in bids if b.site_id == "site-b")
        bid_c = next(b for b in bids if b.site_id == "site-c")
        # site-b gets a cooperative proximity bonus because peer r2 with seeds is nearby
        self.assertGreater(bid_b.utility, bid_c.utility)

    def test_dynamic_role_split_via_auction(self):
        caps = homogeneous_capabilities(["rover-1", "rover-2"])
        planner = CooperativeSwarmPlanner(caps)
        field = MeasuredFieldMap(SITES)

        # site-a is eligible for planting
        field.update(make_frame(1.0, make_robot("rover-1", 0.0, 0.0, t=1.0, samples=make_trio("rover-1", "site-a", 1.0))))

        # Rover 1 is closer to site-a (at 0,0); Rover 2 is at (2.0, 0.0) near site-b (unconfirmed)
        r1 = make_robot("rover-1", 0.1, 0.0, t=2.0, seeds=2)
        r2 = make_robot("rover-2", 1.9, 0.0, t=2.0, seeds=2)
        f = make_frame(2.0, r1, r2)

        intents = planner.decide(f, field)
        intent1 = next(i for i in intents if i.robot_id == "rover-1")
        intent2 = next(i for i in intents if i.robot_id == "rover-2")

        # Rover 1 wins PLANT at site-a
        self.assertEqual(intent1.mode, TaskMode.PLANT)
        self.assertEqual(intent1.target_site_id, "site-a")

        # Rover 2 dynamically splits role to SAMPLE at site-b without conflict
        self.assertEqual(intent2.mode, TaskMode.SAMPLE)
        self.assertEqual(intent2.target_site_id, "site-b")
        self.assertNotEqual(intent1.target_site_id, intent2.target_site_id)

    def test_hard_safety_verifier_rejects_hallucinations(self):
        caps = homogeneous_capabilities(["rover-1"])
        planner = CooperativeSwarmPlanner(caps)
        field = MeasuredFieldMap(SITES)

        # Manually create unsafe bids to test Hard Safety Verifier
        r1 = make_robot("rover-1", 0.0, 0.0, t=1.0, seeds=0)

        # 1. Unsafe: Planting with 0 seeds
        unsafe_bid = AgentBid("rover-1", "site-a", TaskMode.PLANT, 10.0, "hallucinated plant")
        verified = planner._verify_safety(unsafe_bid, r1, field)
        self.assertEqual(verified.mode, TaskMode.HOLD)
        self.assertIn("zero seeds", verified.reason)

        # 2. Unsafe: Planting on unmeasured site
        r1_with_seeds = make_robot("rover-1", 0.0, 0.0, t=1.0, seeds=2)
        unsafe_bid2 = AgentBid("rover-1", "site-b", TaskMode.PLANT, 10.0, "plant unmeasured")
        verified2 = planner._verify_safety(unsafe_bid2, r1_with_seeds, field)
        self.assertEqual(verified2.mode, TaskMode.HOLD)
        self.assertIn("incomplete", verified2.reason)

        # 3. Unsafe: Site not permitted
        unsafe_bid3 = AgentBid("rover-1", "protected", TaskMode.SAMPLE, 10.0, "sample protected")
        verified3 = planner._verify_safety(unsafe_bid3, r1_with_seeds, field)
        self.assertEqual(verified3.mode, TaskMode.HOLD)
        self.assertIn("not permitted", verified3.reason)

    def test_advisor_event_detection_and_offline_fallback(self):
        advisor = GeminiSwarmAdvisor(api_key=None, enabled=True)
        self.assertFalse(advisor.is_available)

        field = MeasuredFieldMap(SITES)

        # 1. Normal state -> no event
        r1 = make_robot("rover-1", 0.0, 0.0, t=1.0, front_range_m=3.0)
        f1 = make_frame(1.0, r1)
        self.assertIsNone(advisor.detect_event(f1, field, None))

        # 2. Route obstruction event: front obstacle <= 0.35m
        r_blocked = make_robot("rover-1", 0.0, 0.0, t=2.0, front_range_m=0.20)
        f2 = make_frame(2.0, r_blocked)
        self.assertEqual(advisor.detect_event(f2, field, f1), "route_obstructed")

        # 3. Robot stranded: sensor fault (None)
        r_fault = make_robot("rover-1", 0.0, 0.0, t=3.0, front_range_m=None)
        f3 = make_frame(3.0, r_fault)
        self.assertEqual(advisor.detect_event(f3, field, f2), "robot_stranded")

        # Offline fallback in planner: runs Tier 1 auction safely without advisor errors
        caps = homogeneous_capabilities(["rover-1"])
        planner = CooperativeSwarmPlanner(caps, advisor=advisor)
        intents = planner.decide(f1, field)
        self.assertEqual(len(intents), 1)
        self.assertEqual(intents[0].mode, TaskMode.SAMPLE)

    def test_integration_with_mission_core_and_local_controller(self):
        caps = homogeneous_capabilities(["rover-1", "rover-2"])
        planner = CooperativeSwarmPlanner(caps)
        field = MeasuredFieldMap(SITES)
        controller = LocalController()
        core = MissionCore(field_map=field, planner=planner, controller=controller)

        # Frame 1: Rover 1 is at site-a with samples; ingests samples, verifies site-a and immediately plans PLANT
        samples_a = make_trio("rover-1", "site-a", t=1.0, gap=0.9, moisture=0.5, firmness=0.5)
        f1 = make_frame(1.0,
                        make_robot("rover-1", 0.0, 0.0, t=1.0, seeds=2, samples=samples_a),
                        make_robot("rover-2", 2.0, 0.0, t=1.0, seeds=2))
        dec1 = core.decide(f1)
        self.assertEqual(len(dec1.intents), 2)
        self.assertEqual(len(dec1.commands), 2)

        plant_intent = next(i for i in dec1.intents if i.robot_id == "rover-1")
        self.assertEqual(plant_intent.mode, TaskMode.PLANT)
        self.assertEqual(plant_intent.target_site_id, "site-a")
        plant_cmd = next(c for c in dec1.commands if c.robot_id == "rover-1")
        self.assertTrue(plant_cmd.release_seed)
        self.assertEqual(plant_cmd.target_site_id, "site-a")

        # Frame 2: While plant is pending acknowledgement, controller holds seed drop and site-a is excluded
        f2 = make_frame(1.5,
                        make_robot("rover-1", 0.0, 0.0, t=1.5, seeds=2),
                        make_robot("rover-2", 2.0, 0.0, t=1.5, seeds=2))
        dec2 = core.decide(f2)
        cmd2 = next(c for c in dec2.commands if c.robot_id == "rover-1")
        self.assertFalse(cmd2.release_seed)

        # Frame 3: Provide plant feedback acknowledgement; site-a becomes excluded from future planting
        ack = PlantFeedback("rover-1", plant_cmd.task_id, True, 2.0, Pose2D(0.01, 0.0, 0.0))
        f3 = make_frame(2.0,
                        make_robot("rover-1", 0.0, 0.0, t=2.0, seeds=1),
                        make_robot("rover-2", 2.0, 0.0, t=2.0, seeds=2),
                        feedback=(ack,))
        dec3 = core.decide(f3)
        self.assertNotIn("site-a", {i.target_site_id for i in dec3.intents})


if __name__ == "__main__":
    unittest.main()
