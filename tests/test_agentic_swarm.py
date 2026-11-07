"""Unit tests for pure Swarm Agentic Coordination (Task Allocation + Rescue Triage)."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ricebot.agent import (
    AmbiguityEvent,
    CooperativeSwarmPlanner,
    RescueVerdict,
    homogeneous_capabilities,
)
from ricebot.contracts import (
    ObservationFrame,
    PlantFeedback,
    Pose2D,
    RobotObservation,
    RobotCommand,
    SensorKind,
    Site,
    SoilSample,
    TaskMode,
)
from ricebot.mission import MeasuredFieldMap, SiteEstimate
from ricebot.runtime import EpisodeRunner, MissionCore


SITES = (
    Site("a", Pose2D(0.0, 0.0, 0.0)),
    Site("b", Pose2D(1.5, 0.0, 0.0)),
    Site("c", Pose2D(3.0, 0.0, 0.0)),
    Site("protected", Pose2D(1.5, 1.2, 0.0), permitted=False, no_go_radius_m=0.4),
)
HOMOGENEOUS_CAPS = homogeneous_capabilities(("rover-1", "rover-2"))


def sample(
    actor: str,
    site_id: str,
    kind: SensorKind,
    value: float,
    t: float,
    sites: tuple[Site, ...] = SITES,
) -> SoilSample:
    center = next(s.center for s in sites if s.site_id == site_id)
    return SoilSample(
        f"{actor}:{site_id}:{kind.value}:{t}",
        actor,
        site_id,
        kind,
        value,
        t,
        center,
    )


def trio(
    actor: str,
    site_id: str,
    t: float,
    gap: float = 0.85,
    moisture: float = 0.5,
    firmness: float = 0.55,
    sites: tuple[Site, ...] = SITES,
) -> tuple[SoilSample, ...]:
    return (
        sample(actor, site_id, SensorKind.MOISTURE, moisture, t, sites),
        sample(actor, site_id, SensorKind.FIRMNESS, firmness, t, sites),
        sample(actor, site_id, SensorKind.STAND_GAP, gap, t, sites),
    )


def robot(
    actor: str,
    x: float,
    y: float,
    t: float,
    values: tuple[SoilSample, ...] = (),
    range_m: float | None = 4.0,
    seeds: int = 2,
    link_ok: bool = True,
    bogged: bool = False,
) -> RobotObservation:
    return RobotObservation(
        actor, Pose2D(x, y, 0.0), t, range_m, seeds, tuple(values), link_ok,
        is_bogged=bogged,
    )


def frame(
    t: float,
    *robots: RobotObservation,
    feedback: tuple[PlantFeedback, ...] = (),
) -> ObservationFrame:
    return ObservationFrame("episode-swarm", t, tuple(robots), feedback)


class HomogeneousSwarmPlannerTests(unittest.TestCase):
    def test_homogeneous_robots_dynamically_split_plant_and_sample_via_auction(self):
        fmap = MeasuredFieldMap(SITES)
        planner = CooperativeSwarmPlanner(HOMOGENEOUS_CAPS)
        f1 = frame(
            1.0,
            robot("rover-1", 1.4, 0.0, 1.0, seeds=2),
            robot("rover-2", 0.1, 0.0, 1.0, trio("rover-2", "a", 1.0, gap=0.9), seeds=2),
        )
        fmap.update(f1)
        intents = planner.decide(f1, fmap)
        by_robot = {i.robot_id: i for i in intents}

        self.assertEqual(by_robot["rover-2"].mode, TaskMode.PLANT)
        self.assertEqual(by_robot["rover-2"].target_site_id, "a")
        self.assertEqual(by_robot["rover-1"].mode, TaskMode.SAMPLE)
        self.assertEqual(by_robot["rover-1"].target_site_id, "b")

    def test_seed_depleted_robot_automatically_becomes_scout_for_peer(self):
        fmap = MeasuredFieldMap(SITES)
        planner = CooperativeSwarmPlanner(HOMOGENEOUS_CAPS)
        f1 = frame(
            1.0,
            robot("rover-1", 0.0, 0.0, 1.0, trio("rover-1", "a", 1.0, gap=0.9), seeds=0),
            robot("rover-2", 0.2, 0.0, 1.0, seeds=2),
        )
        fmap.update(f1)
        intents = planner.decide(f1, fmap)
        by_robot = {i.robot_id: i for i in intents}
        self.assertEqual(by_robot["rover-2"].mode, TaskMode.PLANT)
        self.assertEqual(by_robot["rover-2"].target_site_id, "a")
        self.assertEqual(by_robot["rover-1"].mode, TaskMode.SAMPLE)
        self.assertNotEqual(by_robot["rover-1"].target_site_id, "a")

    def test_rescue_triage_worth_saving_vs_defer_vs_abandon(self):
        """Verify all 3 agentic rescue verdicts: RESCUE_NOW, DEFER_RESCUE, ABANDON_ESCALATE."""
        fmap = MeasuredFieldMap(SITES)
        planner = CooperativeSwarmPlanner(HOMOGENEOUS_CAPS)

        # Case 1: RESCUE_NOW — rover-1 reports a simulated bogged condition with 2 needed seeds,
        # while healthy rover-2 has 0 seeds left. Saving rover-1 is essential!
        f_rescue_now = frame(
            1.0,
            robot("rover-1", 0.1, 0.0, 1.0, bogged=True, seeds=2),
            robot("rover-2", 1.4, 0.0, 1.0, seeds=0),
        )
        intents_now = planner.decide(f_rescue_now, fmap)
        by_id_now = {i.robot_id: i for i in intents_now}
        assessment_now = planner.last_rescue_assessments["rover-1"]
        self.assertEqual(assessment_now.verdict, RescueVerdict.RESCUE_NOW)
        self.assertGreater(assessment_now.net_rescue_value, 0.0)
        self.assertEqual(by_id_now["rover-1"].mode, TaskMode.HOLD)
        # Healthy rover-2 is dispatched to site 'a' (nearest to stranded rover-1) to assist
        self.assertEqual(by_id_now["rover-2"].target_site_id, "a")
        self.assertIn("rescue_assist(rover-1)", by_id_now["rover-2"].reason)

        # Case 2: DEFER_RESCUE — rover-1 is stranded with seeds, but rover-2 has its own
        # seeds AND a confirmed eligible gap at 'b' right next to it -> plant 'b' first!
        f_defer = frame(
            2.0,
            robot("rover-1", 0.1, 0.0, 2.0, bogged=True, seeds=1),
            robot("rover-2", 1.5, 0.0, 2.0, trio("rover-2", "b", 2.0, gap=0.9), seeds=2),
        )
        fmap.update(f_defer)
        intents_defer = planner.decide(f_defer, fmap)
        by_id_defer = {i.robot_id: i for i in intents_defer}
        assessment_defer = planner.last_rescue_assessments["rover-1"]
        self.assertEqual(assessment_defer.verdict, RescueVerdict.DEFER_RESCUE)
        self.assertEqual(by_id_defer["rover-2"].mode, TaskMode.PLANT)
        self.assertEqual(by_id_defer["rover-2"].target_site_id, "b")

        # Case 3: ABANDON_ESCALATE — rover-1 is stranded inside a hazardous no-go area
        # (or has 0 seeds while rover-2 has plenty) -> not worth risking rover-2!
        f_hazard = frame(
            3.0,
            robot("rover-1", 1.5, 1.15, 3.0, bogged=True, seeds=2),
            robot("rover-2", 1.5, 0.0, 3.0, seeds=3),
        )
        planner.decide(f_hazard, fmap)
        assessment_hazard = planner.last_rescue_assessments["rover-1"]
        self.assertEqual(assessment_hazard.verdict, RescueVerdict.ABANDON_ESCALATE)
        self.assertGreaterEqual(assessment_hazard.hazard_risk, 1.5)

    def test_missing_front_range_is_not_an_immobilisation(self):
        fmap = MeasuredFieldMap(SITES)
        planner = CooperativeSwarmPlanner(HOMOGENEOUS_CAPS)
        planner.decide(frame(0.0, robot("rover-1", 0.0, 0.0, 0.0),
                             robot("rover-2", 1.5, 0.0, 0.0)), fmap)
        for tick in range(1, 5):
            t = tick * 0.1
            planner.record_commands((
                RobotCommand("rover-1", t - 0.1, t + 0.1, linear_m_s=0.25),
            ))
            planner.decide(frame(t, robot("rover-1", 0.0, 0.0, t, range_m=None),
                                 robot("rover-2", 1.5, 0.0, t)), fmap)
            self.assertNotIn("rover-1", planner.last_rescue_assessments)

    def test_obstacle_stop_and_intentional_hold_do_not_trigger_rescue(self):
        fmap = MeasuredFieldMap(SITES)
        planner = CooperativeSwarmPlanner(HOMOGENEOUS_CAPS)
        planner.decide(frame(0.0, robot("rover-1", 0.0, 0.0, 0.0),
                             robot("rover-2", 1.5, 0.0, 0.0)), fmap)
        for tick in range(1, 5):
            t = tick * 0.1
            planner.record_commands((
                RobotCommand("rover-1", t - 0.1, t + 0.1, linear_m_s=0.25)
                if tick <= 2
                else RobotCommand("rover-1", t - 0.1, t + 0.1),
            ))
            planner.decide(frame(t, robot("rover-1", 0.0, 0.0, t,
                                         range_m=0.20 if tick <= 2 else 4.0),
                                 robot("rover-2", 1.5, 0.0, t)), fmap)
            self.assertNotIn("rover-1", planner.last_rescue_assessments)

    def test_sustained_commanded_stall_requires_independent_motion_mismatch(self):
        fmap = MeasuredFieldMap(SITES)
        planner = CooperativeSwarmPlanner(HOMOGENEOUS_CAPS, stuck_ticks_threshold=2)
        planner.decide(frame(0.0, robot("rover-1", 0.0, 0.0, 0.0),
                             robot("rover-2", 1.5, 0.0, 0.0)), fmap)
        planner.record_commands((RobotCommand("rover-1", 0.0, 0.2, linear_m_s=0.25),))
        planner.decide(frame(0.1, robot("rover-1", 0.0, 0.0, 0.1),
                             robot("rover-2", 1.5, 0.0, 0.1)), fmap)
        self.assertNotIn("rover-1", planner.last_rescue_assessments)
        planner.record_commands((RobotCommand("rover-1", 0.1, 0.3, linear_m_s=0.25),))
        planner.decide(frame(0.2, robot("rover-1", 0.0, 0.0, 0.2),
                             robot("rover-2", 1.5, 0.0, 0.2)), fmap)
        self.assertIn("rover-1", planner.last_rescue_assessments)

    def test_actual_motion_clears_candidate_stall(self):
        fmap = MeasuredFieldMap(SITES)
        planner = CooperativeSwarmPlanner(HOMOGENEOUS_CAPS, stuck_ticks_threshold=2)
        planner.decide(frame(0.0, robot("rover-1", 0.0, 0.0, 0.0),
                             robot("rover-2", 1.5, 0.0, 0.0)), fmap)
        planner.record_commands((RobotCommand("rover-1", 0.0, 0.2, linear_m_s=0.2),))
        planner.decide(frame(0.1, robot("rover-1", 0.0, 0.0, 0.1),
                             robot("rover-2", 1.5, 0.0, 0.1)), fmap)
        planner.record_commands((RobotCommand("rover-1", 0.1, 0.3, linear_m_s=0.2),))
        planner.decide(frame(0.2, robot("rover-1", 0.02, 0.0, 0.2),
                             robot("rover-2", 1.5, 0.0, 0.2)), fmap)
        self.assertNotIn("rover-1", planner.last_rescue_assessments)

    def test_pluggable_traversal_cost_hook(self):
        fmap = MeasuredFieldMap(SITES)

        def custom_traversal_cost(pose: Pose2D, site: Site) -> float | None:
            if site.site_id == "a":
                return None
            return 1.0 if site.site_id == "c" else 5.0

        planner = CooperativeSwarmPlanner(HOMOGENEOUS_CAPS, travel_cost_fn=custom_traversal_cost)
        intents = planner.decide(
            frame(1.0, robot("rover-1", 0.0, 0.0, 1.0), robot("rover-2", 0.0, 0.0, 1.0)),
            fmap,
        )
        targets = {i.target_site_id for i in intents}
        self.assertNotIn("a", targets)
        self.assertEqual(targets, {"b", "c"})

    def test_mid_episode_quality_overturn_and_route_obstruction_adaptation(self):
        fmap = MeasuredFieldMap(SITES)
        planner = CooperativeSwarmPlanner(HOMOGENEOUS_CAPS)
        core = MissionCore(fmap, planner)

        d1 = core.decide(
            frame(
                1.0,
                robot("rover-1", 1.2, 0.0, 1.0),
                robot("rover-2", 0.2, 0.0, 1.0, trio("rover-2", "a", 1.0, gap=0.9), seeds=0),
            )
        )
        p1 = next(i for i in d1.intents if i.robot_id == "rover-1")
        self.assertEqual(p1.mode, TaskMode.PLANT)
        self.assertEqual(p1.target_site_id, "a")

        d2 = core.decide(
            frame(
                2.0,
                robot("rover-1", 1.0, 0.0, 2.0, range_m=0.25),
                robot(
                    "rover-2",
                    0.2,
                    0.0,
                    2.0,
                    (sample("rover-2", "a", SensorKind.STAND_GAP, 0.1, 2.0),),
                    seeds=0,
                ),
            )
        )
        p2 = next(i for i in d2.intents if i.robot_id == "rover-1")
        self.assertNotEqual(p2.mode, TaskMode.PLANT)
        self.assertNotEqual(p2.target_site_id, "a")
        self.assertIn("replan(site a quality overturned)", p2.reason)

    def test_optional_supervisory_advisor_and_offline_autonomy_fallback(self):
        class MockAdvisor:
            def __init__(self, should_fail: bool = False, bad_proposal: bool = False):
                self.should_fail = should_fail
                self.bad_proposal = bad_proposal
                self.events: list[AmbiguityEvent] = []

            def advise(
                self,
                event: AmbiguityEvent,
                obs_frame: ObservationFrame,
                estimates: dict[str, SiteEstimate],
            ):
                self.events.append(event)
                if self.should_fail:
                    raise RuntimeError("remote service timeout")
                if self.bad_proposal:
                    return {"rover-1": (TaskMode.PLANT, "b", "illegal unconfirmed")}
                return {
                    "rover-1": (TaskMode.SAMPLE, "b", "re-verify b"),
                    "rover-2": (TaskMode.SAMPLE, "c", "explore c"),
                }

        fmap = MeasuredFieldMap(SITES)
        advisor = MockAdvisor()
        planner = CooperativeSwarmPlanner(HOMOGENEOUS_CAPS, advisor=advisor)

        f1 = frame(
            1.0,
            robot("rover-1", 1.0, 0.0, 1.0),
            robot("rover-2", 0.0, 0.0, 1.0, trio("rover-2", "a", 1.0, gap=0.9)),
        )
        fmap.update(f1)
        planner.decide(f1, fmap)
        self.assertEqual(planner.advisor_calls, 0)

        f2 = frame(
            2.0,
            robot("rover-1", 1.0, 0.0, 2.0),
            robot(
                "rover-2",
                0.0,
                0.0,
                2.0,
                (sample("rover-2", "a", SensorKind.STAND_GAP, 0.05, 2.0),),
            ),
        )
        fmap.update(f2)
        intents = planner.decide(f2, fmap)
        self.assertEqual(planner.advisor_calls, 1)
        self.assertEqual(advisor.events[0].kind, "quality_overturned")
        by_id = {i.robot_id: i for i in intents}
        self.assertEqual(by_id["rover-1"].target_site_id, "b")
        self.assertTrue(by_id["rover-1"].reason.startswith("advisor:"))

        f3 = frame(
            3.0,
            robot("rover-1", 1.0, 0.0, 3.0, range_m=0.2, link_ok=False),
            robot("rover-2", 0.0, 0.0, 3.0, link_ok=False),
        )
        fmap.update(f3)
        intents_offline = planner.decide(f3, fmap)
        self.assertEqual(planner.advisor_calls, 1)
        self.assertEqual(planner.advisor_fallbacks, 1)
        self.assertEqual(len(intents_offline), 2)

        advisor.bad_proposal = True
        f4 = frame(
            4.0,
            robot("rover-1", 2.0, 0.0, 4.0, range_m=0.2, link_ok=True),
            robot("rover-2", 0.0, 0.0, 4.0, link_ok=True),
        )
        fmap.update(f4)
        intents_safe = planner.decide(f4, fmap)
        self.assertEqual(planner.advisor_fallbacks, 2)
        self.assertNotEqual(
            next(i for i in intents_safe if i.robot_id == "rover-1").mode, TaskMode.PLANT
        )

    def test_end_to_end_episode_runner_with_homogeneous_robots(self):
        class ScriptedSim:
            def __init__(self):
                self.step_idx = 0

            def reset(self, seed: int) -> ObservationFrame:
                self.step_idx = 0
                return frame(
                    0.0,
                    robot("rover-1", 0.0, 0.0, 0.0, trio("rover-1", "a", 0.0, gap=0.9)),
                    robot("rover-2", 1.4, 0.0, 0.0),
                )

            def step(self, commands):
                self.step_idx += 1
                t = float(self.step_idx)
                fb = ()
                if any(c.release_seed for c in commands):
                    plant_cmd = next(c for c in commands if c.release_seed)
                    fb = (
                        PlantFeedback(
                            plant_cmd.robot_id,
                            plant_cmd.task_id,
                            True,
                            t,
                            Pose2D(0.0, 0.0, 0.0),
                        ),
                    )
                return frame(
                    t,
                    robot("rover-1", 0.0, 0.0, t, seeds=1 if fb else 2),
                    robot("rover-2", 1.5, 0.0, t, trio("rover-2", "b", t, gap=0.85)),
                    feedback=fb,
                )

        core = MissionCore(
            MeasuredFieldMap(SITES),
            CooperativeSwarmPlanner(HOMOGENEOUS_CAPS),
        )
        runner = EpisodeRunner(ScriptedSim(), core)
        decisions = runner.run(seed=42, steps=2)
        self.assertEqual(len(decisions), 2)
        self.assertTrue(
            next(c for c in decisions[0].commands if c.robot_id == "rover-1").release_seed
        )
        r2_intent_1 = next(i for i in decisions[1].intents if i.robot_id == "rover-2")
        r1_intent_1 = next(i for i in decisions[1].intents if i.robot_id == "rover-1")
        self.assertEqual(r2_intent_1.mode, TaskMode.PLANT)
        self.assertEqual(r2_intent_1.target_site_id, "b")
        self.assertEqual(r1_intent_1.mode, TaskMode.SAMPLE)
        self.assertEqual(r1_intent_1.target_site_id, "c")


if __name__ == "__main__":
    unittest.main()
