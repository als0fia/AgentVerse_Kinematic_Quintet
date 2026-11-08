"""Contract tests only. Scripted observations do not prove Isaac Sim physics."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from ricebot.contracts import (
    FaultEvidence, ObservationFrame, OperatorCommandKind, OperatorRequest,
    PlantFeedback, Pose2D, RescueAssessment, RescueVerdict, RobotCapability,
    RobotCommand, RobotObservation, SensorKind, Site, SoilSample, TaskIntent, TaskMode,
)
from ricebot.control import LocalController
from ricebot.mission import MeasuredFieldMap, SimpleMissionPlanner
from ricebot.runtime import (
    EpisodeConfig, EpisodeRunner, EpisodeSession, MissionCore,
    ReplaySimulationPort, create_mujoco_episode_session,
)


SITES = (
    Site('a', Pose2D(0.0, 0.0, 0.0)),
    Site('b', Pose2D(2.0, 0.0, 0.0)),
    Site('protected', Pose2D(1.0, 1.0, 0.0), permitted=False),
)
CAPS = (RobotCapability('planter', can_sample=True, can_plant=True),
        RobotCapability('scout', can_sample=True, can_plant=False))


def sample(actor: str, site: str, kind: SensorKind, value: float,
           t: float, suffix: str = '') -> SoilSample:
    return SoilSample(f'{actor}:{site}:{kind.value}:{t}:{suffix}', actor, site, kind, value, t,
                      next(s.center for s in SITES if s.site_id == site))


def trio(actor: str, site: str, t: float, gap: float = 0.9,
         moisture: float = 0.5, firmness: float = 0.55) -> tuple[SoilSample, ...]:
    return (sample(actor, site, SensorKind.MOISTURE, moisture, t),
            sample(actor, site, SensorKind.FIRMNESS, firmness, t),
            sample(actor, site, SensorKind.STAND_GAP, gap, t))


def robot(actor: str, x: float, y: float, t: float, values=(),
          range_m: float | None = 4.0, seeds: int = 2,
          link_ok: bool = True) -> RobotObservation:
    return RobotObservation(actor, Pose2D(x, y, 0.0), t, range_m,
                            seeds, tuple(values), link_ok)


def frame(t: float, *robots: RobotObservation,
          feedback: tuple[PlantFeedback, ...] = ()) -> ObservationFrame:
    return ObservationFrame('episode-7', t, tuple(robots), feedback)


class ContractTest(unittest.TestCase):
    def test_reject_invalid_observation_and_command(self):
        with self.assertRaises(ValueError):
            sample('scout', 'a', SensorKind.MOISTURE, 1.3, 0)
        with self.assertRaises(ValueError):
            robot('scout', 0, 0, 1.0, trio('scout', 'a', 2.0))
        with self.assertRaises(ValueError):
            RobotCommand('planter', 1, 1, linear_m_s=0.1)
        with self.assertRaises(ValueError):
            RobotCommand('planter', 1, 2, release_seed=True, target_site_id='a', linear_m_s=0.1)
        with self.assertRaises(ValueError):
            RobotCommand('planter', 1, 2, release_seed=True, target_site_id='a')
        with self.assertRaises(ValueError):
            frame(0, robot('scout', 0, 0, 0), robot('scout', 0, 0, 0))

    def test_samples_from_two_robots_fuse_but_are_not_hidden_map(self):
        field = MeasuredFieldMap(SITES)
        first = frame(1, robot('planter', 0, 0, 1, trio('planter', 'a', 1, moisture=0.4)),
                      robot('scout', 0.2, 0, 1, trio('scout', 'a', 1, moisture=0.6)))
        self.assertEqual(field.update(first), 6)
        estimate = field.estimate('a')
        self.assertAlmostEqual(estimate.moisture, 0.5)
        self.assertEqual(estimate.source_robot_ids, ('planter', 'scout'))
        self.assertIsNone(field.estimate('b').moisture)
        self.assertEqual(field.update(first), 0)

    def test_far_away_and_invalid_samples_cannot_reveal_field(self):
        field = MeasuredFieldMap(SITES, sample_radius_m=0.5)
        invalid = SoilSample('invalid', 'planter', 'a', SensorKind.MOISTURE, 0.1, 0, SITES[0].center, False)
        remote = sample('scout', 'a', SensorKind.FIRMNESS, 0.7, 0)
        self.assertEqual(field.update(frame(0, robot('planter', 0, 0, 0, (invalid,)),
                                           robot('scout', 2, 0, 0, (remote,)))), 0)
        self.assertFalse(field.estimate('a').complete)
        field.update(frame(1, robot('planter', 0, 0, 1, (sample('planter', 'a', SensorKind.MOISTURE, .4, 1),))))
        conflict = SoilSample('planter:a:moisture_proxy:1:', 'planter', 'a', SensorKind.MOISTURE, 0.5, 1, SITES[0].center)
        with self.assertRaises(ValueError):
            field.update(frame(1, robot('planter', 0, 0, 1, (conflict,))))

    def test_unknown_site_and_protected_site_not_targets(self):
        planner = SimpleMissionPlanner(CAPS)
        field = MeasuredFieldMap(SITES)
        result = planner.decide(frame(0, robot('planter', 0, 0, 0), robot('scout', 2, 0, 0)), field)
        self.assertEqual({i.mode for i in result}, {TaskMode.SAMPLE})
        self.assertEqual({i.target_site_id for i in result}, {'a', 'b'})
        self.assertNotIn('protected', {i.target_site_id for i in result})

    def test_fault_evidence_contract_and_negative_controls(self):
        valid_stall = FaultEvidence(
            robot_id='planter',
            observed_at_s=4.5,
            commanded_linear_m_s=0.35,
            commanded_angular_rad_s=0.0,
            observed_displacement_m=0.001,
            observed_yaw_delta_rad=0.001,
            observed_speed_m_s=0.005,
            slip_ratio=0.92,
            front_range_m=2.5,
            link_ok=True,
            stalled_ticks=3,
            is_commanded_motion_stall=True,
            confidence=0.95,
        )
        self.assertTrue(valid_stall.is_commanded_motion_stall)
        # Negative control 1: intentional HOLD (zero command) cannot claim motion stall
        with self.assertRaises(ValueError):
            FaultEvidence(
                robot_id='planter',
                observed_at_s=4.5,
                commanded_linear_m_s=0.0,
                commanded_angular_rad_s=0.0,
                is_commanded_motion_stall=True,
                front_range_m=2.5,
            )
        # Negative control 2: missing rangefinder (None) is sensor invalidity, not motion stall
        with self.assertRaises(ValueError):
            FaultEvidence(
                robot_id='planter',
                observed_at_s=4.5,
                commanded_linear_m_s=0.30,
                front_range_m=None,
                is_commanded_motion_stall=True,
            )

    def test_rescue_assessment_bidirectional_field_normalization(self):
        from_policy_a = RescueAssessment(
            stranded_robot_id='planter',
            verdict=RescueVerdict.RESCUE_NOW,
            nearest_site_id='patch_d',
            rescue_benefit=1.30,
            opportunity_cost=0.64,
            hazard_risk=0.0,
            net_rescue_value=0.66,
            helper_robot_id='scout',
            reason='worth saving now',
        )
        self.assertEqual(from_policy_a.rescuer_robot_id, 'scout')
        self.assertAlmostEqual(from_policy_a.expected_saved_benefit, 1.30)
        self.assertAlmostEqual(from_policy_a.rescue_opportunity_cost, 0.64)
        self.assertAlmostEqual(from_policy_a.net_rescue_utility, 0.66)
        self.assertEqual(from_policy_a.rationale, 'worth saving now')

        from_policy_b = RescueAssessment(
            stranded_robot_id='planter',
            verdict='rescue_now',  # type: ignore[arg-type]
            nearest_site_id='patch_d',
            rescuer_robot_id='scout',
            expected_saved_benefit=4.50,
            rescue_opportunity_cost=0.42,
            net_rescue_utility=4.08,
            rationale='Peer rescue saves 4.50',
        )
        self.assertEqual(from_policy_b.verdict, RescueVerdict.RESCUE_NOW)
        self.assertEqual(from_policy_b.helper_robot_id, 'scout')
        self.assertAlmostEqual(from_policy_b.rescue_benefit, 4.50)
        self.assertAlmostEqual(from_policy_b.opportunity_cost, 0.42)
        self.assertAlmostEqual(from_policy_b.net_rescue_value, 4.08)
        self.assertEqual(from_policy_b.reason, 'Peer rescue saves 4.50')


class MissionTest(unittest.TestCase):
    def setUp(self):
        self.field = MeasuredFieldMap(SITES)
        self.core = MissionCore(self.field, SimpleMissionPlanner(CAPS))

    def test_observed_gap_allows_planter_not_scout_to_drop(self):
        first = frame(1, robot('planter', 0, 0, 1), robot('scout', 0, 0, 1, trio('scout', 'a', 1)))
        decision = self.core.decide(first)
        plant = next(i for i in decision.intents if i.robot_id == 'planter')
        self.assertEqual(plant.mode, TaskMode.PLANT)
        command = next(c for c in decision.commands if c.robot_id == 'planter')
        self.assertTrue(command.release_seed)
        self.assertEqual(command.target_site_id, 'a')
        self.assertEqual(command.linear_m_s, 0.0)
        self.assertFalse(next(c for c in decision.commands if c.robot_id == 'scout').release_seed)
        second = self.core.decide(frame(1.5, robot('planter', 0, 0, 1.5), robot('scout', 0, 0, 1.5)))
        self.assertFalse(next(c for c in second.commands if c.robot_id == 'planter').release_seed)
        ack = PlantFeedback('planter', command.task_id, True, 2, Pose2D(.02, 0, 0))
        third = self.core.decide(frame(2, robot('planter', 0, 0, 2),
                                       robot('scout', 0, 0, 2), feedback=(ack,)))
        self.assertNotIn('a', {i.target_site_id for i in third.intents})

    def test_unavailable_range_and_obstacle_safe_stop(self):
        site = {s.site_id: s for s in SITES}
        task = TaskIntent('go', 'planter', TaskMode.PLANT, 0, 2, 'a')
        ctl = LocalController()
        cmd = ctl.command(robot('planter', 0, 0, 1, range_m=None), task, site, CAPS[0])
        self.assertEqual((cmd.linear_m_s, cmd.angular_rad_s, cmd.release_seed), (0, 0, False))
        cmd = ctl.command(robot('planter', 1, 0, 1, range_m=0.1), task, site, CAPS[0])
        self.assertEqual((cmd.linear_m_s, cmd.angular_rad_s, cmd.release_seed), (0, 0, False))
        cmd = ctl.command(robot('planter', 1, 0, 2, range_m=10), task, site, CAPS[0])
        self.assertEqual((cmd.linear_m_s, cmd.angular_rad_s, cmd.release_seed), (0, 0, False))
        cmd = ctl.command(robot('planter', 0, 0, 1, link_ok=False), task, site, CAPS[0])
        self.assertFalse(cmd.release_seed)

    def test_changed_measured_quality_changes_plant_decision(self):
        first = self.core.decide(frame(1, robot('planter', 1.5, 0, 1),
                                       robot('scout', 0, 0, 1, trio('scout', 'a', 1, gap=0.9))))
        self.assertEqual(next(i for i in first.intents if i.robot_id == 'planter').mode, TaskMode.PLANT)
        changed = sample('scout', 'a', SensorKind.STAND_GAP, .1, 2)
        second = self.core.decide(frame(2, robot('planter', 1.5, 0, 2),
                                        robot('scout', 0, 0, 2, (changed,))))
        self.assertAlmostEqual(self.field.estimate('a').stand_gap, 0.5)
        self.assertNotEqual(next(i for i in second.intents if i.robot_id == 'planter').mode, TaskMode.PLANT)

    def test_no_go_straight_line_stops_for_route_planner(self):
        ctl = LocalController()
        sites = {'a': SITES[0], 'b': SITES[1],
                 'blocked': Site('blocked', Pose2D(1, 0, 0), permitted=False, no_go_radius_m=.5)}
        task = TaskIntent('navigate', 'planter', TaskMode.SAMPLE, 0, 2, 'b')
        cmd = ctl.command(robot('planter', 0, 0, 1), task, sites, CAPS[0])
        self.assertEqual((cmd.linear_m_s, cmd.angular_rad_s, cmd.release_seed), (0, 0, False))

    def test_missing_plant_ack_times_out_without_repeated_release(self):
        first = self.core.decide(frame(1, robot('planter', 0, 0, 1),
                                       robot('scout', 0, 0, 1, trio('scout', 'a', 1))))
        self.assertTrue(first.commands[0].release_seed)
        second = self.core.decide(frame(3.1, robot('planter', 0, 0, 3.1),
                                        robot('scout', 0, 0, 3.1)))
        self.assertFalse(next(c for c in second.commands if c.robot_id == 'planter').release_seed)
        self.assertNotIn('a', {i.target_site_id for i in second.intents})

    def test_episode_clock_and_actor_registry_guard(self):
        self.core.decide(frame(0, robot('planter', 0, 0, 0)))
        with self.assertRaises(ValueError):
            self.core.decide(frame(0, robot('planter', 0, 0, 0)))
        with self.assertRaises(ValueError):
            self.core.decide(frame(1, robot('unknown', 0, 0, 1)))

    def test_real_runner_boundary_with_scripted_fixture_not_physics(self):
        class FakeSimulator:
            def __init__(self):
                self.seen = []
                self.tick = 0
            def reset(self, seed):
                self.tick = 0
                return frame(0, robot('planter', 0, 0, 0, range_m=3))
            def step(self, commands):
                self.seen.append(commands)
                self.tick += 1
                return frame(self.tick, robot('planter', 0, 0, self.tick, range_m=0.1))
        class SpyView:
            def __init__(self):
                self.snapshots = []
            def consume(self, f, d):
                self.snapshots.append((f.time_s, len(d.commands)))
        fake, view = FakeSimulator(), SpyView()
        runner = EpisodeRunner(fake, MissionCore(MeasuredFieldMap(SITES), SimpleMissionPlanner(CAPS)), view)
        decisions = runner.run(seed=7, steps=2)
        self.assertEqual(len(fake.seen), 2)
        self.assertEqual(view.snapshots, [(0, 1), (1, 1)])
        self.assertEqual(decisions[0].intents[0].mode, TaskMode.SAMPLE)
        self.assertEqual((decisions[1].commands[0].linear_m_s, decisions[1].commands[0].angular_rad_s), (0, 0))
        self.assertFalse(decisions[1].commands[0].release_seed)

    def test_episode_session_step_once_vs_batch_run_parity_and_replay(self):
        class ScriptedSim:
            def __init__(self):
                self.step_count = 0
            def reset(self, seed: int):
                self.step_count = 0
                return frame(0.0, robot('planter', 0.0, 0.0, 0.0), robot('scout', 0.0, 0.0, 0.0, trio('scout', 'a', 0.0)))
            def step(self, commands):
                self.step_count += 1
                t = round(self.step_count * 0.5, 3)
                return frame(t, robot('planter', 0.1 * self.step_count, 0.0, t), robot('scout', 0.2 * self.step_count, 0.0, t))

        sim_batch = ScriptedSim()
        core_batch = MissionCore(MeasuredFieldMap(SITES), SimpleMissionPlanner(CAPS))
        session_batch = EpisodeSession(sim_batch, core_batch, seed=7, scenario='nominal', policy_name='centralized')
        decisions_batch = session_batch.run(seed=7, steps=4)
        trace_batch = session_batch.export_trace()

        sim_interactive = ScriptedSim()
        core_interactive = MissionCore(MeasuredFieldMap(SITES), SimpleMissionPlanner(CAPS))
        shadow_core = MissionCore(MeasuredFieldMap(SITES), SimpleMissionPlanner(CAPS))
        session_interactive = EpisodeSession(
            sim_interactive,
            core_interactive,
            shadow_policy=shadow_core,
            seed=7,
            scenario='nominal',
            policy_name='centralized',
        )
        session_interactive.reset(seed=7)
        req = OperatorRequest('req-1', OperatorCommandKind.SET_COMMS_MODE, 0.0, payload=(('comm_mode', 'intermittent'),))
        session_interactive.submit_operator_request(req)
        for _ in range(4):
            session_interactive.step_once()
        trace_interactive = session_interactive.export_trace()

        self.assertEqual(sim_batch.step_count, 4)
        self.assertEqual(sim_interactive.step_count, 4)
        self.assertEqual(len(decisions_batch), 4)
        self.assertTrue(trace_batch.matches_parity(trace_interactive))
        self.assertEqual(len(trace_interactive.ticks[0].operator_requests), 1)
        self.assertEqual(trace_interactive.ticks[0].operator_requests[0].request_id, 'req-1')

        # Verify ReplaySimulationPort deterministic replay
        replay_port = ReplaySimulationPort(trace_batch, verify_commands=True)
        replay_core = MissionCore(MeasuredFieldMap(SITES), SimpleMissionPlanner(CAPS))
        replay_session = EpisodeSession(replay_port, replay_core, seed=7, scenario='nominal', policy_name='centralized')
        replay_session.run(seed=7, steps=4)
        self.assertTrue(trace_batch.matches_parity(replay_session.export_trace()))

    def test_mujoco_session_batch_vs_interactive_step_parity(self):
        cfg_batch = EpisodeConfig(scenario='golden_rescue', policy='cooperative', seed=7, steps=65)
        session_batch = create_mujoco_episode_session(cfg_batch)
        session_batch.run(seed=7, steps=65)
        trace_batch = session_batch.export_trace()

        cfg_gui = EpisodeConfig(
            scenario='golden_rescue',
            policy='cooperative',
            seed=7,
            steps=65,
            include_shadow_mycelial=True,
        )
        session_gui = create_mujoco_episode_session(cfg_gui)
        session_gui.reset(seed=7)
        for _ in range(65):
            session_gui.step_once()
        trace_gui = session_gui.export_trace()

        self.assertEqual(len(trace_batch.ticks), 65)
        self.assertTrue(trace_batch.matches_parity(trace_gui))
        self.assertIn('planter', session_gui.rescue_assessments)
        assessment = session_gui.rescue_assessments['planter']
        self.assertEqual(assessment.helper_robot_id, 'scout')
        self.assertEqual(assessment.rescuer_robot_id, 'scout')
        self.assertGreater(assessment.net_rescue_value, 0.0)
        self.assertEqual(assessment.net_rescue_value, assessment.net_rescue_utility)


if __name__ == '__main__':
    unittest.main()
