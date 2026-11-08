"""Independent regression checks for the release candidate."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import unittest
from unittest.mock import patch
from ricebot.runtime import EpisodeConfig, create_mujoco_episode_session
from ricebot.agent import DecentralizedMycelialRuntime
from ricebot.evaluation import run_canonical_episode
from ricebot.agent.rescue import ResidualRecoveryLearner

class ReleaseVerificationTests(unittest.TestCase):
    def test_explicit_policy_is_honoured_in_every_scenario(self):
        for scenario in ('nominal', 'randomized', 'golden_rescue', 'high_risk_abstain'):
            with self.subTest(scenario=scenario):
                cfg=EpisodeConfig(scenario=scenario, policy='mycelial', steps=2)
                session=create_mujoco_episode_session(cfg)
                self.assertIsInstance(session.policy, DecentralizedMycelialRuntime)
                self.assertEqual(session.policy_name, 'mycelial')

    def test_reseed_matches_fresh_scaled_world(self):
        cfg=EpisodeConfig(scenario='randomized',policy='mycelial',num_rovers=4,map_scale=1.6,seed=3,steps=2)
        session=create_mujoco_episode_session(cfg)
        session.reset(3)
        session.step_once()
        frame=session.reset(8)
        fresh=create_mujoco_episode_session(EpisodeConfig(scenario='randomized',policy='mycelial',num_rovers=4,map_scale=1.6,seed=8,steps=2))
        expected=fresh.reset(8)
        self.assertEqual(session.sim.sites,fresh.sim.sites)
        self.assertEqual(frame,expected)
        self.assertEqual(session.config.seed,8)
        self.assertEqual(session.step_once().decision, fresh.step_once().decision)

    def test_mycelial_evaluation_applies_requested_shared_learner(self):
        cfg=EpisodeConfig(policy='mycelial',num_rovers=4,map_scale=1.6,steps=2)
        session=create_mujoco_episode_session(cfg)
        learner=ResidualRecoveryLearner(learning_rate=0.0)
        with patch('ricebot.evaluation.create_mujoco_episode_session',return_value=session):
            run_canonical_episode(cfg,shared_learner=learner)
        self.assertIs(session.policy.orchestrator.tool_belt.learner,learner)

    def test_turning_progress_is_not_a_stall_but_blocked_rotation_is(self):
        from ricebot.contracts import ObservationFrame, RobotObservation, Pose2D
        from ricebot.mujoco_sim.adapter import DEFAULT_CAPABILITIES, DEFAULT_SITES
        for yaw, stalled in ((0.15, False), (0.0, True)):
            runtime=DecentralizedMycelialRuntime(sites=DEFAULT_SITES,capabilities=DEFAULT_CAPABILITIES)
            node=runtime.nodes['planter']
            node.last_pose=Pose2D(0.0,0.0,0.0)
            node.last_time_s=0.0
            node.last_commanded_angular_rad_s=1.0
            frame=ObservationFrame('rotation',0.1,(
                RobotObservation('planter',Pose2D(0.0,0.0,yaw),0.1,1.5,2),
                RobotObservation('scout',Pose2D(1.0,0.0,0.0),0.1,1.5,0),
            ))
            runtime.decide(frame)
            evidence=runtime.last_fault_evidence['planter']
            self.assertEqual(evidence.is_commanded_motion_stall,stalled)
            self.assertEqual(evidence.commanded_angular_rad_s,1.0)
            self.assertAlmostEqual(evidence.observed_yaw_delta_rad,yaw)

    def test_same_seed_reset_replays_physical_motion_and_commands(self):
        for policy in ('mycelial','cooperative'):
            session=create_mujoco_episode_session(EpisodeConfig(policy=policy,num_rovers=4,map_scale=1.6,seed=8))
            session.run(seed=8,steps=25)
            expected=[(t.frame,t.decision) for t in session.ticks]
            session.run(seed=8,steps=25)
            self.assertEqual([(t.frame,t.decision) for t in session.ticks],expected)

    def test_evaluation_denominators_and_exported_duration_are_consistent(self):
        record=run_canonical_episode(EpisodeConfig(policy='cooperative',steps=3))
        self.assertEqual(record.patches_total,record.viable_sites_total+record.non_viable_sites_total)
        self.assertEqual(record.to_schema_record()['elapsed_ms'],record.episode_wall_elapsed_ms)

    def test_robot_contact_alone_is_not_verified_latching(self):
        from ricebot.diagnostics import EpisodeDiagnosticPort
        original=EpisodeDiagnosticPort.finish
        def contact_only(diag):
            result=original(diag)
            result['event_counts']['robot_contact_during_rescue']=1
            return result
        with patch.object(EpisodeDiagnosticPort,'finish',contact_only):
            record=run_canonical_episode(EpisodeConfig(policy='cooperative',steps=3))
        self.assertFalse(record.physical_latch_engaged)
        self.assertEqual(record.physical_extractions_verified,0)

    def test_extracted_feedback_without_subsequent_motion_is_not_mobility(self):
        from types import SimpleNamespace as Obj
        from ricebot.evaluation import _verified_post_recovery_mobility
        from ricebot.contracts import Pose2D
        rec=Obj(extracted=True,target_robot_id='planter',observed_at_s=1.0)
        def tick(t,x):
            return Obj(frame=Obj(time_s=t,robots=[Obj(robot_id='planter',pose=Pose2D(x,0,0))]),decision=Obj(commands=[Obj(robot_id='planter',linear_m_s=0.2)]))
        self.assertEqual(_verified_post_recovery_mobility([tick(1.1,0),tick(1.2,0)],[rec]),set())
        self.assertEqual(_verified_post_recovery_mobility([tick(1.1,0),tick(1.2,0.1)],[rec]),{'planter'})
