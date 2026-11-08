"""Tests for independent diagnostic judgements; no MuJoCo install required."""
from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ricebot.contracts import (
    Decision, ObservationFrame, Pose2D, RobotCommand, RobotObservation,
    TaskIntent, TaskMode,
)
from ricebot.diagnostics import EpisodeDiagnosticPort, TRACE_VERSION


class FakeSimulation:
    def __init__(self, move_m_per_step: float = 0.0, bogged: bool = False):
        self.t = 0.0
        self.x = 0.0
        self.move_m_per_step = move_m_per_step
        self.bogged = bogged
        self.contacts: list[dict] = []
        self.sinkage = 0.01

    def frame(self):
        return ObservationFrame("test-episode", round(self.t, 4), (
            RobotObservation("planter", Pose2D(round(self.x, 4), 0.0, 0.0),
                             round(self.t, 4), 2.0, 1, is_bogged=self.bogged),
            RobotObservation("scout", Pose2D(0.0, 1.0, 0.0),
                             round(self.t, 4), 2.0, 0),
        ))

    def reset(self, seed: int):
        self.t = 0.0
        self.x = 0.0
        return self.frame()

    def step(self, commands):
        self.t = round(self.t + 0.2, 4)
        self.x += self.move_m_per_step
        return self.frame()


class FakeProbe:
    def __init__(self, sim):
        self.sim = sim

    def snapshot(self):
        sim = self.sim
        return {
            "time_s": sim.t,
            "robots": {
                "planter": {
                    "x_m": sim.x, "y_m": 0.0, "z_m": 0.1,
                    "sinkage_m": sim.sinkage, "slip_ratio": 0.5,
                    "belly_clearance_m": 0.04,
                },
                "scout": {
                    "x_m": 0.0, "y_m": 1.0, "z_m": 0.1,
                    "sinkage_m": 0.01, "slip_ratio": 0.1,
                    "belly_clearance_m": 0.04,
                },
            },
            "contacts": list(sim.contacts),
            "latched_pair": None,
        }


class DiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def make_audit(self, sim, scripted_trap=False):
        path = Path(self.tmp.name) / "trace.jsonl"
        audit = EpisodeDiagnosticPort(
            sim, FakeProbe(sim), path,
            scenario="nominal", policy="centralized", scripted_trap=scripted_trap,
        )
        return audit, path

    def tick(self, sim, audit, *, speed=0.25, rescue=False):
        frame = audit.previous
        commands = (
            RobotCommand("planter", frame.time_s, frame.time_s + 0.4,
                         linear_m_s=speed),
        )
        intents = ()
        if rescue:
            intents = (TaskIntent(
                "rescue1", "scout", TaskMode.RESCUE, frame.time_s,
                frame.time_s + 0.4, target_robot_id="planter",
            ),)
            commands += (RobotCommand(
                "scout", frame.time_s, frame.time_s + 0.4,
                recovery_target_id="planter",
            ),)
        decision = Decision(intents, commands, 0)
        audit.consume(frame, decision)
        audit.step(commands)

    def test_no_progress_and_missing_response_are_separate_findings(self):
        sim = FakeSimulation(move_m_per_step=0)
        audit, path = self.make_audit(sim)
        audit.reset(5)
        for _ in range(14):
            self.tick(sim, audit)
        report = audit.finish()
        self.assertEqual(report["event_counts"].get("commanded_no_progress"), 1)
        self.assertEqual(report["event_counts"].get("stall_without_rescue_response"), 1)
        self.assertTrue(report["independent_immobilization_observed"])
        records = [json.loads(s) for s in path.read_text().splitlines()]
        self.assertEqual(records[0]["version"], TRACE_VERSION)
        self.assertEqual(len([r for r in records if r["kind"] == "tick"]), 14)
        self.assertEqual(records[-1]["kind"], "summary")

    def test_hold_and_turning_do_not_count_as_commanded_stall(self):
        sim = FakeSimulation()
        audit, _ = self.make_audit(sim)
        audit.reset(1)
        for _ in range(12):
            self.tick(sim, audit, speed=0.0)
        report = audit.finish()
        self.assertNotIn("commanded_no_progress", report["event_counts"])
        self.assertIn("no_natural_immobilization_observed", report["event_counts"])

    def test_physical_motion_overrides_bogged_flag_in_diagnostics(self):
        sim = FakeSimulation(move_m_per_step=0.05, bogged=True)
        audit, _ = self.make_audit(sim)
        audit.reset(1)
        for _ in range(12):
            self.tick(sim, audit)
        report = audit.finish()
        self.assertNotIn("commanded_no_progress", report["event_counts"])

    def test_rescue_response_and_return_to_motion(self):
        sim = FakeSimulation()
        audit, _ = self.make_audit(sim)
        audit.reset(7)
        for _ in range(8):
            self.tick(sim, audit, rescue=True)
        sim.move_m_per_step = 0.08
        for _ in range(6):
            self.tick(sim, audit, rescue=True)
        report = audit.finish()
        self.assertIn("commanded_no_progress", report["event_counts"])
        self.assertNotIn("stall_without_rescue_response", report["event_counts"])
        self.assertIn("mobility_resumed", report["event_counts"])

    def test_unplanned_contact_deduplicates_and_rescue_contact_is_labelled(self):
        sim = FakeSimulation(move_m_per_step=0.04)
        audit, _ = self.make_audit(sim)
        audit.reset(1)
        sim.contacts = [{
            "actors": ["planter", "scout"], "geoms": ["planter_chassis", "scout_chassis"],
            "penetration_m": 0.007, "normal_force_n": 45.0,
        }]
        self.tick(sim, audit)
        self.tick(sim, audit)
        sim.contacts = []
        self.tick(sim, audit)
        sim.contacts = [{
            "actors": ["planter", "scout"], "geoms": ["planter_dock", "scout_latch"],
            "penetration_m": 0.002, "normal_force_n": 8.0,
        }]
        self.tick(sim, audit, rescue=True)
        report = audit.finish()
        self.assertEqual(report["event_counts"].get("unexpected_robot_collision"), 1)
        self.assertEqual(report["event_counts"].get("robot_contact_during_rescue"), 1)
        self.assertEqual(report["max_sampled_contact_force_n"], 45.0)

    def test_command_mismatch_fails_instead_of_producing_false_evidence(self):
        sim = FakeSimulation()
        audit, _ = self.make_audit(sim)
        frame = audit.reset(1)
        decision = Decision((), (RobotCommand("planter", 0, 1),), 0)
        audit.consume(frame, decision)
        with self.assertRaisesRegex(ValueError, "mismatch"):
            audit.step((RobotCommand("planter", 0, 1, linear_m_s=0.2),))
        audit.finish()


if __name__ == "__main__":
    unittest.main()
