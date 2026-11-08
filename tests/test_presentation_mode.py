"""Read-only presentation-cue tests; no OpenGL display required."""
from types import SimpleNamespace as NS
import unittest

from ricebot.viewer.presentation import presentation_cue


def cue(*, bogged=True, intents=(), verdict=None, helper="planter",
        veto=False, feedback=(), recoveries=(), assessments=None):
    frame = NS(robots=(NS(robot_id="scout", is_bogged=bogged),
                       NS(robot_id="planter", is_bogged=False)),
               recovery_feedback=feedback)
    orders = {} if verdict is None else {
        "scout": NS(verdict=verdict, helper_robot_id=helper,
                    safety_veto_applied=veto)
    }
    return presentation_cue(
        frame, NS(intents=intents),
        fault_evidence={"scout": object()} if bogged else {},
        rescue_assessments=assessments or {},
        orchestrator_decisions=orders, completed_recoveries=recoveries
    )


class TestPresentationCues(unittest.TestCase):
    def test_nominal_does_not_invent_incident(self):
        self.assertEqual(cue(bogged=False).phase, "fieldwork")

    def test_fault_then_assessment(self):
        self.assertEqual(cue().label, "Fault detected")
        self.assertEqual(cue(assessments={"scout": object()}).label,
                         "Risk assessed")

    def test_assignment_is_not_docking(self):
        self.assertEqual(cue(verdict="rescue_now").label, "Helper assigned")
        intent = NS(robot_id="planter", target_robot_id="scout", mode="rescue")
        self.assertEqual(cue(verdict="rescue_now", intents=(intent,)).label,
                         "Rescue approach")

    def test_refusal_requires_decision(self):
        self.assertEqual(cue(verdict="rescue_now", veto=True).label,
                         "Unsafe rescue refused")
        self.assertEqual(cue(verdict="defer_rescue").label, "Rescue deferred")

    def test_latch_and_extraction_need_physical_feedback(self):
        feedback = NS(target_robot_id="scout", helper_robot_id="planter",
                      latched=True, extracted=False,
                      victim_displacement_m=0.0, phase="dock")
        self.assertEqual(cue(feedback=(feedback,)).label, "Docking")
        feedback.victim_displacement_m = 0.03
        self.assertEqual(cue(feedback=(feedback,)).label,
                         "Extraction in progress")

    def test_previous_recovery_does_not_hide_new_fault(self):
        rec = NS(target_robot_id="scout", helper_robot_id="planter",
                 observed_at_s=1.0)
        frame = NS(
            robots=(NS(robot_id="scout", is_bogged=False),
                    NS(robot_id="planter", is_bogged=True)),
            recovery_feedback=(),
        )
        outcome = presentation_cue(
            frame, NS(intents=()), fault_evidence={"planter": NS(observed_at_s=2.0)},
            rescue_assessments={}, orchestrator_decisions={},
            completed_recoveries=(rec,),
        )
        self.assertEqual(outcome.label, "Fault detected")
        self.assertEqual(outcome.victim_id, "planter")

    def test_fieldwork_resumption_requires_recovered_robot_task(self):
        rec = NS(target_robot_id="scout", helper_robot_id="planter")
        other = NS(robot_id="planter", mode="plant")
        scout = NS(robot_id="scout", mode="sample")
        self.assertEqual(cue(bogged=False, recoveries=(rec,),
                             intents=(other,)).label, "Extraction verified")
        self.assertEqual(cue(bogged=False, recoveries=(rec,),
                             intents=(scout,)).label, "Fieldwork resumed")
        self.assertEqual(cue(bogged=True, recoveries=(rec,),
                             intents=(scout,)).label, "Extraction verified")


if __name__ == "__main__":
    unittest.main()
