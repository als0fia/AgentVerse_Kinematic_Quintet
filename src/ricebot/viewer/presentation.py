"""Read-only presentation cues; no physics, planner or navigation changes."""
from __future__ import annotations
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class PresentationCue:
    label: str
    detail: str = ""
    victim_id: str | None = None
    helper_id: str | None = None
    phase: str = "fieldwork"


def presentation_cue(frame: Any, decision: Any, *, fault_evidence: Any,
                     rescue_assessments: Any, orchestrator_decisions: Any,
                     completed_recoveries: Any) -> PresentationCue:
    """Derive captions strictly from observations, commands and physical feedback."""
    robots = {r.robot_id: r for r in frame.robots}
    victim = next((r.robot_id for r in frame.robots if r.is_bogged), None)
    order = None
    for rid, value in orchestrator_decisions.items():
        if rid in robots and (victim is None or rid == victim):
            victim, order = rid, value
            break
    helper = getattr(order, "helper_robot_id", None)
    feedback = frame.recovery_feedback[-1] if frame.recovery_feedback else None
    if feedback is not None:
        victim, helper = feedback.target_robot_id, feedback.helper_robot_id
    if completed_recoveries:
        recovery = completed_recoveries[-1]
        # A past recovery must never conceal a newly observed fault elsewhere,
        # or a new post-recovery incident on the same robot.
        new_fault = any(
            r.is_bogged and (
                r.robot_id != recovery.target_robot_id
                or getattr(fault_evidence.get(r.robot_id), "observed_at_s", -1.0)
                > getattr(recovery, "observed_at_s", float("inf"))
            )
            for r in frame.robots
        )
        if not new_fault:
            victim, helper = recovery.target_robot_id, recovery.helper_robot_id
            resumed = (
                victim in robots and not robots[victim].is_bogged
                and (getattr(robots[victim], "true_speed_m_s", 0.0) or 0.0) > 0.025
                and any(i.robot_id == victim
                        and getattr(i.mode, "value", i.mode) in ("sample", "plant")
                        for i in decision.intents)
            )
            if resumed:
                return PresentationCue("Fieldwork resumed", f"{victim.upper()} returned to task",
                                       victim, helper, "resumed")
            return PresentationCue("Extraction verified", "Physical recovery completed",
                                   victim, helper, "extracted")
    if feedback is not None:
        phase = str(getattr(feedback, "phase", "")).lower()
        if feedback.extracted:
            return PresentationCue("Extraction verified", "Physical recovery feedback",
                                   victim, helper, "extracted")
        if feedback.latched and (feedback.victim_displacement_m > 0.01
                                 or "pull" in phase or "extract" in phase):
            return PresentationCue("Extraction in progress", "Latched victim is moving",
                                   victim, helper, "extracting")
        if feedback.latched or "dock" in phase:
            return PresentationCue("Docking", "Physical docking feedback",
                                   victim, helper, "docking")
    if order is not None:
        verdict = getattr(order.verdict, "value", order.verdict)
        if getattr(order, "safety_veto_applied", False):
            return PresentationCue("Unsafe rescue refused", "Safety gate rejected dispatch",
                                   victim, helper, "refused")
        if verdict == "abandon_escalate":
            return PresentationCue("Rescue declined", "No helper dispatched",
                                   victim, helper, "refused")
        if verdict == "defer_rescue":
            return PresentationCue("Rescue deferred", "Dispatch deferred by decision",
                                   victim, helper, "deferred")
        if verdict == "rescue_now" and helper in robots:
            approaching = any(
                i.robot_id == helper and i.target_robot_id == victim
                and getattr(i.mode, "value", i.mode) == "rescue"
                for i in decision.intents
            )
            if approaching:
                return PresentationCue("Rescue approach",
                                       f"{helper.upper()} heading to {victim.upper()}",
                                       victim, helper, "approach")
            return PresentationCue("Helper assigned", f"{helper.upper()} selected",
                                   victim, helper, "assigned")
    if rescue_assessments and (victim is None or victim in rescue_assessments):
        assessed = victim or next(iter(rescue_assessments))
        return PresentationCue("Risk assessed", f"Options evaluated for {assessed.upper()}",
                               assessed, helper, "assessed")
    if victim and victim in fault_evidence and robots[victim].is_bogged:
        return PresentationCue("Fault detected", f"{victim.upper()} immobilised",
                               victim, helper, "fault")
    if victim:
        return PresentationCue("Robot stranded", "Observed loss of mobility",
                               victim, helper, "stranded")
    return PresentationCue("Fieldwork active", "Live sensing, mapping and planting")
