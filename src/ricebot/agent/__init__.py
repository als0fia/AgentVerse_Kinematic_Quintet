"""Swarm Agentic Coordination package (Task Allocation + Rescue Triage).

Consumes your mapping teammate's MeasuredFieldMap and outputs TaskIntent for
your traversal teammate's LocalController.
"""
from .planner import (
    AgentBid,
    AmbiguityEvent,
    CooperativeSwarmPlanner,
    GeminiSwarmAdvisor,
    RescueAssessment,
    RescueVerdict,
    SupervisoryAdvisor,
    SwarmRobotAgent,
    TravelCostFn,
    homogeneous_capabilities,
)

__all__ = [
    "AgentBid",
    "AmbiguityEvent",
    "CooperativeSwarmPlanner",
    "GeminiSwarmAdvisor",
    "RescueAssessment",
    "RescueVerdict",
    "SupervisoryAdvisor",
    "SwarmRobotAgent",
    "TravelCostFn",
    "homogeneous_capabilities",
]
