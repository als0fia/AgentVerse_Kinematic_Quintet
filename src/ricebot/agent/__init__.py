"""Swarm Agentic Coordination & Decentralised Mycelial SLAM Package (`src/ricebot/agent/`).

Exports:
1. Teammate Tier-1/Tier-2 Swarm Auction & Rescue Triage (`planner.py`):
   - `CooperativeSwarmPlanner`, `SwarmRobotAgent`, `AgentBid`, `RescueVerdict`,
     `RescueAssessment`, `GeminiSwarmAdvisor`, `SupervisoryAdvisor`, `AmbiguityEvent`,
     `TravelCostFn`, `homogeneous_capabilities`
2. Policy B — Zero-Central-Map Mycelial SLAM, UWB/HaLow Gossip & Local Auction (`mycelial_swarm.py`):
   - `DecentralizedMycelialRuntime`, `DecentralizedRobotNode`, `LocalMycelialMap`,
     `LocalSlamGrid`, `MycelialGossipPacket`, `OnboardSensorTelemetry`, `SlamHazardCell`,
     `SoilSafetyEnvelope`, `TaskBidProposal`, `GaussianChannelBelief`
"""
from .mycelial_swarm import (
    DecentralizedMycelialRuntime,
    DecentralizedRobotNode,
    GaussianChannelBelief,
    LocalMycelialMap,
    LocalSlamGrid,
    MycelialGossipPacket,
    OnboardSensorTelemetry,
    SlamHazardCell,
    SoilSafetyEnvelope,
    TaskBidProposal,
)
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
    "DecentralizedMycelialRuntime",
    "DecentralizedRobotNode",
    "GaussianChannelBelief",
    "GeminiSwarmAdvisor",
    "LocalMycelialMap",
    "LocalSlamGrid",
    "MycelialGossipPacket",
    "OnboardSensorTelemetry",
    "RescueAssessment",
    "RescueVerdict",
    "SlamHazardCell",
    "SoilSafetyEnvelope",
    "SupervisoryAdvisor",
    "SwarmRobotAgent",
    "TaskBidProposal",
    "TravelCostFn",
    "homogeneous_capabilities",
]
