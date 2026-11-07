"""Swarm Agentic Coordination & Decentralised Mycelial SLAM Package (`src/ricebot/agent/`).

Exports:
1. Teammate Tier-1/Tier-2 Swarm Auction, Failure Detection & Mechanical Recovery (`planner.py` & `rescue.py`):
   - `CooperativeSwarmPlanner`, `SwarmRobotAgent`, `AgentBid`, `RescueVerdict`,
     `RescueAssessment`, `RescueEngine`, `FailureMode`, `RecoveryAction`,
     `MechanicalRescueTechnique`, `MechanicalActuatorSetpoint`, `DistressSignal`,
     `compute_mechanical_recovery_setpoint`,
     `GeminiSwarmAdvisor`, `SupervisoryAdvisor`, `AmbiguityEvent`,
     `TravelCostFn`, `homogeneous_capabilities`
2. Policy B — Zero-Central-Map Mycelial SLAM, UWB/ESP-NOW Gossip & Local Auction (`mycelial_swarm.py`):
   - `DecentralizedMycelialRuntime`, `DecentralizedRobotNode`, `LocalMycelialMap`,
     `LocalSlamGrid`, `MycelialGossipPacket`, `OnboardSensorTelemetry`, `SlamHazardCell`,
     `SoilSafetyEnvelope`, `TaskBidProposal`, `GaussianChannelBelief`,
     `evaluate_continuous_belief_field`, `sample_continuous_belief_grid`
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
    evaluate_continuous_belief_field,
    sample_continuous_belief_grid,
)
from .planner import (
    AgentBid,
    AmbiguityEvent,
    CooperativeSwarmPlanner,
    GeminiSwarmAdvisor,
    SupervisoryAdvisor,
    SwarmRobotAgent,
    TravelCostFn,
    homogeneous_capabilities,
)
from .rescue import (
    DistressSignal,
    FailureMode,
    MechanicalActuatorSetpoint,
    MechanicalRescueTechnique,
    RecoveryAction,
    RescueAssessment,
    RescueEngine,
    RescueVerdict,
    compute_mechanical_recovery_setpoint,
)

__all__ = [
    "AgentBid",
    "AmbiguityEvent",
    "CooperativeSwarmPlanner",
    "DecentralizedMycelialRuntime",
    "DecentralizedRobotNode",
    "DistressSignal",
    "FailureMode",
    "GaussianChannelBelief",
    "GeminiSwarmAdvisor",
    "LocalMycelialMap",
    "LocalSlamGrid",
    "MechanicalActuatorSetpoint",
    "MechanicalRescueTechnique",
    "MycelialGossipPacket",
    "OnboardSensorTelemetry",
    "RecoveryAction",
    "RescueAssessment",
    "RescueEngine",
    "RescueVerdict",
    "SlamHazardCell",
    "SoilSafetyEnvelope",
    "SupervisoryAdvisor",
    "SwarmRobotAgent",
    "TaskBidProposal",
    "TravelCostFn",
    "compute_mechanical_recovery_setpoint",
    "evaluate_continuous_belief_field",
    "homogeneous_capabilities",
    "sample_continuous_belief_grid",
]
