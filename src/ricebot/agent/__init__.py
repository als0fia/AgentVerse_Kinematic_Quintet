"""Swarm Agentic Coordination & Decentralised Mycelial SLAM Package (`src/ricebot/agent/`).

Exports:
1. Teammate Tier-1/Tier-2 Swarm Auction & Rescue Triage (`planner.py`):
   - `CooperativeSwarmPlanner`, `SwarmRobotAgent`, `AgentBid`, `RescueVerdict`,
     `RescueAssessment`, `GeminiSwarmAdvisor`, `SupervisoryAdvisor`, `AmbiguityEvent`,
     `TravelCostFn`, `homogeneous_capabilities`
2. Policy B — Zero-Central-Map Mycelial SLAM, UWB/HaLow Gossip & Local Auction (`mycelial_swarm.py`):
   - `DecentralizedMycelialRuntime`, `DecentralizedRobotNode`, `LocalMycelialMap`,
     `LocalSlamGrid`, `MycelialGossipPacket`, `OnboardSensorTelemetry`, `SlamHazardCell`,
     `SoilSafetyEnvelope`, `TaskBidProposal`, `GaussianChannelBelief`,
     `evaluate_continuous_belief_field`, `sample_continuous_belief_grid`
3. Mechanical Rescue & Online Residual Learning (`rescue.py`):
   - `ResidualRecoveryLearner`, `RescueEngine`, `DistressSignal`, `FailureMode`,
     `RecoveryAction`, `MechanicalRescueTechnique`, `MechanicalActuatorSetpoint`,
     `compute_mechanical_recovery_setpoint`
4. Native ReAct Swarm Orchestrator (`orchestrator.py`):
   - `SwarmOrchestratorAgent`, `OrchestratorToolBelt`, `IncidentReport`,
     `OperatorPolicy`, `HelperStatus`, `OrchestratorDecision`,
     `OrchestratorSafetyGate`, `DeterministicReasoner`, `GeminiRESTBackend`
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
from .orchestrator import (
    DeterministicReasoner,
    GeminiRESTBackend,
    HelperStatus,
    IncidentReport,
    OperatorPolicy,
    OrchestratorDecision,
    OrchestratorSafetyGate,
    OrchestratorToolBelt,
    SwarmOrchestratorAgent,
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
from .rescue import (
    DistressSignal,
    FailureMode,
    MechanicalActuatorSetpoint,
    MechanicalRescueTechnique,
    RecoveryAction,
    RescueEngine,
    ResidualRecoveryLearner,
    compute_mechanical_recovery_setpoint,
)

__all__ = [
    "AgentBid",
    "AmbiguityEvent",
    "CooperativeSwarmPlanner",
    "DecentralizedMycelialRuntime",
    "DecentralizedRobotNode",
    "DeterministicReasoner",
    "DistressSignal",
    "FailureMode",
    "GaussianChannelBelief",
    "GeminiRESTBackend",
    "GeminiSwarmAdvisor",
    "HelperStatus",
    "IncidentReport",
    "LocalMycelialMap",
    "LocalSlamGrid",
    "MechanicalActuatorSetpoint",
    "MechanicalRescueTechnique",
    "MycelialGossipPacket",
    "OnboardSensorTelemetry",
    "OperatorPolicy",
    "OrchestratorDecision",
    "OrchestratorSafetyGate",
    "OrchestratorToolBelt",
    "RecoveryAction",
    "RescueAssessment",
    "RescueEngine",
    "RescueVerdict",
    "ResidualRecoveryLearner",
    "SlamHazardCell",
    "SoilSafetyEnvelope",
    "SupervisoryAdvisor",
    "SwarmOrchestratorAgent",
    "SwarmRobotAgent",
    "TaskBidProposal",
    "TravelCostFn",
    "compute_mechanical_recovery_setpoint",
    "evaluate_continuous_belief_field",
    "homogeneous_capabilities",
    "sample_continuous_belief_grid",
]
