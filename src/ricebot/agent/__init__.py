"""Swarm Intelligence & Edge Coordination Package (src/ricebot/agent/).

Provides homogeneous multi-robot coordination with Tier-1 micro-edge utility
bidding and optional Tier-2 deliberative event-driven supervisor.
"""
from .advisor import GeminiSwarmAdvisor
from .core import (
    AgentBid,
    CooperativeSwarmPlanner,
    SwarmRobotAgent,
    homogeneous_capabilities,
)

__all__ = [
    "AgentBid",
    "CooperativeSwarmPlanner",
    "GeminiSwarmAdvisor",
    "SwarmRobotAgent",
    "homogeneous_capabilities",
]
