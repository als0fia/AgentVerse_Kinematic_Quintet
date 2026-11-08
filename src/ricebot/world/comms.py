"""Peer-to-peer radio transport, ranging, and RF blackout simulation (`src/ricebot/world/comms.py`).

Implements communications and perception isolation per `docs/FIVE-AGENT-RECONCILIATION.md`,
`docs/audits/2026-11-07-simulation-perception-viewer.md` (Audit Findings M6, T6, T7), and
`docs/checkpoints/2026-11-07-2021-GMT/P3-COMMUNICATIONS-DEPLOYMENT.md`:
- Sub-1 GHz / UWB local radio propagation with bounded communication range (`comm_range_m = 1.2m`).
- Mountain terrace RF shadowing with configurable blackout duty cycles:
  * 'normal': 100% link availability within line-of-sight range.
  * 'intermittent': 60.5% RF blackout duty cycle (valley terrain shadowing).
  * 'blackout' / 'isolated': 100% RF blackout (deep canyon / non-line-of-sight dead zone).
- Negative tests and epistemic isolation: prevents local rover nodes from peeking into
  global simulator truth poses or receiving out-of-range peer measurements.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Sequence

from ..contracts import Pose2D, RobotObservation


class ChannelStatus(str, Enum):
    CONNECTED = "connected"
    DEGRADED = "degraded"
    BLACKOUT = "blackout"


@dataclass(frozen=True)
class PeerLinkStatus:
    """State of the radio link between two rovers."""
    transmitter_id: str
    receiver_id: str
    distance_m: float
    in_range: bool
    status: ChannelStatus
    packet_loss_rate: float
    snr_db: float


class PaddyP2PCommsChannel:
    """Simulated peer-to-peer radio channel between Antenna rovers."""

    def __init__(
        self,
        comm_range_m: float = 1.2,
        mode: str = "normal",
        intermittent_cycle_s: float = 4.0,
        intermittent_blackout_fraction: float = 0.605,
    ) -> None:
        self.comm_range_m = float(comm_range_m)
        self.mode = mode.lower()
        self.intermittent_cycle_s = float(intermittent_cycle_s)
        self.intermittent_blackout_fraction = float(intermittent_blackout_fraction)

    def is_blackout_active(self, time_s: float) -> bool:
        """Return True if environmental RF blackout is currently suppressing transmissions."""
        if self.mode in ("blackout", "isolated"):
            return True
        if self.mode == "intermittent":
            phase = (time_s % self.intermittent_cycle_s) / self.intermittent_cycle_s
            return phase < self.intermittent_blackout_fraction
        return False

    def evaluate_link(
        self,
        tx_id: str,
        tx_pose: Pose2D,
        rx_id: str,
        rx_pose: Pose2D,
        time_s: float,
    ) -> PeerLinkStatus:
        """Evaluate physical link connectivity between two rovers at time_s."""
        dx = tx_pose.x_m - rx_pose.x_m
        dy = tx_pose.y_m - rx_pose.y_m
        dist = math.hypot(dx, dy)
        in_range = dist <= self.comm_range_m

        if not in_range:
            return PeerLinkStatus(
                transmitter_id=tx_id,
                receiver_id=rx_id,
                distance_m=dist,
                in_range=False,
                status=ChannelStatus.BLACKOUT,
                packet_loss_rate=1.0,
                snr_db=-999.0,
            )

        if self.is_blackout_active(time_s):
            return PeerLinkStatus(
                transmitter_id=tx_id,
                receiver_id=rx_id,
                distance_m=dist,
                in_range=True,
                status=ChannelStatus.BLACKOUT,
                packet_loss_rate=1.0,
                snr_db=-15.0,
            )

        # In range and no environmental blackout
        # Realistic Sub-1 GHz path loss model (free space + canopy attenuation)
        loss_rate = max(0.0, min(0.35, 0.05 + 0.25 * (dist / self.comm_range_m) ** 2))
        snr = 24.0 - 15.0 * (dist / self.comm_range_m)
        status = ChannelStatus.CONNECTED if loss_rate < 0.20 else ChannelStatus.DEGRADED

        return PeerLinkStatus(
            transmitter_id=tx_id,
            receiver_id=rx_id,
            distance_m=dist,
            in_range=True,
            status=status,
            packet_loss_rate=loss_rate,
            snr_db=snr,
        )

    def filter_peer_observations_for_robot(
        self,
        query_robot_id: str,
        query_pose: Pose2D,
        all_robot_observations: Sequence[RobotObservation],
        time_s: float,
    ) -> list[RobotObservation]:
        """Return only the peer observations that can legally be received over RF.

        Enforces epistemic isolation (Audit Finding M6, T7):
        - A rover never receives out-of-range peer observations.
        - During blackout, no peer observations are delivered.
        - Query robot's own observation is excluded from peer list.
        """
        permitted_peers: list[RobotObservation] = []
        for obs in all_robot_observations:
            if obs.robot_id == query_robot_id:
                continue
            link = self.evaluate_link(
                tx_id=obs.robot_id,
                tx_pose=obs.pose,
                rx_id=query_robot_id,
                rx_pose=query_pose,
                time_s=time_s,
            )
            if link.in_range and link.status != ChannelStatus.BLACKOUT:
                permitted_peers.append(obs)
        return permitted_peers
