"""Read-only telemetry & top-down swarm belief dashboard package (`src/ricebot/viewer/`)."""
from .dashboard import (
    SwarmBeliefDashboardRecorder,
    render_dashboard_html,
)
from .live_app import LiveSwarmDashboardApp, launch_live_dashboard

__all__ = [
    "LiveSwarmDashboardApp",
    "SwarmBeliefDashboardRecorder",
    "launch_live_dashboard",
    "render_dashboard_html",
]
