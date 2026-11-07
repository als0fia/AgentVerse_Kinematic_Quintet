"""Read-only dashboard and telemetry viewer package (`src/ricebot/viewer/`).

Implements `ReadOnlyView` subscribers (`SwarmBeliefDashboardRecorder`) and interactive
top-down belief heatmap & Bekker-Wong terramechanics HTML exporters (`render_dashboard_html`)
per `docs/COMPONENTS.md` Role 5 without ever mutating simulation physics or leaking hidden
ground truth into the agent runtime.
"""
from .dashboard import SwarmBeliefDashboardRecorder, render_dashboard_html

__all__ = [
    "SwarmBeliefDashboardRecorder",
    "render_dashboard_html",
]
