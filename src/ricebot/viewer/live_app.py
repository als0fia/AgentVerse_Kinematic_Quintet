r"""Unified Side-by-Side 3D MuJoCo Physics & 2D Top-Down Swarm Belief Desktop Application.

Single-window large side-by-side layout reconciled with the Multi-Seed Independent Benchmark Suite:
- **Top Control & Independent Benchmark Toolbar:**
  - Switch **Seed (`1, 2, 3, 4, 5, 7`)**, **Scenario (`nominal`, `bad_soil`, `shifted_moisture`, `obstacle`, `sensor_degraded`)**, **Policy (`mycelial` Policy B, `centralized` Policy A, `single_rover`)**, and **Comms Mode (`normal`, `intermittent`, `isolated`)**.
  - Live **Independent Benchmark Grader & D-0010 Equal-Work Comm Bar**: displays real-time `Viable Seeds Planted`, `Wasted Mud Drops`, `Discrete Site RMSE`, `Continuous 2D Field RMSE`, `D-0010 Policy Wire Bytes vs. 10Hz Stream Baseline (% Bandwidth Saved)`, and `Vectorized Compute Time (ms/tick)`.
- **Left Side — BIG Live 3D MuJoCo Physics Visualiser (`840 x 560`+, dynamically resizable):**
  - Renders the 3D undulating paddy terrace heightfield (`paddy_micro_relief`), real-time 3D vertical wheel-mud sinkage ($z_{\text{sink}}$), wheel-slip fishtailing in soft mud, 3D sunken wheel-rut tracks (`sim.mud_rut_history`), 3D mud-collar rings around bogging wheels, split dual-rover belief beacons, local SLAM hazard cells, and UWB/HaLow gossip beams.
- **Right Side — BIG Interactive 2D Top-Down Belief & Agronomic Fitness Visualiser (`660 x 560`+, vectorized single-image PPM renderer) + Robot Inspector:**
  - Click directly on `PLANTER` or `SCOUT` on the 2D canvas (or select from the bar) to inspect what that specific robot understands about the continuous field (`Planter Local Belief`, `Scout Local Belief`, `|Planter - Scout| Divergence`, or `Hidden Ground Truth f(x,y)`).
  - Switch across 8 continuous spatial layers including the agronomically researched **Wet Direct-Seeded Rice Planting Fitness Index $\Phi_{\text{plant}}(x,y)$**, Soil Moisture $M(x,y)$, Cone Firmness $F(x,y)$, Canopy Stand Gap $G(x,y)$, Plant Density $\rho(x,y)$, Epistemic Uncertainty $\sigma(x,y)$, Mud/SLAM Traversability Risk $R(x,y)$, and Micro-Topography Elevation $z(x,y)$.
"""
from __future__ import annotations

from math import atan2, cos, hypot, pi, sin, sqrt
import random
import time
import tkinter as tk
from typing import Any

import mujoco
import numpy as np

from ..agent import (
    DecentralizedMycelialRuntime,
    SoilSafetyEnvelope,
    sample_continuous_belief_grid,
)
from ..contracts import (
    Decision,
    ObservationFrame,
    OperatorCommandKind,
    OperatorRequest,
    RescueVerdict,
    RobotCapability,
    RobotCommand,
    SensorKind,
    TaskIntent,
    TaskMode,
)
from ..control import ControlLimits, LocalController
from ..mission import MeasuredFieldMap, SimpleMissionPlanner
from ..mujoco_sim.adapter import DEFAULT_CAPABILITIES, DEFAULT_SITES, MujocoPaddySimulation
from ..runtime import EpisodeConfig, EpisodeSession, MissionCore, create_mujoco_episode_session
from .presentation import PresentationCue, presentation_cue
from ..world import (
    RandomizedEpisodeSpec,
    generate_hidden_soil_truth,
    generate_randomized_episode_spec,
)


ROBOT_HEX_COLORS: dict[str, str] = {
    "planter": "#38bdf8",    # blue
    "scout": "#22c55e",      # green
    "planter_2": "#06b6d4",  # cyan
    "scout_2": "#f59e0b",    # amber
    "planter_3": "#a855f7",  # purple
    "scout_3": "#f97316",    # coral
}

ROBOT_RGBA_COLORS: dict[str, list[float]] = {
    "planter": [0.22, 0.68, 0.98, 0.78],
    "scout": [0.22, 0.90, 0.45, 0.78],
    "planter_2": [0.14, 0.82, 0.92, 0.78],
    "scout_2": [0.96, 0.68, 0.16, 0.78],
    "planter_3": [0.68, 0.42, 0.96, 0.78],
    "scout_3": [0.96, 0.45, 0.32, 0.78],
}

FLEET_SIZE_OPTIONS: tuple[tuple[int, str], ...] = (
    (2, "2 Rovers"),
    (3, "3 Rovers"),
    (4, "4 Rovers"),
    (6, "6 Rovers"),
)

MAP_SCALE_OPTIONS: tuple[tuple[float, str], ...] = (
    (1.0, "1.0x Standard (6-Patch)"),
    (1.6, "1.6x Expanded Terrace"),
)

CAMERA_FOV_RAD: float = 62.0 * (pi / 180.0)
CAMERA_HALF_FOV_RAD: float = 0.5 * CAMERA_FOV_RAD
CAMERA_RANGE_M: float = 0.95


def _robot_hex_color(robot_id: str) -> str:
    return ROBOT_HEX_COLORS.get(robot_id, "#38bdf8" if "planter" in robot_id else "#4ade80")


def _robot_rgba_color(robot_id: str) -> list[float]:
    return list(ROBOT_RGBA_COLORS.get(robot_id, [0.35, 0.78, 0.95, 0.72]))


def _wrap_angle_rad(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


def _rgb_array_to_tk_ppm(rgb: np.ndarray) -> bytes:
    """Convert an (H, W, 3) uint8 RGB NumPy array into in-memory binary PPM bytes for tk.PhotoImage."""
    h, w, _ = rgb.shape
    header = f"P6\n{w} {h}\n255\n".encode("ascii")
    return header + np.ascontiguousarray(rgb, dtype=np.uint8).tobytes()


def _grid_to_rgb_ppm(
    grid_2d: Any,
    vmin: float,
    vmax: float,
    palette: str,
    out_w: int,
    out_h: int,
) -> bytes:
    """Vectorized NumPy 2D grid-to-RGB PPM converter (replaces hundreds of per-frame Canvas rectangles with 1 image)."""
    arr = np.asarray(grid_2d, dtype=np.float64)
    # Flip rows vertically so +y (North) is at the top of the 2D canvas
    arr = arr[::-1, :]
    t = np.clip((arr - vmin) / max(1e-6, vmax - vmin), 0.0, 1.0)

    if palette == "fitness":
        r = 195.0 * (1.0 - t) + 25.0 * t
        g = 55.0 + 175.0 * (t**0.85)
        b = 45.0 + 65.0 * t
    elif palette == "moisture":
        r = 215.0 - 185.0 * t
        g = 180.0 - 75.0 * (t**1.1) + 30.0 * (1.0 - np.abs(2.0 * t - 1.0))
        b = 95.0 + 150.0 * (t**0.85)
    elif palette == "firmness":
        r = 120.0 + 95.0 * (1.0 - t)
        g = 65.0 + 150.0 * t
        b = 45.0 + 75.0 * t
    elif palette == "density":
        r = 145.0 - 110.0 * t
        g = 110.0 + 115.0 * t
        b = 65.0 - 20.0 * t
    elif palette == "gap":
        r = 35.0 + 210.0 * t
        g = 150.0 + 55.0 * t - 60.0 * (t * t)
        b = 85.0 - 55.0 * t
    elif palette == "uncertainty":
        r = 20.0 + 225.0 * t
        g = 130.0 - 90.0 * t + 70.0 * (t * t)
        b = 190.0 - 40.0 * t
    elif palette == "elevation":
        r = 50.0 + 175.0 * t
        g = 95.0 + 125.0 * t
        b = 85.0 + 60.0 * (1.0 - t)
    else:  # risk / divergence
        r = 30.0 + 220.0 * (t**0.85)
        g = 175.0 * (1.0 - t)
        b = 95.0 * (1.0 - t) + 110.0 * (t * t)

    rgb_small = np.stack(
        [
            np.clip(r, 0, 255).astype(np.uint8),
            np.clip(g, 0, 255).astype(np.uint8),
            np.clip(b, 0, 255).astype(np.uint8),
        ],
        axis=-1,
    )
    ny, nx, _ = rgb_small.shape
    out_w_safe = max(nx, int(out_w))
    out_h_safe = max(ny, int(out_h))
    row_idx = (np.arange(out_h_safe) * ny) // out_h_safe
    col_idx = (np.arange(out_w_safe) * nx) // out_w_safe
    rgb_upscaled = rgb_small[row_idx[:, None], col_idx[None, :]]
    return _rgb_array_to_tk_ppm(rgb_upscaled)


LAYER_META: dict[str, tuple[str, float, float, str]] = {
    "planting_fitness": ("Rice Planting Fitness Φ(x,y)", 0.0, 0.85, "fitness"),
    "moisture": ("Soil Moisture M(x,y)", 0.25, 0.95, "moisture"),
    "firmness": ("Cone Firmness F(x,y)", 0.15, 0.90, "firmness"),
    "stand_gap": ("Canopy Stand Gap G(x,y)", 0.0, 1.0, "gap"),
    "plant_density": ("Plant Density ρ(x,y)", 0.0, 1.0, "density"),
    "uncertainty": ("Epistemic Uncertainty σ(x,y)", 0.0, 1.0, "uncertainty"),
    "traversability_risk": ("Mud / SLAM Risk R(x,y)", 0.0, 1.0, "risk"),
    "elevation_m": ("Micro-Topography z(x,y) [m]", -0.035, 0.025, "elevation"),
}


class LiveSwarmDashboardApp:
    """Single-window large side-by-side 3D MuJoCo physics simulator and 2D Top-Down Belief & Benchmark Dashboard."""

    RENDER_W = 840
    RENDER_H = 560
    MAP_W = 660
    MAP_H = 560
    GRID_NX = 44
    GRID_NY = 30

    def __init__(
        self,
        root: tk.Tk,
        *,
        initial_scenario: str = "golden_rescue",
        initial_comms: str = "normal",
        initial_policy: str = "cooperative",
        initial_rescue_policy: str = "react_orchestrator",
        seed: int = 7,
        num_rovers: int = 2,
        map_scale: float = 1.0,
        initial_num_rovers: int | None = None,
        initial_map_scale: float | None = None,
        scenario: str | None = None,
        policy: str | None = None,
        presentation_mode: bool = False,
    ) -> None:
        if scenario is not None:
            initial_scenario = scenario
        if policy is not None:
            initial_policy = policy
        self.root = root
        self.presentation_mode = bool(presentation_mode)
        self.presentation_camera_mode = tk.StringVar(value="auto")
        self.presentation_cue = PresentationCue("Fieldwork active")
        self.rescue_approach_tracks: dict[tuple[str, str], list[tuple[float, float]]] = {}
        self.presentation_events: list[tuple[float, str]] = []
        self.root.title(
            "Antenna (PaddyPatch rice.v1) — Canonical ABDE 3D MuJoCo Rescue, 2D Swarm Belief & Orchestrator Suite"
        )
        self.root.geometry("1860x1020")
        try:
            self.root.state("zoomed")
        except Exception:
            pass
        self.root.configure(bg="#0b131e")

        self.seed = int(seed)
        self.num_rovers: int = max(2, min(6, int(initial_num_rovers if initial_num_rovers is not None else num_rovers)))
        self.map_scale: float = float(initial_map_scale if initial_map_scale is not None else map_scale)
        self._synced_num_rovers: int = self.num_rovers
        self._synced_map_scale: float = self.map_scale

        self.seed_var = tk.IntVar(value=self.seed)
        self.seed_entry_var = tk.StringVar(value=str(self.seed))
        self.scenario_var = tk.StringVar(value=initial_scenario)
        self.preset_var = self.scenario_var  # backward-compatible alias
        self.policy_var = tk.StringVar(value=initial_policy)
        self.rescue_policy_var = tk.StringVar(value=initial_rescue_policy)
        self.comms_var = tk.StringVar(value=initial_comms)
        self.num_rovers_var = tk.IntVar(value=self.num_rovers)
        self.fleet_size_var = self.num_rovers_var
        self.map_scale_var = tk.DoubleVar(value=self.map_scale)
        self.camera_var = tk.StringVar(value="isometric")
        self.selected_robot_var = tk.StringVar(value="planter")
        self.belief_view_var = tk.StringVar(value="planter")  # fused | <robot_id> | divergence | ground_truth
        self.layer_var = tk.StringVar(value="planting_fitness")

        self.mass_kg_var = tk.DoubleVar(value=4.2)
        self.wheel_radius_mm_var = tk.DoubleVar(value=58.0)
        self.wheel_width_mm_var = tk.DoubleVar(value=40.0)
        self.human_control_var = tk.BooleanVar(value=False)
        self.manual_lin_m_s: float = 0.0
        self.manual_ang_rad_s: float = 0.0
        self.show_details_var = tk.BooleanVar(value=True)

        self.playing = False
        self.after_id: str | None = None
        self.step_count = 0
        self.last_compute_ms: float = 0.0
        self._tk_3d_img: tk.PhotoImage | None = None
        self._tk_2d_img: tk.PhotoImage | None = None
        self._gl_viewer: Any = None
        self.gl_Ready: bool = False
        self.con: Any = None
        self._hfield_dirty: bool = False
        self._central_core: MissionCore | None = None
        self._central_samples_count: int = 0
        self.randomized_spec: RandomizedEpisodeSpec | None = None
        self._comms_radios: list[tk.Radiobutton] = []
        self.who_buttons_frame: tk.Frame | None = None
        self.session: EpisodeSession

        self.trajectories: dict[str, list[tuple[float, float]]] = {}
        self.visually_discovered_sites: dict[str, set[str]] = {}
        self._init_simulation()
        self._build_ui()
        self._bind_teleop_keys()
        self._refresh_all_views()

    @property
    def selected_robot_id(self) -> str:
        """Currently selected rover ID for telemetry, sizing, and belief inspection."""
        return self.selected_robot_var.get()

    @selected_robot_id.setter
    def selected_robot_id(self, robot_id: str) -> None:
        rid = str(robot_id)
        self.selected_robot_var.set(rid)
        if hasattr(self, "runtime") and self.runtime is not None and rid in self.runtime.nodes:
            self.belief_view_var.set(rid)

    @property
    def selected_belief_robot(self) -> str:
        """Currently active 2D belief perspective ('fused', '<robot_id>', 'divergence', or 'ground_truth')."""
        return self.belief_view_var.get()

    @selected_belief_robot.setter
    def selected_belief_robot(self, view_or_robot_id: str) -> None:
        val = str(view_or_robot_id)
        self.belief_view_var.set(val)
        if hasattr(self, "runtime") and self.runtime is not None and val in self.runtime.nodes:
            self.selected_robot_var.set(val)

    @property
    def comms_applies(self) -> bool:
        """Return True only when the active policy runtime actually uses wireless RF comms settings."""
        return self._central_core is None and not (
            self.policy_var.get() == "single_rover" or self.session.config.single_rover_only
        )

    def _field_bounds(self) -> tuple[tuple[float, float], tuple[float, float]]:
        """Return `(x_bounds, y_bounds)` scaled by `self.map_scale` (identical to rice.v1 defaults at 1.0x)."""
        scale = float(getattr(self, "map_scale", 1.0))
        if abs(scale - 1.0) <= 1e-6:
            return ((-0.25, 2.95), (-0.90, 0.90))
        half_x = 1.60 * scale
        half_y = 0.90 * scale
        return ((round(1.35 - half_x, 3), round(1.35 + half_x, 3)), (round(-half_y, 3), round(half_y, 3)))

    def _upload_hfield_if_needed(self, *, force: bool = False) -> None:
        """Upload updated MuJoCo heightfield to active OpenGL context when dirty or after reset."""
        if not hasattr(self, "sim") or self.sim is None:
            return
        sim_dirty = bool(getattr(self.sim, "_hfield_dirty", False))
        app_dirty = bool(getattr(self, "_hfield_dirty", False))
        if not (force or sim_dirty or app_dirty):
            return
        try:
            con = getattr(self, "con", None) or getattr(getattr(self, "renderer", None), "_mjr_context", None)
            if getattr(self, "gl_Ready", False) and con is not None and self.sim.model.nhfield > 0:
                mujoco.mjr_uploadHField(self.sim.model, con, 0)
            if hasattr(self.sim, "_hfield_dirty"):
                setattr(self.sim, "_hfield_dirty", False)
            self._hfield_dirty = False
        except Exception:
            pass

    def _update_visual_camera_discoveries(self) -> None:
        """Update Tier-1 forward optical camera cone (`62°` FOV, `0.95m` range) site discoveries per rover."""
        if not hasattr(self, "frame") or self.frame is None:
            return
        for r in self.frame.robots:
            vis_set = self.visually_discovered_sites.setdefault(r.robot_id, set())
            for sid, site in self.sim.sites.items():
                if not site.permitted:
                    continue
                dx = site.center.x_m - r.pose.x_m
                dy = site.center.y_m - r.pose.y_m
                dist = hypot(dx, dy)
                if dist <= CAMERA_RANGE_M:
                    rel_ang = abs(_wrap_angle_rad(atan2(dy, dx) - r.pose.yaw_rad))
                    if rel_ang <= CAMERA_HALF_FOV_RAD or dist <= self.sim.probe_radius_m:
                        vis_set.add(sid)

    def apply_custom_seed(self, _event: Any = None) -> None:
        """Apply an arbitrary non-negative integer seed typed into the seed entry box."""
        raw = self.seed_entry_var.get().strip()
        try:
            val = int(raw)
        except ValueError:
            self.seed_entry_var.set(str(self.seed))
            return
        if val < 0:
            self.seed_entry_var.set(str(self.seed))
            return
        self.seed_var.set(val)
        self.reset_episode()

    def apply_fleet_and_map_setup(
        self,
        num_rovers: int | None = None,
        map_scale: float | None = None,
    ) -> None:
        """Update `self.num_rovers` and `self.map_scale` from Main Menu controls and rebuild simulation."""
        if num_rovers is not None:
            self.num_rovers = max(2, min(6, int(num_rovers)))
            self.num_rovers_var.set(self.num_rovers)
        else:
            self.num_rovers = max(2, min(6, int(self.num_rovers_var.get())))
        if map_scale is not None:
            self.map_scale = float(map_scale)
            self.map_scale_var.set(self.map_scale)
        else:
            self.map_scale = float(self.map_scale_var.get())
        self._synced_num_rovers = self.num_rovers
        self._synced_map_scale = self.map_scale
        self._rebuild_simulation()

    def set_fleet_size(self, num_rovers: int) -> None:
        """Convenience setter to switch fleet size (2, 3, 4, or 6 rovers) and rebuild."""
        self.apply_fleet_and_map_setup(num_rovers=int(num_rovers))

    def set_map_scale(self, map_scale: float) -> None:
        """Convenience setter to switch terrace map scale (1.0 or 1.6) and rebuild."""
        self.apply_fleet_and_map_setup(map_scale=float(map_scale))

    def _apply_scenario_preset(self, preset: str | None = None) -> None:
        """Backward-compatible helper to apply a scenario preset and reset the episode."""
        if preset is not None:
            self.scenario_var.set(str(preset))
        self.reset_episode()

    def _randomize_episode(self) -> None:
        """Backward-compatible alias for `trigger_random_scenario`."""
        self.trigger_random_scenario()

    def _rebuild_simulation(self) -> None:
        """Rebuild the MuJoCo simulation and episode session with current `num_rovers` and `map_scale`."""
        was_playing = getattr(self, "playing", False)
        if was_playing:
            self.toggle_play()
        self._init_simulation()
        if hasattr(self, "who_buttons_frame") and self.who_buttons_frame is not None:
            self._refresh_belief_selector_buttons()
        if hasattr(self, "canvas_2d"):
            self._refresh_all_views()
        if was_playing and not self.playing:
            self.toggle_play()

    def _init_simulation(self) -> None:
        self.seed = int(self.seed_var.get())
        if hasattr(self, "seed_entry_var"):
            self.seed_entry_var.set(str(self.seed))

        if hasattr(self, "num_rovers_var"):
            var_nr = int(self.num_rovers_var.get())
            if var_nr != getattr(self, "_synced_num_rovers", self.num_rovers):
                self.num_rovers = var_nr
            else:
                self.num_rovers_var.set(int(self.num_rovers))
        self.num_rovers = max(2, min(6, int(getattr(self, "num_rovers", 2))))
        self._synced_num_rovers = self.num_rovers

        if hasattr(self, "map_scale_var"):
            var_ms = float(self.map_scale_var.get())
            if abs(var_ms - getattr(self, "_synced_map_scale", self.map_scale)) > 1e-6:
                self.map_scale = var_ms
            else:
                self.map_scale_var.set(float(self.map_scale))
        self.map_scale = float(getattr(self, "map_scale", 1.0))
        self._synced_map_scale = self.map_scale

        scen = self.scenario_var.get()
        pol = self.policy_var.get()
        rescue_pol = self.rescue_policy_var.get()
        comms = self.comms_var.get()
        self.randomized_spec = (
            generate_randomized_episode_spec(self.seed) if scen == "randomized" else None
        )
        self.hidden_truth = generate_hidden_soil_truth(scen, self.seed)

        cfg = EpisodeConfig(
            scenario=scen,
            policy=pol,
            comms=comms,
            seed=self.seed,
            steps=195 if scen in ("golden_rescue", "high_risk_abstain", "randomized") else 160,
            single_rover_only=(pol == "single_rover"),
            rescue_policy=rescue_pol,
            include_shadow_mycelial=True,
            num_rovers=getattr(self, "num_rovers", 2),
            map_scale=getattr(self, "map_scale", 1.0),
            active_mycelial_in_rescue=(pol == "mycelial"),
        )
        t0 = time.perf_counter()
        self.session = create_mujoco_episode_session(cfg, include_shadow_mycelial=True)
        self.sim = self.session.sim  # type: ignore[assignment]
        self.hidden_truth = {sid: dict(values) for sid, values in self.sim.world._hidden_site_truth.items()}
        if isinstance(self.session.policy, DecentralizedMycelialRuntime):
            self.runtime = self.session.policy
            self._central_core = None
        else:
            self.runtime = self.session.shadow_policy  # type: ignore[assignment]
            self._central_core = self.session.policy  # type: ignore[assignment]
        self._central_samples_count = 0

        self.frame: ObservationFrame = self.session.reset(self.seed)
        self._hfield_dirty = True
        self.decision: Decision = (
            self.session.last_decision
            if self.session.last_decision is not None
            else Decision(intents=(), commands=(), ingested_samples=0)
        )
        active_rids = [r.robot_id for r in self.frame.robots]
        if hasattr(self, "selected_robot_var") and self.selected_robot_var.get() not in active_rids:
            self.selected_robot_var.set(active_rids[0] if active_rids else "planter")
        if hasattr(self, "belief_view_var"):
            bv = self.belief_view_var.get()
            if bv not in active_rids and bv not in ("fused", "divergence", "ground_truth"):
                self.belief_view_var.set(active_rids[0] if active_rids else "planter")

        self._apply_vehicle_sizing_to_sim()
        self.last_compute_ms = (time.perf_counter() - t0) * 1000.0
        self.step_count = 0
        self.trajectories = {
            r.robot_id: [(r.pose.x_m, r.pose.y_m)] for r in self.frame.robots
        }
        self.rescue_approach_tracks = {}
        self.presentation_events = []
        self.presentation_cue = PresentationCue("Fieldwork active")
        self.visually_discovered_sites = {r.robot_id: set() for r in self.frame.robots}
        self._update_visual_camera_discoveries()

        if hasattr(self, "renderer") and self.renderer is not None:
            try:
                self.renderer.close()
            except Exception:
                pass
        self.renderer = mujoco.Renderer(self.sim.model, height=self.RENDER_H, width=self.RENDER_W)
        self.con = getattr(self.renderer, "_mjr_context", None)
        self.gl_Ready = self.con is not None
        self._upload_hfield_if_needed(force=True)

        self.free_cam = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(self.sim.model, self.free_cam)
        self.free_cam.lookat[:] = [1.22, -0.05, 0.05]
        self.free_cam.distance = 2.55 * max(1.0, 0.85 * self.map_scale) if self.map_scale > 1.0 else 2.55
        self.free_cam.elevation = -28.0
        self.free_cam.azimuth = 118.0

        x_b, y_b = self._field_bounds()
        self.truth_grid = self.sim.world.continuous_terrain.sample_grid(
            nx=self.GRID_NX, ny=self.GRID_NY, x_bounds=x_b, y_bounds=y_b
        )

    def _apply_vehicle_sizing_to_sim(self) -> None:
        m_kg = float(self.mass_kg_var.get())
        r_m = float(self.wheel_radius_mm_var.get()) / 1000.0
        b_m = float(self.wheel_width_mm_var.get()) / 1000.0
        sel = self.selected_robot_var.get()
        if hasattr(self, "session") and self.session is not None:
            t_now = self.frame.time_s if hasattr(self, "frame") and self.frame is not None else 0.0
            active_rids = {r.robot_id for r in self.frame.robots} if hasattr(self, "frame") and self.frame else {"planter", "scout"}
            self.session.submit_operator_request(
                OperatorRequest(
                    request_id=f"sizing-{self.step_count}-{int(time.time()*1000)%10000}",
                    kind=OperatorCommandKind.EXPERIMENT_VEHICLE_SIZING,
                    issued_at_s=t_now,
                    payload=(
                        ("robot_id", sel if sel in active_rids else "all"),
                        ("mass_kg", m_kg),
                        ("wheel_radius_m", r_m),
                        ("wheel_width_m", b_m),
                    ),
                )
            )

    def trigger_mud_fault_injection(self) -> None:
        """Route a validated INJECT_MUD_FAULT command through EpisodeSession."""
        sel = self.selected_robot_var.get()
        t_now = self.frame.time_s if hasattr(self, "frame") and self.frame is not None else 0.0
        site_id = "patch_a" if sel == "scout" else "patch_d"
        self.session.submit_operator_request(
            OperatorRequest(
                request_id=f"inject-mud-{self.step_count}",
                kind=OperatorCommandKind.INJECT_MUD_FAULT,
                issued_at_s=t_now,
                payload=(("robot_id", sel), ("site_id", site_id)),
            )
        )
        self._refresh_all_views()

    def _build_ui(self) -> None:
        # Always-visible navigation; advanced controls remain the original widgets.
        self.presentation_header = tk.Frame(self.root, bg="#0a1724")
        self.presentation_header.pack(fill=tk.X, padx=8, pady=(4, 2))
        tk.Label(
            self.presentation_header, text="ANTENNA   /   RESILIENT ROBOT COLLECTIVE",
            bg="#0a1724", fg="#f8fafc", font=("Segoe UI", 14, "bold"),
        ).pack(side=tk.LEFT, padx=(10, 14), pady=6)
        self.mode_btn = tk.Button(
            self.presentation_header, text="Presentation Mode",
            command=self.toggle_presentation_mode, bg="#1e293b", fg="#f8fafc",
            font=("Segoe UI", 9, "bold"),
        )
        self.mode_btn.pack(side=tk.RIGHT, padx=7)
        self.presentation_controls = tk.Frame(self.presentation_header, bg="#0a1724")
        for label, mode in (("Field", "field"), ("Rescue", "rescue"), ("Auto frame", "auto")):
            tk.Button(
                self.presentation_controls, text=label,
                command=lambda m=mode: self.set_presentation_camera(m),
                bg="#233d54", fg="white", font=("Segoe UI", 9, "bold"),
            ).pack(side=tk.LEFT, padx=2)
        tk.Button(
            self.presentation_controls, text="Restart", command=self.reset_episode,
            bg="#233d54", fg="white", font=("Segoe UI", 9, "bold"),
        ).pack(side=tk.LEFT, padx=2)
        self.presentation_play_btn = tk.Button(
            self.presentation_controls, text="Play", command=self.toggle_play,
            bg="#0284c7", fg="white", font=("Segoe UI", 9, "bold"),
        )
        self.presentation_play_btn.pack(side=tk.LEFT, padx=2)
        self.presentation_event_lbl = tk.Label(
            self.presentation_header, text="", bg="#0a1724", fg="#fbbf24",
            font=("Segoe UI", 10, "bold"), anchor="w",
        )

        # Row 0: Simple, Clean Main Menu — Fleet Size (N-Rover) & Field Map Scale Setup Strip
        self.main_menu_bar = tk.Frame(
            self.root, bg="#0f172a", highlightthickness=1, highlightbackground="#38bdf8"
        )
        self.main_menu_bar.pack(fill=tk.X, padx=8, pady=(5, 2))

        tk.Label(
            self.main_menu_bar,
            text="🧭 MAIN MENU — FLEET & MAP SETUP:",
            bg="#0f172a",
            fg="#38bdf8",
            font=("Segoe UI", 9, "bold"),
        ).pack(side=tk.LEFT, padx=(8, 6), pady=3)

        tk.Label(
            self.main_menu_bar,
            text="Fleet Size:",
            bg="#0f172a",
            fg="#fbbf24",
            font=("Segoe UI", 8, "bold"),
        ).pack(side=tk.LEFT, padx=(4, 2))

        self._fleet_size_radios: list[tk.Radiobutton] = []
        for n_rov, n_lbl in FLEET_SIZE_OPTIONS:
            rb_f = tk.Radiobutton(
                self.main_menu_bar,
                text=n_lbl,
                variable=self.num_rovers_var,
                value=n_rov,
                command=self.apply_fleet_and_map_setup,
                bg="#0f172a",
                fg="#f8fafc",
                selectcolor="#0284c7",
                activebackground="#0f172a",
                font=("Segoe UI", 8, "bold"),
            )
            rb_f.pack(side=tk.LEFT, padx=2)
            self._fleet_size_radios.append(rb_f)

        tk.Label(
            self.main_menu_bar,
            text="| Field Map Scale:",
            bg="#0f172a",
            fg="#fbbf24",
            font=("Segoe UI", 8, "bold"),
        ).pack(side=tk.LEFT, padx=(10, 2))

        self._map_scale_radios: list[tk.Radiobutton] = []
        for m_sc, m_lbl in MAP_SCALE_OPTIONS:
            rb_m = tk.Radiobutton(
                self.main_menu_bar,
                text=m_lbl,
                variable=self.map_scale_var,
                value=m_sc,
                command=self.apply_fleet_and_map_setup,
                bg="#0f172a",
                fg="#f8fafc",
                selectcolor="#0284c7",
                activebackground="#0f172a",
                font=("Segoe UI", 8, "bold"),
            )
            rb_m.pack(side=tk.LEFT, padx=2)
            self._map_scale_radios.append(rb_m)

        self.apply_setup_btn = tk.Button(
            self.main_menu_bar,
            text="🚀 Apply / Launch New Episode",
            command=self.apply_fleet_and_map_setup,
            bg="#0284c7",
            fg="#ffffff",
            font=("Segoe UI", 8, "bold"),
            padx=10,
            pady=2,
        )
        self.apply_setup_btn.pack(side=tk.LEFT, padx=(10, 4), pady=3)

        self.fleet_summary_lbl = tk.Label(
            self.main_menu_bar,
            text="",
            bg="#0f172a",
            fg="#a7f3d0",
            font=("Consolas", 8, "bold"),
        )
        self.fleet_summary_lbl.pack(side=tk.RIGHT, padx=8)

        # Row 1: Top Control Toolbar (Play/Step/Reset + Seed + Scenario + Policy + Comms)
        toolbar = tk.Frame(self.root, bg="#111d2c", highlightthickness=1, highlightbackground="#1f3247")
        toolbar.pack(fill=tk.X, padx=8, pady=(0, 2))

        self.play_btn = tk.Button(
            toolbar,
            text="▶ Play Live 3D Sim",
            command=self.toggle_play,
            bg="#0284c7",
            fg="white",
            font=("Segoe UI", 9, "bold"),
            padx=10,
            pady=2,
        )
        self.play_btn.pack(side=tk.LEFT, padx=5, pady=4)

        tk.Button(
            toolbar,
            text="⏭ Step (0.1s)",
            command=self.step_once,
            bg="#1e293b",
            fg="#e2e8f0",
            font=("Segoe UI", 9, "bold"),
            padx=7,
            pady=2,
        ).pack(side=tk.LEFT, padx=2, pady=4)

        tk.Button(
            toolbar,
            text="↺ Reset",
            command=self.reset_episode,
            bg="#1e293b",
            fg="#e2e8f0",
            font=("Segoe UI", 9),
            padx=7,
            pady=2,
        ).pack(side=tk.LEFT, padx=2, pady=4)

        tk.Button(
            toolbar,
            text="🎲 Randomized Episode",
            command=self.trigger_random_scenario,
            bg="#0f766e",
            fg="#f0fdfa",
            font=("Segoe UI", 8, "bold"),
            padx=7,
            pady=2,
        ).pack(side=tk.LEFT, padx=2, pady=4)

        tk.Button(
            toolbar,
            text="⚡ Inject Mud Fault",
            command=self.trigger_mud_fault_injection,
            bg="#9a3412",
            fg="white",
            font=("Segoe UI", 8, "bold"),
            padx=6,
            pady=2,
        ).pack(side=tk.LEFT, padx=2, pady=4)

        tk.Button(
            toolbar,
            text="⚠ 13kg Mud Belly-Out",
            command=self.trigger_mud_belly_out_demo,
            bg="#b91c1c",
            fg="white",
            font=("Segoe UI", 8, "bold"),
            padx=7,
            pady=2,
        ).pack(side=tk.LEFT, padx=3, pady=4)

        tk.Button(
            toolbar,
            text="✓ 4.2kg Nominal",
            command=self.restore_nominal_sizing,
            bg="#15803d",
            fg="white",
            font=("Segoe UI", 8, "bold"),
            padx=6,
            pady=2,
        ).pack(side=tk.LEFT, padx=2, pady=4)

        self.human_ctrl_btn = tk.Button(
            toolbar,
            text="🛑 Emergency Human Control: OFF",
            command=self.toggle_emergency_human_control,
            bg="#4c1d95",
            fg="#f3e8ff",
            font=("Segoe UI", 8, "bold"),
            padx=8,
            pady=2,
        )
        self.human_ctrl_btn.pack(side=tk.LEFT, padx=4, pady=4)

        # Multi-Seed Selector (Seeds 1..5 + 7 + custom seed entry for exact reproducibility)
        tk.Label(toolbar, text="| Seed:", bg="#111d2c", fg="#fbbf24", font=("Segoe UI", 9, "bold")).pack(
            side=tk.LEFT, padx=(6, 2)
        )
        for s_val in (1, 2, 3, 4, 5, 7):
            tk.Radiobutton(
                toolbar,
                text=str(s_val),
                variable=self.seed_var,
                value=s_val,
                command=self.reset_episode,
                bg="#111d2c",
                fg="#f8fafc",
                selectcolor="#0284c7",
                activebackground="#111d2c",
                font=("Consolas", 8, "bold"),
            ).pack(side=tk.LEFT, padx=1)
        self.seed_entry = tk.Entry(
            toolbar,
            textvariable=self.seed_entry_var,
            width=4,
            bg="#0f172a",
            fg="#fbbf24",
            insertbackground="#fbbf24",
            font=("Consolas", 8, "bold"),
            justify="center",
        )
        self.seed_entry.pack(side=tk.LEFT, padx=(2, 2))
        self.seed_entry.bind("<Return>", self.apply_custom_seed)

        # Scenario Selector (3 Core Presets + Randomized Episode + Existing Scenario Variations)
        tk.Label(toolbar, text="| Scenario:", bg="#111d2c", fg="#94a3b8", font=("Segoe UI", 9, "bold")).pack(
            side=tk.LEFT, padx=(6, 2)
        )
        for scen_id, scen_lbl in (
            ("golden_rescue", "★ 1. Rescue"),
            ("high_risk_abstain", "★ 2. Refusal"),
            ("sensor_degraded", "★ 3. Sensor/Comms Fail"),
            ("randomized", "🎲 Randomized"),
            ("nominal", "Nominal"),
            ("bad_soil", "Bad Soil"),
            ("shifted_moisture", "Shifted Moisture"),
            ("obstacle", "Obstacle"),
        ):
            is_preset = scen_id in ("golden_rescue", "high_risk_abstain", "sensor_degraded", "randomized")
            tk.Radiobutton(
                toolbar,
                text=scen_lbl,
                variable=self.scenario_var,
                value=scen_id,
                command=self.reset_episode,
                bg="#111d2c",
                fg="#fef08a" if is_preset else "#e2e8f0",
                selectcolor="#0284c7",
                activebackground="#111d2c",
                font=("Segoe UI", 8, "bold" if is_preset else "normal"),
            ).pack(side=tk.LEFT, padx=1)

        # Policy Selector (Cooperative Rescue vs Mycelial SLAM Policy B vs Centralised Policy A vs Single Rover)
        tk.Label(toolbar, text="| Policy:", bg="#111d2c", fg="#38bdf8", font=("Segoe UI", 9, "bold")).pack(
            side=tk.LEFT, padx=(6, 2)
        )
        for pol_id, pol_lbl in (
            ("cooperative", "Cooperative Rescue (A+D)"),
            ("mycelial", "Mycelial SLAM (B)"),
            ("centralized", "Centralised (A)"),
            ("single_rover", "Single Rover"),
        ):
            tk.Radiobutton(
                toolbar,
                text=pol_lbl,
                variable=self.policy_var,
                value=pol_id,
                command=self.reset_episode,
                bg="#111d2c",
                fg="#e2e8f0",
                selectcolor="#0284c7",
                activebackground="#111d2c",
                font=("Segoe UI", 8, "bold"),
            ).pack(side=tk.LEFT, padx=1)

        # Comms Selector
        self.rf_label = tk.Label(toolbar, text="| RF:", bg="#111d2c", fg="#94a3b8", font=("Segoe UI", 9, "bold"))
        self.rf_label.pack(side=tk.LEFT, padx=(6, 2))
        self._comms_radios = []
        for c_id, c_lbl in (
            ("normal", "Normal"),
            ("intermittent", "Intermittent"),
            ("isolated", "Isolated"),
        ):
            rb = tk.Radiobutton(
                toolbar,
                text=c_lbl,
                variable=self.comms_var,
                value=c_id,
                command=self.reset_episode,
                bg="#111d2c",
                fg="#e2e8f0",
                disabledforeground="#475569",
                selectcolor="#0284c7",
                activebackground="#111d2c",
                font=("Segoe UI", 8),
            )
            rb.pack(side=tk.LEFT, padx=1)
            self._comms_radios.append(rb)

        self.status_lbl = tk.Label(
            toolbar,
            text="t = 0.00s",
            bg="#111d2c",
            fg="#38bdf8",
            font=("Consolas", 8, "bold"),
        )
        self.status_lbl.pack(side=tk.RIGHT, padx=8)

        # Row 2: Live Independent Benchmark Grader & D-0010 Equal-Work Comm Telemetry Bar
        bench_bar = tk.Frame(self.root, bg="#0d1826", highlightthickness=1, highlightbackground="#1f3247")
        bench_bar.pack(fill=tk.X, padx=8, pady=(0, 2))

        self.bench_metrics_lbl = tk.Label(
            bench_bar,
            text="Independent Benchmark Grader: initializing...",
            bg="#0d1826",
            fg="#38bdf8",
            font=("Consolas", 9, "bold"),
            anchor="w",
            padx=8,
            pady=3,
        )
        self.bench_metrics_lbl.pack(side=tk.LEFT, fill=tk.X, expand=True)

        tk.Button(
            bench_bar,
            text="📊 Live Metrics & On-Demand Baseline Runner",
            command=self._show_multiseed_benchmark_popup,
            bg="#1e293b",
            fg="#fbbf24",
            font=("Segoe UI", 8, "bold"),
            padx=8,
            pady=1,
        ).pack(side=tk.RIGHT, padx=6, pady=2)

        tk.Checkbutton(
            bench_bar,
            text="Show Inspector & Sliders",
            variable=self.show_details_var,
            command=self._toggle_details_drawer,
            bg="#0d1826",
            fg="#cbd5e1",
            selectcolor="#0284c7",
            activebackground="#0d1826",
            font=("Segoe UI", 8, "bold"),
        ).pack(side=tk.RIGHT, padx=6, pady=2)

        # Row 2A: Agent D Rescue Orchestrator & B Physical Recovery Telemetry Bar
        rescue_bar = tk.Frame(self.root, bg="#131c2e", highlightthickness=1, highlightbackground="#334155")
        rescue_bar.pack(fill=tk.X, padx=8, pady=(0, 2))

        tk.Label(
            rescue_bar,
            text="🧠 Rescue Policy (D):",
            bg="#131c2e",
            fg="#fbbf24",
            font=("Segoe UI", 8, "bold"),
        ).pack(side=tk.LEFT, padx=(6, 2))
        for rpol_id, rpol_lbl in (
            ("react_orchestrator", "D: ReAct Orchestrator"),
            ("numerical_triage", "Numerical Triage"),
            ("always_rescue", "Always Rescue"),
            ("never_rescue", "Never Rescue"),
        ):
            tk.Radiobutton(
                rescue_bar,
                text=rpol_lbl,
                variable=self.rescue_policy_var,
                value=rpol_id,
                command=self.reset_episode,
                bg="#131c2e",
                fg="#e2e8f0",
                selectcolor="#0284c7",
                activebackground="#131c2e",
                font=("Segoe UI", 8, "bold" if rpol_id == "react_orchestrator" else "normal"),
            ).pack(side=tk.LEFT, padx=1)

        self.rescue_banner_lbl = tk.Label(
            rescue_bar,
            text="Rescue Orchestrator (D) & Physical Recovery (B): Nominal Fieldwork",
            bg="#131c2e",
            fg="#a7f3d0",
            font=("Consolas", 8, "bold"),
            anchor="w",
            padx=8,
            pady=2,
        )
        self.rescue_banner_lbl.pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Row 2B: Compact Multi-Robot Color-Coded Status, Plan Summary & Emergency Teleop Bar
        plan_bar = tk.Frame(self.root, bg="#09131f", highlightthickness=1, highlightbackground="#1f3247")
        plan_bar.pack(fill=tk.X, padx=8, pady=(0, 3))

        self.robot_plans_lbl = tk.Label(
            plan_bar,
            text="Robot Status & Plans: initializing...",
            bg="#09131f",
            fg="#f8fafc",
            font=("Consolas", 9, "bold"),
            anchor="w",
            padx=8,
            pady=3,
        )
        self.robot_plans_lbl.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.teleop_pad_frame = tk.Frame(plan_bar, bg="#09131f")
        self.teleop_pad_frame.pack(side=tk.RIGHT, padx=6, pady=2)
        for btn_lbl, lin_v, ang_v in (
            ("▲ Fwd (W)", 0.34, 0.0),
            ("▼ Rev (S)", -0.24, 0.0),
            ("◀ Left (A)", 0.14, 0.95),
            ("▶ Right (D)", 0.14, -0.95),
            ("■ E-Stop (Space)", 0.0, 0.0),
        ):
            tk.Button(
                self.teleop_pad_frame,
                text=btn_lbl,
                command=lambda l=lin_v, a=ang_v: self.issue_manual_teleop(l, a),
                bg="#3b0764" if btn_lbl != "■ E-Stop (Space)" else "#991b1b",
                fg="#f3e8ff",
                font=("Segoe UI", 8, "bold"),
                padx=5,
                pady=1,
            ).pack(side=tk.LEFT, padx=1)

        # Row 3: BIG Middle Split Pane (Left = 840x560+ 3D MuJoCo View, Right = 660x560+ 2D Top-Down Belief Map + Inspector)
        mid_frame = tk.Frame(self.root, bg="#0b131e")
        self.mid_frame = mid_frame
        mid_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=2)

        # LEFT PANE: BIG 3D MuJoCo Visualiser
        left_pane = tk.LabelFrame(
            mid_frame,
            text=" BIG 3D MuJoCo Physics Visualiser (Undulating Micro-Topo + 3D Mud Sinkage & Wheel Ruts + Slip + UWB HUD) ",
            bg="#111d2c",
            fg="#38bdf8",
            font=("Segoe UI", 9, "bold"),
            padx=6,
            pady=4,
        )
        self.left_pane = left_pane
        left_pane.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 6))

        cam_bar = tk.Frame(left_pane, bg="#111d2c")
        cam_bar.pack(fill=tk.X, pady=(0, 4))
        tk.Label(cam_bar, text="3D Camera:", bg="#111d2c", fg="#94a3b8", font=("Segoe UI", 8, "bold")).pack(
            side=tk.LEFT, padx=(2, 4)
        )
        for cam_id, cam_lbl in (
            ("isometric", "Isometric Terrace"),
            ("wheel_closeup", "Low Mud/Wheel Close-Up"),
            ("planter_follow_cam", "Follow Planter"),
            ("scout_follow_cam", "Follow Scout"),
            ("planter_ego_cam", "Ego Optical Cam"),
            ("top_overview", "Top 3D"),
        ):
            tk.Radiobutton(
                cam_bar,
                text=cam_lbl,
                variable=self.camera_var,
                value=cam_id,
                command=self._on_camera_radio_change,
                bg="#111d2c",
                fg="#e2e8f0",
                selectcolor="#0284c7",
                activebackground="#111d2c",
                font=("Segoe UI", 8),
            ).pack(side=tk.LEFT, padx=2)

        tk.Button(
            cam_bar,
            text="Pop Out 3D GL Window",
            command=self._launch_optional_gl_viewer,
            bg="#1e293b",
            fg="#38bdf8",
            font=("Segoe UI", 8, "bold"),
            padx=6,
            pady=1,
        ).pack(side=tk.RIGHT, padx=2)

        self.canvas_3d = tk.Canvas(
            left_pane,
            width=self.RENDER_W,
            height=self.RENDER_H,
            bg="#070d14",
            highlightthickness=1,
            highlightbackground="#1f3247",
        )
        self.canvas_3d.pack(fill=tk.BOTH, expand=True)
        self._drag_last_xy: tuple[int, int] | None = None
        self.canvas_3d.bind("<ButtonPress-1>", self._on_3d_mouse_down)
        self.canvas_3d.bind("<B1-Motion>", self._on_3d_mouse_drag)
        self.canvas_3d.bind("<MouseWheel>", self._on_3d_mouse_wheel)
        self.canvas_3d.bind("<Configure>", self._on_3d_canvas_configure)

        # RIGHT PANE: BIG Interactive 2D Top-Down Belief Heatmap & Click-to-Select Robot
        right_pane = tk.LabelFrame(
            mid_frame,
            text=" BIG Interactive 2D Top-Down Belief & Rice-Fitness Visualiser (Click a Robot on Map or Select Below) ",
            bg="#111d2c",
            fg="#38bdf8",
            font=("Segoe UI", 9, "bold"),
            padx=6,
            pady=4,
        )
        self.right_pane = right_pane
        right_pane.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        who_bar = tk.Frame(right_pane, bg="#111d2c")
        who_bar.pack(fill=tk.X, pady=(0, 2))
        tk.Label(
            who_bar,
            text="Inspect World Model Of:",
            bg="#111d2c",
            fg="#fbbf24",
            font=("Segoe UI", 9, "bold"),
        ).pack(side=tk.LEFT, padx=(2, 4))

        self.who_buttons_frame = tk.Frame(who_bar, bg="#111d2c")
        self.who_buttons_frame.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self._refresh_belief_selector_buttons()

        layer_bar = tk.Frame(right_pane, bg="#111d2c")
        layer_bar.pack(fill=tk.X, pady=(0, 4))
        tk.Label(
            layer_bar,
            text="2D Field Layer:",
            bg="#111d2c",
            fg="#94a3b8",
            font=("Segoe UI", 8, "bold"),
        ).pack(side=tk.LEFT, padx=(2, 4))

        for l_id, l_short in (
            ("planting_fitness", "🌾 Rice Fitness Φ"),
            ("moisture", "Moisture M"),
            ("firmness", "Firmness F"),
            ("stand_gap", "Stand Gap G"),
            ("plant_density", "Plant Density ρ"),
            ("uncertainty", "Uncertainty σ"),
            ("traversability_risk", "Mud/SLAM Risk R"),
            ("elevation_m", "Micro-Topo z"),
        ):
            tk.Radiobutton(
                layer_bar,
                text=l_short,
                variable=self.layer_var,
                value=l_id,
                command=self._render_2d_belief_map,
                bg="#111d2c",
                fg="#e2e8f0",
                selectcolor="#0284c7",
                activebackground="#111d2c",
                font=("Segoe UI", 8),
            ).pack(side=tk.LEFT, padx=1)

        map_and_inspector = tk.Frame(right_pane, bg="#111d2c")
        map_and_inspector.pack(fill=tk.BOTH, expand=True)

        self.canvas_2d = tk.Canvas(
            map_and_inspector,
            width=self.MAP_W,
            height=self.MAP_H,
            bg="#070d14",
            highlightthickness=1,
            highlightbackground="#1f3247",
            cursor="hand2",
        )
        self.canvas_2d.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 6))
        self.canvas_2d.bind("<Button-1>", self._on_2d_canvas_click)
        self.canvas_2d.bind("<Configure>", self._on_2d_canvas_configure)

        self.inspector_frame = tk.Frame(
            map_and_inspector, bg="#0d1724", width=310, highlightthickness=1, highlightbackground="#1f3247"
        )
        if self.show_details_var.get():
            self.inspector_frame.pack(side=tk.RIGHT, fill=tk.Y, expand=False)
            self.inspector_frame.pack_propagate(False)

        self.inspector_title_lbl = tk.Label(
            self.inspector_frame,
            text="SELECTED ROBOT BELIEF INSPECTOR",
            bg="#16263b",
            fg="#38bdf8",
            font=("Segoe UI", 9, "bold"),
            pady=4,
        )
        self.inspector_title_lbl.pack(fill=tk.X)

        self.inspector_text = tk.Text(
            self.inspector_frame,
            bg="#0d1724",
            fg="#e2e8f0",
            font=("Consolas", 8),
            wrap=tk.WORD,
            padx=6,
            pady=4,
            borderwidth=0,
        )
        self.inspector_text.pack(fill=tk.BOTH, expand=True)

        # Row 4: Compact Bottom Dock (Bekker-Wong Sliders + Per-Robot Sinkage Cards + UWB Gossip Log)
        self.bottom_frame = tk.Frame(self.root, bg="#0b131e")
        if self.show_details_var.get():
            self.bottom_frame.pack(fill=tk.X, expand=False, padx=8, pady=(2, 5))

        sizing_frame = tk.LabelFrame(
            self.bottom_frame,
            text=" Live 3D Bekker-Wong Mud Sinkage, Wheel Slip & Vehicle Sizing Controls ",
            bg="#111d2c",
            fg="#38bdf8",
            font=("Segoe UI", 8, "bold"),
            padx=6,
            pady=2,
        )
        sizing_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=(0, 6))

        sliders_row = tk.Frame(sizing_frame, bg="#111d2c")
        sliders_row.pack(fill=tk.X)

        for lbl, var, vmin, vmax, res in (
            ("Robot Mass (kg):", self.mass_kg_var, 1.5, 16.0, 0.2),
            ("Wheel Radius (mm):", self.wheel_radius_mm_var, 35.0, 100.0, 1.0),
            ("Wheel Width (mm):", self.wheel_width_mm_var, 20.0, 85.0, 1.0),
        ):
            col = tk.Frame(sliders_row, bg="#111d2c")
            col.pack(side=tk.LEFT, padx=5)
            tk.Label(col, text=lbl, bg="#111d2c", fg="#cbd5e1", font=("Segoe UI", 8, "bold")).pack(anchor="w")
            tk.Scale(
                col,
                variable=var,
                from_=vmin,
                to=vmax,
                resolution=res,
                orient=tk.HORIZONTAL,
                length=135,
                bg="#111d2c",
                fg="#38bdf8",
                highlightthickness=0,
                troughcolor="#070d14",
                command=lambda _v: self._on_sizing_slider_change(),
            ).pack()

        self.terramech_cards_lbl = tk.Label(
            sizing_frame,
            text="",
            bg="#0d1724",
            fg="#e2e8f0",
            font=("Consolas", 8),
            justify=tk.LEFT,
            anchor="w",
            padx=6,
            pady=3,
        )
        self.terramech_cards_lbl.pack(fill=tk.X, pady=(2, 0))

        gossip_frame = tk.LabelFrame(
            self.bottom_frame,
            text=" Peer-to-Peer UWB / Wi-Fi HaLow Mycelial Gossip & Task Negotiation Log ",
            bg="#111d2c",
            fg="#38bdf8",
            font=("Segoe UI", 8, "bold"),
            padx=6,
            pady=2,
        )
        gossip_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.gossip_text = tk.Text(
            gossip_frame,
            height=5,
            bg="#070d14",
            fg="#cbd5e1",
            font=("Consolas", 8),
            wrap=tk.WORD,
            padx=6,
            pady=3,
            borderwidth=0,
        )
        self.gossip_text.pack(fill=tk.BOTH, expand=True)
        self._advanced_top_bars = [
            self.main_menu_bar, toolbar, bench_bar, rescue_bar, plan_bar
        ]
        self._advanced_inline_bars = [cam_bar, who_bar, layer_bar]
        if self.presentation_mode:
            self._apply_presentation_layout()

    def toggle_presentation_mode(self) -> None:
        self.presentation_mode = not self.presentation_mode
        self._apply_presentation_layout()
        self._refresh_all_views()

    def _apply_presentation_layout(self) -> None:
        """Presentation-only widget packing; the EpisodeSession is untouched."""
        if self.presentation_mode:
            for widget in self._advanced_top_bars + self._advanced_inline_bars:
                widget.pack_forget()
            self.inspector_frame.pack_forget()
            self.bottom_frame.pack_forget()
            self.presentation_controls.pack(side=tk.RIGHT, padx=4)
            self.presentation_event_lbl.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            self.mode_btn.configure(text="Engineering / Advanced")
            self.left_pane.configure(text=" LIVE 3D PHYSICS   /   ROBOTS & TERRAIN ")
            self.right_pane.configure(text=" DISCOVERED CONDITIONS   /   ACTUAL MOVEMENT ")
            self._position_presentation_camera()
        else:
            for widget in self._advanced_top_bars:
                widget.pack(before=self.mid_frame, fill=tk.X, padx=8, pady=(0, 2))
            self._advanced_inline_bars[0].pack(
                before=self.canvas_3d, fill=tk.X, pady=(0, 4)
            )
            for widget in self._advanced_inline_bars[1:]:
                widget.pack(before=self.canvas_2d.master, fill=tk.X, pady=(0, 3))
            if self.show_details_var.get():
                self.inspector_frame.pack(side=tk.RIGHT, fill=tk.Y, expand=False)
                self.inspector_frame.pack_propagate(False)
                self.bottom_frame.pack(fill=tk.X, padx=8, pady=(2, 5))
            self.presentation_controls.pack_forget()
            self.presentation_event_lbl.pack_forget()
            self.mode_btn.configure(text="Presentation Mode")
            self.left_pane.configure(
                text=" BIG 3D MuJoCo Physics Visualiser (Undulating Micro-Topo + 3D Mud Sinkage & Wheel Ruts + Slip + UWB HUD) "
            )
            self.right_pane.configure(
                text=" BIG Interactive 2D Top-Down Belief & Rice-Fitness Visualiser (Click a Robot on Map or Select Below) "
            )

    def set_presentation_camera(self, mode: str) -> None:
        if mode not in ("field", "rescue", "auto"):
            raise ValueError("Unrecognised presentation camera preset")
        self.presentation_camera_mode.set(mode)
        self._position_presentation_camera()
        self._render_3d_view()

    def _position_presentation_camera(self) -> None:
        if not self.presentation_mode:
            return
        cue = self.presentation_cue
        poses = {r.robot_id: r.pose for r in self.frame.robots}
        mode = self.presentation_camera_mode.get()
        close = mode == "rescue" or (mode == "auto" and cue.victim_id in poses)
        if close and cue.victim_id in poses:
            victim = poses[cue.victim_id]
            helper = poses.get(cue.helper_id) if cue.helper_id else None
            if helper is not None:
                x = (victim.x_m + helper.x_m) / 2
                y = (victim.y_m + helper.y_m) / 2
                separation = hypot(victim.x_m - helper.x_m, victim.y_m - helper.y_m)
                distance = max(1.35, min(3.7, 1.55 + separation))
            else:
                x, y, distance = victim.x_m, victim.y_m, 1.8
        else:
            (xmin, xmax), (ymin, ymax) = self._field_bounds()
            x, y = (xmin + xmax) / 2, (ymin + ymax) / 2
            distance = 3.1 * max(1.0, self.map_scale)
        self.free_cam.lookat[:] = [x, y, 0.05]
        self.free_cam.distance = distance
        self.free_cam.elevation = -38.0
        self.free_cam.azimuth = 118.0

    def _update_presentation_cue(self) -> None:
        cue = presentation_cue(
            self.frame, self.decision,
            fault_evidence=self.session.fault_evidence,
            rescue_assessments=self.session.rescue_assessments,
            orchestrator_decisions=self.session.orchestrator_decisions,
            completed_recoveries=self.sim.completed_recoveries,
        )
        self.presentation_cue = cue
        self.presentation_event_lbl.configure(
            text=f"LIVE  ·  {cue.label}  /  {cue.detail}"
        )
        if not self.presentation_events or self.presentation_events[-1][1] != cue.label:
            self.presentation_events.append((self.frame.time_s, cue.label))
            self.presentation_events = self.presentation_events[-6:]

    # ------------------------------------------------------------------
    # Dynamic Resize & Interactive Camera/Canvas Handlers
    # ------------------------------------------------------------------

    def _on_3d_canvas_configure(self, event: tk.Event) -> None:  # type: ignore[type-arg]
        new_w = max(480, min(1240, int(event.width) - 4))
        new_h = max(360, min(920, int(event.height) - 4))
        if abs(new_w - self.RENDER_W) >= 28 or abs(new_h - self.RENDER_H) >= 28:
            self.RENDER_W = new_w
            self.RENDER_H = new_h
            try:
                if hasattr(self, "renderer") and self.renderer is not None:
                    self.renderer.close()
                self.renderer = mujoco.Renderer(self.sim.model, height=self.RENDER_H, width=self.RENDER_W)
                self._render_3d_view()
            except Exception:
                pass

    def _on_2d_canvas_configure(self, event: tk.Event) -> None:  # type: ignore[type-arg]
        new_w = max(420, int(event.width) - 4)
        new_h = max(360, int(event.height) - 4)
        if abs(new_w - self.MAP_W) >= 16 or abs(new_h - self.MAP_H) >= 16:
            self.MAP_W = new_w
            self.MAP_H = new_h
            self._render_2d_belief_map()

    def _on_camera_radio_change(self) -> None:
        cam_id = self.camera_var.get()
        if cam_id == "wheel_closeup":
            sel = self.selected_robot_var.get()
            r_xyz = self.sim.data.sensor(f"{sel}_pos").data
            self.free_cam.lookat[:] = [float(r_xyz[0]), float(r_xyz[1]), 0.03]
            self.free_cam.distance = 0.72
            self.free_cam.elevation = -14.0
            self.free_cam.azimuth = 128.0
        elif cam_id == "isometric":
            self.free_cam.lookat[:] = [1.22, -0.05, 0.05]
            self.free_cam.distance = 2.55
            self.free_cam.elevation = -28.0
            self.free_cam.azimuth = 118.0
        self._render_3d_view()

    def _on_3d_mouse_down(self, event: tk.Event) -> None:  # type: ignore[type-arg]
        self._drag_last_xy = (event.x, event.y)

    def _on_3d_mouse_drag(self, event: tk.Event) -> None:  # type: ignore[type-arg]
        if self._drag_last_xy is None:
            return
        dx = event.x - self._drag_last_xy[0]
        dy = event.y - self._drag_last_xy[1]
        self._drag_last_xy = (event.x, event.y)
        if self.camera_var.get() not in ("isometric", "wheel_closeup"):
            self.camera_var.set("isometric")
        self.free_cam.azimuth = (self.free_cam.azimuth - dx * 0.45) % 360.0
        self.free_cam.elevation = max(-85.0, min(-5.0, self.free_cam.elevation - dy * 0.35))
        self._render_3d_view()

    def _on_3d_mouse_wheel(self, event: tk.Event) -> None:  # type: ignore[type-arg]
        if self.camera_var.get() not in ("isometric", "wheel_closeup"):
            self.camera_var.set("isometric")
        delta = -0.16 if event.delta > 0 else 0.16
        self.free_cam.distance = max(0.55, min(5.5, self.free_cam.distance + delta))
        self._render_3d_view()

    def _refresh_belief_selector_buttons(self) -> None:
        """Rebuild dynamic belief perspective radio buttons for ALL (Fused), every active rover, Divergence, and God View."""
        if not hasattr(self, "who_buttons_frame") or self.who_buttons_frame is None:
            return
        for child in self.who_buttons_frame.winfo_children():
            child.destroy()
        self.belief_radio_buttons: dict[str, tk.Radiobutton] = {}

        cur_frame = (
            self.session.current_frame
            if hasattr(self, "session") and self.session is not None and self.session.current_frame is not None
            else getattr(self, "frame", None)
        )
        active_rids = [r.robot_id for r in cur_frame.robots] if cur_frame is not None else ["planter", "scout"]

        options: list[tuple[str, str, str]] = [("fused", "🌐 ALL (Fused)", "#1e293b")]
        for rid in active_rids:
            options.append((rid, f"🤖 {rid.upper()}", "#0c4a6e" if "planter" in rid else "#14532d"))
        options.append(("divergence", "⚡ Divergence", "#451a03"))
        options.append(("ground_truth", "🌍 True Field", "#3b0764"))

        for w_id, w_lbl, w_bg in options:
            rb = tk.Radiobutton(
                self.who_buttons_frame,
                text=w_lbl,
                variable=self.belief_view_var,
                value=w_id,
                command=self._on_belief_view_button,
                bg=w_bg,
                fg=_robot_hex_color(w_id) if w_id in active_rids else "#ffffff",
                selectcolor="#0284c7",
                activebackground=w_bg,
                indicatoron=False,
                font=("Segoe UI", 8, "bold"),
                padx=6,
                pady=2,
            )
            rb.pack(side=tk.LEFT, padx=2)
            self.belief_radio_buttons[w_id] = rb

    def select_belief_robot(self, view_or_robot_id: str) -> None:
        """Select a belief perspective ('fused', any active robot_id, 'divergence', or 'ground_truth') and refresh."""
        self.selected_belief_robot = view_or_robot_id
        self._on_belief_view_button()

    def _on_belief_view_button(self) -> None:
        v = self.belief_view_var.get()
        cur_frame = (
            self.session.current_frame
            if hasattr(self, "session") and self.session is not None and self.session.current_frame is not None
            else getattr(self, "frame", None)
        )
        active_rids = {r.robot_id for r in cur_frame.robots} if cur_frame is not None else {"planter", "scout"}
        if v in active_rids:
            self.selected_robot_var.set(v)
        self._refresh_all_views()

    def _on_2d_canvas_click(self, event: tk.Event) -> None:  # type: ignore[type-arg]
        """Allow clicking directly on any active rover in a 2..6 rover fleet on the 2D map to inspect its belief."""
        wx, wy = self._canvas_to_world(event.x, event.y)
        best_rid: str | None = None
        best_d = 0.36 * max(1.0, 0.85 * float(getattr(self, "map_scale", 1.0)))
        cur_frame = (
            self.session.current_frame
            if hasattr(self, "session") and self.session is not None and self.session.current_frame is not None
            else self.frame
        )
        obs_by_id = {r.robot_id: r for r in cur_frame.robots}
        for rid in [r.robot_id for r in cur_frame.robots]:
            r = obs_by_id[rid]
            d = hypot(wx - r.pose.x_m, wy - r.pose.y_m)
            if d < best_d:
                best_d = d
                best_rid = rid
        if best_rid is not None:
            self.selected_robot_id = best_rid
            self.selected_belief_robot = best_rid
            self._refresh_all_views()

    def _on_sizing_slider_change(self) -> None:
        self._apply_vehicle_sizing_to_sim()
        self._refresh_all_views()

    def trigger_mud_belly_out_demo(self) -> None:
        """Set selected rover mass to 13.0 kg and wheel radius to 40 mm and switch to close-up camera."""
        self.mass_kg_var.set(13.0)
        self.wheel_radius_mm_var.set(40.0)
        self.wheel_width_mm_var.set(26.0)
        self._apply_vehicle_sizing_to_sim()
        self.camera_var.set("wheel_closeup")
        self._on_camera_radio_change()
        self._refresh_all_views()

    def restore_nominal_sizing(self) -> None:
        self.mass_kg_var.set(4.2)
        self.wheel_radius_mm_var.set(58.0)
        self.wheel_width_mm_var.set(40.0)
        self._apply_vehicle_sizing_to_sim()
        self._refresh_all_views()

    def _toggle_details_drawer(self) -> None:
        """Show or hide the secondary inspector and bottom sliders so 3D and 2D viewports stay maximized."""
        if self.show_details_var.get():
            self.inspector_frame.pack(side=tk.RIGHT, fill=tk.Y, expand=False)
            self.inspector_frame.pack_propagate(False)
            self.bottom_frame.pack(fill=tk.X, expand=False, padx=8, pady=(2, 5))
        else:
            self.inspector_frame.pack_forget()
            self.bottom_frame.pack_forget()
        self._refresh_all_views()

    def _bind_teleop_keys(self) -> None:
        """Bind WASD and Space keys for Emergency Human Control teleoperation."""
        self.root.bind("<KeyPress-w>", lambda _e: self.issue_manual_teleop(0.34, 0.0))
        self.root.bind("<KeyPress-s>", lambda _e: self.issue_manual_teleop(-0.24, 0.0))
        self.root.bind("<KeyPress-a>", lambda _e: self.issue_manual_teleop(0.14, 0.95))
        self.root.bind("<KeyPress-d>", lambda _e: self.issue_manual_teleop(0.14, -0.95))
        self.root.bind("<space>", lambda _e: self.issue_manual_teleop(0.0, 0.0))

    def toggle_emergency_human_control(self) -> None:
        """Toggle Emergency Human Control override on the selected robot."""
        active = not self.human_control_var.get()
        self.human_control_var.set(active)
        self.manual_lin_m_s = 0.0
        self.manual_ang_rad_s = 0.0
        sel_rid = self.selected_robot_var.get()
        t_now = self.frame.time_s if hasattr(self, "frame") and self.frame is not None else 0.0
        if active:
            sel = sel_rid.upper()
            self.human_ctrl_btn.configure(
                text=f"🛑 HUMAN CONTROL ACTIVE ({sel})",
                bg="#9333ea",
                fg="#ffffff",
            )
            self.session.submit_operator_request(
                OperatorRequest(
                    request_id=f"teleop-on-{self.step_count}",
                    kind=OperatorCommandKind.TELEOP_OVERRIDE,
                    issued_at_s=t_now,
                    payload=(("robot_id", sel_rid), ("active", True), ("linear_m_s", 0.0), ("angular_rad_s", 0.0)),
                )
            )
        else:
            self.human_ctrl_btn.configure(
                text="🛑 Emergency Human Control: OFF",
                bg="#4c1d95",
                fg="#f3e8ff",
            )
            self.session.submit_operator_request(
                OperatorRequest(
                    request_id=f"teleop-off-{self.step_count}",
                    kind=OperatorCommandKind.TELEOP_OVERRIDE,
                    issued_at_s=t_now,
                    payload=(("robot_id", sel_rid), ("active", False)),
                )
            )
        self._refresh_all_views()

    def issue_manual_teleop(self, lin_m_s: float, ang_rad_s: float) -> None:
        """Engage Emergency Human Control and apply manual velocity command to the selected rover."""
        if not self.human_control_var.get():
            self.toggle_emergency_human_control()
        self.manual_lin_m_s = float(lin_m_s)
        self.manual_ang_rad_s = float(ang_rad_s)
        sel_rid = self.selected_robot_var.get()
        t_now = self.frame.time_s if hasattr(self, "frame") and self.frame is not None else 0.0
        self.session.submit_operator_request(
            OperatorRequest(
                request_id=f"teleop-cmd-{self.step_count}",
                kind=OperatorCommandKind.TELEOP_OVERRIDE,
                issued_at_s=t_now,
                payload=(
                    ("robot_id", sel_rid),
                    ("active", True),
                    ("linear_m_s", self.manual_lin_m_s),
                    ("angular_rad_s", self.manual_ang_rad_s),
                ),
            )
        )
        if not self.playing:
            self.step_once()
        else:
            self._refresh_all_views()

    def _format_robot_plan_sentence(self, rid: str) -> tuple[str, str, str]:
        """Return (status_label, hex_color, human_readable_plan) for a robot."""
        sink = self.sim.latest_terramechanics.get(rid)
        obs_by_id = {r.robot_id: r for r in self.frame.robots}
        r_obs = obs_by_id.get(rid)
        bellied = bool((sink and sink.bellied_out) or (r_obs and r_obs.is_bogged))
        if self.human_control_var.get() and rid == self.selected_robot_var.get():
            return (
                "HUMAN CONTROL",
                "#a855f7",
                f"{rid.upper()} is under Emergency Human Control (v={self.manual_lin_m_s:+.2f}m/s, ω={self.manual_ang_rad_s:+.2f}r/s)",
            )
        intents_by_id = {i.robot_id: i for i in self.decision.intents}
        intent = intents_by_id.get(rid)
        if intent is not None and intent.mode == TaskMode.RESCUE:
            tech = intent.recovery_technique or "rigid_dock_tow"
            tgt = (intent.target_robot_id or "peer").upper()
            return (
                "RESCUING PEER",
                "#f59e0b",
                f"{rid.upper()} is executing [{tech}] to extract stranded {tgt}",
            )
        if bellied:
            orch_dec = self.session.orchestrator_decisions.get(rid)
            if orch_dec is not None and orch_dec.verdict == RescueVerdict.ABANDON_ESCALATE:
                return (
                    "STRANDED (REFUSED)",
                    "#ef4444",
                    f"{rid.upper()} bogged -> D REFUSED rescue (protecting seed-carrier)",
                )
            helper = (
                orch_dec.helper_robot_id.upper()
                if orch_dec and orch_dec.helper_robot_id
                else ("SCOUT" if rid == "planter" else "PLANTER")
            )
            return (
                "IMMOBILIZED (S1)",
                "#ef4444",
                f"{rid.upper()} is immobilized in mud -> {helper} assessing/executing physical rescue",
            )
        if intent is None or intent.mode == TaskMode.HOLD:
            return ("HOLD / STANDBY", "#94a3b8", f"{rid.upper()} is holding position on standby")
        if intent.mode == TaskMode.PLANT:
            return (
                "PLANTING",
                "#38bdf8",
                f"{rid.upper()} is going to plant seed at {intent.target_site_id or 'site'}",
            )
        return (
            "SAMPLING",
            "#22c55e",
            f"{rid.upper()} is going to sample soil at {intent.target_site_id or 'site'}",
        )

    def _launch_optional_gl_viewer(self) -> None:
        try:
            import mujoco.viewer

            if self._gl_viewer is None or not self._gl_viewer.is_running():
                self._gl_viewer = mujoco.viewer.launch_passive(self.sim.model, self.sim.data)
                self._gl_viewer.cam.lookat[:] = [1.22, -0.05, 0.05]
                self._gl_viewer.cam.distance = 2.55
                self._gl_viewer.cam.elevation = -28.0
                self._gl_viewer.cam.azimuth = 118.0
        except Exception:
            pass

    def _show_multiseed_benchmark_popup(self) -> None:
        """Display live-computed episode metrics and provide an on-demand runner for matched baselines."""
        win = tk.Toplevel(self.root)
        win.title("Live Episode Metrics & On-Demand Baseline Evaluation")
        win.geometry("980x580")
        win.configure(bg="#0b131e")

        top_bar = tk.Frame(win, bg="#111d2c")
        top_bar.pack(fill=tk.X, padx=8, pady=6)

        txt = tk.Text(
            win,
            bg="#0d1724",
            fg="#e2e8f0",
            font=("Consolas", 9),
            padx=12,
            pady=10,
            borderwidth=0,
        )
        txt.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        def _render_live_summary(baseline_rows: list[str] | None = None) -> None:
            bm = self._compute_live_benchmark_metrics()
            planted_sites = [
                d.target_site_id for d in self.sim.completed_drops if d.contact_observed_at_s is not None
            ]
            lines = [
                "=== 1. CURRENT LIVE EPISODE TELEMETRY (COMPUTED FROM ACTIVE SESSION) ===",
                f"Seed               : {self.seed}",
                f"Scenario           : {self.scenario_var.get()}",
                f"Swarm Policy       : {self.policy_var.get()}",
                f"Rescue Policy (D)  : {self.rescue_policy_var.get()}",
                f"Comms Mode         : {self.comms_var.get()}",
                f"Simulation Step    : {self.step_count} (t = {self.frame.time_s:.2f}s)",
                f"Viable Seeds       : {bm['viable_planted']} / {bm['viable_target']} (Planted sites: {planted_sites or 'none'})",
                f"Wasted Drops       : {bm['wasted_drops']}",
                f"Physical Recoveries: {len(self.sim.completed_recoveries)}",
                f"Discrete Site RMSE : {bm['discrete_rmse']:.4f}",
                f"2D Field Map RMSE  : {bm['cont_rmse']:.4f}",
                f"D-0010 Wire Bytes  : {bm['actual_wire_bytes']} B vs {bm['stream_10hz_bytes']} B (10Hz stream) -> {bm['saved_pct']:+.1f}% saved",
                "",
                "=== 2. ACTIVE EPISODE ORCHESTRATOR (D) & RECOVERY (B) LOG ===",
            ]
            if self.session.orchestrator_decisions:
                for vid, odec in self.session.orchestrator_decisions.items():
                    mode_tag = "DETERMINISTIC FALLBACK" if odec.fallback_used else "LIVE LLM"
                    tools_str = " -> ".join(tc.tool_name for tc in odec.tool_trace) if odec.tool_trace else "none"
                    lines.append(
                        f"• Stranded={vid} | Verdict={odec.verdict.value} | Mode={mode_tag} | "
                        f"Helper={odec.helper_robot_id or 'none'} | Tech={odec.chosen_technique.value} | Tools={tools_str}"
                    )
                    lines.append(f"  Rationale: {odec.rationale}")
            else:
                lines.append("• No active orchestrator rescue decisions at current tick.")
            if self.sim.completed_recoveries:
                for rec in self.sim.completed_recoveries:
                    lines.append(
                        f"• Recovery @ t={rec.observed_at_s:.1f}s: {rec.helper_robot_id} extracted {rec.target_robot_id} "
                        f"(displacement={rec.victim_displacement_m:.2f}m, peak_force={rec.peak_constraint_force_n:.1f}N)"
                    )
            lines.append("")
            lines.append("=== 3. ON-DEMAND MATCHED-SCENARIO BASELINE RUNNER (GENERATED BY ACTUAL RUNS ONLY) ===")
            if baseline_rows is None:
                lines.append(
                    "Click '▶ Run Live Matched-Scenario Comparison Now' above to execute all 4 rescue policies "
                    "on golden_rescue and high_risk_abstain for the current seed and display actual measured results."
                )
            else:
                lines.extend(baseline_rows)
            txt.configure(state=tk.NORMAL)
            txt.delete("1.0", tk.END)
            txt.insert(tk.END, "\n".join(lines))
            txt.configure(state=tk.DISABLED)

        def _run_live_comparison() -> None:
            from ..runtime import run_baseline_comparison

            txt.configure(state=tk.NORMAL)
            txt.insert(tk.END, "\n\nRunning live MuJoCo baseline comparison for seed=" + str(self.seed) + "...\n")
            txt.configure(state=tk.DISABLED)
            win.update_idletasks()
            rows = run_baseline_comparison(seed=self.seed)
            out_lines = [
                f"{'Scenario':18s} | {'Rescue Policy':18s} | {'Recoveries':10s} | {'Planted':18s} | {'End Bogged':18s} | {'Orch Decisions'}",
                "-" * 110,
            ]
            for r in rows:
                out_lines.append(
                    f"{str(r['scenario']):18s} | {str(r['rescue_policy']):18s} | "
                    f"{int(r['completed_recoveries']):10d} | {str(r['planted_sites']):18s} | "
                    f"{str(r['bogged_robots']):18s} | {str(r['orchestrator_decisions'])}"
                )
            _render_live_summary(out_lines)

        tk.Button(
            top_bar,
            text="↻ Refresh Current Episode Telemetry",
            command=lambda: _render_live_summary(None),
            bg="#0284c7",
            fg="white",
            font=("Segoe UI", 8, "bold"),
            padx=8,
            pady=2,
        ).pack(side=tk.LEFT, padx=4)

        tk.Button(
            top_bar,
            text="▶ Run Live Matched-Scenario Comparison Now",
            command=_run_live_comparison,
            bg="#15803d",
            fg="white",
            font=("Segoe UI", 8, "bold"),
            padx=8,
            pady=2,
        ).pack(side=tk.LEFT, padx=4)

        _render_live_summary(None)

    # ------------------------------------------------------------------
    # Simulation Stepping & Play Loop (Authoritative EpisodeSession)
    # ------------------------------------------------------------------

    def toggle_play(self) -> None:
        self.playing = not self.playing
        if hasattr(self, "presentation_play_btn"):
            self.presentation_play_btn.configure(text="Pause" if self.playing else "Play")
        if self.playing:
            self.play_btn.configure(text="⏸ Pause Sim", bg="#d97706")
            self._tick_loop()
        else:
            self.play_btn.configure(text="▶ Play Live 3D Sim", bg="#0284c7")
            if self.after_id is not None:
                self.root.after_cancel(self.after_id)
                self.after_id = None

    def _tick_loop(self) -> None:
        if not self.playing:
            return
        self.step_once()
        max_steps = max(160, self.session.config.steps)
        if self.step_count >= max_steps:
            self.toggle_play()
            return
        self.after_id = self.root.after(45, self._tick_loop)

    def step_once(self) -> None:
        t0 = time.perf_counter()
        if self.human_control_var.get():
            sel_rid = self.selected_robot_var.get()
            self.session.submit_operator_request(
                OperatorRequest(
                    request_id=f"teleop-step-{self.step_count}",
                    kind=OperatorCommandKind.TELEOP_OVERRIDE,
                    issued_at_s=self.frame.time_s,
                    payload=(
                        ("robot_id", sel_rid),
                        ("active", True),
                        ("linear_m_s", self.manual_lin_m_s),
                        ("angular_rad_s", self.manual_ang_rad_s),
                    ),
                )
            )
        tick = self.session.step_once()
        self.frame = tick.frame
        self.decision = tick.decision
        if self._central_core is not None:
            self._central_samples_count += self.decision.ingested_samples
        self.last_compute_ms = (time.perf_counter() - t0) * 1000.0
        self.step_count = self.session.tick_index
        for r in self.frame.robots:
            self.trajectories.setdefault(r.robot_id, []).append((r.pose.x_m, r.pose.y_m))
        # A helper's approach line is constructed from actual poses acquired
        # after an actual assignment, never from fabricated path waypoints.
        cue = presentation_cue(
            self.frame, self.decision,
            fault_evidence=self.session.fault_evidence,
            rescue_assessments=self.session.rescue_assessments,
            orchestrator_decisions=self.session.orchestrator_decisions,
            completed_recoveries=self.sim.completed_recoveries,
        )
        if cue.victim_id and cue.helper_id and cue.phase in (
            "assigned", "approach", "docking", "extracting", "extracted", "resumed"
        ):
            for robot in self.frame.robots:
                if robot.robot_id == cue.helper_id:
                    track = self.rescue_approach_tracks.setdefault(
                        (cue.victim_id, cue.helper_id), []
                    )
                    track.append((robot.pose.x_m, robot.pose.y_m))
                    if len(track) > 480:
                        del track[:-480]
                    break
        self._update_visual_camera_discoveries()
        if self._gl_viewer is not None and self._gl_viewer.is_running():
            self._gl_viewer.user_scn.ngeom = 0
            self._append_3d_hud_geoms(self._gl_viewer.user_scn)
            self._gl_viewer.sync()
        self._refresh_all_views()

    def _update_comms_applicability_ui(self) -> None:
        """Disable RF controls and annotate label when communication settings do not apply to the active policy."""
        applies = self.comms_applies
        if hasattr(self, "rf_label"):
            self.rf_label.configure(
                text="| RF:" if applies else "| RF (N/A: Shared Map):",
                fg="#94a3b8" if applies else "#64748b",
            )
        for rb in getattr(self, "_comms_radios", ()):
            rb.configure(state=tk.NORMAL if applies else tk.DISABLED)

    def reset_episode(self) -> None:
        self._rebuild_simulation()

    def trigger_random_scenario(self) -> None:
        """Generate and launch a new reproducible Randomized Episode with a fresh seed."""
        candidates = [s for s in range(1, 250) if s != self.seed]
        chosen_seed = random.choice(candidates)
        self.seed_var.set(chosen_seed)
        if hasattr(self, "seed_entry_var"):
            self.seed_entry_var.set(str(chosen_seed))
        self.scenario_var.set("randomized")
        self.reset_episode()
        spec = self.randomized_spec
        comms_note = self.comms_var.get() if self.comms_applies else "N/A (Shared Map)"
        if spec is not None:
            self.status_lbl.configure(
                text=(
                    f"🎲 Seed={chosen_seed} | Viable={','.join(s[-1].upper() for s in spec.viable_sites)} | "
                    f"Trap={spec.trap_site_id}({spec.mud_severity}) | RF={comms_note}"
                )
            )
        else:
            self.status_lbl.configure(
                text=f"🎲 Randomized Episode -> Seed={chosen_seed} | RF={comms_note}"
            )

    # ------------------------------------------------------------------
    # Independent Benchmark Grader & D-0010 Comm Metrics Calculation
    # ------------------------------------------------------------------

    def _compute_live_benchmark_metrics(self) -> dict[str, Any]:
        """Compute live Independent Grader metrics (Viable Seeds, Wasted Drops, Discrete & 2D RMSE, D-0010 Wire Bytes)."""
        envelope = SoilSafetyEnvelope()
        viable_target = sum(
            1
            for sid, site in self.sim.sites.items()
            if site.permitted
            and sid in self.hidden_truth
            and self.hidden_truth[sid][SensorKind.STAND_GAP] >= envelope.min_gap
            and envelope.moisture_range[0] <= self.hidden_truth[sid][SensorKind.MOISTURE] <= envelope.moisture_range[1]
            and envelope.firmness_range[0] <= self.hidden_truth[sid][SensorKind.FIRMNESS] <= envelope.firmness_range[1]
        )
        viable_planted = 0
        wasted_drops = 0
        for d in self.sim.completed_drops:
            if d.contact_observed_at_s is None:
                continue
            st = self.hidden_truth.get(d.target_site_id)
            if (
                st is not None
                and st[SensorKind.STAND_GAP] >= envelope.min_gap
                and envelope.moisture_range[0] <= st[SensorKind.MOISTURE] <= envelope.moisture_range[1]
                and envelope.firmness_range[0] <= st[SensorKind.FIRMNESS] <= envelope.firmness_range[1]
            ):
                viable_planted += 1
            else:
                wasted_drops += 1

        # Discrete Site Map RMSE across permitted patches (planter's belief vs hidden_truth)
        p_map = self.runtime.nodes["planter"].local_map
        sq_errs: list[float] = []
        for sid, site in self.sim.sites.items():
            if not site.permitted:
                continue
            est = p_map.estimate(sid)
            truth_ch = self.hidden_truth.get(sid, {})
            for kind, est_v in (
                (SensorKind.MOISTURE, est.moisture),
                (SensorKind.FIRMNESS, est.firmness),
                (SensorKind.STAND_GAP, est.stand_gap),
            ):
                true_v = truth_ch.get(kind, 0.5)
                pred_v = est_v if est_v is not None else 0.5
                sq_errs.append((pred_v - true_v) ** 2)
        discrete_rmse = sqrt(sum(sq_errs) / max(1, len(sq_errs)))

        # Continuous 2D Field RMSE (moisture + firmness + planting_fitness) using cached grids
        x_b, y_b = self._field_bounds()
        p_grid = sample_continuous_belief_grid(
            p_map, nx=self.GRID_NX, ny=self.GRID_NY, x_bounds=x_b, y_bounds=y_b
        )
        t_layers = self.truth_grid["layers"]
        cont_mse = (
            float(np.mean((np.asarray(p_grid["moisture"]) - np.asarray(t_layers["moisture"])) ** 2))
            + float(np.mean((np.asarray(p_grid["firmness"]) - np.asarray(t_layers["firmness"])) ** 2))
            + float(np.mean((np.asarray(p_grid["planting_fitness"]) - np.asarray(t_layers["planting_fitness"])) ** 2))
        ) / 3.0
        cont_rmse = sqrt(cont_mse)

        # D-0010 Equal-Work Communication Accounting
        stream_10hz_bytes = max(1, self.step_count) * 58 * max(2, len(self.frame.robots))
        pol = self.policy_var.get()
        if pol == "mycelial":
            actual_wire_bytes = self.runtime.total_gossip_bytes
        elif pol == "centralized":
            actual_wire_bytes = stream_10hz_bytes + 28 * self._central_samples_count
        else:
            actual_wire_bytes = 0
        saved_pct = (
            100.0 * (1.0 - (actual_wire_bytes / max(1, stream_10hz_bytes)))
            if stream_10hz_bytes > 0
            else 0.0
        )
        return {
            "viable_planted": viable_planted,
            "viable_target": max(1, viable_target),
            "wasted_drops": wasted_drops,
            "discrete_rmse": discrete_rmse,
            "cont_rmse": cont_rmse,
            "actual_wire_bytes": actual_wire_bytes,
            "stream_10hz_bytes": stream_10hz_bytes,
            "saved_pct": saved_pct,
        }

    # ------------------------------------------------------------------
    # Rendering: 3D MuJoCo View + Vectorized 2D Top-Down Map + Inspector
    # ------------------------------------------------------------------

    def _refresh_all_views(self) -> None:
        self._update_presentation_cue()
        self._update_comms_applicability_ui()
        planted_sites = [
            d.target_site_id for d in self.sim.completed_drops if d.contact_observed_at_s is not None
        ]
        bm = self._compute_live_benchmark_metrics()
        comms_tag = self.comms_var.get() if self.comms_applies else "N/A(Shared)"
        if self.randomized_spec is not None:
            sp = self.randomized_spec
            obs_tag = f"obs@{sp.inject_obstacle_at_s:.1f}s" if sp.inject_obstacle_at_s is not None else "no-obs"
            self.status_lbl.configure(
                text=(
                    f"Seed {self.seed} [Viable:{','.join(s[-1].upper() for s in sp.viable_sites)} "
                    f"Trap:{sp.trap_site_id[-1].upper()}({sp.mud_severity[:5]}) {obs_tag}] | "
                    f"Step {self.step_count:03d} (t={self.frame.time_s:4.1f}s)"
                )
            )
        else:
            self.status_lbl.configure(
                text=(
                    f"Seed {self.seed} | Step {self.step_count:03d} (t={self.frame.time_s:4.1f}s) | "
                    f"RF: {comms_tag} | Compute: {self.last_compute_ms:4.2f}ms"
                )
            )
        self.bench_metrics_lbl.configure(
            text=(
                f"📊 Independent Grader [Seed={self.seed}, Scen={self.scenario_var.get()}, Pol={self.session.config.effective_policy_label.upper()}, RF={comms_tag}]:  "
                f"Viable Seeds: {bm['viable_planted']}/{bm['viable_target']} ({100.0*bm['viable_planted']/bm['viable_target']:.0f}%)  |  "
                f"Wasted Drops: {bm['wasted_drops']}  |  "
                f"Recoveries: {len(self.sim.completed_recoveries)}  |  "
                f"Site RMSE: {bm['discrete_rmse']:.3f}  |  "
                f"2D Field RMSE: {bm['cont_rmse']:.3f}  |  "
                f"D-0010 Comm: {bm['actual_wire_bytes']} B vs {bm['stream_10hz_bytes']} B 10Hz ({bm['saved_pct']:+.1f}% Saved)  |  "
                f"Planted: {planted_sites or 'none'}"
            )
        )
        # Update Agent D Rescue Orchestrator & B Physical Recovery Banner
        orch_decs = self.session.orchestrator_decisions
        if orch_decs:
            parts: list[str] = []
            for vid, odec in orch_decs.items():
                tools_str = "->".join(tc.tool_name for tc in odec.tool_trace) if odec.tool_trace else "deterministic"
                gate_str = "✗VETO" if odec.safety_veto_applied else "✓GATE"
                mode_str = "FALLBACK:DeterministicReasoner" if odec.fallback_used else "LIVE_LLM:GeminiReAct"
                parts.append(
                    f"[{vid.upper()}] {odec.verdict.value.upper()} [{mode_str}] (helper={odec.helper_robot_id or 'none'}, "
                    f"tech={odec.chosen_technique.value}, {gate_str}, tools={tools_str})"
                )
            if self.frame.recovery_feedback:
                rf = self.frame.recovery_feedback[-1]
                parts.append(
                    f" | B-RECOVERY: latch={rf.latched} "
                    f"pull={rf.victim_displacement_m:.2f}m F={rf.peak_constraint_force_n:.1f}N extracted={rf.extracted}"
                )
            elif self.sim.completed_recoveries:
                last_rec = self.sim.completed_recoveries[-1]
                parts.append(
                    f" | ✓ EXTRACTED @ t={last_rec.observed_at_s:.1f}s ({last_rec.victim_displacement_m:.2f}m, "
                    f"F={last_rec.peak_constraint_force_n:.1f}N) -> Resumed Fieldwork"
                )
            self.rescue_banner_lbl.configure(
                text="  ".join(parts),
                fg=(
                    "#4ade80"
                    if self.sim.completed_recoveries
                    else ("#fbbf24" if any(d.verdict == RescueVerdict.RESCUE_NOW for d in orch_decs.values()) else "#f87171")
                ),
            )
        elif self.sim.completed_recoveries:
            last_rec = self.sim.completed_recoveries[-1]
            self.rescue_banner_lbl.configure(
                text=(
                    f"✓ Physical Recovery Complete at t={last_rec.observed_at_s:.1f}s: "
                    f"{last_rec.helper_robot_id.upper()} extracted {last_rec.target_robot_id.upper()} "
                    f"({last_rec.victim_displacement_m:.2f}m displacement, peak F={last_rec.peak_constraint_force_n:.1f}N, "
                    f"tech={last_rec.technique}) -> Resumed autonomous fieldwork"
                ),
                fg="#4ade80",
            )
        else:
            self.rescue_banner_lbl.configure(
                text=f"Rescue Orchestrator ({self.rescue_policy_var.get()}) & Physical Recovery (B): Nominal Fieldwork (0 stranded)",
                fg="#a7f3d0",
            )

        active_rids = [r.robot_id for r in self.frame.robots]
        if hasattr(self, "fleet_summary_lbl") and self.fleet_summary_lbl is not None:
            self.fleet_summary_lbl.configure(
                text=f"Active Fleet: {len(active_rids)} Rovers ({', '.join(r.upper() for r in active_rids)}) | Map Scale: {self.map_scale:.1f}x"
            )
        plan_parts = [f"● {self._format_robot_plan_sentence(rid)[2]}" for rid in active_rids]
        self.robot_plans_lbl.configure(text="   |   ".join(plan_parts))
        self._render_3d_view()
        self._render_2d_belief_map()
        self._update_inspector_and_logs()

    def _append_3d_hud_geoms(self, scn: Any) -> None:
        """Append 3D mud ruts, wheel-sinkage collars, belief spheres, SLAM cells, and UWB gossip beams."""
        identity_mat = np.eye(3, dtype=np.float64).flatten()
        planted_sites = {
            d.target_site_id for d in self.sim.completed_drops if d.contact_observed_at_s is not None
        }
        envelope = SoilSafetyEnvelope()

        def site_rgba(est: Any, planted: bool) -> list[float]:
            if planted:
                return [0.15, 0.95, 0.30, 0.92]
            if not est.complete:
                return [0.95, 0.70, 0.15, 0.60]
            if envelope.eligible(est):
                return [0.15, 0.85, 0.98, 0.88]
            return [0.95, 0.20, 0.18, 0.88]

        def add_sphere(pos: list[float], radius: float, rgba: list[float]) -> None:
            if scn.ngeom >= scn.maxgeom:
                return
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(
                g,
                mujoco.mjtGeom.mjGEOM_SPHERE,
                np.array([radius, 0.0, 0.0], dtype=np.float64),
                np.array(pos, dtype=np.float64),
                identity_mat,
                np.array(rgba, dtype=np.float32),
            )
            scn.ngeom += 1

        def add_cylinder(pos: list[float], radius: float, half_h: float, rgba: list[float]) -> None:
            if scn.ngeom >= scn.maxgeom:
                return
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(
                g,
                mujoco.mjtGeom.mjGEOM_CYLINDER,
                np.array([radius, half_h, 0.0], dtype=np.float64),
                np.array(pos, dtype=np.float64),
                identity_mat,
                np.array(rgba, dtype=np.float32),
            )
            scn.ngeom += 1

        def add_box(pos: list[float], half_sizes: list[float], rgba: list[float]) -> None:
            if scn.ngeom >= scn.maxgeom:
                return
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(
                g,
                mujoco.mjtGeom.mjGEOM_BOX,
                np.array(half_sizes, dtype=np.float64),
                np.array(pos, dtype=np.float64),
                identity_mat,
                np.array(rgba, dtype=np.float32),
            )
            scn.ngeom += 1

        def add_line(p1: list[float], p2: list[float], width: float, rgba: list[float]) -> None:
            if scn.ngeom >= scn.maxgeom:
                return
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(
                g,
                mujoco.mjtGeom.mjGEOM_CAPSULE,
                np.zeros(3, dtype=np.float64),
                np.zeros(3, dtype=np.float64),
                identity_mat,
                np.array(rgba, dtype=np.float32),
            )
            mujoco.mjv_connector(
                g,
                mujoco.mjtGeom.mjGEOM_CAPSULE,
                width,
                np.array(p1, dtype=np.float64),
                np.array(p2, dtype=np.float64),
            )
            scn.ngeom += 1

        # 1. Render 3D sunken wheel-mud rut tracks left behind by the rovers
        for _rid, rx, ry, rz, z_sink in self.sim.mud_rut_history[-60:]:
            rut_alpha = min(0.85, 0.30 + 10.0 * z_sink)
            rut_w = min(0.055, 0.025 + 0.55 * z_sink)
            add_box(
                [rx, ry - 0.11, max(0.002, rz * 0.5)],
                [0.038, rut_w, 0.004 + 0.25 * z_sink],
                [0.18, 0.12, 0.07, rut_alpha],
            )
            add_box(
                [rx, ry + 0.11, max(0.002, rz * 0.5)],
                [0.038, rut_w, 0.004 + 0.25 * z_sink],
                [0.18, 0.12, 0.07, rut_alpha],
            )

        # 2. Render 3D mud-sinkage collars / splash rings around each rover's wheels
        obs_by_id = {r.robot_id: r for r in self.frame.robots}
        for rid, r_obs in obs_by_id.items():
            sink = self.sim.latest_terramechanics.get(rid)
            if sink is None:
                continue
            z_s = sink.sinkage_z_m
            collar_rgba = (
                [0.95, 0.18, 0.15, 0.82]
                if (sink.bellied_out or r_obs.is_bogged)
                else ([0.85, 0.48, 0.12, 0.68] if z_s > 0.024 else [0.32, 0.23, 0.14, 0.52])
            )
            for dx_w in (-0.11, 0.11):
                for dy_w in (-0.12, 0.12):
                    wx = r_obs.pose.x_m + dx_w * cos(r_obs.pose.yaw_rad) - dy_w * sin(r_obs.pose.yaw_rad)
                    wy = r_obs.pose.y_m + dx_w * sin(r_obs.pose.yaw_rad) + dy_w * cos(r_obs.pose.yaw_rad)
                    add_cylinder(
                        [wx, wy, max(0.006, 0.45 * z_s)],
                        0.048 + 0.4 * z_s,
                        max(0.005, 0.45 * z_s),
                        collar_rgba,
                    )

        if self.presentation_mode:
            # Real physical robots stay in the MuJoCo scene. Highlight only
            # observed bogging, chosen helpers, and locally discovered hazards.
            for robot in self.frame.robots:
                if robot.is_bogged:
                    add_sphere([robot.pose.x_m, robot.pose.y_m, 0.29], 0.065,
                               [1.0, 0.12, 0.10, 0.93])
                if robot.robot_id == self.presentation_cue.helper_id:
                    add_sphere([robot.pose.x_m, robot.pose.y_m, 0.28], 0.055,
                               [1.0, 0.72, 0.12, 0.90])
            seen_hazards: set[tuple[int, int]] = set()
            for node in self.runtime.nodes.values():
                for key, cell in node.local_map.slam.cells.items():
                    if key in seen_hazards:
                        continue
                    seen_hazards.add(key)
                    add_box(
                        [cell.x_m, cell.y_m, 0.025], [0.075, 0.075, 0.015],
                        [1.0, 0.20, 0.12, 0.62] if cell.kind == "obstacle"
                        else [0.75, 0.20, 0.92, 0.55]
                    )
            for track in self.rescue_approach_tracks.values():
                for (x1, y1), (x2, y2) in zip(track[-70:-1], track[-69:]):
                    if hypot(x2 - x1, y2 - y1) > 0.004:
                        add_line([x1, y1, 0.06], [x2, y2, 0.06],
                                 0.007, [1.0, 0.67, 0.10, 0.92])
            return

        # 3. Render split dual-rover 3D belief spheres above each patch
        planter_node = self.runtime.nodes.get("planter")
        scout_node = self.runtime.nodes.get("scout")
        for site_id, site in self.sim.sites.items():
            if not site.permitted:
                continue
            cx, cy = site.center.x_m, site.center.y_m
            is_planted = site_id in planted_sites
            if scout_node is not None:
                add_sphere([cx, cy, 0.17], 0.036, site_rgba(scout_node.local_map.estimate(site_id), is_planted))
            if planter_node is not None:
                p_rgba = site_rgba(planter_node.local_map.estimate(site_id), is_planted)
                add_sphere([cx, cy, 0.26], 0.046, p_rgba)
                add_line([cx, cy, 0.01], [cx, cy, 0.22], 0.005, p_rgba)

        # 4. Render 3D SLAM hazard cells
        seen_slam: set[tuple[int, int]] = set()
        for node in self.runtime.nodes.values():
            for key, cell in node.local_map.slam.cells.items():
                if key in seen_slam:
                    continue
                seen_slam.add(key)
                if cell.kind == "obstacle":
                    add_box([cell.x_m, cell.y_m, 0.04], [0.06, 0.06, 0.04], [1.0, 0.30, 0.10, 0.55])
                else:
                    add_box([cell.x_m, cell.y_m, 0.015], [0.07, 0.07, 0.012], [0.75, 0.15, 0.85, 0.45])

        # 5. Render UWB / Wi-Fi HaLow gossip link beams across active rover pairs
        seen_pairs: set[tuple[str, str]] = set()
        for pkt in (() if self.presentation_mode else self.runtime.gossip_history[-15:]):
            if abs(self.frame.time_s - pkt.sent_at_s) <= 0.6:
                pair = (min(pkt.sender_id, pkt.receiver_id), max(pkt.sender_id, pkt.receiver_id))
                if pair not in seen_pairs and pair[0] in obs_by_id and pair[1] in obs_by_id:
                    seen_pairs.add(pair)
                    p1 = obs_by_id[pair[0]].pose
                    p2 = obs_by_id[pair[1]].pose
                    add_line(
                        [p1.x_m, p1.y_m, 0.19],
                        [p2.x_m, p2.y_m, 0.16],
                        0.011,
                        [0.10, 0.98, 0.92, 0.92],
                    )

        # 6. Render LiDAR rays, distress beacons & rescue tow vectors
        for intent in self.decision.intents:
            if intent.robot_id not in obs_by_id:
                continue
            r = obs_by_id[intent.robot_id]
            rx, ry, ryaw = r.pose.x_m, r.pose.y_m, r.pose.yaw_rad
            if r.is_bogged:
                add_sphere([rx, ry, 0.24], 0.042, [1.0, 0.12, 0.12, 0.92])
            beam_len = min(r.front_range_m if r.front_range_m is not None else 0.75, 0.75)
            bx = rx + beam_len * cos(ryaw)
            by = ry + beam_len * sin(ryaw)
            beam_rgba = (
                [1.0, 0.15, 0.15, 0.85]
                if (r.front_range_m is not None and r.front_range_m <= 0.40)
                else [0.25, 0.95, 0.55, 0.45]
            )
            add_line([rx, ry, 0.08], [bx, by, 0.08], 0.004, beam_rgba)
            if intent.mode == TaskMode.RESCUE and intent.target_robot_id in obs_by_id:
                peer_r = obs_by_id[intent.target_robot_id]
                add_line(
                    [rx, ry, 0.14],
                    [peer_r.pose.x_m, peer_r.pose.y_m, 0.14],
                    0.009,
                    [1.0, 0.45, 0.10, 0.92],
                )
            elif intent.target_site_id and intent.target_site_id in self.sim.sites:
                tsite = self.sim.sites[intent.target_site_id]
                line_rgba = (
                    [0.98, 0.84, 0.15, 0.82]
                    if intent.mode == TaskMode.PLANT
                    else _robot_rgba_color(intent.robot_id)
                )
                add_line([rx, ry, 0.11], [tsite.center.x_m, tsite.center.y_m, 0.18], 0.005, line_rgba)

    def _render_3d_view(self) -> None:
        self._upload_hfield_if_needed()
        if self.presentation_mode:
            self._position_presentation_camera()
        cam_choice = "isometric" if self.presentation_mode else self.camera_var.get()
        if cam_choice == "wheel_closeup":
            sel = self.selected_robot_var.get()
            try:
                r_xyz = self.sim.data.sensor(f"{sel}_pos").data
                self.free_cam.lookat[:] = [float(r_xyz[0]), float(r_xyz[1]), 0.03]
            except Exception:
                pass
            self.renderer.update_scene(self.sim.data, camera=self.free_cam)
        elif cam_choice == "isometric":
            self.renderer.update_scene(self.sim.data, camera=self.free_cam)
        else:
            try:
                self.renderer.update_scene(self.sim.data, camera=cam_choice)
            except Exception:
                self.renderer.update_scene(self.sim.data, camera=self.free_cam)

        self._append_3d_hud_geoms(self.renderer.scene)
        rgb = self.renderer.render()
        ppm_bytes = _rgb_array_to_tk_ppm(rgb)
        self._tk_3d_img = tk.PhotoImage(data=ppm_bytes)
        self.canvas_3d.delete("all")
        self.canvas_3d.create_image(0, 0, anchor=tk.NW, image=self._tk_3d_img)

        if self.presentation_mode:
            self.canvas_3d.create_rectangle(
                8, 8, min(self.RENDER_W - 8, 570), 60,
                fill="#09131f", outline="#233d54"
            )
            self.canvas_3d.create_text(
                18, 23, anchor="w",
                text=f"PHYSICAL SIMULATION  ·  {self.frame.time_s:04.1f} s",
                fill="#cbd5e1", font=("Segoe UI", 10, "bold")
            )
            self.canvas_3d.create_text(
                18, 45, anchor="w", text=self.presentation_cue.label,
                fill="#fbbf24", font=("Segoe UI", 13, "bold")
            )
            return

        # Overlay live 3D sinkage & slip telemetry badge directly on the 3D view
        sel = self.selected_robot_var.get()
        obs_by_id = {r.robot_id: r for r in self.frame.robots}
        r_obs = obs_by_id.get(sel)
        sink = self.sim.latest_terramechanics.get(sel)
        pt = self.sim.latest_terrain_points.get(sel)
        slip = self.sim.latest_slip_ratios.get(sel, 0.08)
        if sink and pt:
            is_stuck = bool(sink.bellied_out or (r_obs and r_obs.is_bogged))
            badge_col = "#ef4444" if is_stuck else ("#f59e0b" if sink.sinkage_z_m > 0.025 else "#38bdf8")
            self.canvas_3d.create_rectangle(8, 8, self.RENDER_W - 8, 32, fill="#09131f", outline="#1f3247")
            self.canvas_3d.create_text(
                14,
                20,
                anchor="w",
                text=(
                    f"3D Physics [{sel.upper()}]: Terrain z={pt.elevation_m*1000:+.1f}mm | "
                    f"Mud Sinkage z={sink.sinkage_z_m*1000:.1f}mm (Belly Clr={sink.belly_clearance_m*1000:+.1f}mm) | "
                    f"Wheel Slip={slip*100:.0f}% | {'STRANDED IN MUD!' if is_stuck else 'DRIVING'}"
                ),
                fill=badge_col,
                font=("Consolas", 9, "bold"),
            )

    def _world_to_canvas(self, x_m: float, y_m: float) -> tuple[float, float]:
        x_min, x_max = self.truth_grid["x_bounds"]
        y_min, y_max = self.truth_grid["y_bounds"]
        pad = 24.0
        w = self.MAP_W - 2 * pad
        h = self.MAP_H - 2 * pad
        cx = pad + ((x_m - x_min) / max(1e-6, x_max - x_min)) * w
        cy = self.MAP_H - (pad + ((y_m - y_min) / max(1e-6, y_max - y_min)) * h)
        return cx, cy

    def _canvas_to_world(self, cx: float, cy: float) -> tuple[float, float]:
        x_min, x_max = self.truth_grid["x_bounds"]
        y_min, y_max = self.truth_grid["y_bounds"]
        pad = 24.0
        w = self.MAP_W - 2 * pad
        h = self.MAP_H - 2 * pad
        x_m = x_min + ((cx - pad) / max(1e-6, w)) * (x_max - x_min)
        y_m = y_min + (((self.MAP_H - cy) - pad) / max(1e-6, h)) * (y_max - y_min)
        return x_m, y_m

    def _sample_robot_or_fused_grid(self, view_mode: str) -> dict[str, Any]:
        x_b, y_b = self._field_bounds()
        if view_mode in self.runtime.nodes:
            return sample_continuous_belief_grid(
                self.runtime.nodes[view_mode].local_map,
                nx=self.GRID_NX,
                ny=self.GRID_NY,
                x_bounds=x_b,
                y_bounds=y_b,
            )
        grids = [
            sample_continuous_belief_grid(
                node.local_map, nx=self.GRID_NX, ny=self.GRID_NY, x_bounds=x_b, y_bounds=y_b
            )
            for node in self.runtime.nodes.values()
        ]
        if not grids:
            return self.truth_grid["layers"]
        if len(grids) == 1:
            return grids[0]
        weights = [1.0 / np.maximum(0.05, np.asarray(g["uncertainty"], dtype=np.float64)) for g in grids]
        w_sum = np.sum(weights, axis=0)
        fused: dict[str, Any] = {}
        for k in ("moisture", "firmness", "planting_fitness", "traversability_risk"):
            fused[k] = np.sum([ np.asarray(g[k], dtype=np.float64) * w for g, w in zip(grids, weights) ], axis=0) / w_sum
        fused["uncertainty"] = np.min([np.asarray(g["uncertainty"], dtype=np.float64) for g in grids], axis=0)
        return fused

    def _render_2d_belief_map(self) -> None:
        c = self.canvas_2d
        c.delete("all")

        view_mode = "fused" if self.presentation_mode else self.belief_view_var.get()
        layer = "traversability_risk" if self.presentation_mode else self.layer_var.get()
        layer_title, vmin, vmax, palette = LAYER_META.get(
            layer, ("Field Layer", 0.0, 1.0, "moisture")
        )

        x_b, y_b = self._field_bounds()
        truth_layers = self.truth_grid["layers"]

        if view_mode == "ground_truth":
            src_layer = "mud_depth_m" if layer in ("uncertainty", "traversability_risk") else layer
            grid_2d = truth_layers.get(src_layer, truth_layers["moisture"])
            if src_layer == "mud_depth_m":
                vmin, vmax, palette = 0.02, 0.25, "risk"
        elif view_mode == "divergence":
            grids = [
                sample_continuous_belief_grid(
                    node.local_map, nx=self.GRID_NX, ny=self.GRID_NY, x_bounds=x_b, y_bounds=y_b
                )
                for node in self.runtime.nodes.values()
            ]
            l_key = layer if (grids and layer in grids[0]) else "moisture"
            if len(grids) >= 2:
                arrs = [np.asarray(g[l_key], dtype=np.float64) for g in grids]
                grid_2d = np.max(arrs, axis=0) - np.min(arrs, axis=0)
            else:
                grid_2d = np.zeros_like(np.asarray(truth_layers["moisture"], dtype=np.float64))
            vmin, vmax, palette = 0.0, 0.42, "risk"
        else:
            active_grid = self._sample_robot_or_fused_grid(view_mode)
            if layer == "elevation_m":
                u_arr = np.asarray(active_grid.get("uncertainty", 0.5), dtype=np.float64)
                conf = np.clip((0.50 - u_arr) / 0.35, 0.0, 1.0)
                grid_2d = conf * np.asarray(truth_layers["elevation_m"], dtype=np.float64)
            else:
                grid_2d = active_grid.get(layer, active_grid["moisture"])

        pad = 24
        plot_w = max(64, int(self.MAP_W - 2 * pad))
        plot_h = max(64, int(self.MAP_H - 2 * pad))

        # 1. Draw the entire continuous 2D heatmap as a SINGLE vectorized RGB PPM PhotoImage
        ppm_2d = _grid_to_rgb_ppm(grid_2d, vmin, vmax, palette, plot_w, plot_h)
        self._tk_2d_img = tk.PhotoImage(data=ppm_2d)
        c.create_image(pad, pad, anchor=tk.NW, image=self._tk_2d_img)

        # 2. Draw SLAM hazard cells discovered by the inspected robot(s)
        nodes_to_show = (
            [self.runtime.nodes[view_mode]]
            if view_mode in self.runtime.nodes
            else list(self.runtime.nodes.values())
        )
        for node in nodes_to_show:
            for cell in node.local_map.slam.cells.values():
                sx, sy = self._world_to_canvas(cell.x_m, cell.y_m)
                col = "#ef4444" if cell.kind == "obstacle" else "#c084fc"
                radius = 10 if self.presentation_mode else 6
                c.create_rectangle(sx - radius, sy - radius, sx + radius, sy + radius,
                                   outline=col, width=3 if self.presentation_mode else 2)

        # 3. Draw injected physical obstacle if active
        if self.sim.obstacle_injected:
            ox, oy = self._world_to_canvas(self.sim.obstacle_xy[0], self.sim.obstacle_xy[1])
            c.create_rectangle(ox - 11, oy - 11, ox + 11, oy + 11, fill="#dc2626", outline="#fef08a", width=2)
            c.create_text(ox, oy - 16, text="OBSTACLE", fill="#fef08a", font=("Consolas", 8, "bold"))

        # 4. Draw the 6 terrace sites & selected robot's local site estimate status (Undiscovered vs Camera Discovered vs Probed)
        planted_sites = {
            d.target_site_id for d in self.sim.completed_drops if d.contact_observed_at_s is not None
        }
        envelope = SoilSafetyEnvelope()
        inspected_rid = self.selected_robot_var.get()
        inspected_node = self.runtime.nodes.get(
            inspected_rid, next(iter(self.runtime.nodes.values()))
        )
        probed_set = set(getattr(inspected_node.local_map, "probed_sites", set()))
        disc_set = set(getattr(inspected_node.local_map, "discovered_sites", set())) | self.visually_discovered_sites.get(
            inspected_rid, set()
        )
        if view_mode == "fused":
            for nd in self.runtime.nodes.values():
                probed_set |= set(getattr(nd.local_map, "probed_sites", set()))
                disc_set |= set(getattr(nd.local_map, "discovered_sites", set()))
            for vs in self.visually_discovered_sites.values():
                disc_set |= vs

        for sid, site in self.sim.sites.items():
            sx, sy = self._world_to_canvas(site.center.x_m, site.center.y_m)
            if not site.permitted:
                c.create_oval(sx - 22, sy - 22, sx + 22, sy + 22, outline="#4ade80", width=2, dash=(3, 2))
                c.create_text(sx, sy, text=f"{sid}\n(PROTECTED)", fill="#86efac", font=("Segoe UI", 8, "bold"))
            else:
                est = inspected_node.local_map.estimate(sid)
                is_planted = sid in planted_sites
                is_probed = (sid in probed_set) or est.complete
                is_vis = sid in disc_set
                dash_pat: tuple[int, ...] | None = None
                if is_planted:
                    ring_col = "#22c55e"
                    tag_txt = f"{sid} [PLANTED]"
                elif is_probed and est.complete:
                    if envelope.eligible(est):
                        ring_col = "#38bdf8"
                        tag_txt = f"{sid} [VIABLE]"
                    else:
                        ring_col = "#f87171"
                        tag_txt = f"{sid} [MUD REJECT]"
                elif is_vis:
                    ring_col = "#38bdf8"
                    tag_txt = f"{sid} [VIS]"
                    dash_pat = (4, 2)
                else:
                    ring_col = "#fbbf24"
                    tag_txt = f"{sid} [?]"
                    dash_pat = (2, 3)
                if dash_pat is not None:
                    c.create_oval(sx - 18, sy - 18, sx + 18, sy + 18, outline=ring_col, width=2, dash=dash_pat)
                else:
                    c.create_oval(sx - 18, sy - 18, sx + 18, sy + 18, outline=ring_col, width=2)
                c.create_text(sx, sy - 24, text=tag_txt, fill="#f8fafc", font=("Consolas", 8, "bold"))

        # 5. Draw UWB / Wi-Fi HaLow wireless links between active rovers with recent gossip
        obs_by_id = {r.robot_id: r for r in self.frame.robots}
        seen_pairs_2d: set[tuple[str, str]] = set()
        for pkt in (() if self.presentation_mode else self.runtime.gossip_history[-15:]):
            if abs(self.frame.time_s - pkt.sent_at_s) <= 0.6:
                pair = (min(pkt.sender_id, pkt.receiver_id), max(pkt.sender_id, pkt.receiver_id))
                if pair not in seen_pairs_2d and pair[0] in obs_by_id and pair[1] in obs_by_id:
                    seen_pairs_2d.add(pair)
                    p1x, p1y = self._world_to_canvas(obs_by_id[pair[0]].pose.x_m, obs_by_id[pair[0]].pose.y_m)
                    p2x, p2y = self._world_to_canvas(obs_by_id[pair[1]].pose.x_m, obs_by_id[pair[1]].pose.y_m)
                    c.create_line(p1x, p1y, p2x, p2y, fill="#22d3ee", width=2, dash=(5, 3))
                    c.create_text(
                        0.5 * (p1x + p2x),
                        0.5 * (p1y + p2y) - 8,
                        text="UWB+HaLow GOSSIP",
                        fill="#fef08a",
                        font=("Consolas", 7, "bold"),
                    )

        # Orange: observed helper approach since assignment. No route or
        # navigation waypoints are inferred from the task destination.
        if self.presentation_mode:
            for track in self.rescue_approach_tracks.values():
                coords: list[float] = []
                for x_m, y_m in track[-160:]:
                    cx, cy = self._world_to_canvas(x_m, y_m)
                    coords.extend([cx, cy])
                if len(coords) >= 4:
                    c.create_line(*coords, fill="#fbbf24", width=4)

        # 6. Draw Rover Trajectories, Forward Camera FOV Cones & Clickable Rover Icons
        intents_by_rid = {i.robot_id: i for i in self.decision.intents}
        for rid, r_obs in obs_by_id.items():
            rover_col = _robot_hex_color(rid)
            pts = self.trajectories.get(rid, [])
            if len(pts) >= 2:
                flat_coords: list[float] = []
                for tx, ty in (pts[-160:] if self.presentation_mode else pts[-55:]):
                    cx, cy = self._world_to_canvas(tx, ty)
                    flat_coords.extend([cx, cy])
                c.create_line(
                    *flat_coords,
                    fill=rover_col,
                    width=2,
                )

            rx, ry = self._world_to_canvas(r_obs.pose.x_m, r_obs.pose.y_m)
            is_selected = rid == self.selected_robot_var.get()

            # Forward optical camera FOV cone wedge (62 deg, 0.95m range)
            cone_pts: list[float] = [rx, ry]
            for frac in (-1.0, -0.5, 0.0, 0.5, 1.0):
                ang = r_obs.pose.yaw_rad + frac * CAMERA_HALF_FOV_RAD
                wx = r_obs.pose.x_m + CAMERA_RANGE_M * cos(ang)
                wy = r_obs.pose.y_m + CAMERA_RANGE_M * sin(ang)
                cx_w, cy_w = self._world_to_canvas(wx, wy)
                cone_pts.extend([cx_w, cy_w])
            if not self.presentation_mode:
                c.create_polygon(
                    *cone_pts,
                    fill="",
                    outline="#fef08a" if is_selected else rover_col,
                    width=1,
                    dash=(3, 3),
                )

            if is_selected and not self.presentation_mode:
                c.create_oval(
                    rx - 20, ry - 20, rx + 20, ry + 20, outline="#fef08a", width=3
                )

            status_tag, fill_col, plan_sentence = self._format_robot_plan_sentence(rid)
            if self.presentation_mode and (
                r_obs.is_bogged or rid == self.presentation_cue.victim_id
            ):
                c.create_oval(rx - 24, ry - 24, rx + 24, ry + 24,
                             outline="#ef4444", width=4)
            if self.presentation_mode and rid == self.presentation_cue.helper_id:
                c.create_oval(rx - 23, ry - 23, rx + 23, ry + 23,
                             outline="#fbbf24", width=4)
            c.create_oval(rx - 12, ry - 12, rx + 12, ry + 12, fill=fill_col, outline="#ffffff", width=2)
            hx = rx + 18 * cos(r_obs.pose.yaw_rad)
            hy = ry - 18 * sin(r_obs.pose.yaw_rad)
            c.create_line(rx, ry, hx, hy, fill="#fef08a", width=2)

            intent = intents_by_rid.get(rid)
            if intent and intent.mode == TaskMode.RESCUE and intent.target_robot_id in obs_by_id:
                peer_obs = obs_by_id[intent.target_robot_id]
                tx_c, ty_c = self._world_to_canvas(peer_obs.pose.x_m, peer_obs.pose.y_m)
                c.create_line(rx, ry, tx_c, ty_c, fill="#f97316", width=3, dash=(6, 2))
            elif intent and intent.target_site_id and intent.target_site_id in self.sim.sites:
                tsite = self.sim.sites[intent.target_site_id]
                tx_c, ty_c = self._world_to_canvas(tsite.center.x_m, tsite.center.y_m)
                c.create_line(rx, ry, tx_c, ty_c, fill=fill_col, width=2, dash=(4, 3))

            if self.presentation_mode:
                role = (
                    "STRANDED" if r_obs.is_bogged else
                    "HELPER" if rid == self.presentation_cue.helper_id else
                    "ROVER"
                )
                c.create_text(rx, ry + 24, text=f"{rid.upper()} · {role}",
                              fill="#ffffff", font=("Segoe UI", 10, "bold"))
            else:
                c.create_text(
                    rx, ry + 22,
                    text=f"{rid.upper()} [{status_tag}]",
                    fill="#fef08a" if is_selected else "#f8fafc",
                    font=("Segoe UI", 9, "bold"),
                )
                c.create_text(
                    rx, ry + 35,
                    text=plan_sentence,
                    fill="#cbd5e1",
                    font=("Consolas", 8),
                )

        c.create_rectangle(6, 4, self.MAP_W - 6, 23, fill="#0b131e", outline="#1f3247")
        c.create_text(
            12,
            13,
            anchor="w",
            text=(
                "FUSED SENSOR ESTIMATE   ·   solid = observed track   ·   dashed = target, NOT a route"
                if self.presentation_mode else
                f"View: {view_mode.upper()}  |  Layer: {layer_title} [{vmin:.2f} .. {vmax:.2f}]  (Click any of the {len(obs_by_id)} rovers on map to inspect)"
            ),
            fill="#38bdf8",
            font=("Consolas", 8, "bold"),
        )

    def _update_inspector_and_logs(self) -> None:
        sel_rid = self.selected_robot_var.get()
        node = self.runtime.nodes.get(sel_rid, next(iter(self.runtime.nodes.values())))
        obs_by_id = {r.robot_id: r for r in self.frame.robots}
        r_obs = obs_by_id.get(sel_rid)
        intents_by_id = {i.robot_id: i for i in self.decision.intents}
        r_intent = intents_by_id.get(sel_rid)
        pt = self.sim.latest_terrain_points.get(sel_rid)
        sink = self.sim.latest_terramechanics.get(sel_rid)
        slip_r = self.sim.latest_slip_ratios.get(sel_rid, 0.08)
        tel = self.runtime.latest_sensor_telemetry.get(sel_rid)

        self.inspector_title_lbl.configure(
            text=f"🤖 {sel_rid.upper()}'S WORLD MODEL & TELEMETRY"
        )

        lines: list[str] = []
        if r_obs is not None:
            r_rng = f"{r_obs.front_range_m:.2f}m" if r_obs.front_range_m is not None else "NULL"
            lines.append(
                f"Pose     : ({r_obs.pose.x_m:.2f}m, {r_obs.pose.y_m:.2f}m, yaw={r_obs.pose.yaw_rad:.2f}r)"
            )
            lines.append(
                f"3D Tilt  : Pitch={r_obs.pitch_rad*57.3:+.1f}° | Roll={r_obs.roll_rad*57.3:+.1f}°"
            )
            lines.append(
                f"Seeds    : {r_obs.seeds_remaining}  | Front LiDAR: {r_rng} | Bogged={r_obs.is_bogged}"
            )
        if r_intent is not None:
            lines.append(
                f"Task     : {r_intent.mode.value.upper()} -> {r_intent.target_site_id or r_intent.target_robot_id or 'none'}"
            )
            lines.append(f"Reason   : {r_intent.reason}")

        # Agent D Rescue Orchestrator & B Physical Recovery section
        lines.append(f"\n--- AGENT D ORCHESTRATOR ({self.rescue_policy_var.get().upper()}) ---")
        lines.append("Available : [RESCUE_NOW, INVESTIGATE_PERIMETER, DEFER_RESCUE, ABANDON_ESCALATE]")
        assessments = self.session.rescue_assessments
        fault_ev_map = self.session.fault_evidence
        if self.session.orchestrator_decisions:
            for vid, odec in self.session.orchestrator_decisions.items():
                mode_label = (
                    "DETERMINISTIC FALLBACK (Offline Calibrated Reasoner)"
                    if odec.fallback_used
                    else "LIVE LLM (Gemini ReAct)"
                )
                lines.append(f"• Mode    : {mode_label}")
                asm = assessments.get(vid)
                if asm is not None:
                    lines.append(
                        f"  Assess  : benefit={asm.rescue_benefit:.2f} opp_cost={asm.opportunity_cost:.2f} "
                        f"risk={asm.hazard_risk:.2f} net={asm.net_rescue_value:+.2f}"
                    )
                fev = fault_ev_map.get(vid)
                if fev is not None:
                    lines.append(
                        f"  Evidence: stalled_ticks={fev.stalled_ticks} "
                        f"cmd_v={fev.commanded_linear_m_s:.2f}m/s "
                        f"obs_v={(fev.observed_speed_m_s or 0.0):.3f}m/s "
                        f"slip={fev.slip_ratio:.2f}"
                    )
                lines.append(
                    f"  Selected: {odec.verdict.value.upper()} (Stranded={vid.upper()})"
                )
                lines.append(
                    f"  Helper  : {odec.helper_robot_id or 'none'} | Tech: {odec.chosen_technique.value}"
                )
                lines.append(
                    f"  GateVeto: {odec.safety_veto_applied}"
                )
                if odec.tool_trace:
                    lines.append(
                        f"  Tools   : {' -> '.join(tc.tool_name for tc in odec.tool_trace)}"
                    )
                lines.append(f"  Rationale: {odec.rationale}")
        else:
            lines.append("• Active Incidents: 0 (nominal fieldwork)")
        if self.frame.recovery_feedback:
            rf = self.frame.recovery_feedback[-1]
            phase_str = getattr(rf.phase, "value", str(rf.phase))
            lines.append(
                f"• Live B-Feedback: phase={phase_str} latch={rf.latched} "
                f"pull={rf.victim_displacement_m:.2f}m F={rf.peak_constraint_force_n:.1f}N extracted={rf.extracted}"
            )
        if self.sim.completed_recoveries:
            last_rec = self.sim.completed_recoveries[-1]
            lines.append(
                f"• Completed Recovery (t={last_rec.observed_at_s:.1f}s): {last_rec.helper_robot_id}->{last_rec.target_robot_id} "
                f"pull={last_rec.victim_displacement_m:.2f}m F={last_rec.peak_constraint_force_n:.1f}N"
            )

        if pt is not None and sink is not None:
            lines.append("\n--- 3D TERRAIN, SLIP & BEKKER SINKAGE ---")
            lines.append(
                f"MicroTopo: z={pt.elevation_m*1000:+.1f}mm | slope={pt.slope_magnitude*100:.1f}%"
            )
            lines.append(
                f"Soil     : M={pt.moisture:.2f} | F={pt.firmness:.2f} | Mud={pt.mud_depth_m*1000:.0f}mm"
            )
            lines.append(
                f"Rice Fit : Φ={pt.planting_fitness:.2f} (WDSR Suitability)"
            )
            lines.append(
                f"Sinkage z: {sink.sinkage_z_m*1000:.1f}mm (Belly clr: {sink.belly_clearance_m*1000:+.1f}mm)"
            )
            lines.append(
                f"WheelSlip: s={slip_r*100:.1f}% | Drag Rc={sink.compaction_resistance_n:.1f}N"
            )
            lines.append(
                f"Status   : {'⚠ BELLIED OUT (S1 STUCK)' if (sink.bellied_out or (r_obs and r_obs.is_bogged)) else '✓ TRAVERSABLE'}"
                f" (Risk={sink.immobilization_risk*100:.0f}%)"
            )

        if tel is not None:
            lines.append(f"UWB Peers: {dict(tel.uwb_peer_ranges_m)}")

        lines.append(f"\n--- {sel_rid.upper()}'S PRIVATE GAUSSIAN SITE BELIEFS ---")
        envelope = SoilSafetyEnvelope()
        permitted_sids = [sid for sid, st in sorted(self.sim.sites.items()) if st.permitted]
        probed_sids = set(getattr(node.local_map, "probed_sites", set()))
        disc_sids = set(getattr(node.local_map, "discovered_sites", set())) | self.visually_discovered_sites.get(
            sel_rid, set()
        )
        for sid in permitted_sids:
            est = node.local_map.estimate(sid)
            ch = node.local_map._beliefs[sid]
            mean_unc = sum(b.uncertainty for b in ch.values()) / max(1, len(ch))
            if (sid in probed_sids) or est.complete:
                status = "VIABLE ✓" if envelope.eligible(est) else "MUD REJECT ✗"
            elif sid in disc_sids:
                status = "CAMERA VIS"
            else:
                status = "UNEXPLORED"
            m_str = f"{est.moisture:.2f}" if est.moisture is not None else " ?  "
            f_str = f"{est.firmness:.2f}" if est.firmness is not None else " ?  "
            g_str = f"{est.stand_gap:.2f}" if est.stand_gap is not None else " ?  "
            lines.append(
                f"• {sid}: [{status:12s}] M={m_str} F={f_str} G={g_str} (σ={mean_unc:.2f})"
            )

        lines.append(f"\n--- HIDDEN GROUND TRUTH (SEED {self.seed}) ---")
        if self.randomized_spec is not None:
            sp = self.randomized_spec
            lines.append(
                f"• Spawns : P=({sp.planter_spawn.x_m:.2f}, {sp.planter_spawn.y_m:.2f}, yaw={sp.planter_spawn.yaw_rad:+.2f}) "
                f"S=({sp.scout_spawn.x_m:.2f}, {sp.scout_spawn.y_m:.2f}, yaw={sp.scout_spawn.yaw_rad:+.2f})"
            )
            lines.append(
                f"• Hazard : trap={sp.trap_site_id} ({sp.mud_severity}, robot={sp.trap_robot_id or 'none'}) | "
                f"obs={f'{sp.obstacle_xy} @ {sp.inject_obstacle_at_s:.1f}s' if sp.inject_obstacle_at_s is not None else 'none'}"
            )
        if not self.comms_applies:
            lines.append(
                f"• RF Comms: N/A for {self.session.config.effective_policy_label} (uses shared MeasuredFieldMap)"
            )
        for sid in permitted_sids:
            tr = self.hidden_truth.get(sid, {})
            tm = tr.get(SensorKind.MOISTURE, 0.5)
            tf = tr.get(SensorKind.FIRMNESS, 0.5)
            tg = tr.get(SensorKind.STAND_GAP, 0.5)
            ok = (
                tg >= envelope.min_gap
                and envelope.moisture_range[0] <= tm <= envelope.moisture_range[1]
                and envelope.firmness_range[0] <= tf <= envelope.firmness_range[1]
            )
            lines.append(
                f"• {sid}: [{'TRUE VIABLE' if ok else 'TRUE HAZARD':11s}] M={tm:.2f} F={tf:.2f} G={tg:.2f}"
            )

        lines.append(f"\n--- LOCAL 2D SLAM CELLS ({len(node.local_map.slam.cells)}) ---")
        for cell in list(node.local_map.slam.cells.values())[:4]:
            lines.append(
                f"• ({cell.x_m:.2f}, {cell.y_m:.2f}) {cell.kind}: L={cell.log_odds:.2f} by {cell.discovered_by}"
            )

        self.inspector_text.delete("1.0", tk.END)
        self.inspector_text.insert(tk.END, "\n".join(lines))

        card_lines: list[str] = []
        for rid in [r.robot_id for r in self.frame.robots]:
            s_st = self.sim.latest_terramechanics.get(rid)
            t_pt = self.sim.latest_terrain_points.get(rid)
            slp = self.sim.latest_slip_ratios.get(rid, 0.08)
            ro = obs_by_id.get(rid)
            if s_st and t_pt:
                card_lines.append(
                    f"{rid.upper():9s} (m={s_st.mass_kg:4.1f}kg, r={s_st.wheel_radius_m*1000:2.0f}mm) | "
                    f"Topo z={t_pt.elevation_m*1000:+5.1f}mm | Sink z={s_st.sinkage_z_m*1000:5.1f}mm | "
                    f"Clr={s_st.belly_clearance_m*1000:+5.1f}mm | Slip={slp*100:2.0f}% | "
                    f"Drag={s_st.compaction_resistance_n:4.1f}N | "
                    f"{'STRANDED (S1)!' if (s_st.bellied_out or (ro and ro.is_bogged)) else 'DRIVING'}"
                )
        self.terramech_cards_lbl.configure(text="\n".join(card_lines))

        self.gossip_text.delete("1.0", tk.END)
        if not self.runtime.gossip_history:
            self.gossip_text.insert(
                tk.END,
                "No peer-to-peer gossip packets exchanged yet (rovers exchange compact packets when new site evidence, SLAM hazards, or task bids change).",
            )
        else:
            g_lines: list[str] = []
            for pkt in reversed(self.runtime.gossip_history[-10:]):
                g_lines.append(
                    f"[t={pkt.sent_at_s:4.1f}s] {pkt.sender_id.upper()} -> {pkt.receiver_id.upper()} "
                    f"(UWB d={pkt.uwb_distance_m:.2f}m, AoA={pkt.uwb_aoa_rad:+.2f}r, {pkt.byte_size} B): "
                    + " ; ".join(pkt.dialogue_summary[:2])
                )
            self.gossip_text.insert(tk.END, "\n".join(g_lines))


def launch_live_dashboard(
    *,
    scenario: str = "golden_rescue",
    comms: str = "normal",
    policy: str = "cooperative",
    rescue_policy: str = "react_orchestrator",
    seed: int = 7,
    auto_play: bool = True,
    presentation: bool = False,
) -> None:
    """Launch the unified large side-by-side 3D MuJoCo + 2D Top-Down Belief & Benchmark desktop window."""
    root = tk.Tk()
    app = LiveSwarmDashboardApp(
        root,
        initial_scenario=scenario,
        initial_comms=comms,
        initial_policy=policy,
        initial_rescue_policy=rescue_policy,
        seed=seed,
        presentation_mode=presentation,
    )
    if auto_play:
        root.after(250, app.toggle_play)
    root.mainloop()
