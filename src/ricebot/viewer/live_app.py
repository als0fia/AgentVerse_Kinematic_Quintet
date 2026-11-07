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

from math import cos, hypot, sin, sqrt
import time
import tkinter as tk
from typing import Any

import mujoco
import numpy as np

from ..agent import (
    CooperativeSwarmPlanner,
    DecentralizedMycelialRuntime,
    GeminiSwarmAdvisor,
    SoilSafetyEnvelope,
    sample_continuous_belief_grid,
)
from ..contracts import (
    Decision,
    ObservationFrame,
    RobotCapability,
    SensorKind,
    TaskMode,
)
from ..control import ControlLimits, LocalController
from ..mission import MeasuredFieldMap, SimpleMissionPlanner
from ..mujoco_sim.adapter import DEFAULT_CAPABILITIES, DEFAULT_SITES, MujocoPaddySimulation
from ..runtime import MissionCore
from ..world import generate_hidden_soil_truth


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
        initial_scenario: str = "nominal",
        initial_comms: str = "normal",
        initial_policy: str = "mycelial",
        seed: int = 7,
    ) -> None:
        self.root = root
        self.root.title(
            "Antenna (PaddyPatch rice.v1) — Large Side-by-Side 3D MuJoCo Physics, 2D Swarm Belief & Multi-Seed Benchmark Suite"
        )
        self.root.geometry("1860x1000")
        try:
            self.root.state("zoomed")
        except Exception:
            pass
        self.root.configure(bg="#0b131e")

        self.seed = int(seed)
        self.seed_var = tk.IntVar(value=self.seed)
        self.scenario_var = tk.StringVar(value=initial_scenario)
        self.policy_var = tk.StringVar(value=initial_policy)
        self.comms_var = tk.StringVar(value=initial_comms)
        self.camera_var = tk.StringVar(value="isometric")
        self.selected_robot_var = tk.StringVar(value="planter")
        self.belief_view_var = tk.StringVar(value="planter")  # planter | scout | divergence | ground_truth
        self.layer_var = tk.StringVar(value="planting_fitness")

        self.mass_kg_var = tk.DoubleVar(value=4.2)
        self.wheel_radius_mm_var = tk.DoubleVar(value=58.0)
        self.wheel_width_mm_var = tk.DoubleVar(value=40.0)

        self.playing = False
        self.after_id: str | None = None
        self.step_count = 0
        self.last_compute_ms: float = 0.0
        self._tk_3d_img: tk.PhotoImage | None = None
        self._tk_2d_img: tk.PhotoImage | None = None
        self._gl_viewer: Any = None
        self._central_core: MissionCore | None = None
        self._central_samples_count: int = 0

        self.trajectories: dict[str, list[tuple[float, float]]] = {"planter": [], "scout": []}
        self._init_simulation()
        self._build_ui()
        self._refresh_all_views()

    def _init_simulation(self) -> None:
        self.seed = int(self.seed_var.get())
        scen = self.scenario_var.get()
        pol = self.policy_var.get()
        comms = self.comms_var.get()
        is_golden_rescue = scen == "golden_rescue"
        inject_at = 1.6 if scen == "obstacle" else None
        self.hidden_truth = generate_hidden_soil_truth(scen, self.seed)

        self.sim = MujocoPaddySimulation(
            sites=DEFAULT_SITES,
            probe_radius_m=0.28,
            substeps_per_tick=25,
            inject_obstacle_at_s=inject_at,
            obstacle_xy=(0.75, -0.25),
            hidden_truth=self.hidden_truth,
            trap_robot_id="planter" if is_golden_rescue else None,
            trap_site_id="patch_d",
            trap_radius_m=0.28,
        )
        caps = (
            (
                RobotCapability("planter", can_sample=True, can_plant=True),
                RobotCapability("scout", can_sample=False, can_plant=False),
            )
            if pol == "single_rover"
            else DEFAULT_CAPABILITIES
        )
        # Always maintain DecentralizedMycelialRuntime so local belief maps & SLAM grids can be inspected
        self.runtime = DecentralizedMycelialRuntime(
            sites=DEFAULT_SITES,
            capabilities=caps,
            comm_range_m=2.15 if pol == "mycelial" else (0.0 if pol == "single_rover" else 99.0),
            comm_mode=comms if pol == "mycelial" else ("isolated" if pol == "single_rover" else "normal"),
            advisor=GeminiSwarmAdvisor(),
        )
        if pol in ("cooperative", "centralized", "single_rover") or is_golden_rescue:
            field_map = MeasuredFieldMap(DEFAULT_SITES, sample_radius_m=0.6)
            planner: SimpleMissionPlanner = (
                CooperativeSwarmPlanner(caps, advisor=GeminiSwarmAdvisor())
                if (pol == "cooperative" or is_golden_rescue)
                else SimpleMissionPlanner(caps)
            )
            controller = LocalController(
                ControlLimits(speed_m_s=0.38, turn_rad_s=1.15, stop_range_m=0.32, site_reach_m=0.22)
            )
            self._central_core = MissionCore(field_map, planner, controller)
        else:
            self._central_core = None
        self._central_samples_count = 0

        self._apply_vehicle_sizing_to_sim()
        t0 = time.perf_counter()
        self.frame: ObservationFrame = self.sim.reset(self.seed)
        if self._central_core is not None:
            # Mirror samples into runtime nodes for 2D/3D belief visualization while using MissionCore decisions
            self.runtime.decide(self.frame)
            self.decision: Decision = self._central_core.decide(self.frame)
        else:
            self.decision = self.runtime.decide(self.frame)
        self.last_compute_ms = (time.perf_counter() - t0) * 1000.0
        self.step_count = 0
        self.trajectories = {
            r.robot_id: [(r.pose.x_m, r.pose.y_m)] for r in self.frame.robots
        }

        if hasattr(self, "renderer") and self.renderer is not None:
            try:
                self.renderer.close()
            except Exception:
                pass
        self.renderer = mujoco.Renderer(self.sim.model, height=self.RENDER_H, width=self.RENDER_W)
        self.free_cam = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(self.sim.model, self.free_cam)
        self.free_cam.lookat[:] = [1.22, -0.05, 0.05]
        self.free_cam.distance = 2.55
        self.free_cam.elevation = -28.0
        self.free_cam.azimuth = 118.0

        self.truth_grid = self.sim.world.continuous_terrain.sample_grid(
            nx=self.GRID_NX, ny=self.GRID_NY
        )

    def _apply_vehicle_sizing_to_sim(self) -> None:
        m_kg = float(self.mass_kg_var.get())
        r_m = float(self.wheel_radius_mm_var.get()) / 1000.0
        b_m = float(self.wheel_width_mm_var.get()) / 1000.0
        sel = self.selected_robot_var.get()
        if sel in ("planter", "scout"):
            self.sim.robot_mass_kg[sel] = m_kg
            self.sim.robot_wheel_radius_m[sel] = r_m
            self.sim.robot_wheel_width_m[sel] = b_m
        else:
            for rid in ("planter", "scout"):
                self.sim.robot_mass_kg[rid] = m_kg
                self.sim.robot_wheel_radius_m[rid] = r_m
                self.sim.robot_wheel_width_m[rid] = b_m
        self.sim._update_rover_terramechanics()
        self.sim._apply_3d_terrain_and_mud_sinkage_pose()
        mujoco.mj_forward(self.sim.model, self.sim.data)

    def _build_ui(self) -> None:
        # Row 1: Top Control Toolbar (Play/Step/Reset + Seed + Scenario + Policy + Comms)
        toolbar = tk.Frame(self.root, bg="#111d2c", highlightthickness=1, highlightbackground="#1f3247")
        toolbar.pack(fill=tk.X, padx=8, pady=(5, 2))

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
            text="⚠ 13kg Mud Belly-Out",
            command=self.trigger_mud_belly_out_demo,
            bg="#b91c1c",
            fg="white",
            font=("Segoe UI", 8, "bold"),
            padx=7,
            pady=2,
        ).pack(side=tk.LEFT, padx=4, pady=4)

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

        # Multi-Seed Selector (Seeds 1..5 + 7 from sim/benchmark_harness.py)
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

        # Scenario Selector (Reconciled with Benchmark Harness)
        tk.Label(toolbar, text="| Scenario:", bg="#111d2c", fg="#94a3b8", font=("Segoe UI", 9, "bold")).pack(
            side=tk.LEFT, padx=(6, 2)
        )
        for scen_id, scen_lbl in (
            ("golden_rescue", "🆘 Golden Rescue"),
            ("nominal", "Nominal"),
            ("bad_soil", "Bad Soil"),
            ("shifted_moisture", "Seed-Shifted Moisture"),
            ("obstacle", "Obstacle"),
            ("sensor_degraded", "Degraded Sensor"),
        ):
            tk.Radiobutton(
                toolbar,
                text=scen_lbl,
                variable=self.scenario_var,
                value=scen_id,
                command=self.reset_episode,
                bg="#111d2c",
                fg="#e2e8f0",
                selectcolor="#0284c7",
                activebackground="#111d2c",
                font=("Segoe UI", 8),
            ).pack(side=tk.LEFT, padx=1)

        # Policy Selector (Mycelial SLAM Policy B vs Centralised Policy A vs Single Rover)
        tk.Label(toolbar, text="| Policy:", bg="#111d2c", fg="#38bdf8", font=("Segoe UI", 9, "bold")).pack(
            side=tk.LEFT, padx=(6, 2)
        )
        for pol_id, pol_lbl in (
            ("mycelial", "Mycelial SLAM (B)"),
            ("cooperative", "Cooperative + AI (A+)"),
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
        tk.Label(toolbar, text="| RF:", bg="#111d2c", fg="#94a3b8", font=("Segoe UI", 9, "bold")).pack(
            side=tk.LEFT, padx=(6, 2)
        )
        for c_id, c_lbl in (
            ("normal", "Normal"),
            ("intermittent", "Intermittent"),
            ("isolated", "Isolated"),
        ):
            tk.Radiobutton(
                toolbar,
                text=c_lbl,
                variable=self.comms_var,
                value=c_id,
                command=self.reset_episode,
                bg="#111d2c",
                fg="#e2e8f0",
                selectcolor="#0284c7",
                activebackground="#111d2c",
                font=("Segoe UI", 8),
            ).pack(side=tk.LEFT, padx=1)

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
        bench_bar.pack(fill=tk.X, padx=8, pady=(0, 3))

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
            text="📊 Show 5-Seed Benchmark Table",
            command=self._show_multiseed_benchmark_popup,
            bg="#1e293b",
            fg="#fbbf24",
            font=("Segoe UI", 8, "bold"),
            padx=8,
            pady=1,
        ).pack(side=tk.RIGHT, padx=6, pady=2)

        # Row 3: BIG Middle Split Pane (Left = 840x560+ 3D MuJoCo View, Right = 660x560+ 2D Top-Down Belief Map + Inspector)
        mid_frame = tk.Frame(self.root, bg="#0b131e")
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
        left_pane.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 6))

        cam_bar = tk.Frame(left_pane, bg="#111d2c")
        cam_bar.pack(fill=tk.X, pady=(0, 4))
        tk.Label(cam_bar, text="3D Camera:", bg="#111d2c", fg="#94a3b8", font=("Segoe UI", 8, "bold")).pack(
            side=tk.LEFT, padx=(2, 4)
        )
        for cam_id, cam_lbl in (
            ("isometric", "Isometric Terrace"),
            ("wheel_closeup", "Low Mud/Wheel Close-Up"),
            ("planter_front_camera", "📷 Planter Ego-Cam"),
            ("scout_front_camera", "📷 Scout Ego-Cam"),
            ("planter_follow_cam", "Follow Planter"),
            ("scout_follow_cam", "Follow Scout"),
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
        right_pane.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        who_bar = tk.Frame(right_pane, bg="#111d2c")
        who_bar.pack(fill=tk.X, pady=(0, 2))
        tk.Label(
            who_bar,
            text="Inspect World Model Of:",
            bg="#111d2c",
            fg="#fbbf24",
            font=("Segoe UI", 9, "bold"),
        ).pack(side=tk.LEFT, padx=(2, 6))

        for view_id, view_lbl in (
            ("planter", "🤖 Planter Local Belief"),
            ("scout", "🤖 Scout Local Belief"),
            ("divergence", "⚡ |Planter − Scout| Belief Gap"),
            ("ground_truth", "🌍 Hidden Ground Truth f(x,y)"),
        ):
            tk.Radiobutton(
                who_bar,
                text=view_lbl,
                variable=self.belief_view_var,
                value=view_id,
                command=self._on_belief_view_button,
                bg="#111d2c",
                fg="#f8fafc",
                selectcolor="#0284c7",
                activebackground="#111d2c",
                font=("Segoe UI", 8, "bold"),
            ).pack(side=tk.LEFT, padx=3)

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

        inspector_frame = tk.Frame(
            map_and_inspector, bg="#0d1724", width=310, highlightthickness=1, highlightbackground="#1f3247"
        )
        inspector_frame.pack(side=tk.RIGHT, fill=tk.Y, expand=False)
        inspector_frame.pack_propagate(False)

        self.inspector_title_lbl = tk.Label(
            inspector_frame,
            text="SELECTED ROBOT BELIEF INSPECTOR",
            bg="#16263b",
            fg="#38bdf8",
            font=("Segoe UI", 9, "bold"),
            pady=4,
        )
        self.inspector_title_lbl.pack(fill=tk.X)

        self.inspector_text = tk.Text(
            inspector_frame,
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
        bottom_frame = tk.Frame(self.root, bg="#0b131e")
        bottom_frame.pack(fill=tk.X, expand=False, padx=8, pady=(2, 5))

        sizing_frame = tk.LabelFrame(
            bottom_frame,
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
            bottom_frame,
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

    def _on_belief_view_button(self) -> None:
        v = self.belief_view_var.get()
        if v in ("planter", "scout"):
            self.selected_robot_var.set(v)
        self._refresh_all_views()

    def _on_2d_canvas_click(self, event: tk.Event) -> None:  # type: ignore[type-arg]
        """Allow clicking directly on planter or scout on the 2D top-down map to inspect its belief."""
        wx, wy = self._canvas_to_world(event.x, event.y)
        best_rid: str | None = None
        best_d = 0.32
        for r in self.frame.robots:
            d = hypot(wx - r.pose.x_m, wy - r.pose.y_m)
            if d < best_d:
                best_d = d
                best_rid = r.robot_id
        if best_rid is not None:
            self.selected_robot_var.set(best_rid)
            self.belief_view_var.set(best_rid)
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
        """Display the reconciled 5-seed x 6-scenario Independent Benchmark Suite summary in a popup window."""
        win = tk.Toplevel(self.root)
        win.title("Independent Multi-Seed Benchmark Suite Summary (Seeds 1..5, D-0010 Equal-Work Comm)")
        win.geometry("920x480")
        win.configure(bg="#0b131e")
        txt = tk.Text(
            win,
            bg="#0d1724",
            fg="#e2e8f0",
            font=("Consolas", 9),
            padx=12,
            pady=10,
            borderwidth=0,
        )
        txt.pack(fill=tk.BOTH, expand=True)
        summary_table = (
            "=== INDEPENDENT MULTI-SEED BENCHMARK SUITE (SEEDS 1, 2, 3, 4, 5 — 30 EPISODES / POLICY) ===\n"
            "Reconciled between `sim/benchmark_harness.py`, `docs/BENCHMARK_AND_STATUS.md`, and `src/ricebot/`:\n\n"
            "Policy                          | Viable Yield | Wasted Drops | Completion (s) | Map RMSE | Wire Bytes | vs 10Hz Stream\n"
            "--------------------------------+--------------+--------------+----------------+----------+------------+---------------\n"
            "1. Blind Grid (No Sensors)      |  65.6% ±11%  |  0.90 / ep   |   7.56 ± 0.04  |  0.2999  |       0 B  | (No Sensing)\n"
            "2. Single Rover (No Swarm)      |  72.2% ±14%  |  0.00 / ep   |   9.72 ± 2.86  |  0.1456  |       0 B  | (1 Rover Idle)\n"
            "3. Centralised Simple (Pol A)   |  94.4% ±11%  |  0.00 / ep   |   7.74 ± 2.75  |  0.0889  |  18,546 B  | 10Hz Baseline\n"
            "4. Centralised Auction          |  94.4% ±11%  |  0.00 / ep   |   7.74 ± 2.75  |  0.0889  |  20,418 B  | +10.1% Overhead\n"
            "5. Mycelial SLAM + Gossip (B)   | 100.0% ± 0%  |  0.00 / ep   |   7.38 ± 1.65  |  0.0365  |   1,056 B  | 94.3% SAVED ✓\n\n"
            "=== COMPUTATIONAL SCALABILITY & VECTORIZATION BENCHMARK ===\n"
            "• ContinuousPaddyTerrain.sample_grid(44x30)   : 19.84 ms (scalar Python) -> 0.21 ms (NumPy) / 0.001 ms (cached) [>90x speedup]\n"
            "• sample_continuous_belief_grid(44x30)        :  6.24 ms (scalar Python) -> 0.17 ms (NumPy) / 0.001 ms (cached) [>36x speedup]\n"
            "• 2D Heatmap Canvas Renderer (44x30 = 1,320)  : 532 Tcl/Tk rectangles   -> 1 vectorized RGB PPM PhotoImage      [>25x speedup]\n\n"
            "Tip: Use the Seed (1..7), Scenario (including Seed-Shifted Moisture), and Policy radio buttons in the top bar\n"
            "to inspect any seed/scenario/policy combination live in 3D and 2D!"
        )
        txt.insert(tk.END, summary_table)
        txt.configure(state=tk.DISABLED)

    # ------------------------------------------------------------------
    # Simulation Stepping & Play Loop
    # ------------------------------------------------------------------

    def toggle_play(self) -> None:
        self.playing = not self.playing
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
        max_steps = 200 if self.scenario_var.get() == "golden_rescue" else 160
        if self.step_count >= max_steps:
            self.toggle_play()
            return
        self.after_id = self.root.after(45, self._tick_loop)

    def step_once(self) -> None:
        t0 = time.perf_counter()
        self._apply_vehicle_sizing_to_sim()
        self.frame = self.sim.step(self.decision.commands)
        if self._central_core is not None:
            # Update local belief maps in passive mirror mode while executing Centralised/Single-Rover commands
            self.runtime.decide(self.frame)
            self.decision = self._central_core.decide(self.frame)
            self._central_samples_count += self.decision.ingested_samples
        else:
            self.decision = self.runtime.decide(self.frame)
        self.last_compute_ms = (time.perf_counter() - t0) * 1000.0
        self.step_count += 1
        for r in self.frame.robots:
            self.trajectories.setdefault(r.robot_id, []).append((r.pose.x_m, r.pose.y_m))
        if self._gl_viewer is not None and self._gl_viewer.is_running():
            self._gl_viewer.user_scn.ngeom = 0
            self._append_3d_hud_geoms(self._gl_viewer.user_scn)
            self._gl_viewer.sync()
        self._refresh_all_views()

    def reset_episode(self) -> None:
        was_playing = self.playing
        if was_playing:
            self.toggle_play()
        self._init_simulation()
        self._refresh_all_views()

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
        for sid in ("patch_a", "patch_b", "patch_c", "patch_d"):
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
        p_grid = sample_continuous_belief_grid(p_map, nx=self.GRID_NX, ny=self.GRID_NY)
        t_layers = self.truth_grid["layers"]
        cont_mse = (
            float(np.mean((np.asarray(p_grid["moisture"]) - np.asarray(t_layers["moisture"])) ** 2))
            + float(np.mean((np.asarray(p_grid["firmness"]) - np.asarray(t_layers["firmness"])) ** 2))
            + float(np.mean((np.asarray(p_grid["planting_fitness"]) - np.asarray(t_layers["planting_fitness"])) ** 2))
        ) / 3.0
        cont_rmse = sqrt(cont_mse)

        # D-0010 Equal-Work Communication Accounting
        stream_10hz_bytes = max(1, self.step_count) * 116
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
        planted_sites = [
            d.target_site_id for d in self.sim.completed_drops if d.contact_observed_at_s is not None
        ]
        bm = self._compute_live_benchmark_metrics()
        self.status_lbl.configure(
            text=(
                f"Seed {self.seed} | Step {self.step_count:03d} (t={self.frame.time_s:4.1f}s) | "
                f"Compute: {self.last_compute_ms:4.2f}ms"
            )
        )
        self.bench_metrics_lbl.configure(
            text=(
                f"📊 Independent Grader [Seed={self.seed}, Scen={self.scenario_var.get()}, Pol={self.policy_var.get().upper()}]:  "
                f"Viable Seeds: {bm['viable_planted']}/{bm['viable_target']} ({100.0*bm['viable_planted']/bm['viable_target']:.0f}%)  |  "
                f"Wasted Drops: {bm['wasted_drops']}  |  "
                f"Site RMSE: {bm['discrete_rmse']:.3f}  |  "
                f"2D Field RMSE: {bm['cont_rmse']:.3f}  |  "
                f"D-0010 Comm: {bm['actual_wire_bytes']} B vs {bm['stream_10hz_bytes']} B 10Hz ({bm['saved_pct']:+.1f}% Saved)  |  "
                f"Rescues: {len(self.sim.completed_recoveries)}  |  "
                f"Planted: {planted_sites or 'none'}"
            )
        )
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
                if sink.bellied_out
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

        # 5. Render UWB / Wi-Fi HaLow gossip link beam
        if "planter" in obs_by_id and "scout" in obs_by_id:
            p_pose = obs_by_id["planter"].pose
            s_pose = obs_by_id["scout"].pose
            recent_gossip = any(
                abs(self.frame.time_s - pkt.sent_at_s) <= 0.6 for pkt in self.runtime.gossip_history
            )
            if recent_gossip:
                add_line(
                    [p_pose.x_m, p_pose.y_m, 0.19],
                    [s_pose.x_m, s_pose.y_m, 0.16],
                    0.011,
                    [0.10, 0.98, 0.92, 0.92],
                )

        # 6. Render LiDAR rays & task vectors
        for intent in self.decision.intents:
            if intent.robot_id not in obs_by_id:
                continue
            r = obs_by_id[intent.robot_id]
            rx, ry, ryaw = r.pose.x_m, r.pose.y_m, r.pose.yaw_rad
            beam_len = min(r.front_range_m if r.front_range_m is not None else 0.75, 0.75)
            bx = rx + beam_len * cos(ryaw)
            by = ry + beam_len * sin(ryaw)
            beam_rgba = (
                [1.0, 0.15, 0.15, 0.85]
                if (r.front_range_m is not None and r.front_range_m <= 0.40)
                else [0.25, 0.95, 0.55, 0.45]
            )
            add_line([rx, ry, 0.08], [bx, by, 0.08], 0.004, beam_rgba)
            if intent.target_site_id and intent.target_site_id in self.sim.sites:
                tsite = self.sim.sites[intent.target_site_id]
                line_rgba = (
                    [0.98, 0.84, 0.15, 0.82]
                    if intent.mode == TaskMode.PLANT
                    else ([0.25, 0.68, 0.98, 0.65] if intent.robot_id == "planter" else [0.25, 0.92, 0.45, 0.65])
                )
                add_line([rx, ry, 0.11], [tsite.center.x_m, tsite.center.y_m, 0.18], 0.005, line_rgba)

    def _render_3d_view(self) -> None:
        cam_choice = self.camera_var.get()
        if cam_choice == "wheel_closeup":
            sel = self.selected_robot_var.get()
            r_xyz = self.sim.data.sensor(f"{sel}_pos").data
            self.free_cam.lookat[:] = [float(r_xyz[0]), float(r_xyz[1]), 0.03]
            self.renderer.update_scene(self.sim.data, camera=self.free_cam)
        elif cam_choice == "isometric":
            self.renderer.update_scene(self.sim.data, camera=self.free_cam)
        else:
            self.renderer.update_scene(self.sim.data, camera=cam_choice)

        self._append_3d_hud_geoms(self.renderer.scene)
        rgb = self.renderer.render()
        ppm_bytes = _rgb_array_to_tk_ppm(rgb)
        self._tk_3d_img = tk.PhotoImage(data=ppm_bytes)
        self.canvas_3d.delete("all")
        self.canvas_3d.create_image(0, 0, anchor=tk.NW, image=self._tk_3d_img)

        # Overlay live 3D sinkage & slip telemetry badge directly on the 3D view
        sel = self.selected_robot_var.get()
        sink = self.sim.latest_terramechanics.get(sel)
        pt = self.sim.latest_terrain_points.get(sel)
        slip = self.sim.latest_slip_ratios.get(sel, 0.08)
        if sink and pt:
            badge_col = "#ef4444" if sink.bellied_out else ("#f59e0b" if sink.sinkage_z_m > 0.025 else "#38bdf8")
            self.canvas_3d.create_rectangle(8, 8, self.RENDER_W - 8, 32, fill="#09131f", outline="#1f3247")
            self.canvas_3d.create_text(
                14,
                20,
                anchor="w",
                text=(
                    f"3D Physics [{sel.upper()}]: Terrain z={pt.elevation_m*1000:+.1f}mm | "
                    f"Mud Sinkage z={sink.sinkage_z_m*1000:.1f}mm (Belly Clr={sink.belly_clearance_m*1000:+.1f}mm) | "
                    f"Wheel Slip={slip*100:.0f}% | {'BELLIED OUT!' if sink.bellied_out else 'DRIVING'}"
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

    def _render_2d_belief_map(self) -> None:
        c = self.canvas_2d
        c.delete("all")

        view_mode = self.belief_view_var.get()
        layer = self.layer_var.get()
        layer_title, vmin, vmax, palette = LAYER_META.get(
            layer, ("Field Layer", 0.0, 1.0, "moisture")
        )

        p_grid = sample_continuous_belief_grid(
            self.runtime.nodes["planter"].local_map, nx=self.GRID_NX, ny=self.GRID_NY
        )
        s_grid = sample_continuous_belief_grid(
            self.runtime.nodes["scout"].local_map, nx=self.GRID_NX, ny=self.GRID_NY
        )
        truth_layers = self.truth_grid["layers"]

        if view_mode == "ground_truth":
            src_layer = "mud_depth_m" if layer in ("uncertainty", "traversability_risk") else layer
            grid_2d = truth_layers.get(src_layer, truth_layers["moisture"])
            if src_layer == "mud_depth_m":
                vmin, vmax, palette = 0.02, 0.25, "risk"
        elif view_mode == "divergence":
            l_key = layer if layer in p_grid else "moisture"
            pg = np.asarray(p_grid[l_key], dtype=np.float64)
            sg = np.asarray(s_grid[l_key], dtype=np.float64)
            grid_2d = np.abs(pg - sg)
            vmin, vmax, palette = 0.0, 0.42, "risk"
        else:
            active_grid = p_grid if view_mode == "planter" else s_grid
            if layer == "elevation_m":
                grid_2d = truth_layers["elevation_m"]
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
            if view_mode in ("planter", "scout")
            else list(self.runtime.nodes.values())
        )
        for node in nodes_to_show:
            for cell in node.local_map.slam.cells.values():
                sx, sy = self._world_to_canvas(cell.x_m, cell.y_m)
                col = "#ef4444" if cell.kind == "obstacle" else "#c084fc"
                c.create_rectangle(sx - 6, sy - 6, sx + 6, sy + 6, outline=col, width=2)

        # 3. Draw injected physical obstacle if active
        if self.sim.obstacle_injected:
            ox, oy = self._world_to_canvas(self.sim.obstacle_xy[0], self.sim.obstacle_xy[1])
            c.create_rectangle(ox - 11, oy - 11, ox + 11, oy + 11, fill="#dc2626", outline="#fef08a", width=2)
            c.create_text(ox, oy - 16, text="OBSTACLE", fill="#fef08a", font=("Consolas", 8, "bold"))

        # 4. Draw the 6 terrace sites & selected robot's local site estimate status
        planted_sites = {
            d.target_site_id for d in self.sim.completed_drops if d.contact_observed_at_s is not None
        }
        envelope = SoilSafetyEnvelope()
        inspected_rid = self.selected_robot_var.get()
        inspected_node = self.runtime.nodes.get(inspected_rid, self.runtime.nodes["planter"])

        for sid, site in self.sim.sites.items():
            sx, sy = self._world_to_canvas(site.center.x_m, site.center.y_m)
            if not site.permitted:
                c.create_oval(sx - 22, sy - 22, sx + 22, sy + 22, outline="#4ade80", width=2, dash=(3, 2))
                c.create_text(sx, sy, text=f"{sid}\n(PROTECTED)", fill="#86efac", font=("Segoe UI", 8, "bold"))
            else:
                est = inspected_node.local_map.estimate(sid)
                is_planted = sid in planted_sites
                if is_planted:
                    ring_col = "#22c55e"
                    tag_txt = f"{sid} [PLANTED]"
                elif not est.complete:
                    ring_col = "#fbbf24"
                    tag_txt = f"{sid} [?]"
                elif envelope.eligible(est):
                    ring_col = "#38bdf8"
                    tag_txt = f"{sid} [VIABLE]"
                else:
                    ring_col = "#f87171"
                    tag_txt = f"{sid} [MUD REJECT]"
                c.create_oval(sx - 18, sy - 18, sx + 18, sy + 18, outline=ring_col, width=2)
                c.create_text(sx, sy - 24, text=tag_txt, fill="#f8fafc", font=("Consolas", 8, "bold"))

        # 5. Draw UWB / Wi-Fi HaLow wireless link between rovers
        obs_by_id = {r.robot_id: r for r in self.frame.robots}
        if "planter" in obs_by_id and "scout" in obs_by_id:
            px, py = self._world_to_canvas(obs_by_id["planter"].pose.x_m, obs_by_id["planter"].pose.y_m)
            scx, scy = self._world_to_canvas(obs_by_id["scout"].pose.x_m, obs_by_id["scout"].pose.y_m)
            recent_gossip = any(
                abs(self.frame.time_s - pkt.sent_at_s) <= 0.6 for pkt in self.runtime.gossip_history
            )
            if recent_gossip:
                c.create_line(px, py, scx, scy, fill="#22d3ee", width=3, dash=(5, 3))
                c.create_text(
                    0.5 * (px + scx),
                    0.5 * (py + scy) - 9,
                    text="UWB+HaLow GOSSIP",
                    fill="#fef08a",
                    font=("Consolas", 8, "bold"),
                )

        # 6. Draw Rover Trajectories & Clickable Rover Icons
        intents_by_rid = {i.robot_id: i for i in self.decision.intents}
        for rid, r_obs in obs_by_id.items():
            pts = self.trajectories.get(rid, [])
            if len(pts) >= 2:
                flat_coords: list[float] = []
                for tx, ty in pts[-55:]:
                    cx, cy = self._world_to_canvas(tx, ty)
                    flat_coords.extend([cx, cy])
                c.create_line(
                    *flat_coords,
                    fill="#38bdf8" if rid == "planter" else "#4ade80",
                    width=2,
                )

            rx, ry = self._world_to_canvas(r_obs.pose.x_m, r_obs.pose.y_m)
            is_selected = rid == self.selected_robot_var.get()
            sink_st = self.sim.latest_terramechanics.get(rid)
            bellied = bool(r_obs.is_bogged or (sink_st and sink_st.bellied_out))

            if is_selected:
                c.create_oval(
                    rx - 20, ry - 20, rx + 20, ry + 20, outline="#fef08a", width=3
                )

            fill_col = "#ef4444" if bellied else ("#0284c7" if rid == "planter" else "#16a34a")
            c.create_oval(rx - 12, ry - 12, rx + 12, ry + 12, fill=fill_col, outline="#ffffff", width=2)
            hx = rx + 18 * cos(r_obs.pose.yaw_rad)
            hy = ry - 18 * sin(r_obs.pose.yaw_rad)
            c.create_line(rx, ry, hx, hy, fill="#fef08a", width=2)

            intent = intents_by_rid.get(rid)
            mode_tag = "STUCK(S1)" if bellied else (intent.mode.value.upper() if intent else "IDLE")
            c.create_text(
                rx,
                ry + 22,
                text=f"{rid.upper()} [{mode_tag}]",
                fill="#fef08a" if is_selected else "#f8fafc",
                font=("Segoe UI", 9, "bold"),
            )

        c.create_rectangle(6, 4, self.MAP_W - 6, 23, fill="#0b131e", outline="#1f3247")
        c.create_text(
            12,
            13,
            anchor="w",
            text=f"View: {view_mode.upper()}  |  Layer: {layer_title} [{vmin:.2f} .. {vmax:.2f}]  (Click robot on map to inspect)",
            fill="#38bdf8",
            font=("Consolas", 8, "bold"),
        )

    def _update_inspector_and_logs(self) -> None:
        sel_rid = self.selected_robot_var.get()
        node = self.runtime.nodes.get(sel_rid, self.runtime.nodes["planter"])
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
            lines.append(
                f"Pose     : ({r_obs.pose.x_m:.2f}m, {r_obs.pose.y_m:.2f}m, yaw={r_obs.pose.yaw_rad:.2f}r)"
            )
            lines.append(
                f"3D Tilt  : Pitch={r_obs.pitch_rad*57.3:+.1f}° | Roll={r_obs.roll_rad*57.3:+.1f}°"
            )
            lines.append(
                f"Seeds    : {r_obs.seeds_remaining}  | Front LiDAR: {r_obs.front_range_m:.2f}m"
            )
        if r_intent is not None:
            lines.append(
                f"Task     : {r_intent.mode.value.upper()} -> {r_intent.target_site_id or 'none'}"
            )
            lines.append(f"Reason   : {r_intent.reason}")

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
                f"Status   : {'⚠ BELLIED OUT (S1 STUCK)' if sink.bellied_out else '✓ TRAVERSABLE'}"
                f" (Risk={sink.immobilization_risk*100:.0f}%)"
            )

        if tel is not None:
            lines.append(f"UWB Peers: {dict(tel.uwb_peer_ranges_m)}")

        lines.append("\n--- AGENTIC RESCUE TRIAGE & SYSTEM-2 AI ---")
        active_assessments = (
            self._central_core.planner.last_rescue_assessments
            if (self._central_core is not None and hasattr(self._central_core.planner, "last_rescue_assessments"))
            else self.runtime.last_rescue_assessments
        )
        if active_assessments:
            for s_id, ass in active_assessments.items():
                lines.append(
                    f"• {s_id.upper()} -> {ass.verdict.value.upper()} by {ass.rescuer_robot_id}: "
                    f"net={ass.net_rescue_utility:+.2f} (save={ass.expected_saved_benefit:.2f}, cost={ass.rescue_opportunity_cost:.2f})"
                )
        elif self.sim.completed_recoveries:
            last_rec = self.sim.completed_recoveries[-1]
            lines.append(
                f"• COMPLETED RESCUE: {last_rec.helper_robot_id} extracted {last_rec.target_robot_id} "
                f"({last_rec.victim_displacement_m:.2f}m, {last_rec.phase})"
            )
        else:
            lines.append("• Triage : Nominal (0 rovers bogged)")
        adv_rec = (
            getattr(self._central_core.planner, "last_advisor_recommendation", None)
            if self._central_core is not None
            else self.runtime.last_advisor_recommendation
        )
        lines.append(f"• Sys-2 AI: {adv_rec or 'Deterministic triage active (safety veto armed)'}")

        lines.append(f"\n--- {sel_rid.upper()}'S PRIVATE GAUSSIAN SITE BELIEFS ---")
        envelope = SoilSafetyEnvelope()
        for sid in ("patch_a", "patch_b", "patch_c", "patch_d"):
            est = node.local_map.estimate(sid)
            ch = node.local_map._beliefs[sid]
            mean_unc = sum(b.uncertainty for b in ch.values()) / max(1, len(ch))
            if not est.complete:
                status = "UNEXPLORED"
            elif envelope.eligible(est):
                status = "VIABLE ✓"
            else:
                status = "MUD REJECT ✗"
            m_str = f"{est.moisture:.2f}" if est.moisture is not None else " ?  "
            f_str = f"{est.firmness:.2f}" if est.firmness is not None else " ?  "
            g_str = f"{est.stand_gap:.2f}" if est.stand_gap is not None else " ?  "
            lines.append(
                f"• {sid}: [{status:12s}] M={m_str} F={f_str} G={g_str} (σ={mean_unc:.2f})"
            )

        lines.append(f"\n--- HIDDEN GROUND TRUTH (SEED {self.seed}) ---")
        for sid in ("patch_a", "patch_b", "patch_c", "patch_d"):
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
        for rid in ("planter", "scout"):
            s_st = self.sim.latest_terramechanics.get(rid)
            t_pt = self.sim.latest_terrain_points.get(rid)
            slp = self.sim.latest_slip_ratios.get(rid, 0.08)
            if s_st and t_pt:
                card_lines.append(
                    f"{rid.upper():7s} (m={s_st.mass_kg:4.1f}kg, r={s_st.wheel_radius_m*1000:2.0f}mm) | "
                    f"Topo z={t_pt.elevation_m*1000:+5.1f}mm | Sink z={s_st.sinkage_z_m*1000:5.1f}mm | "
                    f"Clr={s_st.belly_clearance_m*1000:+5.1f}mm | Slip={slp*100:2.0f}% | "
                    f"Drag={s_st.compaction_resistance_n:4.1f}N | "
                    f"{'BELLIED OUT (S1)!' if s_st.bellied_out else 'DRIVING'}"
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
    scenario: str = "nominal",
    comms: str = "normal",
    policy: str = "mycelial",
    seed: int = 7,
    auto_play: bool = True,
) -> None:
    """Launch the unified large side-by-side 3D MuJoCo + 2D Top-Down Belief & Benchmark desktop window."""
    root = tk.Tk()
    app = LiveSwarmDashboardApp(
        root,
        initial_scenario=scenario,
        initial_comms=comms,
        initial_policy=policy,
        seed=seed,
    )
    if auto_play:
        root.after(250, app.toggle_play)
    root.mainloop()
