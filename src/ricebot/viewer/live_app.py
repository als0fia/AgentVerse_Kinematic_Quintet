"""Native Side-by-Side 3D MuJoCo + 2D Top-Down Swarm Belief & Terramechanics Desktop Application.

Combines in a single multi-tab native desktop window (Tkinter + MuJoCo Renderer):
- **Tab 1 — Side-by-Side 3D MuJoCo Physics & Interactive 2D Top-Down Belief Heatmap Dashboard:**
  - **Left Pane:** Live 3D MuJoCo physics view (`mujoco.Renderer`) with 3D belief beacons, local SLAM
    occupancy/mud cells, and UWB/Wi-Fi HaLow gossip links, plus camera view selector and optional
    synced interactive `mujoco.viewer` OpenGL window.
  - **Right Pane:** Interactive 2D Top-Down Continuous Belief Heatmap where clicking directly on a
    robot (`planter` or `scout`) or selecting from the top bar switches the entire heatmap and
    inspector panel to show what that specific robot currently understands about the world
    (`Planter Local Belief`, `Scout Local Belief`, `|Planter - Scout| Divergence`, or `Hidden Ground Truth`),
    across 7 continuous spatial layers (`moisture`, `firmness`, `plant_density`, `stand_gap`,
    `uncertainty`, `traversability_risk`, `elevation_m`).
  - **Bottom Dock:** Live scenario/comms controls, real-time Bekker-Wong vehicle mass & wheel-size
    sliders that directly modulate 3D MuJoCo wheel traction and belly-out immobilization, per-robot
    sinkage & sensor cards, and the scrolling UWB/HaLow Mycelial Gossip log.
- **Tab 2 — Bekker-Wong Wheel-Mud Terramechanics & Vehicle Sizing Curve Explorer:**
  - Live interactive curves of wheel sinkage $z$ (mm), belly clearance margin $c_b - z$ (mm), and
    mud compaction drag $R_c$ (N) across vehicle masses ($1.5..16$ kg) and wheel radii.
- **Tab 3 — 6-Robot RiceSwarm Seasonal Lifecycle & Mud-Rescue Simulator (`src/riceswarm_ai/`):**
  - Embeds the teammates' 6-robot (`RS1..RS6`) condition-based farming lifecycle and Pareto mud-rescue
    simulator directly as an integrated tab.
"""
from __future__ import annotations

from math import cos, hypot, sin
import tkinter as tk
from tkinter import ttk
from typing import Any

import mujoco
import numpy as np

from ..agent import (
    DecentralizedMycelialRuntime,
    SoilSafetyEnvelope,
    sample_continuous_belief_grid,
)
from ..contracts import Decision, ObservationFrame, SensorKind, TaskMode
from ..mujoco_sim.adapter import DEFAULT_CAPABILITIES, DEFAULT_SITES, MujocoPaddySimulation
from ..robot import PLANTER_SPEC, SCOUT_SPEC, evaluate_bekker_terramechanics
from ..world import DEFAULT_HIDDEN_SOIL_TRUTH, UNEXPECTED_BAD_SOIL_TRUTH


def _rgb_array_to_tk_ppm(rgb: np.ndarray) -> bytes:
    """Convert an (H, W, 3) uint8 RGB NumPy array into in-memory binary PPM bytes for tk.PhotoImage."""
    h, w, _ = rgb.shape
    header = f"P6\n{w} {h}\n255\n".encode("ascii")
    return header + np.ascontiguousarray(rgb, dtype=np.uint8).tobytes()


def _colormap_hex(val: float, vmin: float, vmax: float, palette: str) -> str:
    t = max(0.0, min(1.0, (val - vmin) / max(1e-6, vmax - vmin)))
    if palette == "moisture":
        r = int(215 - 185 * t)
        g = int(180 - 75 * (t**1.1) + 30 * (1 - abs(2 * t - 1)))
        b = int(95 + 150 * (t**0.85))
    elif palette == "firmness":
        r = int(120 + 95 * (1 - t))
        g = int(65 + 150 * t)
        b = int(45 + 75 * t)
    elif palette == "density":
        r = int(145 - 110 * t)
        g = int(110 + 115 * t)
        b = int(65 - 20 * t)
    elif palette == "gap":
        r = int(35 + 210 * t)
        g = int(150 + 55 * t - 60 * (t**2))
        b = int(85 - 55 * t)
    elif palette == "uncertainty":
        r = int(20 + 225 * t)
        g = int(130 - 90 * t + 70 * (t**2))
        b = int(190 - 40 * t)
    elif palette == "elevation":
        r = int(50 + 175 * t)
        g = int(95 + 125 * t)
        b = int(85 + 60 * (1 - t))
    else:  # risk / divergence
        r = int(30 + 220 * (t**0.85))
        g = int(175 * (1 - t))
        b = int(95 * (1 - t) + 110 * (t**2))
    r = max(0, min(255, r))
    g = max(0, min(255, g))
    b = max(0, min(255, b))
    return f"#{r:02x}{g:02x}{b:02x}"


LAYER_META: dict[str, tuple[str, float, float, str]] = {
    "moisture": ("Soil Moisture M(x,y)", 0.25, 0.95, "moisture"),
    "firmness": ("Cone Firmness F(x,y)", 0.15, 0.90, "firmness"),
    "plant_density": ("Plant Density ρ(x,y)", 0.0, 1.0, "density"),
    "stand_gap": ("Stand Gap G(x,y)", 0.0, 1.0, "gap"),
    "uncertainty": ("Epistemic Uncertainty σ(x,y)", 0.0, 1.0, "uncertainty"),
    "traversability_risk": ("Mud / SLAM Risk R(x,y)", 0.0, 1.0, "risk"),
    "elevation_m": ("Elevation / Hilliness z(x,y) [m]", -0.02, 0.09, "elevation"),
}


class LiveSwarmDashboardApp:
    """Native multi-tab desktop window showing 3D MuJoCo simulation and 2D Top-Down Belief Dashboard side-by-side."""

    RENDER_W = 500
    RENDER_H = 360
    MAP_W = 540
    MAP_H = 360
    GRID_NX = 28
    GRID_NY = 19

    def __init__(
        self,
        root: tk.Tk,
        *,
        initial_scenario: str = "nominal",
        initial_comms: str = "normal",
        seed: int = 7,
    ) -> None:
        self.root = root
        self.root.title(
            "Antenna (PaddyPatch rice.v1) — Live 3D MuJoCo Simulator & Decentralised Top-Down Belief Dashboard"
        )
        self.root.geometry("1520x920")
        self.root.configure(bg="#0b131e")

        self.seed = seed
        self.scenario_var = tk.StringVar(value=initial_scenario)
        self.comms_var = tk.StringVar(value=initial_comms)
        self.camera_var = tk.StringVar(value="isometric")
        self.selected_robot_var = tk.StringVar(value="planter")
        self.belief_view_var = tk.StringVar(value="planter")  # planter | scout | divergence | ground_truth
        self.layer_var = tk.StringVar(value="moisture")

        self.mass_kg_var = tk.DoubleVar(value=4.2)
        self.wheel_radius_mm_var = tk.DoubleVar(value=58.0)
        self.wheel_width_mm_var = tk.DoubleVar(value=40.0)

        self.playing = False
        self.after_id: str | None = None
        self.step_count = 0
        self._tk_3d_img: tk.PhotoImage | None = None
        self._gl_viewer: Any = None

        self.trajectories: dict[str, list[tuple[float, float]]] = {"planter": [], "scout": []}
        self._init_simulation()
        self._build_ui()
        self._refresh_all_views()

    def _init_simulation(self) -> None:
        scen = self.scenario_var.get()
        comms = self.comms_var.get()
        inject_at = 1.6 if scen == "obstacle" else None
        truth = UNEXPECTED_BAD_SOIL_TRUTH if scen == "bad_soil" else DEFAULT_HIDDEN_SOIL_TRUTH

        self.sim = MujocoPaddySimulation(
            sites=DEFAULT_SITES,
            probe_radius_m=0.28,
            substeps_per_tick=25,
            inject_obstacle_at_s=inject_at,
            obstacle_xy=(0.75, -0.25),
            hidden_truth=truth,
        )
        self.runtime = DecentralizedMycelialRuntime(
            sites=DEFAULT_SITES,
            capabilities=DEFAULT_CAPABILITIES,
            comm_range_m=2.15,
            comm_mode=comms,
        )
        self._apply_vehicle_sizing_to_sim()
        self.frame: ObservationFrame = self.sim.reset(self.seed)
        self.decision: Decision = self.runtime.decide(self.frame)
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
        self.free_cam.lookat[:] = [1.25, -0.05, 0.06]
        self.free_cam.distance = 2.85
        self.free_cam.elevation = -33.0
        self.free_cam.azimuth = 115.0

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

    def _build_ui(self) -> None:
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except Exception:
            pass
        style.configure("TNotebook", background="#0b131e", borderwidth=0)
        style.configure(
            "TNotebook.Tab",
            background="#132030",
            foreground="#94a3b8",
            padding=(14, 7),
            font=("Segoe UI", 10, "bold"),
        )
        style.map(
            "TNotebook.Tab",
            background=[("selected", "#1e3a5f")],
            foreground=[("selected", "#38bdf8")],
        )

        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=8, pady=6)

        # Tab 1: Side-by-Side 3D MuJoCo + 2D Top-Down Belief Dashboard
        self.tab_main = tk.Frame(self.notebook, bg="#0b131e")
        self.notebook.add(
            self.tab_main,
            text="  1. Side-by-Side 3D MuJoCo Sim & 2D Top-Down Robot Belief Dashboard  ",
        )
        self._build_tab_main(self.tab_main)

        # Tab 2: Bekker-Wong Terramechanics & Vehicle Sizing Curve Explorer
        self.tab_bekker = tk.Frame(self.notebook, bg="#0b131e")
        self.notebook.add(
            self.tab_bekker,
            text="  2. Bekker-Wong Wheel-Mud Sinkage & Vehicle Sizing Lab  ",
        )
        self._build_tab_bekker(self.tab_bekker)

        # Tab 3: Combined 6-Robot RiceSwarm Lifecycle Simulator (from src/riceswarm_ai)
        self.tab_riceswarm = tk.Frame(self.notebook, bg="#1a202c")
        self.notebook.add(
            self.tab_riceswarm,
            text="  3. 6-Robot RiceSwarm Seasonal Lifecycle & Mud-Rescue Sim  ",
        )
        self._build_tab_riceswarm(self.tab_riceswarm)

    def _build_tab_main(self, parent: tk.Frame) -> None:
        # Top Control Toolbar
        toolbar = tk.Frame(parent, bg="#111d2c", highlightthickness=1, highlightbackground="#1f3247")
        toolbar.pack(fill=tk.X, padx=8, pady=(6, 4))

        self.play_btn = tk.Button(
            toolbar,
            text="▶ Play Live Sim",
            command=self.toggle_play,
            bg="#0284c7",
            fg="white",
            font=("Segoe UI", 9, "bold"),
            padx=10,
            pady=3,
        )
        self.play_btn.pack(side=tk.LEFT, padx=6, pady=5)

        tk.Button(
            toolbar,
            text="⏭ Step Tick (0.1s)",
            command=self.step_once,
            bg="#1e293b",
            fg="#e2e8f0",
            font=("Segoe UI", 9, "bold"),
            padx=8,
            pady=3,
        ).pack(side=tk.LEFT, padx=3, pady=5)

        tk.Button(
            toolbar,
            text="↺ Reset Episode",
            command=self.reset_episode,
            bg="#1e293b",
            fg="#e2e8f0",
            font=("Segoe UI", 9),
            padx=8,
            pady=3,
        ).pack(side=tk.LEFT, padx=3, pady=5)

        tk.Button(
            toolbar,
            text="⚠ Test Mud Belly-Out (12.5 kg)",
            command=self.trigger_mud_belly_out_demo,
            bg="#b91c1c",
            fg="white",
            font=("Segoe UI", 9, "bold"),
            padx=8,
            pady=3,
        ).pack(side=tk.LEFT, padx=6, pady=5)

        tk.Label(toolbar, text="Scenario:", bg="#111d2c", fg="#94a3b8", font=("Segoe UI", 9, "bold")).pack(
            side=tk.LEFT, padx=(10, 2)
        )
        for scen_id, scen_lbl in (
            ("nominal", "Nominal"),
            ("bad_soil", "Surprise Bad Soil"),
            ("obstacle", "Obstacle Surprise"),
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
            ).pack(side=tk.LEFT, padx=2)

        tk.Label(toolbar, text="| Comms:", bg="#111d2c", fg="#94a3b8", font=("Segoe UI", 9, "bold")).pack(
            side=tk.LEFT, padx=(8, 2)
        )
        for c_id, c_lbl in (
            ("normal", "Normal UWB/HaLow"),
            ("intermittent", "Intermittent Blackout"),
            ("isolated", "Isolated (No Radio)"),
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
            ).pack(side=tk.LEFT, padx=2)

        self.status_lbl = tk.Label(
            toolbar,
            text="t = 0.00s",
            bg="#111d2c",
            fg="#38bdf8",
            font=("Consolas", 9, "bold"),
        )
        self.status_lbl.pack(side=tk.RIGHT, padx=10)

        # Middle Split Pane: Left = 3D MuJoCo View, Right = 2D Top-Down Belief Heatmap
        mid_frame = tk.Frame(parent, bg="#0b131e")
        mid_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        # LEFT PANE: 3D MuJoCo View
        left_pane = tk.LabelFrame(
            mid_frame,
            text=" Live 3D MuJoCo Physics Simulation (Continuous Terrain + Paddy Ant Rovers + 3D Belief/UWB HUD) ",
            bg="#111d2c",
            fg="#38bdf8",
            font=("Segoe UI", 9, "bold"),
            padx=6,
            pady=4,
        )
        left_pane.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=(0, 6))

        cam_bar = tk.Frame(left_pane, bg="#111d2c")
        cam_bar.pack(fill=tk.X, pady=(0, 4))
        tk.Label(cam_bar, text="3D Camera:", bg="#111d2c", fg="#94a3b8", font=("Segoe UI", 8, "bold")).pack(
            side=tk.LEFT, padx=(2, 4)
        )
        for cam_id, cam_lbl in (
            ("isometric", "Isometric Terrace"),
            ("top_overview", "Top Overview"),
            ("planter_follow_cam", "Follow Planter"),
            ("scout_follow_cam", "Follow Scout"),
        ):
            tk.Radiobutton(
                cam_bar,
                text=cam_lbl,
                variable=self.camera_var,
                value=cam_id,
                command=self._render_3d_view,
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
        self.canvas_3d.pack()
        # Allow dragging on the 3D canvas to orbit the isometric camera
        self._drag_last_xy: tuple[int, int] | None = None
        self.canvas_3d.bind("<ButtonPress-1>", self._on_3d_mouse_down)
        self.canvas_3d.bind("<B1-Motion>", self._on_3d_mouse_drag)
        self.canvas_3d.bind("<MouseWheel>", self._on_3d_mouse_wheel)

        tk.Label(
            left_pane,
            text="Tip: Drag on 3D view to orbit camera, scroll wheel to zoom. Spheres above patches = Scout (lower) & Planter (upper) beliefs.",
            bg="#111d2c",
            fg="#94a3b8",
            font=("Segoe UI", 8),
        ).pack(anchor="w", pady=(3, 0))

        # RIGHT PANE: Interactive 2D Top-Down Belief Heatmap & Click-to-Select Robot
        right_pane = tk.LabelFrame(
            mid_frame,
            text=" Interactive 2D Top-Down Belief Heatmap (Click a Robot on Map or Select Below to Inspect Its World Model) ",
            bg="#111d2c",
            fg="#38bdf8",
            font=("Segoe UI", 9, "bold"),
            padx=6,
            pady=4,
        )
        right_pane.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Robot Belief View Selector Bar
        who_bar = tk.Frame(right_pane, bg="#111d2c")
        who_bar.pack(fill=tk.X, pady=(0, 3))
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
            ).pack(side=tk.LEFT, padx=4)

        # Continuous Heatmap Layer Bar
        layer_bar = tk.Frame(right_pane, bg="#111d2c")
        layer_bar.pack(fill=tk.X, pady=(0, 4))
        tk.Label(
            layer_bar,
            text="2D Field Layer:",
            bg="#111d2c",
            fg="#94a3b8",
            font=("Segoe UI", 8, "bold"),
        ).pack(side=tk.LEFT, padx=(2, 6))

        for l_id, l_short in (
            ("moisture", "Moisture M"),
            ("firmness", "Firmness F"),
            ("plant_density", "Plant Density ρ"),
            ("stand_gap", "Stand Gap G"),
            ("uncertainty", "Uncertainty σ"),
            ("traversability_risk", "Mud/SLAM Risk R"),
            ("elevation_m", "Elevation z"),
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
            ).pack(side=tk.LEFT, padx=2)

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
        self.canvas_2d.pack(side=tk.LEFT, padx=(0, 6))
        self.canvas_2d.bind("<Button-1>", self._on_2d_canvas_click)

        # Selected Robot World-Understanding Inspector Panel (to the right of 2D canvas)
        inspector_frame = tk.Frame(map_and_inspector, bg="#0d1724", highlightthickness=1, highlightbackground="#1f3247")
        inspector_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

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
            height=22,
            padx=6,
            pady=4,
            borderwidth=0,
        )
        self.inspector_text.pack(fill=tk.BOTH, expand=True)

        # BOTTOM DOCK: Bekker-Wong Sliders + Per-Robot Sinkage Cards + UWB Gossip Log
        bottom_frame = tk.Frame(parent, bg="#0b131e")
        bottom_frame.pack(fill=tk.BOTH, expand=False, padx=8, pady=(2, 6))

        # Left of bottom dock: Bekker-Wong Vehicle Sizing Sliders + Live Sinkage Cards
        sizing_frame = tk.LabelFrame(
            bottom_frame,
            text=" Live Bekker-Wong Wheel-Mud Terramechanics & Vehicle Sizing (Active in 3D MuJoCo Physics) ",
            bg="#111d2c",
            fg="#38bdf8",
            font=("Segoe UI", 9, "bold"),
            padx=8,
            pady=4,
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
            col.pack(side=tk.LEFT, padx=6)
            tk.Label(col, text=lbl, bg="#111d2c", fg="#cbd5e1", font=("Segoe UI", 8, "bold")).pack(anchor="w")
            tk.Scale(
                col,
                variable=var,
                from_=vmin,
                to=vmax,
                resolution=res,
                orient=tk.HORIZONTAL,
                length=145,
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
            padx=8,
            pady=5,
        )
        self.terramech_cards_lbl.pack(fill=tk.X, pady=(4, 0))

        # Right of bottom dock: Scrolling UWB / Wi-Fi HaLow Gossip & Consensus Log
        gossip_frame = tk.LabelFrame(
            bottom_frame,
            text=" Peer-to-Peer UWB / Wi-Fi HaLow Mycelial Gossip & Task Negotiation Log ",
            bg="#111d2c",
            fg="#38bdf8",
            font=("Segoe UI", 9, "bold"),
            padx=6,
            pady=4,
        )
        gossip_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.gossip_text = tk.Text(
            gossip_frame,
            height=9,
            bg="#070d14",
            fg="#cbd5e1",
            font=("Consolas", 8),
            wrap=tk.WORD,
            padx=6,
            pady=4,
            borderwidth=0,
        )
        self.gossip_text.pack(fill=tk.BOTH, expand=True)

    def _build_tab_bekker(self, parent: tk.Frame) -> None:
        header = tk.Frame(parent, bg="#111d2c", pady=6, padx=10)
        header.pack(fill=tk.X, padx=10, pady=8)
        tk.Label(
            header,
            text=(
                "Analytical Bekker-Wong Pressure-Sinkage z = (p / (kc/b + kφ))^(1/n) + Δz_slip  &  "
                "Janosi-Hanamoto Shear Traction across Vehicle Masses (1.5 kg to 16 kg)"
            ),
            bg="#111d2c",
            fg="#38bdf8",
            font=("Segoe UI", 10, "bold"),
        ).pack(side=tk.LEFT)

        self.bekker_canvas = tk.Canvas(
            parent,
            width=1420,
            height=720,
            bg="#070d14",
            highlightthickness=1,
            highlightbackground="#1f3247",
        )
        self.bekker_canvas.pack(padx=12, pady=6, fill=tk.BOTH, expand=True)

    def _build_tab_riceswarm(self, parent: tk.Frame) -> None:
        """Embed the teammates' 6-Robot RiceSwarm AI visualizer inside Tab 3."""
        try:
            from riceswarm_ai.visualizer import RiceSwarmVisualizer

            # Create an embedded wrapper so RiceSwarmVisualizer builds inside `parent`
            class _EmbeddedRootProxy:
                def __init__(self, frame: tk.Frame, real_root: tk.Tk) -> None:
                    self._frame = frame
                    self._real_root = real_root

                def title(self, _t: str) -> None:
                    pass

                def geometry(self, _g: str) -> None:
                    pass

                def configure(self, **kw: Any) -> None:
                    self._frame.configure(**kw)

                def after(self, ms: int, func: Any = None, *args: Any) -> str:
                    return self._real_root.after(ms, func, *args)

                def after_cancel(self, ident: str) -> None:
                    self._real_root.after_cancel(ident)

                def __getattr__(self, name: str) -> Any:
                    return getattr(self._frame, name)

            proxy = _EmbeddedRootProxy(parent, self.root)
            self._embedded_riceswarm = RiceSwarmVisualizer(proxy, introduce_mud=True)  # type: ignore[arg-type]
        except Exception as exc:
            tk.Label(
                parent,
                text=f"RiceSwarm 6-Robot Tab unavailable: {exc}",
                bg="#1a202c",
                fg="#f87171",
                font=("Consolas", 11),
            ).pack(padx=20, pady=20)

    # ------------------------------------------------------------------
    # Interactive Camera & Canvas Event Handlers
    # ------------------------------------------------------------------

    def _on_3d_mouse_down(self, event: tk.Event) -> None:  # type: ignore[type-arg]
        self._drag_last_xy = (event.x, event.y)

    def _on_3d_mouse_drag(self, event: tk.Event) -> None:  # type: ignore[type-arg]
        if self._drag_last_xy is None:
            return
        dx = event.x - self._drag_last_xy[0]
        dy = event.y - self._drag_last_xy[1]
        self._drag_last_xy = (event.x, event.y)
        self.camera_var.set("isometric")
        self.free_cam.azimuth = (self.free_cam.azimuth - dx * 0.45) % 360.0
        self.free_cam.elevation = max(-85.0, min(-8.0, self.free_cam.elevation - dy * 0.35))
        self._render_3d_view()

    def _on_3d_mouse_wheel(self, event: tk.Event) -> None:  # type: ignore[type-arg]
        self.camera_var.set("isometric")
        delta = -0.18 if event.delta > 0 else 0.18
        self.free_cam.distance = max(0.9, min(5.5, self.free_cam.distance + delta))
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
        best_d = 0.30
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
        """Set selected rover mass to 12.5 kg and wheel radius to 42 mm so it visibly bellies out in mud."""
        self.mass_kg_var.set(12.5)
        self.wheel_radius_mm_var.set(42.0)
        self.wheel_width_mm_var.set(28.0)
        self._apply_vehicle_sizing_to_sim()
        self._refresh_all_views()

    def _launch_optional_gl_viewer(self) -> None:
        try:
            import mujoco.viewer

            if self._gl_viewer is None or not self._gl_viewer.is_running():
                self._gl_viewer = mujoco.viewer.launch_passive(self.sim.model, self.sim.data)
                self._gl_viewer.cam.lookat[:] = [1.25, -0.05, 0.06]
                self._gl_viewer.cam.distance = 2.85
                self._gl_viewer.cam.elevation = -33.0
                self._gl_viewer.cam.azimuth = 115.0
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Simulation Stepping & Play Loop
    # ------------------------------------------------------------------

    def toggle_play(self) -> None:
        self.playing = not self.playing
        if self.playing:
            self.play_btn.configure(text="⏸ Pause Sim", bg="#d97706")
            self._tick_loop()
        else:
            self.play_btn.configure(text="▶ Play Live Sim", bg="#0284c7")
            if self.after_id is not None:
                self.root.after_cancel(self.after_id)
                self.after_id = None

    def _tick_loop(self) -> None:
        if not self.playing:
            return
        self.step_once()
        if self.step_count >= 160:
            self.toggle_play()
            return
        self.after_id = self.root.after(55, self._tick_loop)

    def step_once(self) -> None:
        self._apply_vehicle_sizing_to_sim()
        self.frame = self.sim.step(self.decision.commands)
        self.decision = self.runtime.decide(self.frame)
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
    # Rendering: 3D MuJoCo View + 2D Top-Down Heatmap + Inspector + Bekker Lab
    # ------------------------------------------------------------------

    def _refresh_all_views(self) -> None:
        planted_sites = [
            d.target_site_id for d in self.sim.completed_drops if d.contact_observed_at_s is not None
        ]
        self.status_lbl.configure(
            text=(
                f"Step {self.step_count:03d} | t = {self.frame.time_s:5.2f}s | "
                f"Gossip: {self.runtime.total_gossip_packets} pkts ({self.runtime.total_gossip_bytes} B) | "
                f"Planted: {planted_sites or 'none'}"
            )
        )
        self._render_3d_view()
        self._render_2d_belief_map()
        self._update_inspector_and_logs()
        self._render_bekker_curves_tab()

    def _append_3d_hud_geoms(self, scn: Any) -> None:
        """Append 3D per-robot belief spheres, SLAM hazard cells, UWB gossip beams, and LiDAR rays to `scn`."""
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

        obs_by_id = {r.robot_id: r for r in self.frame.robots}
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
        if cam_choice == "isometric":
            self.renderer.update_scene(self.sim.data, camera=self.free_cam)
        else:
            self.renderer.update_scene(self.sim.data, camera=cam_choice)

        self._append_3d_hud_geoms(self.renderer.scene)
        rgb = self.renderer.render()
        ppm_bytes = _rgb_array_to_tk_ppm(rgb)
        self._tk_3d_img = tk.PhotoImage(data=ppm_bytes)
        self.canvas_3d.delete("all")
        self.canvas_3d.create_image(0, 0, anchor=tk.NW, image=self._tk_3d_img)

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
            pg = p_grid[l_key]
            sg = s_grid[l_key]
            grid_2d = [
                [abs(pg[iy][ix] - sg[iy][ix]) for ix in range(self.GRID_NX)]
                for iy in range(self.GRID_NY)
            ]
            vmin, vmax, palette = 0.0, 0.42, "risk"
        else:
            active_grid = p_grid if view_mode == "planter" else s_grid
            if layer == "elevation_m":
                grid_2d = truth_layers["elevation_m"]
            else:
                grid_2d = active_grid.get(layer, active_grid["moisture"])

        pad = 24.0
        plot_w = self.MAP_W - 2 * pad
        plot_h = self.MAP_H - 2 * pad
        cell_w = plot_w / self.GRID_NX
        cell_h = plot_h / self.GRID_NY

        # 1. Draw continuous 2D heatmap cells
        for iy in range(self.GRID_NY):
            for ix in range(self.GRID_NX):
                val = float(grid_2d[iy][ix])
                col_hex = _colormap_hex(val, vmin, vmax, palette)
                x0 = pad + ix * cell_w
                y0 = self.MAP_H - pad - (iy + 1) * cell_h
                c.create_rectangle(
                    x0, y0, x0 + cell_w + 0.8, y0 + cell_h + 0.8, fill=col_hex, outline=""
                )

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
            c.create_rectangle(ox - 10, oy - 10, ox + 10, oy + 10, fill="#dc2626", outline="#fef08a", width=2)
            c.create_text(ox, oy - 15, text="OBSTACLE", fill="#fef08a", font=("Consolas", 7, "bold"))

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
                c.create_oval(sx - 18, sy - 18, sx + 18, sy + 18, outline="#4ade80", width=2, dash=(3, 2))
                c.create_text(sx, sy, text=f"{sid}\n(PROTECTED)", fill="#86efac", font=("Segoe UI", 7, "bold"))
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
                c.create_oval(sx - 16, sy - 16, sx + 16, sy + 16, outline=ring_col, width=2)
                c.create_text(sx, sy - 22, text=tag_txt, fill="#f8fafc", font=("Consolas", 7, "bold"))

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
                    0.5 * (py + scy) - 8,
                    text="UWB+HaLow GOSSIP",
                    fill="#fef08a",
                    font=("Consolas", 7, "bold"),
                )

        # 6. Draw Rover Trajectories & Clickable Rover Icons
        intents_by_rid = {i.robot_id: i for i in self.decision.intents}
        for rid, r_obs in obs_by_id.items():
            pts = self.trajectories.get(rid, [])
            if len(pts) >= 2:
                flat_coords: list[float] = []
                for tx, ty in pts[-45:]:
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
            bellied = bool(sink_st.bellied_out) if sink_st else False

            # Highlight halo if this robot is currently selected for world-belief inspection
            if is_selected:
                c.create_oval(
                    rx - 18, ry - 18, rx + 18, ry + 18, outline="#fef08a", width=3
                )

            fill_col = "#ef4444" if bellied else ("#0284c7" if rid == "planter" else "#16a34a")
            c.create_oval(rx - 11, ry - 11, rx + 11, ry + 11, fill=fill_col, outline="#ffffff", width=2)
            # Heading indicator line
            hx = rx + 16 * cos(r_obs.pose.yaw_rad)
            hy = ry - 16 * sin(r_obs.pose.yaw_rad)
            c.create_line(rx, ry, hx, hy, fill="#fef08a", width=2)

            intent = intents_by_rid.get(rid)
            mode_tag = "STUCK(S1)" if bellied else (intent.mode.value.upper() if intent else "IDLE")
            c.create_text(
                rx,
                ry + 20,
                text=f"{rid.upper()} [{mode_tag}]",
                fill="#fef08a" if is_selected else "#f8fafc",
                font=("Segoe UI", 8, "bold"),
            )

        # Header overlay inside 2D map
        c.create_rectangle(6, 4, self.MAP_W - 6, 22, fill="#0b131e", outline="#1f3247")
        c.create_text(
            12,
            13,
            anchor="w",
            text=f"View: {view_mode.upper()}  |  Layer: {layer_title} [{vmin:.2f} .. {vmax:.2f}]  (Click any robot icon to inspect)",
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
        tel = self.runtime.latest_sensor_telemetry.get(sel_rid)

        self.inspector_title_lbl.configure(
            text=f"🤖 {sel_rid.upper()}'S PRIVATE WORLD MODEL & SENSOR SUITE"
        )

        lines: list[str] = []
        if r_obs is not None:
            lines.append(
                f"Pose     : ({r_obs.pose.x_m:.2f}m, {r_obs.pose.y_m:.2f}m, yaw={r_obs.pose.yaw_rad:.2f}r)"
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
            lines.append("\n--- CONTINUOUS TERRAIN & BEKKER SINKAGE ---")
            lines.append(
                f"Terrain  : z={pt.elevation_m*1000:.1f}mm | slope={pt.slope_magnitude*100:.1f}%"
            )
            lines.append(
                f"Soil     : M={pt.moisture:.2f} | F={pt.firmness:.2f} | Mud={pt.mud_depth_m*1000:.0f}mm"
            )
            lines.append(
                f"Sinkage z: {sink.sinkage_z_m*1000:.1f}mm (Belly clr: {sink.belly_clearance_m*1000:+.1f}mm)"
            )
            lines.append(
                f"Mud Drag : Rc={sink.compaction_resistance_n:.1f}N | Pull={sink.drawbar_pull_margin_n:+.1f}N"
            )
            lines.append(
                f"Status   : {'⚠ BELLIED OUT (S1 STUCK)' if sink.bellied_out else '✓ TRAVERSABLE'}"
                f" (Risk={sink.immobilization_risk*100:.0f}%)"
            )

        if tel is not None:
            lines.append("\n--- LOCAL ONBOARD SENSOR TELEMETRY ---")
            lines.append(
                f"Slip s   : {tel.slip_ratio*100:.1f}% | Pitch={tel.pitch_deg:+.1f}° Roll={tel.roll_deg:+.1f}°"
            )
            lines.append(f"UWB Peers: {dict(tel.uwb_peer_ranges_m)}")

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
                status = "REJECTED ✗"
            m_str = f"{est.moisture:.2f}" if est.moisture is not None else " ?  "
            f_str = f"{est.firmness:.2f}" if est.firmness is not None else " ?  "
            g_str = f"{est.stand_gap:.2f}" if est.stand_gap is not None else " ?  "
            lines.append(
                f"• {sid}: [{status:10s}] M={m_str} F={f_str} G={g_str} (σ={mean_unc:.2f})"
            )

        lines.append(f"\n--- LOCAL 2D SLAM CELLS ({len(node.local_map.slam.cells)}) ---")
        for cell in list(node.local_map.slam.cells.values())[:4]:
            lines.append(
                f"• ({cell.x_m:.2f}, {cell.y_m:.2f}) {cell.kind}: L={cell.log_odds:.2f} by {cell.discovered_by}"
            )

        self.inspector_text.delete("1.0", tk.END)
        self.inspector_text.insert(tk.END, "\n".join(lines))

        # Update bottom Bekker-Wong comparison cards for both rovers
        card_lines: list[str] = []
        for rid in ("planter", "scout"):
            s_st = self.sim.latest_terramechanics.get(rid)
            t_pt = self.sim.latest_terrain_points.get(rid)
            if s_st and t_pt:
                card_lines.append(
                    f"{rid.upper():7s} (m={s_st.mass_kg:4.1f}kg, r={s_st.wheel_radius_m*1000:2.0f}mm, b={s_st.wheel_width_m*1000:2.0f}mm) | "
                    f"Sink z={s_st.sinkage_z_m*1000:5.1f}mm | Clr={s_st.belly_clearance_m*1000:+5.1f}mm | "
                    f"Drag Rc={s_st.compaction_resistance_n:5.1f}N | SpeedFactor={s_st.speed_penalty_factor*100:3.0f}% | "
                    f"{'BELLIED OUT!' if s_st.bellied_out else 'OK'}"
                )
        self.terramech_cards_lbl.configure(text="\n".join(card_lines))

        # Update scrolling UWB/HaLow Gossip Log
        self.gossip_text.delete("1.0", tk.END)
        if not self.runtime.gossip_history:
            self.gossip_text.insert(
                tk.END,
                "No peer-to-peer gossip packets exchanged yet (rovers exchange compact packets when new site evidence, SLAM hazards, or task bids change).",
            )
        else:
            g_lines: list[str] = []
            for pkt in reversed(self.runtime.gossip_history[-12:]):
                g_lines.append(
                    f"[t={pkt.sent_at_s:4.1f}s] {pkt.sender_id.upper()} -> {pkt.receiver_id.upper()} "
                    f"(UWB d={pkt.uwb_distance_m:.2f}m, AoA={pkt.uwb_aoa_rad:+.2f}r, {pkt.byte_size} B): "
                    + " ; ".join(pkt.dialogue_summary[:2])
                )
            self.gossip_text.insert(tk.END, "\n".join(g_lines))

    def _render_bekker_curves_tab(self) -> None:
        c = self.bekker_canvas
        c.delete("all")
        sel_rid = self.selected_robot_var.get()
        pt = self.sim.latest_terrain_points.get(
            sel_rid, self.sim.world.evaluate_continuous_point(1.4, -0.35)
        )

        w = 1380
        h = 660
        pad_l, pad_r, pad_t, pad_b = 80, 50, 60, 70
        pw = w - pad_l - pad_r
        ph = h - pad_t - pad_b

        c.create_text(
            pad_l,
            24,
            anchor="w",
            text=(
                f"Bekker-Wong Wheel-Mud Sinkage z(m) vs. Robot Mass at {sel_rid.upper()}'s Current Soil "
                f"(Moisture={pt.moisture:.2f}, Firmness={pt.firmness:.2f}, Mud Depth={pt.mud_depth_m*1000:.0f}mm)"
            ),
            fill="#38bdf8",
            font=("Segoe UI", 12, "bold"),
        )

        c.create_rectangle(pad_l, pad_t, pad_l + pw, pad_t + ph, fill="#0b1522", outline="#1f3247", width=2)

        # Draw 44mm Paddy Ant belly clearance threshold line
        max_z_mm = 110.0
        belly_y = pad_t + ph - (44.0 / max_z_mm) * ph
        c.create_line(pad_l, belly_y, pad_l + pw, belly_y, fill="#ef4444", width=2, dash=(6, 4))
        c.create_text(
            pad_l + 14,
            belly_y - 10,
            anchor="w",
            text="Paddy Ant Chassis Belly Clearance Threshold (44 mm) — Above this line = S1 Belly-Out Immobilization",
            fill="#f87171",
            font=("Consolas", 9, "bold"),
        )

        masses = [1.5 + 0.5 * i for i in range(30)]  # 1.5 to 16.0 kg
        configs = [
            ("Small Wheel (r=42mm, b=28mm)", 0.042, 0.028, "#f97316"),
            ("Standard Paddy Ant (r=58mm, b=40mm)", 0.058, 0.040, "#38bdf8"),
            ("Wide Flotation Wheel (r=80mm, b=62mm)", 0.080, 0.062, "#22c55e"),
        ]

        for idx, (label, r_m, b_m, col) in enumerate(configs):
            coords: list[float] = []
            for m_kg in masses:
                st = evaluate_bekker_terramechanics(
                    PLANTER_SPEC,
                    moisture=pt.moisture,
                    firmness=pt.firmness,
                    mud_depth_m=pt.mud_depth_m,
                    slope_magnitude=pt.slope_magnitude,
                    slip_ratio=0.22,
                    mass_override_kg=m_kg,
                    wheel_radius_override_m=r_m,
                    wheel_width_override_m=b_m,
                )
                z_mm = min(max_z_mm, st.sinkage_z_m * 1000.0)
                x_px = pad_l + ((m_kg - 1.5) / (16.0 - 1.5)) * pw
                y_px = pad_t + ph - (z_mm / max_z_mm) * ph
                coords.extend([x_px, y_px])
            c.create_line(*coords, fill=col, width=3)
            c.create_text(
                pad_l + pw - 290,
                pad_t + 25 + idx * 24,
                anchor="w",
                text=f"■ {label}",
                fill=col,
                font=("Segoe UI", 10, "bold"),
            )

        # Plot current slider operating point
        cur_m = float(self.mass_kg_var.get())
        cur_r = float(self.wheel_radius_mm_var.get()) / 1000.0
        cur_b = float(self.wheel_width_mm_var.get()) / 1000.0
        cur_st = evaluate_bekker_terramechanics(
            PLANTER_SPEC,
            moisture=pt.moisture,
            firmness=pt.firmness,
            mud_depth_m=pt.mud_depth_m,
            slope_magnitude=pt.slope_magnitude,
            slip_ratio=0.22,
            mass_override_kg=cur_m,
            wheel_radius_override_m=cur_r,
            wheel_width_override_m=cur_b,
        )
        cx = pad_l + ((cur_m - 1.5) / (16.0 - 1.5)) * pw
        cy = pad_t + ph - (min(max_z_mm, cur_st.sinkage_z_m * 1000.0) / max_z_mm) * ph
        c.create_oval(cx - 8, cy - 8, cx + 8, cy + 8, fill="#fef08a", outline="#ef4444", width=2)
        c.create_text(
            cx + 12,
            cy - 10,
            anchor="w",
            text=f"Active Slider Point: m={cur_m:.1f}kg, z={cur_st.sinkage_z_m*1000:.1f}mm, Rc={cur_st.compaction_resistance_n:.1f}N",
            fill="#fef08a",
            font=("Consolas", 9, "bold"),
        )


def launch_live_dashboard(
    *,
    scenario: str = "nominal",
    comms: str = "normal",
    seed: int = 7,
    auto_play: bool = True,
) -> None:
    """Launch the native side-by-side 3D MuJoCo + 2D Top-Down Belief Heatmap desktop application."""
    root = tk.Tk()
    app = LiveSwarmDashboardApp(
        root,
        initial_scenario=scenario,
        initial_comms=comms,
        seed=seed,
    )
    if auto_play:
        root.after(250, app.toggle_play)
    root.mainloop()
