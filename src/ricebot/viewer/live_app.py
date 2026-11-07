r"""Unified Side-by-Side 3D MuJoCo Physics & 2D Top-Down Swarm Belief Desktop Application.

Single-window side-by-side layout (no extra tabs or low-fidelity grid simulators):
- **Left Side — Live 3D MuJoCo Physics Simulation (`640 x 430`):**
  - Renders the 3D undulating paddy terrace heightfield (`paddy_micro_relief`), physical rover
    pitch/roll over uneven micro-topography, real-time 3D vertical wheel-mud sinkage ($z_{\text{sink}}$),
    wheel-slip fishtailing in soft mud, 3D sunken wheel-rut tracks (`sim.mud_rut_history`), 3D mud-collar
    rings around bogging wheels, split dual-rover belief beacons, local SLAM hazard cells, and UWB/HaLow
    gossip beams.
  - Directly beneath the 3D view: Live Bekker-Wong Vehicle Sizing Sliders (`Robot Mass kg`, `Wheel Radius mm`,
    `Wheel Width mm`) and real-time 3D Terramechanics & Slip Gauges (`Sinkage z`, `Belly Clearance`,
    `Wheel Slip s`, `Mud Drag Rc`, `Drawbar Pull`, and `S1 Belly-Out Status`).
- **Right Side — Interactive 2D Top-Down Belief & Agronomic Fitness Heatmap (`550 x 430`) + Robot Inspector:**
  - Click directly on `PLANTER` or `SCOUT` on the 2D canvas (or select from the top bar) to inspect what
    that specific robot understands about the continuous field (`Planter Local Belief`, `Scout Local Belief`,
    `|Planter - Scout| Divergence`, or `Hidden Ground Truth f(x,y)`).
  - Switch across 8 continuous spatial layers including the agronomically researched **Wet Direct-Seeded
    Rice Planting Fitness Index $\Phi_{\text{plant}}(x,y)$**, Soil Moisture $M(x,y)$, Cone Firmness $F(x,y)$,
    Canopy Stand Gap $G(x,y)$, Plant Density $\rho(x,y)$, Epistemic Uncertainty $\sigma(x,y)$, Mud/SLAM
    Traversability Risk $R(x,y)$, and Micro-Topography Elevation $z(x,y)$.
"""
from __future__ import annotations

from math import cos, hypot, sin
import tkinter as tk
from typing import Any

import mujoco
import numpy as np

from ..agent import (
    DecentralizedMycelialRuntime,
    SoilSafetyEnvelope,
    sample_continuous_belief_grid,
)
from ..contracts import Decision, ObservationFrame, TaskMode
from ..mujoco_sim.adapter import DEFAULT_CAPABILITIES, DEFAULT_SITES, MujocoPaddySimulation
from ..world import DEFAULT_HIDDEN_SOIL_TRUTH, UNEXPECTED_BAD_SOIL_TRUTH


def _rgb_array_to_tk_ppm(rgb: np.ndarray) -> bytes:
    """Convert an (H, W, 3) uint8 RGB NumPy array into in-memory binary PPM bytes for tk.PhotoImage."""
    h, w, _ = rgb.shape
    header = f"P6\n{w} {h}\n255\n".encode("ascii")
    return header + np.ascontiguousarray(rgb, dtype=np.uint8).tobytes()


def _colormap_hex(val: float, vmin: float, vmax: float, palette: str) -> str:
    t = max(0.0, min(1.0, (val - vmin) / max(1e-6, vmax - vmin)))
    if palette == "fitness":
        # Red/brown (unfit anoxic mud or dense crop) -> Amber -> Bright Emerald Green (high WDSR fitness)
        r = int(195 * (1 - t) + 25 * t)
        g = int(55 + 175 * (t**0.85))
        b = int(45 + 65 * t)
    elif palette == "moisture":
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
    """Single-window side-by-side 3D MuJoCo physics simulator and 2D Top-Down Belief & Fitness Dashboard."""

    RENDER_W = 620
    RENDER_H = 420
    MAP_W = 540
    MAP_H = 420
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
            "Antenna (PaddyPatch rice.v1) — Side-by-Side 3D MuJoCo Physics & Decentralised Robot Belief Dashboard"
        )
        self.root.geometry("1540x900")
        self.root.configure(bg="#0b131e")

        self.seed = seed
        self.scenario_var = tk.StringVar(value=initial_scenario)
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
        # Top Control Toolbar
        toolbar = tk.Frame(self.root, bg="#111d2c", highlightthickness=1, highlightbackground="#1f3247")
        toolbar.pack(fill=tk.X, padx=8, pady=(6, 4))

        self.play_btn = tk.Button(
            toolbar,
            text="▶ Play Live 3D Sim",
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
            text="⚠ Test Heavy Mud Sinkage & Belly-Out (13 kg)",
            command=self.trigger_mud_belly_out_demo,
            bg="#b91c1c",
            fg="white",
            font=("Segoe UI", 9, "bold"),
            padx=8,
            pady=3,
        ).pack(side=tk.LEFT, padx=6, pady=5)

        tk.Button(
            toolbar,
            text="✓ Restore 4.2 kg Paddy Ant",
            command=self.restore_nominal_sizing,
            bg="#15803d",
            fg="white",
            font=("Segoe UI", 8, "bold"),
            padx=7,
            pady=3,
        ).pack(side=tk.LEFT, padx=2, pady=5)

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

        # Middle Split Pane: Left = 3D MuJoCo View, Right = 2D Top-Down Belief Heatmap + Inspector
        mid_frame = tk.Frame(self.root, bg="#0b131e")
        mid_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        # LEFT PANE: 3D MuJoCo View
        left_pane = tk.LabelFrame(
            mid_frame,
            text=" Live 3D MuJoCo Physics (Undulating Micro-Topography + 3D Mud Sinkage & Wheel Ruts + Slip + UWB HUD) ",
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
            ("wheel_closeup", "Low Mud/Wheel Close-Up"),
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
        self.canvas_3d.pack()
        self._drag_last_xy: tuple[int, int] | None = None
        self.canvas_3d.bind("<ButtonPress-1>", self._on_3d_mouse_down)
        self.canvas_3d.bind("<B1-Motion>", self._on_3d_mouse_drag)
        self.canvas_3d.bind("<MouseWheel>", self._on_3d_mouse_wheel)

        tk.Label(
            left_pane,
            text=(
                "3D Physics: Rovers pitch/roll over undulating terrain z(x,y), sink vertically by Bekker z_sink(x,y), "
                "leave 3D mud ruts, and slip in soft clay."
            ),
            bg="#111d2c",
            fg="#94a3b8",
            font=("Segoe UI", 8),
        ).pack(anchor="w", pady=(3, 0))

        # RIGHT PANE: Interactive 2D Top-Down Belief Heatmap & Click-to-Select Robot
        right_pane = tk.LabelFrame(
            mid_frame,
            text=" Interactive 2D Top-Down Belief & Rice-Fitness Map (Click a Robot on Map or Select Below) ",
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
            ).pack(side=tk.LEFT, padx=3)

        # Continuous Heatmap Layer Bar
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
            height=24,
            padx=6,
            pady=4,
            borderwidth=0,
        )
        self.inspector_text.pack(fill=tk.BOTH, expand=True)

        # BOTTOM DOCK: Bekker-Wong Sliders + Per-Robot Sinkage Cards + UWB Gossip Log
        bottom_frame = tk.Frame(self.root, bg="#0b131e")
        bottom_frame.pack(fill=tk.BOTH, expand=False, padx=8, pady=(2, 6))

        sizing_frame = tk.LabelFrame(
            bottom_frame,
            text=" Live 3D Bekker-Wong Mud Sinkage, Wheel Slip & Vehicle Sizing Controls ",
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
            height=8,
            bg="#070d14",
            fg="#cbd5e1",
            font=("Consolas", 8),
            wrap=tk.WORD,
            padx=6,
            pady=4,
            borderwidth=0,
        )
        self.gossip_text.pack(fill=tk.BOTH, expand=True)

    # ------------------------------------------------------------------
    # Interactive Camera & Canvas Event Handlers
    # ------------------------------------------------------------------

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
    # Rendering: 3D MuJoCo View + 2D Top-Down Heatmap + Inspector
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
                font=("Consolas", 8, "bold"),
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

            if is_selected:
                c.create_oval(
                    rx - 18, ry - 18, rx + 18, ry + 18, outline="#fef08a", width=3
                )

            fill_col = "#ef4444" if bellied else ("#0284c7" if rid == "planter" else "#16a34a")
            c.create_oval(rx - 11, ry - 11, rx + 11, ry + 11, fill=fill_col, outline="#ffffff", width=2)
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

        c.create_rectangle(6, 4, self.MAP_W - 6, 22, fill="#0b131e", outline="#1f3247")
        c.create_text(
            12,
            13,
            anchor="w",
            text=f"View: {view_mode.upper()}  |  Layer: {layer_title} [{vmin:.2f} .. {vmax:.2f}]  (Click robot to inspect)",
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
            text=f"🤖 {sel_rid.upper()}'S WORLD MODEL & 3D MUD TELEMETRY"
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
            for pkt in reversed(self.runtime.gossip_history[-12:]):
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
    seed: int = 7,
    auto_play: bool = True,
) -> None:
    """Launch the unified side-by-side 3D MuJoCo + 2D Top-Down Belief & Fitness desktop window."""
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
