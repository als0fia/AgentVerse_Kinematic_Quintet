"""Interactive visual simulation for 6-Robot RiceSwarm AI in a Chinese flooded paddy.

Features:
- Complete Condition-Based Farming Lifecycle (Days -25 to 135+).
- Two-Section Paddy Plot with Staggered Batches:
  * Section A (Columns 1-5): Starts Day -25, Transplant target Day 0.
  * Section B (Columns 6-10): Offset +20 days, Starts Day -5, Transplant target Day 20.
- 6 Homogeneous Swarm Robots (RS1 to RS6) that simultaneously scan, plant, weed,
  sample soil/water, harvest ripe grain, and rescue peers stuck in mud.
- Crop growth visualization from seedling to golden ripe rice to harvested stubble.
- Controls: Play/Pause Season, Advance 1 Day, Step Tick, Reset, Trigger Mud Incident, Weight Sliders.
"""
from __future__ import annotations

import tkinter as tk
from .core import (
    CropStage,
    CropState,
    MudRiskTier,
    RecoveryPolicy,
    RecoveryWeights,
    RiceSwarmSimulation,
    Status,
    Terrain,
)


class RiceSwarmVisualizer:
    CELL_SIZE = 58
    PADDING = 28

    def __init__(self, root: tk.Tk, *, introduce_mud: bool = True):
        self.root = root
        self.root.title("RiceSwarm AI — Complete Condition-Based Farming Lifecycle Simulation (China Paddy)")
        self.root.geometry("1460x840")
        self.root.configure(bg="#1a202c")

        self.introduce_mud = introduce_mud
        self.running_ticks = False
        self.running_season = False
        self.after_id: str | None = None

        self.wt_var = tk.DoubleVar(value=0.25)
        self.we_var = tk.DoubleVar(value=0.20)
        self.wc_var = tk.DoubleVar(value=0.30)
        self.ws_var = tk.DoubleVar(value=0.15)
        self.wr_var = tk.DoubleVar(value=0.10)
        self.show_truth_var = tk.BooleanVar(value=False)

        self.sim = self._build_sim()
        self._build_ui()
        self._render()

    def _build_sim(self) -> RiceSwarmSimulation:
        total = self.wt_var.get() + self.we_var.get() + self.wc_var.get() + self.ws_var.get() + self.wr_var.get()
        if total <= 0:
            total = 1.0
        weights = RecoveryWeights(
            time=round(self.wt_var.get() / total, 4),
            energy=round(self.we_var.get() / total, 4),
            crop_damage=round(self.wc_var.get() / total, 4),
            soil_disturbance=round(self.ws_var.get() / total, 4),
            recovery_risk=round(self.wr_var.get() / total, 4),
        )
        return RiceSwarmSimulation(
            policy=RecoveryPolicy(weights),
            introduce_mud=self.introduce_mud,
            start_day=-25,
        )

    def _build_ui(self) -> None:
        main_frame = tk.Frame(self.root, bg="#1a202c")
        main_frame.pack(fill=tk.BOTH, expand=True, padx=12, pady=10)

        # Left column: Grid canvas + Status Banner
        left_frame = tk.Frame(main_frame, bg="#1a202c")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=False)

        # Top section label header
        header_frame = tk.Frame(left_frame, bg="#1a202c")
        header_frame.pack(fill=tk.X, pady=(0, 4))
        tk.Label(
            header_frame,
            text="◄ SECTION A (Batch 1: Day -25 Start)      |      SECTION B (Batch 2: +20d Offset) ►",
            font=("Segoe UI", 10, "bold"),
            fg="#63b3ed",
            bg="#1a202c",
        ).pack()
        tk.Label(
            header_frame,
            text="⚡ Edge Compute Budget: ~0.4ms / decision  |  Power: ~2.4W (STM32/Pi5)  |  Cloud GPU: $0 (100% Offline Standalone)",
            font=("Consolas", 8, "bold"),
            fg="#48bb78",
            bg="#1a202c",
        ).pack(pady=(2, 0))

        cw = self.sim.field.width * self.CELL_SIZE + 2 * self.PADDING
        ch = self.sim.field.height * self.CELL_SIZE + 2 * self.PADDING
        self.canvas = tk.Canvas(
            left_frame,
            width=cw,
            height=ch,
            bg="#111620",
            highlightthickness=1,
            highlightbackground="#2d3748",
        )
        self.canvas.pack(pady=(0, 6))

        self.status_banner = tk.Label(
            left_frame,
            text="",
            font=("Consolas", 10, "bold"),
            fg="#e2e8f0",
            bg="#2d3748",
            padx=10,
            pady=6,
            anchor="w",
        )
        self.status_banner.pack(fill=tk.X)

        # Right column: Controls + Telemetry + AI Decisions
        right_frame = tk.Frame(main_frame, bg="#1a202c")
        right_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(12, 0))

        ctrl_frame = tk.LabelFrame(
            right_frame,
            text=" Agricultural Lifecycle & Swarm Controls (6 Robots) ",
            font=("Segoe UI", 9, "bold"),
            fg="#63b3ed",
            bg="#2d3748",
            padx=8,
            pady=8,
        )
        ctrl_frame.pack(fill=tk.X, pady=(0, 6))

        btn_row = tk.Frame(ctrl_frame, bg="#2d3748")
        btn_row.pack(fill=tk.X)

        self.play_season_btn = tk.Button(
            btn_row,
            text="▶ Play Season",
            command=self.toggle_play_season,
            bg="#38a169",
            fg="white",
            font=("Segoe UI", 9, "bold"),
            width=12,
        )
        self.play_season_btn.pack(side=tk.LEFT, padx=3)

        tk.Button(
            btn_row,
            text="⏭ Advance 1 Day",
            command=self.advance_day_once,
            bg="#2b6cb0",
            fg="white",
            font=("Segoe UI", 9, "bold"),
            width=14,
        ).pack(side=tk.LEFT, padx=3)

        tk.Button(
            btn_row,
            text="Step Tick",
            command=self.step_tick_once,
            bg="#4a5568",
            fg="white",
            font=("Segoe UI", 9),
            width=9,
        ).pack(side=tk.LEFT, padx=3)

        tk.Button(
            btn_row,
            text="↺ Reset",
            command=self.reset_sim,
            bg="#718096",
            fg="white",
            font=("Segoe UI", 9, "bold"),
            width=7,
        ).pack(side=tk.LEFT, padx=3)

        tk.Button(
            btn_row,
            text="⚠ Mud Stuck (RS2)",
            command=lambda: self.force_stuck("RS2"),
            bg="#dd6b20",
            fg="white",
            font=("Segoe UI", 9, "bold"),
            width=16,
        ).pack(side=tk.LEFT, padx=3)

        tk.Button(
            btn_row,
            text="🚧 Inject Obstacle",
            command=self.inject_obstacle,
            bg="#9b2c2c",
            fg="white",
            font=("Segoe UI", 9, "bold"),
            width=15,
        ).pack(side=tk.LEFT, padx=3)

        chk_row = tk.Frame(ctrl_frame, bg="#2d3748")
        chk_row.pack(fill=tk.X, pady=(6, 0))
        tk.Checkbutton(
            chk_row,
            text="Reveal Hidden Ground Truth",
            variable=self.show_truth_var,
            command=self._render,
            bg="#2d3748",
            fg="#e2e8f0",
            selectcolor="#1a202c",
            activebackground="#2d3748",
        ).pack(side=tk.LEFT)

        # Multi-Objective Sliders
        weights_frame = tk.LabelFrame(
            right_frame,
            text=" Multi-Objective Recovery Weights: J = wt·T + we·E + wc·C + ws·S + wr·R ",
            font=("Segoe UI", 8, "bold"),
            fg="#63b3ed",
            bg="#2d3748",
            padx=8,
            pady=4,
        )
        weights_frame.pack(fill=tk.X, pady=(0, 6))

        for label_text, var in (
            ("wt (Time)", self.wt_var),
            ("we (Energy)", self.we_var),
            ("wc (Crop Damage)", self.wc_var),
            ("ws (Soil Disturbance)", self.ws_var),
            ("wr (Recovery Risk)", self.wr_var),
        ):
            row = tk.Frame(weights_frame, bg="#2d3748")
            row.pack(fill=tk.X, pady=1)
            tk.Label(
                row,
                text=label_text,
                width=18,
                anchor="w",
                fg="#cbd5e0",
                bg="#2d3748",
                font=("Consolas", 8),
            ).pack(side=tk.LEFT)
            tk.Scale(
                row,
                variable=var,
                from_=0.0,
                to=1.0,
                resolution=0.05,
                orient=tk.HORIZONTAL,
                command=lambda _v: self._update_weights(),
                bg="#2d3748",
                fg="#cbd5e0",
                highlightthickness=0,
                troughcolor="#1a202c",
                length=180,
            ).pack(side=tk.RIGHT, fill=tk.X, expand=True)

        # Telemetry & AI coordinator log
        log_frame = tk.LabelFrame(
            right_frame,
            text=" Farming Lifecycle Dashboard & AI Coordinator Telemetry ",
            font=("Segoe UI", 9, "bold"),
            fg="#63b3ed",
            bg="#2d3748",
            padx=8,
            pady=6,
        )
        log_frame.pack(fill=tk.BOTH, expand=True)

        self.log_text = tk.Text(
            log_frame,
            bg="#0d1117",
            fg="#c9d1d9",
            font=("Consolas", 9),
            wrap=tk.WORD,
            height=16,
            padx=6,
            pady=6,
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def _update_weights(self) -> None:
        total = self.wt_var.get() + self.we_var.get() + self.wc_var.get() + self.ws_var.get() + self.wr_var.get()
        if total <= 0:
            total = 1.0
        self.sim.policy = RecoveryPolicy(
            RecoveryWeights(
                time=round(self.wt_var.get() / total, 4),
                energy=round(self.we_var.get() / total, 4),
                crop_damage=round(self.wc_var.get() / total, 4),
                soil_disturbance=round(self.ws_var.get() / total, 4),
                recovery_risk=round(self.wr_var.get() / total, 4),
            )
        )
        self.sim.coordinator.policy = self.sim.policy

    def toggle_play_season(self) -> None:
        self.running_season = not self.running_season
        self.play_season_btn.configure(
            text="⏸ Pause" if self.running_season else "▶ Play Season",
            bg="#e53e3e" if self.running_season else "#38a169",
        )
        if self.running_season:
            self._season_loop()
        elif self.after_id:
            self.root.after_cancel(self.after_id)
            self.after_id = None

    def _season_loop(self) -> None:
        if not self.running_season:
            return
        if self.sim.current_day < 145 and len(self.sim.field.harvested) < len(self.sim.field.crop_targets):
            self.sim.advance_day()
            self._render()
            self.after_id = self.root.after(350, self._season_loop)
        else:
            self.running_season = False
            self.play_season_btn.configure(text="▶ Play Season", bg="#38a169")
            self._render()

    def advance_day_once(self) -> None:
        if self.running_season:
            self.toggle_play_season()
        self.sim.advance_day()
        self._render()

    def step_tick_once(self) -> None:
        if self.running_season:
            self.toggle_play_season()
        self.sim.step()
        self._render()

    def reset_sim(self) -> None:
        if self.running_season:
            self.toggle_play_season()
        self.sim = self._build_sim()
        self._render()

    def force_stuck(self, robot_name: str) -> None:
        self._update_weights()
        self.sim.trigger_immobilisation(robot_name)
        self._render()

    def inject_obstacle(self) -> None:
        candidates = [(3, 4), (5, 5), (6, 4), (1, 5), (8, 5), (4, 3), (7, 6)]
        target = None
        for c in candidates:
            if self.sim.field.truth.get(c) != Terrain.BLOCKED and not self.sim.field.belief[c].obstacle:
                target = c
                break
        if target is None:
            target = (3, 4)
        self.sim.field.truth[target] = Terrain.BLOCKED
        self.sim.field.belief[target].terrain = Terrain.BLOCKED
        self.sim.field.belief[target].obstacle = True
        for r in self.sim.robots.values():
            if target in r.planned_route:
                r.planned_route = []
                r.destination = None
        self.sim.event("obstacle_injected", position=target)
        self._render()

    def _cell_rect(self, x: int, y: int) -> tuple[int, int, int, int]:
        cs = self.CELL_SIZE
        px = self.PADDING + x * cs
        py = self.PADDING + (self.sim.field.height - 1 - y) * cs
        return px, py, px + cs, py + cs

    def _cell_center(self, x: int, y: int) -> tuple[float, float]:
        x0, y0, x1, y1 = self._cell_rect(x, y)
        return (x0 + x1) / 2, (y0 + y1) / 2

    def _render(self) -> None:
        c = self.canvas
        c.delete("all")
        f = self.sim.field
        show_truth = self.show_truth_var.get()

        for (x, y), cell in f.belief.items():
            x0, y0, x1, y1 = self._cell_rect(x, y)
            terrain = f.truth[(x, y)] if show_truth else cell.terrain

            # Flooded Chinese rice paddy water / terrain coloring
            if cell.obstacle:
                fill = "#742a2a"  # Injected dynamic obstacle
            elif terrain == Terrain.BLOCKED:
                fill = "#4a5568"  # Tian Geng bund dike
            elif (x, y) in f.risk_incidents or (show_truth and terrain == Terrain.SOFT):
                fill = "#9b2c2c"  # Soft mud hazard
            elif (x, y) in f.harvested:
                fill = "#744210"  # Golden-brown harvested stubble
            elif (x, y) in f.crop_targets:
                plant = f.plants.get((x, y))
                if plant and plant.planted:
                    if plant.grain_maturity >= 0.90:
                        fill = "#d69e2e"  # Ripe golden rice
                    elif plant.stage in (CropStage.PANICLE_DEV, CropStage.HEADING_FLOWERING, CropStage.GRAIN_FILLING):
                        fill = "#2f855a"  # Lush mature green
                    else:
                        fill = "#38a169"  # Active vegetative green
                else:
                    fill = "#2b6cb0"  # Flooded target
            elif cell.confidence > 0.8:
                fill = "#90cdf4"  # Clear flooded water surface (scanned)
            else:
                fill = "#2b6cb0"  # Unscanned water surface

            c.create_rectangle(x0, y0, x1, y1, fill=fill, outline="#2d3748", width=1)

            # Coordinates
            c.create_text(
                x0 + 3, y0 + 3,
                text=f"{x},{y}",
                anchor="nw",
                font=("Consolas", 6),
                fill="#1a202c" if fill not in ("#4a5568", "#9b2c2c", "#2b6cb0", "#744210") else "#e2e8f0",
            )

            # Water and mud risk text
            w_d = f.water[(x, y)] if show_truth else (cell.water_depth or 0.08)
            tier = cell.mud_risk_tier.value[:3]
            c.create_text(
                x1 - 3, y1 - 3,
                text=f"W:{w_d*100:.0f}cm\nM:{tier}",
                anchor="se",
                font=("Consolas", 6),
                fill="#1a202c" if fill not in ("#4a5568", "#9b2c2c", "#2b6cb0", "#744210") else "#e2e8f0",
            )

            # Crop stage and status text
            if (x, y) in f.crop_targets:
                cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
                plant = f.plants.get((x, y))
                if (x, y) in f.harvested:
                    c.create_text(cx, cy - 2, text="HARVESTED\n0.85kg", font=("Segoe UI", 7, "bold"), fill="#fefcbf")
                elif (x, y) in f.planted:
                    mat = plant.grain_maturity if plant else 0.0
                    if mat >= 0.90:
                        c.create_text(cx, cy - 6, text="RIPE RICE", font=("Segoe UI", 7, "bold"), fill="#744210")
                        c.create_text(cx, cy + 6, text=f"Mat:{mat*100:.0f}%", font=("Consolas", 6, "bold"), fill="#744210")
                    elif (x, y) in f.weeded:
                        c.create_text(cx, cy - 6, text="TENDED", font=("Segoe UI", 7, "bold"), fill="#ffffff")
                        c.create_text(cx, cy + 6, text=f"Til:{plant.tiller_count if plant else 1}", font=("Consolas", 6), fill="#e2e8f0")
                    else:
                        weed_lvl = f.weed_density.get((x, y), 0.7)
                        c.create_text(cx, cy - 6, text="PLANTED", font=("Segoe UI", 7, "bold"), fill="#1a202c")
                        c.create_text(cx, cy + 6, text=f"Weed:{weed_lvl:.2f}", font=("Consolas", 6, "bold"), fill="#742a2a")
                else:
                    prod = cell.productivity_score
                    c.create_oval(cx - 16, cy - 14, cx + 16, cy + 10, outline="#3182ce", width=2, dash=(2, 2))
                    c.create_text(cx, cy - 2, text=f"TARGET\nP:{prod:.2f}", font=("Consolas", 6, "bold"), fill="#ebf8ff")

            if cell.obstacle:
                cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
                c.create_text(cx, cy, text="🚧 OBSTACLE", font=("Segoe UI", 7, "bold"), fill="#feb2b2")
            elif (x, y) in f.risk_incidents:
                cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
                c.create_text(cx, cy - 4, text="MUD STUCK", font=("Segoe UI", 7, "bold"), fill="#fff5f5")

        # Draw Section Divider (between column 5 and 6)
        div_x = self.PADDING + 6 * self.CELL_SIZE - 2
        top_y = self.PADDING
        bot_y = self.PADDING + self.sim.field.height * self.CELL_SIZE
        c.create_line(div_x, top_y, div_x, bot_y, fill="#e2e8f0", width=3, dash=(6, 4))

        # Draw planned routes
        route_palette = ["#f6ad55", "#68d391", "#63b3ed", "#f687b3", "#b794f4", "#faf089"]
        for idx, (name, robot) in enumerate(sorted(self.sim.robots.items())):
            if len(robot.planned_route) > 1:
                pts = []
                for rx, ry in robot.planned_route:
                    cx, cy = self._cell_center(rx, ry)
                    pts.extend((cx, cy))
                c.create_line(
                    *pts,
                    fill=route_palette[idx % len(route_palette)],
                    width=2,
                    dash=(4, 2),
                )

        # Draw 6 robots
        for idx, (name, robot) in enumerate(sorted(self.sim.robots.items())):
            cx, cy = self._cell_center(*robot.pos)
            color = "#e53e3e" if robot.status == Status.STUCK else (
                "#ed8936" if robot.status == Status.TO_RESCUE else "#3182ce"
            )
            c.create_oval(cx - 15, cy - 15, cx + 15, cy + 15, fill=color, outline="#ffffff", width=2)
            c.create_text(cx, cy - 3, text=name, font=("Segoe UI", 8, "bold"), fill="#ffffff")
            c.create_text(cx, cy + 7, text=robot.status.value[:4].upper(), font=("Consolas", 6), fill="#e2e8f0")

        # Status banner
        sec_a = f.sections["A"]
        sec_b = f.sections["B"]
        p_a = sum(1 for p in sec_a.targets if p in f.planted)
        p_b = sum(1 for p in sec_b.targets if p in f.planted)
        h_a = sum(1 for p in sec_a.targets if p in f.harvested)
        h_b = sum(1 for p in sec_b.targets if p in f.harvested)

        self.status_banner.configure(
            text=(
                f"DAY {self.sim.current_day:3d} [{self.sim.current_weather.value.upper()}] | "
                f"SEC A: {sec_a.current_stage.value} (Planted:{p_a}/6, Harvest:{h_a}/6) | "
                f"SEC B: {sec_b.current_stage.value} (Planted:{p_b}/6, Harvest:{h_b}/6) | "
                f"RESCUES: {self.sim.rescue_successes}/{self.sim.rescue_count}"
            )
        )

        self._update_log_panel()

    def _update_log_panel(self) -> None:
        sec_a = self.sim.field.sections["A"]
        sec_b = self.sim.field.sections["B"]
        lines: list[str] = []
        lines.append(f"=== AGRICULTURAL LIFECYCLE: DAY {self.sim.current_day} ({self.sim.current_weather.value.upper()}) ===")
        lines.append(
            f"• Section A (Eff Day {sec_a.effective_day(self.sim.current_day):+d}): "
            f"Stage={sec_a.current_stage.value} | Sown={sec_a.nursery_sown}/12 | "
            f"Planted={sum(1 for p in sec_a.targets if p in self.sim.field.planted)}/6 | "
            f"Harvested={sec_a.harvested_count}/6 ({sec_a.total_yield_kg:.2f}kg)"
        )
        lines.append(
            f"• Section B (Eff Day {sec_b.effective_day(self.sim.current_day):+d}, +20d Offset): "
            f"Stage={sec_b.current_stage.value} | Sown={sec_b.nursery_sown}/12 | "
            f"Planted={sum(1 for p in sec_b.targets if p in self.sim.field.planted)}/6 | "
            f"Harvested={sec_b.harvested_count}/6 ({sec_b.total_yield_kg:.2f}kg)"
        )

        lines.append("\n=== 6 EMBODIED SWARM ROBOTS: [PERCEIVE] → [DECIDE] → [ACT] (100% Offline, Nothing to Phone Home To) ===")
        for name, r in sorted(self.sim.robots.items()):
            action_desc = f"{r.current_action} @ {r.target_cell}" if r.target_cell else "scanning/idle"
            cell = self.sim.field.belief.get(r.pos)
            sense_info = f"W:{cell.water_depth or 0.08:.2f}m, Mud:{cell.mud_risk_tier.value[:3]}" if cell else "scanning"
            lines.append(
                f"• {name} | Pos:{r.pos} | PERCEIVE: [{sense_info}] | DECIDE: {r.status.value.upper():8s} | "
                f"ACT: {action_desc:14s} | Bat:{r.battery:4.1f}%"
            )

        harvest_map = self.sim.coordinator.compute_priority_harvesting_map()
        if harvest_map:
            lines.append("\n=== PRIORITY HARVESTING ASSIGNMENTS (Ready Ripe Grain) ===")
            for pt, mat in harvest_map[:3]:
                lines.append(f"  Ripe Stand {pt}: Grain Maturity = {mat*100:.1f}%")

        planting_map = self.sim.coordinator.compute_priority_planting_map()
        if planting_map:
            lines.append("\n=== PRIORITY TRANSPLANTING ASSIGNMENTS ===")
            for pt, score in planting_map[:3]:
                c = self.sim.field.belief[pt]
                lines.append(f"  Target {pt}: Productivity={score:.3f} (Fert={c.fertility:.2f}, Water={c.water_depth:.2f}m)")

        weeding_map = self.sim.coordinator.compute_priority_weeding_map()
        if weeding_map:
            lines.append("\n=== PRECISION MECHANICAL WEEDING ASSIGNMENTS ===")
            for pt, urg in weeding_map[:3]:
                lines.append(f"  Planted Stand {pt}: Weed Coverage Urgency={urg*100:.1f}%")

        if self.sim.coordinator.active_rescue_decision is not None:
            d = self.sim.coordinator.active_rescue_decision
            stuck_id = self.sim.coordinator.active_stuck_robot
            lines.append(f"\n=== MUD RESCUE COORDINATION (Peer {stuck_id} Immobilised) ===")
            lines.append(f"Selected Action   : {d.action} (Responder: {d.selected_responder})")
            lines.append(f"Reasoning         : {d.reason}")
            lines.append(f"Evaluated Costs J : {d.costs}")
            for ev in d.responder_evaluations:
                if ev.feasible:
                    lines.append(f"  ✓ {ev.responder_id}: stance={ev.stance}, steps={len(ev.route)-1}, J={ev.total_cost:.2f}")

        lines.append("\n=== RECENT FIELD & CROP EVENTS ===")
        for ev in self.sim.log[-5:]:
            lines.append(f"[Day {ev.get('day', 0):3d} | t={ev.get('tick', 0):02d}] {ev.get('event')}: { {k: v for k, v in ev.items() if k not in ('tick', 'day', 'event', 'normalised_components', 'safety_constraints', 'responder_evaluations')} }")

        self.log_text.delete("1.0", tk.END)
        self.log_text.insert(tk.END, "\n".join(lines))


def run_gui(*, introduce_mud: bool = True) -> None:
    root = tk.Tk()
    RiceSwarmVisualizer(root, introduce_mud=introduce_mud)
    root.mainloop()


if __name__ == "__main__":
    run_gui()
