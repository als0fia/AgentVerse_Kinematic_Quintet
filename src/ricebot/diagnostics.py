"""Independent, read-only MuJoCo episode instrumentation for developer-agent audits.

Physics truth is held in the evaluator only. No readings from this module are
returned to the planner or used to modify the robot's controls.
"""
from __future__ import annotations

from collections import Counter, deque
from dataclasses import asdict
import json
from math import hypot
from pathlib import Path
from typing import Any

from .contracts import Decision, ObservationFrame, RobotCommand, TaskMode

TRACE_VERSION = "antenna.diagnostics.v1"


class MujocoPhysicsProbe:
    """Sample body pose, terrain state and solver contacts at each completed tick.

    End-of-tick sampling can miss transient contacts inside a MuJoCo step.
    Contact records prove solver contact at sample time, not an impact history.
    """

    def __init__(self, sim: Any, robot_ids: tuple[str, ...] | None = None):
        import mujoco

        if robot_ids is None:
            robot_ids = tuple(spec.robot_id for spec in sim.rover_specs)
        self.sim, self.mj, self.robot_ids = sim, mujoco, robot_ids
        model = sim.model
        roots = {
            rid: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, rid + "_body")
            for rid in robot_ids
        }
        if any(body < 0 for body in roots.values()):
            raise ValueError("robot root body missing; update diagnostic body mapping")
        self.body_owner: dict[int, str | None] = {}
        for body in range(model.nbody):
            parent = body
            owner = None
            while parent > 0:
                for rid, root in roots.items():
                    if parent == root:
                        owner = rid
                        break
                if owner is not None:
                    break
                parent = int(model.body_parentid[parent])
            self.body_owner[body] = owner

    def snapshot(self) -> dict[str, Any]:
        import numpy as np

        sim, mj = self.sim, self.mj
        model, data = sim.model, sim.data
        robots: dict[str, dict[str, Any]] = {}
        for rid in self.robot_ids:
            pos = data.sensor(rid + "_pos").data
            sink = sim.latest_terramechanics.get(rid)
            terrain = sim.latest_terrain_points.get(rid)
            robots[rid] = {
                "x_m": round(float(pos[0]), 5),
                "y_m": round(float(pos[1]), 5),
                "z_m": round(float(pos[2]), 5),
                "sinkage_m": getattr(sink, "sinkage_z_m", None),
                "belly_clearance_m": getattr(sink, "belly_clearance_m", None),
                "slip_ratio": sim.latest_slip_ratios.get(rid),
                "terrain_elevation_m": getattr(terrain, "elevation_m", None),
            }
        contacts: list[dict[str, Any]] = []
        for idx in range(int(data.ncon)):
            con = data.contact[idx]
            g1, g2 = int(con.geom1), int(con.geom2)
            a = self.body_owner.get(int(model.geom_bodyid[g1]))
            b = self.body_owner.get(int(model.geom_bodyid[g2]))
            if a == b or (a is None and b is None):
                continue
            names = [
                mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, g1) or str(g1),
                mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, g2) or str(g2),
            ]
            if a is None or b is None:
                other_name = names[0] if a is None else names[1]
                if any(s in other_name.lower() for s in ("ground", "terrain", "hfield")):
                    continue  # normal wheel-ground support is not a defect
            force = np.zeros(6, dtype=float)
            mj.mj_contactForce(model, data, idx, force)
            contacts.append({
                "actors": sorted([a or "environment", b or "environment"]),
                "geoms": names,
                "penetration_m": round(max(0.0, -float(con.dist)), 6),
                "normal_force_n": round(max(0.0, float(force[0])), 4),
            })
        # Keep full robot-robot evidence and strongest environment contacts.
        contacts.sort(key=lambda x: ("environment" not in x["actors"],
                                     x["normal_force_n"]), reverse=True)
        return {
            "time_s": round(float(data.time), 5),
            "robots": robots,
            "contacts": contacts[:32],
            "contact_count_before_filter": int(data.ncon),
            "latched_pair": list(sim._active_latched_pair) if sim._active_latched_pair else None,
        }


class EpisodeDiagnosticPort:
    """SimulationPort wrapper and ReadOnlyView, with versioned JSONL evidence.

    No shared schema, simulator source or planner behaviour changes are needed.
    """

    def __init__(
        self, simulation: Any, probe: Any, trace_path: str | Path, *,
        scenario: str, policy: str, scripted_trap: bool,
        stall_window_s: float = 1.2,
        stall_command_m: float = 0.12,
        stall_actual_m: float = 0.035,
    ):
        self.sim, self.probe = simulation, probe
        self.trace_path = Path(trace_path)
        self.trace_path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.trace_path.open("w", encoding="utf-8")
        self.scenario, self.policy, self.scripted_trap = scenario, policy, scripted_trap
        self.window_s, self.command_m, self.actual_m = (
            stall_window_s, stall_command_m, stall_actual_m
        )
        self.seed: int | None = None
        self.previous: ObservationFrame | None = None
        self.before: dict[str, Any] | None = None
        self.decision: Decision | None = None
        self.tick = 0
        self.events: list[dict[str, Any]] = []
        self.counts: Counter[str] = Counter()
        self.motion: dict[str, deque[tuple[float, float, float, float]]] = {}
        self.stalled: set[str] = set()
        self.stall_since: dict[str, float] = {}
        self.response_seen: set[str] = set()
        self.unanswered: set[str] = set()
        self.contact_active: set[tuple[str, str]] = set()
        self.max_contact_force_n = 0.0
        self.max_sinkage_m: dict[str, float] = {}
        self.max_slip_ratio: dict[str, float] = {}
        self.min_peer_center_distance_m: float | None = None
        self.physical_stall_seen = False
        self._closed = False

    def _write(self, record: dict[str, Any]) -> None:
        self.stream.write(json.dumps(record, allow_nan=False, separators=(",", ":")) + "\n")
        self.stream.flush()

    def _event(self, code: str, severity: str, time_s: float,
               actors: list[str], evidence: dict[str, Any], hint: str) -> None:
        event = {
            "kind": "finding", "code": code, "severity": severity,
            "time_s": round(time_s, 4), "actors": actors,
            "evidence": evidence, "investigate": hint,
        }
        self.events.append(event)
        self.counts[code] += 1
        self._write(event)

    def reset(self, seed: int) -> ObservationFrame:
        if self.seed is not None:
            raise RuntimeError("create a new EpisodeDiagnosticPort for each run")
        self.seed = seed
        frame = self.sim.reset(seed)
        self.previous, self.before = frame, self.probe.snapshot()
        self._write({
            "kind": "run", "version": TRACE_VERSION, "episode_id": frame.episode_id,
            "seed": seed, "scenario": self.scenario, "policy": self.policy,
            "scripted_trap": self.scripted_trap,
            "stall_thresholds": {
                "window_s": self.window_s, "commanded_distance_m": self.command_m,
                "actual_distance_m": self.actual_m,
            },
        })
        return frame

    def consume(self, frame: ObservationFrame, decision: Decision) -> None:
        if self.previous is None or frame != self.previous or self.decision is not None:
            raise RuntimeError("consume must match current simulation frame")
        self.decision = decision
        for intent in decision.intents:
            if intent.mode == TaskMode.RESCUE and intent.target_robot_id:
                self.response_seen.add(intent.target_robot_id)

    def step(self, commands: tuple[RobotCommand, ...]) -> ObservationFrame:
        if self.previous is None or self.before is None or self.decision is None:
            raise RuntimeError("call consume(frame, decision) before step")
        if commands != self.decision.commands:
            raise ValueError("diagnostic command mismatch; refusing misleading log")
        next_frame = self.sim.step(commands)
        after = self.probe.snapshot()
        t = next_frame.time_s
        dt = t - self.previous.time_s
        if dt <= 0:
            raise ValueError("non-increasing simulator clock")
        by_cmd = {c.robot_id: c for c in commands}
        by_obs = {r.robot_id: r for r in next_frame.robots}
        deltas: dict[str, float] = {}
        for rid, pos in after["robots"].items():
            prev = self.before["robots"].get(rid)
            if prev is None:
                continue
            traveled = hypot(pos["x_m"] - prev["x_m"], pos["y_m"] - prev["y_m"])
            deltas[rid] = round(traveled, 6)
            cmd = by_cmd.get(rid)
            # Only explicit drive commands count. A rotation or HOLD does not.
            desired = abs(cmd.linear_m_s) * dt if cmd and abs(cmd.linear_m_s) >= 0.08 else 0.0
            window = self.motion.setdefault(rid, deque())
            window.append((t, dt, desired, traveled))
            while window and window[0][0] <= t - self.window_s:
                window.popleft()
            requested = sum(p[2] for p in window)
            actual = sum(p[3] for p in window)
            driven_s = sum(p[1] for p in window if p[2] > 0)
            stalled_now = (driven_s >= 0.8 * self.window_s and
                           requested >= self.command_m and actual <= self.actual_m)
            if stalled_now and rid not in self.stalled:
                self.stalled.add(rid)
                self.stall_since[rid] = t
                self.physical_stall_seen = True
                self._event("commanded_no_progress", "warning", t, [rid], {
                    "commanded_m": round(requested, 4), "actual_path_m": round(actual, 4),
                    "drive_duration_s": round(driven_s, 4), "window_s": self.window_s,
                }, "Inspect traction, contacts, drive actuation and the corresponding planner decision.")
            if rid in self.stalled:
                if not stalled_now and actual > self.actual_m * 2:
                    self.stalled.remove(rid)
                    self.stall_since.pop(rid, None)
                    self.response_seen.discard(rid)
                    self.unanswered.discard(rid)
                    self._event("mobility_resumed", "info", t, [rid],
                                {"actual_path_m": round(actual, 4)},
                                "Correlate regained movement with rescue, terrain and command changes.")
                elif rid not in self.response_seen and rid not in self.unanswered:
                    if t - self.stall_since[rid] >= 1.0:
                        self.unanswered.add(rid)
                        self._event("stall_without_rescue_response", "warning", t, [rid], {
                            "stall_since_s": self.stall_since[rid],
                            "current_task_modes": [
                                i.mode.value for i in self.decision.intents if i.robot_id == rid
                            ],
                        }, "A physical stall was measured, but no RESCUE intent targets this robot. Verify whether hold/defer/abstain was justified.")
            for name, key in (("max_sinkage_m", "sinkage_m"), ("max_slip_ratio", "slip_ratio")):
                value = pos.get(key)
                if value is not None:
                    target = self.max_sinkage_m if name == "max_sinkage_m" else self.max_slip_ratio
                    target[rid] = max(float(value), target.get(rid, 0.0))
            if pos.get("belly_clearance_m") is not None and pos["belly_clearance_m"] <= 0:
                # Soil model says belly-out; this alone does NOT prove movement stopped.
                self.counts["soil_model_belly_out_ticks"] += 1
            if rid in by_obs and by_obs[rid].pose is not None:
                drift = hypot(pos["x_m"] - by_obs[rid].pose.x_m,
                              pos["y_m"] - by_obs[rid].pose.y_m)
                if drift > 0.12:
                    self._event("large_pose_discrepancy", "warning", t, [rid], {
                        "truth_vs_observation_m": round(drift, 4),
                    }, "Check observation update timing and localization error.")
        robots = after["robots"]
        if len(robots) >= 2:
            ids = sorted(robots)
            for i, rid in enumerate(ids):
                for peer in ids[i + 1:]:
                    distance = hypot(robots[rid]["x_m"] - robots[peer]["x_m"],
                                     robots[rid]["y_m"] - robots[peer]["y_m"])
                    if self.min_peer_center_distance_m is None:
                        self.min_peer_center_distance_m = distance
                    else:
                        self.min_peer_center_distance_m = min(self.min_peer_center_distance_m, distance)
        contacts_now: set[tuple[str, str]] = set()
        for c in after.get("contacts", []):
            pair = tuple(c["actors"])
            new_contact = pair not in self.contact_active and pair not in contacts_now
            self.max_contact_force_n = max(self.max_contact_force_n, c["normal_force_n"])
            if "environment" not in pair:
                contacts_now.add(pair)
                intended = (
                    any(cmd.recovery_target_id in pair and cmd.robot_id in pair
                        for cmd in commands)
                    or set(after.get("latched_pair") or ()) == set(pair)
                )
                if new_contact:
                    self._event(
                        "robot_contact_during_rescue" if intended else "unexpected_robot_collision",
                        "info" if intended else "error", t, list(pair),
                        {"geoms": c["geoms"], "penetration_m": c["penetration_m"],
                         "normal_force_n": c["normal_force_n"], "rescue_command": intended},
                        "Inspect contact mechanics and separation after docking; force and penetration are sampled only at tick boundaries."
                    )
            elif c["normal_force_n"] >= 100.0:
                key = ("environment", pair[1] if pair[0] == "environment" else pair[0])
                contacts_now.add(key)
                if new_contact:
                    self._event("strong_environment_contact", "warning", t, list(pair),
                                {"geoms": c["geoms"], "normal_force_n": c["normal_force_n"]},
                                "Inspect obstacle geometry, impact velocity and wheel/robot clearance.")
        self.contact_active = contacts_now
        self.tick += 1
        self._write({
            "kind": "tick", "tick": self.tick, "time_s": t,
            "observation": asdict(next_frame),
            "intents": [asdict(i) for i in self.decision.intents],
            "commands": [asdict(c) for c in commands],
            "truth": after, "step_distance_m": deltas,
        })
        self.previous, self.before, self.decision = next_frame, after, None
        return next_frame

    def finish(self) -> dict[str, Any]:
        if self._closed:
            raise RuntimeError("diagnostics already finalized")
        self._closed = True
        if self.tick == 0:
            self._event("no_steps", "warning", 0.0, [], {},
                        "Run at least one physics step to produce diagnostic evidence.")
        if not self.scripted_trap and not self.physical_stall_seen:
            self._event("no_natural_immobilization_observed", "info",
                        self.previous.time_s if self.previous else 0.0, [],
                        {"ticks": self.tick},
                        "The episode never demonstrated a physical stall; this does not prove natural failures exist.")
        if self.scripted_trap and not self.physical_stall_seen:
            self._event("scripted_trap_without_independent_stall", "warning",
                        self.previous.time_s if self.previous else 0.0, [],
                        {"ticks": self.tick},
                        "A configured trap alone is not evidence of physical immobilization. Inspect drive-versus-motion traces.")
        summary = {
            "version": TRACE_VERSION, "seed": self.seed,
            "scenario": self.scenario, "policy": self.policy,
            "scripted_trap": self.scripted_trap, "ticks": self.tick,
            "trace_jsonl": str(self.trace_path),
            "event_counts": dict(self.counts),
            "findings": self.events,
            "max_sampled_contact_force_n": round(self.max_contact_force_n, 3),
            "min_peer_center_distance_m": (
                round(self.min_peer_center_distance_m, 4)
                if self.min_peer_center_distance_m is not None else None
            ),
            "max_model_sinkage_m": self.max_sinkage_m,
            "max_model_slip_ratio": self.max_slip_ratio,
            "independent_immobilization_observed": self.physical_stall_seen,
            "limitations": [
                "Contacts sampled only at physics tick boundaries, not every substep.",
                "Commanded no-progress is a suspicion; terrain, obstacles, safe stops and towing require contextual review.",
                "Analytical sinkage/slip are simulated soil-model outputs, not real experimental measurements.",
                "No finding or absence of findings certifies a successful rescue or natural failure."
            ],
        }
        self._write({"kind": "summary", "summary": summary})
        self.stream.close()
        return summary
