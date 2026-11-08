"""Composition root with a narrow simulator port. No fake Isaac integration."""
from __future__ import annotations

from typing import Protocol

from .contracts import Decision, ObservationFrame, RobotCommand, TaskMode
from .control import LocalController
from .mission import MeasuredFieldMap, SimpleMissionPlanner


class SimulationPort(Protocol):
    """Isaac/MuJoCo adapter implements this, owning all articulation/contact physics."""

    def reset(self, seed: int) -> ObservationFrame: ...
    def step(self, commands: tuple[RobotCommand, ...]) -> ObservationFrame: ...


class ReadOnlyView(Protocol):
    """Optional subscriber; cannot issue motor commands or change simulation."""

    def consume(self, frame: ObservationFrame, decision: Decision) -> None: ...


class MissionCore:
    """Testable single tick: sensed frame -> map -> tasks -> bounded commands."""

    def __init__(self, field_map: MeasuredFieldMap, planner: SimpleMissionPlanner,
                 controller: LocalController | None = None):
        self.field_map = field_map
        self.planner = planner
        self.controller = controller or LocalController()
        self._last_time: float | None = None
        self._last_episode: str | None = None
        self._pending_plant: dict[str, tuple[str, str, float]] = {}  # robot -> (site, task_id, sent_at)
        self._attempted_plant_sites: set[str] = set()

    def decide(self, frame: ObservationFrame) -> Decision:
        if self._last_episode is None:
            self._last_episode = frame.episode_id
        if frame.episode_id != self._last_episode:
            raise ValueError("new episode needs a fresh MissionCore/reset map")
        if self._last_time is not None and frame.time_s <= self._last_time:
            raise ValueError("non-increasing frame time")
        self._last_time = frame.time_s
        observations = {r.robot_id: r for r in frame.robots}
        if any(robot_id not in self.planner.capabilities for robot_id in observations):
            raise ValueError("observation from unknown actor")
        for feedback in frame.plant_feedback:
            pending = self._pending_plant.get(feedback.robot_id)
            if pending and pending[1] == feedback.task_id:
                self._pending_plant.pop(feedback.robot_id)
                # No automatic repeated drop, even on a jam or missing landing.
                # Future recovery policy can explicitly inspect/authorise retry.
                self._attempted_plant_sites.add(pending[0])
        for robot_id, (site_id, _, sent_at_s) in tuple(self._pending_plant.items()):
            if frame.time_s > sent_at_s + 2.0:
                # Missing acknowledgement: safe abstention instead of indefinite wait/retry.
                self._pending_plant.pop(robot_id)
                self._attempted_plant_sites.add(site_id)
        accepted = self.field_map.update(frame)
        excluded = frozenset(self._attempted_plant_sites | {site for site, _, _ in self._pending_plant.values()})
        intents = self.planner.decide(frame, self.field_map, excluded)
        mech_setpoints = getattr(self.planner, "active_mechanical_setpoints", {})
        commands = []
        for task in intents:
            peer_pose = (
                observations[task.target_robot_id].pose
                if task.mode == TaskMode.RESCUE and task.target_robot_id in observations
                else None
            )
            cmd = self.controller.command(
                observations[task.robot_id],
                task,
                self.field_map.sites,
                self.planner.capabilities[task.robot_id],
                peer_pose=peer_pose,
            )
            if task.robot_id in self._pending_plant:
                cmd = RobotCommand(task.robot_id, frame.time_s, frame.time_s + 0.2, task_id=task.task_id)
            elif cmd.release_seed:
                self._pending_plant[task.robot_id] = (cmd.target_site_id, cmd.task_id, frame.time_s)
            setpoint = mech_setpoints.get(task.robot_id)
            if setpoint is not None and task.mode in (TaskMode.HOLD, TaskMode.RESCUE):
                spade_rad = max(0.0, min(0.85, float(setpoint.spade_rad))) if task.mode == TaskMode.HOLD else cmd.spade_rad
                pitch_rad = max(-0.20, min(0.15, float(setpoint.dock_pitch_rad))) if task.mode == TaskMode.RESCUE else cmd.dock_pitch_rad
                latch_rad = max(0.0, min(0.65, float(setpoint.latch_rad))) if task.mode == TaskMode.RESCUE else cmd.latch_rad
                cmd = RobotCommand(
                    robot_id=cmd.robot_id,
                    issued_at_s=cmd.issued_at_s,
                    expires_at_s=cmd.expires_at_s,
                    linear_m_s=cmd.linear_m_s,
                    angular_rad_s=cmd.angular_rad_s,
                    release_seed=cmd.release_seed,
                    target_site_id=cmd.target_site_id,
                    task_id=cmd.task_id,
                    recovery_target_id=cmd.recovery_target_id,
                    dock_pitch_rad=pitch_rad,
                    latch_rad=latch_rad,
                    spade_rad=spade_rad,
                )
            commands.append(cmd)
        # Optional planner feedback: only actual issued motor commands can
        # substantiate a later commanded-motion stall; a task intent cannot.
        record_commands = getattr(self.planner, "record_commands", None)
        if record_commands is not None:
            record_commands(tuple(commands))
        return Decision(intents, tuple(commands), accepted)


class EpisodeRunner:
    """A thin integration point; tests may use a scripted port, not physics proof."""

    def __init__(self, sim: SimulationPort, core: MissionCore, view: ReadOnlyView | None = None):
        self.sim, self.core, self.view = sim, core, view

    def run(self, seed: int, steps: int) -> list[Decision]:
        if steps < 1:
            raise ValueError("run needs at least one step")
        frame = self.sim.reset(seed)
        decisions: list[Decision] = []
        for _ in range(steps):
            decision = self.core.decide(frame)
            if self.view is not None:
                self.view.consume(frame, decision)
            decisions.append(decision)
            frame = self.sim.step(decision.commands)
        return decisions
