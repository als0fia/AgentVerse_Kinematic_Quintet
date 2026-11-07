"""Composition root with a narrow simulator port. No fake Isaac integration."""
from __future__ import annotations

from typing import Protocol

from .contracts import (Decision, ObservationFrame, RobotCommand)
from .control import LocalController
from .mission import MeasuredFieldMap, SimpleMissionPlanner


class SimulationPort(Protocol):
    """Isaac adapter implements this, owning all articulation/contact physics."""

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
        commands = []
        for task in intents:
            cmd = self.controller.command(observations[task.robot_id], task,
                           self.field_map.sites, self.planner.capabilities[task.robot_id])
            if task.robot_id in self._pending_plant:
                cmd = RobotCommand(task.robot_id, frame.time_s, frame.time_s + 0.2, task_id=task.task_id)
            elif cmd.release_seed:
                self._pending_plant[task.robot_id] = (cmd.target_site_id, cmd.task_id, frame.time_s)
            commands.append(cmd)
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
