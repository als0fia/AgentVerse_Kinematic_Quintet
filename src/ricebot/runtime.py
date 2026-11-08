"""Composition root with a narrow simulator port and authoritative EpisodeSession. No fake Isaac integration."""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Protocol

from .contracts import (
    Decision,
    EpisodeTrace,
    FaultEvidence,
    ObservationFrame,
    OperatorCommandKind,
    OperatorRequest,
    RescueAssessment,
    RobotCapability,
    RobotCommand,
    TaskMode,
    TickTrace,
)
from .control import ControlLimits, LocalController
from .mission import MeasuredFieldMap, SimpleMissionPlanner


class SimulationPort(Protocol):
    """Isaac/MuJoCo adapter implements this, owning all articulation/contact physics."""

    def reset(self, seed: int) -> ObservationFrame: ...
    def step(self, commands: tuple[RobotCommand, ...]) -> ObservationFrame: ...


class PolicyRuntime(Protocol):
    """Single-decision-per-tick policy interface implemented by MissionCore and DecentralizedMycelialRuntime."""

    def decide(self, frame: ObservationFrame) -> Decision: ...


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
            commands.append(cmd)
        # Optional planner feedback: only actual issued motor commands can
        # substantiate a later commanded-motion stall; a task intent cannot.
        record_commands = getattr(self.planner, "record_commands", None)
        if record_commands is not None:
            record_commands(tuple(commands))
        return Decision(intents, tuple(commands), accepted)


@dataclass(frozen=True)
class EpisodeConfig:
    """Canonical episode configuration shared across CLI, 3D viewer, HTML recorder, and live GUI."""

    scenario: str = "nominal"
    policy: str = "mycelial"
    rescue_policy: str = "react_orchestrator"
    comms: str = "normal"
    seed: int = 7
    steps: int = 140
    single_rover_only: bool = False
    probe_radius_m: float = 0.28
    substeps_per_tick: int = 25
    obstacle_xy: tuple[float, float] = (0.75, -0.25)
    trap_robot_id: str | None = None
    trap_site_id: str = "patch_d"
    trap_radius_m: float = 0.28
    sensor_dropout_robot_id: str | None = None
    sensor_dropout_window_s: tuple[float, float] | None = None
    expose_bogged_flag: bool = True
    comm_range_m: float = 2.15
    include_shadow_mycelial: bool = False

    def __post_init__(self) -> None:
        if self.seed < 0 or isinstance(self.seed, bool) or self.steps < 1:
            raise ValueError("seed must be non-negative int and steps >= 1")
        if self.scenario in {"golden_rescue", "high_risk_abstain"} and self.steps == 140:
            object.__setattr__(self, "steps", 195)
        elif self.scenario == "sensor_degraded" and self.steps == 140:
            object.__setattr__(self, "steps", 165)

        if self.scenario == "high_risk_abstain" and self.trap_site_id == "patch_d":
            object.__setattr__(self, "trap_site_id", "patch_a")

        if self.trap_robot_id is None:
            if self.scenario == "golden_rescue":
                object.__setattr__(self, "trap_robot_id", "planter")
            elif self.scenario == "high_risk_abstain":
                object.__setattr__(self, "trap_robot_id", "scout")

        if self.scenario == "sensor_degraded":
            if self.sensor_dropout_robot_id is None:
                object.__setattr__(self, "sensor_dropout_robot_id", "planter")
            if self.sensor_dropout_window_s is None:
                object.__setattr__(self, "sensor_dropout_window_s", (1.2, 2.4))

    @property
    def is_golden_rescue(self) -> bool:
        return self.scenario == "golden_rescue"

    @property
    def is_rescue_scenario(self) -> bool:
        return self.scenario in {"golden_rescue", "high_risk_abstain"} or self.trap_robot_id is not None

    @property
    def effective_policy_label(self) -> str:
        if self.is_golden_rescue or self.scenario == "high_risk_abstain":
            return "cooperative_swarm_rescue"
        if self.single_rover_only or self.policy == "single_rover":
            return "single_rover"
        return self.policy


class EpisodeSession:
    """Single authoritative owner of episode identity, clock, reset, policy decision, physics step, and trace.

    Enforces Invariant 1 from `docs/FIVE-AGENT-RECONCILIATION.md`:
    - Exactly ONE authoritative `policy.decide(frame)` and ONE `sim.step(decision.commands)` per tick.
    - Identical execution order (`reset` -> per-tick `[decide -> notify views -> step]`) whether run
      in headless batch mode (`run(seed, steps)`) or stepped interactively (`step_once()`).
    - Optional `shadow_policy` is strictly a passive, non-acting observer: its commands are discarded
      and never reach `sim.step` or `ReadOnlyView` subscribers.
    - Operator interventions (`OperatorRequest`) enter a validated queue and are logged on `TickTrace`.
    """

    def __init__(
        self,
        sim: SimulationPort,
        policy: PolicyRuntime,
        *,
        views: tuple[ReadOnlyView, ...] | list[ReadOnlyView] = (),
        shadow_policy: PolicyRuntime | None = None,
        seed: int = 7,
        scenario: str = "nominal",
        policy_name: str = "mycelial",
        rescue_policy: str = "react_orchestrator",
        comms_mode: str = "normal",
        operator_handler: Callable[[SimulationPort, PolicyRuntime, OperatorRequest], None] | None = None,
    ) -> None:
        if seed < 0 or isinstance(seed, bool):
            raise ValueError("seed must be a non-negative integer")
        self.sim = sim
        self.policy = policy
        self.views: tuple[ReadOnlyView, ...] = tuple(views)
        self.shadow_policy = shadow_policy
        self.seed = seed
        self.scenario = scenario
        self.policy_name = policy_name
        self.rescue_policy = rescue_policy
        self.comms_mode = comms_mode
        self.operator_handler = operator_handler

        self.current_frame: ObservationFrame | None = None
        self.last_decision: Decision | None = None
        self.tick_index: int = 0
        self.paused: bool = False
        self._ticks: list[TickTrace] = []
        self._pending_operator_requests: list[OperatorRequest] = []
        self.applied_operator_requests: list[OperatorRequest] = []
        self.rescue_assessments: dict[str, RescueAssessment] = {}
        self.fault_evidence: dict[str, FaultEvidence] = {}
        self.orchestrator_decisions: dict[str, Any] = {}
        self.recovery_monitoring: list[dict[str, Any]] = []
        self.teleop_overrides: dict[str, tuple[float, float]] = {}

    def submit_operator_request(self, request: OperatorRequest) -> None:
        """Queue a typed operator request to be applied at the session boundary (never mid-substep)."""
        if not isinstance(request, OperatorRequest):
            raise TypeError("operator intervention must be a typed OperatorRequest")
        if request.kind == OperatorCommandKind.PAUSE:
            self.paused = True
            self.applied_operator_requests.append(request)
            return
        if request.kind == OperatorCommandKind.RESUME:
            self.paused = False
            self.applied_operator_requests.append(request)
            return
        self._pending_operator_requests.append(request)

    def reset(self, seed: int | None = None) -> ObservationFrame:
        """Reset simulation and session trace state for a deterministic episode."""
        if seed is not None:
            if seed < 0 or isinstance(seed, bool):
                raise ValueError("seed must be a non-negative integer")
            self.seed = seed
        self.current_frame = self.sim.reset(self.seed)
        self.last_decision = None
        self.tick_index = 0
        self.paused = False
        self._ticks.clear()
        self._pending_operator_requests.clear()
        self.applied_operator_requests.clear()
        self.rescue_assessments.clear()
        self.fault_evidence.clear()
        self.orchestrator_decisions.clear()
        self.recovery_monitoring.clear()
        self.teleop_overrides.clear()
        return self.current_frame

    def _collect_normalized_rescue_assessments(self) -> dict[str, RescueAssessment]:
        """Extract and normalize RescueAssessment objects from policy or policy.planner."""
        raw = getattr(self.policy, "last_rescue_assessments", None)
        if raw is None and hasattr(self.policy, "planner"):
            raw = getattr(getattr(self.policy, "planner"), "last_rescue_assessments", None)
        if not raw or not isinstance(raw, dict):
            return {}
        return {str(k): RescueAssessment.from_any(v) for k, v in raw.items()}

    def _collect_fault_evidence(self) -> dict[str, FaultEvidence]:
        """Extract sensed FaultEvidence objects from policy or policy.planner."""
        raw = getattr(self.policy, "last_fault_evidence", None)
        if raw is None and hasattr(self.policy, "planner"):
            raw = getattr(getattr(self.policy, "planner"), "last_fault_evidence", None)
        if not raw or not isinstance(raw, dict):
            return {}
        return {str(k): v for k, v in raw.items() if isinstance(v, FaultEvidence)}

    def _collect_orchestrator_decisions(self) -> dict[str, Any]:
        """Extract validated OrchestratorDecision objects from policy or policy.planner."""
        raw = getattr(self.policy, "last_orchestrator_decisions", None)
        if raw is None and hasattr(self.policy, "planner"):
            raw = getattr(getattr(self.policy, "planner"), "last_orchestrator_decisions", None)
        if not raw or not isinstance(raw, dict):
            return {}
        return dict(raw)

    def _collect_recovery_monitoring(self) -> list[dict[str, Any]]:
        """Extract closed-loop physical recovery monitoring records from policy or policy.planner."""
        raw = getattr(self.policy, "recovery_monitoring_history", None)
        if raw is None and hasattr(self.policy, "planner"):
            raw = getattr(getattr(self.policy, "planner"), "recovery_monitoring_history", None)
        if not raw or not isinstance(raw, list):
            return []
        return list(raw)

    def step_once(self) -> TickTrace:
        """Advance the episode by exactly one policy decision and one simulation physics step."""
        if self.current_frame is None:
            self.reset(self.seed)
        assert self.current_frame is not None

        applied_this_tick: list[OperatorRequest] = []
        while self._pending_operator_requests:
            req = self._pending_operator_requests.pop(0)
            if req.kind == OperatorCommandKind.TELEOP_OVERRIDE:
                params = dict(req.payload)
                target_rid = str(params.get("robot_id", "planter"))
                if params.get("active", False):
                    lin = max(-0.5, min(0.5, float(params.get("linear_m_s", 0.0))))
                    ang = max(-2.0, min(2.0, float(params.get("angular_rad_s", 0.0))))
                    self.teleop_overrides[target_rid] = (lin, ang)
                else:
                    self.teleop_overrides.pop(target_rid, None)
            elif self.operator_handler is not None:
                self.operator_handler(self.sim, self.policy, req)
            applied_this_tick.append(req)
            self.applied_operator_requests.append(req)

        frame_before = self.current_frame
        decision = self.policy.decide(frame_before)
        if self.teleop_overrides:
            overridden_cmds: list[RobotCommand] = []
            for cmd in decision.commands:
                if cmd.robot_id in self.teleop_overrides:
                    lin, ang = self.teleop_overrides[cmd.robot_id]
                    overridden_cmds.append(
                        RobotCommand(
                            robot_id=cmd.robot_id,
                            issued_at_s=frame_before.time_s,
                            expires_at_s=frame_before.time_s + 0.25,
                            linear_m_s=lin,
                            angular_rad_s=ang,
                            task_id="teleop:override",
                        )
                    )
                else:
                    overridden_cmds.append(cmd)
            decision = Decision(decision.intents, tuple(overridden_cmds), decision.ingested_samples)
        self.last_decision = decision

        if self.shadow_policy is not None and self.shadow_policy is not self.policy:
            # Passive shadow observer updates its internal belief state for comparison only;
            # its returned Decision is intentionally discarded and never actuates physics.
            self.shadow_policy.decide(frame_before)

        self.rescue_assessments = self._collect_normalized_rescue_assessments()
        self.fault_evidence = self._collect_fault_evidence()
        self.orchestrator_decisions = self._collect_orchestrator_decisions()
        self.recovery_monitoring = self._collect_recovery_monitoring()

        for view in self.views:
            view.consume(frame_before, decision)

        next_frame = self.sim.step(decision.commands)
        self.current_frame = next_frame

        trace_entry = TickTrace(
            tick_index=self.tick_index,
            time_s=frame_before.time_s,
            frame=frame_before,
            decision=decision,
            next_time_s=next_frame.time_s,
            operator_requests=tuple(applied_this_tick),
        )
        self._ticks.append(trace_entry)
        self.tick_index += 1
        return trace_entry

    def run(self, seed: int | None = None, steps: int = 1) -> list[Decision]:
        """Run `steps` authoritative ticks from a fresh `reset(seed)` and return all decisions."""
        if steps < 1:
            raise ValueError("run needs at least one step")
        self.reset(seed if seed is not None else self.seed)
        decisions: list[Decision] = []
        for _ in range(steps):
            tick = self.step_once()
            decisions.append(tick.decision)
        return decisions

    @property
    def ticks(self) -> tuple[TickTrace, ...]:
        return tuple(self._ticks)

    def export_trace(self) -> EpisodeTrace:
        """Export an immutable EpisodeTrace for parity checks, grading, and deterministic replay."""
        episode_id = (
            self._ticks[0].frame.episode_id
            if self._ticks
            else (self.current_frame.episode_id if self.current_frame is not None else f"episode-{self.seed}")
        )
        return EpisodeTrace(
            episode_id=episode_id,
            seed=self.seed,
            scenario=self.scenario,
            policy=self.policy_name,
            comms_mode=self.comms_mode,
            ticks=tuple(self._ticks),
        )


class ReplaySimulationPort:
    """Deterministic `SimulationPort` backed by a recorded `EpisodeTrace` for offline replay and testing."""

    def __init__(self, trace: EpisodeTrace, *, verify_commands: bool = True) -> None:
        if not trace.ticks:
            raise ValueError("ReplaySimulationPort requires a non-empty EpisodeTrace")
        self.trace = trace
        self.verify_commands = verify_commands
        self._cursor: int = 0

    def reset(self, seed: int) -> ObservationFrame:
        if seed != self.trace.seed:
            raise ValueError(f"replay seed {seed} does not match recorded trace seed {self.trace.seed}")
        self._cursor = 0
        return self.trace.ticks[0].frame

    def step(self, commands: tuple[RobotCommand, ...]) -> ObservationFrame:
        if self._cursor >= len(self.trace.ticks):
            raise IndexError("replay exhausted recorded ticks")
        expected = self.trace.ticks[self._cursor].decision.commands
        if self.verify_commands and commands != expected:
            raise ValueError("replay command mismatch against recorded TickTrace")
        self._cursor += 1
        if self._cursor < len(self.trace.ticks):
            return self.trace.ticks[self._cursor].frame
        last_tick = self.trace.ticks[-1]
        return ObservationFrame(
            episode_id=last_tick.frame.episode_id,
            time_s=last_tick.next_time_s,
            robots=tuple(
                type(r)(
                    robot_id=r.robot_id,
                    pose=r.pose,
                    time_s=last_tick.next_time_s,
                    front_range_m=r.front_range_m,
                    seeds_remaining=r.seeds_remaining,
                    samples=(),
                    link_ok=r.link_ok,
                    pitch_rad=r.pitch_rad,
                    roll_rad=r.roll_rad,
                    true_speed_m_s=r.true_speed_m_s,
                    row_cross_track_m=r.row_cross_track_m,
                    peer_distances=r.peer_distances,
                    is_bogged=r.is_bogged,
                )
                for r in last_tick.frame.robots
            ),
        )


def _default_mujoco_operator_handler(
    sim: SimulationPort,
    policy: PolicyRuntime,
    request: OperatorRequest,
) -> None:
    """Apply logged operator experiment requests at the session tick boundary."""
    params = dict(request.payload)
    if request.kind == OperatorCommandKind.SET_COMMS_MODE:
        mode = str(params.get("comm_mode", "normal"))
        if hasattr(policy, "comm_mode"):
            setattr(policy, "comm_mode", mode)
    elif request.kind == OperatorCommandKind.INJECT_OBSTACLE:
        x_m = float(params.get("x_m", 1.25))
        y_m = float(params.get("y_m", -0.50))
        inject_fn = getattr(sim, "inject_dynamic_obstacle", None)
        if callable(inject_fn):
            inject_fn(x_m, y_m)
    elif request.kind == OperatorCommandKind.INJECT_MUD_FAULT:
        robot_id = str(params.get("robot_id", "planter"))
        inject_mud = getattr(sim, "inject_mud_fault", None)
        if callable(inject_mud):
            inject_mud(robot_id)
    elif request.kind == OperatorCommandKind.EXPERIMENT_VEHICLE_SIZING:
        target_robot = str(params.get("robot_id", "all"))
        targets = ("planter", "scout") if target_robot == "all" else (target_robot,)
        for rid in targets:
            if "mass_kg" in params and hasattr(sim, "robot_mass_kg"):
                getattr(sim, "robot_mass_kg")[rid] = float(params["mass_kg"])
            if "wheel_radius_m" in params and hasattr(sim, "robot_wheel_radius_m"):
                getattr(sim, "robot_wheel_radius_m")[rid] = float(params["wheel_radius_m"])
            if "wheel_width_m" in params and hasattr(sim, "robot_wheel_width_m"):
                getattr(sim, "robot_wheel_width_m")[rid] = float(params["wheel_width_m"])


def create_mujoco_episode_session(
    config: EpisodeConfig | None = None,
    *,
    views: tuple[ReadOnlyView, ...] | list[ReadOnlyView] = (),
    advisor: Any = None,
) -> EpisodeSession:
    """Canonical factory constructing one authoritative MuJoCo SimulationPort + PolicyRuntime session."""
    from .agent import CooperativeSwarmPlanner, DecentralizedMycelialRuntime
    from .mujoco_sim.adapter import DEFAULT_CAPABILITIES, DEFAULT_SITES, MujocoPaddySimulation
    from .world import generate_hidden_soil_truth

    cfg = config or EpisodeConfig()
    is_single = cfg.single_rover_only or cfg.policy == "single_rover"
    inject_at = 1.6 if cfg.scenario == "obstacle" else None
    truth = generate_hidden_soil_truth(cfg.scenario, cfg.seed)
    sim = MujocoPaddySimulation(
        sites=DEFAULT_SITES,
        probe_radius_m=cfg.probe_radius_m,
        substeps_per_tick=cfg.substeps_per_tick,
        inject_obstacle_at_s=inject_at,
        obstacle_xy=cfg.obstacle_xy,
        hidden_truth=truth,
        trap_robot_id=cfg.trap_robot_id,
        trap_site_id=cfg.trap_site_id,
        trap_radius_m=cfg.trap_radius_m,
        sensor_dropout_robot_id=cfg.sensor_dropout_robot_id,
        sensor_dropout_window_s=cfg.sensor_dropout_window_s,
        expose_bogged_flag=cfg.expose_bogged_flag,
    )
    caps = (
        (
            RobotCapability("planter", can_sample=True, can_plant=True),
            RobotCapability("scout", can_sample=False, can_plant=False),
        )
        if is_single
        else DEFAULT_CAPABILITIES
    )

    if cfg.is_rescue_scenario or cfg.policy == "cooperative":
        field_map = MeasuredFieldMap(DEFAULT_SITES, sample_radius_m=0.6)
        planner: SimpleMissionPlanner = CooperativeSwarmPlanner(
            caps,
            advisor=advisor,
            rescue_policy=cfg.rescue_policy,
        )
        controller = LocalController(
            ControlLimits(speed_m_s=0.38, turn_rad_s=1.15, stop_range_m=0.32, site_reach_m=0.22)
        )
        policy_runtime: PolicyRuntime = MissionCore(field_map, planner, controller)
    elif cfg.policy == "mycelial" and not is_single:
        policy_runtime = DecentralizedMycelialRuntime(
            sites=DEFAULT_SITES,
            capabilities=caps,
            comm_range_m=cfg.comm_range_m,
            comm_mode=cfg.comms,
            advisor=advisor,
            rescue_policy=cfg.rescue_policy,
        )
    else:
        field_map = MeasuredFieldMap(DEFAULT_SITES, sample_radius_m=0.6)
        planner = SimpleMissionPlanner(caps)
        controller = LocalController(
            ControlLimits(speed_m_s=0.38, turn_rad_s=1.15, stop_range_m=0.32, site_reach_m=0.22)
        )
        policy_runtime = MissionCore(field_map, planner, controller)

    shadow_policy: PolicyRuntime | None = None
    if cfg.include_shadow_mycelial and not isinstance(policy_runtime, DecentralizedMycelialRuntime):
        shadow_policy = DecentralizedMycelialRuntime(
            sites=DEFAULT_SITES,
            capabilities=caps,
            comm_range_m=0.0 if is_single else 99.0,
            comm_mode="isolated" if is_single else "normal",
            advisor=advisor,
            rescue_policy=cfg.rescue_policy,
        )

    return EpisodeSession(
        sim=sim,
        policy=policy_runtime,
        views=views,
        shadow_policy=shadow_policy,
        seed=cfg.seed,
        scenario=cfg.scenario,
        policy_name=cfg.effective_policy_label,
        rescue_policy=cfg.rescue_policy,
        comms_mode=cfg.comms,
        operator_handler=_default_mujoco_operator_handler,
    )


class EpisodeRunner:
    """A thin integration point wrapping EpisodeSession; tests may use a scripted port, not physics proof."""

    def __init__(self, sim: SimulationPort, core: PolicyRuntime, view: ReadOnlyView | None = None):
        self.sim, self.core, self.view = sim, core, view
        self.session = EpisodeSession(
            sim=sim,
            policy=core,
            views=(view,) if view is not None else (),
        )

    def run(self, seed: int, steps: int) -> list[Decision]:
        if steps < 1:
            raise ValueError("run needs at least one step")
        return self.session.run(seed=seed, steps=steps)
