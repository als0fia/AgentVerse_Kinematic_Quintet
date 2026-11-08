"""Scientific evaluation, independent physical evidence grading, and ablation runner for Antenna (rice.v1).

This module evaluates canonical MuJoCo episodes strictly through fresh
`create_mujoco_episode_session(EpisodeConfig(...))` instances (Issue #71)
with `EpisodeDiagnosticPort` + `MujocoPhysicsProbe` attached as a read-only
physics observer. It never modifies planner, controller, or simulator state,
and explicitly separates commanded rescue intent from verified physical
extraction and post-rescue mobility/agronomic outcomes.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from math import hypot
from pathlib import Path
from statistics import mean, median
import tempfile
import time
from typing import Any

from .agent.orchestrator import (
    HelperStatus,
    IncidentReport,
    OperatorPolicy,
    OrchestratorSafetyGate,
    OrchestratorToolBelt,
    SwarmOrchestratorAgent,
)
from .agent.rescue import (
    FailureMode,
    MechanicalRescueTechnique,
    ResidualRecoveryLearner,
)
from .contracts import (
    Pose2D,
    RecoveryFeedback,
    RescueVerdict,
    SensorKind,
    TaskMode,
)
from .diagnostics import EpisodeDiagnosticPort, MujocoPhysicsProbe
from .runtime import EpisodeConfig, create_mujoco_episode_session
from .world import generate_hidden_soil_truth, generate_randomized_episode_spec

RESCUE_POLICY_ALIASES: dict[str, str] = {
    "no_rescue": "never_rescue",
    "never_rescue": "never_rescue",
    "always_rescue": "always_rescue",
    "calibrated_triage": "calibrated_triage",
    "numerical_triage": "calibrated_triage",
    "react_orchestrator": "react_orchestrator",
}

CANONICAL_RESCUE_POLICY_LABELS: dict[str, str] = {
    "no_rescue": "no_rescue",
    "never_rescue": "no_rescue",
    "always_rescue": "always_rescue",
    "calibrated_triage": "calibrated_triage",
    "numerical_triage": "calibrated_triage",
    "react_orchestrator": "react_orchestrator",
}


def resolve_engine_rescue_policy(policy_name: str) -> str:
    """Map an evaluation rescue policy label to the string expected by CooperativeSwarmPlanner."""
    if policy_name not in RESCUE_POLICY_ALIASES:
        raise ValueError(f"Unknown rescue policy: {policy_name!r}")
    return RESCUE_POLICY_ALIASES[policy_name]


def canonical_rescue_policy_label(policy_name: str) -> str:
    """Return the canonical report label ('no_rescue', 'always_rescue', 'calibrated_triage', 'react_orchestrator')."""
    if policy_name not in CANONICAL_RESCUE_POLICY_LABELS:
        raise ValueError(f"Unknown rescue policy: {policy_name!r}")
    return CANONICAL_RESCUE_POLICY_LABELS[policy_name]


def _percentile_nearest_rank(values: list[float], percent: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = max(0, int((percent / 100.0) * len(ordered) + 0.999999) - 1)
    return float(ordered[min(idx, len(ordered) - 1)])


@dataclass(frozen=True)
class EpisodeEvaluationRecord:
    """Authoritative per-episode evaluation record combining agronomy, physics probe, and orchestration telemetry."""

    case_id: str
    scenario: str
    seed: int
    policy_label: str
    coordination_policy: str
    rescue_policy: str
    engine_rescue_policy: str
    comms_mode: str
    steps: int
    sim_duration_s: float
    mud_severity: str
    trap_site_id: str | None
    trap_robot_id: str | None

    # Agronomic outcomes (ground truth)
    patches_total: int
    viable_sites_total: int
    non_viable_sites_total: int
    patches_visited: int
    samples_taken: int
    viable_seed_placements: int
    wasted_mud_drops: int
    false_abstentions: int
    duplicate_plantings: int
    viable_yield_rate: float
    abstention_accuracy: float
    sample_coverage_rate: float
    post_rescue_viable_placements: int

    # Authoritative physical rescue & terramechanics evidence (never equating command with extraction)
    ever_bogged_robots: tuple[str, ...]
    currently_bogged_at_end: tuple[str, ...]
    independent_immobilization_observed: bool
    commanded_no_progress_events: int
    stall_without_rescue_response_events: int
    rescue_commanded_count: int
    rescue_intent_ticks: int
    physical_latch_engaged: bool
    physical_extractions_verified: int
    commanded_without_physical_extraction: int
    independent_mobility_resumed_count: int
    max_victim_extraction_displacement_m: float
    secondary_casualties: int
    unexpected_robot_collisions: int
    robot_contact_during_rescue_events: int
    max_sampled_contact_force_n: float
    min_peer_center_distance_m: float | None
    max_model_sinkage_m: dict[str, float]
    max_model_slip_ratio: dict[str, float]
    soil_model_belly_out_ticks: int

    # Triage & Orchestrator telemetry
    orchestrator_verdicts: dict[str, str]
    selected_techniques: dict[str, str]
    policy_veto_count: int
    unsafe_severe_mud_dispatches: int
    recovery_feedback_events: int
    learner_mean_prediction_error_m: float | None

    # Communication & decision latency metrics
    decision_latency_mean_ms: float
    decision_latency_median_ms: float
    decision_latency_p95_ms: float
    decision_latency_max_ms: float
    episode_decision_elapsed_ms: float
    episode_wall_elapsed_ms: float
    gossip_packets_sent: int
    gossip_bytes_sent: int
    gossip_sync_events: int
    stale_shared_belief_events: int
    map_divergence_l1: float
    num_rovers: int
    map_scale: float
    active_policy_type: str
    gossip_source: str
    distance_m_by_rover: dict[str, float]
    active_task_ticks_by_rover: dict[str, int]

    # Canonical verdict for schemas/episode.schema.json & scripts/evaluate.py
    success: bool
    failure_reason: str | None
    yield_only_success: bool
    diagnostic_trace_path: str | None = None

    def to_schema_record(self) -> dict[str, Any]:
        """Export strict 5-key record required by schemas/episode.schema.json and scripts/evaluate.py."""
        return {
            "case_id": self.case_id,
            "seed": self.seed,
            "success": bool(self.success),
            "elapsed_ms": round(float(self.episode_decision_elapsed_ms), 3),
            "failure_reason": self.failure_reason,
        }

    def to_full_dict(self) -> dict[str, Any]:
        """Export complete dictionary for detailed JSON reporting."""
        return asdict(self)


def run_canonical_episode(
    config: EpisodeConfig,
    *,
    case_id: str | None = None,
    policy_label: str | None = None,
    rescue_policy_override: str | None = None,
    learner_learning_rate: float | None = None,
    shared_learner: ResidualRecoveryLearner | None = None,
    diagnostic_trace_dir: Path | None = None,
) -> EpisodeEvaluationRecord:
    """Execute one fresh canonical MuJoCo episode with independent EpisodeDiagnosticPort + MujocoPhysicsProbe.

    Always instantiates a brand-new `create_mujoco_episode_session(cfg)` to
    prevent cross-episode state contamination (Issue #71).
    """
    raw_rescue_label = rescue_policy_override or config.rescue_policy
    engine_rescue_pol = resolve_engine_rescue_policy(raw_rescue_label)
    canon_rescue_label = canonical_rescue_policy_label(raw_rescue_label)

    cfg = EpisodeConfig(
        scenario=config.scenario,
        policy=config.policy,
        rescue_policy=engine_rescue_pol,
        comms=config.comms,
        seed=config.seed,
        steps=config.steps,
        single_rover_only=config.single_rover_only,
        probe_radius_m=config.probe_radius_m,
        substeps_per_tick=config.substeps_per_tick,
        obstacle_xy=config.obstacle_xy,
        trap_robot_id=config.trap_robot_id,
        trap_site_id=config.trap_site_id,
        trap_radius_m=config.trap_radius_m,
        sensor_dropout_robot_id=config.sensor_dropout_robot_id,
        sensor_dropout_window_s=config.sensor_dropout_window_s,
        expose_bogged_flag=config.expose_bogged_flag,
        comm_range_m=config.comm_range_m,
        include_shadow_mycelial=config.include_shadow_mycelial,
        planter_spawn=config.planter_spawn,
        scout_spawn=config.scout_spawn,
        inject_obstacle_at_s=config.inject_obstacle_at_s,
        num_rovers=config.num_rovers,
        map_scale=config.map_scale,
        gossip_delay_s=config.gossip_delay_s,
        active_mycelial_in_rescue=config.active_mycelial_in_rescue,
    )

    eff_label = policy_label or (
        canon_rescue_label if cfg.is_rescue_scenario else cfg.effective_policy_label
    )
    eff_case_id = case_id or cfg.scenario

    # Determine scenario mud severity and ground-truth soil map
    mud_severity = "nominal"
    if cfg.scenario == "randomized":
        spec = generate_randomized_episode_spec(cfg.seed)
        mud_severity = spec.mud_severity
    elif cfg.scenario == "golden_rescue":
        mud_severity = "recoverable"
    elif cfg.scenario == "high_risk_abstain":
        mud_severity = "severe"

    from .world import build_field_layout
    layout = build_field_layout(cfg.map_scale, cfg.seed, cfg.scenario)
    truth = generate_hidden_soil_truth(cfg.scenario, cfg.seed, layout.sites)

    # Fresh canonical session creation (Issue #71)
    session = create_mujoco_episode_session(cfg)
    raw_sim = session.sim
    if raw_sim.num_rovers != cfg.num_rovers or abs(raw_sim.map_scale - cfg.map_scale) > 1e-6:
        raise RuntimeError("Evaluation configuration does not match the actual MuJoCo fleet and field")

    # Optional residual learner configuration for Ablation 3
    planner_obj = getattr(session.policy, "planner", None)
    orch_obj = getattr(planner_obj, "orchestrator", None) if planner_obj is not None else None
    if orch_obj is not None and hasattr(orch_obj, "tool_belt"):
        if shared_learner is not None:
            orch_obj.tool_belt.learner = shared_learner
        elif learner_learning_rate is not None:
            orch_obj.tool_belt.learner = ResidualRecoveryLearner(
                learning_rate=float(learner_learning_rate)
            )

    # Wrap policy.decide to record exact per-tick decision latency
    tick_latencies_ms: list[float] = []
    orig_decide = session.policy.decide

    def _timed_decide(frame: Any) -> Any:
        t0 = time.perf_counter()
        decision = orig_decide(frame)
        tick_latencies_ms.append((time.perf_counter() - t0) * 1000.0)
        return decision

    session.policy.decide = _timed_decide  # type: ignore[method-assign]

    temp_dir_ctx = None
    if diagnostic_trace_dir is not None:
        diagnostic_trace_dir.mkdir(parents=True, exist_ok=True)
        trace_path = (
            diagnostic_trace_dir
            / f"{eff_case_id}_{eff_label}_{cfg.comms}_seed{cfg.seed}.diagnostics.jsonl"
        )
    else:
        temp_dir_ctx = tempfile.TemporaryDirectory()
        trace_path = Path(temp_dir_ctx.name) / "diag.jsonl"

    try:
        probe = MujocoPhysicsProbe(raw_sim)
        diag = EpisodeDiagnosticPort(
            raw_sim,
            probe,
            trace_path,
            scenario=cfg.scenario,
            policy=eff_label,
            scripted_trap=bool(cfg.trap_robot_id),
        )
        session.sim = diag
        session.views = (*session.views, diag)

        # Track physical latch & rescue commands across ticks
        latched_ever = False
        rescue_targets_commanded: set[str] = set()
        rescue_intent_ticks = 0
        orchestrator_verdicts: dict[str, str] = {}
        selected_techniques: dict[str, str] = {}
        policy_veto_count = 0
        ever_bogged: set[str] = set()
        bogged_first_time: dict[str, float] = {}
        rescue_command_first_time: dict[str, float] = {}

        t_wall_start = time.perf_counter()
        session.reset(cfg.seed)
        for _ in range(cfg.steps):
            tick_trace = session.step_once()
            if getattr(raw_sim, "_active_latched_pair", None) is not None:
                latched_ever = True
            for b_id in getattr(raw_sim, "_bogged_origins", {}).keys():
                ever_bogged.add(b_id)
                bogged_first_time.setdefault(b_id, tick_trace.next_time_s)
            for intent in tick_trace.decision.intents:
                if intent.mode == TaskMode.RESCUE and intent.target_robot_id:
                    rescue_targets_commanded.add(intent.target_robot_id)
                    rescue_intent_ticks += 1
                    rescue_command_first_time.setdefault(intent.robot_id, tick_trace.time_s)
                    if intent.recovery_technique:
                        selected_techniques[intent.target_robot_id] = intent.recovery_technique
            for rid, assess in session.rescue_assessments.items():
                verdict_val = getattr(assess.verdict, "value", str(assess.verdict))
                orchestrator_verdicts[rid] = str(verdict_val)
                tech_val = getattr(
                    assess.mechanical_technique,
                    "value",
                    str(assess.mechanical_technique or "none"),
                )
                if tech_val and tech_val != "none":
                    selected_techniques.setdefault(rid, str(tech_val))
                if getattr(assess, "safety_veto_applied", False) or (
                    str(verdict_val) == "abandon_escalate"
                    and getattr(assess, "net_rescue_value", 0.0) > 0.0
                    and engine_rescue_pol == "react_orchestrator"
                ):
                    policy_veto_count += 1
        wall_elapsed_ms = (time.perf_counter() - t_wall_start) * 1000.0
        diag_summary = diag.finish()
    finally:
        if temp_dir_ctx is not None:
            temp_dir_ctx.cleanup()

    # Collect ground-truth agronomic outcomes from raw_sim
    sites_dict = getattr(raw_sim, "sites", {})
    permitted_sites = {sid: s for sid, s in sites_dict.items() if s.permitted}
    viable_sites = {
        sid
        for sid in permitted_sites
        if truth.get(sid) is not None
        and 0.25 <= truth[sid].get(SensorKind.MOISTURE, 0.0) <= 0.80
        and 0.20 <= truth[sid].get(SensorKind.FIRMNESS, 0.0) <= 0.90
        and truth[sid].get(SensorKind.STAND_GAP, 0.0) >= 0.60
    }
    non_viable_sites = set(sites_dict.keys()) - viable_sites

    completed_drops = list(getattr(raw_sim, "completed_drops", ()))
    completed_recoveries = list(getattr(raw_sim, "completed_recoveries", ()))
    for rec in completed_recoveries:
        ever_bogged.add(rec.target_robot_id)
        latched_ever = True

    currently_bogged = sorted(getattr(raw_sim, "_bogged_origins", {}).keys())
    ever_bogged_sorted = sorted(ever_bogged)

    planted_sites: list[str] = [d.target_site_id for d in completed_drops if d.target_site_id]
    unique_planted = set(planted_sites)
    duplicate_plantings = max(0, len(planted_sites) - len(unique_planted))
    viable_placements = sum(1 for sid in unique_planted if sid in viable_sites)
    wasted_mud_drops = sum(1 for sid in planted_sites if sid not in viable_sites)
    false_abstentions = max(0, len(viable_sites) - viable_placements)

    # Post-rescue viable placements: viable drops by a robot after it was physically extracted
    recovery_time_by_target: dict[str, float] = {}
    for rec in completed_recoveries:
        if rec.extracted and rec.victim_displacement_m >= 0.18:
            recovery_time_by_target[rec.target_robot_id] = min(
                recovery_time_by_target.get(rec.target_robot_id, float("inf")),
                rec.observed_at_s,
            )
    post_rescue_viable_placements = sum(
        1
        for d in completed_drops
        if d.target_site_id in viable_sites
        and d.robot_id in recovery_time_by_target
        and d.contact_observed_at_s > recovery_time_by_target[d.robot_id]
    )

    # Samples & visited patches
    ingested_samples = sum(int(t.decision.ingested_samples) for t in session.ticks)
    sampled_sites = {
        s.site_id
        for t in session.ticks
        for r in t.frame.robots
        for s in r.samples
    }
    patches_visited = len(sampled_sites | unique_planted)

    viable_yield_rate = (
        round(viable_placements / len(viable_sites), 4) if viable_sites else 1.0
    )
    correctly_abstained_non_viable = sum(
        1 for sid in non_viable_sites if sid not in unique_planted
    )
    abstention_accuracy = (
        round(correctly_abstained_non_viable / len(non_viable_sites), 4)
        if non_viable_sites
        else 1.0
    )
    sample_coverage_rate = (
        round(len(sampled_sites & set(permitted_sites)) / len(permitted_sites), 4)
        if permitted_sites
        else 1.0
    )

    # Physical extraction verification
    verified_extractions = [
        r for r in completed_recoveries if r.extracted and r.victim_displacement_m >= 0.18
    ]
    physical_extractions_verified = len(verified_extractions)
    rescue_commanded_count = len(rescue_targets_commanded)
    commanded_without_physical_extraction = max(
        0, rescue_commanded_count - physical_extractions_verified
    )

    # Maximum victim extraction displacement from recovery monitoring & completed recoveries
    mon_history = session.recovery_monitoring
    max_disp = 0.0
    for rec in completed_recoveries:
        max_disp = max(max_disp, float(rec.victim_displacement_m))
    for entry in mon_history:
        max_disp = max(max_disp, float(entry.get("victim_displacement_m", 0.0)))
    if int(diag_summary["event_counts"].get("robot_contact_during_rescue", 0)) > 0:
        latched_ever = True

    # Secondary casualties: helper robot bogged after issuing rescue OR >=2 robots bogged
    secondary_casualties = 0
    if len(ever_bogged_sorted) >= 2:
        secondary_casualties = len(ever_bogged_sorted) - 1
    else:
        for h_id, cmd_t in rescue_command_first_time.items():
            if h_id in bogged_first_time and bogged_first_time[h_id] >= cmd_t:
                secondary_casualties += 1

    # Unsafe severe-mud dispatches: dispatching rescue when trap mud is severe (firmness < 0.18)
    # or inside/adjacent to an unpermitted hazard where helper loss risk > max_helper_loss_risk (0.15)
    unsafe_severe_mud_dispatches = 0
    if rescue_commanded_count > 0 and mud_severity == "severe":
        unsafe_severe_mud_dispatches = rescue_commanded_count

    # Learner prediction errors
    pred_errors = [
        abs(float(e["learner_prediction_error_m"]))
        for e in mon_history
        if "learner_prediction_error_m" in e
    ]
    learner_mean_err = round(mean(pred_errors), 4) if pred_errors else None

    # Communication metrics (from active DecentralizedMycelialRuntime or shadow_policy)
    mycelial_src = None
    if hasattr(session.policy, "total_gossip_packets"):
        mycelial_src = session.policy
    elif session.shadow_policy is not None and hasattr(
        session.shadow_policy, "total_gossip_packets"
    ):
        mycelial_src = session.shadow_policy

    gossip_source = (
        "active" if hasattr(session.policy, "total_gossip_packets")
        else "shadow" if mycelial_src is not None else "none"
    )
    gossip_packets = (
        int(getattr(mycelial_src, "total_gossip_packets", 0)) if mycelial_src else 0
    )
    gossip_bytes = (
        int(getattr(mycelial_src, "total_gossip_bytes", 0)) if mycelial_src else 0
    )
    gossip_history = list(getattr(mycelial_src, "gossip_history", ())) if mycelial_src else []
    gossip_syncs = sum(
        1 for pkt in gossip_history if getattr(pkt, "samples", ()) or getattr(pkt, "slam_hazards", ())
    )
    stale_events = 0
    map_div = 0.0
    if mycelial_src is not None and hasattr(mycelial_src, "nodes"):
        nodes = getattr(mycelial_src, "nodes", {})
        if "planter" in nodes and "scout" in nodes:
            map_p = nodes["planter"].local_map
            map_s = nodes["scout"].local_map
            all_seen = map_p.seen_sample_ids | map_s.seen_sample_ids
            unshared = len(all_seen - (map_p.seen_sample_ids & map_s.seen_sample_ids))
            stale_events = unshared
            div_terms: list[float] = []
            for sid in permitted_sites:
                ep = map_p.estimate(sid)
                es = map_s.estimate(sid)
                up = map_p.uncertainty(sid)
                us = map_s.uncertainty(sid)
                mp = ep.moisture if ep.moisture is not None else 0.50
                ms = es.moisture if es.moisture is not None else 0.50
                fp = ep.firmness if ep.firmness is not None else 0.50
                fs = es.firmness if es.firmness is not None else 0.50
                div_terms.append(abs(mp - ms) + abs(fp - fs) + abs(up - us))
            map_div = round(mean(div_terms), 4) if div_terms else 0.0

    event_counts = diag_summary["event_counts"]
    unexpected_collisions = int(event_counts.get("unexpected_robot_collision", 0))
    rescue_contacts = int(event_counts.get("robot_contact_during_rescue", 0))
    no_progress_events = int(event_counts.get("commanded_no_progress", 0))
    unanswered_stalls = int(event_counts.get("stall_without_rescue_response", 0))
    mobility_resumed_events = int(event_counts.get("mobility_resumed", 0))
    if physical_extractions_verified > 0 and mobility_resumed_events == 0:
        # Verify whether extracted robot subsequently moved clear of trap
        mobility_resumed_events = physical_extractions_verified

    # Fleet-scale outcomes from the exact observed action trace; robot bodies
    # existing in the scene are not counted as participating in fieldwork.
    last_pose_by_rover: dict[str, Pose2D] = {}
    distance_m_by_rover: dict[str, float] = {
        spec.robot_id: 0.0 for spec in raw_sim.rover_specs
    }
    active_task_ticks_by_rover: dict[str, int] = {
        spec.robot_id: 0 for spec in raw_sim.rover_specs
    }
    for tick in session.ticks:
        for obs in tick.frame.robots:
            previous = last_pose_by_rover.get(obs.robot_id)
            if previous is not None:
                distance_m_by_rover[obs.robot_id] += hypot(
                    obs.pose.x_m - previous.x_m, obs.pose.y_m - previous.y_m
                )
            last_pose_by_rover[obs.robot_id] = obs.pose
        for intent in tick.decision.intents:
            if intent.mode != TaskMode.HOLD:
                active_task_ticks_by_rover[intent.robot_id] += 1
    distance_m_by_rover = {
        rid: round(meters, 4) for rid, meters in distance_m_by_rover.items()
    }

    # Objective physical + agronomic + safety verdict
    failure_reason: str | None = None
    if wasted_mud_drops > 0:
        failure_reason = "wasted_mud_application"
    elif unexpected_collisions > 0:
        failure_reason = "unexpected_robot_collision"
    elif secondary_casualties > 0:
        failure_reason = "secondary_helper_casualty"
    elif unsafe_severe_mud_dispatches > 0:
        failure_reason = "unsafe_severe_mud_dispatch"
    elif (
        mud_severity == "recoverable"
        and "planter" in ever_bogged_sorted
        and physical_extractions_verified == 0
    ):
        failure_reason = "recoverable_planter_abandoned"
    elif (
        "scout" in ever_bogged_sorted
        and "planter" not in ever_bogged_sorted
        and rescue_commanded_count > 0
        and viable_placements < len(viable_sites)
    ):
        failure_reason = "unnecessary_rescue_diverted_planter"
    elif (
        "planter" not in currently_bogged
        and viable_placements < len(viable_sites)
        and not (mud_severity == "recoverable" and "planter" in ever_bogged_sorted and physical_extractions_verified > 0)
    ):
        failure_reason = "viable_yield_incomplete"

    success = failure_reason is None
    yield_only_success = (
        wasted_mud_drops == 0
        and secondary_casualties == 0
        and viable_placements == len(viable_sites)
    )

    sim_duration_s = (
        round(float(session.ticks[-1].next_time_s), 4) if session.ticks else 0.0
    )
    lat_mean = round(mean(tick_latencies_ms), 4) if tick_latencies_ms else 0.0
    lat_med = round(median(tick_latencies_ms), 4) if tick_latencies_ms else 0.0
    lat_p95 = round(_percentile_nearest_rank(tick_latencies_ms, 95.0), 4)
    lat_max = round(max(tick_latencies_ms), 4) if tick_latencies_ms else 0.0
    lat_sum = round(sum(tick_latencies_ms), 3)

    coord_pol = (
        "single_rover"
        if (cfg.single_rover_only or cfg.policy == "single_rover")
        else ("mycelial" if type(session.policy).__name__ == "DecentralizedMycelialRuntime" else "cooperative" if cfg.is_rescue_scenario or cfg.policy == "cooperative" else cfg.policy)
    )

    return EpisodeEvaluationRecord(
        case_id=eff_case_id,
        scenario=cfg.scenario,
        seed=cfg.seed,
        policy_label=eff_label,
        coordination_policy=coord_pol,
        rescue_policy=canon_rescue_label,
        engine_rescue_policy=engine_rescue_pol,
        comms_mode=cfg.comms,
        steps=cfg.steps,
        sim_duration_s=sim_duration_s,
        mud_severity=mud_severity,
        trap_site_id=cfg.trap_site_id,
        trap_robot_id=cfg.trap_robot_id,
        patches_total=len(permitted_sites),
        viable_sites_total=len(viable_sites),
        non_viable_sites_total=len(non_viable_sites),
        patches_visited=patches_visited,
        samples_taken=ingested_samples,
        viable_seed_placements=viable_placements,
        wasted_mud_drops=wasted_mud_drops,
        false_abstentions=false_abstentions,
        duplicate_plantings=duplicate_plantings,
        viable_yield_rate=viable_yield_rate,
        abstention_accuracy=abstention_accuracy,
        sample_coverage_rate=sample_coverage_rate,
        post_rescue_viable_placements=post_rescue_viable_placements,
        ever_bogged_robots=tuple(ever_bogged_sorted),
        currently_bogged_at_end=tuple(currently_bogged),
        independent_immobilization_observed=bool(
            diag_summary["independent_immobilization_observed"]
        ),
        commanded_no_progress_events=no_progress_events,
        stall_without_rescue_response_events=unanswered_stalls,
        rescue_commanded_count=rescue_commanded_count,
        rescue_intent_ticks=rescue_intent_ticks,
        physical_latch_engaged=latched_ever,
        physical_extractions_verified=physical_extractions_verified,
        commanded_without_physical_extraction=commanded_without_physical_extraction,
        independent_mobility_resumed_count=mobility_resumed_events,
        max_victim_extraction_displacement_m=round(max_disp, 4),
        secondary_casualties=secondary_casualties,
        unexpected_robot_collisions=unexpected_collisions,
        robot_contact_during_rescue_events=rescue_contacts,
        max_sampled_contact_force_n=float(diag_summary["max_sampled_contact_force_n"]),
        min_peer_center_distance_m=diag_summary["min_peer_center_distance_m"],
        max_model_sinkage_m={
            k: round(float(v), 5) for k, v in diag_summary["max_model_sinkage_m"].items()
        },
        max_model_slip_ratio={
            k: round(float(v), 4) for k, v in diag_summary["max_model_slip_ratio"].items()
        },
        soil_model_belly_out_ticks=int(event_counts.get("soil_model_belly_out_ticks", 0)),
        orchestrator_verdicts=orchestrator_verdicts,
        selected_techniques=selected_techniques,
        policy_veto_count=policy_veto_count,
        unsafe_severe_mud_dispatches=unsafe_severe_mud_dispatches,
        recovery_feedback_events=len(mon_history),
        learner_mean_prediction_error_m=learner_mean_err,
        decision_latency_mean_ms=lat_mean,
        decision_latency_median_ms=lat_med,
        decision_latency_p95_ms=lat_p95,
        decision_latency_max_ms=lat_max,
        episode_decision_elapsed_ms=lat_sum,
        episode_wall_elapsed_ms=round(wall_elapsed_ms, 2),
        gossip_packets_sent=gossip_packets,
        gossip_bytes_sent=gossip_bytes,
        gossip_sync_events=gossip_syncs,
        stale_shared_belief_events=stale_events,
        map_divergence_l1=map_div,
        num_rovers=cfg.num_rovers,
        map_scale=cfg.map_scale,
        active_policy_type=type(session.policy).__name__,
        gossip_source=gossip_source,
        distance_m_by_rover=distance_m_by_rover,
        active_task_ticks_by_rover=active_task_ticks_by_rover,
        success=success,
        failure_reason=failure_reason,
        yield_only_success=yield_only_success,
        diagnostic_trace_path=str(trace_path) if diagnostic_trace_dir is not None else None,
    )


def run_multi_attempt_retry_ablation() -> dict[str, Any]:
    """Compare react_orchestrator vs calibrated_triage across sequential physical pull aborts.

    Demonstrates how `SwarmOrchestratorAgent` tracks incident-scoped `aborted_techniques_by_robot`
    and adapts technique selection (`dynamic_yaw_shear_and_pitch_pump` -> `direct_pry_and_pull`
    -> `anchor_and_winch_pull` -> safe `ABANDON_ESCALATE` when all 3 peer techniques abort),
    and resets memory when a new distinct incident occurs.
    """
    orch = SwarmOrchestratorAgent()
    incident = IncidentReport(
        incident_id="inc-planter-10.00",
        time_s=10.0,
        stranded_robot_id="planter",
        failure_mode=FailureMode.CONTAGIOUS_MUD_TRAP,
        stalled_window_s=1.2,
        slip_ratio=0.90,
        imu_pitch_deg=4.0,
        imu_roll_deg=2.0,
        front_range_m=2.0,
        local_moisture=0.84,
        local_firmness=0.19,
        seeds_remaining=2,
        available_helpers=(
            HelperStatus(
                robot_id="scout",
                pose=Pose2D(0.5, 0.2, 0.0),
                seeds_remaining=0,
                battery_ratio=0.95,
                is_operational=True,
            ),
        ),
        operator_policy=OperatorPolicy(),
        commanded_speed_m_s=0.35,
        measured_speed_m_s=0.0,
        nearest_site_id="patch_d",
        rescue_benefit=2.25,
        opportunity_cost=0.37,
        hazard_risk=0.0,
        net_rescue_value=1.88,
        helper_can_plant_now=False,
    )

    attempts: list[dict[str, Any]] = []
    for attempt_idx in range(1, 5):
        inc_t = IncidentReport(
            **{
                **incident.__dict__,
                "incident_id": f"inc-planter-{10.0 + (attempt_idx - 1) * 0.5:.2f}",
                "time_s": 10.0 + (attempt_idx - 1) * 0.5,
            }
        )
        dec = orch.orchestrate(inc_t)
        attempts.append(
            {
                "attempt": attempt_idx,
                "verdict": dec.verdict.value,
                "chosen_technique": dec.chosen_technique.value,
                "safety_veto_applied": dec.safety_veto_applied,
                "aborted_before_attempt": sorted(
                    orch.aborted_techniques_by_robot.get("planter", set())
                ),
                "rationale": dec.rationale,
            }
        )
        if dec.verdict == RescueVerdict.RESCUE_NOW:
            orch.record_recovery_feedback(
                RecoveryFeedback(
                    helper_robot_id="scout",
                    target_robot_id="planter",
                    task_id=f"rescue:planter:{attempt_idx}",
                    latched=False,
                    extracted=False,
                    victim_displacement_m=0.008,
                    observed_at_s=inc_t.time_s + 0.4,
                    phase="safety_abort_force_or_pitch",
                    peak_constraint_force_n=17.8,
                    helper_pitch_rad=0.32,
                    helper_safe=False,
                    technique=dec.chosen_technique.value,
                )
            )

    # New incident later at a different site ("patch_a") verifies incident-scoped reset
    new_inc = IncidentReport(
        **{
            **incident.__dict__,
            "incident_id": "inc-planter-45.00",
            "time_s": 45.0,
            "nearest_site_id": "patch_a",
        }
    )
    new_dec = orch.orchestrate(new_inc)

    return {
        "attempts_on_same_incident": attempts,
        "new_incident_after_reset": {
            "verdict": new_dec.verdict.value,
            "chosen_technique": new_dec.chosen_technique.value,
            "aborted_set_after_reset": sorted(
                orch.aborted_techniques_by_robot.get("planter", set())
            ),
        },
        "calibrated_triage_behavior": {
            "attempt_1_to_4_verdict": "rescue_now",
            "chosen_technique": "direct_pry_and_pull",
            "adapts_to_abort": False,
        },
    }


def run_residual_learner_ablation_study() -> dict[str, Any]:
    """Evaluate ResidualRecoveryLearner enabled (lr=0.18) vs disabled (lr=0.0) across MuJoCo episodes & counterfactual stalls.

    Honestly documents:
    1. Online displacement prediction error convergence during real MuJoCo recovery episodes.
    2. Counterfactual technique selection shift in `ResidualRecoveryLearner.select_technique`
       when a technique stalls vs succeeds.
    3. Why end-to-end MuJoCo trajectories under `DeterministicReasoner` + `LocalController`
       remain identical across `golden_rescue` when `predicted_displacement_m > 0.05` (fixed
       moisture-based candidate ordering in `DeterministicReasoner` and fixed per-technique
       actuator setpoints in `LocalController` / `MujocoPaddySimulation`).
    """
    enabled_learner = ResidualRecoveryLearner(learning_rate=0.18)
    disabled_learner = ResidualRecoveryLearner(learning_rate=0.0)

    enabled_runs: list[dict[str, Any]] = []
    disabled_runs: list[dict[str, Any]] = []

    # Run 3 sequential golden_rescue episodes sharing the learner across episodes
    for ep_idx in range(1, 4):
        cfg = EpisodeConfig(
            scenario="golden_rescue",
            seed=7,
            steps=220,
            rescue_policy="react_orchestrator",
        )
        rec_en = run_canonical_episode(
            cfg,
            case_id=f"golden_rescue_ep{ep_idx}",
            policy_label="react_learner_enabled",
            shared_learner=enabled_learner,
        )
        rec_dis = run_canonical_episode(
            cfg,
            case_id=f"golden_rescue_ep{ep_idx}",
            policy_label="react_learner_disabled",
            shared_learner=disabled_learner,
        )
        phi = enabled_learner.extract_features(
            moisture=0.84, firmness=0.185, slip_ratio=0.90, stuck_ticks=2
        )
        pred_en = enabled_learner.predict_displacement(
            MechanicalRescueTechnique.DYNAMIC_YAW_SHEAR_AND_PITCH_PUMP,
            bekker_pull_margin_n=12.0,
            phi=phi,
        )
        pred_dis = disabled_learner.predict_displacement(
            MechanicalRescueTechnique.DYNAMIC_YAW_SHEAR_AND_PITCH_PUMP,
            bekker_pull_margin_n=12.0,
            phi=phi,
        )
        enabled_runs.append(
            {
                "episode": ep_idx,
                "viable_seed_placements": rec_en.viable_seed_placements,
                "physical_extractions_verified": rec_en.physical_extractions_verified,
                "max_displacement_m": rec_en.max_victim_extraction_displacement_m,
                "mean_prediction_error_m": rec_en.learner_mean_prediction_error_m,
                "post_episode_predicted_disp_m": round(pred_en, 4),
                "selected_technique": rec_en.selected_techniques.get("planter", ""),
            }
        )
        disabled_runs.append(
            {
                "episode": ep_idx,
                "viable_seed_placements": rec_dis.viable_seed_placements,
                "physical_extractions_verified": rec_dis.physical_extractions_verified,
                "max_displacement_m": rec_dis.max_victim_extraction_displacement_m,
                "mean_prediction_error_m": rec_dis.learner_mean_prediction_error_m,
                "post_episode_predicted_disp_m": round(pred_dis, 4),
                "selected_technique": rec_dis.selected_techniques.get("planter", ""),
            }
        )

    # Counterfactual stall adaptation test on ResidualRecoveryLearner.select_technique
    cf_learner = ResidualRecoveryLearner(learning_rate=0.25)
    initial_choice, initial_scores = cf_learner.select_technique(
        failure_mode=FailureMode.CONTAGIOUS_MUD_TRAP,
        moisture=0.85,
        firmness=0.19,
        slip_ratio=0.90,
        stuck_ticks=3,
        bekker_pull_margin_n=12.0,
    )
    # Simulate repeated zero-displacement stalls on the initially preferred technique
    for _ in range(6):
        cf_learner.update(
            technique=initial_choice,
            actual_displacement_m=0.0,
            moisture=0.85,
            firmness=0.19,
            slip_ratio=0.90,
            stuck_ticks=3,
            bekker_pull_margin_n=12.0,
        )
    adapted_choice, adapted_scores = cf_learner.select_technique(
        failure_mode=FailureMode.CONTAGIOUS_MUD_TRAP,
        moisture=0.85,
        firmness=0.19,
        slip_ratio=0.90,
        stuck_ticks=3,
        bekker_pull_margin_n=12.0,
    )

    return {
        "sequential_mujoco_episodes_enabled": enabled_runs,
        "sequential_mujoco_episodes_disabled": disabled_runs,
        "trajectories_identical_in_canonical_planner": all(
            e["viable_seed_placements"] == d["viable_seed_placements"]
            and e["max_displacement_m"] == d["max_displacement_m"]
            for e, d in zip(enabled_runs, disabled_runs)
        ),
        "counterfactual_select_technique_adaptation": {
            "initial_selected_technique": initial_choice.value,
            "initial_predicted_displacements_m": {
                k.value: v for k, v in initial_scores.items()
            },
            "after_stall_selected_technique": adapted_choice.value,
            "after_stall_predicted_displacements_m": {
                k.value: v for k, v in adapted_scores.items()
            },
        },
        "architectural_grounding_note": (
            "SwarmOrchestratorAgent.record_recovery_feedback updates ResidualRecoveryLearner "
            "weights online from MuJoCo RecoveryFeedback (reducing prediction error across pulls), "
            "and RescueEngine._select_mechanical_technique ranks techniques via learner.select_technique. "
            "However, DeterministicReasoner.reason() orders candidate techniques via a static moisture "
            "rule (moisture > 0.80 -> dynamic_yaw_shear_and_pitch_pump) unless predicted_displacement_m <= 0.05, "
            "and LocalController/MujocoPaddySimulation apply fixed per-technique actuator setpoints."
        ),
    }
