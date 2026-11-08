"""Generate high-fidelity ReAct fine-tuning dataset for Swarm Orchestrator SLMs (`scripts/generate_slm_dataset.py`).

Generates diverse agricultural emergency scenarios covering:
1. Contagious Mud Trap (Viable Rescue)
2. Seed Carrier Protection (Honest Refusal)
3. Sensor Dropout False Alarms (Ambiguity Resolution)
4. Hazardous Perimeter (Anti-Cascading Refusal)
5. Severe Rollover / Ditch Tilt (Anchor Spades & Unsafe to Dock)
6. Asymmetric Bund Rut (Yaw / Pitch Realignment)
7. Low-Battery Helper Abort (Secondary Casualty Avoidance)
8. High-Center Bund Belly (Self-Jacking Dibber Oscillation)

Uses `DeterministicReasoner` to compute verified Pareto-optimal tool call traces,
outputting universal Hugging Face / OpenAI `messages` format JSONL datasets.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import random
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ricebot.agent import (
    DeterministicReasoner,
    FailureMode,
    HelperStatus,
    IncidentReport,
    MechanicalRescueTechnique,
    OperatorPolicy,
    OrchestratorSafetyGate,
    OrchestratorToolBelt,
    RescueVerdict,
    SwarmOrchestratorAgent,
)
from ricebot.contracts import Pose2D, TaskMode


def format_react_assistant_response(trace: list, decision) -> str:
    """Format multi-round tool traces and final decision into standard ReAct response."""
    blocks = []
    for step in trace:
        blocks.append(
            f"Thought: {step.thought}\n"
            f"Action: {step.tool_name}\n"
            f"Action Input: {json.dumps(step.arguments, sort_keys=True)}\n"
            f"Observation: {json.dumps(step.observation, sort_keys=True)}"
        )

    final_payload = {
        "verdict": decision.verdict.value,
        "failure_mode": decision.failure_mode.value,
        "chosen_technique": decision.chosen_technique.value,
        "helper_robot_id": decision.helper_robot_id,
        "rationale": decision.rationale,
    }
    blocks.append(
        f"Thought: Synthesizing verified observations into final mission decision.\n"
        f"Final Decision:\n{json.dumps(final_payload, indent=2)}"
    )
    return "\n\n".join(blocks)


def create_incident(category: str, seed: int, idx: int) -> IncidentReport:
    """Create a physically grounded IncidentReport for a specific scenario category."""
    rng = random.Random(seed * 1000 + idx)

    time_s = round(10.0 + rng.uniform(0.0, 180.0), 1)
    stranded_id = rng.choice(["planter", "rover-2", "ant-alpha"])
    helper_id = "scout" if stranded_id != "scout" else "rover-1"

    if category == "mud_rescue_viable":
        # Trapped in localized mud pan, but helper perimeter is firm loam
        return IncidentReport(
            incident_id=f"inc-mud-{idx:03d}",
            time_s=time_s,
            stranded_robot_id=stranded_id,
            failure_mode=FailureMode.CONTAGIOUS_MUD_TRAP,
            stalled_window_s=round(rng.uniform(3.0, 8.0), 1),
            slip_ratio=round(rng.uniform(0.82, 0.96), 3),
            imu_pitch_deg=round(rng.uniform(-4.0, 4.0), 1),
            imu_roll_deg=round(rng.uniform(-3.0, 3.0), 1),
            front_range_m=round(rng.uniform(0.12, 0.30), 2),
            local_moisture=round(rng.uniform(0.68, 0.78), 2),
            local_firmness=round(rng.uniform(0.28, 0.38), 2),
            seeds_remaining=rng.choice([0, 1, 2]),
            available_helpers=(
                HelperStatus(
                    robot_id=helper_id,
                    pose=Pose2D(round(rng.uniform(0.5, 2.0), 2), round(rng.uniform(-0.8, 0.8), 2), 0.0),
                    seeds_remaining=0,  # 0 seeds = safe helper!
                    battery_ratio=round(rng.uniform(0.65, 0.95), 2),
                    is_operational=True,
                ),
            ),
            operator_policy=OperatorPolicy(
                max_helper_loss_risk=0.30,
                protect_seed_carrier=True,
            ),
        )

    elif category == "seed_carrier_protection":
        # Trapped in mud, but ONLY available helper holds precious seed inventory
        return IncidentReport(
            incident_id=f"inc-seed-protect-{idx:03d}",
            time_s=time_s,
            stranded_robot_id=stranded_id,
            failure_mode=FailureMode.CONTAGIOUS_MUD_TRAP,
            stalled_window_s=round(rng.uniform(4.0, 10.0), 1),
            slip_ratio=round(rng.uniform(0.85, 0.98), 3),
            imu_pitch_deg=round(rng.uniform(-5.0, 5.0), 1),
            imu_roll_deg=round(rng.uniform(-4.0, 4.0), 1),
            front_range_m=0.18,
            local_moisture=0.92,
            local_firmness=0.14,
            seeds_remaining=0,
            available_helpers=(
                HelperStatus(
                    robot_id=helper_id,
                    pose=Pose2D(1.5, 0.4, 0.0),
                    seeds_remaining=rng.choice([2, 4]),  # Holds seeds!
                    battery_ratio=0.88,
                    is_operational=True,
                ),
            ),
            operator_policy=OperatorPolicy(
                max_helper_loss_risk=0.15,
                protect_seed_carrier=True,  # STRICT policy
            ),
        )

    elif category == "sensor_dropout":
        # LiDAR dropped out (None), but clean odometry, normal slip, and level attitude
        return IncidentReport(
            incident_id=f"inc-sensor-dropout-{idx:03d}",
            time_s=time_s,
            stranded_robot_id=stranded_id,
            failure_mode=FailureMode.SENSOR_DROPOUT,
            stalled_window_s=round(rng.uniform(2.5, 6.0), 1),
            slip_ratio=round(rng.uniform(0.04, 0.12), 3),
            imu_pitch_deg=round(rng.uniform(-1.5, 1.5), 1),
            imu_roll_deg=round(rng.uniform(-1.0, 1.0), 1),
            front_range_m=None,  # Sensor fault!
            local_moisture=0.48,
            local_firmness=0.72,
            seeds_remaining=rng.choice([0, 2]),
            available_helpers=(
                HelperStatus(helper_id, Pose2D(0.8, -0.2, 0.0), seeds_remaining=0),
            ),
            operator_policy=OperatorPolicy(),
        )

    elif category == "hazardous_perimeter":
        # Trapped in mud, and standoff perimeter is ALSO soupy mud (secondary casualty hazard)
        return IncidentReport(
            incident_id=f"inc-haz-perimeter-{idx:03d}",
            time_s=time_s,
            stranded_robot_id=stranded_id,
            failure_mode=FailureMode.CONTAGIOUS_MUD_TRAP,
            stalled_window_s=round(rng.uniform(3.0, 7.0), 1),
            slip_ratio=round(rng.uniform(0.88, 0.96), 3),
            imu_pitch_deg=round(rng.uniform(-3.0, 3.0), 1),
            imu_roll_deg=round(rng.uniform(-3.0, 3.0), 1),
            front_range_m=0.20,
            local_moisture=0.94,
            local_firmness=0.11,
            seeds_remaining=1,
            available_helpers=(
                HelperStatus(helper_id, Pose2D(1.0, -0.5, 0.0), seeds_remaining=0),
            ),
            operator_policy=OperatorPolicy(
                max_helper_loss_risk=0.08,  # Very low tolerance
            ),
        )

    elif category == "rollover_trench":
        # High lateral roll or nose pitch into deep ditch (unsafe to dock!)
        return IncidentReport(
            incident_id=f"inc-rollover-{idx:03d}",
            time_s=time_s,
            stranded_robot_id=stranded_id,
            failure_mode=FailureMode.TIP_OVER_OR_TRENCH,
            stalled_window_s=round(rng.uniform(2.0, 5.0), 1),
            slip_ratio=round(rng.uniform(0.10, 0.40), 3),
            imu_pitch_deg=round(rng.choice([-24.0, 25.0]), 1),
            imu_roll_deg=round(rng.choice([-22.0, 23.5]), 1),
            front_range_m=round(rng.uniform(0.05, 0.25), 2),
            local_moisture=0.62,
            local_firmness=0.45,
            seeds_remaining=rng.choice([0, 1]),
            available_helpers=(
                HelperStatus(helper_id, Pose2D(1.2, 0.1, 0.0), seeds_remaining=0),
            ),
            operator_policy=OperatorPolicy(),
        )

    elif category == "asymmetric_bund_rut":
        # Nose-down pitch into bund with elevated rear strike bar
        return IncidentReport(
            incident_id=f"inc-bund-rut-{idx:03d}",
            time_s=time_s,
            stranded_robot_id=stranded_id,
            failure_mode=FailureMode.PITCH_NOSE_DOWN_STALL,
            stalled_window_s=round(rng.uniform(3.0, 6.0), 1),
            slip_ratio=round(rng.uniform(0.55, 0.75), 3),
            imu_pitch_deg=round(rng.uniform(-16.0, -12.0), 1),
            imu_roll_deg=round(rng.uniform(-2.0, 2.0), 1),
            front_range_m=0.10,
            local_moisture=round(rng.uniform(0.52, 0.62), 2),
            local_firmness=round(rng.uniform(0.48, 0.60), 2),
            seeds_remaining=0,
            available_helpers=(
                HelperStatus(helper_id, Pose2D(0.6, -0.3, 0.0), seeds_remaining=0),
            ),
            operator_policy=OperatorPolicy(max_helper_loss_risk=0.25),
        )

    elif category == "low_battery_abort":
        # Trapped in mud, but only available helper has critically low battery (<20%)
        return IncidentReport(
            incident_id=f"inc-low-batt-{idx:03d}",
            time_s=time_s,
            stranded_robot_id=stranded_id,
            failure_mode=FailureMode.CONTAGIOUS_MUD_TRAP,
            stalled_window_s=round(rng.uniform(3.5, 8.0), 1),
            slip_ratio=round(rng.uniform(0.85, 0.95), 3),
            imu_pitch_deg=round(rng.uniform(-3.0, 3.0), 1),
            imu_roll_deg=round(rng.uniform(-2.0, 2.0), 1),
            front_range_m=0.18,
            local_moisture=0.74,
            local_firmness=0.32,
            seeds_remaining=0,
            available_helpers=(
                HelperStatus(
                    robot_id=helper_id,
                    pose=Pose2D(1.4, -0.4, 0.0),
                    seeds_remaining=0,
                    battery_ratio=round(rng.uniform(0.12, 0.18), 2),
                    is_operational=True,
                ),
            ),
            operator_policy=OperatorPolicy(
                max_helper_loss_risk=0.25,
            ),
        )

    else:  # high_center_bund_belly
        # Chassis beached/high-centered on dry dividing bund berm, wheels spinning in air
        return IncidentReport(
            incident_id=f"inc-bund-belly-{idx:03d}",
            time_s=time_s,
            stranded_robot_id=stranded_id,
            failure_mode=FailureMode.SURFACE_OBSTRUCTION,
            stalled_window_s=round(rng.uniform(4.0, 9.0), 1),
            slip_ratio=round(rng.uniform(0.92, 0.99), 3),
            imu_pitch_deg=round(rng.uniform(-2.0, 2.0), 1),
            imu_roll_deg=round(rng.uniform(-2.0, 2.0), 1),
            front_range_m=0.35,
            local_moisture=round(rng.uniform(0.22, 0.32), 2),
            local_firmness=round(rng.uniform(0.62, 0.72), 2),
            seeds_remaining=rng.choice([0, 1]),
            available_helpers=(
                HelperStatus(helper_id, Pose2D(1.8, 0.5, 0.0), seeds_remaining=0),
            ),
            operator_policy=OperatorPolicy(),
        )


def build_dataset(num_train: int = 150, num_eval: int = 30, seed: int = 42) -> tuple[list[dict], list[dict]]:
    """Build train and eval splits with full ReAct conversations."""
    tool_belt = OrchestratorToolBelt()
    agent = SwarmOrchestratorAgent(tool_belt=tool_belt)
    system_prompt = (
        "You are the System-2 Deliberative Swarm Orchestrator for agricultural robots in a rice terrace.\n"
        "Your role is to reason through potential robot immobilizations, resolve sensor ambiguities, "
        "protect valuable seed-carrying rovers, and choose whether to execute a physical recovery or safely refuse.\n\n"
        "AVAILABLE TOOLS:\n"
        "1. inspect_incident(): Verifies true physical stall vs sensor dropout.\n"
        "2. probe_perimeter(helper_id, standoff_m): Checks soil firmness around the trapped peer.\n"
        "3. predict_recovery_displacement(technique, moisture, firmness, slip_ratio): Queries the ResidualRecoveryLearner.\n"
        "4. evaluate_farmer_risk(helper_id, technique, estimated_helper_risk): Validates against farmer policies.\n"
        "5. authorize_recovery_skill(technique, helper_id): Formulates mechanical setpoints.\n"
        "6. refuse_and_rebalance(reason): Declines rescue, alerts farmer, reallocates tasks."
    )

    categories = [
        "mud_rescue_viable",
        "seed_carrier_protection",
        "sensor_dropout",
        "hazardous_perimeter",
        "rollover_trench",
        "asymmetric_bund_rut",
        "low_battery_abort",
        "high_center_bund_belly",
    ]

    total_episodes = num_train + num_eval
    train_records: list[dict] = []
    eval_records: list[dict] = []

    for i in range(total_episodes):
        cat = categories[i % len(categories)]
        incident = create_incident(cat, seed=seed, idx=i)
        
        # Execute Deterministic Reasoner to obtain ground-truth Pareto-optimal ReAct trace
        decision = agent.orchestrate(incident)
        
        # Extract the tool trace records generated by the reasoner
        trace = list(decision.tool_trace)
        assistant_content = format_react_assistant_response(trace, decision)
        
        user_content = (
            f"NEW INCIDENT TELEMETRY DETECTED:\n"
            f"{json.dumps(incident.to_dict(), indent=2)}\n\n"
            f"Analyze this incident step-by-step using ReAct. Call the required inspection tools, "
            f"assess farmer policy risks, and output your final mission decision."
        )

        record = {
            "incident_id": incident.incident_id,
            "category": cat,
            "verdict": decision.verdict.value,
            "chosen_technique": decision.chosen_technique.value,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_content},
                {"role": "assistant", "content": assistant_content},
            ],
        }

        if i < num_train:
            train_records.append(record)
        else:
            eval_records.append(record)

    return train_records, eval_records


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate ReAct SLM dataset for Swarm Orchestrator")
    parser.add_argument("--train-size", type=int, default=150, help="Number of training examples")
    parser.add_argument("--eval-size", type=int, default=30, help="Number of evaluation examples")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for reproducibility")
    parser.add_argument("--out-dir", type=str, default="data", help="Output directory")
    args = parser.parse_args()

    out_path = ROOT / args.out_dir
    out_path.mkdir(parents=True, exist_ok=True)

    print(f"Generating Swarm Orchestrator SLM Dataset ({args.train_size} train, {args.eval_size} eval)...")
    train_data, eval_data = build_dataset(num_train=args.train_size, num_eval=args.eval_size, seed=args.seed)

    train_file = out_path / "orchestrator_slm_train.jsonl"
    eval_file = out_path / "orchestrator_slm_eval.jsonl"

    with open(train_file, "w", encoding="utf-8") as f:
        for r in train_data:
            f.write(json.dumps(r) + "\n")

    with open(eval_file, "w", encoding="utf-8") as f:
        for r in eval_data:
            f.write(json.dumps(r) + "\n")

    print(f"Done! Wrote {len(train_data)} train records to {train_file}")
    print(f"Done! Wrote {len(eval_data)} eval records to {eval_file}")

    # Summary statistics
    verdicts = {}
    for r in train_data:
        verdicts[r["verdict"]] = verdicts.get(r["verdict"], 0) + 1
    print("\nDataset Verdict Breakdown:")
    for v, count in verdicts.items():
        print(f"  - {v}: {count} ({count/len(train_data)*100:.1f}%)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
