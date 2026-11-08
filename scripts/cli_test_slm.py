"""Interactive Terminal CLI to test the fine-tuned Swarm ReAct SLM on Modal.

Run:
  $env:PYTHONUTF8=1; python scripts/cli_test_slm.py
"""
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ricebot.agent.orchestrator import ModalSLMBackend

SCENARIOS = {
    "1": {
        "name": "Mud Trap (Viable Rescue: Firm perimeter, helper has safe clearance)",
        "json": """{
  "incident_id": "inc-mud-001",
  "stranded_robot_id": "ant-alpha",
  "failure_mode": "contagious_mud_trap",
  "stalled_window_s": 6.5,
  "slip_ratio": 0.88,
  "local_moisture": 0.68,
  "local_firmness": 0.36,
  "available_helpers": [{"robot_id": "scout", "seeds_remaining": 0, "battery_ratio": 0.85}],
  "operator_policy": {"protect_seed_carrier": true, "max_helper_loss_risk": 0.3}
}"""
    },
    "2": {
        "name": "Seed Carrier Protection (Helper holds remaining seeds -> Must Refuse)",
        "json": """{
  "incident_id": "inc-seed-002",
  "stranded_robot_id": "planter",
  "failure_mode": "contagious_mud_trap",
  "stalled_window_s": 5.2,
  "slip_ratio": 0.95,
  "local_moisture": 0.92,
  "local_firmness": 0.15,
  "available_helpers": [{"robot_id": "scout", "seeds_remaining": 4, "battery_ratio": 0.90}],
  "operator_policy": {"protect_seed_carrier": true, "max_helper_loss_risk": 0.15}
}"""
    },
    "3": {
        "name": "Sensor Dropout False Alarm (Zero rangefinder, clean odometry -> Hold)",
        "json": """{
  "incident_id": "inc-sensor-003",
  "stranded_robot_id": "rover-1",
  "failure_mode": "sensor_dropout",
  "stalled_window_s": 3.0,
  "slip_ratio": 0.05,
  "front_range_m": null,
  "local_moisture": 0.45,
  "local_firmness": 0.75,
  "available_helpers": [{"robot_id": "scout", "seeds_remaining": 0, "battery_ratio": 1.0}],
  "operator_policy": {"protect_seed_carrier": true, "max_helper_loss_risk": 0.2}
}"""
    },
    "4": {
        "name": "Low-Battery Helper Abort (Helper at 14% battery -> Refuse to avoid dual stranding)",
        "json": """{
  "incident_id": "inc-low-batt-004",
  "stranded_robot_id": "ant-alpha",
  "failure_mode": "contagious_mud_trap",
  "stalled_window_s": 5.0,
  "slip_ratio": 0.90,
  "local_moisture": 0.74,
  "local_firmness": 0.32,
  "available_helpers": [{"robot_id": "scout", "seeds_remaining": 0, "battery_ratio": 0.14}],
  "operator_policy": {"protect_seed_carrier": true, "max_helper_loss_risk": 0.25}
}"""
    },
    "5": {
        "name": "High-Center Bund Belly (Chassis beached on berm, wheels in air -> Self-Oscillate Dibber)",
        "json": """{
  "incident_id": "inc-bund-belly-005",
  "stranded_robot_id": "planter",
  "failure_mode": "surface_obstruction",
  "stalled_window_s": 6.0,
  "slip_ratio": 0.98,
  "local_moisture": 0.25,
  "local_firmness": 0.68,
  "available_helpers": [{"robot_id": "scout", "seeds_remaining": 0, "battery_ratio": 0.90}],
  "operator_policy": {"protect_seed_carrier": true, "max_helper_loss_risk": 0.25}
}"""
    }
}

def main():
    print("=" * 72)
    print(" INTERACTIVE MODAL SLM TERMINAL RUNNER")
    print(" Model: Qwen/Qwen2.5-0.5B-Instruct + Modal LoRA Adapter")
    print(" Deployed on: antenna-swarm-slm-service")
    print("=" * 72)
    
    print("\nSelect a test scenario to send to the model:")
    for k, v in SCENARIOS.items():
        print(f"  [{k}] {v['name']}")
    
    choice = sys.argv[1] if len(sys.argv) > 1 else (input("\nEnter choice (1-5) [default: 1]: ").strip() or "1")
    selected = SCENARIOS.get(choice, SCENARIOS["1"])
    
    prompt = (
        "<|im_start|>system\n"
        "You are the System-2 Deliberative Swarm Orchestrator for agricultural robots in a rice terrace.\n"
        "Your role is to reason through potential robot immobilizations, resolve sensor ambiguities, protect valuable seed-carrying rovers, and choose whether to execute a physical recovery or safely refuse.\n"
        "<|im_end|>\n"
        f"<|im_start|>user\nNEW INCIDENT TELEMETRY DETECTED:\n{selected['json']}\n\n"
        "Analyze this incident step-by-step using ReAct. Call the required inspection tools, assess farmer policy risks, and output your final mission decision.\n<|im_end|>\n"
        "<|im_start|>assistant\n"
    )
    
    print(f"\n[1/2] Connecting to deployed Modal GPU endpoint...")
    backend = ModalSLMBackend()
    
    print(f"[2/2] Generating ReAct decision for: {selected['name']}...\n")
    t0 = time.time()
    response = backend.call(prompt)
    dt = time.time() - t0
    
    print("-" * 72)
    print("MODEL OUTPUT (Latency: {:.2f}s):".format(dt))
    print("-" * 72)
    print(response or "[No response received]")
    print("-" * 72)

if __name__ == "__main__":
    main()
