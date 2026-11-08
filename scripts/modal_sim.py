"""Modal cloud runner for MuJoCo 3D Rice Terrace and Swarm Simulations.

Spawns isolated cloud containers to run headless physics episodes, batch sweeps,
and scenario evaluations in parallel across your Modal compute pool.
"""
from pathlib import Path
import modal

ROOT = Path(__file__).resolve().parents[1]

app = modal.App("antenna-swarm-sim")

# Container image with Linux MuJoCo and code mounts
sim_image = (
    modal.Image.debian_slim(python_version="3.11")
    .pip_install("mujoco>=3.2.0", "numpy>=1.24.0")
    .add_local_dir(str(ROOT / "src"), remote_path="/root/src")
    .add_local_dir(str(ROOT / "scripts"), remote_path="/root/scripts")
)


@app.function(image=sim_image, timeout=120)
def run_cloud_simulation(scenario: str = "golden_rescue", policy: str = "cooperative", seed: int = 7, steps: int = 195) -> dict:
    """Runs a single 3D MuJoCo episode in an isolated cloud worker."""
    import sys
    sys.path.insert(0, "/root/src")
    sys.path.insert(0, "/root/scripts")

    from run_mujoco_paddypatch import run_demo

    res = run_demo(
        scenario=scenario,
        policy=policy,
        seed=seed,
        steps=steps,
        viewer_seconds=0.0,
    )

    return {
        "scenario": res["scenario"],
        "policy": res["policy"],
        "seed": res["seed"],
        "steps": res["steps"],
        "viable_seed_placements": res["viable_seed_placements"],
        "completed_recoveries": len(res["completed_recoveries"]),
        "recoveries": res["completed_recoveries"],
        "total_travel_m": res["total_travel_m"],
    }


@app.function(image=sim_image, timeout=120)
def run_incline_climb(ramp_angle_deg: float = 22.0) -> dict:
    """Simulates the 2-tier terraced earthen bund ramp ascent in the cloud."""
    import sys
    sys.path.insert(0, "/root/src")
    sys.path.insert(0, "/root/scripts")

    import mujoco
    from simulate_terrace_incline import build_terrace_incline_world_xml

    xml_str = build_terrace_incline_world_xml(ramp_angle_deg=ramp_angle_deg)
    model = mujoco.MjModel.from_xml_string(xml_str)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    z0 = float(data.sensor("planter_pos").data[2])
    act_fl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "planter_act_fl")
    act_fr = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "planter_act_fr")
    act_rl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "planter_act_rl")
    act_rr = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "planter_act_rr")

    for act in (act_fl, act_fr, act_rl, act_rr):
        data.ctrl[act] = 12.0

    for _ in range(750):
        mujoco.mj_step(model, data)

    z_final = float(data.sensor("planter_pos").data[2])
    elevation_gain_mm = (z_final - z0) * 1000.0

    return {
        "ramp_angle_deg": ramp_angle_deg,
        "initial_z_m": round(z0, 3),
        "final_z_m": round(z_final, 3),
        "elevation_gain_mm": round(elevation_gain_mm, 1),
        "climb_success": elevation_gain_mm > 50.0,
    }


@app.local_entrypoint()
def main():
    print("\n" + "=" * 70)
    print(" SPAWNING CLOUD SIMULATION SPACES ON MODAL (ananya4231)")
    print("=" * 70)

    # 1. Run Golden Rescue episode in Modal Cloud
    print("\n-> Launching Golden Rescue on remote Modal worker...")
    rescue_res = run_cloud_simulation.remote(scenario="golden_rescue", policy="cooperative", seed=7, steps=195)
    print(f"   Status: Complete!")
    print(f"   Recoveries Completed: {rescue_res['completed_recoveries']}")
    print(f"   Viable Seeds Planted: {rescue_res['viable_seed_placements']}/2")
    if rescue_res['recoveries']:
        rec = rescue_res['recoveries'][0]
        print(f"   Rescue Details: Helper {rec.get('helper_robot_id')} extracted victim {rec.get('target_robot_id')} ({rec.get('victim_displacement_m')}m)")

    # 2. Run 22-degree Terrace Incline Ramp Climb in Modal Cloud
    print("\n-> Launching 22-degree Terrace Incline Climb on remote Modal worker...")
    climb_res = run_incline_climb.remote(ramp_angle_deg=22.0)
    print(f"   Status: Elevation Gain = +{climb_res['elevation_gain_mm']} mm (Climb Success: {climb_res['climb_success']})")

    # 3. Parallel Batch Sweep across scenarios concurrently in Modal Cloud
    print("\n-> Fanning out 3 scenarios concurrently in parallel on Modal...")
    scenarios = ["nominal", "bad_soil", "golden_rescue"]
    results = list(run_cloud_simulation.map(scenarios))
    for r in results:
        print(f"   [{r['scenario'].upper()}]: recoveries={r['completed_recoveries']}, planted={r['viable_seed_placements']}, travel={r['total_travel_m']}m")

    print("\n" + "=" * 70)
    print(" ALL MODAL CLOUD WORKERS EXECUTED CLEANLY!")
    print("=" * 70 + "\n")
