"""Run: PYTHONPATH=sample python -m riceswarm_ai.cli --output result.json"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from .core import RiceSwarmSimulation, RecoveryWeights, RecoveryPolicy


def draw_snapshot(sim: RiceSwarmSimulation, path: str) -> None:
    """Optional static image using matplotlib; MuJoCo not required."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    field = sim.field
    fig, ax = plt.subplots(figsize=(9, 7))
    palette = {'firm': '#d9e6bd', 'soft': '#9dc9db', 'blocked': '#595853',
               'unknown': '#f1f0ea'}
    for (x, y), cell in field.belief.items():
        ax.add_patch(Rectangle((x, y), 1, 1, color=palette[cell.terrain.value],
                               ec='#acb6a4', linewidth=0.4))
    for x, y in field.crop_targets:
        if (x, y) in field.harvested:
            lbl = 'H'
        elif (x, y) in field.planted:
            lbl = 'P'
        else:
            lbl = '·'
        ax.text(x+0.5, y+0.5, lbl, ha='center', va='center', fontsize=12)
    for r in sim.robots.values():
        ax.plot(r.pos[0]+0.5, r.pos[1]+0.5, 'o', markersize=18)
        ax.text(r.pos[0]+0.5, r.pos[1]+0.5, r.name, ha='center', va='center',
                fontsize=8, color='white', weight='bold')
    ax.set_xlim(0, field.width)
    ax.set_ylim(0, field.height)
    ax.set_xticks(range(field.width+1))
    ax.set_yticks(range(field.height+1))
    ax.set_aspect('equal')
    ax.set_title(f'RiceSwarm AI | Day {sim.current_day} | Planted {len(field.planted)}/{len(field.crop_targets)} | Harvested {len(field.harvested)}')
    ax.grid(False)
    fig.tight_layout()
    fig.savefig(path, dpi=145)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description='RiceSwarm AI Complete Lifecycle Demo')
    parser.add_argument('--output', default='riceswarm_result.json', help='JSON event trace')
    parser.add_argument('--png', default=None, help='Optional last-frame map PNG')
    parser.add_argument('--gui', action='store_true', help='Launch interactive 2D tkinter visualisation')
    parser.add_argument('--season', action='store_true', help='Run full condition-based agricultural season (Days -25 to 135+)')
    parser.add_argument('--days', type=int, default=135, help='Target day for season simulation (default: 135)')
    parser.add_argument('--no-mud', action='store_true', help='Run no-failure baseline')
    parser.add_argument('--mujoco', action='store_true', help='Compile and update mocap MuJoCo scene')
    parser.add_argument('--xml', default=None, help='Export standalone MJCF scene (MuJoCo not required)')
    parser.add_argument('--gemini', action='store_true', help='Use optional Gemini recovery adviser')
    parser.add_argument('--time-weight', type=float, default=0.25)
    parser.add_argument('--energy-weight', type=float, default=0.20)
    parser.add_argument('--crop-weight', type=float, default=0.30)
    parser.add_argument('--soil-weight', type=float, default=0.15)
    parser.add_argument('--risk-weight', type=float, default=0.10)
    args = parser.parse_args()

    if args.gui:
        from .visualizer import run_gui
        run_gui(introduce_mud=not args.no_mud)
        return

    total = args.time_weight + args.energy_weight + args.crop_weight + args.soil_weight + args.risk_weight
    if total <= 0:
        total = 1.0
    weights = RecoveryWeights(
        round(args.time_weight / total, 4),
        round(args.energy_weight / total, 4),
        round(args.crop_weight / total, 4),
        round(args.soil_weight / total, 4),
        round(args.risk_weight / total, 4),
    )
    policy = RecoveryPolicy(weights)
    if args.gemini:
        from .ai_agent import AgentAdvisedRecoveryPolicy, GeminiRecoveryAdviser
        key = os.environ.get('GEMINI_API_KEY', '')
        model = os.environ.get('GEMINI_MODEL', '')
        if not key or not model:
            parser.error('For --gemini set GEMINI_API_KEY and GEMINI_MODEL')
        policy = AgentAdvisedRecoveryPolicy(GeminiRecoveryAdviser(key, model), weights)

    sim = RiceSwarmSimulation(introduce_mud=not args.no_mud, policy=policy, start_day=-25)

    if args.xml:
        from .mujoco_bridge import build_mjcf
        Path(args.xml).write_text(build_mjcf(sim.field), encoding='utf-8')
    if args.mujoco:
        from .mujoco_bridge import MujocoBridge
        bridge = MujocoBridge(sim)
        sim.observers.append(bridge.update)

    if args.season:
        print(f"[RiceSwarm] Simulating complete rice farming lifecycle (Days -25 to {args.days}+)...")
        result = sim.run_season(target_day=args.days)
    else:
        result = sim.run()

    if args.gemini:
        result['gemini_adviser_error'] = policy.last_agent_error

    Path(args.output).write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    if args.png:
        draw_snapshot(sim, args.png)

    summary_fields = {
        'phase': result['phase'],
        'day': result['day'],
        'weather': result['weather'],
        'ticks': result['ticks'],
        'crop_targets': result['crop_targets'],
        'planted': result['planted'],
        'weeded': result['weeded'],
        'harvested': result['harvested'],
        'section_a': result['section_a'],
        'section_b': result['section_b'],
        'rescue_attempts': result['rescue_attempts'],
        'successful_recoveries': result['successful_recoveries'],
        'human_escalations': result['human_escalations'],
        'unsafe_crop_movements': result['unsafe_crop_movements'],
    }
    print(json.dumps(summary_fields, indent=2))
    print(f'Event trace saved to: {args.output}')


if __name__ == '__main__':
    main()
