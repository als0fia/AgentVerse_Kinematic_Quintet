# Antenna
### Resilient collective robotics for rice-field recovery
**The Kinematic Quintet · AgentVerse 2026 · Robotics / Embodied AI**

Antenna is a MuJoCo simulation of cooperating agricultural rovers that **observe field conditions, allocate sensing and planting work, and decide when rescuing an immobilised teammate is worth the risk**. A stranded robot can lose its remaining work; a poorly judged rescue can strand a second robot. Antenna makes this trade-off observable through a perception–decision–physical-action loop.

**This repository contains the simulation and experimental implementation.** It does not claim a deployed agricultural robot, field-validated mud physics, physical radios, or a proven live-LLM advantage.

## Run the actual demonstration

Presentation Mode is a view of the **same live MuJoCo episode and robot observations** as the engineering dashboard. Its camera framing, rover trails and event captions are derived from the ongoing simulation.

Install Python 3.11+ and launch from the repository root on a graphical computer:

~~~powershell
python -m pip install -r requirements-mujoco.txt
python scripts/run_mujoco_paddypatch.py --presentation --scenario golden_rescue --policy cooperative --rescue-policy react_orchestrator --seed 7
~~~

The controlled **golden rescue** exercises a stranded rover, bounded peer docking/extraction and continued planting. To show the **different safe decision** when a rescue would endanger the helper:

~~~powershell
python scripts/run_mujoco_paddypatch.py --presentation --scenario high_risk_abstain --policy cooperative --seed 7
~~~

For fleet scaling, randomized maps, private beliefs and modelled gossip:

~~~powershell
python scripts/run_mujoco_paddypatch.py --dashboard --scenario randomized --policy mycelial --seed 8
~~~

In the dashboard select 2, 4 or 6 rovers, choose field scale 1.0× or 1.6×, and apply. The expanded field includes additional physical planting sites and equally equipped sensing/planting rovers. The standard two-rover fixtures retain historical scenario-specific inventories.

**See [run instructions](docs/RUN.md) and the [submission/demo checklist](docs/SUBMISSION.md).**

## System architecture

~~~text
Seeded MuJoCo world and physical rover models
                  |
   simulated soil/range/slip/peer observations
                  |
        EpisodeSession (runtime.py)
                  |
      selected acting coordination policy
    cooperative/shared map     OR     Mycelial/private beliefs
                                    + delayed peer gossip
                  |
         incident and mission context
       bounded rescue-risk/orchestration
         deterministic safety gate
                  |
       typed drive / probe / plant /
        dock / latch / extract commands
                  |
      simulated physical consequences
                  |
     independent evaluator + read-only UI
~~~

- **src/ricebot/world/** — deterministic soil, sites, hazards and physical field layout.
- **src/ricebot/mujoco_sim/** and **src/ricebot/robot/** — physical world, articulated rovers, planting and recovery hardware.
- **src/ricebot/agent/** — cooperative and decentralized policies, local beliefs, gossip, fault evidence and safe rescue decisions.
- **src/ricebot/runtime.py** — the authoritative observation/action episode loop.
- **src/ricebot/evaluation.py**, **scripts/** and **src/ricebot/viewer/** — independent outcomes, reproducible experiments and live presentation.

See the [architecture map](docs/ARCHITECTURE.md).

## Results that were actually observed

The independent verification branch (commit **32ce0a3**) ran on **Windows / Python 3.13.14 / MuJoCo 4.0.0**: 142 tests passed with no skips, the desktop GUI was exercised, and 18 full simulated randomized episodes were recorded. This submission branch starts from later Presentation Mode commit **60c7283**, whose [Linux CI run passed 149 tests (two display-dependent skips)](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/actions/runs/42478808632).

The recorded experiment used seeds 3 and 8, fleets of 2/4/6 rovers, physical scale 1.6×, and matched Cooperative / Mycelial normal / Mycelial isolated controllers.

| Observed result | Scope |
| --- | --- |
| With gossip, more viable seeds planted in **5 of 6** matched pairs; one tie | Simulated randomized trials |
| Private-map disagreement lower with gossip in **6 of 6** pairs | Synthetic local belief maps |
| All rover instances moved; each episode physically planted **3–9** viable seeds | 18 continuous 240-tick MuJoCo episodes |
| Two four-rover Mycelial trials recorded collisions; one six-rover trial had a secondary casualty | Safety limitations, not hidden |
| **Zero successful live-LLM calls in this study** | No claim that larger LLMs improve performance |

These findings are encouraging experimental observations, **not** statistical proof of real-world performance, networking throughput or agronomic yield. See [testing and limitations](docs/TESTING.md) and the [18-run machine-readable evidence](experiments/release_verification_20261108.json).

## Reproduce the evaluation

~~~powershell
python scripts/check_repo.py
python scripts/sync_skills.py --check
python -m unittest discover -s tests -v
python scripts/run_mujoco_paddypatch.py --scenario golden_rescue --policy cooperative --seed 7 --steps 220 --headless
python scripts/run_scaled_field_study.py --seeds 3 8 --fleets 2 4 6 --scales 1.6 --policies mycelial cooperative --comms normal isolated --rescue-policies react_orchestrator --steps 240 --out artifacts/release-study.json
~~~

The full 18-trial study takes longer than a focused smoke test. Generated logs go to gitignored **artifacts/**. Live model experiments require separate opt-in; API keys are never stored in the repository.

## Scope and provenance

Antenna implements simulated perception, real MuJoCo-controlled actions, bounded recovery skills, safety refusal, peer mapping and reproducible fieldwork. Its soil hazards are procedurally configured with physical traction effects, not an externally validated spontaneous geology model. Robotics hardware, radio transceivers, deployed decentralized processes, agricultural productivity and live-model superiority remain unverified.

The historical farm lifecycle prototype in **src/riceswarm_ai/** is retained for regression and provenance only. **src/ricebot/** is the single authoritative submission runtime. Parallel human/AI-assisted development and third-party dependencies are described in [provenance](docs/PROVENANCE.md).

**The repository alone is not a submission receipt.** The organiser separately requests a one-page technical PDF and a continuous simulation video of up to one minute, plus a live five-minute pitch if shortlisted. The team must verify portal requirements, approve and complete the actual entry: [submission checklist](docs/SUBMISSION.md).
