# Antenna application source

**Authoritative submission runtime: [ricebot/](ricebot/).** The Antenna simulation uses the single MuJoCo world and EpisodeSession observation/action loop. The older seasonal farming prototype [riceswarm_ai/](riceswarm_ai/) is retained for historical provenance and existing regression tests; it is not part of the demonstrated autonomous rescue runtime.

## Main modules

- **ricebot/contracts.py** — typed observation, soil sample, task, bounded command and recovery feedback contracts.
- **ricebot/world/** — deterministic field/soil generator, randomized hazards, scalable physical field layout and planting sites.
- **ricebot/robot/** — articulated MuJoCo rover specification and rigid recovery mechanism assets.
- **ricebot/mujoco_sim/** — the one physical simulator, collision/contact, planting proxy and physical docking/extraction feedback.
- **ricebot/runtime.py** — EpisodeConfig and EpisodeSession, selected acting policy, reset and reproducible event loop.
- **ricebot/agent/** — CooperativeSwarmPlanner, DecentralizedMycelialRuntime with local maps/gossip, incident assessment, deterministic baseline, safety gate and optional explicitly enabled LLM backend.
- **ricebot/evaluation.py** — independent grading of observed physical outcomes against synthetic hidden soil truth.
- **ricebot/viewer/** — 3D MuJoCo plus 2D belief inspector, with a factual judge-facing Presentation Mode.

**Current demo:** the cooperative golden rescue fixture verifies simulated docking, victim extraction and resumed seed work; high-risk abstention is a separate refusal case. The expanded Mycelial fleet is tested with up to six robots, but recovery is scenario-dependent and collisions/secondary casualties occur in some study runs. The controller cannot read the evaluator's hidden soil truth.

From the repository root:

~~~powershell
python -m pip install -r requirements-mujoco.txt
python scripts/run_mujoco_paddypatch.py --presentation --scenario golden_rescue --policy cooperative --seed 7
python -m unittest discover -s tests -v
~~~

Read the [current project README](../README.md), [architecture](../docs/ARCHITECTURE.md), [launch instructions](../docs/RUN.md), [test evidence](../docs/TESTING.md) and [provenance](../docs/PROVENANCE.md). Historical agent scopes in earlier commits are no longer the current implementation.
