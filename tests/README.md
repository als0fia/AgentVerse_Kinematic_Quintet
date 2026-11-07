# Test conventions and evidence boundaries

Run the checked-in Python suite from the repository root with Python 3.11+ after installing the pinned MuJoCo dependency:

```bash
python -m pip install -r requirements-mujoco.txt
python scripts/check_repo.py
python scripts/sync_skills.py --check
python -m unittest discover -s tests -v
```

GitHub Actions runs these checks with Python 3.12. A successful Actions run verifies the checked-in Python assertions and MuJoCo headless tests on that commit; it does **not** verify a local graphical demo, an Isaac Sim scene, hardware or the full end-to-end Antenna mission.

## What the tests cover

- `test_repo_check.py`, `test_skill_interop.py`, `test_component_docs.py`: repository/skill/documentation consistency.
- `test_evaluate.py`: the episode evaluator and its **synthetic** `fixtures/` input data. These fixtures are not product-performance results.
- `test_ricebot_foundation.py`, `test_agent_protocol.py`: typed `rice.v1` contracts, scripted sensor/action decisions, control/mission safety and protocol behaviour, without Isaac Sim.
- `test_world_and_robot.py`, `test_mujoco_sim.py`: field/world, robot embodiment and MuJoCo simulation integration, including the separately scripted rigid-docking/tandem extraction test. A measured extraction in that test is **mechanics evidence**, not autonomous rescue.
- `test_agentic_swarm.py`: centralised Policy A task bidding and heuristic rescue triage using controlled inputs.
- `test_mycelial_swarm.py`: simulated per-rover Policy B beliefs, gossip/blackouts and task changes in one simulation process; not deployed radios or distributed hardware.

## What is still unverified

There is no passing test of the **integrated** fault-detection → contextual rescue-or-abstain decision → bounded physical recovery actuation → independent recovery/second-robot safety check → continued rice-field work episode. Nor do these tests establish trained rescue control, locally deployed Jetson inference, wireless UWB/Wi-Fi HaLow hardware, real agronomic efficacy or parity with Isaac Sim.

For new functionality, add a deterministic regression scenario that observes the resulting simulated world state independently of the planner's own claims. Keep test fixture provenance, actual command/output, simulator version, seed, baseline and failure limitations visible in the PR. See [TESTING](../docs/TESTING.md) and [DEMO](../docs/DEMO.md) for judge-facing evidence requirements.
