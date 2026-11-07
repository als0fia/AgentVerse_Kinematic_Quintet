# Antenna application core

The `src/ricebot/` package is a **standard-library Python contract and baseline**, not a working Isaac Sim simulation.

- `contracts.py`: typed site, local sample, robot observation, task, bounded motion and seed feedback messages.
- `mission.py`: deduplicated **measured** local sample map and deterministic sample/plant/hold baseline.
- `control.py`: conservative local motion and release intentions, safe stops for missing range, stale tasks or protected straight-line routes.
- `runtime.py`: `SimulationPort` (reset/step), `ReadOnlyView`, and `MissionCore` coordinating one observation/action tick.

**Next developer:** implement a *real Isaac Sim* `SimulationPort`, not a competing app architecture. Its actual USD/PhysX rover articulations and sensors must provide current, local measurements and observed physical drop/contact `PlantFeedback`; reject stale/invalid commands even if core policy fails. The hidden ideal soil/stand field stays evaluator-only.

Run `python -m unittest discover -s tests -p 'test_ricebot_foundation.py' -v` from the repo root. These tests are **scripted contract checks**, not physical planting proof. See [RUN](../docs/RUN.md), [CONTRACTS](../docs/CONTRACTS.md) and [TEAM](../docs/TEAM.md).
