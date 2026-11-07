# Antenna application core (rice.v1 + existing MuJoCo physics)

The `src/ricebot/` package is the canonical **Antenna** runtime boundary. Its contracts, map/planner/controller and MissionCore are standard-library Python; [the MuJoCo adapter](ricebot/mujoco_sim/README.md) already provides **tested physical rover/soil/seed physics**, and the robot module provides a **separate scripted rigid-docking extraction test**. Neither proves a full autonomous mission-driven rescue. Isaac Sim translation remains unimplemented.

- `contracts.py`: typed site, local sample, robot observation, task, bounded motion and seed feedback messages.
- `mission.py`: deduplicated **measured** local sample map and deterministic sample/plant/hold baseline.
- `control.py`: conservative local motion and release intentions, safe stops for missing range, stale tasks or protected straight-line routes.
- `runtime.py`: `SimulationPort` (reset/step), `ReadOnlyView`, and `MissionCore` coordinating one observation/action tick.

**Next developer:** implement one bounded **physical rescue** skill with the integrator and human mechanical owner in the **same** MuJoCo reset/step runtime. Consume a real sustained fault indication (not missing range or deliberate HOLD), choose a risk/abstain strategy, invoke the existing rigid latch/pitch-lift safely, verify physical victim displacement/helper risk and resume site/seed work. The new central planner detector now receives actual bounded motor commands through MissionCore and tests safe stops; it still does not command extraction or prove recovery. Do not duplicate the simulator or silently modify shared robot geometry. See [current RUN assignment](../docs/RUN.md#current-minimum-working-rescue-integration-assignment).


Run `python -m unittest discover -s tests -p 'test_ricebot_foundation.py' -v` from the repo root. These tests are **scripted contract checks**, not physical planting proof. See [RUN](../docs/RUN.md), [CONTRACTS](../docs/CONTRACTS.md) and [TEAM](../docs/TEAM.md).
