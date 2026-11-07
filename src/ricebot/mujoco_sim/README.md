# MuJoCo Rapid-Prototyping Composition Adapter (`src/ricebot/mujoco_sim/`)

## 1. Purpose
Composes the simulator-independent **World & Sensor Host** ([src/ricebot/world/README.md](../world/README.md)) and **Robot Embodiment** ([src/ricebot/robot/README.md](../robot/README.md)) into a runnable **MuJoCo 4.0.0** `SimulationPort` backend (`MujocoPaddySimulation`) for rapid local prototyping on 8 GB VRAM machines before **proposed, separately verified** translation to `src/ricebot/isaac/`.

## 2. Public Entry Points
- `MujocoPaddySimulation` in `src/ricebot/mujoco_sim/adapter.py`
  - `.reset(seed: int) -> ObservationFrame`
  - `.step(commands: tuple[RobotCommand, ...]) -> ObservationFrame`
  - `.completed_drops -> tuple[SeedDropEvent, ...]`
- `compose_paddypatch_mjcf(obstacle_xy) -> str`

## 3. Input and Output Contracts Used
- Implements `SimulationPort` (`src/ricebot/runtime.py`) using `ObservationFrame`, `RobotObservation`, `RobotCommand`, and `PlantFeedback` from [docs/CONTRACTS.md](../../../docs/CONTRACTS.md).
- Delegates all soil/stand truth and location-gated sampling to `PaddyTerraceWorld` (`src/ricebot/world/`) and all 4-wheel skid-steer kinematics and rover geometry to `src/ricebot/robot/`.

## 4. Prohibited Dependencies
- Must **never** import or mutate `MissionCore` (`src/ricebot/mission.py`).
- Must **never** inline hidden soil truth or rover kinematics directly; those live in `src/ricebot/world/` and `src/ricebot/robot/` respectively per [docs/COMPONENTS.md](../../../docs/COMPONENTS.md).

## 5. Verification Commands
Requires **Python 3.11+** (MuJoCo 4.0.0) and the pinned optional dependency from repo root:

```powershell
python -m pip install -r requirements-mujoco.txt
python -m unittest discover -s tests -v
python scripts/run_mujoco_paddypatch.py --headless
```

The headless integration suite exercises MuJoCo only. It does **not** prove that the unimplemented Isaac adapter has matching physics or contact behavior.

## 6. Known Limitations
- MuJoCo models the paddy soil bed as a rigid high-friction plane (`friction="1.2 0.01 0.005"`) with location-gated cone-index firmness and moisture probes rather than CFD mud deformation.
