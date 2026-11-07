# Robot Embodiment (`src/ricebot/robot/`)

## 1. Purpose
Owns the physical rover embodiment (`RoverChassisSpec`), 4-wheel differential skid-steer kinematics (`compute_4wheel_skid_steer`), forward 3-ray LiDAR and odometry sensor mount frames, seed-hopper release nozzle geometry, and robot-only articulation/MJCF builders (`build_rovers_and_seeds_xml`) for `planter` and `scout`.

## 2. Public Entry Points
- `RoverChassisSpec`, `PLANTER_SPEC`, `SCOUT_SPEC`, `DEFAULT_CAPABILITIES` in `src/ricebot/robot/rovers.py`
- `compute_4wheel_skid_steer(spec, linear_m_s, angular_rad_s) -> (w_fl, w_fr, w_rl, w_rr)`
- `build_rovers_and_seeds_xml() -> (bodies_xml, actuators_xml, sensors_xml)`

## 3. Input and Output Contracts Used
- Consumes bounded `(linear_m_s, angular_rad_s)` velocity commands from `RobotCommand` and outputs 4-wheel angular velocity targets in `rad/s`.
- Exports `RobotCapability` tuples (`DEFAULT_CAPABILITIES`) matching [docs/CONTRACTS.md](../../../docs/CONTRACTS.md).

## 4. Prohibited Dependencies
- Must **never** own field patch layouts, hidden soil truth, or location-gated soil sampling (those belong to [src/ricebot/world/README.md](../world/README.md)).
- Must **never** import `MissionCore` or modify mission planning rules.

## 5. Verification Command
```powershell
python -m unittest discover -s tests -p "test_world_and_robot.py" -v
```

## 6. Known Limitations
- Wheel slip amplification (`1.85x`) is calibrated for 4-wheel differential skid-steer on the flat paddy terrace plane (`friction="1.4 0.01 0.005"`). When translating to `src/ricebot/isaac/`, the same `compute_4wheel_skid_steer` helper maps directly to `ArticulationAction(joint_velocities=...)` on the 4 wheel revolute joints.
