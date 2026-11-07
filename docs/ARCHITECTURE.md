# Runtime architecture

**State: Robotics challenge confirmed; simulator and product architecture remain unselected.** After project selection, keep this as the sole current architectural description. Avoid creating multiple incompatible diagrams.

## Component map

| Component | Responsibility | Input contract | Output contract | Owner | Tested? |
| --- | --- | --- | --- | --- | --- |
| User / world interface | TBD | TBD | TBD | unassigned | no |
| Perception / state | Only if needed | TBD | TBD | unassigned | no |
| High-level reasoning | Only if AI adds value | TBD | TBD | unassigned | no |
| Deterministic executor | Validated actions / bounded control | TBD | TBD | unassigned | no |
| Independent verifier | Outcome and test instrumentation | TBD | TBD | unassigned | no |

Delete irrelevant rows when deciding the project. Component boundaries should match actual folders, public functions and run commands, not decorative boxes.

## Minimum required embodied loop (challenge constraint, not product selection)

```text
simulated world -> simulated sensor(s) -> perception / state estimate
      ^                                     |
      |                                     v
 outcome / new observation <- bounded action <- decision / replanning
      |                                     ^
      +-- independent evaluator + disturbance injector + trace
```

A valid first episode must visibly **use at least one sensor input**, make an observable decision that is **not just a fixed waypoint script**, execute an action, and react to an **unexpected mid-run change** (for example a blocked route or displaced object). Keep an independent observer for success, perception/action frames and latency. **Avoid giving the agent perfect ground-truth simulator state if the submission claims sensor-based perception**; ground truth may be accessed by the evaluator only.

The recommended control topology keeps the **time-sensitive action loop local to the simulator / robot runtime**, with explicit bounds, timeouts and graceful stop/re-plan. An optional remote reasoning model must have a narrow, infrequent role whose loss does not silently disable every decision. A remote *simulator/stream* (e.g. hosted Isaac Sim) is distinct from a remote *decision service*. Record both separately in RUN when selected.

The simulation engine is undecided: NVIDIA Isaac Sim is organiser-recommended, but **MuJoCo / PyBullet / Gazebo / Webots** are organiser-named alternatives. Prefer the one that yields an observable running episode earliest on the actual team's resources; avoid costly GPU provisioning until compatibility, credits, streaming security and fallback are established.

## Required design information after decision

- **Data and control flow:** observation → optional AI decision → validation → bounded execution → measured outcome; show retries, human override and failure exits.
- **Runtime topology:** executable entrypoints, model/server/process placement, data sources, simulator/robot if any, ports, external APIs and trust boundaries.
- **Units and frames:** metres vs centimetres, radians vs degrees, body/world/camera frames, timestamp origin, coordinate handedness, seed format. Never assume a convention.
- **Performance budgets:** intended latency for control loops vs deliberation, compute and monetary budgets; measure observed values separately.
- **Safety:** tool allowlist, action limits, invalid-state handling, timeouts, permission boundary, kill switch for physical motion.
- **Operational recovery:** expected failures, restart/reset path, deterministic fixtures, fallback seed, clean reproduction command.
- **Versioning:** any incompatible inter-module change updates `docs/CONTRACTS.md`, fixtures/tests, callers, and the integration owner before merge.

## Engineering principles

1. Start with a walking skeleton: input → at least one actual operation → independent observable result.
2. Hide optional LLMs, VLMs and simulation behind narrow replaceable interfaces; they may fail or be offline.
3. Keep hard realtime/safety control in a deterministic controller rather than an unbounded language-model loop.
4. Make state transitions and errors machine-inspectable (structured logs, IDs, timestamps and seeds), not just natural-language narration.
5. Test the seams between modules before polishing internal implementations.
