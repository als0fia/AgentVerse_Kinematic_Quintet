# Runtime architecture

**State: stack-neutral scaffold, architecture not chosen.** After project selection, keep this as the sole current architectural description. Avoid creating multiple incompatible diagrams.

## Component map

| Component | Responsibility | Input contract | Output contract | Owner | Tested? |
| --- | --- | --- | --- | --- | --- |
| User / world interface | TBD | TBD | TBD | unassigned | no |
| Perception / state | Only if needed | TBD | TBD | unassigned | no |
| High-level reasoning | Only if AI adds value | TBD | TBD | unassigned | no |
| Deterministic executor | Validated actions / bounded control | TBD | TBD | unassigned | no |
| Independent verifier | Outcome and test instrumentation | TBD | TBD | unassigned | no |

Delete irrelevant rows when deciding the project. Component boundaries should match actual folders, public functions and run commands, not decorative boxes.

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
