# Component ownership, replaceability and audit standard

**Alternative versions and reconciliation:** See [SIMULATOR-VARIANTS](SIMULATOR-VARIANTS.md) for the pinned feature-source catalog and workflow to port useful capabilities from other team branches or independent frontends. This guide continues to own **module boundaries and contracts**; the variant registry owns **where alternative implementations live**. A source branch is not a replacement for a module contract or a second canonical runtime. For any world, robot, controller or GUI feature migration, use the [simulation/perception/viewer audit](audits/2026-11-07-simulation-perception-viewer.md) as a current-SHA-verification checklist, not as timeless test results.

**Applies to all meaningful PaddyPatch modules and future features.** This is the canonical guide for **program sections** (world, robot, mapping, controls, evaluation), not Git branches. [TEAM](TEAM.md) governs Git collaboration; [CONTRACTS](CONTRACTS.md) defines the real `rice.v1` data model. Read only the relevant section and component README for the task. Do **not** add an agent file per function or a new framework for documentation.

## Current source versus proposed functionality

As of the `rice.v1` kernel integration, `src/ricebot/contracts.py`, `mission.py`, `control.py`, `runtime.py` and scripted-observation unit tests **exist**. They provide typed messages, simple measured field-map baseline, intent/command generation and a simulation **Protocol**. **Real Isaac Sim rover physics, world, soil probes, seed-drop/contact, independent evaluator and second scout are not yet evidenced by these tests.** First verify latest code/branch before restating this snapshot. Independently created local MuJoCo sandboxes are prototypes, not shared integrated code.

## Runtime ownership — five separable blocks, one integrator

| Area | Intended owned paths (create only when implementing) | Owns | Never independently owns |
| --- | --- | --- | --- |
| **Kernel/integrator** | `src/ricebot/contracts.py`, `runtime.py` and composition runner | Public `SimulationPort` / frame/command semantics; episode orchestration; compatibility fixtures | Robot joint modelling or soil ground truth |
| **World and sensor host** | `src/ricebot/world/`, scene/world assets, scoped world tests | Tiny field (3–6 sites), public lanes/no-go, hidden synthetic spatial soil/stand distribution, local location-gated noisy probes, mid-run scene disturbances | Chassis/hopper design, independent mission policy, changing shared `rice.v1` schema |
| **Robot embodiment** | `src/ricebot/robot/`, robot assets and scoped robot tests | Chassis and joints, wheel motion bounds, geometry/collision, mount frames, seed hopper/release mechanism, physical actuation feedback | Soil generator, mission planner or application runner |
| **Isaac adapter/composition** | `src/ricebot/isaac/`, scene launcher and integration tests | **One** integration writer constructs world + rover + Isaac physics and implements actual `SimulationPort.reset/step`; translates bounded commands to joints and sampled sensors/physics contact to frames | Own separate `rice.v1` clone; mock-success animation; rewriting other owners' assets |
| **Intelligence and evaluator** | `src/ricebot/agent/` and `evaluation/`, `viewer/` (distinct owners) | Shared estimated map, assignment/route/site decisions; independent hidden-truth grading, baselines, read-only displays | Privileged world state in planner or UI driving physics |

The human and model split is **interchangeable**: one teammate may use ChatGPT for world and another Gemini for robot, **or vice versa**. Neither tool gets exclusive architectural authority from its brand. Name one human owner per block and one **kernel/Isaac integration owner** for shared seams; roles can be exchanged explicitly without moving all files. The [assigned GitHub issues](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/issues) and latest PRs supersede the illustrative paths when more specific.

### The actual swap boundary: do not invent another kernel

The **existing** public interface is:

```python
class SimulationPort(Protocol):
    def reset(self, seed: int) -> ObservationFrame: ...
    def step(self, commands: tuple[RobotCommand, ...]) -> ObservationFrame: ...
```

See [actual code](../src/ricebot/runtime.py) and [message fields](../src/ricebot/contracts.py). The `SimulationPort` **world/physics adapter** builds both rover bodies and supplies real observations; `RobotCommand` is high-level bounded motion/plant intent, **not necessarily a direct wheel torque**. A narrow robot-internal adapter may map those commands to real joints/hopper actuators, but it must not define a second episode scheduler, field oracle or mission agent.

Before implementation, world+robot developers and integrator agree the **smallest** fixture:
- World coordinate convention, metres/radians/seconds and timestamp behaviour.
- Unique robot IDs, planter vs scout capability, attached sensor naming/frame and sample radius.
- Asset scale/footprint/turning radius and accepted clearance for 3–6 sites.
- Drive velocity/angular and safe-stop bounds (controller vs actual wheel interface).
- Hopper mounting/release frame, launch geometry, seed proxy shape and **measured physics landing/contact**.
- What is reported as observation vs what remains **hidden evaluator-only world truth**.
- Deterministic reset/step and single unexpected mid-run obstacle/site change.

Record agreed names/units in the **existing** contracts/fixture via the sole kernel owner. Specialists propose deltas; they must not independently introduce duplicate message types.

### Mock rover first → approved designed rover next

The world developer may progress using a **small physically actuated** mock rover with bounded wheel drive, front range/odometry, IDs, sensor mounts and a basic physical seed release. The robot developer builds the approved replacement independently in a stable reference scene. When connecting, the integration owner swaps **robot assets/actuator mappings**, not the mission core or world. This is **replaceability of the software seam**, not an assertion that the new robot will move/plant identically: mass, inertia, clearance, radius, reach and collision constraints require retesting.

Minimum side-by-side mock/real acceptance: same `SimulationPort` frame/commands; physical wheel motion; changing physics-based range/odometry after surprise; invalid/stale command safe HOLD; capability check (scout cannot plant); real hopper release and independently observed landing/contact; no ghost inventory decrement; consistent frame/time/ID and hidden truth blocked from agent. If hardware/Isaac is unavailable, label test **fixture-only / MuJoCo-only / not run on Isaac**.

## Small component README (when real code exists)

A meaningful implemented module such as `src/ricebot/world/` or `src/ricebot/robot/` should have a short, maintained `README.md` covering **only**:
1. **Purpose / intended beneficiary outcome / current scope:** what actually works versus planned and explicitly out-of-scope.
2. **Human owner, agent/tool and upstream/downstream boundaries:** source of truth for contract version and known consumers; ownership assignments may change without rewrites.
3. **Public interface:** imports, parameter names/types, units, coordinate frames, time/TTL, validity, errors, no oracle path; link to `docs/CONTRACTS.md` rather than copying its schema.
4. **Setup and exact run/test commands:** required actual simulator/version/asset licence, sample fixture/seed, expected signal and how verified; never mark unrun tests as passed.
5. **Invariants and failures:** reset, bounded control, uncertainty/missing data, seed contact and safe-stop where relevant; known limitations.
6. **Extending/replacing:** which files the owner edits and what integration/checks are required for a swap.

**No empty READMEs or nested AGENTS files in directories that have no implementation.** Only introduce a component-local `AGENTS.md` when its agent editing rules genuinely differ from root `AGENTS.md`; local READMEs teach humans and all models. Keep comments/docstrings useful and focused rather than producing long routine call logs or duplicating the code line-by-line.

### Definition of done for a substantial feature

- Scoped feature performs a **measurable product job** and has an agreed owner.
- Public contract and all known consumers still agree; no duplicate field/task/world truth.
- New tests cover normal, invalid and failure cases; actual local checks and any absent integration checks are both reported.
- Component README/docstrings describe the **implementation that exists**, not a wished-for one; source licences and limitations identified.
- Relevant real robot/scene checks added when the relevant physics host exists, with independent contact/observation evidence.
- PR/handoff names exact changed paths, observed commands/results, meaningful trace/seed, integration dependencies and unverified claims.
- Fast lane: a truly small isolated fix may update only the existing docs if affected; **no new planning document or bureaucracy** for every edit.

## Repeatable independent audit (any feature, on demand)

Use the [module-audit skill](../.agents/skills/module-audit/SKILL.md) when asked to *audit, streamline, split, reassign or review* a module. By default **audit only**, not mass refactor. Compare fresh main/feature diff and authoritative mission requirements, then:

1. **Reconstruct:** actual imports, callers, tests, recent PR and owner. Separate verified code, mocks, hypothetical docs and obsolete statements.
2. **Correctness:** nominal/failure paths, reproducible inputs, off-by-one, timing, invariants, deterministic resets; run tests when a real runtime is available.
3. **Modularity:** dependencies and cycles, side effects, duplicated runtime/physics state, replaceability, path ownership and hidden coupling.
4. **Interfaces:** frame/units/actor/time/TTL/validity/unknown/error semantics; code/test/docs consistency at both producer and consumer.
5. **Concurrency:** whether ChatGPT/Gemini/humans can independently own paths and fixtures; propose a one-writer seam where conflicts exist.
6. **Embodied evidence (where relevant):** no oracle samples, actual motor/seed physics, new obstacle perceived during one episode, safe HOLD on bad commands, independent grader and matched baseline.
7. **Documentation & readiness:** component README, exact runnable setup/commands, tests really observed, licence/provenance, next owner and blockers.

Output **critical / major / minor findings with file/line and proof**, exact unrun checks, smallest fix, owner, integration dependency and **ready / conditional / blocked**. Finding no evidence of failure is not a pass. Only make edits when the user explicitly authorises implementation; never let an audit silently rewrite a teammate's subsystem.

## Handoff and order of operations

1. Kernel integrator supplies one **tested** `rice.v1` fixture and code snapshot.
2. World developer builds tiny field and physical mock rover; robot developer builds the real articulated asset/reference scene **in parallel**, each on owned paths; both send dimension/coordinate/sensor/hopper seam details.
3. **Current P0:** one MuJoCo integrator reuses both existing modules and `SimulationPort` (no parallel kernel) to connect detected fault, bounded rigid-dock recovery and independent resumed fieldwork. The tested seed-drop remains a regression. A later Isaac adapter is only a separately tested translation.
4. Map/scout and evaluator consume valid observations and contact records; inject mid-run obstacle/new local sample.
5. Independent reviewer runs direct + combined tests, records matched-seed baseline, and keeps the ≤60-second continuous demo reproducible.

Use existing [TEAM collaboration](TEAM.md) for branch/PR mechanics and [workstation guide](WORKSTATION-SETUP.md) for each developer's local setup. A simulated field alone or a realistic model alone does **not** satisfy the selected product.
