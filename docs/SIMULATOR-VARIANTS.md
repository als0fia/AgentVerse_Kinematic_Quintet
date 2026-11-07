# Simulator variants and feature reconciliation — Antenna

**Purpose:** Give every human and coding agent a discoverable map of *different contributors' implementations* without treating them as independent final products. When asked to “reconcile everything,” an integrator should **select and port valuable features according to the current product goal**, not combine entire programs or overwrite somebody's work.

**Authority:** [AGENTS](../AGENTS.md) → [PRODUCT](PRODUCT.md) / [DECISIONS](DECISIONS.md) for product goal → [ARCHITECTURE](ARCHITECTURE.md), [CONTRACTS](CONTRACTS.md) and [COMPONENTS](COMPONENTS.md) for owned seams → this **feature-source registry** for alternatives → [TESTING](TESTING.md), [simulation-boundary audit](audits/2026-11-07-simulation-perception-viewer.md) and [DEMO](DEMO.md) for evidence. **Only merged `main` code is canonical.** A branch README, mock application or isolated demo does not redefine the deployed/tested product.

**Inventory snapshot:** 7 November 2026, observed from [`main` `eb801ca`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/commit/eb801cab68ba5bd69006088152ac8c2d531081f7), after PRs [#48](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/48), [#50](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/50) and [#52](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/52). This is an **index, not a live branch monitor**. Sources can move; before acting, refresh SHA, PR status, code and test evidence. Commit author is not proof of current work ownership. No source files, tests or local MuJoCo runtime were exercised by this inventory.

## What counts as a “version”

Keep these distinctions; mixing them is the main failure mode:

- **Canonical application:** `src/ricebot/` with one MuJoCo `SimulationPort`, world/robot composition, agent, controller, evaluation and viewers. `scripts/run_mujoco_paddypatch.py` is the actual headless/3D episode entry point. There is **one physical world and one reset/step authority**.
- **Coordination choice within that application:** Policy A (`centralized`) and Policy B (`mycelial`, simulated per-robot maps and gossip) run against the same basic MuJoCo composition. **These are policy alternatives, not two separate physics simulators.** The `golden_rescue` episode has used the centralized coordinator; inspect the current source before making claims about another policy.
- **Experimental feature or separate scenario:** a standalone MuJoCo script may explore a different hill, mud model or rescue technique. Its attractive visuals or results do not become evidence for the canonical rescue episode without an actual adapter and same-scene validation.
- **Alternative UI/mock application:** may offer useful presentation and interaction design while retaining its **own artificial state transitions**. Reuse UI concepts/components, not its mock physics as robot observations.
- **Historical or superseded branch:** Git history preserves prior ideas, but its old source and instructions have no authority over current `main`.

**Feature provenance is the unit of integration**: identify a *behaviour*, its source commit and dependencies, not “merge branch X into everything.”

## Known sources and candidate features

| ID and source pinned at inventory | Capability worth inspecting | Integration status and boundaries | Safe disposition |
| --- | --- | --- | --- |
| **V0 — canonical** [`main` `eb801ca`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/commit/eb801cab68ba5bd69006088152ac8c2d531081f7), `src/ricebot/`, `scripts/run_mujoco_paddypatch.py`, `src/ricebot/viewer/live_app.py` | Two-rover MuJoCo rice-field task, built-in scenarios, golden rescue, simulated perception/probes, typed command/feedback, viewer, Policy A/B | **Merged; CI green** on this snapshot (78 tests in preceding integration checks). CI alone does not verify independent realistic traction or a recorded GUI run. PR #52 documents remaining perception/controller/viewer boundary risks. | **Keep** as single authority; new work integrates *into* its contracts, not by replacing it. |
| **V1 — post-M1 camera/ramp experiment** [`feat/m1-antenna-integrated-rescue` `c7e0465`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/commit/c7e04650650be465204fe42dd5375980d5caafe8), [delta since merged `9e847fd`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/compare/9e847fdf2130b27cef800abd8ad611ffa7a4aae6...c7e04650650be465204fe42dd5375980d5caafe8) | New ego-camera RGB-D ramp detection / visual servo, robot geometry/tests, changed `scripts/simulate_terrace_incline.py` | **Candidate post-merge work**, not the already merged #48 rescue integration. One newer commit modifies robot and standalone terrace script. Local visual/perception reliability not independently checked here. | **Assess separately** for actual rescue-demo need; respect robot-geometry owner. Port only a bounded camera/terrain sensing feature after testing its sensor provenance; do not swap the main scene. |
| **V2 — alternative React console** [`feature/user_interface` `0b177ef`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/commit/0b177efa844d89cc748df5a3798d77fff44a42de), `src/App.tsx`, `src/components/`, `src/logic/swarmEngine.ts`, `src/data/initialSwarmState.ts` | Multi-panel roster, scenario triage, map, telemetry, human controls; React/Vite front end | **Independent Git history (no common ancestor with main)**. Its TypeScript state/kinematics engine seeds five demo robots and drives UI logic; **not proven connected to MuJoCo**. Its own test claims were not executed here. | **Potentially port presentational ideas**, only through a read-only telemetry/supervised command interface. Never direct-merge unrelated history or replace MuJoCo state with `initialSwarmState`. |
| **V3 — alternate agent/rescue techniques** [`feature/agentic-swarm-intelligence` `bf55a79`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/commit/bf55a79c19939f240729239233c24deae9767fe4), notably `src/ricebot/agent/rescue.py` | Failure classification, modeled distress packets, multiple rescue techniques and actuator setpoints; other local map/agent edits | **Diverged after earlier agentic PR #35**; portions of its architecture are already on main, but `rescue.py` is absent from the checked main tree. Its alternative action/assessment types and direct actuator assumptions need contract and physics review. | **Extract specific justified behaviours only** after comparing against canonical `TaskMode`, `RescueAssessment`, `RobotCommand` and safety veto; do not duplicate the planner or write motors from the strategic layer. |
| **V4 — experimental rescue/climate scripts** [`docs/rescue-policy-math-and-improvisation` `0e67ef4`](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/commit/0e67ef48454328029d96951f8d327058717c5226), `scripts/eval_rescue_techniques.py`, `scripts/sim_extreme_climate_typhoon.py`, `docs/RESCUE_POLICY_RESEARCH.md` | Proposed rescue-technique comparisons, typhoon/rollover/righting cases and mathematical background | **Experimental, diverged branch**; alternative scenarios and claimed results are not verified against the canonical golden episode by this registry. Some strategy assumptions may conflict with M4 physics findings. | **Keep as research/negative-test candidates**. Validate physical assumptions, same-seed baselines and correct world/model ownership *before* importing a technique. |
| **V5 — abstract full-season simulation** [`src/riceswarm_ai/` on main](../src/riceswarm_ai/README.md) | Rice lifecycle, crop-stage and farm operations, legacy map/visualizer | **Merged but explicitly not the authoritative physical Antenna simulator**. Its abstract state transitions are not MuJoCo docking/contact/traction. | **Park** as reusable research/benchmark fixtures only; do not blend its lifecycle clock or imagined physical state into MuJoCo perception. |
| **V6 — separate terrace experiment already on main** [`scripts/simulate_terrace_incline.py`](../scripts/simulate_terrace_incline.py) | Alternative terraced hill/ramp movement scene | **Merged auxiliary script**, separate from `golden_rescue`; V1 contains later changes. | **Park** unless a product goal requires it; use as mechanical test input, not a second judge-facing simulation runtime. |
| **Review source — simulation boundary audit** [PR #52](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/52), [audit](audits/2026-11-07-simulation-perception-viewer.md) | Known risks at simulation ↔ sensor ↔ agent ↔ viewer boundaries, and a repair-agent handoff | **Merged review, not a simulator variant.** Findings are SHA-specific and may be fixed by later patches; revalidate them before action. | **Required gate** for any reconciliation touching observation, physics, rescue or viewer authority. |

Other old `docs/*`, `feature/homogeneous-robot-rigid-rescue`, `feature/riceswarm-mvp` and `feature/mujoco-simulation-port` branches include merged or superseded history. **Do not count branches as independent feature gaps.** Inspect all currently active branches, local worktrees, teammate PRs and changes after the pinned heads above; a new branch is not automatically registered here.

## Light feature record — update alongside a substantial branch/PR

Do **not** create one Markdown document per code version. For a materially new alternate implementation, update **one row in the table above** (or one short section here if it needs more explanation) and use the existing component README / PR for details. Minimum useful handoff:

```text
Feature ID / behaviour: e.g. camera-based ramp alignment (not "my whole simulator")
Human owner (if confirmed) and contact route: ... / unknown
Source branch + exact commit SHA + PR/issue: ...
Relative to main: already merged? genuinely new diff? common ancestor?
Entry point and affected paths: ...
World/robot/agent/controller/viewer/evaluator owner: ...
Which outputs are measured simulated sensors vs hidden truth vs operator injection: ...
Dependencies and conflicting contracts/models: ...
Proof actually run (host, engine version, command, seed, pass/fail/log/clip): ...
Known failures and unavailable verification: ...
Product goal supported and minimum observable acceptance criterion: ...
Recommendation: adopt / adapt / park / reject (decision maker + reason)
```

**When to update:** whenever a teammate publishes a substantial alternate physics scene, GUI, agent planner or recovery technique; on PR handoff; and when it is merged, superseded or discarded. Keep the old commit links for provenance. A tiny pure-logic fix does not require bookkeeping.

## “Reconcile everything” — mandatory integrator workflow

**0. Interpret the user's goal, not the branch list.** First read [PRODUCT](PRODUCT.md) / [DECISIONS](DECISIONS.md), the latest user requirements and [DECISION synthesis](checkpoints/2026-11-07-2021-GMT/DECISION.md). If no more specific user goal is given, optimize for **one Antenna rescue-first episode with credible perception, a risk-aware rescue/refusal decision, bounded physical MuJoCo action, independent grading, resumed rice work and a readable demo**. Do not interpret “everything” as “every old feature is required.” Report conflicts between goals and candidate code rather than silently picking a new product.

**1. Reconstruct what really exists.** Pin latest `main` and every source branch SHA; list open PRs, commit dates, active human owners and base ancestry. Treat an open/active branch as in progress unless its owner explicitly hands it off. Locate independent worktrees or unpushed local changes via their human owners; GitHub cannot reveal unsent work. Compare *unique feature commits* against their last integrated ancestor, not just `main...branch` (which can misleadingly show lots of previously merged changes). Use PR review and file history to identify features already on main.

**2. Produce a decision matrix *before* changing code.** For each genuinely new behaviour, fill:

| Feature | Human benefit and scenario | Source SHA / code / tests | Current equivalent | Contract/physics conflict | Decision and proof gate |
| --- | --- | --- | --- | --- | --- |
| Example: RGB-D ramp alignment | Better approach on inclined terrain | V1 camera/ramp delta | Existing simulated range/pose | Camera data provenance, scene geometry | **Adapt only if needed**; demonstrate live rendered input → bounded steering on main scene |
| Example: React triage display | Judges can read rescue rationale | V2 `src/components/` | Main Tk/MuJoCo viewer | React mock engine is independent | **Port display only**, consume canonical telemetry and test GUI/headless equivalence |

Use **adopt** (compatible, test-backed), **adapt** (port the useful piece), **park** (interesting but unnecessary or unverified), **reject** (duplicates, unsafe, violates scope). Surface incompatible assumptions, ownership and missing proof. Ask a human to decide unresolved mechanical/product trade-offs; do not claim all features can co-exist.

**3. Design a single target architecture and an incremental PR sequence.** Physics stays in **one MuJoCo model/data + reset/step owner**; terrain and hidden field truth belong to world/simulator/evaluator, not robot perception. Robot embodiment owns geometry, cameras and actuators; a typed, timestamped observation boundary exposes **only simulated permitted sensing** to local estimation and planner. A bounded controller executes commands; an LLM may only advise at a higher, validated level. Read-only viewers consume the *same* state/events; any operator intervention goes through an explicit logged scenario/command interface. Do not run a second world loop in a separate GUI or give an agent perfect qpos, mud-trap or recovery-outcome truth. [See audited invariants](audits/2026-11-07-simulation-perception-viewer.md).

**4. Integrate by feature, not by blanket merge.** Create a fresh integration worktree/branch from current `main`. First reconcile shared schemas with the **one assigned integrator**. For alternatives implementing the same behaviour, consider a *temporary* adapter/abstraction so they can be tested on the same `SimulationPort` interface; remove redundant logic after the winning implementation is proven. Port one bounded change with its tests, then rerun the affected episode. Use cherry-pick only for truly isolated compatible commits, and record original source SHA in the PR. Never bulk-merge an unrelated history UI branch, substitute a new physics engine, or add feature flags for every old version.

**5. Prove the combined result on one named source revision.** At minimum: existing Foundation CI and all unit/integration tests; exact local MuJoCo scene/seed; headless golden rescue and safe-HOLD/missing sensor negative controls; peak force, pitch/contact and second-casualty checks; a mid-run change with permitted sensor → decision → bounded action trace; actual post-event fieldwork; and **GUI and headless agreement** for the same seed, including the absence of unlogged viewer-side simulation mutation. Compare a competent deterministic baseline on matched episodes. If local or graphical tests cannot run, label them **NOT RUN** and leave mechanical claims unverified.

**6. Review, merge and archive decisions.** Each PR should name source IDs/SHAs, which features were *ported*, what was declined and why, exact tests and failures, risk and rollback. Independent reviewer checks the [boundary audit](audits/2026-11-07-simulation-perception-viewer.md). Only merge green, scoped PRs according to [TEAM](TEAM.md) and human owner agreement; do not overwrite active branches, delete originals, force-push, change visibility or submit without approval. After merge, update this registry row to **on main at commit X**, refresh component docs, and leave Git/PR history for the rejected or deferred alternatives.

### Minimal read-only discovery commands

From a local clone with an authorized `origin` remote:

```sh
git fetch origin
git status --short --branch
git branch -r --sort=-committerdate
git log --oneline --all -- src/ricebot/mujoco_sim/ src/ricebot/viewer/ scripts/
git log --oneline origin/main..origin/BRANCH_NAME
git diff --name-status origin/main...origin/BRANCH_NAME
# If a merged feature branch gained later work, compare from its known integrated head:
git diff 9e847fdf2130b27cef800abd8ad611ffa7a4aae6..c7e04650650be465204fe42dd5375980d5caafe8 -- src/ricebot/robot/ scripts/simulate_terrace_incline.py
```

Replace `BRANCH_NAME` with a real branch and verify both SHAs before trusting results. **Three-dot compare requires a common ancestor; `feature/user_interface` did not have one in this snapshot.** Inspect that branch in a separate read-only worktree or show exact files independently; port selected UI components through a new main-based branch rather than merging unrelated histories. GitHub's [branch comparison](https://docs.github.com/en/pull-requests/how-tos/commit-changes/comparing-commits) and [keeping a PR in sync](https://docs.github.com/en/pull-requests/how-tos/create-pull-requests/keeping-your-pull-request-in-sync-with-the-base-branch) describe supported comparison/reconciliation mechanics.

## Technical basis and why this is lightweight

This workflow deliberately combines **feature provenance + one canonical runtime + reviewed PRs**, rather than a second monorepo, giant source copies, an automated “merge every branch” bot or 30 permanent switches. It follows [GitHub's PR and code-review guidance](https://docs.github.com/en/pull-requests/reference/managing-and-standardizing-pull-requests), [Branch by Abstraction](https://martinfowler.com/bliki/BranchByAbstraction.html) for gradual temporary replacement behind an existing interface, and [feature-toggle lifecycle guidance](https://martinfowler.com/articles/feature-toggles.html) (switches themselves add complexity, so use only when a specific controlled comparison needs them). Important product decisions continue in the existing [decision log](DECISIONS.md), not a parallel ADR system.

**Definition of done for a reconciliation request:** (a) one chosen physical runtime, (b) a per-feature adopt/adapt/park/reject explanation tied to goals, (c) original branch/commit provenance, (d) combined source/test results and honest missing proof, (e) no active teammate overwrite, and (f) updated registry and canonical documentation. **Do not call a merge complete merely because Git reports no conflicts.**
