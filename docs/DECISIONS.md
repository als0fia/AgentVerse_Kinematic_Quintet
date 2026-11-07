# Decision log (architecture and product scope)

This records **durable rationale**, not a running task/status list. Put implementation tasks in issues and code evidence in PRs. Add entries when trade-offs affect other components or the product mission.

## D-0001 — Keep the foundation stack-neutral until challenge reveal

- **Date:** 7 November 2026
- **Status:** adopted for foundation preparation; product still unselected
- **Context:** The team repository initially contained only a README; the current robotics challenge and provided stack had not been verified.
- **Decision:** Bootstrap a small, cross-agent documentation/testing/CI layer with Python standard-library tooling, but no fabricated robot stack, API scaffolding or chosen idea.
- **Reason:** Prevent agents from building incompatible assumptions before team selection. Retain a runnable proof of repo health and a reusable evaluation adapter.
- **Trade-off:** Actual integration, simulator dependencies and application tests cannot be written accurately yet.
- **Revisit:** Immediately when organiser brief and team direction are known. Choose and document actual runtime and components then.

## D-0002 — Embodied loop required; simulator and mission still team decisions

- **Date:** 7 November 2026
- **Status:** adopted mandatory challenge constraints, **not a project/stack selection**
- **Source:** onsite organiser Robotics track slides 35–38 and recorded presenter explanation; technical details in [EVENT](EVENT.md).
- **Decision:** A submission must be evaluated as a closed simulated **sensor → decision → action → feedback** loop, including an observable unplanned change. Avoid a scripted sequence dressed as autonomy. Use local/inside-runtime core decisions where practical; isolate optional remote LLM deliberation from the short-horizon control loop.
- **Options open:** Isaac Sim headless stream on verified suitable compute, or MuJoCo/PyBullet/Gazebo/Webots where team resources make a faster deterministic first episode possible. No hardware, GPU, model credits, simulator or project is selected or verified in this record.
- **Engineering impact:** product selection needs a meaningful human problem; architecture must expose actual sensor and action interfaces; testing must compare fixed-path and disturbed episodes; demo needs one continuous ≤60s video and one-page technical PDF.
- **Trade-off:** premature advanced engine/cloud integration could consume the whole hackathon; fallback simulator and network-off tests preserve an achievable demo. The five official Robotics percentages are resolved from the clearer attendee photograph; account/credit details and selected product remain unverified.
- **Revisit:** once team selects mission, tests available compute and proves first simulated episode.

## Decision template

```text
D-XXXX — [Decision title]
Date / status / owners:
Problem and constraints:
Options considered and baseline:
Decision and why:
Interface/tests/risks affected:
Revisit trigger:
```
