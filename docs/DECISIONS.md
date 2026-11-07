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
