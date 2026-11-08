# Antenna documentation

This directory contains **current submission documentation** alongside the team's historical engineering and research records. Start with the documents below. Older checkpoint and implementation-plan documents remain for attribution and review but should **not** be used to describe the current working product.

## Start here — judge/reviewer path

1. [Product and beneficiary](PRODUCT.md): what the system does, the rescue dilemma, alternatives, scope and limitations.
2. [Architecture](ARCHITECTURE.md): one physical MuJoCo runtime, sensors, acting policies, recovery safeguards, evaluation and viewer.
3. [Run guide](RUN.md): Windows install, golden rescue, refusal, scaled swarm, actual GUI verification and troubleshooting.
4. [Measured testing](TESTING.md): Windows and CI verification, 18-trial fleet study, paired gossip results and known failures.
5. [Live demo](DEMO.md): operator walkthrough and what the audience should actually see.
6. [Submission checklist](SUBMISSION.md): one-page PDF, ≤60-second continuous video, portal constraints and human sign-off.
7. [Attribution](PROVENANCE.md): dependencies, AI-assisted work, pre-existing work concerns and incomplete licence review.

The [root README](../README.md) is the concise public-facing repository entry point.

## Technical reference

- [Contracts](CONTRACTS.md) and [component boundaries](COMPONENTS.md) for implementation details.
- [World, sensors and physical robotics](../src/ricebot/world/README.md) and [robot module](../src/ricebot/robot/README.md).
- [Diagnostics](DIAGNOSTICS.md) for debug traces; not a substitute for physics outcomes.
- [Event-specific Robotics requirements](EVENT.md), sourced from organiser materials available to the team.
- [Corrected sector/market evidence](COMMERCIALIZATION_CHINA.md), for qualified beneficiary context only.

## Development history / exploratory research

[Decision history](DECISIONS.md), [team collaboration notes](TEAM.md), [original research collection](RESEARCH.md), [simulator variants](SIMULATOR-VARIANTS.md), [parallel integration history](FIVE-AGENT-RECONCILIATION.md), [older audits](audits/), [seven-agent checkpoints](checkpoints/) and [deferred edge/Isaac proposals](SWARM_EDGE_PLAN.md) are **historical or speculative context**, not current release acceptance criteria.

The historical calendar prototype in [src/riceswarm_ai](../src/riceswarm_ai/) remains only for traceability and its separate tests. Do not cite its seasonal yield estimates as empirical Antenna results.

**Release principles:** keep one authoritative working simulation; differentiate real measured effects from programmed synthetic fixtures; disclose unsuccessful trials; do not infer real deployment, deployed radio or live-LLM benefit from code presence; do not place private participant records or secrets in the public repository.
