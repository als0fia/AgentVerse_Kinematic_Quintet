# Provenance, dependencies and AI assistance

**Antenna / The Kinematic Quintet / AgentVerse 2026.** This file identifies the provenance supported by the checked-in source and branch history. It is a disclosure record, **not a declaration that the organiser has approved the entry**.

## What is our implementation?

The submitted runtime is **src/ricebot/** and the MuJoCo model assets at **src/ricebot/robot/assets/**. The team integrated simulated sensors, a bounded observation/action loop, two coordination policies, an incident-oriented recovery controller, docking and extraction physics, a 3D/2D UI, and independent evaluation scripts. The model is a **simulation**; it is not a physical field robot.

The historical **src/riceswarm_ai/** agricultural-season prototype, early planning/research documents and branches may include work predating the final competition scope. They are kept for regression history and source traceability; do not describe the historical prototype as the evaluated Antenna runtime.

The precise author-by-author ownership and any event restrictions on reusing previously developed code must be checked by the human team against the organiser's final rules. Git history helps establish chronology; it does not itself prove a permission or a licence.

## External dependencies

| Dependency | Used for | Source and licence handling |
| --- | --- | --- |
| Python 3.11+ and standard library | Control flow, schemas, Tkinter GUI, tests and scripts | Python Software Foundation licence applies to the standard Python distribution; verify local bundled Tcl/Tk terms where distribution requires it |
| **MuJoCo 4.0.0** | Physical simulation, contact, articulated rover/docking model and renderer | Official project: https://github.com/google-deepmind/mujoco ; Apache-2.0 upstream, check distributed binary notices |
| **NumPy** (version not pinned by current requirements file) | Numerical arrays and field/heightgrid calculations | Official project: https://github.com/numpy/numpy ; upstream BSD licence, verify installed version and notices |

The reproducible dependency list is [requirements-mujoco.txt](../requirements-mujoco.txt). Pinning NumPy for strict environment reproducibility remains a follow-up; do not invent an exact version here.

No third-party training dataset, proprietary robot CAD package or downloaded texture file was identified in the **tracked submission tree**. This is not a warranty about every historic branch or external research reference. The tracked XML rover assets should receive human confirmation of their authoring history before final submission.

## AI-assisted development

The repository contains code, tests, architecture notes and review changes produced through a team workflow involving **human developers and AI coding assistants**, including Antigravity/Gemini and ChatGPT/Codex-style assistance. These tools were used for implementation ideas, code edits, debugging, test design, documentation and integration checks. The human team owns the product choice, verification and final submission decision.

Do not describe AI-generated or jointly authored material as exclusively handwritten work. Do not present model-generated explanations or mock traces as independent real-world verification. Exact prompts and private conversation logs are intentionally excluded from the release branch.

The project also contains an **optional** Gemini REST model backend for high-level rescue reasoning. In the recorded 18-episode release study the live-model request count was **zero**; decisions were produced by the existing deterministic fallback and policy mechanisms. The live model comparison script is opt-in and call-capped. Neither its presence nor a model identifier demonstrates actual successful inference.

## Simulation claims and data isolation

- Hidden synthetic soil truth is used by the MuJoCo world and independent evaluator; the acting robot policy receives the permitted sensor observations and local beliefs.
- The programmed hazard placement and simulated soil/traction laws are research fixtures, not a validated model of agricultural failure rates.
- Gossip volumes reflect simulated peer packet logs, not measured UWB/Wi-Fi HaLow hardware throughput.
- Rescue is verified by the simulator's measured latch/contact, displacement, motion and planting feedback, within the tested configurations.
- Human researchers must check claims against [test evidence](TESTING.md) and [recorded trials](../experiments/release_verification_20261108.json).

## Checks before submission

1. Have the team confirm ownership and whether any model/CAD/text/assets were imported from external repositories, papers, photos or pre-event work.
2. Verify the event-specific AI assistance, reuse, attribution and intellectual-property requirements in the organiser's actual portal or direct correspondence. These were **not fully established** by the captured slides.
3. If a third-party asset is distributed, include its original licence/notices and ensure redistribution is permitted. Do not assign a blanket team licence until the team has agreed.
4. Never commit API keys, account credentials, private chats, personal identifiers, participant screenshots or restricted source material. Secrets are excluded through .gitignore but must also be manually checked.
5. Verify that the screenshots, video and one-page PDF reflect actual recorded behaviour and are team-approved.

**Project licence:** no explicit repository-wide licence has been selected or granted in this file. Contact the team before assuming unrestricted reuse.
