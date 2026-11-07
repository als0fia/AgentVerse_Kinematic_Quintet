---
name: problem-first
description: Assess a proposed hackathon project against real user utility, nearest prior art, deterministic baselines, technical AI necessity, event constraints, and a minimal falsifiable demo.
---

# Problem-first selection

1. Read `docs/EVENT.md` for the now **revealed Robotics challenge** and verified rules: at least one simulated sensor, observation-driven action, and changed-scene adaptation. Treat any still-obscured scoring weights or credit terms as unknown.
2. Write a specific person/workflow and the **observable action** the product performs. Reject purely cosmetic agent wrappers.
3. Find the nearest existing implementation/paper and strongest conventional tool, controller, optimiser or VLM baseline. Explain which parts they already solve.
4. Identify the **single uncertainty** where model judgement plausibly helps. State a falsifiable comparison and a case likely to fail.
5. Propose the smallest reproducible **simulated sensor → decision → action → new observation** episode, plus one disturbance within the same continuous run. Specify an independent outcome check, latency, bounded failure and what a fixed-path baseline does.
6. Establish a real first-hour simulator/stream preflight before assuming Isaac Sim or cloud GPU. MuJoCo/PyBullet/Gazebo/Webots are allowed alternatives. Discuss actual hardware, remote-vs-local inference, permissions, credits, data and time before choosing. Preserve alternatives; team human explicitly selects.
7. Update `docs/PRODUCT.md` only for a **selected** product and `docs/DECISIONS.md` for enduring trade-offs. No code or fanciful performance results from ideation alone.

Deliver: human benefit; real action; AI necessity; baseline; demo; risk; go/clarify/park with evidence and sources.
