---
name: embodied-loop
description: Use when a team is implementing or debugging the AgentVerse simulated Robotics challenge: setting up a simulator, a real sensor-to-decision-to-action loop, unexpected scene changes, local autonomy and a continuous demonstration.
---

# Build and verify an embodied simulated agent

**Activate for the Robotics entry's simulated sensor/action integration, not for general ideation, slides, agent-team coordination or ordinary code reviews.** Read only `docs/EVENT.md`, the chosen mission, relevant source/contract/test files and `docs/RUN.md` when needed.

1. **Preflight the executable simulator before agent code.** Use the team's actually accessible environment. Try NVIDIA Isaac Sim + approved protected WebRTC stream only if suitable GPU/NVENC, driver, account/credits and permissions are verified. Otherwise prove a small MuJoCo/PyBullet/Gazebo/Webots local episode. Do not install from unreviewed commands, spend credits or expose unauthenticated stream.
2. **Define one complete useful episode** for a named user: initial environment/seed, allowed simulated sensor(s), action envelope, goal and simulator-ground-truth success check. The candidate *may not* use privileged ground-truth state if it claims to perceive through a sensor.
3. **Build the shortest closed loop first:** sample sensor and time → estimate state → choose a bounded action from observations → execute in simulator → sample changed state → log all of this. The decision need not be an LLM: choose the simplest controller that works, and justify a model only for genuinely ambiguous judgement.
4. **Disturb the scene while it runs** (not a scripted second clip): insert an unplanned obstacle, moved target or sensor dropout. Show perception of the change and a different action or a safe stop. Include both successful and failed seeds and confirm the exact same conditions against a fixed-script baseline.
5. **Keep autonomy concrete:** identify where the simulator, policy and optional reasoning model execute; record external-call frequency and a loss-of-network/LLM scenario. Never put a high-rate control loop behind unreliable cloud inference. Remote renderer does not by itself establish or refute local decision autonomy.
6. **Measure before decorating:** action and observation timestamps, episode success, collision/safety failures, replan latency, disturbance response, and runtime/stream FPS where actually observed. Run focused tests, then a true end-to-end episode. The existing `scripts/evaluate.py` measures *episode* time, not policy-step latency.
7. **Prepare truthful submission evidence:** reproducible command + seed + tested commit, a ≤60-second **single continuous** stream recording with sensor/perception next to visible actions and one mid-run change, and a one-page technical summary including real hardware and latency, limitations, key failure/safety. Five-minute pitch applies only if shortlisted.
8. **Handoff:** report branch/PR, tested simulator and version, exact command, trace/video link if authorised, measured result, baseline, and what remains blocked. Ask human integrator before changing shared contracts or submitting.

**Do not** declare a demo successful because an LLM wrote a plan, create guessed cloud endpoints, claim unissued credits or equate a hard-coded animation to embodied autonomy. If a complete simulator is unavailable, prove a *minimal local* physics-and-sensor loop and report its limitations rather than fabricating Isaac Sim access.

References on demand: [Robotics event brief](../../../docs/EVENT.md), [runtime preflight](../../../docs/RUN.md), [test protocol](../../../docs/TESTING.md), [submission deliverables](../../../docs/DEMO.md). Follow current organiser/portal information if it later conflicts.
