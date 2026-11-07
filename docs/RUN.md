# Reproducible setup, run and recovery

**Application runtime not yet chosen.** Do not falsely claim installation, simulator/robot access, models, cloud credits, supported operating systems or a live demo.

## Foundation checks — actually available

```bash
python scripts/check_repo.py
python scripts/sync_skills.py --check
python -m unittest discover -s tests -v
python scripts/evaluate.py --candidate tests/fixtures/candidate.jsonl --baseline tests/fixtures/baseline.jsonl
```

Run `python scripts/sync_skills.py --write` after editing any canonical `.agents/skills/` file. The check command fails if the complete Claude mirrors diverge or skills are missing/extra. Python 3.10+ standard library only. These checks validate **repo structure and evaluation tooling**, not any robotics entry.

## Optional Gemini CLI development-agent preflight

The repository does **not** require Gemini/Argon to run its foundation checks. When a teammate elects to use Gemini CLI, verify in that local installation: `/memory show` (imports), `/skills list` (canonical skills) and `/agents list` (built-in and project specialists). The read-only `.gemini/agents/independent-reviewer.md` is a **definition**, not proof it has launched or that `gemini-3.8-flash` is accessible.

Google's current API documentation confirms the `gemini-4-argon` and `gemini-3.8-flash` IDs for API use. No workspace API keys, model quotas, AI Studio entitlements, Antigravity installation, cloud spend or extra agent runtimes are assumed here. Do not run presenter-suggested remote installer commands without reviewing source/version and required permissions. If an actual robotics app uses Gemini, record pinned dependencies, configuration names (never secret values), errors, cost, latency and a no-network or safe-failure path below.

## Simulator/stream preflight — do this before writing agent code

**Updated priority:** later organiser email (7 Nov 15:16) recommends **MuJoCo on a laptop first**; earlier slide recommends a live stream for Isaac. Prove a running scene and captured sensor/action output quickly. The team has **not yet installed or proven** either route.

| Route | First check | Risks and decision gate |
| --- | --- | --- |
| **Isaac Sim (organiser-recommended)** | Obtain actual organiser-shared setup GitHub link and credited GPU allocation. Confirm NVIDIA compatibility, GPU NVENC, operating system, memory and streaming path; launch a sample scene and connect to stream | [Isaac Sim hardware requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html) are significant. [WebRTC docs](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/manual_livestream_clients.html) warn endpoints lack authentication/encryption; **never expose them publicly without an approved protected topology**. Do not assume all Modal GPU types support ray tracing/NVENC |
| **MuJoCo (now organiser-recommended first)** | In a local virtual environment: `python -m pip install mujoco` then `python -m mujoco.viewer` (correct module invocation; organiser email contains a line-break/space typo); verify window and sensor → action episode | [Official MuJoCo Python docs](https://github.com/google-deepmind/mujoco/blob/main/doc/python.rst) confirm installation/viewer. Run from actual workstation; pin version after smoke test. |
| **PyBullet, Gazebo, Webots (organiser-authorised alternatives)** | Establish an actual tested scene and frame output before assigning dependent modules | Choose only when someone can get an executable loop quickly; versions/installation commands TBD |

**No-cost permission gate:** No cloud, billing, remote streaming port, API token or public endpoint may be created before the team verifies account/credits, total quota, source/terms and spending/deployment approval. Recording a presentation claim is not a successful preflight. Do not accept a third-party setup script without reading its repo, version and security footprint.

**Minimal scene smoke criteria:** known seed/reset, one simulated sensor value sampled and timestamped, one bounded decision made from the observation, one visible action executed, and a later sensor sample showing the environment consequence. Add a basic mid-episode disturbance and safe stop before advancing to multi-agent sophistication. Measure actual frames per second, physics step, sensor-update rate, inference p50/p95 and recovery delay; never substitute guesses.

**Runtime topology disclosure, once implemented:** where **simulation** executes, where **policy/decision loop** executes, where **video stream** terminates, external LLM call frequency, degraded-network behaviour, data/privacy/asset licensing and billed GPU time. “Locally autonomous” must describe the *decision loop* honestly even if rendering is on a cloud GPU.

## Fill immediately after selecting the product stack

| Action | Exact command / requirement |
| --- | --- |
| Supported OS/platform | TBD |
| Minimum CPU/GPU and optional hardware | TBD; do not assume NVIDIA or a robot |
| Clean install / pinned dependency versions | TBD |
| Offline or no-secrets mode | TBD |
| Secrets configuration **names only** | TBD; never record values |
| Start the actual system | TBD |
| One deterministic smoke-demo with seed | TBD |
| Unit and integration tests | TBD |
| Independent baseline run and evaluator export | TBD |
| Reset simulator/robot and recover from errors | TBD |
| Troubleshooting/log location | TBD |

A team member who did **not** develop the project should be able to reproduce the minimum demo from this file. Record commands actually validated on a clean environment and link the tested commit. For physical hardware, add an operator stop/abort procedure and a simulator-only or replay fallback.
