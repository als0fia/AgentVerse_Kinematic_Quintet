# Reproducible setup, run and recovery

**P0 product frozen as RiceWatch autonomous **inspection/diagnosis**, not mitigation.** Application runtime not yet chosen; no simulator run verified. Do not falsely claim installation, simulator/robot access, models, cloud credits, supported operating systems or a live demo.

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

## Next agent first integration task — not yet implemented

**Goal:** make a real physics rover, sampled sensor and bounded local control loop function before five humans/agents implement competing feature stacks. A human integrator owns the first code branch and shared contract, and a human mechanical reviewer chooses/approves the rover's basic body and sensor geometry. Read [PRODUCT](PRODUCT.md), [ARCHITECTURE](ARCHITECTURE.md), the proposed `rice.v0` interfaces in [CONTRACTS](CONTRACTS.md), and [five-human ownership/assignment guide](TEAM.md#selected-rice-project-five-human-work-split-proposed-not-yet-assigned-by-name).

**First coding agent assignment (copy into agent on its own branch/worktree):**

```text
Build the FIRST executable RiceWatch autonomous INSPECTION architecture,
NOT a flood-control or gate-manipulation robot.
Read AGENTS.md, PRODUCT, ARCHITECTURE, CONTRACTS, RUN and the two matching
embodied-loop/contracts-first skills. Use the latest main SHA and own the
initial src/ricebot/contracts.py, sim adapter, runner and integration tests.
Confirm with the human owner a simple wheeled rover design on a service bank.
Try actual local MuJoCo first. If unavailable, give exact logs and ask the
human integrator to approve an allowed fallback; no unauthorised cloud usage.
Implement one physically moving rover with a genuinely sampled range sensor,
finite bounded commands and an observation-driven safe-stop/replan.
Inject one new obstacle MID-EPISODE via the simulator, not the agent API.
Freeze a minimal tested v0 observation/intent/control fixture and create a
reproducible seed/reset/headless runner. A viewer may subscribe but cannot own
world physics or command robot joints.
Show ACTUAL commands executed, pass/fail results, a sensor-action-sensor
trace, time/latency and limitations. No LLM, water CFD, gate/valve/pump actuation, manipulator,
multi-agent rescue or invented successful tests in the first PR.
Do not merge, spend credits or submit without human approval.
```

**First PR's independent acceptance:** an actual actuated wheel/position change, actual range value change, mid-run sensed obstruction, bounded changed controller action or safe HOLD, deterministic seed, a tested interface fixture, and exact setup/test commands. A mocked JSON sequence **alone** fails this gate. The intended future command shape is `python -m ricebot.runner --scenario blocked_route --seed 7 --headless`—this **does not exist yet**, and the next agent must replace it with an actually working invocation.

**After first integration:** P0b **must** add an independently observed local water-level measurement only when the rover reaches an observation point; model both real abnormal rise and a false station-gauge alert, and return `verified abnormal / likely false alarm / unresolved` without using hidden simulator truth. P0c adds same-seed baseline, independent grader and continuous video. Human-owned parallel tasks: world/sensor, rover/control, incident planner, evaluation/UI. **P1 gate/valve/flood mitigation is wholly optional after P0**, as is P2 helper-robot recovery; neither is required for P0 success.

## Selected rice mission — proposed simulator smoke path

**Product selected, runtime and geometry not yet approved or installed.** Use the organiser-recommended **MuJoCo laptop-first** route unless a teammate demonstrates a faster already-running compatible simulator. A human mechanical reviewer chooses a realistic provisional body: compact wheeled bund-inspection rover versus an explicitly simulated tracked/amphibious alternative. **Prove actuated wheels, collision, fresh range/depth/odometry sample and a changed-action response before inventing an agent stack.**

1. In an approved local development environment, install/run the exact simulator version after preflight; record actual commands, physics timestep, frames, operating system and GPU usage once verified.
2. Spawn a physical rover beside a stylised rice paddy with a passable service edge and a drainage outlet; show real collision/terrain geometry, wheel actuation and at least one timestamped real simulator sensor value.
3. Implement minimal waypoint movement under local feedback, bounded speed and collision/edge stop. Spawn a **mid-run unexpected obstruction** and show actual replanning in the same episode; reset deterministic seed and repeat.
4. **Required for P0:** feed an ambiguous fixed-gauge warning into an incident triage component. Rover travels to a second observation location, obtains an actual local sensor measurement (not a privileged true-state lookup), diagnoses `verified abnormal / likely false alarm / unresolved`, and records its sources. Test both a genuine abnormal rise and a faulty fixed gauge.
5. **P1 completely optional, not a P0 requirement:** only if P0 is already reproducible and its demo secured, consider a separately physically actuated gate/lever and synthetic water-balance proxy. No gate or water reduction is needed for a finished RiceWatch submission.
6. Produce independent baseline and candidate JSONL results with real episode traces (not the included synthetic fixtures), and a continuous video; log unverified claims separately.

**Design gates for human approval:** bund width and safe rover service path, wheelbase/ground clearance, approximate mass/inertia, traction assumptions, rover range/odometry plus **physically meaningful water-observation placement**, motor bounds, battery return policy and crop no-go regions. Gate access/contact-force design only if the team separately opts into P1. A water model is not a CFD flood forecast; a successful simulation is not evidence of actual yield protection. Agentic developer tooling may help draft the MJCF/URDF, but no hardware build, purchases or real-world deployment are authorised.

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
