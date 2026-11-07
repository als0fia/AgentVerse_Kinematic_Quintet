# Workstation onboarding — every human or coding agent

**Scope:** reproducible **local** onboarding for PaddyPatch's Python `rice.v1` core, the **checked-in MuJoCo prototype and toy smoke**, and the separately gated **Isaac Sim target**. This document is a recipe, **not a report of installs on another teammate's machine**, and not permission to spend money or alter another developer's files. Read [AGENTS](../AGENTS.md), [RUN](RUN.md) and [component ownership](COMPONENTS.md) before modifying a feature.

## Safe start: choose a lane

| Lane | Who | What counts as complete |
| --- | --- | --- |
| Core-only | Anyone with Python 3.10+ | Importable kernel and repository's standard-library checks really pass |
| Local simulator sanity | Windows/macOS/Linux developer | An isolated `mujoco` Python environment can advance a real toy physics body and report a measured joint sensor; this is **not** an Isaac Sim or PaddyPatch result |
| Optional future engine translation | Approved Isaac Sim owner on qualifying machine/host | **Not current P0**; actual Isaac version, real wheel/sensor/seed physics and matching rescue safety evidence required before any parity claim |
| Coding-agent harness | Optional Antigravity/Gemini CLI user | Confirm what this installed client actually discovers; model/credit/skills availability is not assumed |

**Current P0 is the checked-in MuJoCo rescue-first stack**, with existing physical seed/rover tests and scripted rigid-dock mechanics. Isaac Sim is an unverified optional future translation, not an initial setup prerequisite. Do not describe MuJoCo as Isaac or the independent extraction test as a full rescued mission. The main kernel, planner and world/robot interfaces are simulator-independent.

## A. Inspect and preserve the computer before installing

1. Identify OS, existing project directory, Git remotes, working tree and local files; don't create a second outer project and **never reset somebody's work**. On Windows PowerShell:
   ```powershell
   Get-Location
   Get-Command py, python, git, node, npm, nvidia-smi -ErrorAction SilentlyContinue
   py -3 --version
   git --version
   if (Get-Command nvidia-smi -ErrorAction SilentlyContinue) { nvidia-smi }
   Get-CimInstance Win32_ComputerSystem | Select-Object TotalPhysicalMemory
   Get-PSDrive C
   ```
   Commands may be unavailable; report which checks actually ran. Do not publish user IDs, machine paths, keys, project identifiers or private sponsor details in the public repo.
2. Check the exact current branch, working-tree state and [open assigned tasks](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/issues) before writing:
   ```powershell
   git status --short
   git branch --show-current
   git remote -v
   ```
   If there is no local repo yet, the human owner may explicitly authorise a clone:
   ```powershell
   git clone https://github.com/als0fia/AgentVerse_Kinematic_Quintet.git
   cd AgentVerse_Kinematic_Quintet
   ```
   **When another agent owns the team repo**, you may clone/fetch/read a reference checkout, but **do not edit or push it** until ownership changes. Run experiments in a separate sibling folder, not under a read-only mirror.
3. Before taking a coding assignment, use a unique task branch/worktree **only after the human integrator approves it**. Never start two writable agents in the same checkout. Avoid force-push, auto-merge and edits to `contracts.py`/`runtime.py` when they belong to the kernel integrator. See [TEAM](TEAM.md).

## B. Core-only setup on Windows (required minimal path)

Run inside the verified **team repo root**. Use a supported installed Python **3.10+**, not an assumed exact version:

```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe --version
.\.venv\Scripts\python.exe scripts\check_repo.py
.\.venv\Scripts\python.exe scripts\sync_skills.py --check
.\.venv\Scripts\python.exe -m unittest discover -s tests -p test_ricebot_foundation.py -v
```

**Core-only setup intentionally excludes MuJoCo.** Full test discovery requires the dependency from section C; otherwise `test_mujoco_sim.py` cannot import.

If `py` is missing, use the verified `python` executable; if Python is absent, use an approved official Python installer. A virtual environment is ignored by `.gitignore`. The core tools use the standard library and **must not require Isaac Sim, a Gemini key or MuJoCo**.

On Linux/macOS, use `python3 -m venv .venv`, then `.venv/bin/python` for the same checks. If platform/installed Python differs, **report the exact command actually used**; never write that a test passed without executing it.

## C. Optional lightweight local physics smoke (not final engine)

For the toy smoke **or current PaddyPatch MuJoCo prototype**, use **Python 3.11+** (MuJoCo 4.0.0 requires it) and install the pinned [optional dependency](../requirements-mujoco.txt), if permitted:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements-mujoco.txt
.\.venv\Scripts\python.exe scripts\smoke_mujoco.py
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --headless
# Optional GUI on a desktop:
.\.venv\Scripts\python.exe scripts\run_mujoco_paddypatch.py --viewer-seconds 8
```

On Linux/macOS use `.venv/bin/python`. The **toy smoke** only checks a basic joint; the **full MuJoCo integration suite** exercises the checked-in rover, sensors and seed-contact prototype. CI runs headlessly; an interactive viewer must be closed by its owner. Neither proves Isaac Sim, field yield or complete hackathon readiness. Core-only development still needs no MuJoCo.

**Do not copy files from one teammate's local Antigravity sandbox into the team repo without an explicit integration assignment.** A previous local prototype reported physics tests on one machine; treat that as *local experimental evidence*, never as universal reproducibility.

## D. Optional later Isaac Sim translation (not rescue P0)

- Read [NVIDIA Isaac Sim system requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html) and run the official **Compatibility Checker** on the proposed host. The linked 5.1 guidance lists **32 GB RAM and 16 GB VRAM** as minimum for supported x86_64 workstation usage. The approved actual version may have different requirements; verify and pin it before importing SDK modules. A laptop with **8 GB VRAM and ~16 GB RAM** must **not** be declared a working local Isaac host from its specs alone.
- Record host OS/driver/GPU/VRAM/RAM/actual Isaac version, supported installation approach and permissions. Keep assets/licences, packages and large downloads out of Git; respect provider terms. Isaac Sim containers are Linux-only per the cited 5.1 requirements. Do not run a remote port or GPU instance without an explicit authorised plan.
- If a human separately approves later Isaac translation, the **Isaac adapter owner** uses the [historical Isaac target information](RUN.md#next-agent-first-integration-task--isaac-sim-planting-first-skeleton-not-yet-implemented) and [issue #7](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/issues/7), with **current rescue-first gates** from [RUN](RUN.md#current-minimum-working-rescue-integration-assignment). Confirm one **actual physics** joint, changing range reading, and physical seed release/contact with traces; `rice.v1` fake-port unit tests are not equivalent.
- If only an approved cloud workstation can run Isaac, verify GPU RTX/RT cores and renderer, quotas/charge implications, security, frame output and shutdown instructions **before** launch. Never rely on a presumed free tier or automatically raise quotas/spending caps.
- If Isaac cannot run, record version/driver/logs as a future translation blocker; **continue the already approved MuJoCo P0** rather than delaying the rescue demo. Do not invent graphics or hardware performance.

## E. Optional coding-agent setup (human-owned tools)

**First:** read only the relevant repo files (`AGENTS.md`, `GEMINI.md`, specific component/skill and tested interfaces). An Antigravity project being open does **not** prove that Gemini CLI subagents are running or installed.

- **Antigravity (optional):** follow the installed client's actual sign-in flow. A hackathon-issued **temporary Google Cloud project** may be used only for the event and only with organiser permission; it may be billed even if its quota tier is high. Google's [Cloud onboarding codelab](https://codelabs.developers.google.com/get-started-with-antigravity-on-google-cloud?hl=en) documents **Use Business Account → Continue with Google Cloud → Pay As You Go → existing project ID**, not a personal subscription route. Region availability/model previews depend on project; follow the organiser's permitted configuration. **Do not publish the ID, create API keys, add payment cards, enable billing, enlarge spend caps or upload credentials.** Nothing here assumes every teammate has the same entitlement. Antigravity's native agent tooling may differ from CLI; inspect it directly.
- **Gemini CLI (optional):** only if Node/npm are available and the human approves its installation, install the official `@google/gemini-cli` package using the documented method; confirm executable `gemini`. From a trusted repo checkout, use `/memory show`, `/skills list`, `/agents list`, `/tools` to verify the actual installed version discovers `GEMINI.md`, `.agents/skills`, and `.gemini/agents`. [Official skills documentation](https://github.com/google-gemini/gemini-cli/blob/main/docs/cli/skills.md) confirms the workspace alias. A file on disk does not launch a worker; authenticate only via approved methods.
- **Other coding agents:** read `AGENTS.md` and the [component documentation standard](COMPONENTS.md), use only bounded own-file assignments, run actual local checks and return PR evidence. Model choice is a tool preference, **not a replacement for test evidence**.
- All collaborators must **export code and reproducible artefacts to authorised durable storage before temporary event accounts expire**. The temporary account's projects may be deleted after the event. Do not use it for unrelated work.

## F. Faults, evidence and handoff

| Symptom | First response | Do not do |
| --- | --- | --- |
| Python launcher missing | Locate working Python 3.10+ or request permission for official install | Guess a working `.venv` path |
| Import error / tests fail | Capture exact Python/path/trace, verify active environment and dependencies | Hide exception or claim green |
| MuJoCo viewer stays open | A GUI viewer can be intentionally long-lived; close only session-owned viewer when finished | Kill unrelated Python processes |
| No Isaac host or OOM | Document hardware/driver/compatibility check and propose approved host/fallback | Claim headless mode removes RAM/VRAM constraints |
| Model denied / quota error | Check project and entitlement using permitted UI, record nonsecret error | Add card, increase spend cap or retry thousands of calls |
| Concurrent repo edits | Work in owned worktree; compare latest head and hand off | Reset, force-push, clobber other components |

**Required local handoff:** platform+versions; actual working directory; branch/commit if authorised; commands **run** and observed results; simulator/model/tools actually verified; tests and known gaps; files created (including local-only files); any remaining approvals. Use [component README](COMPONENTS.md#small-component-readme-when-real-code-exists) for implemented modules. No one gets to call the simulation "ready" until a physics-based integrated episode succeeds.
