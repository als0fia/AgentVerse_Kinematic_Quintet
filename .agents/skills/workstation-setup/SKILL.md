---
name: workstation-setup
description: Inspect and safely configure a developer computer for the existing rice.v1 Python kernel, optional MuJoCo local physics, gated Isaac Sim host and optional Gemini or Antigravity coding tools with reproducible test evidence.
---

# Set up a new team developer's workstation safely

Follow the human-approved setup scope. The **canonical procedure** is [WORKSTATION-SETUP](../../../docs/WORKSTATION-SETUP.md); do not copy lengthy install instructions into this skill.

1. **Inspect first:** current OS, real workspace path, Git working tree, Python/node/GPU/RAM, existing packages and agent client. Keep private paths, project IDs, credentials and secrets out of public reports. Protect active agent/user work.
2. **Choose lane:** core-only Python standard-library, optional local MuJoCo toy physics, or **approved real Isaac Sim** host. Isaac is the chosen target; do not substitute MuJoCo silently. No cloud spending or massive SDK installs without permission.
3. **Repository boundary:** clone only when human authorised; when another agent owns main/reference checkout, fetch/read only. Before coding choose a unique branch/worktree and owned paths. Never reset, force-push or edit another agent's files.
4. **Perform actual verification:** `scripts/check_repo.py`, `scripts/sync_skills.py --check`, `python -m unittest discover -s tests -v`; for optional MuJoCo install `mujoco`, run `scripts/smoke_mujoco.py` and open the official viewer if supported; classify core, toy simulator and embodied Isaac separately.
5. **AI tools:** Antigravity and Gemini CLI are optional, differently equipped clients. Verify real installed models/tooling and discovery instead of assuming. Use only team-approved temporary event account and project, no hardcoded API key or billing changes, no extra paid batch probes.
6. **Isaac gate:** compare real host to NVIDIA minimum specs and compatibility checker; pin actual installed API and test a physical rover/range/seed contact. If blocked, report details and ask for human fallback/host choice.
7. **Handoff:** exact commands and observed outcomes, versions, paths (private to team where appropriate), simulator status, files created, component owner, unresolved blockers and approvals. Do not mark target simulation ready from a fake port or mock-only suite.

No hosted agent can claim it configured a teammate's computer unless an actual connected local runtime executed the commands.
