# Gemini CLI entrypoint

@./AGENTS.md

The import above is the canonical repository engineering contract; do not duplicate or override it here. Gemini CLI discovers project skills in `.agents/skills/` once the workspace is trusted and skills are enabled. Use `/memory show` to confirm the import and `/skills list` to inspect discovery. An ordinary Gemini chat does not automatically load these files: give it the repository context explicitly.

**Current required build:** [Antenna P0](docs/PRODUCT.md) = **NVIDIA Isaac Sim** two-rover rice stand-gap mapping + physical seed placement. Start one actuated rover and range sensor, **then immediately prove a hopper-released rigid-body seed actually lands**; next implement second locally sampling rover, shared estimated soil/stand-quality map, route/site allocation and a mid-run surprise. Recovery, Antenna and flood mitigation/gates are **optional**. [RUN](docs/RUN.md#next-agent-first-integration-task--isaac-sim-planting-first-skeleton-not-yet-implemented) has the exact first coding task; [CONTRACTS](docs/CONTRACTS.md) is the integrator-owned `rice.v1` interface. Verify installed Isaac version/host compatibility before promising execution; no cloud spend or automatic alternative-engine switch. The import of AGENTS.md remains authoritative.

**New workstation:** read [setup runbook](docs/WORKSTATION-SETUP.md) and [workstation-setup skill](.agents/skills/workstation-setup/SKILL.md) to install/test only authorised tools. For a feature audit use [module-audit](.agents/skills/module-audit/SKILL.md) and report findings first; audit is **read-only** unless explicitly changed to implementation. [COMPONENTS](docs/COMPONENTS.md) defines component READMEs and world/robot ownership (human/ChatGPT/Gemini roles interchangeable).

## Team defaults (when locally supported)

- **Main session:** a teammate starts this session explicitly (a GitHub issue or ChatGPT assignment does not start it). Coordinate the human-approved smallest runnable slice; consult [team workflow](docs/TEAM.md) for model tiers, bounded context packets, owned branches and GitHub issue/PR threads. Request fresh code/docs through tools rather than pasting entire conversations or logs.
- **Handoff:** for changes, use the project PR template and return branch/commit, exact commands and **observed** results. Report blockers on the relevant issue/PR; never change shared contracts or deploy/spend without human authorisation.
- **Scoped builder:** with a human-approved assignment and own worktree, invoke `@scoped-implementer` for a bounded implementation + focused tests. No branch/owned-path assignment means no write. Its model/tool configuration must be verified in the local CLI; CLI execution is not implied by GitHub comments.
- **Subagents:** confirm `/agents list`; project-defined `.gemini/agents/` specialists have isolated contexts and deliberately limited tools. Use read-only review before requesting write permissions. A model name in a definition does not certify account access.
- **Not implied:** API keys, paid cloud execution, remote A2A hosts, live microphone/video, or autonomous agent orchestration. The presenter’s AI Studio/Antigravity suggestions are optional, not required dependencies.
