# Gemini CLI entrypoint

@./AGENTS.md

The import above is the canonical repository engineering contract; do not duplicate or override it here. Gemini CLI discovers project skills in `.agents/skills/` once the workspace is trusted and skills are enabled. Use `/memory show` to confirm the import and `/skills list` to inspect discovery. An ordinary Gemini chat does not automatically load these files: give it the repository context explicitly.

**Current explicit build scope:** [RiceWatch P0](docs/PRODUCT.md) = one simulated rice-field inspection rover that gathers an independent local water-level reading, diagnoses a possibly false station alert and adapts to a mid-episode obstruction. **Operating a gate, pumping, changing water depth or robot-to-robot rescue are OPTIONAL ONLY and must not appear in the first build's acceptance tests.** For the initial coding agent use [RUN](docs/RUN.md#next-agent-first-integration-task--not-yet-implemented). All shared schema changes are integrator-owned. This clarifies—not overrides—imported `AGENTS.md`.

## Team defaults (when locally supported)

- **Main session:** a teammate starts this session explicitly (a GitHub issue or ChatGPT assignment does not start it). Coordinate the human-approved smallest runnable slice; consult [team workflow](docs/TEAM.md) for model tiers, bounded context packets, owned branches and GitHub issue/PR threads. Request fresh code/docs through tools rather than pasting entire conversations or logs.
- **Handoff:** for changes, use the project PR template and return branch/commit, exact commands and **observed** results. Report blockers on the relevant issue/PR; never change shared contracts or deploy/spend without human authorisation.
- **Scoped builder:** with a human-approved assignment and own worktree, invoke `@scoped-implementer` for a bounded implementation + focused tests. No branch/owned-path assignment means no write. Its model/tool configuration must be verified in the local CLI; CLI execution is not implied by GitHub comments.
- **Subagents:** confirm `/agents list`; project-defined `.gemini/agents/` specialists have isolated contexts and deliberately limited tools. Use read-only review before requesting write permissions. A model name in a definition does not certify account access.
- **Not implied:** API keys, paid cloud execution, remote A2A hosts, live microphone/video, or autonomous agent orchestration. The presenter’s AI Studio/Antigravity suggestions are optional, not required dependencies.
