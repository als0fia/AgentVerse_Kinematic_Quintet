# Gemini CLI entrypoint

@./AGENTS.md

The import above is the canonical repository engineering contract; do not duplicate or override it here. Gemini CLI discovers project skills in `.agents/skills/` once the workspace is trusted and skills are enabled. Use `/memory show` to confirm the import and `/skills list` to inspect discovery. An ordinary Gemini chat does not automatically load these files: give it the repository context explicitly.

## Team defaults (when locally supported)

- **Main session:** coordinate the human-approved smallest runnable slice; consult [team workflow](docs/TEAM.md) for model tiers, bounded context packets, owned branches and GitHub issue/PR threads. Request fresh code/docs through tools rather than pasting entire conversations or logs.
- **Subagents:** confirm `/agents list`; project-defined `.gemini/agents/` specialists have isolated contexts and deliberately limited tools. Use read-only review before requesting write permissions. A model name in a definition does not certify account access.
- **Not implied:** API keys, paid cloud execution, remote A2A hosts, live microphone/video, or autonomous agent orchestration. The presenter’s AI Studio/Antigravity suggestions are optional, not required dependencies.
