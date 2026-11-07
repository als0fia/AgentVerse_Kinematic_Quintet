# Claude Code entrypoint

@AGENTS.md

Claude Code reads this `CLAUDE.md` at session start. The import above ensures the complete common contract is active, including when Claude prefers `CLAUDE.md` over `AGENTS.md`.

Claude-native agents remain in `.claude/agents/`. Complete, discoverable Claude skill definitions remain in `.claude/skills/`; they are verified byte-identical mirrors of `.agents/skills/` via `python scripts/sync_skills.py --check`. Never replace them with partial wrappers.
