# Gemini CLI entrypoint

@./AGENTS.md

The import above is the canonical repository engineering contract; do not duplicate or override it here. Gemini CLI discovers project skills in `.agents/skills/` once the workspace is trusted and skills are enabled. Use `/memory show` to confirm the import and `/skills list` to inspect discovery. An ordinary Gemini chat does not automatically load these files: give it the repository context explicitly.
