# Method references and useful hackathon evidence

**Reviewed 7 November 2026.** These are adaptable engineering ideas, **not official AgentVerse judging rules** and not proof that one library makes teams win. This is a short methods reference; product-specific prior art belongs next to the product hypothesis after selection.

| Reference | Adapted technique | What we deliberately avoid |
| --- | --- | --- |
| [Anthropic Skills](https://github.com/anthropics/skills) and [skill-creator](https://github.com/anthropics/skills/blob/main/skills/skill-creator/SKILL.md) | Retain full native `.claude/skills/*/SKILL.md` metadata and procedural content, plus native `.claude/agents/` | Installing arbitrary community skills or loading every prompt into every agent |
| [obra/superpowers](https://github.com/obra/superpowers) | RED–GREEN–REFACTOR, reproducing root causes, verification before claiming done, worktree isolation | Heavy mandatory rituals for trivial fixes |
| [GitHub Spec Kit](https://github.github.com/spec-kit/) | Specify → plan → small tasks → implement → converge; independent idea/bug paths | Lengthy specs without actual coding time |
| [BMAD](https://bmadcode.com/method) | Phase/role clarity and explicit handoffs through versioned artifacts | Pretending separate role prompts are independent human contributors |
| [12-Factor Agents](https://github.com/humanlayer/12-factor-agents) | Small agents, explicit state, bounded tools/control flow and clear failure boundaries | Unbounded LLM loops for real-time actuation |
| [MLH standard rules](https://github.com/MLH/mlh-policies/blob/main/standard-hackathon-rules.md) | Show the built functionality and disclose prior work/AI properly where applicable | Assuming MLH governs this event |
| [Devpost AI hackathon guide](https://www.devpost.com/) and public rules/examples | Value of idea, implementation and real-world impact as common dimensions | Claiming any generic 20/30% weights are AgentVerse's rubric |
| [Official AgentVerse](https://agentverse.uclai.co.uk/) | Actual event timetable and current event authority | Treating last year's event or public guess as current track brief |

## Real-world win discipline

- **Usefulness:** name one concrete beneficiary and action they would choose to use, not a fictional persona.
- **Differentiation:** explicitly test a strong alternative: deterministic controller, existing product, model without tools, or standard optimiser. An LLM pasted into UI is not novelty.
- **Surprise and depth:** demonstrate an uncertain/novel case where the system decides what to do, not a canned prompt-response.
- **Reliability:** preserve denominator, failed cases, latency, environment and test instructions; never cherry-pick one lucky run.
- **Narrative:** problem → physical/simulated action → independently verified result → edge case → honest limitations. Show code execution before polishing pitch slides.

Research for product prior art belongs in current `docs/PRODUCT.md` (claim and nearest baseline) and source references here if extensive; avoid creating large shadow dossiers in this public repository.

## Cross-model discovery research — 7 November 2026

Current primary documentation, not inferred behaviour from an untested client:

| Runtime | Documented entrypoints | Decision |
| --- | --- | --- |
| [Claude Code memory](https://code.claude.com/docs/en/memory), [skills/subagents](https://claude.com/resources/articles/steering-claude-code-skills-hooks-rules-subagents-and-more) | `CLAUDE.md` supports `@AGENTS.md` import. `.claude/skills/<name>/SKILL.md` and `.claude/agents/*.md` are native. | Keep the entire Claude skill body and all four Claude subagents, import canonical rules. |
| [Codex project instructions](https://developers.openai.com/cookbook/examples/codex/iterating-development-workflows-with-codex), [skill location](https://learn.chatgpt.com/docs/build-skills) | `AGENTS.md` and repository `.agents/skills/<name>/SKILL.md` | Make `.agents/skills/` canonical for the six existing workflows. |
| [Gemini CLI context](https://geminicli.com/docs/cli/gemini-md/), [workspace skills](https://geminicli.com/docs/cli/skills/) | `GEMINI.md` with `@./AGENTS.md`; `.agents/skills/` recognized alongside `.gemini/skills/`. Workspace trust/activation may be required. | Use direct shared imports and skills, no second workflow fork. |
| [OpenAI guidance on lean instructions](https://developers.openai.com/blog/rethinking-skills-and-prompts-for-gpt-6-astra) | Avoid large unconditional reading requirements for small tasks. | Keep safety/contract gates, load only docs relevant to the task. |

**Preservation strategy:** `.agents/skills/` is canonical and `.claude/skills/` is a **generated full-copy mirror**, verified byte-for-byte by a dependency-free script, tests and CI. Do not substitute a one-line adapter that degrades Claude Code's native skill loading. No new Gemini subagent framework is imposed; parallelism remains assigned through Git branches, contracts and PRs. Actual CLI discovery and end-to-end execution must be tested by teammates in those installed runtimes; static checks alone do not establish those results.


## Cross-agent Git concurrency evidence — 7 November 2026

- [GitHub REST refs update](https://docs.github.com/en/rest/git/refs?apiVersion=2022-11-28): a ref update defaults to fast-forward only (`force: false`); the connected GitHub API tool also supports an `expected_sha` guard. Reject a stale head rather than overwriting it. Contents API changes to existing files require the current blob SHA.
- [Gemini CLI shell](https://geminicli.com/docs/tools/shell/) and [experimental worktrees](https://geminicli.com/docs/cli/git-worktrees/): Gemini can run normal local `git` commands with user permission; worktrees provide separate local working directories for concurrent sessions. The built-in Gemini worktree UI is optional/experimental, not required.
- [OpenAI multi-agent guidance](https://developers.openai.com/api/docs/guides/agents-api/multi-agent) and [lean-instructions research](https://developers.openai.com/blog/rethinking-skills-and-prompts-for-gpt-6-astra): parallelism helps independent tasks but increases coordination overhead for dependent steps; keep a small repo contract and on-demand skills. No need for a new task system or elaborate orchestrator.

See [TEAM](TEAM.md#practical-parallel-work-without-ceremony) for the short, human-readable actions. These are **working agreements**, not assertions that all agents share a runtime, that every client has Git permissions, or that concurrency hazards can be eliminated without reviewing merges.
