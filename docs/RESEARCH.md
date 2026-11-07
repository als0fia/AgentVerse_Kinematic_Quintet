# Method references and useful hackathon evidence

**Reviewed 7 November 2026.** These are adaptable engineering ideas, **not official AgentVerse judging rules** and not proof that one library makes teams win. This is a short methods reference; product-specific prior art belongs next to the product hypothesis after selection.

| Reference | Adapted technique | What we deliberately avoid |
| --- | --- | --- |
| [Anthropic Skills](https://github.com/anthropics/skills) and [skill-creator](https://github.com/anthropics/skills/blob/main/skills/skill-creator/SKILL.md) | Short, task-scoped `.claude/skills/*/SKILL.md` with metadata and progressive reading | Installing arbitrary community skills or loading every prompt into every agent |
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
