# Agent instructions — AgentVerse team submission repository

The repo is public. It contains **shared submission engineering**, not personal hackathon operations. Never read private personal-ops files as an input to team coding or paste restricted BuilderBase/Discord/team content here.

## Boot sequence (smallest relevant context)

1. Follow this file as the single repository-wide instruction contract. Use `README.md` for orientation; inspect the current working tree/branch and relevant diffs, issues and PRs before changing code.
2. For project selection or a changed brief, use `docs/EVENT.md` and `docs/PRODUCT.md`; for parallel work use `docs/TEAM.md`. If the challenge or project is unresolved, work only on reusable infrastructure or explicitly requested research.
3. For interface/runtime changes, use `docs/ARCHITECTURE.md`, `docs/CONTRACTS.md`, `docs/RUN.md` and relevant tests. For evaluation/demo use `docs/TESTING.md` / `docs/DEMO.md`. For tiny fixes, do not read unrelated documents.
4. Load only the **matching** `.agents/skills/<skill>/SKILL.md` when its workflow applies; do not inject all skills, histories and tool outputs into every prompt. Gemini CLI loads task skills on demand. Skills do not grant extra tools or authority.

## Order of work

1. Identify a real beneficiary, environment and observable outcome. Check closest non-LLM alternative/prior art; require actual AI-added judgement.
2. For a tiny independent fix, implement and test directly within the assigned branch/task; no new issue, planning document or approval ceremony is required. For concurrent or cross-module work, communicate owner, branch, intended paths, acceptance check and dependencies in an existing issue/PR or short human message.
3. For any shared interface change, specify fields, types, units, frame, semantics, version, errors and fixtures **before** consumers implement against it. Integrator owns shared contracts.
4. If others are working simultaneously, use a separate branch (and a separate local worktree/checkout if sharing a machine). Avoid two active writers to the same file or contract; coordinate shared changes through the integrator. Small non-overlapping solo work follows the current team branch convention.
5. Test in layers: pure logic → module contract → integration → real end-to-end episode; compare the nearest meaningful baseline. Keep deterministic random seeds and environment details.
6. Debug by reproducing, collecting exact traces/logs, locating root cause, applying the minimal correction, adding a regression test and rerunning relevant checks. No speculative patch stacking.
7. Report changed paths, commands **actually** run, results, commit/PR, unknowns, blockers and rollback path. Never claim "tested" from code reading.
8. Keep one canonical current-truth document per subject; link to it instead of copying. Git history retains earlier decisions.

## Multi-agent cooperation

- Team human/integration lead owns task allocation, merge order, integration and final demo.
- Implementation agents own isolated modules and corresponding tests; shared contracts change only through coordinated review.
- Independent evaluator probes claims and edge cases without changing the test to make a failing candidate appear successful.
- Demo lead owns the audience story and fallback but may not invent production capability.
- Follow the **minimal concurrency and GitHub safety protocol** in `docs/TEAM.md`: start from fresh head, work in an owned branch, never force-overwrite others' commits, and recheck remote state before final write/merge. This applies equally to GitHub API connector agents and local git agents.
- At handoff, report branch/commit/PR, touched paths and exact test evidence; include contract version and risks only if relevant. No always-growing session journal.
- GitHub Issues help track meaningful work; a quick fix does **not** require an issue. A short explicit human assignment is enough to separate two agents' work.

## Safety and claims

- Do not run opaque downloaded scripts, destructive commands, high-risk external actions or send secrets to a model/tool.
- No spending/cloud credit use, public deployment, submission, messaging outsiders, secrets, force pushes, deletion or visibility change without explicit human permission.
- Confirm challenge timing and prior work/AI disclosure rules with organisers. An absence of a restriction is not permission.
- The robot's high-rate safety-critical control should be deterministic or appropriate specialised control; use an LLM only where its reasoning adds value and bound its authority.
- Always distinguish actual hardware operation, simulation, replay, scripted fallback, and proposed future work.

## Task routing

| Task | Read |
| --- | --- |
| Assess direction | `.agents/skills/problem-first/SKILL.md` + `docs/PRODUCT.md` |
| Plan shared interface | `.agents/skills/contracts-first/SKILL.md` + `docs/CONTRACTS.md` |
| Build component | `.agents/skills/implement-and-verify/SKILL.md` + tests |
| Diagnose bug | `.agents/skills/root-cause-debug/SKILL.md` |
| Coordinate parallel agents | `.agents/skills/parallel-handoff/SKILL.md` + `docs/TEAM.md` |
| Compare evidence | `.agents/skills/evaluate-and-demo/SKILL.md` + `docs/TESTING.md` |

## Gemini-first entrypoints and scoped delegation

- **Primary:** Gemini CLI imports `AGENTS.md` through `GEMINI.md`. The six canonical on-demand skills live in `.agents/skills/`; optional specialised Gemini subagents are under `.gemini/agents/`. Run `/memory show`, `/skills list`, `/agents list` to verify actual discovery when locally installed. No file automatically starts a worker.
- **Model/context routing:** follow [team coordination](docs/TEAM.md). Prefer Gemini 4 Argon for complex cross-module reasoning and Gemini 3.8 Flash for bounded specialist work, *if the team has access*. Supply the smallest relevant files, latest commit, interface and test fixture, not every chat or full repo. Use measured results, not marketing cache-hit guarantees.
- **Coordination:** GitHub issue/PR threads and owned branches/worktrees are the default shared interface. No automatic live agent messenger or global inbox. Human integrator approves priorities and merging, and each AI run must be invoked explicitly.
- **Other tooling:** Codex may follow the same neutral source `AGENTS.md` and skills. Claude-native entrypoints, agents and full skill mirrors are **retained as legacy compatibility only**, not the preferred route; existing CI currently checks their parity. Do not delete them or change shared CI without reconciling the team. Any assistant without workspace-native discovery must read relevant files explicitly.
- Do not impose multi-agent ceremony on tiny solo changes. Create independent subagents only when a bounded task actually benefits; Gemini CLI subagents do not recursively delegate.

The role/routing guidance above is for **developer agents**, not a commitment to use a particular model inside the eventual robotics product.

The existence of an agent file or automation does **not** mean any assistant has access to multiple runtimes, is executing in background, or has passed a test.
