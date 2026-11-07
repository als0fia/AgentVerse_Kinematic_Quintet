# Agent instructions — AgentVerse team submission repository

The repo is public. It contains **shared submission engineering**, not personal hackathon operations. Never read private personal-ops files as an input to team coding or paste restricted BuilderBase/Discord/team content here.

## Boot sequence (smallest relevant context)

1. Read `README.md` and this file; inspect the current working tree/branch and existing diffs, issues and PRs before modifying anything.
2. Read `docs/EVENT.md`, `docs/PRODUCT.md`, and `docs/TEAM.md` for the actual current challenge, *selected* direction and ownership. If unresolved, work only on reusable infrastructure or explicitly requested research.
3. For implementation, read `docs/ARCHITECTURE.md`, `docs/CONTRACTS.md`, `docs/RUN.md` and the relevant tests. For evaluation/demo read `docs/TESTING.md` / `docs/DEMO.md`.
4. Read only a matching `.claude/skills/*/SKILL.md` when a task needs its procedure; the skill does not grant special tools or authority.

## Order of work

1. Identify a real beneficiary, environment and observable outcome. Check closest non-LLM alternative/prior art; require actual AI-added judgement.
2. Create a short task issue/PR description: acceptance check, owner, paths, dependencies, risks. Do not turn a five-minute fix into a speculative PRD.
3. For any shared interface change, specify fields, types, units, frame, semantics, version, errors and fixtures **before** consumers implement against it. Integrator owns shared contracts.
4. Work in an isolated branch or worktree; **do not edit another agent's assigned files** without coordinating. Prefer one-writer per file and merge through PR/integration owner.
5. Test in layers: pure logic → module contract → integration → real end-to-end episode; compare the nearest meaningful baseline. Keep deterministic random seeds and environment details.
6. Debug by reproducing, collecting exact traces/logs, locating root cause, applying the minimal correction, adding a regression test and rerunning relevant checks. No speculative patch stacking.
7. Report changed paths, commands **actually** run, results, commit/PR, unknowns, blockers and rollback path. Never claim "tested" from code reading.
8. Keep one canonical current-truth document per subject; link to it instead of copying. Git history retains earlier decisions.

## Multi-agent cooperation

- Team human/integration lead owns task allocation, merge order, integration and final demo.
- Implementation agents own isolated modules and corresponding tests; shared contracts change only through coordinated review.
- Independent evaluator probes claims and edge cases without changing the test to make a failing candidate appear successful.
- Demo lead owns the audience story and fallback but may not invent production capability.
- Pass a **written handoff in the PR description** using `docs/TEAM.md`: task/branch, files touched, contract version, test command + observed output, unverified assumptions, integration steps. No always-growing session journal.
- GitHub Issues are the work queue; docs are system truths. If no Issues access, communicate assignments before simultaneous edits.

## Safety and claims

- Do not run opaque downloaded scripts, destructive commands, high-risk external actions or send secrets to a model/tool.
- No spending/cloud credit use, public deployment, submission, messaging outsiders, secrets, force pushes, deletion or visibility change without explicit human permission.
- Confirm challenge timing and prior work/AI disclosure rules with organisers. An absence of a restriction is not permission.
- The robot's high-rate safety-critical control should be deterministic or appropriate specialised control; use an LLM only where its reasoning adds value and bound its authority.
- Always distinguish actual hardware operation, simulation, replay, scripted fallback, and proposed future work.

## Task routing

| Task | Read |
| --- | --- |
| Assess direction | `.claude/skills/problem-first/SKILL.md` + `docs/PRODUCT.md` |
| Plan shared interface | `.claude/skills/contracts-first/SKILL.md` + `docs/CONTRACTS.md` |
| Build component | `.claude/skills/implement-and-verify/SKILL.md` + tests |
| Diagnose bug | `.claude/skills/root-cause-debug/SKILL.md` |
| Coordinate parallel agents | `.claude/skills/parallel-handoff/SKILL.md` + `docs/TEAM.md` |
| Compare evidence | `.claude/skills/evaluate-and-demo/SKILL.md` + `docs/TESTING.md` |

The existence of an agent file or automation does **not** mean any assistant has access to multiple runtimes, is executing in background, or has passed a test.
