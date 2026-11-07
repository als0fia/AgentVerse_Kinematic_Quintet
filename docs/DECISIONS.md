# Decision log (architecture and product scope)

This records **durable rationale**, not a running task/status list. Put implementation tasks in issues and code evidence in PRs. Add entries when trade-offs affect other components or the product mission.

## D-0001 — Keep the foundation stack-neutral until challenge reveal

- **Date:** 7 November 2026
- **Status:** adopted for foundation preparation; product still unselected
- **Context:** The team repository initially contained only a README; the current robotics challenge and provided stack had not been verified.
- **Decision:** Bootstrap a small, cross-agent documentation/testing/CI layer with Python standard-library tooling, but no fabricated robot stack, API scaffolding or chosen idea.
- **Reason:** Prevent agents from building incompatible assumptions before team selection. Retain a runnable proof of repo health and a reusable evaluation adapter.
- **Trade-off:** Actual integration, simulator dependencies and application tests cannot be written accurately yet.
- **Revisit:** Immediately when organiser brief and team direction are known. Choose and document actual runtime and components then.

## D-0002 — Embodied loop required; simulator and mission still team decisions

- **Date:** 7 November 2026
- **Status:** adopted mandatory challenge constraints, **not a project/stack selection**
- **Source:** onsite organiser Robotics track slides 35–38 and recorded presenter explanation; technical details in [EVENT](EVENT.md).
- **Decision:** A submission must be evaluated as a closed simulated **sensor → decision → action → feedback** loop, including an observable unplanned change. Avoid a scripted sequence dressed as autonomy. Use local/inside-runtime core decisions where practical; isolate optional remote LLM deliberation from the short-horizon control loop.
- **Options open:** Isaac Sim headless stream on verified suitable compute, or MuJoCo/PyBullet/Gazebo/Webots where team resources make a faster deterministic first episode possible. No hardware, GPU, model credits, simulator or project is selected or verified in this record.
- **Engineering impact:** product selection needs a meaningful human problem; architecture must expose actual sensor and action interfaces; testing must compare fixed-path and disturbed episodes; demo needs one continuous ≤60s video and one-page technical PDF.
- **Trade-off:** premature advanced engine/cloud integration could consume the whole hackathon; fallback simulator and network-off tests preserve an achievable demo. The earlier glare-related rubric uncertainty is **superseded by D-0003's full clear-slide evidence**; account/credit details remain unverified.
- **Revisit:** once team selects mission, tests available compute and proves first simulated episode.

## D-0003 — Full rubric verified; impact is an evidence and demonstration gate

- **Date:** 7 November 2026
- **Status:** evidence corrected from additional clearer organiser photograph; **not** a selected project
- **Evidence:** attendee-supplied unobscured “How you'll be judged” slide; [EVENT](EVENT.md#confirmed-robotics-judging-rubric--clear-organiser-photograph-7-november-2026).
- **Decision:** use the **actual** Robotics weights: perception 25%, adaptation 25%, autonomy 15%, **relevance and impact 20%**, ambition/technical quality 15%. A proposal must name a credible human need with source evidence, and its most important failure must be handled on screen. An exciting simulation without real utility is not enough.
- **Effects:** [PRODUCT](PRODUCT.md) becomes the concept-selection evidence gate; [DEMO](DEMO.md) maps that evidence to the one-page PDF and continuous simulation recording. Existing algorithms and specialised models remain meaningful baselines.
- **Open:** actual beneficiary, mission, resources and working demo; no scored team performance is asserted.

## D-0004 — GitHub-mediated developer-agent communication; human integrator

- **Date:** 7 November 2026
- **Status:** process adopted in documentation; worker runtime/model entitlements not exercised
- **Alternatives:** (a) single developer and human review, (b) GitHub branches/PRs with bounded Gemini CLI sessions, (c) hosted multi-agent messaging service.
- **Decision:** start with (a); use (b) when independent modules genuinely exist. ChatGPT may actively implement/review via its GitHub connector; Gemini sessions/subagents must be started by authorised local operators. Each worker owns branch/paths, returns observed test results and a PR for shared code; human integrator merges and approves external actions. No automatic direct ChatGPT→Gemini calls or remote A2A service.
- **Reasons:** first-party [engineering evidence](RESEARCH.md#development-agent-communication-and-model-choice-evidence--7-november-2026) favours simple workflows, context isolation, explicit handoff and independent verification. Model roles are provisional and must be measured on actual accessible tools.
- **Testability:** the PR template and regression checks protect rubric consistency/skill mirrors; **none of this is a running robot or a measured multi-agent benchmark**.
- **Revisit:** when two separately runnable module tasks exist or repeated communication failures justify automation.

## D-0005 — Rice-field monitoring and adaptive response selected; physical design is human-reviewed

- **Date:** 7 November 2026.
- **Status:** user/team-direction expressed in this conversation: **proceed with the rice version**; individual teammate ownership, physical robot design and implemented scope need explicit integration review. No claim that a simulator, robot or mitigation already exists.
- **Beneficiary:** rice-field irrigation/drainage operators who need to identify abnormal excessive water rise or blocked drains and respond without unnecessary crop damage or robot exposure. Normal shallow paddy ponding is not a fault. Current alternatives: human observation, gauges, scheduled inspection, conventional water-control and reactive local planners.
- **Selected product system:** **Monitoring, Investigation & Targeted Mitigation**, applied to a rice-field incident. General reusable seam is uncertainty-aware monitoring → physical investigation → constrained intervention or safe escalation → independent confirmation.
- **Minimum committed demo (P0):** a physically actuated simulated rover reads permitted simulated sensors, travels to inspect an abnormal drainage/water observation, responds to an unexpected mid-run obstacle or changed evidence through fresh perception, and completes or safely abstains. Test a credible non-agent baseline on matched seeds. Keep a human-led mechanical review and independently checked collision/energy limits.
- **Desired follow-on (P1):** physically actuate a small reachable drainage gate/lever (or similar bounded intervention) and verify a water-depth/flow change in a **clearly simplified** field model. If manipulation or water physics is not working, report only inspection/recommendation; do not claim mitigation was executed.
- **Optional framework extensions (P2):** robot failure/recovery decision (wait, self-recover, dispatch helper or escalate) and alternative domain adapters such as reforestation and lunar/space construction. Real robot-to-robot rescue and cross-domain transfer are **not** required this weekend and are not proven by a shared software interface.
- **Embodiment trade-off:** start with a *reference* bund/service-edge wheeled rover for simple realistic locomotion and access to outlets. Human mechanical designer/integrator must validate geometry, surface width/slopes, stability, traction, wheel/track choice, splash/crop limits, sensor mount, battery and gate reach/forces; amphibious or tracked field-traversing robots add buoyancy/soil/sinkage physics and remain design alternatives. Sources: [2026 paddy robot chassis review](https://pmc.ncbi.nlm.nih.gov/articles/PMC12901939/) and [floating wheel paddy robot study](https://www.fujipress.jp/jrm/rb/robot003800020404/?lang=ja).
- **Why not precision planting as core:** no evidence that expensive microlocation selection outperforms standard mechanised rice seeding/transplanting; keep original ideas/history but focus the submitted claim on monitoring, fault investigation, safe physical response and measurable adaptation.
- **Model decision:** System 0 deterministic local controller/safety; System 1 fast sensing and constrained planning; slow LLM/VLM only if a genuine ambiguous diagnosis or inspection-action choice beats Bayesian/ordinary planner under matched tests. Do not make frontier model calls an actuator-rate dependency.
- **Owners and gate:** [PRODUCT](PRODUCT.md) owns present mission, [ARCHITECTURE](ARCHITECTURE.md) owns component and body design, [CONTRACTS](CONTRACTS.md) owns typed interfaces, [RUN](RUN.md) tested launch details, [TESTING](TESTING.md) metrics and [DEMO](DEMO.md) honest video. Human integrator approves the initial body, shared interface and scope. No public deployment, spending, final submission or fabricated results.

## Decision template

```text
D-XXXX — [Decision title]
Date / status / owners:
Problem and constraints:
Options considered and baseline:
Decision and why:
Interface/tests/risks affected:
Revisit trigger:
```
