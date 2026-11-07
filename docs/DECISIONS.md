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

## D-0006 — Freeze P0 as autonomous rice-water inspection and diagnosis; mitigation optional

- **Date:** 7 November 2026.
- **Status:** explicit user scope correction, **supersedes D-0005 wherever it might suggest a drainage intervention is required**. Existing D-0005 preserves why rice was chosen; current requirements live in [PRODUCT](PRODUCT.md).
- **Decision:** **RiceWatch P0 is an autonomous physical inspection and verification rover** for a suspicious water-level alert, **not** a flood-mitigation or gate-operating robot. One rover physically reaches a local sampling point, compares independent local water readings with a possibly faulty fixed gauge, handles an unexpected mid-run route obstruction, and returns `verified abnormal`, `likely false alarm` or `unresolved`. The local controller remains safe without a remote LLM. At least one real simulated range/odometry sensor and a truly independent **location-gated** water measurement must inform decisions; the agent never sees hidden world truth.
- **Mandatory gates:** P0a moving physics rover + sensor-driven obstacle replanning; P0b actual rice-water verification with true-rise/false-gauge cases; P0c independent same-seed baseline and ≤60s continuous sensor/action/surprise recording plus 1-page PDF. The skeleton alone is not the completed rice product.
- **Optional-only P1:** physical gate/valve/pump manipulation, hydrology/flow change or other mitigation **only if time remains after P0 is tested and the core recording secured**. No actuator or mitigation test may block P0 completion. Advice to a human operator is not physical mitigation.
- **Optional P2:** second robot/rescue, communications fallback, reforestation/space adaptation. These are not demo requirements or proven transfer claims.
- **Human design and five-person coordination:** physical rover embodiment and local water-probe location reviewed by mechanical owner; one integrator freezes shared seam and others own independent small features, not monolithic rewrites. [TEAM](TEAM.md) and [RUN](RUN.md) govern execution.
- **Reason:** makes the weekend deliverable coherent and falsifiable, avoids unsupported claims of crop saving or flood prevention, and reduces manipulator, hydrology and multi-robot dependencies while still meeting perception, adaptation, autonomy and impact criteria.
- **Revisit only:** an explicit later human scope decision *after* P0 evidence; no agent may silently promote P1 into P0.

## D-0007 — Physical cooperative rice planting supersedes RiceWatch warning-only scope; Isaac Sim chosen

- **Date:** 7 November 2026; user decision. **Status:** selected **project mission and simulator target**, *not* a verified working scene, farm validation or team-wide signed acceptance.
- **Why change:** a monitoring/recovery system without successfully doing the underlying agricultural job does not demonstrate a useful robot. Precision whole-field spot selection is hard to justify against existing conventional seeding; target a narrower **patchy/failed stand gap-replanting problem**, avoiding unsupported universal superiority claims.
- **Current required P0:** **PaddyPatch** (working name). Prepared dry/direct-seeded/partly drained rice field with irregular existing stand/gaps and spatially varying *synthetic* moisture/firmness; a physically simulated planter and independently sensing scout; fused estimated field-quality/stand map and sample provenance; site selection, capability-aware/obstacle-aware safe routes and limited seed budget; **actual rigid-body seed proxy release/contact**, plus surprise-driven task/route replanning. Physical planting first, intelligent recovery later.
- **Development order:** (a) **Isaac Sim** actual physics rover/range/surprise + typed seam; (b) **physical hopper/dropper + measured landing**; (c) second real rover/soil probe, shared map, route and plant allocations; (d) independent baselines, surprise episode, ≤60s continuous video and one-page PDF. Earlier MuJoCo-preferred preflight is superseded; use MuJoCo only by later explicit human choice if Isaac unavailable.
- **Simulator constraint:** Isaac Sim user-selected but actual version/compatible GPU/system RAM, environment, scene and local/remote execution **unverified**. NVIDIA 5.1 guidance lists 32GB RAM / 16GB VRAM minimum; 8GB laptop GPUs may be insufficient. Test actual hardware and record real metrics before parallel worker rollout; no unapproved cloud spending or streaming.
- **Boundaries:** `rice.v1` typed observation/source samples, field belief (mean/uncertainty), bounded physical robot actions, plant command and actual contact result; hidden soil truth grader only. One human integrator freezes shared schema; physical robot designer approves vehicle and release mechanism, separate human owners build world/scout, map/planner, independent baseline/UI.
- **Prior decisions D-0005/D-0006 retained as historical context**, but their rice flood alert/diagnosis P0 and “no planting required” claims **are superseded**. Flood monitoring/physical gate mitigation, robot-to-robot recovery and reforestation/space transfer remain **optional after P0 physical planting/evaluation**.
- **Falsification:** first real seed placement must occur early. A pretty soil heatmap, moving UI sprite, hard-coded ideal soil map, imagined sample, text-only planter, fake rescue, unvalidated soil yield claim or staged video fails selected scope. Compare against competent uniform/greedy and deterministic joint mapping/routing baselines on same hidden synthetic conditions.
- **Revisit:** failed Isaac compatibility, inability to drop a seed physically, lack of distinct two-rover measurements, unrealistic soil model, no credible baseline or explicit later user/team directive.

## D-0008 — Antenna is the canonical selected product name; do not fork scope

- **Date:** 7 November 2026.
- **Source:** direct human team instruction in ChatGPT project conversation: “Antenna is canonical.”
- **Decision:** **Antenna** is the current official internal product name for the cooperative rice stand-gap mapping and targeted replanting submission. **PaddyPatch** was the earlier working name for the same P0 mission, not an alternative active entry. **RiceWatch** denotes superseded flood-warning investigation scope. The separately pushed **RiceSwarm AI** full-season/fleet concept is independent exploratory work, not the canonical application path.
- **Implementation owner:** the integrated `main` source under `src/ricebot/` and the existing `rice.v1` interface; keep package/file names stable during the hackathon rather than renaming working code for branding.
- **Branch authority:** merged historical branch heads and open research/experimental branches are not competing canonical product definitions. Integrate only specifically reviewed changes through a compatible PR. Do not mass-replace historical decision names, overwrite current simulator/CI setup, or merge an unrelated full farming simulation into P0.
- **Scope remains unchanged:** measured local sensing → shared map → coordinated task/route selection → physically simulated planting and contact → response to an unexpected change. Recovery/flood mitigation optional; MuJoCo is the current tested development backend, Isaac Sim a separately unverified integration target.
- **History:** D-0005, D-0006 and D-0007 are preserved verbatim as earlier decisions, not rewritten to imply that Antenna was a prior rejected inspection-only product.

## D-0009 — Rescue-first Antenna mission; rice field stays useful work

- **Date/source:** 7 November 2026, explicit user's later clarification. **Status:** new user-directed product/research priority; code integration, physical learned skill and full team signoff are not claimed.
- **Decision:** the **primary feature is a robot collective that decides when and how to physically rescue a failed member** using sensor/health evidence, calculated traction/second-casualty risk, uncertain terrain/weather inputs, urgency and operator priorities. Rice-field local measurements, mapping and physical seed placement provide the first meaningful mission; preserve all working source and tests. This **supersedes D-0007's rescue-optional product priority** without overwriting historical records.
- **Naming stays D-0008:** **Antenna** is canonical; PaddyPatch earlier working name; experimental RiceSwarm AI is not the authoritative main runtime. Do not renumber prior decisions.
- **What already exists:** PR #23 single-process task bids + heuristic rescue triage + optional Gemini REST advisor, and PR #24 homogeneous Paddy Ant MJCF rigid docking/cam-latch/pitch-lift/tandem extraction. The latter has a **scripted MuJoCo test** that asserts victim displacement >0.08 m. There is **no yet verified integrated incident→contextual decision→physical recovery→continued fieldwork episode**, no trained extraction policy and no actual P2P/distributed Jetson LLM.
- **Next gate:** distinguish true immobilisation from `front_range_m=None`; propose bounded typed recovery action/results via sole integrator; use physically valid dock mechanism to rescue only when risk/safety thresholds permit; independent hidden-truth grader compares never/always/deterministic risk/optional contextual model and post-incident mission value. Correct **not rescuing** matters.
- **Optional deployment research:** independently operating local robot policies with no internet requirement; compare an offline small model and a larger quantised **one-model** split across verified Jetson devices. Split inference is not multi-LLM debate, may be slow, and breaks when a required shard/node disappears; always provide local safe fallback. Team Jetson ownership/cost/performance are unverified, no spending/creation authorised.
- **Scope and prior art:** physical recovery, learned control, risk-aware planning, task auctions and model parallelism have predecessors; evidence against strong baselines and actual physics is necessary. [PRODUCT](PRODUCT.md), [SWARM_EDGE_PLAN](SWARM_EDGE_PLAN.md), [RESCUE_EDGE_RESEARCH](RESCUE_EDGE_RESEARCH.md) are the up-to-date specification/research.

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
