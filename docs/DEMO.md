# Demonstration and submission gate

**Latest user correction — rescue-first (7 Nov 2026):** The primary product is risk-aware robot-to-robot *physical recovery*, with rice sensing/mapping and planting as meaningful mission context. Earlier planting-first/rescue-optional language below is historical implementation guidance, not the current product priority. Existing physical code and tests are preserved. The merged swarm planner's triage and `rescue_assist` rendezvous do **not** implement actual physical recovery. [Current product](PRODUCT.md), [architecture/research plan](SWARM_EDGE_PLAN.md), [research](RESCUE_EDGE_RESEARCH.md), [D-0008](DECISIONS.md).

**Status:** Robotics deliverables are now specified by organiser slides; no submission or tested product demo has been verified.

## Proof-first demonstration

1. **Human problem (one sentence):** who needs the action, in what real situation, and what happens without this solution?
2. **Challenge:** show the real scene, sensor trace, training failure or request. No fabricated user study.
3. **AI decision:** point to the ambiguous judgement; say why an ordinary algorithm or controller isn't equivalent.
4. **Actual change:** show the robot/simulator/system **perform** a useful action, not merely answer in chat.
5. **Independent result:** measured success/failure, baseline comparison and one honest hard case.
6. **Limits:** distinguish simulation, mock, replay and hardware. State real latency/expense only when measured.

## DeepMind presentation guidance — not the official judging rubric

**Source:** attendee photographs of the Google DeepMind AgentVerse presentation, 7 November 2026. Presenter described a 10-second first impression and an effective **30-second working moment**. Practical application to a Robotics entry:

- Lead with a **real user and an observable problem**, then immediately show a live sensor stream, robot/simulation episode or genuinely variable input; do not begin with several slides.
- Demonstrate the **closed loop**: perception → AI judgement / experiment design → deterministic/simulated action → **independent measurement** → human intervention/approval when physical or high-stakes.
- If meaningful, use more than one modality and a richer interface than a plain chat box; **do not add webcam, speech or 3D for decoration** or hide an incomplete core behind polished media.
- The presenter recommended proof in code (such as Python / optimiser verification), authentic SDK calls and a short fallback recording. Never dress up hard-coded results, fake a model call, or call a narrated animation a real agent.
- Treat the advertised “Best Use of Gemini” named prize as **slide information**, not a verified, complete award rubric. Participation in that award or using Gemini cannot replace the Robotics success criteria.

**Working pace suggested by the speaker:** Saturday morning reduce to one user/pain and one 30-second capability; Saturday afternoon prove core loop before styling; Saturday evening layer optional senses and rehearsals; Sunday stop new features roughly **two hours before the verified deadline**, record a short backup and test the demo from a clean checkout. These are suggested *internal timeboxes*, **not organiser schedule changes**. The published latest BuilderBase portal now says **Sunday 8 Nov 13:00 GMT** (older site was 14:00), so use the earlier deadline unless organiser verifies an extension.

## Preserved agricultural baseline: ≤60-second rice planting episode (previous priority)

**User-facing pitch:** “A rice field has irregular failed emergence. Two autonomous robots gather different soil and crop-condition measurements, build a shared suitability map and coordinate where to replant—with fewer wasted seeds and safer routes. They must keep planting when new ground conditions invalidate their plan.” **Simulated benefit only; no claim of measured rice survival/yield.**

| Target video segment | What the judge must actually see |
| --- | --- |
| 0–10 s | Prepared/partly drained stylised rice plot with patchy crop stand, two physical rovers, seed inventory and **uncertain shared soil map**; no hidden-quality labels in agent view |
| 10–23 s | Two robots **physically move and sample separate local areas**; sensor readings, robot IDs/timestamps and belief/uncertainty heatmap change as evidence is fused |
| 23–33 s | Planner chooses viable stand-gap sites and distinct **safe routes** for scout and planter; actual bounded drive commands and positions shown |
| 33–44 s | Mid-episode new obstacle or locally discovered unsuitable soil **invalidates a chosen target/route**; fresh sensor drives map/target/route update; no scripted second clip |
| 44–55 s | Planter physically reaches a valid gap and **releases seed proxy from its tool**, with rigid-body landing/contact/inventory result visible |
| 55–60 s | Independent outcome: actual landed seeds in valid gaps, synthetic benefit/seed or coverage, safety violations and a baseline comparison **only if truly measured** |

The real episode may need simplification; do not falsify elapsed runtimes or pretend video transitions are continuous. Maintain one genuine changing-world episode showing sensor → decision → physical actuation → new observation; even if video cannot fit every metric, provide logs and one-page PDF with tested independent results. The UI is **read-only**, no hidden world truth steering policy.

**Not in the P0 demo:** flood warning as main mission, opening a drainage gate or lowering water, robot towing/rescue, extra reforestation or lunar environment, VLA training, photorealistic field science. These can be named as optional future applications only after planting is real. Present a failed/drop misfire as a real failure, not a successful ghost planting.

**Mechanical plausibility:** direct-seeded or partially drained field, safe traversable lanes, physical rover wheels/dropper and local probe/range. Synthetic soil quality and establishment evaluation explicitly labelled. [PRODUCT](PRODUCT.md) and [TESTING](TESTING.md) govern what is actually claimed.

## Demo card (fill with actual working entry)

| Item | Value |
| --- | --- |
| Beneficiary and goal | TBD |
| Entry point and exact command | TBD |
| Test fixture/scene, seed and reset | TBD |
| Expected action/effect | TBD |
| Independent pass signal | TBD |
| Measured baseline | TBD |
| One surprising edge case | TBD |
| Observed live-demo result/commit | TBD |
| Fallback recording or deterministic replay (clearly labelled) | TBD |
| Operator and handover | TBD |

## Judge-aligned story: real usefulness, not only technical spectacle

The [confirmed organiser rubric](EVENT.md#confirmed-robotics-judging-rubric--clear-organiser-photograph-7-november-2026) assigns **20% to relevance and impact** and explicitly values handling the **most critical failure mode on screen**. The pitch and one-page PDF must be specific: beneficiary; measured or source-backed need; why the current controller/robot/workflow fails; what useful outcome improves; and how our system behaves in its worst credible scenario. Do not invent interviews, adoption, statistics or simulated safety certification. This user story complements, rather than replaces, the 50% perception/adaptation evidence.

## Robotics track deliverables (7 Nov organiser slides 35–38)

These **supersede generic demo suggestions** where they differ.

| Requirement | Content / capture criterion | Owner / status |
| --- | --- | --- |
| **Technical summary — one page PDF** | Real user/problem, features, overall perception–decision–action architecture, which model runs where, hardware and measured latency, limitations, key failure mode and safety | **Not yet authored**; PDF creation belongs to the team once product evidence exists |
| **Simulation video — up to one minute** | Actual simulation stream, sensor/perception data **alongside** agent actions, and one unexpected mid-run scene change in a **single continuous take** | **Not yet recorded**; use genuine episode, not edited transitions or canned path |
| **Pitch — five minutes if shortlisted** | What problem is solved, how the robot works, demonstrated capabilities, limitations and value to the beneficiary | **Not yet required for every team**; prepare a grounded outline after core behaviour works |

**Suggested storyboards (illustrative only):** start with the user problem and sensor viewpoint; show first action; introduce one unexpected change; show detected state and replan; show task outcome and limitation. This is not permission to fake the event. Record a separately labelled fallback video only if the genuine continuous run cannot be shown live.

## Submission preflight (must be human checked)

- [ ] Portal **deadline and timezone** reconfirmed (organiser timetable: **Sunday 8 Nov, 13:00 GMT** according to latest BuilderBase portal (older site said 14:00; confirm with staff), checked 7 Nov).
- [ ] Current track brief and entry criteria complied with; no unsupported historical MLH rules substituted.
- [ ] Team is eligible; repository URL and access meet actual requirements.
- [ ] All required fields/video/images/links confirmed from current portal.
- [ ] README, exact install and demo commands work on selected commit.
- [ ] No secrets, private captures, unlicensed assets, undisclosed preexisting work or inaccurate AI attribution.
- [ ] Metrics/claims are reproducible and simulation vs reality explicit.
- [ ] Human team owner has approved final content and action.
- [ ] Submission receipt or portal confirmation has been observed and recorded by authorised team member.

A ready GitHub repo, a prepared video or a form draft **is not** submission confirmation. Prioritise a known-good executable vertical slice over graphics-only polish. Judging duration and exact scoring must come from current organiser instructions, not assumed conventions.
