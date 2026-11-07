# Demonstration and submission gate

**Status:** Robotics deliverables are now specified by organiser slides; no submission or tested product demo has been verified.

## Proof-first demonstration

1. **Human problem (one sentence):** who needs the action, in what real situation, and what happens without this solution?
2. **Challenge:** show the real scene, sensor trace, training failure or request. No fabricated user study.
3. **AI decision:** point to the ambiguous judgement; say why an ordinary algorithm or controller isn't equivalent.
4. **Actual change:** show the robot/simulator/system **perform** a useful action, not merely answer in chat.
5. **Independent result:** measured success/failure, baseline comparison and one honest hard case.
6. **Limits:** distinguish simulation, mock, replay and hardware. State real latency/expense only when measured.

## Rice flood-response demo storyboard — proposal, not recorded

**Single continuous ≤60-second physics-simulator recording**, with robot movement and its **actual sensor** plus selected task/drive action visible side-by-side. A simple water-flow/level visual may be labelled **synthetic**. The exact recording sequence and timings below are a *target*, not existing performance.

| Approx. elapsed video | What judge sees | What it proves |
| --- | --- | --- |
| 0–10s | service-bank rover and paddy outlet; fixed station reports an abnormal water-level observation; robot's front-range reading and current goal displayed | credible inspection beneficiary, sensor provenance, initial task |
| 10–24s | rover travels to investigate using bounded local control; live range/odometry and command values change | physics-actuated, observation-driven movement |
| 24–35s | new obstacle is injected **while recording**; sensor subsequently detects it; planned course becomes unsafe | surprise is an actual world change, not hand-picked second clip |
| 35–50s | local controller reroutes/repositions or holds safely, with trace of changed decision and observed progress | adaptation and failure handling without cloud motor-loop dependence |
| 50–60s | independent success/failure reading and, **only if genuinely implemented**, a visibly actuated outlet gate with subsequent water-proxy measurement | measured outcome and honest limits; without gate, clearly pitch **inspection/triage**, not autonomous flood prevention |

**Best-case future demo:** the scout distinguishes real rising water from a false sensor alarm, dispatches an appropriate gate actor, gate joint moves and independent synthetic hydrology responds. Add a stuck-scout/helper case only if physics and traces prove real assistance. Do not force all extensions into one fragile recording.

**Judge-facing question:** “When the rice-field drainage warning might be wrong, can a robot physically gather evidence, adapt to a surprise and trigger only justified flood mitigation?” Compare the same episode against a fixed route/static alarm and a competent reactive inspection controller; mention a more capable belief/active-sensing baseline when tested.

**What cannot be claimed:** no verified hydrology, seedling health/yield, field hardware, functional robot rescue, online/offline deployment or simulator runtime until actual run evidence. A dashboard with a moving icon is not embodied action. The user-selected direction and the five-person implementation split are documented; the human robot-design lead must confirm the rover body/sensor/actuator feasibility first.

## DeepMind presentation guidance — not the official judging rubric

**Source:** attendee photographs of the Google DeepMind AgentVerse presentation, 7 November 2026. Presenter described a 10-second first impression and an effective **30-second working moment**. Practical application to a Robotics entry:

- Lead with a **real user and an observable problem**, then immediately show a live sensor stream, robot/simulation episode or genuinely variable input; do not begin with several slides.
- Demonstrate the **closed loop**: perception → AI judgement / experiment design → deterministic/simulated action → **independent measurement** → human intervention/approval when physical or high-stakes.
- If meaningful, use more than one modality and a richer interface than a plain chat box; **do not add webcam, speech or 3D for decoration** or hide an incomplete core behind polished media.
- The presenter recommended proof in code (such as Python / optimiser verification), authentic SDK calls and a short fallback recording. Never dress up hard-coded results, fake a model call, or call a narrated animation a real agent.
- Treat the advertised “Best Use of Gemini” named prize as **slide information**, not a verified, complete award rubric. Participation in that award or using Gemini cannot replace the Robotics success criteria.

**Working pace suggested by the speaker:** Saturday morning reduce to one user/pain and one 30-second capability; Saturday afternoon prove core loop before styling; Saturday evening layer optional senses and rehearsals; Sunday stop new features roughly **two hours before the verified deadline**, record a short backup and test the demo from a clean checkout. These are suggested *internal timeboxes*, **not organiser schedule changes**. The published latest BuilderBase portal now says **Sunday 8 Nov 13:00 GMT** (older site was 14:00), so use the earlier deadline unless organiser verifies an extension.

## Demo card (fill with actual working entry)

| Item | Value |
| --- | --- |
| Beneficiary and goal | Rice growers/drainage operators: investigate an abnormal outlet/water rise and avoid harmful or unnecessary intervention (**working proposal; benefit unmeasured**) |
| Entry point and exact command | TBD |
| Test fixture/scene, seed and reset | TBD |
| Expected action/effect | TBD |
| Independent pass signal | TBD |
| Measured baseline | TBD |
| One surprising edge case | Obstacle appears mid-rover inspection; sensors cause reroute or safe hold (**not yet recorded**) |
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
