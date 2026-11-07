# Demonstration and submission gate

**Current user-directed product scope — Antenna, rescue-first (7 Nov 2026):** The point of the system is **risk-aware robot-to-robot physical recovery** while robots carry out useful rice-field sensing, mapping and planting. The earlier planting-first/rescue-optional instructions below record existing work, not the current primary research question. [D-0008](DECISIONS.md) remains the canonical **naming** decision; new [D-0009](DECISIONS.md) records the **scope** correction. The merged rigid-docking MJCF and `test_mujoco_rigid_docking_cam_latch_and_tandem_extraction` show a **scripted physics extraction test** (victim moves >0.08 m), but not an autonomous sensed-fault→rescue→mission-continuation episode, learned policy or live Jetson inference. [SWARM_EDGE_PLAN](SWARM_EDGE_PLAN.md) / [RESCUE_EDGE_RESEARCH](RESCUE_EDGE_RESEARCH.md).

**Status:** Robotics deliverables are now specified by organiser slides; no submission or tested product demo has been verified.

## Rescue-first judge story

Antenna robots visibly do useful rice-field work, a rover experiences a **real physical fault**, the system uses local observations and mission/terrain uncertainty to decide whether to assist, and the helper either executes a bounded **physical** rigid-docking extraction or safely refuses. Grade actual contact/displacement, second-robot risk and post-rescue fieldwork in the same continuous ≤60-second episode. Today's rigid-docking unit test is scripted physics, **not yet** an integrated recovery policy, learned skill or cloud-free distributed LLM. If it is not integrated before submission, show the actual agricultural physics and scripted extraction honestly, without implying autonomy. [Current PRODUCT](PRODUCT.md).

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

## Antenna: rescue-first continuous demonstration target (NOT yet recorded)

**Beneficiary story:** two cooperative agricultural rovers perform real mapped/sampled rice work. One becomes physically immobilised; the other must judge whether intervention is safer and more valuable than continuing its own tasks. The system verifies recovery (or justified abstention) in physical simulation before claiming any benefit. Agricultural yield, farm labour savings and real mud behaviour are *not measured*. See [PRODUCT](PRODUCT.md) and [TESTING](TESTING.md).

| Target 60-second segment | What judges should actually observe | Evidence requirement |
| --- | --- | --- |
| 0–10 s | Two physical rovers already carrying out a sampled soil/seed task in one MuJoCo scene | live poses, issued motor commands and sensed local task observations |
| 10–20 s | **Unexpected** stall/traction event affects one rover during active movement | previous commanded motion, subsequent measured displacement/heading, sensor-validity and incident timer; not just a “stuck” toggle or missing front range |
| 20–32 s | Supervisor weighs crop/mission value, uncertain route risk, rescuer traction and recovery alternatives | typed calculated verdict with alternative baseline including possible **defer/abstain**; model output never drives wheel torques |
| 32–47 s | If judged feasible, helper approaches, physically couples rigid latch and extracts; otherwise safe refusal, hazard mark and task reassignment | coupling/contact state, bounded actuator events, **measured victim displacement and helper safety**; isolated scripted docking test alone is insufficient |
| 47–55 s | Recovered rover or remaining helper returns to meaningful sensing/seed task | new *actual* probe/seed-contact feedback after incident, or an explicit safe abort |
| 55–60 s | Independent result and clear limitations | timestamps, success/failure, second casualty, time/energy/crop risk, comparison baseline using the same scenario seed |

**Hard truth gate:** the repo currently lacks this fully integrated autonomous rescue episode. The merged PR #24 scripted rigid docking physics test and the real seed-contact/Policy B unit tests are separate evidence. Do not splice separate recordings or describe a scripted reset as an adaptive uninterrupted run. Preserve the planting-only storyboard as historical Git history. Flood gates, full crop season, multi-GPU model splitting and additional rescue mechanisms are not P0.

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
