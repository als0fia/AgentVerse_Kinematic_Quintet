# Demonstration and submission gate

**Status:** Nothing has been submitted. No tested product demo exists yet.

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

**Working pace suggested by the speaker:** Saturday morning reduce to one user/pain and one 30-second capability; Saturday afternoon prove core loop before styling; Saturday evening layer optional senses and rehearsals; Sunday stop new features roughly **two hours before the verified deadline**, record a short backup and test the demo from a clean checkout. These are suggested *internal timeboxes*, **not organiser schedule changes**. The published Sunday submission planning time remains 14:00 London time until the portal says otherwise.

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

## Submission preflight (must be human checked)

- [ ] Portal **deadline and timezone** reconfirmed (organiser timetable: Sunday 8 Nov, **14:00 London time**, checked 7 Nov).
- [ ] Current track brief and entry criteria complied with; no unsupported historical MLH rules substituted.
- [ ] Team is eligible; repository URL and access meet actual requirements.
- [ ] All required fields/video/images/links confirmed from current portal.
- [ ] README, exact install and demo commands work on selected commit.
- [ ] No secrets, private captures, unlicensed assets, undisclosed preexisting work or inaccurate AI attribution.
- [ ] Metrics/claims are reproducible and simulation vs reality explicit.
- [ ] Human team owner has approved final content and action.
- [ ] Submission receipt or portal confirmation has been observed and recorded by authorised team member.

A ready GitHub repo, a prepared video or a form draft **is not** submission confirmation. Prioritise a known-good executable vertical slice over graphics-only polish. Judging duration and exact scoring must come from current organiser instructions, not assumed conventions.
