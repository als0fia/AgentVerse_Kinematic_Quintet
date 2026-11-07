# Event constraints — verify before irreversible decisions

**Checked 7 November 2026** against the [organiser website](https://agentverse.uclai.co.uk/) and previously user-supplied participant dashboard. Organiser/portal is authoritative, not this file or a generic MLH rulebook.

| Fact | Current observation | Source and qualification |
| --- | --- | --- |
| Event | AgentVerse 2026, UCL AI Society | Organiser site |
| Dates | 7–8 November 2026, London time | Organiser site |
| Track reveal | Saturday **12:30** | Organiser timetable, checked 7 Nov |
| Team formation deadline | Saturday **15:00** (extension announced by organiser 7 Nov 13:56) | Organiser email supersedes the earlier 14:00 timetable |
| Hacking | Saturday **14:00–22:00** sessions; Sunday **09:00–14:00** published sprint, but **stop for the earlier 13:00 portal submission cutoff** | Organiser timetable versus newer participant portal; don't treat the session end as a submission extension |
| **Submission — latest portal** | **Sunday 8 Nov, 13:00 GMT** latest BuilderBase dashboard (7 Nov ~15:30); older organiser site/deck listed 14:00 | **Conflict:** use earliest 13:00 until staff/portal clarify; do not assume an extra hour |
| Judging | Sunday **15:00–16:00** | Organiser timetable |
| Team size | Organiser site says **3–5** | Check actual portal and any contradictory listing |
| Robotics theme (onsite slide) | **Embodied AI — “Turn perception into safe physical action.”** | Organiser presentation photographed by attendee on 7 Nov 2026. This is a track direction, **not** the exact scored challenge prompt or proof of mandated architecture |
| Examples on Robotics slide | “3D drone fleets & assistive wearables”; “MuJoCo twins or offline Gemma 4”; caption “NVIDIA & UCL Robotics” | Illustrative slide examples and named collaborators, **not** evidence of supplied drones, wearables, simulator instances, GPUs, model access or credits |
| Model/tool slide | “Gemini 4 Argon” (deep reasoning); “3.8 Flash + Gemma 4” (speed/offline); “3.8 Live + Speech” (vision/voice); “Banana · Lyria · Veo” (media) | Organiser slide photographed 7 Nov. **Gemini 4 Argon and Gemma 4 are distinct labels.** These are presentations, not verified entitlements, installed packages or tested model IDs |
| Simulation requirement | **Simulated embodied agent** with at least one simulated sensor, a decision made from current sensed state, and an observable action; not just a pre-scripted route | **Robotics track organiser slide 35**, attendee photo 7 Nov 2026; exact sensor type and solution unrestricted in captured slide |
| Simulator choices | **MuJoCo now recommended first** in later organiser Robotics setup email; Isaac Sim on Modal is **stretch for experienced teams** | Earlier slide favoured Isaac Sim; newer organiser instructions recommend laptop MuJoCo. PyBullet, Gazebo and Webots remain permitted |
| Modal credits/GPU | Organiser email says **approximately $50 credit each**, with redemption via official form; **actual redemption, quota and billing not verified** | 7 Nov 15:16 organiser broadcast; use L40S rather than A100/H100 for Isaac/RTX if that stretch route is chosen; no GPU run asserted |
| Hardware/sim/credits | User has received a separate **temporary hackathon Google development-account** email; **not proof of login or API entitlement** | 7 Nov 15:22 email. No credentials in Git; hackathon-only usage, registered-team sharing boundary and ephemeral deletion after event |
| Prior work, AI, IP, submission fields | **Not verified** | Obtain actual written event-specific rules |

## Challenge capture (fill only from verified reveal)

- **Observed Robotics challenge statement:** “Give an agent a body.” Build an embodied agent that perceives a simulated world, reasons from sensed observations and acts autonomously. **Slides 35–38**, user-shared organiser photos on 7 Nov 2026; written official portal PDF/link not yet captured.
- **Judging:** the additional **clear, complete organiser-slide photograph supplied by the attendee on 7 November 2026** resolves all five categories and weights. See the exact 100% rubric below. Presenter guidance warns against cloud reasoning for each control decision; remote simulation streaming is not itself disallowed. Sponsor-specific prize criteria remain unverified.
- **Technical constraints/resources:** at least one simulated sensor, observable action (navigation/movement/manipulation/communication), autonomous reaction to sensed state, and demonstrated recovery when a scene changes. Recommended Isaac Sim + headless streaming; others listed above. Actual Modal GPU credits, sensor feed, available GPU and model endpoints are *not* validated.
- **Deliverables (slide 37):** (1) **one-page PDF technical summary** with beneficiary/problem, features, architecture, model placements, hardware/latency, limitations and safety; (2) **≤1-minute video from simulation stream** showing perception and actions side-by-side and an unexpected change in a **single continuous take**; (3) **five-minute live pitch only if shortlisted**. Portal submission fields, upload formats and link requirements still unverified.
- **Allowed reuse, licences and AI disclosure:** not captured
- **Observation time and source:** 7 Nov 2026, initial DeepMind introduction slides plus newer **Robotics-specific organiser slides 35–38** and attendee-provided presenter STT. The Robotics-specific later material supersedes earlier broad examples where narrower. Written brief link still pending. Model slide also mentions a claimed 1M window / Antigravity sandbox for Gemini 4 Argon, Gemma 4 E2B/E4B offline, and a reasoning-effort dial. Treat these as **slide claims only**, not verified technical access or implementation decisions
- **Conflicts or interpretations for organiser clarification:** none recorded

## Confirmed Robotics judging rubric — clear organiser photograph, 7 November 2026

**Primary evidence:** Additional attendee-supplied, unobscured photograph of the organiser's **“How you'll be judged”** slide on 7 November 2026, superseding earlier obscured photographs. The percentages are now fully legible and add to **100%**. This is the Robotics scoring rubric, not DeepMind's optional sponsor-prize guidance.

| Category | Weight | Organiser's top-score description (faithful paraphrase) |
| --- | ---: | --- |
| Perception-driven action | **25%** | Every decision is traceable to perception, and behaviour corrects during execution |
| Adaptation to change | **25%** | The scene changes mid-sequence; the agent visibly replans |
| Operational autonomy | **15%** | Executes within a safe edge-device budget and behaves sensibly when unsure |
| Relevance and impact | **20%** | Addresses a well-evidenced need and **handles its most critical failure mode on screen** |
| Ambition and technical quality | **15%** | A technically demanding, original approach executed to a high standard |

**Selection consequences:** The **20% impact** category is substantive: write down the actual beneficiary/workflow, evidence for need (source, observed workflow, user testimony or credible documented problem), measurable benefit, and most consequential realistic failure before declaring a concept selected. A dramatic robot with no useful job is weaker than an evidenced tool with a credible test. The two **25% behaviour** categories still require a true sensed-action loop and unplanned mid-episode change; the 15% autonomy category requires bounded on-device/inside-runtime decision-making and explicit uncertainty handling. Do not confuse a video showing a failure with handling that failure.

## Implementation implications from Robotics-specific official reveal

**Perceive → decide → act, with feedback and variation.** The agent must consume **at least one sensor measured from the simulated world** and produce a grounded action that affects the next world state. Require visible changes in inputs and actions when an obstacle, goal, equipment state or scene condition changes. A prerecorded trajectory, fixed sequence, or UI narration without an actual closed loop is not a passing demo.

**Autonomy is a behaviour, not a model brand.** Keep short-horizon action selection and safety checks inside the simulation/robot runtime where practical; benchmark an offline/network-loss condition rather than assuming cloud AI. A high-level LLM can help plan/diagnose if justified, but remote inference on every actuation tick is risky for latency, reliability and the rubric. **Remote GPU simulation/streaming is a separate issue from remote agent decision-making.**

**NVIDIA caveats independently verified:** [Isaac Sim requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html) are demanding and [WebRTC livestream docs](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/manual_livestream_clients.html) require NVENC, with a warning against insecure public stream exposure. [MuJoCo Python](https://mujoco.readthedocs.io/en/stable/python.html) is a documented alternative for a lighter local test loop. The presenter's vague GPU model in STT, the example GitHub launcher, issued Modal credits and live endpoint permissions remain unverified.

**Engineering checklist:** a reproducible scene/seed; instrumented sensor; inspectable decision/log; action with bounds; mid-episode disturbance; behaviour change; objectively scored outcome; contrasted baseline; visible video capture; complete summary/pitch. No selected product, simulator, installed models, tests or credits are implied merely by this checklist. Official portal remains final authority for submission mechanics.

## DeepMind onsite technical guidance (attendee photographs, 7 November)

The Google DeepMind presenter showed three **optional** starting routes, not mandatory submission scaffolds:

- **AI Studio Build:** https://aistudio.google.com/build — prototype UI, shader/3D elements and Gemini calls; sample model string `gemini-4-argon`.
- **AI Studio Live:** https://aistudio.google.com/live — webcam/voice and low-latency multimodal tests; example strings `gemini-3.8-live`, `gemini-3.8-flash-tts` and `nano-banana-2.1`. These **slide strings** have not all been confirmed as currently available API identifiers.
- **Antigravity + Skills:** suggested IDE/CLI tools and the command `npx skills add google-gemini/gemini-skills`. **Do not run fetched scripts or install tools just because they appeared on a slide**; inspect the source, version, licences and execution effects first.

The slide encouraged collecting any onsite credits from the DeepMind booth; the **amount, applicability, account permissions and billing consequences remain unverified**. It also presented a **“Best Use of Gemini” named prize** and described combining modalities, a genuine working reasoning/proof loop, and an experience beyond a plain chat box. This is **presenter guidance and claimed award framing**, not the complete official scoring rubric or a rule that every project must use several models.

The presenter offered examples of recent London projects and described an internal audit of “200+” previous Gemini hackathon repositories. Treat these as **presenter-reported examples**, not independently verified competition results, safety proofs or measured performance.

**Externally checked model/API facts (7 Nov):** Google documents Gemini 4 Argon model ID `gemini-4-argon`, the Interactions API and reasoning levels LOW / MEDIUM / HIGH. See [Google's model guide](https://docs.cloud.google.com/gemini-enterprise-agent-platform/models/guides/gemini-4-argon) and [Interactions API](https://ai.google.dev/gemini-api/docs/interactions-overview). That verifies API documentation, **not our team account's entitlement**. Automatic implicit caching exists but “95% cached” is **not a guaranteed rate**. For use as software-engineering assistants, see [team model routing](TEAM.md); for the actual presentation, see [demo guidance](DEMO.md).

Do not paste confidential screenshots, participant emails, Discord transcripts or access tokens. Summarise only team-authorised constraints and source provenance.

## Later organiser resources and live portal revision — 7 November ~15:30 GMT

**Source class:** participant-provided BuilderBase overview and organiser emails received 7 Nov 15:16/15:22; new information supersedes earlier *recommendation* for first simulator and old submission-time planning. Dashboard says **“Live, submissions close tomorrow at 13:00 GMT”**; old site and deck said 14:00 Sunday. **Operationally plan for Sunday 8 Nov 13:00 GMT and ask organiser to resolve the discrepancy.** Dashboard confirms linked real GitHub repo (**1 of 2** submission fields done), remaining item **“Simulation — Add”**; do not infer its upload field details or that a final submission has been completed.

The newer official setup email recommends **MuJoCo on laptops**, with **Modal optional for heavyweight GPU training**; Isaac Sim + L40S Modal is a stretch and may output frames instead of interactive streaming. Source examples: https://github.com/google-deepmind/mujoco , https://github.com/google-deepmind/mujoco_menagerie , and https://github.com/TheFloatingString/isaac-lab-on-modal . **Actual access, model inference entitlements and running physics remain unverified.** A temporary Google hackathon account has been received by email, but credentials are never saved here; terms restrict it to official event work and say account contents will be deleted after the event. No account activation, credits redemption or paid use is asserted.
