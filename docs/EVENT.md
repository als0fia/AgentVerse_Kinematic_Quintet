# Event constraints — verify before irreversible decisions

**Checked 7 November 2026** against the [organiser website](https://agentverse.uclai.co.uk/) and previously user-supplied participant dashboard. Organiser/portal is authoritative, not this file or a generic MLH rulebook.

| Fact | Current observation | Source and qualification |
| --- | --- | --- |
| Event | AgentVerse 2026, UCL AI Society | Organiser site |
| Dates | 7–8 November 2026, London time | Organiser site |
| Track reveal | Saturday **12:30** | Organiser timetable, checked 7 Nov |
| Team formation deadline | Saturday **14:00** | Organiser timetable, checked 7 Nov |
| Hacking | Saturday **14:00–22:00** sessions; Sunday **09:00–14:00** final sprint | Organiser timetable; breaks as posted |
| Submission | Sunday **14:00** | Organiser timetable and participant dashboard; **hard planning gate**, verify portal |
| Judging | Sunday **15:00–16:00** | Organiser timetable |
| Team size | Organiser site says **3–5** | Check actual portal and any contradictory listing |
| Robotics theme (onsite slide) | **Embodied AI — “Turn perception into safe physical action.”** | Organiser presentation photographed by attendee on 7 Nov 2026. This is a track direction, **not** the exact scored challenge prompt or proof of mandated architecture |
| Examples on Robotics slide | “3D drone fleets & assistive wearables”; “MuJoCo twins or offline Gemma 4”; caption “NVIDIA & UCL Robotics” | Illustrative slide examples and named collaborators, **not** evidence of supplied drones, wearables, simulator instances, GPUs, model access or credits |
| Model/tool slide | “Gemini 4 Argon” (deep reasoning); “3.8 Flash + Gemma 4” (speed/offline); “3.8 Live + Speech” (vision/voice); “Banana · Lyria · Veo” (media) | Organiser slide photographed 7 Nov. **Gemini 4 Argon and Gemma 4 are distinct labels.** These are presentations, not verified entitlements, installed packages or tested model IDs |
| Hardware/sim/credits | **Not verified** | Ask at reveal / official resources |
| Prior work, AI, IP, submission fields | **Not verified** | Obtain actual written event-specific rules |

## Challenge capture (fill only from verified reveal)

- **Exact official problem statement/link:** not captured; do not reinterpret the broad Robotics track theme or example use cases as a mandatory challenge
- **Judging rubric + sponsor eligibility:** not captured
- **Mandatory technical constraints and resources:** not captured
- **Demo/video/repository/format requirements:** not captured
- **Allowed reuse, licences and AI disclosure:** not captured
- **Observation time and source:** 7 Nov 2026, onsite organiser slides supplied by an attendee (photos in conversation); not independently confirmed by a written portal brief. Model slide also mentions a claimed 1M window / Antigravity sandbox for Gemini 4 Argon, Gemma 4 E2B/E4B offline, and a reasoning-effort dial. Treat these as **slide claims only**, not verified technical access or implementation decisions
- **Conflicts or interpretations for organiser clarification:** none recorded

## DeepMind onsite technical guidance (attendee photographs, 7 November)

The Google DeepMind presenter showed three **optional** starting routes, not mandatory submission scaffolds:

- **AI Studio Build:** https://aistudio.google.com/build — prototype UI, shader/3D elements and Gemini calls; sample model string `gemini-4-argon`.
- **AI Studio Live:** https://aistudio.google.com/live — webcam/voice and low-latency multimodal tests; example strings `gemini-3.8-live`, `gemini-3.8-flash-tts` and `nano-banana-2.1`. These **slide strings** have not all been confirmed as currently available API identifiers.
- **Antigravity + Skills:** suggested IDE/CLI tools and the command `npx skills add google-gemini/gemini-skills`. **Do not run fetched scripts or install tools just because they appeared on a slide**; inspect the source, version, licences and execution effects first.

The slide encouraged collecting any onsite credits from the DeepMind booth; the **amount, applicability, account permissions and billing consequences remain unverified**. It also presented a **“Best Use of Gemini” named prize** and described combining modalities, a genuine working reasoning/proof loop, and an experience beyond a plain chat box. This is **presenter guidance and claimed award framing**, not the complete official scoring rubric or a rule that every project must use several models.

The presenter offered examples of recent London projects and described an internal audit of “200+” previous Gemini hackathon repositories. Treat these as **presenter-reported examples**, not independently verified competition results, safety proofs or measured performance.

**Externally checked model/API facts (7 Nov):** Google documents Gemini 4 Argon model ID `gemini-4-argon`, the Interactions API and reasoning levels LOW / MEDIUM / HIGH. See [Google's model guide](https://docs.cloud.google.com/gemini-enterprise-agent-platform/models/guides/gemini-4-argon) and [Interactions API](https://ai.google.dev/gemini-api/docs/interactions-overview). That verifies API documentation, **not our team account's entitlement**. Automatic implicit caching exists but “95% cached” is **not a guaranteed rate**. For use as software-engineering assistants, see [team model routing](TEAM.md); for the actual presentation, see [demo guidance](DEMO.md).

Do not paste confidential screenshots, participant emails, Discord transcripts or access tokens. Summarise only team-authorised constraints and source provenance.
