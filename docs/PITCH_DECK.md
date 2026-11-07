# Antenna — five-minute Robotics shortlist pitch (evidence-controlled draft)

**Track:** AgentVerse 2026, Robotics. **Product:** Antenna — risk-aware peer rescue during useful rice-field work. **Status:** proposed shortlist talk, not a recorded/presented talk. **Source:** [PR #48 integrated rescue](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/pull/48) and [combined main GitHub checks](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/actions/runs/42418413738) on commit `16768dc`. Do not deliver numbers not reproduced in the final demo.

**Claim boundary:** The 78-test GitHub suite passed on the merged MuJoCo code, including a one-scenario rescue episode with force/pitch limits and two subsequent seed drops. This is a **simulation prototype**, not a physical rover, real rice/soil field trial, wireless sensor network, Jetson benchmark, proven model advantage, or verified farm economics. The current fault signal can incorporate simulator-internal `is_bogged` state; label it a simulated proxy until a permitted-sensor-only diagnosis is independently established. The operator's video, PDF and portal confirmation remain separately required.

## Slide 1 — Whose problem, and why rescue? (0:00–0:45)

**Show:** A rice-field rover performing a useful sample/plant task, followed by a stranded machine and a second rover facing a risky retrieval decision.

**Say:** A small robot can sense and work in a field yet be stranded by soft terrain. Sending a second robot to tow it may recover valuable work—or immobilize both. The beneficiary is the farmer/operator managing several machines. The measurable outcome is **safe restoration of mobility and useful task progress, or a justified refusal that avoids a second casualty**.

**Caveat:** The field problem is plausible; actual prevalence, yield and labor savings have not been measured by this project.

## Slide 2 — The single physical-simulation system (0:45–1:35)

**Show:** Two simulated four-wheel Paddy Ant rovers in one MuJoCo terrace; the mapped sampling/planting sites, modeled rigid front/rear docking assembly, bounded drive, coupling and seed-release.

**Say:** We reused one existing `src/ricebot/` physics and `rice.v1` observation/action loop, rather than creating another farm simulator. The simulation includes location-gated synthetic soil probes, range readings, simplified terramechanics, a rescue coupling, and seed-ground contact verification.

**Important:** The geometry is modeled. The simulation latch uses a MuJoCo equality constraint; it is not a manufactured mechanical latch tested at physical load. No deployed radio, camera-perception model or hardware rescue mechanism is claimed.

## Slide 3 — Where decisions happen (1:35–2:25)

**Show:** An observed incident progressing through a typed decision/action/feedback diagram:

`observation + command history → deterministic rescue-worth assessment / safety veto → TaskMode.RESCUE or HOLD/DEFER/ABANDON → bounded approach/latch/extraction → RecoveryFeedback → resumed fieldwork`.

**Say:** Fast, low-level movement and interlocks are deterministic. The supervisor considers mission opportunity cost and the risk of losing its helper. On the golden scenario the current `CooperativeSwarmPlanner` is the centralized controller. Separate Policy B local maps and simulated gossip exist, but they are **not** the controller of this golden rescue episode.

**AI honesty:** The optional text-model adviser exists as exploratory assistance. Neither a deployed LLM tool-loop nor an improvement over calibrated deterministic rescue triage has been independently demonstrated. We will not call a numerical threshold a novel LLM agent.

## Slide 4 — One uninterrupted surprise-and-response episode (2:25–3:40)

**Show:** A **real continuous ≤60-second recording** from the final checked-in MuJoCo command, with actual simulator time, physical scene and contemporaneous simulated sensor/decision/action traces visible. Do not substitute a hand-edited animation or a separate docking fixture without labeling it.

**Story beats:** (1) rover executes fieldwork; (2) a mid-episode mud trap stalls it; (3) sensed/proxy evidence and mission trade-offs produce a rescue decision; (4) helper approaches, engages a bounded modeled cam-latch and attempts extraction; (5) the latch opens and the rover resumes seed planting, **or** safety risk requires HOLD/refusal.

**Current automated evidence:** The integrated test asserts one completed recovery, simulated peak equality-constraint force <250 N, absolute chassis pitch <0.12 rad and **two** subsequent seed drops. These are tests under the configured scenario—not proof of traction robustness or success on untested incidents.

**Presenter action:** Replace all generic statements above with exact timestamps and events observed in the final screen capture; do not manufacture sensor readouts, distances, forces, weather, or crop conditions.

## Slide 5 — Evidence versus baselines (3:40–4:30)

| Question | Current observed or checked-in evidence | What remains unproven |
| --- | --- | --- |
| Does the complete rescue call chain execute? | Merged `golden_rescue` test passes in 78-test CI run | Broad reliability across seeds/hazards |
| Is coupling numerically bounded in that episode? | Test asserts force <250 N and pitch <0.12 rad | Physical latch realism and external force calibration |
| Does fieldwork resume? | The golden test asserts two completed drops | Field yield/placement accuracy outside simulation |
| Does rescue improve mission utility against a non-agent baseline? | Deterministic planner and example evaluation infrastructure exist | Fair matched no-rescue, always-rescue and calibrated-risk comparisons |
| Is safe refusal robust? | Existing triage/safety logic | Independent high-drag, failed-latch and secondary-casualty tests |
| Does an LLM/tool-loop beat a deterministic controller? | Optional adviser/fallback | Real model-call logs, changed decisions, matched outcomes and latency |

**Say:** The result is a concrete integration demonstration, not a claim of a 100% recovery rate, higher yields, zero second casualties under all conditions or an invented benchmark advantage. Highlight the limits and one decisive next test.

## Slide 6 — Practical next steps (4:30–5:00)

**Show:** Three evidence-gated milestones: mechanical/sensor-provenance validation; robust incident/abstention trials; physical hardware and field evaluation later.

**Say:** The engineering priority is not another simulator, climate model or cloud service. It is to prove safe, repeatable robot-to-robot recovery under more than one terrain condition. Later real hardware work would require verified actuator forces, sensor calibration, radio limits, battery/power numbers, permitted field tests and realistic economics.

**Do not claim:** Measured Jetson CPU utilization/power, 12 ms inference, a manufacturing BOM/payback period, 100% communication resilience, production deployment or measured operator/agricultural benefit.

## Judge questions — concise answers

**Why not use one heavier tractor?** That is a legitimate alternative and needs an application-specific cost/soil-disturbance comparison. Our prototype tests a possible recovery capability for small field robots; it does not prove commercial or agronomic superiority.

**Where does AI help rather than ordinary code?** Low-level control should stay deterministic. Contextual supervisory judgement might help weigh uncertain external information, operator priorities, new evidence and changing mission objectives. That advantage must be demonstrated against a good numerical decision policy. We have not yet proven it.

**How do you avoid losing the second robot?** The intended safety policy assesses approach and towing risk and can refuse a rescue. The prototype has bounded controller mechanics; a complete independent dangerous-terrain/secondary-casualty acceptance sweep is still required.

**What is simulated?** The robots, terrain, sensors/radios and coupling are MuJoCo/synthetic models. The seed contact and recovery feedback are in-simulator physical events; field validation and deployed networked AI are future work.

**Could the solver pull the robots together unnaturally?** Yes. M4 identified this risk. The merged integration uses tighter docking and softer constraints, with assertions on recorded constraint force and pitch. That narrows risk in one test; it does not replace broader physical validation or a real prototype.

---

**Operational prerequisite:** Match every screenshot, on-screen metric and quote to the final executed run. Prepare the organiser-required ≤60-second single-take video and one-page technical PDF independently. The human team owns the participant-portal submission and final receipt; neither CI nor this draft is a submission.
