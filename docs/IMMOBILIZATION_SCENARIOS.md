# Antenna — eight physical immobilisation scenarios and independent tests

**Source:** Reconciled from teammate PR #22, 7 Nov 2026. These S1–S8 are **hypotheses to simulate**, not a validated frequency distribution for Southern China's rice fields. [RESCUE_EDGE_RESEARCH](RESCUE_EDGE_RESEARCH.md) owns scientific alternatives; [TESTING](TESTING.md) owns acceptance; [robot mechanics](../src/ricebot/robot/README.md) owns the actually merged **rigid docking/cam-latch/pitch-lift**, not a cable/winch.

| ID | Hypothesised cause | Evidence to simulate and log (with uncertainty) | Bounded response to evaluate |
| --- | --- | --- | --- |
| **S1** | Weak substrate, chassis belly sinkage, wheel spin | commanded vs independently observed displacement; wheel rotation, contact forces, body height | Stop excavating; evaluate helper only if safe approach and load budget exist |
| **S2** | Asymmetric mud traction / yaw lock | requested vs observed yaw, left/right wheel effort and translation | limited reverse along validated safe corridor, then replan or abstain |
| **S3** | High-centred on bund/ridge with drive wheels unloaded | body–ridge contacts, normal load, orientation, no motion despite wheel rotation | Hold; rigid dock if geometry permits, otherwise defer to human |
| **S4** | Drop into irrigation ditch or edge / tip risk | abrupt orientation/pose change; independent range/contact evidence | Immediate stop; exclude unsafe helper position and rollover risk |
| **S5** | Straw/roots entangled in axle or seed mechanism | high motor effort with low joint rotation, tool feedback, stagnant pose | bounded clearing motion only if safe; otherwise isolate and reallocate |
| **S6** | Stiction or suction on restart after planting/sampling pause | prior intentional HOLD, restart command, contact/effort, no translation | small validated self-attempt; real clay suction is **not** in current rigid-bed physics |
| **S7** | Rigid dock/cam-latch overload or second-robot traction failure | latch state, relative poses, both robots' actual displacement, collision and forces | abort unsafe helper actuation, unlatch safely if possible, defer rescue |
| **S8** | Narrow row pinching the chassis with separate RF/range occlusion | physical clearance/contact plus independent invalid/noisy sensor flags | distinguish true jam from radio dropout; stop, investigate or controlled reverse |

The previous PR assumed a **tether/winch** topology, a 200 Hz sensor stack, a 300–400 N cable limit, tight UWB AoA measurements and fixed 5 s, tilt or slip cutoffs. These **are not properties of the tested rover**; winch would need an explicit redesign. Do not put such thresholds into runtime or assert them as Chinese equipment standards.

## Negative controls — prevent fabricated stuck-robot diagnoses

- Intentional HOLD, parking, deliberate seed release and robot sampling must not become rescue calls.
- A missing forward rangefinder value alone is a **sensor fault**, not immobilisation.
- Unchanged distance to one neighbour cannot distinguish both robots moving together from no motion or lateral translation.
- Wheel spin without forward motion may be slip, invalid localisation, collision, traction loss or a bad command. Motor current and contact data narrow the ambiguity but do not eliminate it.
- A disabled robot may also lose compute/radio; safe recovery logic cannot require a model shard on that disabled unit.

## Runnable evaluation work needed — not yet accomplished

1. Start a real permitted sensor → decision → bounded motion rice-field episode and inject one S1–S8 condition mid-run. Keep the evaluator's hidden world truth separate from agent input.
2. Log command, odometry, wheel/joint motion, pose error, contact, sensor-valid mask, timestamps, power and message provenance. Explicitly distinguish traction loss, mechanical stall, intentional safe stop and communications failure.
3. Compare **never rescue**, **always rescue**, deterministic calibrated risk/utility optimisation and any optional contextual reasoning model under matched seeds. Include rational safe refusal where intervention risks a second casualty.
4. A typed high-level rescue intent must pass safety gating before invoking the **existing physical rigid docking** mechanism. An LLM must not issue raw torques.
5. Independently record latch engagement, actual physical extraction, second-robot outcome, further crop damage, task/seed/site work after rescue, time/energy cost and failure rates. A rendezvous, a RESCUE_NOW text label or the isolated scripted docking test alone does **not** prove autonomous rescue.
6. Repeat with changing traction, occlusion, heterogeneous risk, sensor false positives and dropped gossip; document latency and abstentions.

**AI necessity is an experiment, not a foregone conclusion:** compute quantitative risks and resource budgets mathematically. An optional LLM can interpret hard-to-formalise user priorities and uncertain incident context, but only belongs in the loop if its additional measured decisions justify latency and failure modes against a serious deterministic baseline.

**Preservation:** original eight incident classes and cooperative choices from PR #22 retained; unvalidated sensors, winch designs, fabricated fixed numeric constraints and claims of implemented autonomy corrected.
