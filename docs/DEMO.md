# Live demonstration — Antenna

**Current authoritative recording and portal checklist:** [SUBMISSION.md](SUBMISSION.md). This document is the operator-facing walkthrough of the actual release application; older speculative hackathon storyboards are retained in Git history, not treated as product evidence.

## What the audience must understand

**Beneficiary:** someone supervising small agricultural robots working in soft or irregular ground.

**Problem:** one rover becomes stranded; rescue can save useful work, but sending a helper into dangerous mud can turn one immobilised rover into two. Human remote intervention may be slow or disruptive.

**Product action:** Antenna continuously simulates sensing, mission work and risk assessment; it can send another rover to dock and physically recover its peer or refuse a dangerous dispatch while continuing other tasks.

**Evidence:** MuJoCo contact, latch, measured displacement, seed contact, helper survival, private belief maps and recorded baseline comparisons. The result is a **simulation**, with synthetic soils and configured hazards.

## Choose one complete continuous episode for the recording

Use a known-good commit and graphics host. From the repository root:

~~~powershell
python scripts/run_mujoco_paddypatch.py --presentation --scenario golden_rescue --policy cooperative --rescue-policy react_orchestrator --seed 7
~~~

The seeded golden scenario is the **best verified end-to-end physical rescue fixture**. It is reproducible and automatically operates the real MuJoCo model; it is not a pre-rendered video. Demonstrate the disturbance and response using live sensor readings and commands in the 3D/2D view.

The required video is **≤60 seconds, a single continuous capture**. Show the ongoing simulation rather than switching between independently reset scenarios within the video. If an observed GUI/physics run cannot meet the recording requirement, do not disguise separate runs as continuous. The chosen episode must visibly include a **mid-episode event** and changed behaviour.

## Operator sequence

1. Open the known-good Presentation Mode build. Check window scaling, framerate, current seed and selected controller.
2. Position camera to show both rovers and the relevant field area; keep at least one sensor/perception layer and the acting command/decision visible.
3. Start recording once the scenario begins. Avoid editing or cutting around a physics reset.
4. Observe meaningful fieldwork before the hazard is encountered, a measured immobilisation incident, the selected helper decision, bounded approach/coupling/extraction and subsequent motion or planting.
5. Show the independently observed outcome and clearly state this is a controlled synthetic MuJoCo fixture.
6. Stop within 60 seconds and check the resulting clip's actual length, legibility, audio policy and upload compatibility.

**Do not invent a screen recording path or claim one exists until a team member has actually captured and verified it.**

## Alternate live demonstrations (not edits to the same clip)

**Safe refusal:**

~~~powershell
python scripts/run_mujoco_paddypatch.py --presentation --scenario high_risk_abstain --policy cooperative --seed 7
~~~

This controlled fixture intentionally withholds a rescue when the helper would face unacceptable risk.

**Scalable coordination:**

~~~powershell
python scripts/run_mujoco_paddypatch.py --dashboard --scenario randomized --policy mycelial --seed 8
~~~

In the engineering setup select 2, 4 or 6 rovers and a larger physical field. Inspect actual tasks, per-rover belief maps, gossip packet logs, planting, safety incidents and failed tasks. Larger-fleet rescue is not assumed to be as reliable as the controlled two-rover golden recovery.

## Five-minute pitch only if shortlisted

Suggested natural flow, rather than a rigid script:

- Introduce the stranded-rover dilemma and who benefits.
- Show the live two-rover event and the physical decision outcome.
- Explain the observation → coordination → safety gate → bounded actuator architecture.
- Show one measured matched Mycelial communication comparison and mention the competent deterministic rescue baselines.
- Close with the failure cases and the necessary real-world validation before deployment.

**No claim of live LLM benefit:** the recorded fleet evaluation made zero provider calls and used the deterministic fallback. A model-name slide or source-code backend is not evidence of live model reasoning.

## Evidence and hard cases

See [TESTING.md](TESTING.md) and [recorded experiments](../experiments/release_verification_20261108.json) for exact measurements. Six matched gossip pairs show 5 positive and 1 tied placement outcomes; these are not statistically sufficient to generalise to farms. Two four-rover runs had collisions, and one six-rover rescue suffered a helper casualty. These are important safety limitations.

The programme is **not** a physical deployment, crop-yield prediction system, RF throughput validation, or universally successful rescue machine. The current judge-facing rendering does not alter the simulator, and its measured trails must not be presented as proposed obstacle-free path planning.

## Submission separation

The team must separately approve and submit the code link, one-page technical PDF and continuous ≤60-second clip to the actual organiser portal, and retain the receipt. The repo is not the portal; this branch preparation performs no final submission.

Read [SUBMISSION.md](SUBMISSION.md) for the live deadline, deliverable review and human sign-off.
