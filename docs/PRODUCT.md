# Product — Antenna

**Robotics track, AgentVerse 2026 · The Kinematic Quintet**

## How this meets the Robotics relevance criterion

**Relevance and impact — 20%** of the organiser's recorded Robotics judging rubric requires a meaningful beneficiary and an observable response to the most consequential failure mode. Our specific test is whether Antenna can handle a stranded field rover without unnecessarily sacrificing another rover, while continuing real simulated soil and planting tasks. We have measured these outcomes in MuJoCo; real farming impact has not been validated. See [EVENT](EVENT.md) for the other four categories and source qualifications.

## Who benefits and what changes?

Antenna investigates an operational problem faced by future operators of small field robots: when a rover becomes immobilised in uncertain terrain, should another rover interrupt productive work to help, or would that create a second casualty? A fully autonomous fleet needs a defensible answer, not an unconditional rescue instruction.

**Proposed beneficiary:** agricultural robotics operators working in small, wet or irregular field areas, particularly where remotely supervising each machine may be expensive or impractical. The use case is motivated by documented difficulty mechanising hilly rice cultivation; see the sourced [commercial and sector evidence review](COMMERCIALIZATION_CHINA.md). We have **not** interviewed adopters, measured rescue frequency in real fields or established a commercial market size.

**Observable product outcome:** keep useful planting and soil-mapping work moving while a fleet detects immobilisation, evaluates rescue risk, physically assists where justified, or refuses unsafe assistance and reallocates work.

## What the current program does

1. Creates deterministic simulated rice-field conditions with protected sites, synthetic soil variations and configurable traction hazards. Expanded runs create physically larger fields and additional planting tasks.
2. Simulates 2–6 articulated rover bodies, real MuJoCo actions, wheel slip, local probe/range observations, seed-drop contact and bounded mechanical docking/extraction.
3. Runs an authoritative **EpisodeSession** observation–policy–action–feedback loop. The selected coordination policy genuinely produces commands; it is not silently replaced by a diagnostic shadow policy.
4. Offers a **cooperative/shared-map controller** and a **Mycelial/decentralized controller** with per-rover local beliefs, delayed modelled peer gossip, task bidding and finite communication modes.
5. Classifies mobility incidents from available command and sensor evidence, evaluates a helper's task value and risk, and uses guarded rescue/recovery actions. The golden cooperative case includes rigid docking and measurable extraction; a high-risk case refuses assistance.
6. Exposes actual simulation and local beliefs through a 3D/2D engineering dashboard and read-only judge-facing Presentation Mode.
7. Runs matched scientific baseline and ablation experiments using an independent evaluator that can access hidden synthetic truth for scoring, not to direct the controller.

## What makes the agentic decision useful?

The immediate alternatives are understandable and strong: **never rescue** (maximise helper safety), **always rescue** (risk a second rover), and **calibrated numerical triage** (rescue when calculated utility exceeds risk). Antenna implements these baselines alongside an incident-based orchestrator with a tool-oriented reasoning interface and deterministic safety gate.

The design separates **interpretation and high-level strategy** from **bounded physical control**. Model output, when explicitly enabled, cannot directly set motor torques or override hard safety limits. Recorded fleet experiments used the **deterministic offline fallback** and made **zero LLM requests**; therefore the current evidence does not prove that an LLM, or a larger model, outperforms the numerical baseline.

Mycelial coordination has a more direct measured benefit in the available small synthetic study: six matched normal-vs-isolated communication pairs showed 5 improved planting results, 1 tie, and lower inter-rover map divergence in all 6 pairs. This is useful provisional evidence, not a statistically reliable farm performance claim.

## Definition of a verified episode

A meaningful demonstration observes a rover doing fieldwork, a real simulated obstruction/traction failure, the controller's incident response, a physical recovery **or** safe refusal, and subsequent task/mobility outcomes. Count:

- **Observation:** issued commands, range/soil/pose/wheel observations, measured stalled motion and known uncertainty.
- **Decision:** actual active policy and bounded rescue verdict or task assignment.
- **Action:** physics-step motor/docking/seed commands and corresponding MuJoCo contact/positions.
- **Result:** recorded latch, physical extraction displacement, helper loss, resumed rover mobility, viable seed contacts, crop-safe behaviour and unfinished tasks.
- **Comparison:** identical scenario, seed, fleet size and field size under a competent baseline.

The synthetic ground-truth evaluator may score decisions; it cannot be passed as oracle sensor information to the active controller.

## Scope and limitations

- **Physical scope:** simulated MuJoCo rover bodies and programmed terrain/contact models. The heightfield's visual relief, terrain contact modelling and sinkage/traction approximations are **not** independently calibrated to actual rice-field conditions.
- **Failure scope:** randomized episodes are reproducible combinations of generated positions, sites and configured hazardous encounters. Do not claim every immobilisation emerges without preconfigured hazard mechanics.
- **Fleet scope:** expanded fleets have identical sampling/planting capabilities and default inventory; the **legacy two-rover fixtures** retain different inventories to reproduce a controlled rescue paradox. Dynamic task assignment is separate from fixed hardware roles.
- **Evidence limits:** collisions and a secondary casualty occurred in the 18-run fleet study; full completion is not universal. Report these failures rather than selecting only positive seeds.
- **Model and hardware limits:** no successful live LLM inference in the recorded release experiments, no physical RF network, no Jetson deployment, no validated real crop yield, no general safety certification and no demonstrated Isaac Sim implementation.

See [architecture](ARCHITECTURE.md), [testing and measurements](TESTING.md), [demo](SUBMISSION.md) and [provenance](PROVENANCE.md).

## Near-term validation beyond the hackathon

A credible next step would measure more seeds and terrain severities, compare the risk-aware orchestrator against calibrated triage with actual interventions, quantify safety and latency distributions, and test on realistic contact/soil data. Later physical-robot tests would need real soil mechanics calibration, instrumented force and traction sensors, radio/network reliability and human safety procedures. Those are **future research**, not capabilities represented as delivered.
