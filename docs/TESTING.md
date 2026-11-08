# Testing, measured results and limitations

**Antenna release candidate · 8 November 2026**

This document distinguishes executable checks, local GUI observations and limited simulated experimental measurements. It deliberately avoids converting historical test results into real-world robot performance claims.

## Verification ledger

| Source | Revision/environment | What it proves |
| --- | --- | --- |
| [Independent Windows release check](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/tree/fix/antenna-final-codex-verification) | **32ce0a3**, Windows, Python 3.13.14, MuJoCo 4.0.0 | **142 tests passed, no skips**; recorded local GUI opening and exercise of six-rover playback, planting, pausing, scenario and control changes |
| [Presentation Mode GitHub Actions](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/actions/runs/42478808632) | **60c7283**, Ubuntu CI Python 3.12 with MuJoCo 4.0.0 | **149 tests run successfully; 2 GUI/display-dependent skips**, static repository and skill checks |
| [18-run experimental report](../experiments/release_verification_20261108.json) | Windows verification branch, seeded 240-tick episodes | Continuous MuJoCo per-rover movement, task decisions, seed contacts, rescue/refusal, collisions, map divergence and simulated gossip traffic |
| Submission branch | **release/agentverse-2026-submission** | Documents and source are curated from a pinned green code revision; rerun CI and GUI smoke checks for its exact final SHA |

**No official submission receipt, final one-page PDF, continuous video or production robot is verified here.**

## Physical rescue acceptance

The golden case uses the controlled cooperative controller (not an all-mode promise):

~~~powershell
python scripts/run_mujoco_paddypatch.py --scenario golden_rescue --policy cooperative --seed 7 --steps 220 --headless
~~~

The independent Windows verifier recorded:

- A bounded rescue decision/command within the same MuJoCo episode as physical seed work.
- A rigid latch and physically measured victim displacement at least **0.18 m**.
- Resumption of independent rover motion and **two viable subsequent planting contacts**.

The high-risk fixture:

~~~powershell
python scripts/run_mujoco_paddypatch.py --scenario high_risk_abstain --policy cooperative --seed 7 --steps 220 --headless
~~~

verifies safe non-dispatch and two viable planting contacts for this scenario. The deliberately naive **always rescue** policy can strand another robot; this is a meaningful negative control. A component's typed technique or scenario configuration does not itself prove successful physical extraction.

## Expanded fleet and gossip experiment

**Reproduction command** (the released machine-readable report records these exact settings):

~~~powershell
python scripts/run_scaled_field_study.py --seeds 3 8 --fleets 2 4 6 --scales 1.6 --policies mycelial cooperative --comms normal isolated --rescue-policies react_orchestrator --steps 240 --out artifacts/release-study.json
~~~

Trial design: two seeds × three fleet sizes × three controller/communication treatments = **18 continuously simulated 24-second runs**, including **six normal-versus-isolated Mycelial pairs** at matching seed, fleet, scale and rescue policy. The evaluator preserves the real acting policy and effective fleet settings; hidden soil truth is used for scoring.

### Paired Mycelial results

| Matched configuration | More viable seeds with normal gossip | More sites visited | Map divergence versus isolation |
| --- | ---: | ---: | --- |
| Seed 3, 2 rovers | +1 | +3 | Lower |
| Seed 3, 4 rovers | +2 | +4 | Lower |
| Seed 3, 6 rovers | +1 | +2 | Lower |
| Seed 8, 2 rovers | 0 | +1 | Lower |
| Seed 8, 4 rovers | +1 | 0 | Lower |
| Seed 8, 6 rovers | +1 | +3 | Lower |

**Interpretation:** modelled peer gossip improved planting in five of six pairs, tied one, and decreased belief-map disagreement in all six. The study cannot establish statistical significance or scale to an uncontrolled field. Simulated gossip packet bytes are **not hardware RF bandwidth measurements**. A coordination advantage is not necessarily a rescue advantage; different controllers made different rescue choices.

### Unfavourable outcomes

- **Two four-rover Mycelial trials** (one normal and one isolated) recorded rover collisions.
- **One six-rover normal Mycelial run** recorded a second casualty during rescue.
- Several runs failed to finish all viable planting work and/or failed to extract a stranded rover.
- The built-in binary success/failure classifier carries historical two-rover assumptions and **must not** be reported as a general six-rover mission success rate.
- Total planted seeds increase with fleet inventory and work availability; do not interpret higher absolute yield as efficiency without normalising by time, rover count and task density.

The full report records individual observations, including actual physical extraction and continued work. No favourable outcome was substituted for a failed seed.

## LLM and orchestrator claims

The code supports a tool-oriented incident orchestrator, a deterministic safety gate, a **deterministic offline fallback** and an optional Gemini REST backend. The 18 fleet-evaluation runs recorded **zero LLM calls**. Therefore:

- No live-model reasoning, inference latency, token expenditure or model-size advantage has been demonstrated by these trials.
- Any comparison of larger versus smaller models must use matched incident inputs, real provider responses and response/fallback/latency counts, with an explicitly authorised API call budget.
- The deterministic calibrated-triage, no-rescue and always-rescue baselines are important competent comparisons; equality of outcomes is **not** evidence that the agent outperformed them.
- LLM outputs never directly write unbounded torques; final dispatch and actuation obey a deterministic safety gate.

The opt-in comparison runner is **scripts/compare_orchestrator_models.py**; do not execute paid calls without approval.

## What automated tests do and do not establish

The test suite includes Python contract checks, seeded world reproducibility, physical MuJoCo integration, rescue controls, no-oracle fault negative controls, active decentralized policy selection, field enlargement, per-rover activity, a 120-tick six-rover episode, and matched gossip evaluation. [The latest presentation CI](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/actions/runs/42478808632) completed successfully for the pinned base.

**Automated checks do not establish:** a new Windows commit's rendered GUI, the readability of a 60-second recording, general mud/geology validity, real-field autonomy, physical safety certification, hardware RF throughput, an LLM call when none occurred or a submitted hackathon entry.

## Run the entire suite

~~~powershell
python -m pip install -r requirements-mujoco.txt
python scripts/check_repo.py
python scripts/sync_skills.py --check
python -m unittest discover -s tests -v
~~~

Headless CI skips Tkinter tests without a display; Windows GUI integration must be run on the demonstration laptop. Record exact SHA, Python/MuJoCo versions, test summary, scenario/seed, measured scene outcomes and limitations.

## Physical and scientific limitations

- This is **MuJoCo**, not Isaac Sim, and not deployed hardware. Soil metrics, crop suitability and hazard distributions are synthetic.
- Many randomized failures are configured hazardous-site entries with simulated wheel/friction effects, not naturally emergent field epidemiology. The visible heightfield is not equivalent to a validated deformable-ground contact model.
- Real-world rice yield, soil compaction accuracy, farm labour savings, field economics, rescue incidence and safety have not been measured.
- Peer gossip is modelled inside one application process; no real decentralized radio stack has been deployed.
- Controlled golden rescue and refusal tests are intentionally reproducible benchmarks, not statistical proofs that recovery succeeds everywhere.
- The expected judge clip and PDF are separate deliverables; see [SUBMISSION](SUBMISSION.md).

**Principle:** report the actual observed action, physical effect, matched baseline and failure—then explain why it matters. Preserve hard cases.
