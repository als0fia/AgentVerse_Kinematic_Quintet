# Antenna experiment evidence

## Verified release study

[release_verification_20261108.json](release_verification_20261108.json) is a **recorded Windows MuJoCo experiment** from the separately verified release branch (commit **32ce0a3**), using Python 3.13.14 and MuJoCo 4.0.0.

It contains **18 continuous randomized episodes** (seeds 3 and 8; fleet sizes 2, 4 and 6; physical map scale 1.6×; 240 physics/controller ticks each) across Mycelial normal gossip, Mycelial isolated communications and shared-map cooperative policy.

Six matched Mycelial normal-versus-isolated comparisons are included. Normal gossip produced **five positive and one tied** viable-placement differences; belief-map divergence was lower in all six pairs. The same record contains unsuccessful outcomes: collisions in two four-rover trials, one six-rover secondary casualty, unfinished planting and failed extractions. It reports **zero live LLM requests**. Do not turn synthetic packet byte counts into measured RF bandwidth or treat two seeds as statistical validation.

**Reproduction:**

~~~powershell
python scripts/run_scaled_field_study.py --seeds 3 8 --fleets 2 4 6 --scales 1.6 --policies mycelial cooperative --comms normal isolated --rescue-policies react_orchestrator --steps 240 --out artifacts/release-study.json
~~~

Generated full reports belong under gitignored **artifacts/** unless selected for careful, team-approved evidence retention. Report the tested commit, actual Python/MuJoCo environment, seed and effective controller with each numerical claim.

## Historical data

[season_report.json](season_report.json) is from the earlier **src/riceswarm_ai/** agricultural-season experiment. It is **not** independent evidence of Antenna's real-world planting, crop yield or automated physical rescue. It remains for prototype history and regression context.

## Evidence rules

Use [docs/TESTING](../docs/TESTING.md) for interpretation, [docs/PROVENANCE](../docs/PROVENANCE.md) for AI and dependency attribution, and [docs/SUBMISSION](../docs/SUBMISSION.md) for the judge deliverables. Never claim a model response, hardware event, video capture or submission receipt that was not actually observed.
