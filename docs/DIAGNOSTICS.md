# Independent episode diagnostics (Antenna)

This is **developer-only instrumentation**, not an extra controller or a substitute for a physically working rescue. It is intentionally outside rice.v1 mission contracts, existing sim physics and the main demo runner. Owners: the evaluation/debugging worker and the human integrator.

## What it observes

- **Physical robot-to-robot contact:** MuJoCo solver contact pairs, contacting geoms, penetration depth and sampled normal force. Contact commanded as part of a rescue is labelled separately; ordinary bumper collisions are errors. A centre-distance metric supplements contacts but is **not** a collision detector by itself.
- **Failed movement / candidate immobilisation:** compare actual world-coordinate travelled distance with the *commands actually issued* over a rolling time window. Drive requests without progress produce 'commanded_no_progress'; absent subsequent rescue intent produces 'stall_without_rescue_response'. Neither finding alone proves a controller bug: safe stops, obstructions, planned abstention and towing must be inspected.
- **Recovery evidence:** first recovery intent, continued movement, 'mobility_resumed', and original simulator 'RecoveryFeedback' are recorded on the same timeline. A recovery success flag is not a substitute for independent motion.
- **Soil dynamics:** time-aligned physics Z, modelled sinkage, belly clearance, and slip ratio, so an agent can check whether increasing mud hazard actually reduces traction/mobility. These are *simulator models*, not validated real-world mud physics.
- **Failure coverage:** the output explicitly says when a no-trap episode does **not** demonstrate an immobilisation. A scripted 'golden_rescue' trap is declared in metadata; it is not passed off as naturally emergent physics.

This instrumentation does **not** attempt to repair existing stuckness detection, 'is_bogged', privilege boundaries, rover collision avoidance, or mud mechanics. Those are owned elsewhere. It provides falsifiable evidence to improve them later.

## Run it

Install project prerequisites per [RUN](RUN.md). Headless, for example:

~~~bash
python scripts/diagnose_mujoco.py --scenario golden_rescue --seed 7 --steps 195
python scripts/diagnose_mujoco.py --scenario golden_rescue --no-scripted-trap --seed 7 --seed 11 --steps 195
python scripts/diagnose_mujoco.py --scenario obstacle --seed 7 --seed 8
python scripts/diagnose_mujoco.py --scenario nominal --policy mycelial --seed 7
python -m unittest discover -s tests -p test_diagnostics.py -v
~~~

By default, all output is in Git-ignored 'artifacts/diagnostics/'. Set '--out-dir' for another location. Each run writes a complete '.jsonl' time series and '-summary.json'; the CLI prints a small multi-seed JSON index. A 0 exit status means the audit tool executed, **not** that no faults were found. For automatic checks, use '--fail-on error' or '--fail-on warning' explicitly; findings are still available when the runner exits 1.

## Machine-readable trace contract

Version: 'antenna.diagnostics.v1', a developer-evaluation format, not an existing 'rice.v1' wire schema. One JSON object per line:

1. 'kind=run': seed, scene, policy, scripted-trap configuration, stall thresholds.
2. 'kind=finding': fault code, severity, sim time, actor IDs, measured evidence, investigation hint.
3. 'kind=tick': complete current observation, intents, issued commands, independent physics snapshot (world body positions, contacts, sinkage/slip), per-robot step travel.
4. 'kind=summary': whole-episode aggregates, all findings, measurements and limitations.

Inspection example:

~~~bash
python -c "import json,sys; [print(json.dumps(r,indent=2)) for r in map(json.loads,open(sys.argv[1])) if r['kind']=='finding']" artifacts/diagnostics/golden_rescue-cooperative-seed7-trap.jsonl
~~~

**Failure codes:**

| Code | What it *actually* establishes |
| --- | --- |
| 'unexpected_robot_collision' | Sampled solver contact exists between two robots, with no matching rescue command at that tick. |
| 'robot_contact_during_rescue' | Robot contact occurred while rescue docking/towing was commanded; review force even if intentional. |
| 'strong_environment_contact' | A non-ground obstacle contact exceeded the chosen 100 N review threshold. |
| 'commanded_no_progress' | Driven travel requested at least 0.12 m over ~1.2 s; measured XY path at most 0.035 m. |
| 'stall_without_rescue_response' | Candidate stall persisted 1 s after initial detection with no rescue intent targeting that robot. It may be a justified abstention. |
| 'mobility_resumed' | Motion returned after an observed stall. |
| 'large_pose_discrepancy' | Independent truth-vs-observation discrepancy greater than 0.12 m at a sampled tick. |
| 'scripted_trap_without_independent_stall' | A synthetic trap was configured but movement evidence did not establish immobilisation. |
| 'no_natural_immobilization_observed' | No measured stall was seen without a configured scripted trap; not proof the model can never get stuck. |
| 'episode_exception' | Runner raised an exception; partial timeline and stack trace are retained. |

## Debugging agent workflow

1. Run a **known-good canonical seed** and record its current Git SHA, scenario and output. Repeat on at least two other seeds before generalising.
2. Run the **same configuration without the scripted trap** to distinguish deliberately injected immobilisation from naturally emergent traction, sinkage and contact effects.
3. Extract each finding; inspect the 1–2 seconds of adjacent tick records and original issuing commands, not just the summary flag. Classify it as physical/simulator failure, planner response failure, expected rescue contact, expected safe abstention, or false-positive diagnostic.
4. Reproduce with the exact seed and steps. Change the smallest relevant **owning module** in a separate approved branch; do not automatically change shared contracts or robot geometry from this audit. Add a regression test that demonstrates the defect before fixing it.
5. Re-run the diagnostic episode and the canonical Golden Rescue integration test. Report the before/after measurement and any new regressions to the integrator.

### Known limits

End-of-tick MuJoCo contact snapshots miss **transient substep collisions**. They cannot certify an episode collision-free. Ground contacts are filtered out to avoid ordinary wheel support noise; the tool is not a continuous collision impulse profiler. Sliding/no-progress thresholds are heuristic and need baseline calibration. Simulation-derived sinkage values must be reconciled with **observed movement**; they are not evidence of real-world agricultural validity. Output may be large and is intentionally Git-ignored. The model-facing rescue planner receives none of this ground truth.

When the active integrated runner changes, adapt the **separate diagnostic adapter/CLI**, not the production control interface, and rerun all tests. Keep the logger disposable and non-blocking during the live demo.
