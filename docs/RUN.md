# Reproducible setup, run and recovery

**Application runtime not yet chosen.** Do not falsely claim installation, simulator/robot access, models, cloud credits, supported operating systems or a live demo.

## Foundation checks — actually available

```bash
python scripts/check_repo.py
python scripts/sync_skills.py --check
python -m unittest discover -s tests -v
python scripts/evaluate.py --candidate tests/fixtures/candidate.jsonl --baseline tests/fixtures/baseline.jsonl
```

Run `python scripts/sync_skills.py --write` after editing any canonical `.agents/skills/` file. The check command fails if the complete Claude mirrors diverge or skills are missing/extra. Python 3.10+ standard library only. These checks validate **repo structure and evaluation tooling**, not any robotics entry.

## Fill immediately after selecting the product stack

| Action | Exact command / requirement |
| --- | --- |
| Supported OS/platform | TBD |
| Minimum CPU/GPU and optional hardware | TBD; do not assume NVIDIA or a robot |
| Clean install / pinned dependency versions | TBD |
| Offline or no-secrets mode | TBD |
| Secrets configuration **names only** | TBD; never record values |
| Start the actual system | TBD |
| One deterministic smoke-demo with seed | TBD |
| Unit and integration tests | TBD |
| Independent baseline run and evaluator export | TBD |
| Reset simulator/robot and recover from errors | TBD |
| Troubleshooting/log location | TBD |

A team member who did **not** develop the project should be able to reproduce the minimum demo from this file. Record commands actually validated on a clean environment and link the tested commit. For physical hardware, add an operator stop/abort procedure and a simulator-only or replay fallback.
