# Modal Cloud Simulation & SLM Fine-Tuning Report

**Branch:** `tests/modal`  
**Date:** November 8, 2026  
**Workspace:** Modal (`ananya4231`)  
**Hardware:** Nvidia T4 GPU & 8-core CPU containers  

---

## 1. Executive Summary

This branch consolidates:
1. **Modal Cloud Simulation Execution** (`scripts/modal_sim.py`): MuJoCo 4.0.0 paddy patch simulations and parallel multi-scenario runs running inside Debian-slim Linux containers on Modal.
2. **System-2 Swarm ReAct SLM Fine-Tuning** (`scripts/modal_train_slm.py`): LoRA fine-tuning of `Qwen/Qwen2.5-0.5B-Instruct` on 180 multi-turn robot rescue trajectories with persistent weights stored in Modal Volume `antenna-slm-weights`.
3. **Interactive Visual Dashboard** (`dashboard.html`): Browser-based visualization of terrace geometry, swarm trajectories, soil sinkage heatmaps, and rescue timelines.

---

## 2. Modal Cloud Simulation (`scripts/modal_sim.py`)

### Architecture
- **Base Image:** `modal.Image.debian_slim(python_version="3.11").pip_install("mujoco>=3.1.0", "numpy>=1.24.0")`
- **Execution Modes:**
  - `run_cloud_simulation(scenario="golden_rescue")`: Executes terramechanics mud entrapment, helper dispatch, and seed planting.
  - `run_incline_climb(incline_deg=22.0)`: Verifies terrace wall ascending capabilities (+279.9 mm forward displacement).
  - `sweep_scenarios_parallel(["nominal", "obstacle", "golden_rescue"])`: Distributed map across 3 independent cloud workers in parallel.

### Verification Results
- **Golden Rescue:** Stranded robot extracted +18.02 cm forward motion ($\ge 18$ cm target); planted 2/2 seeds; 100% mission success.
- **Incline Climb:** +279.9 mm climbed on 22° ramp without tipping over.
- **Parallel Sweep:** Completed across 3 distributed Modal containers in 19.8 seconds total.

---

## 3. Swarm Orchestrator SLM Fine-Tuning (`scripts/modal_train_slm.py`)

### Purpose
To transition from hardcoded heuristic rules to a deliberative **System-2 Small Language Model (SLM)** capable of:
1. Calling diagnostic tools (`inspect_incident`, `probe_perimeter`, `predict_recovery_displacement`, `evaluate_farmer_risk`).
2. Protecting seed-carrying rovers from secondary entrapment.
3. Exercising honest refusals when recovery risk exceeds farmer policy thresholds.

### Fine-Tuning Specifications
- **Base Model:** `Qwen/Qwen2.5-0.5B-Instruct` (500M parameters)
- **Method:** PEFT / LoRA ($r=16, \alpha=32$, dropout=0.05) on `q_proj`, `k_proj`, `v_proj`, `o_proj`, `gate_proj`, `up_proj`, `down_proj`.
- **Trainable Parameters:** 8,798,208 (1.75% of total).
- **Compute:** Nvidia T4 GPU on Modal (3 epochs, batch size 2, gradient accumulation 4, cosine LR $2\times 10^{-4}$).
- **Training Time:** 178.9 seconds (~3.0 minutes).
- **Cost:** ~$0.03 (covered by free credits).

### Training Metrics & Convergence
| Metric | Initial (Step 0) | Epoch 1 | Epoch 2 | Final (Epoch 3) |
|---|---|---|---|---|
| **Training Loss** | 2.482 | 0.4405 | 0.0654 | **0.0583** |
| **Validation Loss** | — | 0.2601 | 0.0675 | **0.0601** |
| **Eval Token Accuracy** | 56.85% | 93.96% | 97.32% | **97.68%** |

### Output Artifacts
Saved to persistent Modal Volume `antenna-slm-weights` under path `antenna_qwen_react_adapter/`:
- `adapter_model.safetensors` (35.2 MB)
- `adapter_config.json`
- `tokenizer_config.json`, `tokenizer.json`, `chat_template.jinja`

---

## 4. Run Commands & Reproducibility Guide

Ensure Modal CLI is authenticated (`modal token new` or logged in):

### A. Run Cloud MuJoCo Simulations on Modal
```powershell
$env:PYTHONUTF8=1; python -m modal run scripts/modal_sim.py
```

### B. Run SLM LoRA Fine-Tuning on Modal GPU
```powershell
$env:PYTHONUTF8=1; python -m modal run scripts/modal_train_slm.py
```

### C. Download Weights from Modal Volume to Local Workspace
```powershell
# List files in the volume
python -m modal volume ls antenna-slm-weights antenna_qwen_react_adapter

# Download complete adapter bundle to local weights/ directory
python -m modal volume get -e main antenna-slm-weights antenna_qwen_react_adapter weights/
```

### D. Run Local Headless MuJoCo Simulation & Policy Benchmark
```powershell
# Run single golden rescue
python scripts/run_mujoco_paddypatch.py --scenario golden_rescue --headless

# Compare all 3 policies across scenarios
python scripts/run_mujoco_paddypatch.py --compare-policies --headless
```

### E. Launch Interactive Visual Dashboard
```powershell
Start-Process dashboard.html
```
