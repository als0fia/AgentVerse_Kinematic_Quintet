# Simulation Seed Invariance & Determinism Audit — Agent Repair Handoff

**Audit performed:** 8 November 2026, ~03:00 GMT.  
**Source baseline:** `main` at commit `494d9af` (post-PR #63).  
**Scope:** Root cause analysis for simulation seed invariance across (1) the 2D farming cycle simulation (`src/riceswarm_ai/`) and (2) the MuJoCo terraced paddy simulation and evaluation harness (`src/ricebot/`, `scripts/`).  
**Context:** When running multi-seed simulations or evaluating variance across seeds, agents/evaluators observed that animations, soil conditions, and robot trajectories remain identical or nearly identical across different seed values.

---

## 1. Executive Summary & Root Causes

The repository contains two simulation layers, each exhibiting distinct determinism and seed-discarding bugs:

1. **2D Farming Lifecycle (`src/riceswarm_ai/`):** Complete absence of stochasticity. `import random` is present at line 24 of `core.py` but is **never called anywhere in the file**. The field maps (fertility, water depth, weed density) are computed via deterministic modular arithmetic on cell coordinates `(x, y)`, and neither `Field` nor `RiceSwarmSimulation` accepts a `seed` argument.
2. **MuJoCo Terraced Paddy Simulation (`src/ricebot/`):**
   - **Seed discarded in soil generation:** `generate_hidden_soil_truth(scenario, seed)` in `src/ricebot/world/terrace.py` takes a `seed` parameter, but immediately returns the static constant dictionary `DEFAULT_HIDDEN_SOIL_TRUTH` for all scenarios except `"shifted_moisture"`.
   - **Truth frozen across resets:** `PaddyTerraceWorld.reset(seed)` resets continuous terrain micro-elevation but preserves the `self._hidden_site_truth` dictionary initialized at simulator construction time, preventing evaluation loops from testing varying ground truths across episodes.
   - **Dashboard export bypass:** `scripts/run_mujoco_paddypatch.py` hardcodes `DEFAULT_HIDDEN_SOIL_TRUTH` / `UNEXPECTED_BAD_SOIL_TRUTH` during dashboard exports rather than calling `generate_hidden_soil_truth(scen, seed)`.

---

## 2. Detailed Findings & Exact Evidence

### Finding A: 2D Farming Cycle Simulation (`src/riceswarm_ai`)

#### A.1 Unused `random` Import & Hardcoded Field Formulas
- **Location:** `src/riceswarm_ai/core.py#L24` and `src/riceswarm_ai/core.py#L185-L188`
- **Evidence:**
  ```python
  # Line 24:
  import random  # Imported but NEVER used in core.py

  # Lines 185-188:
  fertility = 0.50 + 0.03 * ((x * 4 + y * 3) % 9)
  water = 2.0 + 0.4 * ((x * 2 + y * 5) % 8)
  weed = 0.10 + 0.04 * ((x * 7 + y) % 7)
  ```
- **Consequence:** 
  The grid cells are generated via static modular arithmetic formulas. Weather updates, crop growth increments, and nursery dispatch dates are hardcoded deterministic state machines. There is no `seed` parameter in `Field.__init__` or `RiceSwarmSimulation.__init__`. Every invocation generates the exact same visual frame sequence.

---

### Finding B: MuJoCo Terraced Paddy World (`src/ricebot`)

#### B.1 `generate_hidden_soil_truth` Discards Seed for Non-`shifted_moisture` Scenarios
- **Location:** `src/ricebot/world/terrace.py#L98-L105`
- **Evidence:**
  ```python
  def generate_hidden_soil_truth(scenario: str, seed: int) -> dict[str, dict[str, float]]:
      if scenario == "bad_soil":
          return {k: dict(v) for k, v in UNEXPECTED_BAD_SOIL_TRUTH.items()}
      if scenario != "shifted_moisture":
          return {k: dict(v) for k, v in DEFAULT_HIDDEN_SOIL_TRUTH.items()}
      rng = random.Random(seed)
      ...
  ```
- **Consequence:**
  When passing `--seed 42`, `--seed 100`, or iterating seeds in `evaluate.py`, any scenario other than `"shifted_moisture"` (such as `"nominal"`, `"golden_rescue"`, `"obstacle"`, `"sensor_degraded"`, `"bad_soil"`) completely ignores `seed` and returns the identical constant table `DEFAULT_HIDDEN_SOIL_TRUTH`. Evaluator perception probes across seeds receive invariant soil ground truth.

#### B.2 `PaddyTerraceWorld.reset()` Never Re-samples Soil Truth
- **Location:** `src/ricebot/world/terrace.py#L449-L456`
- **Evidence:**
  ```python
  def reset(self, seed: int | None = None) -> None:
      if seed is not None:
          self.continuous_terrain = ContinuousPaddyTerrain(seed=seed)
      self.soil_samples.clear()
      self.paddy_cells.clear()
      self.canopy_points.clear()
  ```
- **Consequence:**
  `self._hidden_site_truth` is initialized in `__init__` and never refreshed upon `world.reset(seed)`. In multi-episode benchmarking sweeps (e.g. `evaluate.py`), subsequent episodes reuse the soil truth established in episode 0.

#### B.3 Dashboard Truth Hardcoding
- **Location:** `scripts/run_mujoco_paddypatch.py#L528`
- **Evidence:**
  ```python
  truth = UNEXPECTED_BAD_SOIL_TRUTH if scen == "bad_soil" else DEFAULT_HIDDEN_SOIL_TRUTH
  ```
- **Consequence:**
  Multi-scenario dashboard export does not pass `seed` to `generate_hidden_soil_truth(scen, seed)`, causing the dashboard summary to show static constant truth values.

---

## 3. Recommended Minimal Fixes for Implementing Agents

1. **For `src/riceswarm_ai/core.py`:**
   - Add optional `seed: int | None = None` to `RiceSwarmSimulation` and `Field`.
   - Initialize a local `self.rng = random.Random(seed)` in `Field`.
   - Apply bounded stochastic perturbations around base field parameters (fertility, water depth, weed density) using `self.rng.uniform(...)` or `self.rng.gauss(...)` while preserving valid invariant bounds (e.g., moisture in `[0.0, 1.0]`, water depth `>= 0.0`).

2. **For `src/ricebot/world/terrace.py`:**
   - In `generate_hidden_soil_truth(scenario: str, seed: int)`:
     - When `seed is not None`, apply bounded random variations (e.g., `±0.05` moisture, `±0.04` bearing capacity) for all scenarios around their respective base truth profiles (`DEFAULT_HIDDEN_SOIL_TRUTH` or `UNEXPECTED_BAD_SOIL_TRUTH`), rather than short-circuiting on `if scenario != "shifted_moisture":`.
   - In `PaddyTerraceWorld.reset(self, seed: int | None = None)`:
     - If `seed is not None`, re-sample `self._hidden_site_truth = generate_hidden_soil_truth(self.scenario, seed)`.

3. **For `scripts/run_mujoco_paddypatch.py`:**
   - Replace line 528 with:
     ```python
     truth = generate_hidden_soil_truth(scen, seed)
     ```
