# Agentic Swarm & Edge Architecture Plan (`src/ricebot/agent/`)

**Scope owner:** Swarm Intelligence / Task & Rescue Coordination (`src/ricebot/agent/`)  
**Branch:** `feature/agentic-swarm-intelligence`  
**Status:** Local draft for teammate review — decoupled from Mapping (`field_map`) and Traversal (`controller`).

---

## 1. Core Design Decisions for the Team

1. **Homogeneous Swarm (All Robots Are Identical):**
   - Every rover in the swarm has the exact same hardware capabilities: both local soil/stand sensors (`can_sample=True`) and a seed hopper (`can_plant=True`), created via `homogeneous_capabilities(["rover-1", "rover-2", ...])`.
   - Nobody is hardcoded as "Scout" or "Planter." Roles emerge dynamically on every decision cycle based on each robot's position, remaining seed count (`seeds_remaining`), and the shared map.

2. **Clean Separation from Mapping & Traversal Teammates:**
   - **Mapping Teammate -> Swarm Agent:** We only call `field_map.estimate(site_id)` to read `SiteEstimate` (`moisture`, `firmness`, `stand_gap`, `complete`, plus optional `.uncertainty` / `.confidence`).
   - **Swarm Agent -> Traversal Teammate:** We output `tuple[TaskIntent, ...]` (`mode = SAMPLE | PLANT | HOLD`, `target_site_id`, `reason`). Traversal owns how the rover drives there and avoids obstacles (`RobotCommand`).
   - **Optional Traversal Hook:** Traversal can pass `travel_cost_fn(robot_pose, site) -> distance_m | None` into `CooperativeSwarmPlanner` so task bids use real lane path distances instead of straight-line Euclidean distance.

---

## 2. How We Handle Edge Compute, AI, and Commercial Viability

Putting a $1,000 GPU on every small rice-field rover is too expensive and drains battery, while relying on a remote cloud LLM for every wheel tick fails rural connectivity and loses the **15% Operational Autonomy** judging criterion.

We solve this with a **2-Tier Hybrid Edge AI Architecture (System 1 + System 2)**:

```text
+-------------------------------------------------------------------------+
| TIER 2: SHARED FIELD-EDGE HUB (1 per farm / solar box or farmer tablet) |
| - Hardware: Single ~$150 edge AI box (Jetson Orin Nano / local Gemma 4) |
|   or Gemini Flash API when connectivity permits ($0 extra per rover).   |
| - Role (System 2 — Deliberative AI Brain):                              |
|   Triggered ONLY on discrete events (stranded robot rescue triage,      |
|   soil quality overturns, blocked routes, or task transitions).         |
|   Reasons over swarm JSON state to reallocate tasks or approve rescue.  |
+------------------------------------+------------------------------------+
                                     |
                   Low-bandwidth JSON intent / radio link
                   (If link_ok=False, Tier 1 runs 100% standalone)
                                     |
                                     v
+-------------------------------------------------------------------------+
| TIER 1: ON-ROVER MICRO-EDGE (Runs onboard every homogeneous rover)      |
| - Hardware: ~$20 sub-1W microcontroller / Pi Zero + local mesh radio.   |
| - Role (System 1 — Fast Autonomous Utility Agent & Safety Gate):        |
|   1. SwarmRobotAgent: Evaluates Explore (SAMPLE) vs. Exploit (PLANT)    |
|      utility bids locally in <1 ms.                                     |
|   2. Peer-to-Peer Auction: Resolves task bids without duplicate sites;  |
|      auto-switches seed-depleted rovers (seeds=0) into full-time scouts.|
|   3. Hard Safety Verifier: Rejects any unsafe AI hallucination (e.g.,   |
|      planting on an unverified site) and provides instant fallback      |
|      when offline (link_ok=False).                                      |
+-------------------------------------------------------------------------+
```

---

## 3. What Makes the Swarm Agentic (4 Mechanisms)

### A. Autonomous Explore (`SAMPLE`) vs. Exploit (`PLANT`) Bidding (`SwarmRobotAgent`)
Every rover runs its own `SwarmRobotAgent` and computes utility bids (`AgentBid`) for every candidate site:
- **Exploit (`PLANT`):** High utility if the site is confirmed eligible (`complete=True` and within soil moisture/firmness/gap thresholds) and this rover has `seeds_remaining > 0`, minus travel cost.
- **Explore (`SAMPLE`):** Value-of-Information (VoI) utility for inspecting unconfirmed sites—including a **cooperative proximity bonus** for scouting sites near teammates that still have seeds.

### B. Emergent Role Switching via Decentralized Auction (`CooperativeSwarmPlanner`)
- **Dynamic Role Split:** If only `site-a` is confirmed eligible, the closer rover wins the `PLANT` bid for `site-a`, while the second identical rover automatically pivots to its next-best bid (`SAMPLE` at `site-b`) instead of colliding or idling.
- **Seed-Depletion Transition:** As soon as a rover drops its last seed (`seeds_remaining == 0`), it stops bidding on `PLANT` and boosts its `SAMPLE` utility, automatically becoming a dedicated **scout** for peers that still carry seeds.

### C. "Is the Robot Worth Saving?" Rescue Triage (`evaluate_rescue_worth`)
When a rover gets stuck (`moved < 0.02m` while blocked for $N$ ticks) or suffers a range-sensor fault (`front_range_m is None`), the swarm evaluates whether diverting a healthy peer is worth it:

$$\text{NetRescueValue} = \text{RescueBenefit} \text{ (stranded seeds + offline data)} - \text{OpportunityCost} \text{ (active planting + travel)} - \text{HazardRisk} \text{ (near no-go zone)}$$

This produces one of three autonomous verdicts (`RescueVerdict`):
1. **`RESCUE_NOW`:** Stranded rover carries needed seeds (`seeds_remaining > 0`) in a safe zone while healthy peers lack seeds -> healthy peer diverts to the nearest safe site to assist (`rescue_assist`).
2. **`DEFER_RESCUE`:** Stranded rover is worth saving, **but** the healthy peer has its own seeds and a confirmed crop gap ready right now -> **plant the confirmed crop gap first**, then rescue once planting is finished.
3. **`ABANDON_ESCALATE`:** Stranded rover has `0` seeds left or is trapped inside a hazardous `permitted=False` zone where the rescuer could also get lost -> stranded rover stays in safe `HOLD` (beacon for human pickup after the mission) while healthy peers finish the field.

### D. Event-Driven AI Supervisor with Safety Guardrails (`GeminiSwarmAdvisor`)
- When a mid-episode surprise occurs (`quality_overturned`, `route_obstructed`, or `robot_stranded`), `GeminiSwarmAdvisor` prompts the AI model (Gemini REST API or local edge LLM) with the live JSON state of all robots, seed counts, and site estimates.
- Every AI suggestion is checked by `_try_advisor`: it **rejects** planting on unmeasured sites, duplicate site assignments, or planting with `0` seeds, and immediately falls back to the Tier-1 auction if `link_ok=False`.

---

## 4. How Teammates Plug This In

No changes to [contracts.py](../src/ricebot/contracts.py) or [runtime.py](../src/ricebot/runtime.py) are required:

```python
from ricebot.agent import (
    CooperativeSwarmPlanner,
    GeminiSwarmAdvisor,
    homogeneous_capabilities,
)
from ricebot.mission import MeasuredFieldMap
from ricebot.runtime import EpisodeRunner, MissionCore

caps = homogeneous_capabilities(["rover-1", "rover-2"])

planner = CooperativeSwarmPlanner(
    capabilities=caps,
    advisor=GeminiSwarmAdvisor(),  # optional AI supervisor; safe offline fallback if no API key
    travel_cost_fn=None,           # Traversal teammate can pass their path-distance function here
)

core = MissionCore(
    field_map=MeasuredFieldMap(sites),  # Mapping teammate's map class plugs in here
    planner=planner,
    controller=controller,              # Traversal teammate's controller plugs in here
)
```
