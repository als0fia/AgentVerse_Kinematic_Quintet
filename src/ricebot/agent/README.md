# Swarm Intelligence & Edge Coordination (`src/ricebot/agent/`)

**Scope:** Swarm Intelligence / Task Coordination  
**Branch:** `feature/agentic-swarm-intelligence`  
**Status:** Decoupled from Mapping (`field_map`) and Traversal (`controller`).

---

## 1. Core Architecture

### Homogeneous Swarm (Identical Capabilities)
All rovers are created with identical physical capabilities via `homogeneous_capabilities(["rover-1", "rover-2", ...])`:
- Local soil/stand sensors (`can_sample=True`)
- Seed hopper (`can_plant=True`)

Nobody is hardcoded as "Scout" or "Planter." Roles emerge dynamically on each decision tick based on robot poses, remaining seed counts (`seeds_remaining`), and observed field knowledge.

### Clean Boundary Seams
- **Mapping -> Swarm Agent:** We only call `field_map.estimate(site_id)` to read `SiteEstimate` (`moisture`, `firmness`, `stand_gap`, `complete`).
- **Swarm Agent -> Traversal:** We output `tuple[TaskIntent, ...]` (`mode = SAMPLE | PLANT | HOLD`, `target_site_id`, `reason`). Traversal owns waypoint driving and obstacle avoidance (`RobotCommand`).
- **Optional Traversal Hook:** `travel_cost_fn(robot_pose, site) -> distance_m | None` allows using actual graph/lane distances for bidding instead of Euclidean straight-line distance.

---

## 2. 2-Tier Hybrid Edge AI Architecture

```
+-------------------------------------------------------------------------+
| TIER 2: SHARED FIELD-EDGE HUB (1 per farm / solar box or tablet)       |
| - GeminiSwarmAdvisor: System 2 deliberative AI brain                    |
| - Triggered ONLY on discrete surprise events (route obstruction,        |
|   quality overturned, range sensor fault)                               |
| - Immediate safe fallback when offline (link_ok=False)                  |
+------------------------------------+------------------------------------+
                                     |
                   Low-bandwidth JSON intent / mesh radio
                                     |
                                     v
+-------------------------------------------------------------------------+
| TIER 1: ON-ROVER MICRO-EDGE (Runs onboard every homogeneous rover)      |
| 1. SwarmRobotAgent: Local Explore (SAMPLE) vs Exploit (PLANT) utility   |
|    bids evaluated onboard in <1 ms.                                     |
| 2. Peer-to-Peer Auction: Decentralized auction resolves task bids       |
|    without duplicate sites; dynamic role splitting.                     |
| 3. Seed-Depletion Transition: Rovers with seeds=0 auto-transition       |
|    into dedicated scouts boosting VoI for peers with seeds.             |
| 4. Hard Safety Verifier: Strictly rejects out-of-envelope or            |
|    unconfirmed planting actions.                                        |
+-------------------------------------------------------------------------+
```

---

## 3. Usage & Integration

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
    advisor=GeminiSwarmAdvisor(),  # optional AI supervisor; safe offline fallback
    travel_cost_fn=None,           # optional lane distance hook
)
core = MissionCore(
    field_map=MeasuredFieldMap(sites),
    planner=planner,
    controller=controller,
)
```
