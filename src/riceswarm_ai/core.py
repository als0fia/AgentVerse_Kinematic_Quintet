"""Deterministic, observable RiceSwarm environment and multi-agent controller.

Complete Condition-Based Rice Farming Lifecycle Simulation for Chinese flooded rice paddy:
- Single-season transplanted rice crop across 145+ agricultural days (Day -25 to Day 120+).
- Two-Section Paddy Plot with Staggered Batches (Section A at Day -25 / transplant Day 0;
  Section B offset +20 days at Day -5 / transplant Day 20).
- Swarm of 6 Homogeneous Multi-Functional Robots (RS1 through RS6):
  Simultaneously senses, plants, weeds, samples soil/water, harvests, and rescues peers.
- Continuous Environmental Monitoring:
  * Water depth, temperature, pH, trend tracking.
  * 4-Tier mud/terrain risk monitoring (LOW, MODERATE, HIGH, VERY_HIGH).
  * Soil fertility tracking (N, P, K, pH) at Days -7, 30, 60, 90 or post-event.
- Condition-based decision architecture:
  Observe -> Evaluate Conditions -> Generate Task -> Execute -> Verify -> Update Field State.
- Multi-objective soft mud rescue optimisation:
  J = w_t T + w_e E + w_c C + w_s S + w_r R.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import heapq
from math import inf
import random
from typing import Callable, Iterable

Point = tuple[int, int]


class Terrain(str, Enum):
    FIRM = "firm"
    SOFT = "soft"
    BLOCKED = "blocked"
    UNKNOWN = "unknown"


class Status(str, Enum):
    IDLE = "idle"
    WORKING = "working"
    PLANTING = "planting"
    WEEDING = "weeding"
    SAMPLING = "sampling"
    HARVESTING = "harvesting"
    STUCK = "stuck"
    TO_RESCUE = "to_rescue"
    RECOVERED = "recovered"
    DONE = "done"


class Role(str, Enum):
    OMNI = "omni"


class CropState(str, Enum):
    NONE = "none"
    TARGET = "target"
    PLANTED = "planted"
    WEEDED = "weeded"
    HARVESTED = "harvested"


class CropStage(str, Enum):
    NURSERY_ESTABLISHMENT = "nursery_establishment"  # Days -25 to -15
    SEEDLING_GROWTH = "seedling_growth"              # Days -15 to -7
    FIELD_PREPARATION = "field_preparation"          # Days -7 to -1
    TRANSPLANTING = "transplanting"                  # Day 0 onwards
    EARLY_ESTABLISHMENT = "early_establishment"      # Days 1 to 7
    ESTABLISHMENT = "establishment"                  # Days 8 to 14 (gap filling)
    EARLY_TILLERING = "early_tillering"              # Days 15 to 21
    TILLERING = "tillering"                          # Days 22 to 30
    ACTIVE_TILLERING = "active_tillering"            # Days 31 to 42
    LATE_TILLERING = "late_tillering"                # Days 43 to 55
    PANICLE_DEV = "panicle_dev"                      # Days 56 to 70
    HEADING_FLOWERING = "heading_flowering"          # Days 71 to 85
    GRAIN_FILLING = "grain_filling"                  # Days 86 to 105
    RIPENING = "ripening"                            # Days 106 to 115
    HARVEST_READY = "harvest_ready"                  # Days 116 to 120+
    HARVESTED = "harvested"                          # Post-harvest


class MudRiskTier(str, Enum):
    LOW = "LOW"             # 0.00 - 0.29
    MODERATE = "MODERATE"   # 0.30 - 0.59
    HIGH = "HIGH"           # 0.60 - 0.79
    VERY_HIGH = "VERY_HIGH" # 0.80 - 1.00


def get_mud_risk_tier(risk: float) -> MudRiskTier:
    if risk < 0.30:
        return MudRiskTier.LOW
    if risk < 0.60:
        return MudRiskTier.MODERATE
    if risk < 0.80:
        return MudRiskTier.HIGH
    return MudRiskTier.VERY_HIGH


class WeatherType(str, Enum):
    SUNNY = "sunny"
    CLOUDY = "cloudy"
    RAINY = "rainy"
    HEAVY_RAIN = "heavy_rain"


@dataclass
class PlantState:
    pos: Point
    section: str  # "A" or "B"
    stage: CropStage = CropStage.FIELD_PREPARATION
    age_days: float = 0.0
    days_since_transplant: float = 0.0
    height_cm: float = 0.0
    tiller_count: int = 1
    panicle_dev: float = 0.0      # 0.0 to 1.0
    grain_maturity: float = 0.0   # 0.0 to 1.0 (>= 0.95 triggers harvest)
    health: float = 1.0           # 0.0 to 1.0
    alive: bool = True
    planted: bool = False
    weeded_count: int = 0
    harvested: bool = False
    yield_g: float = 0.0


@dataclass
class SectionState:
    name: str  # "Section A" or "Section B"
    calendar_offset: int = 0  # 0 for A (transplant Day 0), 20 for B (transplant Day 20)
    targets: set[Point] = field(default_factory=set)
    nursery_pos: Point = (0, 0)
    nursery_seeds: int = 12
    nursery_sown: int = 0
    nursery_moisture: float = 0.75
    seedlings_ready: int = 0
    field_prepped: bool = False
    water_target_depth: float = 0.08  # meters (Chinese shallow flooding standard)
    current_stage: CropStage = CropStage.NURSERY_ESTABLISHMENT
    harvested_count: int = 0
    total_yield_kg: float = 0.0

    def effective_day(self, current_day: int) -> int:
        return current_day - self.calendar_offset

    def update_stage_from_day(self, current_day: int) -> CropStage:
        eff = self.effective_day(current_day)
        if eff < -15:
            st = CropStage.NURSERY_ESTABLISHMENT
        elif eff < -7:
            st = CropStage.SEEDLING_GROWTH
        elif eff < 0:
            st = CropStage.FIELD_PREPARATION
        elif eff == 0:
            st = CropStage.TRANSPLANTING
        elif eff <= 7:
            st = CropStage.EARLY_ESTABLISHMENT
        elif eff <= 14:
            st = CropStage.ESTABLISHMENT
        elif eff <= 21:
            st = CropStage.EARLY_TILLERING
        elif eff <= 30:
            st = CropStage.TILLERING
        elif eff <= 42:
            st = CropStage.ACTIVE_TILLERING
        elif eff <= 55:
            st = CropStage.LATE_TILLERING
        elif eff <= 70:
            st = CropStage.PANICLE_DEV
        elif eff <= 85:
            st = CropStage.HEADING_FLOWERING
        elif eff <= 105:
            st = CropStage.GRAIN_FILLING
        elif eff <= 115:
            st = CropStage.RIPENING
        elif eff <= 125:
            st = CropStage.HARVEST_READY
        else:
            st = CropStage.HARVESTED
        self.current_stage = st
        return st


@dataclass
class DailyReport:
    day: int
    weather: WeatherType
    section_a_stage: CropStage
    section_b_stage: CropStage
    section_a_planted: int
    section_b_planted: int
    section_a_harvested: int
    section_b_harvested: int
    avg_water_depth: float
    avg_weed_density: float
    avg_mud_risk: float
    mud_risk_tier: MudRiskTier
    active_rescues: int
    soil_sampled: bool
    summary: str


@dataclass
class Cell:
    terrain: Terrain = Terrain.UNKNOWN
    water_depth: float | None = None
    water_temp: float | None = None
    fertility: float | None = None
    soil_n: float | None = None
    soil_p: float | None = None
    soil_k: float | None = None
    soil_ph: float | None = None
    weed_density: float = 0.0  # [0.0, 1.0] weed pressure
    mud_risk: float = 0.0      # [0.0, 1.0] slip / sinkage hazard
    obstacle: bool = False
    crop_state: CropState = CropState.NONE
    crop_stage: CropStage | None = None
    grain_maturity: float = 0.0
    confidence: float = 0.0
    last_seen: int = -1

    @property
    def productivity_score(self) -> float:
        """Estimated transplanting productivity [0, 1] based on observed agronomy."""
        if self.confidence <= 0 or self.obstacle or self.terrain in (Terrain.BLOCKED, Terrain.SOFT):
            return 0.0
        fert = self.fertility if self.fertility is not None else 0.5
        depth = self.water_depth if self.water_depth is not None else 0.08
        water_suitability = max(0.0, 1.0 - abs(depth - 0.09) * 5.0)
        safety_factor = max(0.0, 1.0 - self.mud_risk)
        return round(0.55 * fert + 0.30 * water_suitability + 0.15 * safety_factor, 4)

    @property
    def weeding_urgency(self) -> float:
        """Weeding priority score [0, 1] for already planted rice cells."""
        if self.crop_state != CropState.PLANTED or self.confidence <= 0:
            return 0.0
        return round(self.weed_density, 4)

    @property
    def mud_risk_tier(self) -> MudRiskTier:
        return get_mud_risk_tier(self.mud_risk)


@dataclass
class Robot:
    name: str
    role: Role = Role.OMNI
    pos: Point = (0, 0)
    battery: float = 100.0
    status: Status = Status.IDLE
    can_plant: bool = True
    can_weed: bool = True
    recovery_capacity: float = 3.5  # Capable of pulling peers from mud
    stuck_from: Point | None = None
    planned_route: list[Point] = field(default_factory=list)
    destination: Point | None = None
    current_action: str = ""  # "plant", "weed", "harvest", "sample", "prep", "water", "rescue"
    target_cell: Point | None = None
    planted_count: int = 0
    weeded_count: int = 0
    harvested_count: int = 0
    travelled: int = 0


@dataclass(frozen=True)
class CandidateResponderEval:
    responder_id: str
    feasible: bool
    rejection_reason: str = ""
    stance: Point | None = None
    route: tuple[Point, ...] = ()
    raw_metrics: dict[str, float] = field(default_factory=dict)
    normalised_metrics: dict[str, float] = field(default_factory=dict)
    total_cost: float = inf


@dataclass(frozen=True)
class Decision:
    action: str
    reason: str
    alternatives: tuple[str, ...]
    costs: dict[str, float]
    rescue_stance: Point | None = None
    rescue_route: tuple[Point, ...] = ()
    selected_responder: str | None = None
    normalised_components: dict[str, dict[str, float]] = field(default_factory=dict)
    rejected_actions: dict[str, str] = field(default_factory=dict)
    safety_constraints: tuple[str, ...] = ()
    responder_evaluations: tuple[CandidateResponderEval, ...] = ()


class Field:
    """True physical environment simulating a flooded Chinese paddy plot (12x10 grid).

    Divided into Section A (columns 1..5) and Section B (columns 6..10).
    Perimeter boundaries and irrigation bunds frame the plot.
    """
    def __init__(self, width: int = 12, height: int = 10):
        self.width, self.height = width, height
        self.truth: dict[Point, Terrain] = {
            (x, y): Terrain.FIRM for y in range(height) for x in range(width)
        }
        # In Chinese paddy systems, water inlet/outlet bund dikes and central stones are impassable
        for y in (3, 6):
            self.truth[(5, y)] = Terrain.BLOCKED
        self.truth[(9, 4)] = Terrain.BLOCKED

        # Puddle mud sinkage hazard (soft mud layer with high sinkage risk)
        self.truth[(8, 7)] = Terrain.SOFT
        self.mud_resistance: dict[Point, float] = {(8, 7): 2.5}

        # Standing water depth (typical Chinese flooded paddy ~0.06 - 0.18 m, deep in ditches)
        self.water = {p: 0.08 + ((p[0] * 2 + p[1]) % 4) * 0.015 for p in self.truth}
        self.water[(8, 7)] = 0.30
        self.water[(9, 4)] = 0.0

        # Water temperature (approx 22-28°C in subtropical paddy)
        self.water_temp = {p: 24.0 + ((p[0] + p[1]) % 5) * 0.5 for p in self.truth}

        # Soil fertility profile (alluvial rice soil)
        self.fertility = {
            (x, y): round(0.50 + 0.03 * ((x * 4 + y * 3) % 9), 2)
            for y in range(height) for x in range(width)
        }
        # Soil N-P-K and pH
        self.soil_n = {(x, y): round(80.0 + 3.0 * ((x + y * 2) % 7), 1) for y in range(height) for x in range(width)}
        self.soil_p = {(x, y): round(35.0 + 2.0 * ((x * 2 + y) % 5), 1) for y in range(height) for x in range(width)}
        self.soil_k = {(x, y): round(110.0 + 4.0 * ((x + y) % 8), 1) for y in range(height) for x in range(width)}
        self.soil_ph = {(x, y): round(6.3 + 0.05 * ((x * 3 + y) % 6), 2) for y in range(height) for x in range(width)}

        # Weed pressure map (paddy weeds like barnyard grass / Echinochloa crus-galli)
        self.weed_density = {
            (x, y): round(0.30 + 0.05 * ((x + y * 2) % 10), 2)
            for y in range(height) for x in range(width)
        }

        # 12 designated crop planting targets arranged along paddy planting ridges:
        # Section A: 6 targets (columns 2 & 4)
        # Section B: 6 targets (columns 7 & 10)
        self.crop_targets: set[Point] = {
            (x, y)
            for x in (2, 4, 7, 10)
            for y in (2, 5, 8)
        }
        for pt in self.crop_targets:
            self.fertility[pt] = round(0.85 + 0.02 * ((pt[0] + pt[1]) % 5), 2)
            self.water[pt] = 0.09
            self.weed_density[pt] = 0.75  # High weed pressure before tending

        # Plant biological state models
        self.plants: dict[Point, PlantState] = {}
        for pt in self.crop_targets:
            sec = "A" if pt[0] <= 5 else "B"
            self.plants[pt] = PlantState(pos=pt, section=sec)

        # Two sections with staggered calendar offsets
        self.sections: dict[str, SectionState] = {
            "A": SectionState(
                name="Section A",
                calendar_offset=0,  # Starts Day -25, Transplant target Day 0
                targets={p for p in self.crop_targets if p[0] <= 5},
                nursery_pos=(0, 0),
            ),
            "B": SectionState(
                name="Section B",
                calendar_offset=20,  # Starts Day -5, Transplant target Day 20 (+20 days staggered)
                targets={p for p in self.crop_targets if p[0] >= 6},
                nursery_pos=(11, 0),
            ),
        }

        # Swarm belief map (starts partially known based on prior survey records)
        self.belief: dict[Point, Cell] = {}
        for p in self.truth:
            is_target = p in self.crop_targets
            self.belief[p] = Cell(
                terrain=Terrain.FIRM if self.truth[p] != Terrain.BLOCKED else Terrain.BLOCKED,
                water_depth=0.08,
                water_temp=24.0,
                fertility=0.60 if is_target else 0.45,
                soil_n=80.0,
                soil_p=35.0,
                soil_k=110.0,
                soil_ph=6.4,
                weed_density=0.50 if is_target else 0.20,
                mud_risk=0.05,
                obstacle=self.truth[p] == Terrain.BLOCKED,
                crop_state=CropState.TARGET if is_target else CropState.NONE,
                crop_stage=CropStage.NURSERY_ESTABLISHMENT if is_target else None,
                confidence=0.25,
                last_seen=-1,
            )

        self.planted: set[Point] = set()
        self.weeded: set[Point] = set()
        self.harvested: set[Point] = set()
        self.risk_incidents: set[Point] = set()
        self.last_soil_sample_day: int = -999

    def inside(self, p: Point) -> bool:
        return p in self.truth

    def neighbors(self, p: Point) -> Iterable[Point]:
        x, y = p
        for q in ((x + 1, y), (x, y + 1), (x - 1, y), (x, y - 1)):
            if self.inside(q):
                yield q

    def _cell_crop_state(self, p: Point) -> CropState:
        if p in self.harvested:
            return CropState.HARVESTED
        if p in self.weeded:
            return CropState.WEEDED
        if p in self.planted:
            return CropState.PLANTED
        if p in self.crop_targets:
            return CropState.TARGET
        return CropState.NONE

    def observe(self, pos: Point, tick: int, radius: int = 1) -> list[Point]:
        """Robots continuously scan sensors while traversing, updating shared belief map."""
        observed = []
        for p in self.truth:
            if max(abs(p[0] - pos[0]), abs(p[1] - pos[1])) <= radius:
                terrain = self.truth[p]
                water = self.water[p]
                fert = self.fertility[p]
                weed = self.weed_density[p]
                is_obs = terrain == Terrain.BLOCKED
                mud_r = min(1.0, (0.75 if terrain == Terrain.SOFT else 0.05) + max(0.0, water - 0.12) * 1.8)
                if p in self.risk_incidents:
                    mud_r = 1.0

                c_stage = None
                grain_mat = 0.0
                if p in self.plants:
                    c_stage = self.plants[p].stage
                    grain_mat = self.plants[p].grain_maturity

                self.belief[p] = Cell(
                    terrain=terrain,
                    water_depth=water,
                    water_temp=self.water_temp[p],
                    fertility=fert,
                    soil_n=self.soil_n[p],
                    soil_p=self.soil_p[p],
                    soil_k=self.soil_k[p],
                    soil_ph=self.soil_ph[p],
                    weed_density=weed,
                    mud_risk=round(mud_r, 3),
                    obstacle=is_obs,
                    crop_state=self._cell_crop_state(p),
                    crop_stage=c_stage,
                    grain_maturity=round(grain_mat, 3),
                    confidence=1.0,
                    last_seen=tick,
                )
                observed.append(p)
        return observed

    def note_immobilisation(self, p: Point, tick: int) -> None:
        self.risk_incidents.add(p)
        self.belief[p] = Cell(
            terrain=Terrain.SOFT,
            water_depth=self.water[p],
            water_temp=self.water_temp[p],
            fertility=self.fertility[p],
            soil_n=self.soil_n[p],
            soil_p=self.soil_p[p],
            soil_k=self.soil_k[p],
            soil_ph=self.soil_ph[p],
            weed_density=self.weed_density[p],
            mud_risk=1.0,
            obstacle=False,
            crop_state=self._cell_crop_state(p),
            crop_stage=self.plants[p].stage if p in self.plants else None,
            confidence=1.0,
            last_seen=tick,
        )

    def mark_planted(self, p: Point, current_day: int = 0) -> None:
        self.planted.add(p)
        if p in self.plants:
            plant = self.plants[p]
            plant.planted = True
            plant.stage = CropStage.TRANSPLANTING
            plant.days_since_transplant = 0.0
            plant.height_cm = 16.0
        if p in self.belief:
            self.belief[p].crop_state = CropState.PLANTED
            self.belief[p].crop_stage = CropStage.TRANSPLANTING

    def mark_weeded(self, p: Point) -> None:
        self.weeded.add(p)
        self.weed_density[p] = 0.04
        if p in self.plants:
            self.plants[p].weeded_count += 1
        if p in self.belief:
            self.belief[p].crop_state = CropState.WEEDED
            self.belief[p].weed_density = 0.04

    def mark_harvested(self, p: Point) -> float:
        self.harvested.add(p)
        yield_kg = 0.85
        if p in self.plants:
            plant = self.plants[p]
            plant.harvested = True
            plant.stage = CropStage.HARVESTED
            plant.yield_g = 850.0
            yield_kg = 0.85
        if p in self.belief:
            self.belief[p].crop_state = CropState.HARVESTED
            self.belief[p].crop_stage = CropStage.HARVESTED
        return yield_kg

    def traversable(self, p: Point) -> bool:
        """Driving through already planted rice (unharvested) or mud hazards damages crops/stalls robots."""
        if not self.inside(p) or p in self.risk_incidents:
            return False
        # Planted crops cannot be traversed until harvested
        if p in self.planted and p not in self.harvested:
            return False
        if self.truth[p] in (Terrain.BLOCKED, Terrain.SOFT):
            return False
        cell = self.belief[p]
        if cell.terrain in (Terrain.BLOCKED, Terrain.SOFT) or cell.obstacle:
            return False
        return True

    def movement_cost(self, p: Point) -> float:
        cell = self.belief[p]
        return 1.0 + cell.mud_risk * 1.5 + (1.5 if cell.confidence < 0.5 else 0.0)

    def field_report(self) -> dict[str, int | float]:
        total_maturity = sum(pl.grain_maturity for pl in self.plants.values())
        avg_mat = total_maturity / len(self.plants) if self.plants else 0.0
        return {
            "total_cells": len(self.belief),
            "surveyed_cells": sum(c.confidence >= 0.9 for c in self.belief.values()),
            "crop_targets": len(self.crop_targets),
            "planted_cells": len(self.planted),
            "weeded_cells": len(self.weeded),
            "harvested_cells": len(self.harvested),
            "known_hazards": len(self.risk_incidents),
            "avg_grain_maturity": round(avg_mat, 3),
        }


def shortest_path(
    field: Field,
    start: Point,
    goal: Point,
    forbidden: set[Point] | None = None,
) -> list[Point] | None:
    """A* pathfinding on belief map protecting planted crops and avoiding hazards."""
    forbidden = forbidden or set()
    if not field.inside(start) or not field.inside(goal) or goal in forbidden:
        return None
    if start == goal:
        return [start]
    if not field.traversable(goal):
        return None
    frontier: list[tuple[float, int, Point]] = []
    counter = 0
    heapq.heappush(frontier, (0.0, counter, start))
    g = {start: 0.0}
    back: dict[Point, Point] = {}
    while frontier:
        _, _, cur = heapq.heappop(frontier)
        if cur == goal:
            break
        for nxt in field.neighbors(cur):
            if nxt in forbidden:
                continue
            if not field.traversable(nxt):
                continue
            cost = g[cur] + field.movement_cost(nxt)
            if nxt not in g or cost < g[nxt]:
                g[nxt] = cost
                counter += 1
                h = abs(nxt[0] - goal[0]) + abs(nxt[1] - goal[1])
                heapq.heappush(frontier, (cost + h, counter, nxt))
                back[nxt] = cur
    if goal not in back and start != goal:
        return None
    curr = goal
    out = [curr]
    while curr != start:
        curr = back[curr]
        out.append(curr)
    out.reverse()
    return out


@dataclass(frozen=True)
class RecoveryWeights:
    time: float = 0.25
    energy: float = 0.20
    crop_damage: float = 0.30
    soil_disturbance: float = 0.15
    recovery_risk: float = 0.10

    def total(self) -> float:
        return round(
            self.time + self.energy + self.crop_damage + self.soil_disturbance + self.recovery_risk,
            6,
        )


class RecoveryPolicy:
    """Multi-objective rescue optimization when a swarm unit encounters soft mud."""
    def __init__(self, weights: RecoveryWeights | None = None):
        self.weights = weights or RecoveryWeights()
        assert self.weights.total() == 1.0, "Weights must sum to 1.0"
        self.last_agent_error: str | None = None

    def evaluate(
        self,
        field: Field,
        stuck_robot: Robot,
        swarm: dict[str, Robot],
    ) -> Decision:
        candidates = [r for r in swarm.values() if r.name != stuck_robot.name and r.status != Status.STUCK]
        stuck_loc = stuck_robot.pos
        tow_exit = stuck_robot.stuck_from or (max(0, stuck_loc[0] - 1), stuck_loc[1])
        stuck_mud_resistance = field.mud_resistance.get(stuck_loc, 2.2)

        peer_evals: list[CandidateResponderEval] = []
        for peer in candidates:
            if peer.recovery_capacity < stuck_mud_resistance:
                peer_evals.append(CandidateResponderEval(
                    responder_id=peer.name,
                    feasible=False,
                    rejection_reason=f"Capacity {peer.recovery_capacity} < mud resistance {stuck_mud_resistance}",
                ))
                continue

            viable_stances = [
                nb for nb in field.neighbors(stuck_loc)
                if field.traversable(nb) and nb != tow_exit and nb not in field.planted
            ]
            if not viable_stances:
                viable_stances = [
                    nb for nb in field.neighbors(stuck_loc)
                    if field.inside(nb) and field.truth[nb] != Terrain.BLOCKED and nb not in field.planted
                ]
            if not viable_stances:
                peer_evals.append(CandidateResponderEval(
                    responder_id=peer.name,
                    feasible=False,
                    rejection_reason="No safe unplanted stance adjacent to stuck peer",
                ))
                continue

            best_route: list[Point] | None = None
            best_stance: Point | None = None
            for stance in viable_stances:
                path = shortest_path(field, peer.pos, stance, forbidden={stuck_loc})
                if path is not None and (best_route is None or len(path) < len(best_route)):
                    best_route = path
                    best_stance = stance

            if best_route is None or best_stance is None:
                peer_evals.append(CandidateResponderEval(
                    responder_id=peer.name,
                    feasible=False,
                    rejection_reason="No non-crossing route found avoiding crops and mud",
                ))
                continue

            path_len = len(best_route) - 1
            raw_t = float(path_len + 3)
            raw_e = float(path_len * 0.8 + 8.0)
            raw_c = float(sum(1.0 for pt in best_route if pt in field.planted))
            raw_s = float(field.mud_resistance.get(best_stance, 1.0) * 0.5)
            raw_r = float(max(0.0, 100.0 - peer.battery) / 100.0 * 2.0)

            # Normalise components to [0, 1]
            norm_t = min(1.0, raw_t / 20.0)
            norm_e = min(1.0, raw_e / 30.0)
            norm_c = min(1.0, raw_c / 1.0)
            norm_s = min(1.0, raw_s / 3.0)
            norm_r = min(1.0, raw_r / 2.0)

            cost = (
                self.weights.time * norm_t
                + self.weights.energy * norm_e
                + self.weights.crop_damage * norm_c
                + self.weights.soil_disturbance * norm_s
                + self.weights.recovery_risk * norm_r
            )
            peer_evals.append(CandidateResponderEval(
                responder_id=peer.name,
                feasible=True,
                stance=best_stance,
                route=tuple(best_route),
                raw_metrics={"T": raw_t, "E": raw_e, "C": raw_c, "S": raw_s, "R": raw_r},
                normalised_metrics={"norm_T": norm_t, "norm_E": norm_e, "norm_C": norm_c, "norm_S": norm_s, "norm_R": norm_r},
                total_cost=round(cost, 4),
            ))

        feasible_evals = [e for e in peer_evals if e.feasible]
        costs: dict[str, float] = {}
        norm_comps: dict[str, dict[str, float]] = {}

        costs["self_recovery"] = 0.85
        norm_comps["self_recovery"] = {"norm_T": 0.3, "norm_E": 0.8, "norm_C": 0.0, "norm_S": 0.9, "norm_R": 0.9}
        costs["human_intervention"] = 0.92
        norm_comps["human_intervention"] = {"norm_T": 1.0, "norm_E": 0.1, "norm_C": 0.5, "norm_S": 0.7, "norm_R": 0.2}

        if feasible_evals:
            feasible_evals.sort(key=lambda e: (e.total_cost, e.responder_id))
            best_peer = feasible_evals[0]
            costs["cooperative_rescue"] = best_peer.total_cost
            norm_comps["cooperative_rescue"] = best_peer.normalised_metrics

            return Decision(
                action="cooperative_rescue",
                reason=f"Selected optimal responder {best_peer.responder_id} with minimum multi-objective cost J={best_peer.total_cost}",
                alternatives=("cooperative_rescue", "self_recovery", "human_intervention"),
                costs=costs,
                rescue_stance=best_peer.stance,
                rescue_route=best_peer.route,
                selected_responder=best_peer.responder_id,
                normalised_components=norm_comps,
                rejected_actions={"self_recovery": "High sinkage spin risk in soft mud", "human_intervention": "High labor latency and foot damage"},
                safety_constraints=("zero_crop_crossing", "stable_stance_required"),
                responder_evaluations=tuple(peer_evals),
            )

        costs["cooperative_rescue"] = inf
        return Decision(
            action="human_intervention",
            reason="No feasible autonomous swarm rescuer available without exceeding risk or crop constraints",
            alternatives=("human_intervention", "self_recovery"),
            costs=costs,
            normalised_components=norm_comps,
            rejected_actions={"cooperative_rescue": "No feasible peer route or recovery capacity"},
            safety_constraints=("zero_crop_crossing",),
            responder_evaluations=tuple(peer_evals),
        )


class SwarmCoordinator:
    """Central AI Coordinator dynamically allocating tasks across both sections."""
    def __init__(self, field: Field, robots: dict[str, Robot], policy: RecoveryPolicy):
        self.field = field
        self.robots = robots
        self.policy = policy
        self.active_rescue_decision: Decision | None = None
        self.active_stuck_robot: str | None = None

    def compute_priority_planting_map(self) -> list[tuple[Point, float]]:
        scores: list[tuple[Point, float]] = []
        for target in sorted(self.field.crop_targets):
            if target in self.field.planted:
                continue
            # Respect section stage: only plant if effective day >= 0 and field prepped
            plant_info = self.field.plants.get(target)
            if plant_info:
                sec = self.field.sections.get(plant_info.section)
                if sec and not sec.field_prepped and sec.current_stage == CropStage.FIELD_PREPARATION:
                    continue
            cell = self.field.belief[target]
            score = cell.productivity_score
            scores.append((target, score))
        scores.sort(key=lambda item: item[1], reverse=True)
        return scores

    def compute_priority_weeding_map(self) -> list[tuple[Point, float]]:
        scores: list[tuple[Point, float]] = []
        for target in sorted(self.field.planted):
            if target in self.field.weeded or target in self.field.harvested:
                continue
            cell = self.field.belief[target]
            # Condition: only weed if weed density >= 0.15 (15%)
            if cell.weed_density >= 0.15:
                scores.append((target, cell.weeding_urgency))
        scores.sort(key=lambda item: item[1], reverse=True)
        return scores

    def compute_priority_harvesting_map(self) -> list[tuple[Point, float]]:
        scores: list[tuple[Point, float]] = []
        for target in sorted(self.field.planted):
            if target in self.field.harvested:
                continue
            plant = self.field.plants.get(target)
            cell = self.field.belief[target]
            # Harvest condition: grain maturity >= 0.95 and field drained (water <= 0.04m)
            mat = plant.grain_maturity if plant else cell.grain_maturity
            depth = cell.water_depth if cell.water_depth is not None else 0.0
            if mat >= 0.95 and depth <= 0.04:
                scores.append((target, round(mat, 4)))
        scores.sort(key=lambda item: item[1], reverse=True)
        return scores

    def plan_next_task_for_robot(
        self,
        robot: Robot,
        assigned_targets: set[Point],
    ) -> tuple[str, Point, list[Point], float] | None:
        """Assigns harvesting, planting, or weeding based on current conditions."""
        occupied = {r.pos for r in self.robots.values() if r.name != robot.name}

        # Priority 1: Harvest ripe golden stands
        harvest_candidates = [
            (pt, score) for pt, score in self.compute_priority_harvesting_map()
            if pt not in assigned_targets
        ]
        best_harvest: tuple[float, int, Point, list[Point], float] | None = None
        for target, mat_score in harvest_candidates:
            paths = []
            for stand in self.field.neighbors(target):
                if stand in occupied:
                    continue
                path = shortest_path(self.field, robot.pos, stand)
                if path is not None:
                    paths.append(path)
            if paths:
                paths.sort(key=lambda pth: (len(pth), pth[-1]))
                best_pth = paths[0]
                rank = len(best_pth) - (mat_score * 4.0)
                entry = (rank, len(best_pth), target, best_pth, mat_score)
                if best_harvest is None or entry < best_harvest:
                    best_harvest = entry

        # Priority 2: Plant high-productivity crop targets
        planting_candidates = [
            (pt, score) for pt, score in self.compute_priority_planting_map()
            if pt not in assigned_targets
        ]
        best_plant: tuple[float, int, Point, list[Point], float] | None = None
        for target, prod_score in planting_candidates:
            paths = []
            for stand in self.field.neighbors(target):
                if stand in occupied:
                    continue
                path = shortest_path(self.field, robot.pos, stand)
                if path is not None:
                    paths.append(path)
            if paths:
                paths.sort(key=lambda pth: (len(pth), pth[-1]))
                best_pth = paths[0]
                rank = len(best_pth) - (prod_score * 3.0)
                entry = (rank, len(best_pth), target, best_pth, prod_score)
                if best_plant is None or entry < best_plant:
                    best_plant = entry

        # Priority 3: Weed planted stands exceeding 15% weed pressure
        weeding_candidates = [
            (pt, score) for pt, score in self.compute_priority_weeding_map()
            if pt not in assigned_targets
        ]
        best_weed: tuple[float, int, Point, list[Point], float] | None = None
        for target, weed_score in weeding_candidates:
            paths = []
            for stand in self.field.neighbors(target):
                if stand in occupied:
                    continue
                path = shortest_path(self.field, robot.pos, stand)
                if path is not None:
                    paths.append(path)
            if paths:
                paths.sort(key=lambda pth: (len(pth), pth[-1]))
                best_pth = paths[0]
                rank = len(best_pth) - (weed_score * 2.5)
                entry = (rank, len(best_pth), target, best_pth, weed_score)
                if best_weed is None or entry < best_weed:
                    best_weed = entry

        if best_harvest is not None:
            _, _, target, path, score = best_harvest
            return "harvest", target, path, score
        if best_plant is not None:
            _, _, target, path, score = best_plant
            return "plant", target, path, score
        if best_weed is not None:
            _, _, target, path, score = best_weed
            return "weed", target, path, score
        return None

    def coordinate_rescue_for_stuck_robot(self, stuck_robot: Robot) -> Decision:
        decision = self.policy.evaluate(self.field, stuck_robot, self.robots)
        self.active_rescue_decision = decision
        self.active_stuck_robot = stuck_robot.name
        return decision


class RiceSwarmSimulation:
    """Full-lifecycle simulation of 6 multi-functional swarm robots in a Chinese rice paddy."""
    def __init__(
        self,
        *,
        policy: RecoveryPolicy | None = None,
        introduce_mud: bool = True,
        start_day: int = -25,
    ):
        self.field = Field()
        # 6 Homogeneous Multi-Functional Swarm Robots positioned on perimeter dikes
        self.robots = {
            "RS1": Robot("RS1", Role.OMNI, pos=(0, 1)),
            "RS2": Robot("RS2", Role.OMNI, pos=(0, 4)),
            "RS3": Robot("RS3", Role.OMNI, pos=(0, 8)),
            "RS4": Robot("RS4", Role.OMNI, pos=(11, 1)),
            "RS5": Robot("RS5", Role.OMNI, pos=(11, 5)),
            "RS6": Robot("RS6", Role.OMNI, pos=(11, 8)),
        }
        self.policy = policy or RecoveryPolicy()
        self.coordinator = SwarmCoordinator(self.field, self.robots, self.policy)
        self.introduce_mud = introduce_mud
        self.mud_injected = False
        self.tick_count = 0
        self.current_day = start_day
        self.current_weather = WeatherType.SUNNY
        self.phase = "active"
        self.log: list[dict] = []
        self.daily_reports: list[DailyReport] = []
        self.rescue_count = 0
        self.rescue_successes = 0
        self.human_escalations = 0
        self.unsafe_moves = 0
        self.observers: list[Callable[[RiceSwarmSimulation], None]] = []

    def event(self, kind: str, **details: object) -> None:
        self.log.append({"tick": self.tick_count, "day": self.current_day, "event": kind, **details})

    def trigger_immobilisation(self, robot_name: str = "RS2") -> bool:
        """Manually trigger a robot to become stuck in soft mud for testing."""
        robot = self.robots.get(robot_name)
        if robot is None or robot.status == Status.STUCK:
            return False
        prev = robot.stuck_from
        if prev is None:
            for nb in self.field.neighbors(robot.pos):
                if self.field.traversable(nb):
                    prev = nb
                    break
        if prev is None:
            prev = (max(0, robot.pos[0] - 1), robot.pos[1])
        self.field.truth[robot.pos] = Terrain.SOFT
        self.field.water[robot.pos] = 0.32
        self.field.mud_resistance[robot.pos] = 2.2
        robot.stuck_from = prev
        robot.status = Status.STUCK
        robot.planned_route = []
        robot.destination = None
        self.field.note_immobilisation(robot.pos, self.tick_count)
        self.coordinator.coordinate_rescue_for_stuck_robot(robot)
        self.event("immobilised", robot=robot.name, pos=robot.pos, from_pos=prev, manual=True)
        return True

    def _move(self, robot: Robot, nxt: Point) -> bool:
        if nxt not in self.field.neighbors(robot.pos):
            raise ValueError(f"Non-adjacent move {robot.pos} -> {nxt}")
        if nxt in self.field.planted and nxt not in self.field.harvested:
            self.unsafe_moves += 1
            raise AssertionError(f"Robot {robot.name} crossed planted rice at {nxt}")
        if not self.field.traversable(nxt):
            raise AssertionError(f"Invalid motion into non-traversable cell {nxt}")
        if nxt in {r.pos for r in self.robots.values() if r.name != robot.name and r.status != Status.STUCK}:
            return False

        prev = robot.pos
        robot.pos = nxt
        robot.travelled += 1
        robot.battery = max(0.0, robot.battery - 0.7)

        # Dynamic soft mud encounter
        if self.field.truth[nxt] == Terrain.SOFT and nxt in self.field.mud_resistance:
            robot.stuck_from = prev
            robot.status = Status.STUCK
            robot.planned_route = []
            robot.destination = None
            self.field.note_immobilisation(nxt, self.tick_count)
            self.coordinator.coordinate_rescue_for_stuck_robot(robot)
            d = self.coordinator.active_rescue_decision
            self.event("immobilised", robot=robot.name, pos=nxt, from_pos=prev)
            if d is not None:
                self.event(
                    "rescue_decision",
                    stuck_robot=robot.name,
                    action=d.action,
                    selected_responder=d.selected_responder,
                    costs=d.costs,
                    reason=d.reason,
                )
            return True

        self.field.observe(robot.pos, self.tick_count, radius=1)
        return True

    def _handle_rescue_movement(self, rescuer: Robot, d: Decision) -> None:
        stuck_name = self.coordinator.active_stuck_robot
        if stuck_name not in self.robots:
            return
        stuck_robot = self.robots[stuck_name]
        if rescuer.pos == d.rescue_stance:
            assert stuck_robot.stuck_from is not None
            resistance = self.field.mud_resistance.get(stuck_robot.pos, 2.2)
            if stuck_robot.stuck_from == rescuer.pos or (stuck_robot.stuck_from in self.field.planted and stuck_robot.stuck_from not in self.field.harvested):
                self.event("recovery_aborted", reason="Tow cell blocked or planted")
                self.human_escalations += 1
                rescuer.status = Status.IDLE
                self.coordinator.active_rescue_decision = None
                return
            if rescuer.recovery_capacity < resistance:
                self.event("recovery_aborted", reason="Insufficient capacity")
                self.human_escalations += 1
                rescuer.status = Status.IDLE
                self.coordinator.active_rescue_decision = None
                return

            stuck_robot.pos = stuck_robot.stuck_from
            stuck_robot.stuck_from = None
            stuck_robot.status = Status.RECOVERED
            stuck_robot.battery = max(0.0, stuck_robot.battery - 5.0)
            rescuer.battery = max(0.0, rescuer.battery - 8.0)
            rescuer.status = Status.IDLE
            rescuer.planned_route = []
            rescuer.destination = None
            self.rescue_successes += 1
            self.coordinator.active_rescue_decision = None
            self.coordinator.active_stuck_robot = None
            self.event("recovered", robot=stuck_robot.name, by=rescuer.name, to=stuck_robot.pos)
            return

        assert d.rescue_stance is not None
        path = shortest_path(self.field, rescuer.pos, d.rescue_stance, forbidden={stuck_robot.pos})
        if path is None:
            self.event("rescue_path_invalidated", rescuer=rescuer.name)
            rescuer.status = Status.IDLE
            self.coordinator.active_rescue_decision = None
            return
        if len(path) > 1:
            rescuer.planned_route = list(path)
            self._move(rescuer, path[1])

    def step(self) -> None:
        if self.phase == "done":
            return
        self.tick_count += 1

        # Check rescue status
        active_decision = self.coordinator.active_rescue_decision
        if active_decision is not None and active_decision.action == "cooperative_rescue":
            responder_name = active_decision.selected_responder
            if responder_name in self.robots:
                responder = self.robots[responder_name]
                if responder.status != Status.TO_RESCUE:
                    responder.status = Status.TO_RESCUE
                    self.rescue_count += 1
                self._handle_rescue_movement(responder, active_decision)

        # Coordinate work for remaining swarm robots
        assigned_targets: set[Point] = {
            r.target_cell for r in self.robots.values()
            if r.target_cell is not None and r.status in (Status.WORKING, Status.PLANTING, Status.WEEDING, Status.HARVESTING)
        }

        for name, robot in sorted(self.robots.items()):
            self.field.observe(robot.pos, self.tick_count, radius=1)

            if robot.status in (Status.STUCK, Status.TO_RESCUE, Status.DONE):
                continue

            # In mid-operation, introduce one unexpected mud hazard if enabled
            if (
                self.introduce_mud
                and not self.mud_injected
                and len(self.field.planted) >= 3
                and robot.status == Status.WORKING
                and len(robot.planned_route) > 1
            ):
                step = robot.planned_route[1]
                self.field.truth[step] = Terrain.SOFT
                self.field.water[step] = 0.32
                self.field.mud_resistance[step] = 2.4
                self.mud_injected = True
                self.event("environment_change", pos=step, desc="Submerged soft mud hazard developed in flooded plot")

            # Check if robot has a valid task or needs one
            if robot.target_cell is None or not robot.planned_route:
                plan = self.coordinator.plan_next_task_for_robot(robot, assigned_targets)
                if plan is None:
                    robot.status = Status.IDLE
                    continue
                action_type, target, path, score = plan
                robot.current_action = action_type
                robot.target_cell = target
                robot.planned_route = list(path)
                robot.destination = path[-1]
                robot.status = Status.WORKING
                assigned_targets.add(target)

            # Follow planned route to reach adjacent stand
            if len(robot.planned_route) > 1:
                nxt = robot.planned_route[1]
                if not self.field.traversable(nxt):
                    robot.planned_route = []
                    robot.target_cell = None
                    robot.status = Status.IDLE
                    continue
                moved = self._move(robot, nxt)
                if moved:
                    if robot.status != Status.STUCK:
                        robot.planned_route = robot.planned_route[1:]
                    else:
                        robot.planned_route = []
            elif len(robot.planned_route) == 1:
                target = robot.target_cell
                if target is not None:
                    if robot.current_action == "plant" and target not in self.field.planted:
                        self.field.mark_planted(target, self.current_day)
                        robot.planted_count += 1
                        self.event("planted", robot=robot.name, target=target)
                    elif robot.current_action == "weed" and target in self.field.planted and target not in self.field.weeded:
                        self.field.mark_weeded(target)
                        robot.weeded_count += 1
                        self.event("weeded", robot=robot.name, target=target)
                    elif robot.current_action == "harvest" and target in self.field.planted and target not in self.field.harvested:
                        yield_kg = self.field.mark_harvested(target)
                        robot.harvested_count += 1
                        sec_key = "A" if target[0] <= 5 else "B"
                        self.field.sections[sec_key].harvested_count += 1
                        self.field.sections[sec_key].total_yield_kg += yield_kg
                        self.event("harvested", robot=robot.name, target=target, yield_kg=yield_kg)
                robot.target_cell = None
                robot.planned_route = []
                robot.destination = None
                robot.status = Status.IDLE

        for observer in self.observers:
            observer(self)

    def advance_day(self) -> DailyReport:
        """Simulates 1 agricultural day: weather, biological development, condition eval, actions."""
        self.current_day += 1

        # 1. Weather determination
        roll = (self.current_day * 17 + 7) % 100
        if roll < 55:
            self.current_weather = WeatherType.SUNNY
            evaporation = 0.005
            rain = 0.0
            temp_delta = 1.0
        elif roll < 80:
            self.current_weather = WeatherType.CLOUDY
            evaporation = 0.002
            rain = 0.0
            temp_delta = -0.5
        elif roll < 95:
            self.current_weather = WeatherType.RAINY
            evaporation = 0.0
            rain = 0.025
            temp_delta = -2.0
        else:
            self.current_weather = WeatherType.HEAVY_RAIN
            evaporation = 0.0
            rain = 0.060
            temp_delta = -3.5
            self.event("weather_alert", type="heavy_rain", rainfall_m=rain)

        # 2. Update environmental conditions across field
        for p in self.field.truth:
            if self.field.truth[p] != Terrain.BLOCKED:
                self.field.water[p] = max(0.0, min(0.35, self.field.water[p] - evaporation + rain))
                self.field.water_temp[p] = max(18.0, min(32.0, self.field.water_temp[p] + temp_delta * 0.1))

        # 3. Update section stages
        sec_a = self.field.sections["A"]
        sec_b = self.field.sections["B"]
        stage_a = sec_a.update_stage_from_day(self.current_day)
        stage_b = sec_b.update_stage_from_day(self.current_day)

        # Pre-harvest drainage trigger: lower and maintain drained water depth before and during harvest
        for sec in (sec_a, sec_b):
            eff = sec.effective_day(self.current_day)
            if eff >= 106:
                # Pre-harvest drainage maintains shallow dry field for harvesting equipment
                for pt in sec.targets:
                    self.field.water[pt] = min(0.02, max(0.0, self.field.water[pt] - 0.04))

        # Daily environmental monitoring update into belief map
        for p in self.field.truth:
            mud_r = min(1.0, (0.75 if self.field.truth[p] == Terrain.SOFT else 0.05) + max(0.0, self.field.water[p] - 0.12) * 1.8)
            if p in self.field.risk_incidents:
                mud_r = 1.0
            c_stage = self.field.plants[p].stage if p in self.field.plants else None
            grain_mat = self.field.plants[p].grain_maturity if p in self.field.plants else 0.0
            cell = self.field.belief[p]
            cell.water_depth = self.field.water[p]
            cell.water_temp = self.field.water_temp[p]
            cell.mud_risk = round(mud_r, 3)
            cell.crop_stage = c_stage
            cell.grain_maturity = round(grain_mat, 3)

        # 4. Biological plant growth simulation
        for pt, plant in self.field.plants.items():
            sec = self.field.sections[plant.section]
            eff = sec.effective_day(self.current_day)
            plant.stage = sec.current_stage

            if eff < 0:
                # Nursery stage
                plant.age_days = max(0.0, float(eff + 25))
                plant.height_cm = min(15.0, plant.age_days * 0.6)
            elif plant.planted and not plant.harvested:
                plant.days_since_transplant += 1.0
                plant.age_days += 1.0

                # Tillering
                if 15 <= eff <= 55:
                    if plant.tiller_count < 16:
                        plant.tiller_count += 1
                    plant.height_cm = min(65.0, 15.0 + eff * 0.9)
                # Panicle & Flowering
                elif 56 <= eff <= 85:
                    plant.panicle_dev = min(1.0, (eff - 55) / 30.0)
                    plant.height_cm = min(95.0, 65.0 + (eff - 55) * 0.8)
                # Grain filling
                elif 86 <= eff <= 105:
                    plant.panicle_dev = 1.0
                    plant.grain_maturity = min(0.85, (eff - 85) / 25.0)
                # Ripening
                elif 106 <= eff:
                    plant.grain_maturity = min(1.0, 0.85 + (eff - 105) * 0.015)

                # Weed proliferation if not kept low
                if self.field.weed_density[pt] < 0.8:
                    self.field.weed_density[pt] = min(0.9, self.field.weed_density[pt] + 0.015)

        # 5. Nursery moisture & readiness checks
        for sec in (sec_a, sec_b):
            eff = sec.effective_day(self.current_day)
            if -25 <= eff < -15:
                sec.nursery_sown = min(sec.nursery_seeds, sec.nursery_sown + 3)
            if eff >= -1:
                sec.field_prepped = True
                sec.seedlings_ready = sec.nursery_sown

        # 6. Periodic soil sampling check (approx Days -7, 30, 60, 90 or post heavy rain)
        sampled = False
        if (
            self.current_day in (-7, 30, 60, 90)
            or (self.current_weather == WeatherType.HEAVY_RAIN and self.current_day - self.field.last_soil_sample_day > 7)
        ):
            sampled = True
            self.field.last_soil_sample_day = self.current_day
            self.event("soil_sampled", day=self.current_day, n=82.0, p=36.0, k=112.0, ph=6.45)

        # 7. Execute operations for the day: run robot simulation ticks to carry out tasks
        # Each day robots have a movement/action budget (e.g. 25 ticks)
        for _ in range(25):
            self.step()

        # 8. Compile daily report
        avg_water = sum(self.field.water.values()) / len(self.field.water)
        avg_weed = sum(self.field.weed_density.values()) / len(self.field.weed_density)
        all_mud = [self.field.belief[p].mud_risk for p in self.field.belief]
        avg_mud = sum(all_mud) / len(all_mud)
        mud_tier = get_mud_risk_tier(avg_mud)

        p_a = sum(1 for p in sec_a.targets if p in self.field.planted)
        p_b = sum(1 for p in sec_b.targets if p in self.field.planted)
        h_a = sum(1 for p in sec_a.targets if p in self.field.harvested)
        h_b = sum(1 for p in sec_b.targets if p in self.field.harvested)

        summary_msg = (
            f"Day {self.current_day} ({self.current_weather.value}): "
            f"Sec A [{stage_a.value} planted={p_a}/6 h={h_a}/6], "
            f"Sec B [{stage_b.value} planted={p_b}/6 h={h_b}/6], "
            f"Water={avg_water*100:.1f}cm, Mud={mud_tier.value}"
        )
        report = DailyReport(
            day=self.current_day,
            weather=self.current_weather,
            section_a_stage=stage_a,
            section_b_stage=stage_b,
            section_a_planted=p_a,
            section_b_planted=p_b,
            section_a_harvested=h_a,
            section_b_harvested=h_b,
            avg_water_depth=round(avg_water, 3),
            avg_weed_density=round(avg_weed, 3),
            avg_mud_risk=round(avg_mud, 3),
            mud_risk_tier=mud_tier,
            active_rescues=self.rescue_count,
            soil_sampled=sampled,
            summary=summary_msg,
        )
        self.daily_reports.append(report)
        return report

    def run_season(self, target_day: int = 135) -> dict:
        """Run through the full season from current_day up to target_day."""
        while self.current_day < target_day:
            self.advance_day()
        return self.summary()

    def run(self, max_steps: int = 500) -> dict:
        for _ in range(max_steps):
            if self.phase == "done":
                break
            # Also auto-complete if all crops are harvested
            if len(self.field.harvested) == len(self.field.crop_targets):
                self.phase = "done"
                break
            self.step()
        return self.summary()

    def summary(self) -> dict:
        sec_a = self.field.sections["A"]
        sec_b = self.field.sections["B"]
        return {
            "phase": self.phase,
            "day": self.current_day,
            "weather": self.current_weather.value,
            "ticks": self.tick_count,
            "crop_targets": len(self.field.crop_targets),
            "planted": len(self.field.planted),
            "weeded": len(self.field.weeded),
            "harvested": len(self.field.harvested),
            "section_a": {
                "stage": sec_a.current_stage.value,
                "planted": sum(1 for p in sec_a.targets if p in self.field.planted),
                "harvested": sec_a.harvested_count,
                "yield_kg": round(sec_a.total_yield_kg, 2),
            },
            "section_b": {
                "stage": sec_b.current_stage.value,
                "planted": sum(1 for p in sec_b.targets if p in self.field.planted),
                "harvested": sec_b.harvested_count,
                "yield_kg": round(sec_b.total_yield_kg, 2),
            },
            "field_report": self.field.field_report(),
            "rescue_attempts": self.rescue_count,
            "successful_recoveries": self.rescue_successes,
            "human_escalations": self.human_escalations,
            "unsafe_crop_movements": self.unsafe_moves,
            "robots": {
                k: {
                    "position": v.pos,
                    "battery": round(v.battery, 1),
                    "status": v.status.value,
                    "planted": v.planted_count,
                    "weeded": v.weeded_count,
                    "harvested": v.harvested_count,
                    "travelled": v.travelled,
                }
                for k, v in self.robots.items()
            },
            "events_count": len(self.log),
            "daily_reports_count": len(self.daily_reports),
        }
