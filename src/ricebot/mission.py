"""Minimal measured-data fusion and deterministic task baseline.

This is intentionally NOT a calibrated agronomy estimator or an AI model.
The mapping/planning owner can replace these implementations behind the APIs.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import hypot

from .contracts import (
    ObservationFrame, RobotCapability, SensorKind, Site, SoilSample,
    TaskIntent, TaskMode,
)


@dataclass(frozen=True)
class SiteEstimate:
    site_id: str
    moisture: float | None = None
    firmness: float | None = None
    stand_gap: float | None = None
    sample_count: int = 0
    source_robot_ids: tuple[str, ...] = ()

    @property
    def complete(self) -> bool:
        return self.moisture is not None and self.firmness is not None and self.stand_gap is not None


class MeasuredFieldMap:
    """Cell-local sample means; unknown cells remain unknown (no oracle map)."""

    def __init__(self, sites: tuple[Site, ...], sample_radius_m: float = 0.6):
        if sample_radius_m <= 0:
            raise ValueError("sample radius must be positive")
        if not sites or len({s.site_id for s in sites}) != len(sites):
            raise ValueError("sites must be nonempty and have unique IDs")
        self.sites = {s.site_id: s for s in sites}
        self.sample_radius_m = sample_radius_m
        self._samples: dict[str, SoilSample] = {}

    def update(self, frame: ObservationFrame) -> int:
        accepted = 0
        for robot in frame.robots:
            for sample in robot.samples:
                if not sample.valid or sample.site_id not in self.sites:
                    continue
                site = self.sites[sample.site_id]
                site_distance = hypot(sample.location.x_m - site.center.x_m,
                                      sample.location.y_m - site.center.y_m)
                robot_distance = hypot(sample.location.x_m - robot.pose.x_m,
                                       sample.location.y_m - robot.pose.y_m)
                # Adapter MUST sample locally. Enforce approximate sensor locality
                # from its observed pose rather than using privileged world truth.
                if max(site_distance, robot_distance) > self.sample_radius_m:
                    continue
                previous = self._samples.get(sample.sample_id)
                if previous is not None:
                    if previous != sample:
                        raise ValueError("sample ID reused with different data")
                    continue
                self._samples[sample.sample_id] = sample
                accepted += 1
        return accepted

    def estimate(self, site_id: str) -> SiteEstimate:
        if site_id not in self.sites:
            raise KeyError(site_id)
        measurements = [s for s in self._samples.values() if s.site_id == site_id]
        def mean(kind: SensorKind) -> float | None:
            values = [s.value for s in measurements if s.kind == kind]
            return (sum(values) / len(values)) if values else None
        return SiteEstimate(
            site_id=site_id,
            moisture=mean(SensorKind.MOISTURE),
            firmness=mean(SensorKind.FIRMNESS),
            stand_gap=mean(SensorKind.STAND_GAP),
            sample_count=len(measurements),
            source_robot_ids=tuple(sorted({s.robot_id for s in measurements})),
        )


class SimpleMissionPlanner:
    """Inspectable baseline; future agent replaces decide(), not the contracts.

    Public allowed sites and measured values only. No hidden suitability field.
    Initially assigns a survey to unknown sites; plants only where all three
    locally measured synthetic proxies meet the configured safe envelope.
    """

    def __init__(self, capabilities: tuple[RobotCapability, ...],
                 min_gap: float = 0.6, moisture_range: tuple[float, float] = (0.25, 0.8),
                 firmness_range: tuple[float, float] = (0.2, 0.9)):
        if not capabilities or len({c.robot_id for c in capabilities}) != len(capabilities):
            raise ValueError("unique nonempty capabilities required")
        self.capabilities = {c.robot_id: c for c in capabilities}
        if not 0 <= min_gap <= 1 or any(not 0 <= lo <= hi <= 1 for lo, hi in (moisture_range, firmness_range)):
            raise ValueError("synthetic thresholds must lie within 0..1")
        self.min_gap = min_gap
        self.moisture_range = moisture_range
        self.firmness_range = firmness_range

    def _eligible(self, estimate: SiteEstimate) -> bool:
        return bool(estimate.complete and estimate.stand_gap >= self.min_gap
                    and self.moisture_range[0] <= estimate.moisture <= self.moisture_range[1]
                    and self.firmness_range[0] <= estimate.firmness <= self.firmness_range[1])

    def decide(self, frame: ObservationFrame, field_map: MeasuredFieldMap,
               excluded_sites: frozenset[str] = frozenset()) -> tuple[TaskIntent, ...]:
        intents: list[TaskIntent] = []
        assigned: set[str] = set()
        robots = sorted(frame.robots, key=lambda r: (not self.capabilities[r.robot_id].can_plant, r.robot_id))
        for robot in robots:
            capability = self.capabilities[robot.robot_id]
            candidates = [site for site in field_map.sites.values() if site.permitted and site.site_id not in assigned and site.site_id not in excluded_sites]
            def distance(site: Site) -> float:
                return hypot(robot.pose.x_m - site.center.x_m, robot.pose.y_m - site.center.y_m)
            ready = sorted((site for site in candidates
                            if self._eligible(field_map.estimate(site.site_id))),
                           key=lambda site: (-field_map.estimate(site.site_id).stand_gap, distance(site), site.site_id))
            unknown = sorted((site for site in candidates
                              if not field_map.estimate(site.site_id).complete),
                             key=lambda site: (distance(site), site.site_id))
            if capability.can_plant and robot.seeds_remaining > 0 and ready:
                mode, target, reason = TaskMode.PLANT, ready[0], "observed eligible gap"
            elif capability.can_sample and unknown:
                mode, target, reason = TaskMode.SAMPLE, unknown[0], "need local evidence"
            else:
                mode, target, reason = TaskMode.HOLD, None, "no validated accessible work"
            if target is not None:
                assigned.add(target.site_id)
            intents.append(TaskIntent(
                task_id=f"{frame.episode_id}:{frame.time_s:.3f}:{robot.robot_id}:{mode.value}",
                robot_id=robot.robot_id, mode=mode, target_site_id=target.site_id if target else None,
                issued_at_s=frame.time_s, expires_at_s=frame.time_s + 1.0, reason=reason,
            ))
        return tuple(intents)
