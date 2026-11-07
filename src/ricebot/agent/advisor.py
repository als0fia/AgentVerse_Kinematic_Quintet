"""Tier 2: Shared Field-Edge Hub (src/ricebot/agent/advisor.py).

Deliberative AI Supervisor with Safety Guardrails:
- Single edge AI box / tablet per farm (or Gemini API when connectivity permits)
- Triggered ONLY on discrete events (quality_overturned, route_obstructed, robot_stranded)
- Reasons over swarm JSON state to reallocate tasks
- Immediate safe fallback to Tier-1 micro-edge auction when offline (link_ok=False)
"""
from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from typing import Any

from ..contracts import (
    ObservationFrame,
    RobotCapability,
    TaskIntent,
    TaskMode,
)
from ..mission import MeasuredFieldMap


class GeminiSwarmAdvisor:
    """Event-Driven AI Supervisor with Safety Guardrails (Tier 2 System 2)."""

    def __init__(
        self,
        api_key: str | None = None,
        model: str = "gemini-1.5-flash",
        timeout_s: float = 2.0,
        enabled: bool = True,
    ):
        self.api_key = api_key or os.environ.get("GEMINI_API_KEY")
        self.model = model
        self.timeout_s = timeout_s
        self.enabled = enabled
        self._last_event: str | None = None
        self._site_eligibility_cache: dict[str, bool] = {}

    @property
    def is_available(self) -> bool:
        return bool(self.enabled and self.api_key)

    def detect_event(
        self,
        frame: ObservationFrame,
        field_map: MeasuredFieldMap,
        prev_frame: ObservationFrame | None,
    ) -> str | None:
        """Detects whether a mid-episode surprise event occurred warranting System 2 intervention."""
        # 1. Route obstruction: robot front obstacle within stop range
        for robot in frame.robots:
            if robot.front_range_m is not None and robot.front_range_m <= 0.35:
                self._last_event = "route_obstructed"
                return "route_obstructed"

        # 2. Robot stranded / range sensor fault
        for robot in frame.robots:
            if robot.front_range_m is None:
                self._last_event = "robot_stranded"
                return "robot_stranded"

        # 3. Quality overturned: site eligibility flipped after fresh samples
        for site_id in field_map.sites:
            est = field_map.estimate(site_id)
            if est.complete:
                currently_eligible = bool(
                    est.stand_gap is not None and est.stand_gap >= 0.6
                    and est.moisture is not None and 0.25 <= est.moisture <= 0.8
                    and est.firmness is not None and 0.2 <= est.firmness <= 0.9
                )
                if site_id in self._site_eligibility_cache:
                    was_eligible = self._site_eligibility_cache[site_id]
                    if was_eligible and not currently_eligible:
                        self._site_eligibility_cache[site_id] = currently_eligible
                        self._last_event = "quality_overturned"
                        return "quality_overturned"
                self._site_eligibility_cache[site_id] = currently_eligible

        return None

    def _build_payload(
        self,
        frame: ObservationFrame,
        field_map: MeasuredFieldMap,
        capabilities: dict[str, RobotCapability],
        event: str,
        excluded_sites: frozenset[str],
    ) -> dict[str, Any]:
        """Constructs compact JSON state for the deliberative AI model."""
        robots_data = []
        for r in frame.robots:
            robots_data.append({
                "robot_id": r.robot_id,
                "x_m": round(r.pose.x_m, 2),
                "y_m": round(r.pose.y_m, 2),
                "seeds_remaining": r.seeds_remaining,
                "front_range_m": round(r.front_range_m, 2) if r.front_range_m is not None else None,
                "can_plant": capabilities[r.robot_id].can_plant,
                "can_sample": capabilities[r.robot_id].can_sample,
            })

        sites_data = []
        for s in field_map.sites.values():
            if not s.permitted or s.site_id in excluded_sites:
                continue
            est = field_map.estimate(s.site_id)
            sites_data.append({
                "site_id": s.site_id,
                "center_x": round(s.center.x_m, 2),
                "center_y": round(s.center.y_m, 2),
                "complete": est.complete,
                "stand_gap": round(est.stand_gap, 2) if est.stand_gap is not None else None,
                "moisture": round(est.moisture, 2) if est.moisture is not None else None,
                "firmness": round(est.firmness, 2) if est.firmness is not None else None,
                "samples": est.sample_count,
            })

        return {
            "episode_id": frame.episode_id,
            "time_s": round(frame.time_s, 2),
            "event": event,
            "robots": robots_data,
            "sites": sites_data,
        }

    def advise(
        self,
        frame: ObservationFrame,
        field_map: MeasuredFieldMap,
        capabilities: dict[str, RobotCapability],
        event: str,
        excluded_sites: frozenset[str] = frozenset(),
    ) -> tuple[TaskIntent, ...] | None:
        """Prompts Gemini model with live JSON state and returns suggested TaskIntents or None."""
        if not self.is_available:
            return None

        payload = self._build_payload(frame, field_map, capabilities, event, excluded_sites)
        prompt = (
            "You are an agricultural multi-robot swarm supervisor. "
            f"An unexpected event occurred: {event}. "
            "Given the JSON state below, allocate exactly one task per robot. "
            "Rules: Only plant if site is complete and eligible and robot has seeds > 0. "
            "No duplicate sites. If unsure, assign sample or hold. "
            "Respond ONLY with a JSON object: {\"assignments\": [{\"robot_id\": \"...\", \"mode\": \"plant\"|\"sample\"|\"hold\", \"site_id\": \"...\", \"reason\": \"...\"}]}\n\n"
            f"State:\n{json.dumps(payload)}"
        )

        try:
            url = f"https://generativelanguage.googleapis.com/v1beta/models/{self.model}:generateContent?key={self.api_key}"
            req_body = json.dumps({
                "contents": [{"parts": [{"text": prompt}]}],
                "generationConfig": {"temperature": 0.1, "maxOutputTokens": 300},
            }).encode("utf-8")

            req = urllib.request.Request(
                url,
                data=req_body,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                data = json.loads(resp.read().decode("utf-8"))

            text = data["candidates"][0]["content"]["parts"][0]["text"].strip()
            # Clean possible markdown wrapping
            if text.startswith("```json"):
                text = text[7:]
            if text.startswith("```"):
                text = text[3:]
            if text.endswith("```"):
                text = text[:-3]
            parsed = json.loads(text.strip())
            assignments = parsed.get("assignments", [])

            intents: list[TaskIntent] = []
            for item in assignments:
                rid = item["robot_id"]
                mode_str = item["mode"].lower()
                target_site = item.get("site_id") or None
                reason = item.get("reason", f"Gemini advisor: {event}")

                mode_map = {
                    "plant": TaskMode.PLANT,
                    "sample": TaskMode.SAMPLE,
                    "hold": TaskMode.HOLD,
                }
                mode = mode_map.get(mode_str, TaskMode.HOLD)
                if mode == TaskMode.HOLD:
                    target_site = None

                intents.append(
                    TaskIntent(
                        task_id=f"{frame.episode_id}:{frame.time_s:.3f}:{rid}:{mode.value}",
                        robot_id=rid,
                        mode=mode,
                        target_site_id=target_site,
                        issued_at_s=frame.time_s,
                        expires_at_s=frame.time_s + 1.0,
                        reason=reason,
                    )
                )

            # Ensure all robots in frame are covered
            covered = {i.robot_id for i in intents}
            if covered != {r.robot_id for r in frame.robots}:
                return None

            return tuple(intents)

        except (urllib.error.URLError, json.JSONDecodeError, KeyError, TimeoutError, OSError):
            # Safe instant fallback to Tier 1 on any network timeout, bad JSON, or API error
            return None
