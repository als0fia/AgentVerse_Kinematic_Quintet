"""Optional Gemini *high-level* recovery adviser for 6-robot rice swarm."""
from __future__ import annotations

from dataclasses import replace
import json
from typing import Callable
from urllib.request import Request, urlopen
from .core import Decision, Field, RecoveryPolicy, RecoveryWeights, Robot


class GeminiRecoveryAdviser:
    def __init__(
        self,
        api_key: str,
        model: str,
        *,
        timeout: float = 12.0,
        opener: Callable | None = None,
    ):
        if not api_key or not model:
            raise ValueError("Provide a Gemini API key and an available model ID")
        if not all(c.isascii() and (c.isalnum() or c in ".-_") for c in model):
            raise ValueError("Invalid model identifier")
        self.api_key, self.model, self.timeout = api_key, model, timeout
        self.opener = opener or urlopen

    def select(
        self,
        decision: Decision,
        field: Field,
        stuck_robot: Robot,
    ) -> tuple[str, str]:
        situation = {
            "objective": (
                "Safely coordinate retrieval of an immobilised rice swarm robot in a flooded paddy plot. "
                "Minimise crop damage, soil disturbance, time and energy. Do not invent unmeasured observations."
            ),
            "stuck_robot": {
                "name": stuck_robot.name,
                "position": stuck_robot.pos,
                "battery": stuck_robot.battery,
                "stuck_from": stuck_robot.stuck_from,
            },
            "allowed_actions_and_costs": decision.costs,
            "normalised_cost_components": decision.normalised_components,
            "rejected_unsafe_actions": decision.rejected_actions,
            "responder_evaluations": [
                {
                    "responder": ev.responder_id,
                    "feasible": ev.feasible,
                    "stance": ev.stance,
                    "route": ev.route,
                    "total_cost": ev.total_cost if ev.feasible else None,
                    "rejection_reason": ev.rejection_reason,
                }
                for ev in decision.responder_evaluations
            ],
            "default_safe_action": decision.action,
            "required_output": {"action": "one allowed action", "reason": "short explanation"},
        }
        payload = {
            "systemInstruction": {
                "parts": [
                    {
                        "text": (
                            "You are an agricultural swarm recovery mission adviser for Chinese rice cultivation. "
                            "Respond ONLY with a JSON object containing action and reason. Choose ONLY from "
                            "allowed_actions_and_costs."
                        )
                    }
                ]
            },
            "contents": [{"role": "user", "parts": [{"text": json.dumps(situation)}]}],
            "generationConfig": {"responseMimeType": "application/json", "temperature": 0},
        }
        url = f"https://generativelanguage.googleapis.com/v1beta/models/{self.model}:generateContent"
        request = Request(
            url,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json", "x-goog-api-key": self.api_key},
            method="POST",
        )
        with self.opener(request, timeout=self.timeout) as response:
            body = json.loads(response.read().decode("utf-8"))
        text = body["candidates"][0]["content"]["parts"][0]["text"]
        result = json.loads(text)
        choice = result.get("action")
        if choice not in decision.costs:
            raise ValueError(f"Gemini proposed forbidden action: {choice!r}")
        return choice, str(result.get("reason", ""))[:240]


class AgentAdvisedRecoveryPolicy(RecoveryPolicy):
    """Use deterministic feasibility plus optional LLM choice; fail closed."""
    def __init__(self, adviser: GeminiRecoveryAdviser, weights: RecoveryWeights | None = None):
        super().__init__(weights)
        self.adviser = adviser
        self.last_agent_error: str | None = None

    def evaluate(
        self,
        field: Field,
        stuck_robot: Robot,
        swarm: dict[str, Robot],
    ) -> Decision:
        safe = super().evaluate(field, stuck_robot, swarm)
        try:
            action, justification = self.adviser.select(safe, field, stuck_robot)
            if action not in safe.costs:
                raise ValueError(f"Adviser selected action outside feasible set: {action!r}")
        except Exception as exc:
            self.last_agent_error = f"{type(exc).__name__}: {exc}"
            return safe
        self.last_agent_error = None
        return replace(
            safe,
            action=action,
            reason=f"Gemini adviser (safety-filtered): {justification}",
            rescue_stance=safe.rescue_stance if action == "cooperative_rescue" else None,
            rescue_route=safe.rescue_route if action == "cooperative_rescue" else (),
            selected_responder=safe.selected_responder if action == "cooperative_rescue" else None,
        )
