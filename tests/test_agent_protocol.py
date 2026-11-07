"""Check that the current official rubric and developer-agent handoff stay coherent.

Documentation contract checks only: they do not validate any robotics runtime.
"""
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WEIGHTS = {
    "Perception-driven action": 25,
    "Adaptation to change": 25,
    "Operational autonomy": 15,
    "Relevance and impact": 20,
    "Ambition and technical quality": 15,
}


class AgentProtocolDocsTests(unittest.TestCase):
    def test_full_robotics_rubric_has_five_named_categories_and_sums_to_100(self):
        text = (ROOT / "docs/EVENT.md").read_text(encoding="utf-8")
        actual = dict(
            (name.strip(), int(weight))
            for name, weight in re.findall(r"^\| (.+?) \| \*\*(\d+)%\*\* \|", text, re.M)
        )
        self.assertEqual(actual, WEIGHTS)
        self.assertEqual(sum(actual.values()), 100)
        self.assertIn("handles its most critical failure mode", text)

    def test_beneficiary_gate_and_communication_chain_are_written(self):
        product = (ROOT / "docs/PRODUCT.md").read_text(encoding="utf-8")
        team = (ROOT / "docs/TEAM.md").read_text(encoding="utf-8")
        self.assertIn("Relevance and impact — 20%", product)
        for word in ("ASSIGN", "BLOCKED", "HANDOFF", "READY FOR REVIEW", "INTEGRATED"):
            self.assertIn(word, team)
        self.assertIn("ChatGPT", team)
        self.assertIn("Gemini", team)

    def test_pull_request_template_requests_observed_checks_not_promises(self):
        template = (ROOT / ".github/pull_request_template.md").read_text(encoding="utf-8")
        self.assertIn("actually run", template)
        self.assertIn("observed result", template)
        self.assertIn("human integrator", template)
        self.assertIn("critical failure", template)

    def test_agent_handoff_skill_matches_claude_compatibility_mirror(self):
        source = ROOT / ".agents/skills/parallel-handoff/SKILL.md"
        mirror = ROOT / ".claude/skills/parallel-handoff/SKILL.md"
        self.assertEqual(source.read_bytes(), mirror.read_bytes())


if __name__ == "__main__":
    unittest.main()
