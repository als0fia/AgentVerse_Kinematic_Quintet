"""Regression checks for reusable human/agent onboarding and module contracts.

These are DOCUMENTATION checks. They do not install software or prove Isaac Sim.
"""
import ast
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class ComponentDocumentationTests(unittest.TestCase):
    def test_core_runbook_and_components_exist(self):
        workstation = (ROOT / "docs/WORKSTATION-SETUP.md").read_text(encoding="utf-8")
        components = (ROOT / "docs/COMPONENTS.md").read_text(encoding="utf-8")
        for claim in ("check_repo.py", "sync_skills.py --check", "unittest discover",
                      "smoke_mujoco.py", "Isaac Sim", "Antigravity", "Gemini CLI"):
            self.assertIn(claim, workstation)
        for claim in ("SimulationPort", "world", "robot", "integration", "README.md",
                      "hidden", "audit", "interchangeable"):
            self.assertIn(claim, components.lower() if claim in {"world", "robot", "integration", "hidden", "audit", "interchangeable"} else components)

    def test_audit_and_setup_skills_are_mirrored(self):
        for name in ("module-audit", "workstation-setup"):
            original = ROOT / f".agents/skills/{name}/SKILL.md"
            mirror = ROOT / f".claude/skills/{name}/SKILL.md"
            self.assertEqual(original.read_bytes(), mirror.read_bytes())

    def test_optional_toy_preflight_is_not_isaac_or_mission(self):
        script = (ROOT / "scripts/smoke_mujoco.py").read_text(encoding="utf-8")
        ast.parse(script)
        self.assertIn("mj_step", script)
        self.assertIn("mj_resetData", script)
        self.assertIn("NOT TESTED: Isaac Sim", script)


if __name__ == "__main__":
    unittest.main()
