"""Cross-model agent skill and instruction contract regression tests."""
import shutil
import tempfile
import unittest
from pathlib import Path

from scripts.check_repo import ROOT
from scripts.sync_skills import EXPECTED_SKILLS, find_skill_errors


class InteropTests(unittest.TestCase):
    def test_both_native_skill_trees_are_identical(self):
        self.assertEqual(find_skill_errors(ROOT), [])
        self.assertEqual(len(EXPECTED_SKILLS), 9)
        self.assertIn('embodied-loop', EXPECTED_SKILLS)
        self.assertIn('module-audit', EXPECTED_SKILLS)
        self.assertIn('workstation-setup', EXPECTED_SKILLS)

    def test_entrypoints_import_the_canonical_instructions(self):
        claude = (ROOT / 'CLAUDE.md').read_text(encoding='utf-8')
        gemini = (ROOT / 'GEMINI.md').read_text(encoding='utf-8')
        self.assertIn('\n@AGENTS.md\n', claude)
        self.assertIn('\n@./AGENTS.md\n', gemini)
        self.assertTrue((ROOT / 'AGENTS.md').is_file())
        self.assertEqual(len(list((ROOT / '.claude/agents').glob('*.md'))), 4)

    def test_detects_mirror_drift_and_can_restore_without_weakening(self):
        with tempfile.TemporaryDirectory() as tmp:
            sandbox = Path(tmp)
            for d in ('.agents/skills', '.claude/skills'):
                shutil.copytree(ROOT / d, sandbox / d)
            target = sandbox / '.claude/skills/problem-first/SKILL.md'
            target.write_text(target.read_text(encoding='utf-8') + '\nModified incorrectly.\n', encoding='utf-8')
            self.assertTrue(any('drift: problem-first' in x for x in find_skill_errors(sandbox)))
            self.assertEqual(find_skill_errors(sandbox, repair=True), [])
            self.assertEqual(target.read_bytes(), (sandbox / '.agents/skills/problem-first/SKILL.md').read_bytes())

    def test_missing_canonical_skill_fails_even_when_claude_mirror_exists(self):
        with tempfile.TemporaryDirectory() as tmp:
            sandbox = Path(tmp)
            for d in ('.agents/skills', '.claude/skills'):
                shutil.copytree(ROOT / d, sandbox / d)
            shutil.rmtree(sandbox / '.agents/skills/root-cause-debug')
            problems = find_skill_errors(sandbox, repair=True)
            self.assertTrue(any('missing canonical skill: root-cause-debug' in x for x in problems))
            self.assertTrue(any('extra Claude skill' in x for x in problems))

    def test_shared_team_guards_are_documented(self):
        root = (ROOT / 'docs/TEAM.md').read_text(encoding='utf-8')
        self.assertIn('expected-head/lease check', root)
        self.assertIn('non-fast-forward', root)
        self.assertIn('separate worktrees', root)
        self.assertIn('no extra ticket/meeting', root)
        self.assertIn('Three short prompts humans can give agents', root)

    def test_missing_claude_mirror_is_detected_and_repaired(self):
        with tempfile.TemporaryDirectory() as tmp:
            sandbox = Path(tmp)
            for d in ('.agents/skills', '.claude/skills'):
                shutil.copytree(ROOT / d, sandbox / d)
            target = sandbox / '.claude/skills/evaluate-and-demo/SKILL.md'
            target.unlink()
            self.assertTrue(any('missing or symlink Claude mirror' in x for x in find_skill_errors(sandbox)))
            self.assertEqual(find_skill_errors(sandbox, repair=True), [])
            self.assertTrue(target.is_file())


if __name__ == '__main__':
    unittest.main()
