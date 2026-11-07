import unittest
from pathlib import Path

from scripts.check_repo import ROOT, find_errors


class FoundationTests(unittest.TestCase):
    def test_foundation_paths_references_and_skill_metadata(self):
        self.assertEqual(find_errors(ROOT), [])

    def test_contract_schema_valid_json(self):
        import json
        schema = json.loads((ROOT / "schemas/episode.schema.json").read_text(encoding="utf-8"))
        self.assertEqual(schema["type"], "object")
        self.assertFalse(schema["additionalProperties"])


if __name__ == "__main__":
    unittest.main()
