#!/usr/bin/env python3
"""Small zero-dependency check for foundational docs, skill metadata and local links."""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[1]
REQUIRED = [
    "README.md", "AGENTS.md", "CLAUDE.md", "GEMINI.md", "docs/EVENT.md", "docs/PRODUCT.md",
    "docs/ARCHITECTURE.md", "docs/CONTRACTS.md", "docs/TEAM.md", "docs/RUN.md",
    "docs/TESTING.md", "docs/DEMO.md", "docs/PROVENANCE.md", "docs/DECISIONS.md",
    "docs/RESEARCH.md", "docs/COMPONENTS.md", "docs/WORKSTATION-SETUP.md", "schemas/episode.schema.json", "scripts/evaluate.py",
    ".github/workflows/foundation-checks.yml", "scripts/sync_skills.py",
]
LINK = re.compile(r"(?<!!)\[[^\]]*\]\(([^)]+)\)")


def find_errors(root: Path = ROOT) -> list[str]:
    errors: list[str] = []
    for path in REQUIRED:
        if not (root / path).is_file():
            errors.append(f"missing: {path}")
    for path in sorted(root.rglob("SKILL.md")):
        if ".venv" in path.parts:
            continue
        content = path.read_text(encoding="utf-8")
        if not content.startswith("---\n") or not re.search(r"(?m)^name: [a-z][a-z0-9-]+$", content) or not re.search(r"(?m)^description: .{20,}$", content):
            errors.append(f"invalid skill frontmatter: {path.relative_to(root)}")
        if not content.count("\n---\n"):
            errors.append(f"unclosed skill frontmatter: {path.relative_to(root)}")
    if not [p for p in root.rglob("SKILL.md") if ".venv" not in p.parts]:
        errors.append("no task-scoped skills")
    schema_path = root / "schemas/episode.schema.json"
    if schema_path.is_file():
        try:
            schema = json.loads(schema_path.read_text(encoding="utf-8"))
            if set(schema.get("required", [])) != {"case_id", "seed", "success", "elapsed_ms"}:
                errors.append("episode schema required fields differ from evaluator")
        except ValueError as exc:
            errors.append(f"invalid episode schema JSON: {exc}")
    for md in sorted(root.rglob("*.md")):
        if ".venv" in md.parts:
            continue
        for match in LINK.finditer(md.read_text(encoding="utf-8")):
            href = match.group(1).strip().split(" ", 1)[0].strip("<>")
            path_part = unquote(href.split("#", 1)[0].split("?", 1)[0])
            if not path_part or re.match(r"^[a-z][a-z0-9+.-]*:", path_part, re.I):
                continue
            dest = (md.parent / path_part).resolve()
            if not dest.is_relative_to(root.resolve()) or not dest.exists():
                errors.append(f"broken local link in {md.relative_to(root)}: {href}")
    # Both discoverability paths must remain complete, not merely present.
    if __package__:
        from .sync_skills import find_skill_errors
    else:
        from sync_skills import find_skill_errors
    errors.extend(find_skill_errors(root))
    return errors


def main() -> int:
    errors = find_errors()
    if errors:
        for error in errors:
            print("FAIL:", error, file=sys.stderr)
        return 1
    print("OK: required docs, portable skill parity, schema syntax, and local Markdown links")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
