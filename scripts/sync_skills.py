#!/usr/bin/env python3
"""Check or regenerate full Claude skill mirrors from the model-neutral source.

No third-party dependencies. Never delete skill directories automatically.
"""
from __future__ import annotations

import argparse
import re
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CANONICAL = Path('.agents/skills')
CLAUDE = Path('.claude/skills')
# Preserve the six original workflows plus one challenge-specific workflow: accidental removal is a regression.
EXPECTED_SKILLS = frozenset({
    'problem-first', 'contracts-first', 'implement-and-verify',
    'root-cause-debug', 'parallel-handoff', 'evaluate-and-demo',
    'embodied-loop',
})


def find_skill_errors(root: Path = ROOT, *, repair: bool = False) -> list[str]:
    """Validate both native discovery trees; --write repairs *only* mirrors."""
    errors: list[str] = []
    source = root / CANONICAL
    mirror = root / CLAUDE
    if not source.is_dir() or not mirror.is_dir():
        return [f'missing skill directory: {p}' for p in (CANONICAL, CLAUDE)
                if not (root / p).is_dir()]

    canonical_dirs = {p.name for p in source.iterdir() if p.is_dir()}
    mirrored_dirs = {p.name for p in mirror.iterdir() if p.is_dir()}
    for name in sorted(EXPECTED_SKILLS - canonical_dirs):
        errors.append(f'missing canonical skill: {name}')
    for name in sorted(canonical_dirs - EXPECTED_SKILLS):
        errors.append(f'unregistered canonical skill (review and update the registry): {name}')
    for name in sorted(mirrored_dirs - canonical_dirs):
        errors.append(f'extra Claude skill (manual review required): {name}')

    for name in sorted(canonical_dirs):
        src = source / name / 'SKILL.md'
        dst = mirror / name / 'SKILL.md'
        if src.is_symlink() or not src.is_file():
            errors.append(f'missing or symlink canonical SKILL.md: {name}')
            continue
        body = src.read_text(encoding='utf-8')
        frontmatter = re.match(r'\A---\n(.*?)\n---\n', body, re.S)
        if not frontmatter:
            errors.append(f'invalid canonical frontmatter: {name}')
            continue
        header = frontmatter.group(1)
        match = re.search(r'^name: ([a-z][a-z0-9-]+)$', header, re.M)
        desc = re.search(r'^description: (.{20,})$', header, re.M)
        if not match or match.group(1) != name or not desc:
            errors.append(f'incorrect canonical skill name/description: {name}')
            continue
        if not body[frontmatter.end():].strip():
            errors.append(f'canonical skill has no procedure: {name}')
            continue
        if repair:
            dst.parent.mkdir(parents=True, exist_ok=True)
            if dst.is_symlink():
                errors.append(f'refusing to overwrite symlink mirror: {name}')
                continue
            if not dst.is_file() or dst.read_bytes() != src.read_bytes():
                shutil.copyfile(src, dst)
        if not dst.is_file() or dst.is_symlink():
            errors.append(f'missing or symlink Claude mirror: {name}')
        elif dst.read_bytes() != src.read_bytes():
            errors.append(f'Claude mirror drift: {name}')
        # Catch forgotten extra files/resources which need purposeful synchronization.
        extra_source = {p.relative_to(source / name).as_posix() for p in (source / name).rglob('*') if p.is_file()} - {'SKILL.md'}
        extra_mirror = {p.relative_to(mirror / name).as_posix() for p in (mirror / name).rglob('*') if p.is_file()} - {'SKILL.md'}
        if extra_source or extra_mirror:
            errors.append(f'unmanaged skill resources in {name}: source={sorted(extra_source)}, mirror={sorted(extra_mirror)}')
    return errors


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    g = parser.add_mutually_exclusive_group(required=True)
    g.add_argument('--check', action='store_true', help='fail if models would receive different skill bodies')
    g.add_argument('--write', action='store_true', help='regenerate full Claude copies from canonical skills, never delete')
    args = parser.parse_args(argv)
    errors = find_skill_errors(repair=args.write)
    if errors:
        for error in errors:
            print('FAIL:', error, file=sys.stderr)
        return 1
    print('OK: all registered skills are valid and byte-identical in both native locations')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
