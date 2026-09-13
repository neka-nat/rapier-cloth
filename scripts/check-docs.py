#!/usr/bin/env python3
"""Check public Markdown links, anchors and package documentation boundaries."""
import argparse
import re
import subprocess
from collections import Counter
from pathlib import Path
from urllib.parse import unquote, urlsplit


FENCES = re.compile(r"^(`{3,}|~{3,}).*?^\1\s*$", re.MULTILINE | re.DOTALL)
LINKS = re.compile(r"!?\[[^\]\n]*\]\(\s*(<[^>]+>|[^\s)]+)(?:\s+[^)]*)?\)")
INTERNAL_NAMES = {"progress.md", "package-design.ja.md", "implementation-plan.ja.md"}


def anchors(text):
    result = set(re.findall(r'\bid=["\']([^"\']+)["\']', text))
    counts = Counter()
    for heading in re.findall(r"^#{1,6}\s+(.+?)\s*#*\s*$", FENCES.sub("", text), re.MULTILINE):
        heading = re.sub(r"\[([^]]+)\]\([^)]*\)", r"\1", heading)
        slug = re.sub(r"[^\w\- ]", "", heading.lower()).replace(" ", "-")
        result.add(slug if not counts[slug] else f"{slug}-{counts[slug]}")
        counts[slug] += 1
    return result


def check(root):
    documents = set(root.glob("*.md"))
    for directory in ["docs", "examples", "demos", "crates"]:
        documents.update(
            path for path in (root / directory).rglob("*.md")
            if not set(path.relative_to(root).parts) & {"node_modules", "target", "dist", "test-results", ".internal"}
        )
    errors = []
    if (root / ".git").exists():
        tracked_internal = subprocess.check_output(
            ["git", "ls-files", "--", ".internal"], cwd=root, text=True
        ).strip()
        if tracked_internal:
            errors.append("Internal material must not be tracked: " + tracked_internal)
    links = 0
    for path in sorted(documents):
        text = path.read_text()
        relative = path.relative_to(root)
        if path.name in INTERNAL_NAMES or "evidence" in relative.parts:
            errors.append(f"{relative}: historical work record in public documentation")
        if ".internal" in text:
            errors.append(f"{relative}: reference to local internal material")
        for match in LINKS.finditer(FENCES.sub("", text)):
            target = unquote(match[1].strip("<>"))
            parsed = urlsplit(target)
            if parsed.scheme or parsed.netloc:
                continue
            links += 1
            dest = (path.parent / parsed.path).resolve() if parsed.path else path
            if not dest.is_relative_to(root):
                errors.append(f"{relative}: link escapes documentation root: {target}")
            elif not dest.exists():
                errors.append(f"{relative}: missing link target: {target}")
            elif parsed.fragment and dest.suffix == ".md" and parsed.fragment not in anchors(dest.read_text()):
                errors.append(f"{relative}: missing anchor: {target}")
    if errors:
        raise SystemExit("\n".join(errors))
    print(f"Checked {len(documents)} public Markdown files and {links} local links: {root}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent.parent)
    check(parser.parse_args().root.resolve())
