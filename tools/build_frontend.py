#!/usr/bin/env python3
"""Build DBB's dependency-free, self-contained frontend deterministically."""

from __future__ import annotations

import argparse
import re
import shutil
from pathlib import Path


LINK_PATTERN = re.compile(
    r'<link\s+inline\s+href=(["\'])([^"\']+)\1\s*/?>', re.IGNORECASE
)
SCRIPT_PATTERN = re.compile(
    r'<script\s+inline\s+src=(["\'])([^"\']+)\1\s*>\s*</script>',
    re.IGNORECASE,
)
REMAINING_INLINE = re.compile(r'<(?:link|script)\b[^>]*\binline\b', re.IGNORECASE)


def confined_source(root: Path, relative_name: str, suffix: str) -> Path:
    source = (root / relative_name).resolve()
    if not source.is_relative_to(root) or source.suffix.lower() != suffix:
        raise ValueError(f"refusing invalid inline source: {relative_name}")
    if not source.is_file():
        raise FileNotFoundError(f"inline source not found: {relative_name}")
    return source


def build(source_root: Path, output_root: Path) -> None:
    source_root = source_root.resolve()
    output_root = output_root.resolve()
    if output_root == source_root or source_root not in output_root.parents:
        raise ValueError("output must be a child of the frontend source directory")

    pages = sorted(source_root.glob("*.html"))
    if not any(page.name == "index.html" for page in pages):
        raise FileNotFoundError("frontend index.html not found")

    built: dict[str, str] = {}
    for page in pages:
        built[page.name] = inline_page(source_root, page)

    if output_root.exists():
        shutil.rmtree(output_root)
    output_root.mkdir(parents=True)
    for name, text in built.items():
        (output_root / name).write_text(text, encoding="utf-8", newline="\n")
    for pattern in ("*.png", "*.ico"):
        for asset in sorted(source_root.glob(pattern), key=lambda item: item.name):
            shutil.copyfile(asset, output_root / asset.name)


def inline_page(source_root: Path, html_path: Path) -> str:
    """Inline every <link inline>/<script inline> in one page.

    Every page is self-contained: the ESP serves each asset independently and
    there is no shared cache to rely on, so a page that referenced an external
    stylesheet would render unstyled.
    """
    html = html_path.read_text(encoding="utf-8")
    link_count = 0
    script_count = 0

    def inline_css(match: re.Match[str]) -> str:
        nonlocal link_count
        link_count += 1
        source = confined_source(source_root, match.group(2), ".css")
        return "<style>\n" + source.read_text(encoding="utf-8") + "\n</style>"

    def inline_js(match: re.Match[str]) -> str:
        nonlocal script_count
        script_count += 1
        source = confined_source(source_root, match.group(2), ".js")
        return "<script>\n" + source.read_text(encoding="utf-8") + "\n</script>"

    html = LINK_PATTERN.sub(inline_css, html)
    html = SCRIPT_PATTERN.sub(inline_js, html)
    # Counts are page-specific now, so assert the invariant that actually matters
    # - nothing left unresolved - rather than fixed per-page totals.
    if link_count < 1 or script_count < 1 or REMAINING_INLINE.search(html):
        raise ValueError(
            f"{html_path.name}: unresolved inline assets "
            f"(css={link_count}, js={script_count})"
        )
    return html


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    build(args.input, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
