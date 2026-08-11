#!/usr/bin/env python3
"""Generate a deterministic C lookup table for firmware-embedded web assets."""

from __future__ import annotations

import argparse
import mimetypes
from pathlib import Path


MIME_OVERRIDES = {
    ".ico": "image/x-icon",
    ".js": "application/javascript",
    ".svg": "image/svg+xml",
}


def c_bytes(data: bytes) -> str:
    if not data:
        return "    0x00"
    rows = []
    for offset in range(0, len(data), 12):
        rows.append("    " + ", ".join(f"0x{byte:02x}" for byte in data[offset:offset + 12]))
    return ",\n".join(rows)


def mime_type(path: Path) -> str:
    return MIME_OVERRIDES.get(path.suffix.lower(), mimetypes.guess_type(path.name)[0] or "application/octet-stream")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output-c", required=True, type=Path)
    parser.add_argument("--output-h", required=True, type=Path)
    args = parser.parse_args()

    assets = sorted(path for path in args.input.rglob("*") if path.is_file())
    if not assets:
        raise SystemExit(f"no web assets found under {args.input}")

    args.output_c.parent.mkdir(parents=True, exist_ok=True)
    args.output_h.parent.mkdir(parents=True, exist_ok=True)

    header = """#pragma once
#include <stddef.h>
#include <stdint.h>

typedef struct {
    const char *path;
    const char *content_type;
    const uint8_t *data;
    size_t size;
} dbb_web_asset_t;

const dbb_web_asset_t *dbb_web_asset_find(const char *path);
size_t dbb_web_asset_count(void);
"""

    source = [
        '#include "dbb_web_assets.h"',
        "",
        "#include <string.h>",
        "",
    ]
    entries = []
    for index, path in enumerate(assets):
        data = path.read_bytes()
        uri = "/" + path.relative_to(args.input).as_posix()
        symbol = f"dbb_web_asset_{index}"
        source.extend([
            f"static const uint8_t {symbol}[] = {{",
            c_bytes(data),
            "};",
            "",
        ])
        entries.append((uri, mime_type(path), symbol, len(data)))

    source.append("static const dbb_web_asset_t DBB_WEB_ASSETS[] = {")
    for uri, content_type, symbol, size in entries:
        source.append(f'    {{"{uri}", "{content_type}", {symbol}, {size}U}},')
    source.extend([
        "};",
        "",
        "const dbb_web_asset_t *dbb_web_asset_find(const char *path) {",
        "    if (path == NULL) {",
        "        return NULL;",
        "    }",
        "    for (size_t i = 0; i < dbb_web_asset_count(); ++i) {",
        "        if (strcmp(DBB_WEB_ASSETS[i].path, path) == 0) {",
        "            return &DBB_WEB_ASSETS[i];",
        "        }",
        "    }",
        "    return NULL;",
        "}",
        "",
        "size_t dbb_web_asset_count(void) {",
        "    return sizeof(DBB_WEB_ASSETS) / sizeof(DBB_WEB_ASSETS[0]);",
        "}",
        "",
    ])

    args.output_h.write_text(header, encoding="utf-8", newline="\n")
    args.output_c.write_text("\n".join(source), encoding="utf-8", newline="\n")


if __name__ == "__main__":
    main()
