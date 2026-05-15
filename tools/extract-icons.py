#!/usr/bin/env python3
"""Extract Material Symbols icon names used by the v12 dashboard.

The output is intentionally simple so it can feed a font-subsetting pass.
It scans inline HTML and JS template strings for:

  <span class="material-symbols-rounded">icon_name</span>
"""

from __future__ import annotations

import re
import sys
from pathlib import Path


ICON_RE = re.compile(
    r'class=["\'][^"\']*material-symbols-rounded[^"\']*["\'][^>]*>\s*([a-z0-9_]+)\s*<',
    re.IGNORECASE,
)


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print("usage: extract-icons.py <dashboard.html> [out.txt]", file=sys.stderr)
        return 2
    src = Path(argv[1])
    text = src.read_text(encoding="utf-8", errors="ignore")
    icons = sorted(set(m.group(1) for m in ICON_RE.finditer(text)))
    out = "\n".join(icons) + ("\n" if icons else "")
    if len(argv) >= 3:
        Path(argv[2]).write_text(out, encoding="utf-8")
    else:
        print(out, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
