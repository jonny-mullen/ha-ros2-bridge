# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Require the two SPDX header lines at the top of every .py and .sh file.

Expected header (a shebang on line 1 is allowed and skipped):

    # SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
    # SPDX-License-Identifier: Apache-2.0

Usage:

    python scripts/check_spdx.py            walk the repository (parent of scripts/)
    python scripts/check_spdx.py --root DIR walk DIR instead
    python scripts/check_spdx.py PATH ...   check only the given files

The walk skips .git, .venv, build, install, log, node_modules and cache directories.
Every offender is printed with its reason. Exit status is 1 when there is at least one
offender, otherwise 0. This script is the single licence-header gate: ament_copyright is
not used because it cannot read SPDX headers (recorded in docs/decisions.md).
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from pathlib import Path

CHECKED_SUFFIXES = frozenset({".py", ".sh"})
SKIP_DIRS = frozenset(
    {
        ".git",
        ".venv",
        "venv",
        "build",
        "install",
        "log",
        "node_modules",
        "__pycache__",
        ".mypy_cache",
        ".pytest_cache",
        ".ruff_cache",
    }
)
COPYRIGHT_RE = re.compile(
    r"^# SPDX-FileCopyrightText: \d{4}(?:-\d{4})? ha-ros2-bridge contributors$"
)
LICENSE_LINE = "# SPDX-License-Identifier: Apache-2.0"


def check_file(path: Path) -> str | None:
    """Return the reason a file fails the header check, or None when it passes."""
    try:
        text = path.read_text(encoding="utf-8-sig")
    except UnicodeDecodeError:
        return "not valid UTF-8"
    lines = text.splitlines()
    if lines and lines[0].startswith("#!"):
        lines = lines[1:]
    if len(lines) < 2:
        return "shorter than the two SPDX header lines"
    if not COPYRIGHT_RE.match(lines[0]):
        return "first header line is not the SPDX-FileCopyrightText line"
    if lines[1] != LICENSE_LINE:
        return "second header line is not the SPDX-License-Identifier line"
    return None


def walk(root: Path) -> list[Path]:
    """Collect every .py and .sh file under root, skipping SKIP_DIRS."""
    found: list[Path] = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = sorted(d for d in dirnames if d not in SKIP_DIRS)
        for name in sorted(filenames):
            if Path(name).suffix in CHECKED_SUFFIXES:
                found.append(Path(dirpath) / name)
    return found


def main(argv: list[str] | None = None) -> int:
    """Run the check and return the process exit status."""
    parser = argparse.ArgumentParser(description="Check SPDX headers on .py and .sh files.")
    parser.add_argument(
        "paths", nargs="*", type=Path, help="files to check (default: walk --root)"
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parent.parent,
        help="directory to walk when no paths are given (default: the repository root)",
    )
    args = parser.parse_args(argv)

    root = args.root.resolve()
    files = [p.resolve() for p in args.paths] if args.paths else walk(root)
    offenders: list[tuple[Path, str]] = []
    for path in files:
        reason = check_file(path)
        if reason is not None:
            offenders.append((path, reason))

    for path, reason in offenders:
        try:
            shown = path.relative_to(root).as_posix()
        except ValueError:
            shown = path.as_posix()
        print(f"{shown}: {reason}")

    print(f"check_spdx: {len(files)} file(s) checked, {len(offenders)} offender(s)")
    return 1 if offenders else 0


if __name__ == "__main__":
    sys.exit(main())
