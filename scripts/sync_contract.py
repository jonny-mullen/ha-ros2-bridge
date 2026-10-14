# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Keep the ROS 2 package's vendored copy of the contract schemas in step.

The ROS 2 package tests validate every published payload against a vendored
copy of ``contract/schemas/*.json`` because ``colcon test`` runs from the
installed package, not from the repository. This script copies the schemas
into ``ros2/ros2_ha_bridge/ros2_ha_bridge/schemas/`` and, with ``--check``,
reports whether the copies are byte-identical without changing anything::

    python scripts/sync_contract.py          # copy (and remove stale copies)
    python scripts/sync_contract.py --check  # exit 1 if missing or different

``contract/schemas`` is the only hand-edited location; never edit the copy.
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SOURCE_DIR = REPO_ROOT / "contract" / "schemas"
TARGET_DIR = REPO_ROOT / "ros2" / "ros2_ha_bridge" / "ros2_ha_bridge" / "schemas"
PATTERN = "*.schema.json"


def _listing(folder: Path) -> dict[str, Path]:
    if not folder.is_dir():
        return {}
    return {path.name: path for path in sorted(folder.glob(PATTERN)) if path.is_file()}


def check(source: Path, target: Path) -> int:
    """Report differences between source and target; return the failure count."""
    sources = _listing(source)
    targets = _listing(target)
    if not sources:
        print(f"FAIL  no {PATTERN} files found in {source}")
        return 1
    failures = 0
    for name, path in sources.items():
        copy = targets.get(name)
        if copy is None:
            print(f"  MISSING  {name}  (expected at {target / name})")
            failures += 1
        elif copy.read_bytes() != path.read_bytes():
            print(f"  DIFFERS  {name}")
            failures += 1
        else:
            print(f"  ok       {name}")
    for name in targets:
        if name not in sources:
            print(f"  STALE    {name}  (present in {target} but not in {source})")
            failures += 1
    if failures:
        print(
            f"RESULT: FAIL ({failures} problem(s)); "
            "run scripts/sync_contract.py to refresh the vendored copy"
        )
        return failures
    print(f"RESULT: PASS ({len(sources)} schema(s) identical)")
    return 0


def sync(source: Path, target: Path) -> int:
    """Copy every schema into the target folder and remove stale copies."""
    sources = _listing(source)
    if not sources:
        print(f"FAIL  no {PATTERN} files found in {source}")
        return 1
    target.mkdir(parents=True, exist_ok=True)
    for name, path in sources.items():
        destination = target / name
        if destination.is_file() and destination.read_bytes() == path.read_bytes():
            print(f"  unchanged  {name}")
            continue
        shutil.copyfile(path, destination)
        print(f"  copied     {name}")
    for name, path in _listing(target).items():
        if name not in sources:
            path.unlink()
            print(f"  removed    {name}  (no longer in {source})")
    print(f"RESULT: synced {len(sources)} schema(s) into {target}")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--check",
        action="store_true",
        help="compare only; exit non-zero when a copy is missing or differs",
    )
    parser.add_argument("--source", type=Path, default=SOURCE_DIR)
    parser.add_argument("--target", type=Path, default=TARGET_DIR)
    args = parser.parse_args(argv)
    source = args.source.resolve()
    target = args.target.resolve()
    if args.check:
        return 1 if check(source, target) else 0
    return sync(source, target)


if __name__ == "__main__":
    sys.exit(main())
