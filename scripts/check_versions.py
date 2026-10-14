# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Check that every place the project states its version agrees.

Sources, all relative to the repository root:

    custom_components/ros2_bridge/manifest.json      "version"
    custom_components/ros2_bridge/const.py           VERSION = "..."
    ros2/ros2_ha_bridge/package.xml                  <version>...</version>
    ros2/ros2_ha_bridge/setup.py                     version="..."
    ros2/ros2_ha_bridge/ros2_ha_bridge/__init__.py   __version__ = "..."
    CHANGELOG.md                                     first release heading; "## [Unreleased]" is read
                                                     for pending changes, not for a version

Usage:

    python scripts/check_versions.py                         every source must agree
    python scripts/check_versions.py --expect 0.1.0          ... and equal 0.1.0
    python scripts/check_versions.py --expect 0.1.0 --released
                                                             ... and CHANGELOG.md must have a 0.1.0 heading
    python scripts/check_versions.py --changelog-section 0.1.0
                                                             print the CHANGELOG.md body for 0.1.0

Between releases the code may run ahead of CHANGELOG.md: when the five code sources agree
on a version newer than the last release heading and "## [Unreleased]" has content, the
heading is reported as the last release and left out of the comparison, so the flag-less
check (pre-commit, contract.yml) and --expect pass while the release is being prepared. The
release workflow passes --released, which insists on a heading for the version, so a tag
without its CHANGELOG.md section still fails. A code version older than the last release, or
a newer one with nothing under Unreleased, fails as a disagreement. A CHANGELOG.md whose only
heading is "## [Unreleased]" names no version yet; that is reported but only fails with
--released. Exit status 0 when everything agrees, 1 otherwise.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

MANIFEST = Path("custom_components/ros2_bridge/manifest.json")
CONST = Path("custom_components/ros2_bridge/const.py")
PACKAGE_XML = Path("ros2/ros2_ha_bridge/package.xml")
SETUP_PY = Path("ros2/ros2_ha_bridge/setup.py")
ROS_INIT = Path("ros2/ros2_ha_bridge/ros2_ha_bridge/__init__.py")
CHANGELOG = Path("CHANGELOG.md")

SEMVER = r"\d+\.\d+\.\d+(?:[-+][0-9A-Za-z.-]+)?"
HEADING_RE = re.compile(r"^##\s+\[?v?(" + SEMVER + r")\]?(?:\s|$)")
UNRELEASED_RE = re.compile(r"^##\s+\[?unreleased\]?", re.IGNORECASE)
ANY_HEADING_RE = re.compile(r"^##\s")
LINK_REFERENCE_RE = re.compile(r"^\[[^\]]+\]:\s+\S+\s*$")
QUOTED = r"""["']([^"']+)["']"""
CONST_RE = re.compile(r"^VERSION\s*(?::\s*Final(?:\[str\])?)?\s*=\s*" + QUOTED, re.MULTILINE)
# package.xml is read as text on purpose: it is a first-party file and a regex avoids the
# XML parser entirely (no entity expansion, nothing fetched).
PACKAGE_XML_RE = re.compile(r"<version>\s*([^<\s]+)\s*</version>")
SETUP_RE = re.compile(r"\bversion\s*=\s*" + QUOTED)
INIT_RE = re.compile(r"^__version__\s*=\s*" + QUOTED, re.MULTILINE)


class SourceError(Exception):
    """A version source is missing or cannot be read."""


def _read(root: Path, rel: Path) -> str:
    path = root / rel
    if not path.is_file():
        raise SourceError("missing")
    return path.read_text(encoding="utf-8")


def _match(pattern: re.Pattern[str], text: str, what: str) -> str:
    found = pattern.search(text)
    if found is None:
        raise SourceError(f"no {what} found")
    return found.group(1)


def read_manifest(root: Path) -> str:
    """Version from manifest.json."""
    data = json.loads(_read(root, MANIFEST))
    version = data.get("version")
    if not isinstance(version, str):
        raise SourceError("no version string")
    return version


def read_const(root: Path) -> str:
    """VERSION constant from const.py."""
    return _match(CONST_RE, _read(root, CONST), "VERSION assignment")


def read_package_xml(root: Path) -> str:
    """<version> element from package.xml (read as text, not parsed as XML)."""
    return _match(PACKAGE_XML_RE, _read(root, PACKAGE_XML), "<version> element")


def read_setup_py(root: Path) -> str:
    """version keyword from setup.py (read as text, never executed)."""
    return _match(SETUP_RE, _read(root, SETUP_PY), "version= keyword")


def read_ros_init(root: Path) -> str:
    """__version__ from the ROS package's __init__.py."""
    return _match(INIT_RE, _read(root, ROS_INIT), "__version__ assignment")


def changelog_sections(text: str) -> list[tuple[str | None, list[str]]]:
    """Split CHANGELOG.md into (version or None for Unreleased, body lines) per heading."""
    sections: list[tuple[str | None, list[str]]] = []
    body: list[str] | None = None
    for line in text.splitlines():
        if ANY_HEADING_RE.match(line):
            if UNRELEASED_RE.match(line):
                body = []
                sections.append((None, body))
                continue
            heading = HEADING_RE.match(line)
            if heading is None:
                body = None
                continue
            body = []
            sections.append((heading.group(1), body))
            continue
        if body is not None and not LINK_REFERENCE_RE.match(line):
            body.append(line)
    return sections


def read_changelog(root: Path) -> str | None:
    """Latest release version in CHANGELOG.md, or None when only Unreleased exists."""
    for version, _body in changelog_sections(_read(root, CHANGELOG)):
        if version is not None:
            return version
    return None


def has_unreleased_changes(root: Path) -> bool:
    """True when CHANGELOG.md has an Unreleased section with something written under it."""
    return any(
        version is None and any(line.strip() for line in body)
        for version, body in changelog_sections(_read(root, CHANGELOG))
    )


def release_key(version: str) -> tuple[int, int, int]:
    """The numeric triple of a version for ordering (a pre-release suffix is ignored)."""
    major, minor, patch = (
        int(part) for part in version.split("-", 1)[0].split("+", 1)[0].split(".")
    )
    return (major, minor, patch)


def changelog_section(root: Path, version: str) -> str | None:
    """Body of the CHANGELOG.md section for version, or None when absent."""
    for found, body in changelog_sections(_read(root, CHANGELOG)):
        if found == version:
            lines = list(body)
            while lines and not lines[0].strip():
                lines.pop(0)
            while lines and not lines[-1].strip():
                lines.pop()
            return "\n".join(lines) + "\n"
    return None


def main(argv: list[str] | None = None) -> int:
    """Run the check and return the process exit status."""
    parser = argparse.ArgumentParser(description="Check that all version strings agree.")
    parser.add_argument("--expect", metavar="X.Y.Z", help="the version every source must equal")
    parser.add_argument(
        "--released",
        action="store_true",
        help="also require CHANGELOG.md to carry a release heading for the version",
    )
    parser.add_argument(
        "--changelog-section",
        metavar="X.Y.Z",
        help="print the CHANGELOG.md body for this version and exit (1 when absent)",
    )
    parser.add_argument("--root", type=Path, default=ROOT, help="repository root")
    args = parser.parse_args(argv)
    root: Path = args.root.resolve()

    if args.changelog_section:
        try:
            body = changelog_section(root, args.changelog_section)
        except SourceError as err:
            print(f"{CHANGELOG.as_posix()}: {err}", file=sys.stderr)
            return 1
        if body is None:
            print(
                f"{CHANGELOG.as_posix()}: no section for {args.changelog_section}", file=sys.stderr
            )
            return 1
        sys.stdout.write(body)
        return 0

    readers = (
        (MANIFEST, read_manifest),
        (CONST, read_const),
        (PACKAGE_XML, read_package_xml),
        (SETUP_PY, read_setup_py),
        (ROS_INIT, read_ros_init),
    )
    versions: dict[str, str] = {}
    problems: list[str] = []
    for rel, reader in readers:
        try:
            versions[rel.as_posix()] = reader(root)
        except SourceError as err:
            problems.append(f"{rel.as_posix()}: {err}")

    changelog_name = CHANGELOG.as_posix()
    changelog_version: str | None = None
    changelog_missing = False
    pending: str | None = None  # the code version running ahead of the last release heading
    code_versions = sorted(set(versions.values()))
    try:
        changelog_version = read_changelog(root)
        unreleased = has_unreleased_changes(root)
    except SourceError as err:
        changelog_missing = True
        problems.append(f"{changelog_name}: {err}")
    else:
        if changelog_version is None:
            if args.released:
                problems.append(f"{changelog_name}: no release heading (only Unreleased)")
        elif (
            not args.released
            and unreleased
            and len(code_versions) == 1
            and release_key(code_versions[0]) > release_key(changelog_version)
        ):
            pending = code_versions[0]
        else:
            versions[changelog_name] = changelog_version

    width = max((len(name) for name in versions), default=len(changelog_name))
    for name, version in versions.items():
        print(f"{name:<{width}}  {version}")
    if changelog_version is None and not changelog_missing:
        print(f"{changelog_name:<{width}}  (no release heading yet, only Unreleased)")
    elif pending is not None:
        print(
            f"{changelog_name:<{width}}  (last release {changelog_version}; "
            f"{pending} pending under Unreleased)"
        )

    distinct = sorted(set(versions.values()))
    if len(distinct) > 1:
        problems.append("sources disagree: " + ", ".join(distinct))
    if args.expect and distinct != [args.expect]:
        problems.append(f"expected {args.expect}, found " + (", ".join(distinct) or "nothing"))

    if problems:
        for problem in problems:
            print(f"FAIL: {problem}")
        return 1
    print(f"OK: every source agrees on {distinct[0]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
