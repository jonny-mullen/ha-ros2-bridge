# SPDX-FileCopyrightText: 2026 ha-ros2-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Check that every row of docs/failure-modes.md names tests that exist today.

The failure table in ``docs/failure-modes.md`` has the columns Failure, Mechanism,
What HA shows and Test ids. Each Test ids cell holds one or more ids written as
``path/to/test_file.py::test_name`` (a parametrised id such as
``test_name[case]`` is accepted and reduced to ``test_name``), or the words
``no test yet``. This script:

- parses the table and extracts every ``path::test_name``;
- checks that the file exists under the repository root and contains a
  ``def test_name(`` or ``async def test_name(`` at the start of a line;
- counts the ``no test yet`` cells and requires each such row to be listed in the
  document's final "Rows without a test" section, so the gap stays visible;
- prints one status line per row and a summary.

Usage:

    python scripts/check_failure_modes.py                 check docs/failure-modes.md
    python scripts/check_failure_modes.py --root DIR      use DIR as the repository root
    python scripts/check_failure_modes.py --doc PATH      check another document

Exit status is 1 when a test id names a missing file or function, when a row has
neither a test id nor ``no test yet``, when a ``no test yet`` row is missing from
the gap section, or when the table cannot be found; otherwise 0. Rows marked
``no test yet`` are allowed (and reported) so that CI stays green while the gap is
written down. ``contract.yml`` runs this after ``check_versions.py``.
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DOC = Path("docs/failure-modes.md")

#: The table header, lower-cased, that marks the failure table.
HEADER = ("failure", "mechanism", "what ha shows", "test ids")
#: The words that mark a row with no test, case-insensitive.
NO_TEST = "no test yet"
#: The heading of the section that must list every row marked NO_TEST.
GAP_HEADING = "## Rows without a test"

TEST_ID_RE = re.compile(
    r"(?P<path>[A-Za-z0-9_][A-Za-z0-9_./-]*\.py)::(?P<name>[A-Za-z_][A-Za-z0-9_]*)"
    r"(?P<params>\[[^\]]*\])?"
)
TABLE_LINE_RE = re.compile(r"^\s*\|.*\|\s*$")
SEPARATOR_RE = re.compile(r"^\s*\|(\s*:?-+:?\s*\|)+\s*$")


@dataclass(frozen=True)
class TestId:
    """One ``path::test_name`` reference as written in the document."""

    path: str
    name: str

    def __str__(self) -> str:
        return f"{self.path}::{self.name}"


@dataclass
class Row:
    """One parsed row of the failure table."""

    line: int
    failure: str
    test_ids: tuple[TestId, ...]
    no_test: bool
    problems: list[str] = field(default_factory=list)


def split_cells(line: str) -> list[str]:
    """The cells of a Markdown table line, leading and trailing pipes removed."""
    stripped = line.strip().removeprefix("|").removesuffix("|")
    return [cell.strip() for cell in stripped.split("|")]


def plain(text: str) -> str:
    """Cell text without code spans or surrounding whitespace."""
    return text.replace("`", "").strip()


def parse_test_cell(cell: str) -> tuple[tuple[TestId, ...], bool]:
    """The test ids in a cell and whether it is marked ``no test yet``."""
    text = plain(cell)
    ids = tuple(
        TestId(match.group("path"), match.group("name")) for match in TEST_ID_RE.finditer(text)
    )
    return ids, NO_TEST in text.lower()


def parse_table(text: str) -> list[Row]:
    """Rows of the first table whose header matches :data:`HEADER`; empty when absent."""
    lines = text.splitlines()
    rows: list[Row] = []
    index = 0
    while index < len(lines):
        line = lines[index]
        if TABLE_LINE_RE.match(line):
            cells = tuple(plain(cell).lower() for cell in split_cells(line))
            if cells == HEADER:
                index += 1
                if index < len(lines) and SEPARATOR_RE.match(lines[index]):
                    index += 1
                while index < len(lines) and TABLE_LINE_RE.match(lines[index]):
                    row_cells = split_cells(lines[index])
                    row = Row(index + 1, plain(row_cells[0]), (), False)
                    if len(row_cells) != len(HEADER):
                        row.problems.append(
                            f"expected {len(HEADER)} cells, found {len(row_cells)}"
                        )
                    else:
                        row.test_ids, row.no_test = parse_test_cell(row_cells[3])
                    rows.append(row)
                    index += 1
                return rows
        index += 1
    return rows


def gap_section(text: str) -> str | None:
    """The body of the "Rows without a test" section, or ``None`` when absent."""
    lines = text.splitlines()
    for number, line in enumerate(lines):
        if line.strip() == GAP_HEADING:
            body: list[str] = []
            for later in lines[number + 1 :]:
                if later.startswith("## "):
                    break
                body.append(later)
            return "\n".join(body)
    return None


def definition_pattern(name: str) -> re.Pattern[str]:
    return re.compile(rf"^[ \t]*(?:async[ \t]+)?def[ \t]+{re.escape(name)}[ \t]*\(", re.MULTILINE)


class Checker:
    """Resolves test ids against the repository, caching file contents."""

    def __init__(self, root: Path) -> None:
        self.root = root
        self._sources: dict[str, str | None] = {}

    def source(self, path: str) -> str | None:
        if path not in self._sources:
            candidate = self.root / path
            if candidate.is_file():
                self._sources[path] = candidate.read_text(encoding="utf-8")
            else:
                self._sources[path] = None
        return self._sources[path]

    def problem(self, test_id: TestId) -> str | None:
        """Why a test id does not resolve, or ``None`` when it does."""
        source = self.source(test_id.path)
        if source is None:
            return f"{test_id}: file {test_id.path} does not exist"
        if definition_pattern(test_id.name).search(source) is None:
            return f"{test_id}: no 'def {test_id.name}(' in {test_id.path}"
        return None


def check(root: Path, doc: Path) -> int:
    """Run the check and return the exit status; prints the report."""
    doc_path = doc if doc.is_absolute() else root / doc
    if not doc_path.is_file():
        print(f"FAIL: {doc.as_posix()} not found under {root}")
        return 1
    text = doc_path.read_text(encoding="utf-8")
    rows = parse_table(text)
    if not rows:
        print(f"FAIL: no table with the header {' | '.join(HEADER)} found in {doc.as_posix()}")
        return 1

    checker = Checker(root)
    failures = 0
    gaps: list[Row] = []
    distinct: set[TestId] = set()
    references = 0
    for row in rows:
        for test_id in row.test_ids:
            references += 1
            distinct.add(test_id)
            problem = checker.problem(test_id)
            if problem is not None:
                row.problems.append(problem)
        if row.no_test and row.test_ids:
            row.problems.append("cell says 'no test yet' but also names a test id")
        elif not row.no_test and not row.test_ids:
            row.problems.append("no test id and not marked 'no test yet'")
        if row.problems:
            failures += 1
            print(f"FAIL  line {row.line:>4}  {row.failure}")
            for problem in row.problems:
                print(f"        {problem}")
        elif row.no_test:
            gaps.append(row)
            print(f"GAP   line {row.line:>4}  {row.failure}: no test yet")
        else:
            count = len(row.test_ids)
            print(f"OK    line {row.line:>4}  {row.failure}: {count} test id(s)")

    section = gap_section(text)
    if gaps:
        if section is None:
            failures += 1
            print(
                f"FAIL: {len(gaps)} row(s) say 'no test yet' but the document has no "
                f"'{GAP_HEADING}' section"
            )
        else:
            section_plain = plain(section)
            for row in gaps:
                if row.failure not in section_plain:
                    failures += 1
                    print(f"FAIL: row '{row.failure}' is not listed under '{GAP_HEADING}'")

    print(
        f"{len(rows)} rows, {references} test references ({len(distinct)} distinct), "
        f"{len(gaps)} row(s) without a test, {failures} failure(s)"
    )
    if failures:
        print(
            "FAIL: fix the rows above (or mark a row 'no test yet' and list it in the gap section)"
        )
        return 1
    print(f"OK: every test id in {doc.as_posix()} resolves to an existing test")
    return 0


def main(argv: list[str] | None = None) -> int:
    """Command-line entry point."""
    parser = argparse.ArgumentParser(
        description="Check that every failure-modes row names tests that exist."
    )
    parser.add_argument("--root", type=Path, default=ROOT, help="repository root")
    parser.add_argument(
        "--doc", type=Path, default=DOC, help="document to check (relative to the root)"
    )
    args = parser.parse_args(argv)
    return check(args.root.resolve(), args.doc)


if __name__ == "__main__":
    sys.exit(main())
