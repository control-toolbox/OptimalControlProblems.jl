#!/usr/bin/env python3
"""Render what a pull-request run tested, from the test reports it uploaded.

Each environment writes ``test/reports/<environment>.md`` (a table of
``test | problem | modelings | status | outcome``); the workflow downloads them
as ``<reports>/<artifact>/<environment>.md``. The report of a ``:valid`` problem
says ``see the test summary`` because a failure fails the job, so the table shows
what was visited and what stayed a draft, not a pass/fail per problem.

Usage:

    results.py --reports artifacts [--catalogue ext/MetaData] [--summary FILE]

The Markdown is printed on standard output and appended to ``--summary`` when
given. ``--catalogue`` is the directory of the problems; the problems of the
catalogue that no report names are listed as not tested.
"""

import argparse
import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CATALOGUE = ROOT / "ext" / "MetaData"
NOTHING_TO_SAY = "see the test summary"

# The report tables are read by the parser of the cron.
_spec = importlib.util.spec_from_file_location("cron_report", ROOT / "tools" / "cron" / "report.py")
_cron = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_cron)


def tested_rows(rows):
    """One row per (platform, problem, status): modelings and outcomes merged, in order."""
    merged = {}
    for row in rows:
        entry = merged.setdefault(
            (row["platform"], row["problem"], row["status"]), {"modelings": [], "outcomes": []}
        )
        for field, value in (("modelings", row["modelings"]), ("outcomes", row["outcome"])):
            if value not in entry[field]:
                entry[field].append(value)
    return merged


def render(rows, catalogue, reports_found=True):
    """Markdown report of the run: tested problems, then the problems no report names."""
    lines = ["## Tests run", ""]
    if not rows:
        reason = "no test report was uploaded" if reports_found else "the reports directory is missing"
        lines += [f"No result to show: {reason} (the tests did not run, or failed before reporting).", ""]
    else:
        lines += ["| Platform | Problem | Modelings | Status | Outcome |", "| --- | --- | --- | --- | --- |"]
        for (platform, problem, status), entry in sorted(tested_rows(rows).items()):
            outcome = ", ".join("tested" if o == NOTHING_TO_SAY else o for o in entry["outcomes"])
            lines.append(
                f"| {platform} | `{problem}` | {', '.join(entry['modelings'])} | {status} | {outcome} |"
            )
        lines.append("")
    seen = {row["problem"] for row in rows}
    missing = sorted(p.stem for p in Path(catalogue).glob("*.jl") if p.stem not in seen)
    if missing:
        lines += [
            f"<details><summary>Not tested in this run ({len(missing)})</summary>",
            "",
            ", ".join(f"`{p}`" for p in missing),
            "",
            "</details>",
            "",
        ]
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--reports", type=Path, required=True)
    parser.add_argument("--catalogue", type=Path, default=DEFAULT_CATALOGUE)
    parser.add_argument("--summary", type=Path, help="file the Markdown report is appended to")
    ns = parser.parse_args(argv)
    found = ns.reports.is_dir()
    rows = _cron.parse_reports(ns.reports) if found else []
    text = render(rows, ns.catalogue, found)
    print(text)
    if ns.summary:
        with ns.summary.open("a") as handle:
            handle.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
