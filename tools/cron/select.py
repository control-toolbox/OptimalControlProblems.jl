"""The problems a rotating cron runs, in order (#435).

A cron with a time budget runs the problems of its modeling one per process, while the budget
lasts. This script gives their order: the problems tested longest ago first, by the date of
their last result for the scope on the `results` branch (`problems/<problem>/result.json`,
written by `report.py`). A problem with no result comes first; ties are broken by name. A run
that crashed wrote no result, so its problems stay first for the next run.

Usage:
    python3 tools/cron/select.py --scope ModelingToolkit \
        --models-dir ext/ModelingToolkitModels --metadata-dir ext/MetaData \
        [--results-dir <checkout of the results branch>]

Prints the problems, comma-separated, ready for `problem=`.
"""
from __future__ import annotations

import argparse
import json
import pathlib


def problems_of(models_dir: pathlib.Path, metadata_dir: pathlib.Path) -> list[str]:
    """The problems of a modeling: a file `<problem>.jl` in its directory (the single source
    of truth, `ext/README.md`) for a problem of the collection (a metadata file)."""
    collection = {path.stem for path in metadata_dir.glob("*.jl")}
    return sorted(path.stem for path in models_dir.glob("*.jl") if path.stem in collection)


def last_date(results_dir: pathlib.Path | None, problem: str, scope: str) -> str:
    """The date of the last result of `problem` for `scope` (ISO 8601), "" if none."""
    if results_dir is None:
        return ""
    path = results_dir / "problems" / problem / "result.json"
    if not path.is_file():
        return ""
    entries = json.loads(path.read_text()).get("entries", [])
    return max((e.get("date", "") for e in entries if e.get("scope") == scope), default="")


def rotation(problems: list[str], dates: dict[str, str]) -> list[str]:
    """`problems` ordered for the rotation: no result first, then the oldest; by name in ties.
    ISO 8601 dates in UTC sort as strings, and "" sorts first."""
    return sorted(problems, key=lambda problem: (dates.get(problem, ""), problem))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scope", required=True)
    parser.add_argument("--models-dir", type=pathlib.Path, required=True)
    parser.add_argument("--metadata-dir", type=pathlib.Path, required=True)
    parser.add_argument("--results-dir", type=pathlib.Path)
    args = parser.parse_args()

    results_dir = args.results_dir if args.results_dir and args.results_dir.is_dir() else None
    problems = problems_of(args.models_dir, args.metadata_dir)
    dates = {problem: last_date(results_dir, problem, args.scope) for problem in problems}
    print(",".join(rotation(problems, dates)))


if __name__ == "__main__":
    main()
