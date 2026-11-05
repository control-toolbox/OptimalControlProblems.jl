#!/usr/bin/env python3
"""Compute the ``test_args`` of ``test/runtests.jl`` for a workflow run.

The scope labels of a pull request (``test <environment> backend``,
``test comparison``) select the test environments. When every file a pull
request changes is a per-problem file, the problems those files name narrow the
run further through the ``problem=a,b`` argument. Anything else — a shared file,
a removed or renamed file, a truncated file list, no file at all, a release or
the ``test all problems`` override — leaves the run complete: narrowing happens
only on positive evidence.

Usage:

    scope.py --event pull_request --labels '["test JuMP backend"]' \
        --expected-files 3 < changed_files.tsv

``--labels`` is the JSON array of label names (``null`` outside pull_request
events is accepted and means no label). ``--expected-files`` is the changed-file
count reported by the event payload; when it does not match the number of rows
read, the list was truncated and the run stays complete. Standard input is a
list of ``status<TAB>filename`` rows as returned by the pull request files API.
The arguments are printed on standard output (an empty line when nothing is
selected); ``--catalogue`` is the directory of the problems (default ``ext/MetaData``); the reason of the decision is printed on standard error for the log.
"""

import argparse
import json
import re
import sys
from pathlib import Path

OVERRIDE_LABEL = "test all problems"
RELEASE_LABEL = "release"

# Per-problem files: one file per problem in each per-problem directory. The
# problem is the file stem. Every other path (extension entry points such as
# `ext/JuMPModels.jl`, `src/`, `test/`, the workflows) is a shared change.
PER_PROBLEM = re.compile(
    r"^ext/MetaData/[^/]+\.jl$"
    r"|^ext/Descriptions/[^/]+\.md$"
    r"|^ext/[^/]+Models/[^/]+\.jl$"
)
KEPT_STATUSES = ("added", "modified")

# The problems of the collection: one `<problem>.jl` per problem in `ext/MetaData`. A file
# of a per-problem directory whose stem is not one (a helper such as
# `ext/JuMPModels/schemes.jl`) is a shared file.
DEFAULT_CATALOGUE = Path(__file__).resolve().parents[2] / "ext" / "MetaData"


def environments(labels):
    """Turn `test <environment> backend` / `test comparison` labels into arguments."""
    return [
        label[len("test "):].removesuffix(" backend")
        for label in labels
        if label.startswith("test ") and label != OVERRIDE_LABEL
    ]


def changed_problems(rows, expected, catalogue):
    """Problems named by the changed files, or a reason to run the complete
    catalogue. `rows` are `status<TAB>filename` pairs; `catalogue` is the directory
    of the problems (`ext/MetaData`)."""
    if expected is not None and expected != len(rows):
        return None, f"file list truncated ({len(rows)} of {expected} read): running everything"
    if not rows:
        return None, "no changed file: running everything"
    problems = []
    for row in rows:
        if len(row) != 2:
            return None, f"unreadable file row {row!r}: running everything"
        status, filename = row
        if status not in KEPT_STATUSES:
            return None, f"removed or renamed file `{filename}`: running everything"
        if not PER_PROBLEM.match(filename):
            return None, f"shared change `{filename}`: running everything"
        name = filename.rsplit("/", 1)[-1].rsplit(".", 1)[0]
        if not (catalogue / f"{name}.jl").is_file():
            return None, f"`{filename}` is not the file of a problem: running everything"
        problems.append(name)
    problems = sorted(set(problems))
    return problems, f"changed problems: {','.join(problems)}"


def test_args(event, labels, rows, expected, catalogue=DEFAULT_CATALOGUE):
    """The `test_args` string and the reason of the decision."""
    args = environments(labels)
    if event != "pull_request":
        reason = f"{event} event: running everything"
    elif RELEASE_LABEL in labels:
        reason = "release label: running everything"
    elif OVERRIDE_LABEL in labels:
        reason = "override label: running everything"
    else:
        problems, reason = changed_problems(rows, expected, catalogue)
        if problems:
            args.append("problem=" + ",".join(problems))
    return " ".join(args), reason


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--event", default="pull_request")
    parser.add_argument("--labels", default="[]")
    parser.add_argument("--expected-files", default="")
    parser.add_argument("--catalogue", type=Path, default=DEFAULT_CATALOGUE)
    ns = parser.parse_args(argv)
    try:
        labels = json.loads(ns.labels) or []
    except json.JSONDecodeError:
        print(f"::warning::unreadable label list {ns.labels!r}: no label applied", file=sys.stderr)
        labels = []
    rows = [
        tuple(line.rstrip("\n").split("\t", 1))
        for line in sys.stdin
        if line.strip()
    ]
    try:
        expected = int(ns.expected_files) if ns.expected_files else None
    except ValueError:
        expected = None
    args, reason = test_args(ns.event, labels, rows, expected, ns.catalogue)
    print(reason, file=sys.stderr)
    print(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
