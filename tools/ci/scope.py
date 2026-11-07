#!/usr/bin/env python3
"""Compute the ``test_args`` of ``test/runtests.jl`` for a workflow run.

On a pull request, the run tests what the pull request changes (#405): each changed file
gives a set of test environments and a set of problems, and the run is their union. The
rules are read from the repository, not written as a list of paths:

- the environments are the directories of ``test/suite/`` with a ``Project.toml``, except
  the opt-in ones (``OPT_IN`` in ``test/runtests.jl``), which only a label runs;
- ``ext/<M>Models/<p>.jl``, a problem of a modeling: the environment ``M`` and
  ``comparison``, for the problem ``p``; another file of ``ext/<M>Models/`` (the JuMP
  schemes): the same environments, every problem;
- ``ext/<E>.jl``, the entry point of an extension: the environments whose ``Project.toml``
  holds every trigger of ``E`` (``[extensions]`` of the package's ``Project.toml``);
- ``ext/MetaData/**`` and ``ext/Descriptions/**``, a file named after a problem: every
  environment, for that problem;
- ``test/suite/<env>/**``: the environment ``env``;
- ``test/common/<f>.jl``: the environments whose tests name ``<f>.jl`` (an ``include``),
  directly or through other files of ``test/common/``; every environment if
  ``test/runtests.jl`` names it;
- ``test/cron/**``, ``test/robustness/**``, ``tools/**`` (their unit tests run in the
  ``scope`` job), ``docs/`` (built by its own workflow) and Markdown files outside
  ``ext/``: no environment, ``core`` only;
- anything else (``src/``, ``Project.toml``, ``test/runtests.jl``, the workflows...):
  everything.

Everything runs as well on a removed or renamed file, an unreadable or truncated file list,
no file at all, the ``release`` label, or an event other than a pull request: narrowing
happens only on positive evidence.

Labels only add: ``test <environment> backend`` and ``test comparison`` add an environment
(with the problems of the automatic scope), ``test all problems`` every problem,
``test everything`` every environment and problem; ``test convergence`` adds the
convergence report. ``core`` always runs.

Usage:

    scope.py --event pull_request --labels '["test JuMP backend"]' \
        --expected-files 3 < changed_files.tsv

``--labels`` is the JSON array of label names (``null`` outside pull_request events is
accepted and means no label). ``--expected-files`` is the changed-file count reported by
the event payload; when it does not match the number of rows read, the list was truncated
and everything runs. Standard input is a list of ``status<TAB>filename`` rows as returned
by the pull request files API. The arguments are printed on standard output (an empty line
runs everything); the reason of the decision is printed on standard error for the log.
``--root`` is the repository (default: the one of this script). ``--summary FILE`` appends
a Markdown report of the decision to FILE (the job summary).
"""

import argparse
import json
import re
import sys
import tomllib
from pathlib import Path

ALL_PROBLEMS_LABEL = "test all problems"
EVERYTHING_LABEL = "test everything"
# Turns the convergence report on (off by default, #360): the argument `convergence`.
CONVERGENCE_LABEL = "test convergence"
RELEASE_LABEL = "release"
NOT_ENVIRONMENTS = (ALL_PROBLEMS_LABEL, EVERYTHING_LABEL, CONVERGENCE_LABEL)

COMPARISON = "comparison"
KEPT_STATUSES = ("added", "modified")
DEFAULT_ROOT = Path(__file__).resolve().parents[2]

# Every environment, or every problem.
ALL = "all"


class Repository:
    """What the rules read from the repository at `root`."""

    def __init__(self, root):
        self.root = Path(root)
        suite = self.root / "test" / "suite"
        runtests = self.root / "test" / "runtests.jl"
        self.runtests = runtests.read_text() if runtests.is_file() else ""
        match = re.search(r"const OPT_IN\s*=\s*\(([^)]*)\)", self.runtests)
        self.opt_in = set(re.findall(r'"([^"]+)"', match.group(1))) if match else set()
        self.deps = {
            d.name: set(tomllib.loads((d / "Project.toml").read_text()).get("deps", {}))
            for d in sorted(suite.iterdir()) if (d / "Project.toml").is_file()
        } if suite.is_dir() else {}
        self.environments = [e for e in self.deps if e not in self.opt_in]
        project = self.root / "Project.toml"
        extensions = tomllib.loads(project.read_text()).get("extensions", {}) if project.is_file() else {}
        self.triggers = {
            name: [t] if isinstance(t, str) else list(t) for name, t in extensions.items()
        }
        self.problems = {p.stem for p in (self.root / "ext" / "MetaData").glob("*.jl")}
        self.common = self.root / "test" / "common"

    def names(self, path, name):
        """Whether the file `path` names the file `name` (``"name"``, an include)."""
        return f'"{name}"' in path.read_text()

    def includers(self, name):
        """The environments whose tests name `name`, directly or through test/common/, or
        ALL when test/runtests.jl does."""
        if f'"{name}"' in self.runtests:
            return ALL
        names, found, envs = {name}, True, set()
        while found:
            found = False
            for f in self.common.glob("*.jl"):
                if f.name not in names and any(self.names(f, n) for n in names):
                    names.add(f.name)
                    found = True
        if any(f'"{n}"' in self.runtests for n in names):
            return ALL
        for env in self.environments:
            for f in (self.root / "test" / "suite" / env).rglob("*.jl"):
                if any(self.names(f, n) for n in names):
                    envs.add(env)
        return envs


def environments_of_labels(labels):
    """Turn `test <environment> backend` / `test comparison` labels into environments."""
    return [
        label[len("test "):].removesuffix(" backend")
        for label in labels
        if label.startswith("test ") and label not in NOT_ENVIRONMENTS
    ]


def classify(row, repo):
    """Verdict on one `status<TAB>filename` row: `(environments, problems, kind)`.
    `environments` and `problems` are sets or ALL; `kind` says why in a few words. A file
    that forces everything gives `(ALL, ALL, kind)`."""
    everything = lambda kind: (ALL, ALL, kind)
    if len(row) != 2:
        return everything("unreadable row")
    status, filename = row
    if status not in KEPT_STATUSES:
        return everything("removed or renamed file")
    parts = filename.split("/")
    stem = parts[-1].rsplit(".", 1)[0]
    modeling = re.fullmatch(r"ext/([^/]+)Models/.+", filename)
    if modeling:
        env = modeling.group(1)
        if env not in repo.deps:
            return everything(f"modeling `{env}` without a test environment")
        envs = {env, COMPARISON} - repo.opt_in
        if len(parts) == 3 and parts[2].endswith(".jl") and stem in repo.problems:
            return envs, {stem}, f"problem `{stem}` of the modeling `{env}`"
        return envs, ALL, f"shared file of the modeling `{env}`"
    if len(parts) == 2 and parts[0] == "ext" and stem in repo.triggers:
        needed = set(repo.triggers[stem])
        envs = {e for e in repo.environments if needed <= repo.deps[e]}
        return envs, ALL, f"extension `{stem}`"
    if parts[0] == "ext" and len(parts) > 2 and parts[1] in ("MetaData", "Descriptions"):
        if stem in repo.problems:
            return ALL, {stem}, f"problem `{stem}`"
        return everything("not the file of a problem")
    if parts[:2] == ["test", "suite"] and len(parts) > 3:
        env = parts[2]
        if env in repo.environments:
            return {env}, ALL, f"tests of `{env}`"
        if env in repo.opt_in or env == "core":
            return set(), set(), f"tests of `{env}` (not run by this rule)"
    if parts[:2] == ["test", "common"] and len(parts) == 3 and filename.endswith(".jl"):
        envs = repo.includers(parts[2])
        if envs == ALL:
            return everything("shared test file, used by the dispatcher")
        return envs, ALL, "shared test file"
    if parts[:2] in (["test", "cron"], ["test", "robustness"]) or parts[0] == "tools":
        return set(), set(), "not tested on a pull request"
    if parts[0] == "docs" or (filename.endswith(".md") and parts[0] != "ext"):
        return set(), set(), "documentation"
    return everything("shared change")


def union(a, b):
    return ALL if ALL in (a, b) else a | b


def scope(rows, expected, repo):
    """`(environments, problems, reason, verdicts)` of the changed files; `verdicts` is
    `(row, environments, problems, kind)` per file. Every row is classified; the reason
    is that of the first row that forces everything."""
    if expected is not None and expected != len(rows):
        return ALL, ALL, f"file list truncated ({len(rows)} of {expected} read): running everything", []
    if not rows:
        return ALL, ALL, "no changed file: running everything", []
    verdicts = [(row, *classify(row, repo)) for row in rows]
    for row, envs, problems, kind in verdicts:
        if envs == ALL and problems == ALL:
            return ALL, ALL, f"{kind} `{row[-1]}`: running everything", verdicts
    envs, problems = set(), set()
    for _, e, p, _ in verdicts:
        envs, problems = union(envs, e), union(problems, p)
    return envs, problems, "changed files", verdicts


def test_args(event, labels, rows, expected, repo):
    """The `test_args` string, the reason of the decision and the verdict on each file."""
    verdicts = []
    if event != "pull_request":
        envs, problems, reason = ALL, ALL, f"{event} event: running everything"
    elif RELEASE_LABEL in labels:
        envs, problems, reason = ALL, ALL, "release label: running everything"
    elif EVERYTHING_LABEL in labels:
        envs, problems, reason = ALL, ALL, "`test everything` label: running everything"
    else:
        envs, problems, reason, verdicts = scope(rows, expected, repo)
        added = environments_of_labels(labels)
        if added:
            # an environment of a label alone (no changed file in it) runs every problem
            problems = ALL if not problems else problems
            envs = union(envs, set(added))
            reason += "; environments added by label: " + ", ".join(added)
        if ALL_PROBLEMS_LABEL in labels:
            problems = ALL
            reason += "; every problem (label)"
    order = repo.environments + sorted(repo.opt_in)
    if envs == ALL:
        args = []
    elif not envs:
        args = ["core"]
    else:
        args = sorted(envs, key=lambda e: (order.index(e) if e in order else len(order), e))
    if CONVERGENCE_LABEL in labels:
        args.append("convergence")
    if envs and problems != ALL and problems:
        args.append("problem=" + ",".join(sorted(problems)))
    return " ".join(args), reason, verdicts


def show(value):
    if value == ALL:
        return "all"
    return ", ".join(f"`{v}`" for v in sorted(value)) or "none"


def summary(event, args, reason, verdicts):
    """Markdown report of the decision, for the job summary of the run."""
    words = args.split()
    problems = [a[len("problem="):] for a in words if a.startswith("problem=")]
    environments = [a for a in words if not a.startswith("problem=") and a != "convergence"]
    lines = [
        "## Test scope",
        "",
        f"- **Event**: `{event}`",
        "- **Problems**: " + (", ".join(f"`{p}`" for p in ",".join(problems).split(",")) if problems else "all"),
        "- **Environments**: " + (", ".join(f"`{e}`" for e in environments) or "all"),
        f"- **Reason**: {reason}",
    ]
    if verdicts:
        lines += ["", "<details><summary>Changed files</summary>", "",
                  "| Status | File | Environments | Problems | Why |",
                  "| --- | --- | --- | --- | --- |"]
        for row, envs, probs, kind in verdicts:
            status, filename = row if len(row) == 2 else ("?", " ".join(row))
            lines.append(f"| {status} | `{filename}` | {show(envs)} | {show(probs)} | {kind} |")
        lines += ["", "</details>"]
    return "\n".join(lines) + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--event", default="pull_request")
    parser.add_argument("--labels", default="[]")
    parser.add_argument("--expected-files", default="")
    parser.add_argument("--summary", type=Path, help="file the Markdown report is appended to")
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
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
    args, reason, verdicts = test_args(ns.event, labels, rows, expected, Repository(ns.root))
    if ns.summary:
        with ns.summary.open("a") as handle:
            handle.write(summary(ns.event, args, reason, verdicts))
    print(reason, file=sys.stderr)
    print(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
