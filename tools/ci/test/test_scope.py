"""Tests of `tools/ci/scope.py` — the pull-request test scoping of `CI.yml`.

Run: `python3 -m unittest discover -s tools/ci/test -p 'test_*.py' -v` from the
repository root. Every case feeds the script a label list and a
`status<TAB>filename` file list on standard input, and checks the printed
`test_args` and the logged reason. Narrowing to `problem=` happens only when
every changed file is an added or modified per-problem file.
"""

import json
import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools" / "ci" / "scope.py"


def run_scope(files=(), labels=(), event="pull_request", expected=None):
    """Run the script on `status, filename` pairs; return (stdout, stderr)."""
    rows = "".join(f"{status}\t{name}\n" for status, name in files)
    command = [
        sys.executable, str(SCRIPT),
        "--event", event,
        "--labels", json.dumps(None if labels is None else list(labels)),
    ]
    if expected is not None:
        command += ["--expected-files", str(expected)]
    out = subprocess.run(command, input=rows, capture_output=True, text=True)
    return out.stdout.strip(), out.stderr.strip()


def problem(paths, status="modified"):
    return [(status, path) for path in paths]


class TestScope(unittest.TestCase):
    # Narrowing cases: every changed file is a per-problem file.
    def test_one_problem_every_directory(self):
        out, err = run_scope(problem([
            "ext/MetaData/beam.jl",
            "ext/JuMPModels/beam.jl",
            "ext/Descriptions/beam.md",
        ]))
        self.assertEqual("problem=beam", out)
        self.assertIn("changed problems: beam", err)

    def test_several_problems_sorted_and_unique(self):
        out, _ = run_scope(problem([
            "ext/MetaData/beam.jl",
            "ext/JuMPModels/beam.jl",
            "ext/Descriptions/robot.md",
        ]))
        self.assertEqual("problem=beam,robot", out)

    def test_model_directory(self):
        out, _ = run_scope(problem(["ext/OptimalControlModels/beam.jl"]))
        self.assertEqual("problem=beam", out)

    def test_draft_problem(self):
        # A draft problem is narrowed like any other: it runs as draft.
        out, _ = run_scope(problem(["ext/MetaData/mountain_car.jl"]))
        self.assertEqual("problem=mountain_car", out)

    def test_new_problem_added_everywhere(self):
        out, _ = run_scope(problem([
            "ext/MetaData/new_problem.jl",
            "ext/JuMPModels/new_problem.jl",
            "ext/Descriptions/new_problem.md",
        ], status="added"))
        self.assertEqual("problem=new_problem", out)

    # Composition with the scope labels.
    def test_scope_label_composes(self):
        out, _ = run_scope(
            problem(["ext/MetaData/beam.jl"]),
            labels=["test JuMP backend"],
        )
        self.assertEqual("JuMP problem=beam", out)

    def test_comparison_label_composes(self):
        out, _ = run_scope(
            problem(["ext/MetaData/beam.jl"]),
            labels=["test comparison"],
        )
        self.assertEqual("comparison problem=beam", out)

    def test_labels_without_narrowing(self):
        out, _ = run_scope(
            problem(["src/OptimalControlProblems.jl"]),
            labels=["test JuMP backend", "test comparison"],
        )
        self.assertEqual("JuMP comparison", out)

    # Full-catalogue cases.
    def test_extension_entry_point_is_shared(self):
        out, err = run_scope(problem(["ext/JuMPModels.jl"]))
        self.assertEqual("", out)
        self.assertIn("shared change", err)

    def test_unexpected_name_under_ext(self):
        out, _ = run_scope(problem(["ext/helper.jl"]))
        self.assertEqual("", out)

    def test_shared_file(self):
        out, _ = run_scope(problem(["ext/MetaData/beam.jl", "README.md"]))
        self.assertEqual("", out)

    def test_removed_file(self):
        out, err = run_scope(problem(["ext/MetaData/beam.jl"], status="removed"))
        self.assertEqual("", out)
        self.assertIn("removed or renamed", err)

    def test_renamed_file(self):
        out, _ = run_scope(problem([
            "ext/MetaData/beam.jl",
            "ext/MetaData/beam2.jl",
        ], status="renamed"))
        self.assertEqual("", out)

    def test_no_file(self):
        out, err = run_scope([])
        self.assertEqual("", out)
        self.assertIn("no changed file", err)

    def test_truncated_file_list(self):
        out, err = run_scope(problem(["ext/MetaData/beam.jl"]), expected=2)
        self.assertEqual("", out)
        self.assertIn("truncated", err)

    def test_override_label(self):
        out, err = run_scope(
            problem(["ext/MetaData/beam.jl"]),
            labels=["test all problems", "test JuMP backend"],
        )
        self.assertEqual("JuMP", out)
        self.assertIn("override label", err)

    def test_release_label(self):
        out, err = run_scope(
            problem(["ext/MetaData/beam.jl"]),
            labels=["release"],
        )
        self.assertEqual("", out)
        self.assertIn("release label", err)

    def test_push_event(self):
        out, err = run_scope(problem(["ext/MetaData/beam.jl"]), event="push")
        self.assertEqual("", out)
        self.assertIn("push event", err)

    def test_null_labels(self):
        out, _ = run_scope(problem(["ext/MetaData/beam.jl"]), labels=None)
        self.assertEqual("problem=beam", out)


if __name__ == "__main__":
    unittest.main()
