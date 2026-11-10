"""Tests of `tools/ci/scope.py` — the pull-request test scoping of `CI.yml` (#327, #405).

Run: `python3 -m unittest discover -s tools/ci/test -p 'test_*.py' -v` from the
repository root. Every case feeds the script a label list and a
`status<TAB>filename` file list on standard input, on a fixture repository (below), and
checks the printed `test_args` and the logged reason.
"""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools" / "ci" / "scope.py"


# The fixture repository: the layout the rules read, as in this repository.
def write(path, text=""):
    file = Path(REPO, path)
    file.parent.mkdir(parents=True, exist_ok=True)
    file.write_text(text)


def deps(*names):
    return "[deps]\n" + "".join(f'{n} = "uuid"\n' for n in names)


REPO = tempfile.mkdtemp(prefix="scope-repo-")
write("Project.toml", '[extensions]\nJuMPModels = "JuMP"\nOptimalControlModels = "OptimalControl"\n')
write("test/runtests.jl", 'const OPT_IN = ("ModelingToolkit", "GPU")\n'
      'include(joinpath(@__DIR__, "common", "options.jl"))\n'
      'script = joinpath(@__DIR__, "common", "run_environment.jl")\n')
write("test/suite/JuMP/Project.toml", deps("JuMP", "Test"))
write("test/suite/OptimalControl/Project.toml", deps("OptimalControl", "Test"))
write("test/suite/OptimalControlScalarized/Project.toml", deps("OptimalControl", "Test"))
write("test/suite/comparison/Project.toml", deps("JuMP", "OptimalControl", "Test"))
write("test/suite/GPU/Project.toml", deps("OptimalControl", "CUDA"))
write("test/suite/core/test_a.jl")
write("test/suite/JuMP/test_smoke.jl", 'include(joinpath(@__DIR__, "..", "..", "common", "solve_jump.jl"))\n')
write("test/suite/JuMP/test_methods.jl", 'include(joinpath(@__DIR__, "..", "..", "common", "methods.jl"))\n')
write("test/suite/OptimalControl/test_methods.jl", 'include(joinpath(@__DIR__, "..", "..", "common", "methods.jl"))\n')
write("test/suite/OptimalControl/test_smoke.jl", 'include(joinpath(@__DIR__, "..", "..", "common", "optimalcontrol_tests.jl"))\n')
write("test/suite/OptimalControlScalarized/test_smoke.jl", 'include(joinpath(@__DIR__, "..", "..", "common", "optimalcontrol_tests.jl"))\n')
write("test/suite/comparison/test_init.jl", 'include(joinpath(@__DIR__, "..", "..", "common", "comparison.jl"))\n')
write("test/suite/GPU/test_contract.jl", 'include(joinpath(@__DIR__, "..", "..", "common", "capabilities.jl"))\n')
write("test/common/comparison.jl", 'include(joinpath(@__DIR__, "solve_jump.jl"))\n'
      'include(joinpath(@__DIR__, "solve_optimalcontrol.jl"))\n')
write("test/common/optimalcontrol_tests.jl", 'include(joinpath(@__DIR__, "solve_optimalcontrol.jl"))\n')
write("test/common/run_environment.jl", 'include(joinpath(@__DIR__, "status.jl"))\n')
for name in ("solve_jump", "solve_optimalcontrol", "methods", "capabilities", "options",
             "status", "unused"):
    if not Path(REPO, "test/common", f"{name}.jl").exists():
        write(f"test/common/{name}.jl")
for name in ("beam", "beam2", "mountain_car", "robot"):
    write(f"ext/MetaData/{name}.jl")


def command(labels, event, expected, extra=()):
    c = [sys.executable, str(SCRIPT), "--event", event, "--root", REPO,
         "--labels", json.dumps(None if labels is None else list(labels)), *extra]
    if expected is not None:
        c += ["--expected-files", str(expected)]
    return c


def run_scope(files=(), labels=(), event="pull_request", expected=None):
    """Run the script on `status, filename` pairs; return (stdout, stderr)."""
    rows = "".join("\t".join(row) + "\n" for row in files)
    out = subprocess.run(command(labels, event, expected), input=rows, capture_output=True, text=True)
    return out.stdout.strip(), out.stderr.strip()


def run_summary(files=(), labels=(), event="pull_request", expected=None):
    """Run the script with `--summary`; return the Markdown it wrote."""
    rows = "".join("\t".join(row) + "\n" for row in files)
    with tempfile.NamedTemporaryFile("r", suffix=".md") as report:
        c = command(labels, event, expected, ["--summary", report.name])
        subprocess.run(c, input=rows, capture_output=True, text=True, check=True)
        return report.read()


def changed(paths, status="modified"):
    return [(status, path) for path in paths]


class TestRules(unittest.TestCase):
    """One case per rule of the table of #405."""

    def scope(self, *paths):
        return run_scope(changed(paths))[0]

    def test_problem_of_a_modeling(self):
        self.assertEqual("JuMP comparison problem=beam", self.scope("ext/JuMPModels/beam.jl"))
        self.assertEqual("OptimalControlScalarized comparison problem=robot",
                         self.scope("ext/OptimalControlScalarizedModels/robot.jl"))

    def test_shared_file_of_a_modeling(self):
        self.assertEqual("JuMP comparison", self.scope("ext/JuMPModels/schemes/build.jl"))
        # a helper next to the problems is not a problem (#327: `problem=schemes`)
        self.assertEqual("JuMP comparison", self.scope("ext/JuMPModels/schemes.jl"))

    def test_modeling_without_environment(self):
        out, err = run_scope(changed(["ext/CasADiModels/beam.jl"]))
        self.assertEqual("", out)
        self.assertIn("without a test environment", err)

    def test_extension_entry_point(self):
        # the environments that load every trigger of the extension, opt-in ones excepted
        self.assertEqual("OptimalControl OptimalControlScalarized comparison",
                         self.scope("ext/OptimalControlModels.jl"))
        self.assertEqual("JuMP comparison", self.scope("ext/JuMPModels.jl"))

    def test_metadata_and_description(self):
        self.assertEqual("problem=beam", self.scope("ext/MetaData/beam.jl"))
        self.assertEqual("problem=robot", self.scope("ext/Descriptions/robot.md"))
        self.assertEqual("problem=beam", self.scope("ext/MetaData/references/beam.toml"))

    def test_metadata_file_not_a_problem(self):
        out, err = run_scope(changed(["ext/MetaData/helper.jl"]))
        self.assertEqual("", out)
        self.assertIn("not the file of a problem", err)

    def test_tests_of_an_environment(self):
        self.assertEqual("JuMP", self.scope("test/suite/JuMP/test_smoke.jl"))
        self.assertEqual("comparison", self.scope("test/suite/comparison/Project.toml"))

    def test_tests_of_core_or_opt_in(self):
        self.assertEqual("core", self.scope("test/suite/core/test_a.jl"))
        self.assertEqual("core", self.scope("test/suite/GPU/test_contract.jl"))

    def test_shared_test_file(self):
        # included by the tests of JuMP and, through comparison.jl, of comparison
        self.assertEqual("JuMP comparison", self.scope("test/common/solve_jump.jl"))
        # solve_optimalcontrol.jl: also through the shared runner of the OptimalControl
        # modelings (#441), whose own change concerns these two environments only
        self.assertEqual(
            "OptimalControl OptimalControlScalarized comparison",
            self.scope("test/common/solve_optimalcontrol.jl"),
        )
        self.assertEqual(
            "OptimalControl OptimalControlScalarized",
            self.scope("test/common/optimalcontrol_tests.jl"),
        )
        self.assertEqual("JuMP OptimalControl", self.scope("test/common/methods.jl"))
        # included by an opt-in environment only, or by no test
        self.assertEqual("core", self.scope("test/common/capabilities.jl"))
        self.assertEqual("core", self.scope("test/common/unused.jl"))

    def test_shared_test_file_of_the_dispatcher(self):
        # named by runtests.jl, directly or through run_environment.jl
        for name in ("options", "run_environment", "status"):
            out, err = run_scope(changed([f"test/common/{name}.jl"]))
            self.assertEqual("", out, name)
            self.assertIn("used by the dispatcher", err)

    def test_not_run_on_a_pull_request(self):
        self.assertEqual("core", self.scope("test/cron/JuMP/test_convergence.jl"))
        self.assertEqual("core", self.scope("test/robustness/JuMP/test_robustness.jl"))
        self.assertEqual("core", self.scope("tools/ci/scope.py", "tools/cron/report.py"))

    def test_documentation(self):
        self.assertEqual("core", self.scope("docs/make.jl", "docs/src/index.md"))
        self.assertEqual("core", self.scope("README.md", "test/README.md"))

    def test_shared_change(self):
        for path in ("src/OptimalControlProblems.jl", "Project.toml", "test/runtests.jl",
                     ".github/workflows/CI.yml", "ext/helper.jl"):
            out, err = run_scope(changed([path]))
            self.assertEqual("", out, path)
            self.assertIn("running everything", err)

    def test_union(self):
        # one product environments × problems: slightly more than needed, accepted
        self.assertEqual("JuMP OptimalControl comparison problem=beam,robot",
                         self.scope("ext/JuMPModels/beam.jl", "ext/OptimalControlModels/robot.jl"))
        # a modeling with every problem and a problem: every problem of the modeling
        self.assertEqual("JuMP comparison",
                         self.scope("ext/JuMPModels/beam.jl", "ext/JuMPModels/schemes/build.jl"))
        # every environment for a problem, with a narrower file: every environment
        self.assertEqual("problem=beam",
                         self.scope("ext/MetaData/beam.jl", "ext/JuMPModels/beam.jl"))
        self.assertEqual("JuMP comparison problem=beam",
                         self.scope("ext/JuMPModels/beam.jl", "docs/src/index.md"))


class TestFallbacks(unittest.TestCase):
    """Everything runs unless the changed files say otherwise."""

    def test_one_shared_file_runs_everything(self):
        out, _ = run_scope(changed(["ext/MetaData/beam.jl", "src/a.jl"]))
        self.assertEqual("", out)

    def test_removed_file(self):
        out, err = run_scope(changed(["ext/MetaData/beam.jl"], status="removed"))
        self.assertEqual("", out)
        self.assertIn("removed or renamed", err)

    def test_renamed_file(self):
        out, _ = run_scope(changed(["ext/MetaData/beam.jl", "ext/MetaData/beam2.jl"], status="renamed"))
        self.assertEqual("", out)

    def test_unreadable_row(self):
        out, err = run_scope([("modified",)])
        self.assertEqual("", out)
        self.assertIn("unreadable row", err)

    def test_no_file(self):
        out, err = run_scope([])
        self.assertEqual("", out)
        self.assertIn("no changed file", err)

    def test_truncated_file_list(self):
        out, err = run_scope(changed(["ext/MetaData/beam.jl"]), expected=2)
        self.assertEqual("", out)
        self.assertIn("truncated", err)

    def test_release_label(self):
        out, err = run_scope(changed(["ext/MetaData/beam.jl"]), labels=["release", "test JuMP backend"])
        self.assertEqual("", out)
        self.assertIn("release label", err)

    def test_push_event(self):
        out, err = run_scope(changed(["ext/MetaData/beam.jl"]), event="push")
        self.assertEqual("", out)
        self.assertIn("push event", err)


class TestLabels(unittest.TestCase):
    """Labels add to the automatic scope; they never restrict it."""

    def test_environment_label_adds(self):
        out, _ = run_scope(changed(["ext/OptimalControlModels/beam.jl"]), labels=["test JuMP backend"])
        self.assertEqual("JuMP OptimalControl comparison problem=beam", out)

    def test_environment_label_never_restricts(self):
        out, _ = run_scope(changed(["src/a.jl"]), labels=["test JuMP backend", "test comparison"])
        self.assertEqual("", out)
        out, _ = run_scope(changed(["ext/MetaData/beam.jl"]), labels=["test JuMP backend"])
        self.assertEqual("problem=beam", out)

    def test_environment_label_alone_runs_every_problem(self):
        out, err = run_scope(changed(["docs/src/index.md"]), labels=["test comparison"])
        self.assertEqual("comparison", out)
        self.assertIn("added by label: comparison", err)

    def test_all_problems_label(self):
        out, err = run_scope(changed(["ext/JuMPModels/beam.jl"]), labels=["test all problems"])
        self.assertEqual("JuMP comparison", out)
        self.assertIn("every problem (label)", err)

    def test_everything_label(self):
        out, err = run_scope(changed(["docs/src/index.md"]), labels=["test everything"])
        self.assertEqual("", out)
        self.assertIn("`test everything` label", err)

    def test_convergence_label(self):
        # not an environment: the argument `convergence`, which composes with the rest
        out, _ = run_scope(changed(["ext/JuMPModels/beam.jl"]), labels=["test convergence"])
        self.assertEqual("JuMP comparison convergence problem=beam", out)
        out, _ = run_scope(changed(["src/a.jl"]), labels=["test convergence"])
        self.assertEqual("convergence", out)

    def test_opt_in_environment_by_label(self):
        out, _ = run_scope(changed(["docs/src/index.md"]), labels=["test ModelingToolkit backend"])
        self.assertEqual("ModelingToolkit", out)

    def test_null_labels(self):
        out, _ = run_scope(changed(["ext/MetaData/beam.jl"]), labels=None)
        self.assertEqual("problem=beam", out)


class TestSummary(unittest.TestCase):
    def test_narrowed(self):
        md = run_summary(changed(["ext/JuMPModels/beam.jl", "ext/Descriptions/robot.md"]))
        self.assertIn("**Problems**: `beam`, `robot`", md)
        self.assertIn("**Environments**: all", md)
        self.assertIn("| modified | `ext/JuMPModels/beam.jl` | `JuMP`, `comparison` | `beam` |", md)
        self.assertIn("| modified | `ext/Descriptions/robot.md` | all | `robot` |", md)

    def test_core_only(self):
        md = run_summary(changed(["docs/make.jl"]))
        self.assertIn("**Environments**: `core`", md)
        self.assertIn("| modified | `docs/make.jl` | none | none | documentation |", md)

    def test_shared_change_lists_every_file(self):
        md = run_summary(changed(["src/a.jl", "ext/MetaData/beam.jl"]))
        self.assertIn("**Problems**: all", md)
        self.assertIn("**Environments**: all", md)
        self.assertIn("| modified | `src/a.jl` | all | all | shared change |", md)
        self.assertIn("`ext/MetaData/beam.jl` | all | `beam`", md)

    def test_truncated_list_has_no_table(self):
        md = run_summary(changed(["ext/MetaData/beam.jl"]), expected=5)
        self.assertIn("truncated", md)
        self.assertNotIn("<details>", md)

    def test_push_event(self):
        md = run_summary(event="push")
        self.assertIn("push event: running everything", md)
        self.assertNotIn("<details>", md)

    def test_label_everything_has_no_table(self):
        md = run_summary(changed(["ext/MetaData/beam.jl"]), labels=["test everything"])
        self.assertIn("`test everything` label", md)
        self.assertNotIn("<details>", md)


if __name__ == "__main__":
    unittest.main()
