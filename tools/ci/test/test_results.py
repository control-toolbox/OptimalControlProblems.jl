"""Tests of `tools/ci/results.py` — the results section of the pull-request summary.

Run: `python3 -m unittest discover -s tools/ci/test -p 'test_*.py' -v` from the
repository root. Each case writes report files as the workflow downloads them
(`<reports>/<artifact>/<environment>.md`) and checks the rendered Markdown.
"""

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools" / "ci" / "results.py"

CATALOGUE = tempfile.mkdtemp(prefix="results-catalogue-")
for name in ("beam", "robot", "rocket"):
    Path(CATALOGUE, f"{name}.jl").touch()

HEADER = "# Test report: JuMP\n\n| test | problem | modelings | status | outcome |\n| --- | --- | --- | --- | --- |\n"


def run_results(reports=None, summary=None):
    """Write `reports` = {relative path: table rows}; return (stdout, summary text)."""
    directory = Path(tempfile.mkdtemp(prefix="results-reports-"))
    for relative, rows in (reports or {}).items():
        path = directory / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(HEADER + rows)
    target = Path(tempfile.mkdtemp()) / "summary.md"
    command = [sys.executable, str(SCRIPT), "--reports",
               str(directory if reports is not None else directory / "missing"),
               "--catalogue", CATALOGUE, "--summary", str(target)]
    out = subprocess.run(command, capture_output=True, text=True, check=True)
    return out.stdout, target.read_text()


class TestResults(unittest.TestCase):
    def test_narrowed_run_lists_the_untested(self):
        out, written = run_results({
            "ci-1.13-ubuntu-latest/JuMP.md": "| visited | beam | JuMP | valid | see the test summary |\n",
        })
        self.assertIn("| ci-1.13-ubuntu-latest | `beam` | JuMP | valid | tested |", out)
        self.assertIn("Not tested in this run (2)", out)
        self.assertIn("`robot`, `rocket`", out)
        self.assertEqual(out.strip(), written.strip())

    def test_complete_run_has_no_untested_block(self):
        rows = "".join(f"| visited | {p} | JuMP | valid | see the test summary |\n"
                       for p in ("beam", "robot", "rocket"))
        out, _ = run_results({"ci-1.13-ubuntu-latest/JuMP.md": rows})
        self.assertNotIn("Not tested", out)

    def test_draft_keeps_its_outcome(self):
        out, _ = run_results({
            "ci-1.13-ubuntu-latest/JuMP.md": "| visited | robot | JuMP | draft | fail |\n",
        })
        self.assertIn("| `robot` | JuMP | draft | fail |", out)

    def test_modelings_and_platforms_stay_apart(self):
        out, _ = run_results({
            "ci-a/JuMP.md": "| visited | beam | JuMP | valid | see the test summary |\n",
            "ci-a/OptimalControl.md": "| visited | beam | OptimalControl | valid | see the test summary |\n",
            "ci-b/JuMP.md": "| visited | beam | JuMP | valid | see the test summary |\n",
        })
        self.assertIn("| ci-a | `beam` | JuMP, OptimalControl | valid | tested |", out)
        self.assertIn("| ci-b | `beam` | JuMP | valid | tested |", out)

    def test_duplicate_rows_are_merged(self):
        row = "| visited | beam | JuMP | valid | see the test summary |\n"
        out, _ = run_results({"ci-a/JuMP.md": row * 3})
        self.assertEqual(1, out.count("`beam`"))

    def test_no_report_at_all(self):
        out, _ = run_results({})
        self.assertIn("No result to show", out)
        self.assertIn("Not tested in this run (3)", out)

    def test_missing_reports_directory(self):
        out, _ = run_results(None)
        self.assertIn("the reports directory is missing", out)


if __name__ == "__main__":
    unittest.main()
