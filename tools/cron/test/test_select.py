import importlib.util
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest


ROOT = pathlib.Path(__file__).parents[3]
SPEC = importlib.util.spec_from_file_location("cron_select", ROOT / "tools/cron/select.py")
SELECT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SELECT)


def write_result(results: pathlib.Path, problem: str, entries: list[dict]) -> None:
    folder = results / "problems" / problem
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "result.json").write_text(json.dumps({"problem": problem, "entries": entries}))


class TestSelect(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = pathlib.Path(self.tmp.name)
        self.models = root / "ext" / "MTKModels"
        self.metadata = root / "ext" / "MetaData"
        self.results = root / "results"
        for directory in (self.models, self.metadata, self.results):
            directory.mkdir(parents=True)
        for problem in ("alpha", "beta", "gamma", "delta"):
            (self.metadata / f"{problem}.jl").write_text("")
            (self.metadata / f"{problem}.md").write_text("")
        for problem in ("alpha", "beta", "gamma", "delta"):
            (self.models / f"{problem}.jl").write_text("")
        # a helper file of the modeling is not a problem
        (self.models / "helpers.jl").write_text("")

    def tearDown(self):
        self.tmp.cleanup()

    def test_problems_of_a_modeling(self):
        self.assertEqual(
            ["alpha", "beta", "delta", "gamma"],
            SELECT.problems_of(self.models, self.metadata),
        )

    def test_last_date_of_the_scope(self):
        write_result(
            self.results,
            "alpha",
            [
                {"scope": "MTK", "date": "2026-11-10T04:00:00+00:00"},
                {"scope": "MTK", "date": "2026-11-12T04:00:00+00:00"},
                # another scope does not count
                {"scope": "JuMP", "date": "2026-11-19T04:00:00+00:00"},
            ],
        )
        self.assertEqual(
            "2026-11-12T04:00:00+00:00", SELECT.last_date(self.results, "alpha", "MTK")
        )
        self.assertEqual("", SELECT.last_date(self.results, "beta", "MTK"))
        self.assertEqual("", SELECT.last_date(None, "alpha", "MTK"))

    def test_rotation_oldest_first(self):
        dates = {
            "alpha": "2026-11-12T04:00:00+00:00",
            "beta": "2026-11-10T04:00:00+00:00",
            "gamma": "",  # never tested: first
            "delta": "2026-11-12T04:00:00+00:00",  # tie with alpha: by name
        }
        self.assertEqual(
            ["gamma", "beta", "alpha", "delta"],
            SELECT.rotation(["alpha", "beta", "gamma", "delta"], dates),
        )

    def test_command_line(self):
        write_result(self.results, "alpha", [{"scope": "MTK", "date": "2026-11-12T04:00:00+00:00"}])
        write_result(self.results, "beta", [{"scope": "MTK", "date": "2026-11-11T04:00:00+00:00"}])
        out = subprocess.run(
            [
                sys.executable,
                str(ROOT / "tools/cron/select.py"),
                "--scope", "MTK",
                "--models-dir", str(self.models),
                "--metadata-dir", str(self.metadata),
                "--results-dir", str(self.results),
            ],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        self.assertEqual("delta,gamma,beta,alpha", out)
        # no results branch yet: every problem by name
        out = subprocess.run(
            [
                sys.executable,
                str(ROOT / "tools/cron/select.py"),
                "--scope", "MTK",
                "--models-dir", str(self.models),
                "--metadata-dir", str(self.metadata),
                "--results-dir", str(self.results / "missing"),
            ],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        self.assertEqual("alpha,beta,delta,gamma", out)


if __name__ == "__main__":
    unittest.main()
