import importlib.util
import json
import pathlib
import tempfile
import unittest


ROOT = pathlib.Path(__file__).parents[3]
SPEC = importlib.util.spec_from_file_location("cron_report", ROOT / "tools/cron/report.py")
REPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPORT)


class FakeGitHub:
    def __init__(self, issues=None):
        self.data = issues or []
        self.comments = []
        self.added_labels = []

    def issues(self):
        return self.data

    def create_issue(self, title, body, labels):
        issue = {
            "number": len(self.data) + 1,
            "title": title,
            "body": body,
            "labels": [{"name": label} for label in labels],
            "state": "open",
        }
        self.data.append(issue)
        return issue

    def update_issue(self, number, **payload):
        issue = self.data[number - 1]
        issue.update(payload)
        if "labels" in payload:
            issue["labels"] = [{"name": label} for label in payload["labels"]]
        return issue

    def add_labels(self, number, labels):
        issue = self.data[number - 1]
        issue.setdefault("labels", [])
        issue["labels"].extend({"name": label} for label in labels)
        self.added_labels.extend(labels)

    def comment(self, number, body):
        self.comments.append((number, body))


class ReportTest(unittest.TestCase):
    def rows(self, *outcomes, modelings="JuMP", status="valid"):
        return [
            {
                "test": f"test_convergence solver={outcome}",
                "problem": "beam",
                "modelings": modelings,
                "status": status,
                "outcome": outcome,
                "platform": "cpu",
                "source": "cpu/JuMP-cron.md",
            }
            for outcome in outcomes
        ]

    def test_parser_keeps_artifact_platform(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "cron-JuMP-1.13-ubuntu-latest" / "JuMP-cron.md"
            path.parent.mkdir(parents=True)
            path.write_text(
                "# Report\n\n"
                "| test | problem | modelings | status | outcome |\n"
                "| --- | --- | --- | --- | --- |\n"
                "| test_convergence | beam | JuMP | valid | fail |\n"
            )
            rows = REPORT.parse_reports(pathlib.Path(directory))
        self.assertEqual(rows[0]["platform"], "cron-JuMP-1.13-ubuntu-latest")

    def test_findings_requires_every_draft_row_to_pass(self):
        rows = self.rows("pass", "fail", status="draft")
        self.assertEqual(REPORT.findings(rows)["draft"], [])

    def test_hidden_key_supports_modeling_names_with_spaces(self):
        row = self.rows("fail", modelings="JuMP vs OptimalControl")[0]
        key = REPORT.issue_key(row)
        issue = {"body": f"<!-- cron-key: {key} -->", "state": "open", "labels": [{"name": "cron-bug"}]}
        self.assertIs(REPORT.matching_issue([issue], key), issue)

    def test_bug_issue_is_idempotent_and_tracks_configuration_changes(self):
        github = FakeGitHub()
        state = {}
        rows = self.rows("fail")
        REPORT.update_bug_issues(
            github, github.issues(), rows, rows, "JuMP", "run-1", "date", state, False
        )
        REPORT.update_bug_issues(
            github, github.issues(), rows, rows, "JuMP", "run-2", "date", state, False
        )
        self.assertEqual(len(github.data), 1)
        self.assertEqual(github.comments, [])

        changed = self.rows("fail")
        changed[0]["test"] = "test_convergence solver=madnlp"
        REPORT.update_bug_issues(
            github, github.issues(), changed, changed, "JuMP", "run-3", "date", state, False
        )
        self.assertEqual(len(github.comments), 1)

    def test_success_closes_issue_only_after_two_observed_passes(self):
        issue = {
            "number": 1,
            "body": "<!-- cron-key: beam|JuMP -->",
            "labels": [{"name": "cron-bug"}],
            "state": "open",
        }
        github = FakeGitHub([issue])
        state = {}
        rows = self.rows("pass")
        for _ in range(2):
            REPORT.prepare_state(rows, "JuMP", False, state)
        REPORT.update_bug_issues(
            github, github.issues(), rows, [], "JuMP", "run", "date", state, False
        )
        self.assertEqual(issue["state"], "closed")
        self.assertEqual(issue["state_reason"], "completed")

    def test_missing_report_does_not_close_issue(self):
        issue = {
            "number": 1,
            "body": "<!-- cron-key: beam|JuMP -->",
            "labels": [{"name": "cron-bug"}],
            "state": "open",
        }
        github = FakeGitHub([issue])
        state = {"beam|JuMP": {"scope": "JuMP", "successes": 1, "reopenings": 0}}
        REPORT.prepare_state([], "JuMP", False, state)
        REPORT.update_bug_issues(
            github, github.issues(), [], [], "JuMP", "run", "date", state, False
        )
        self.assertEqual(issue["state"], "open")
        self.assertEqual(github.comments, [])

    def test_not_planned_issue_is_left_alone(self):
        issue = {
            "number": 1,
            "body": "<!-- cron-key: beam|JuMP -->",
            "labels": [{"name": "cron-bug"}],
            "state": "closed",
            "state_reason": "not_planned",
        }
        github = FakeGitHub([issue])
        rows = self.rows("fail")
        REPORT.update_bug_issues(
            github, github.issues(), rows, rows, "JuMP", "run", "date", {}, False
        )
        self.assertEqual(issue["state"], "closed")
        self.assertEqual(github.comments, [])

    def test_general_breakage_does_not_open_per_problem_issues(self):
        github = FakeGitHub()
        rows = []
        for index in range(6):
            row = self.rows("fail")[0]
            row["problem"] = f"problem-{index}"
            rows.append(row)
        REPORT.update_bug_issues(
            github, github.issues(), rows, rows, "JuMP", "run", "date", {}, False
        )
        self.assertEqual(github.data, [])
        self.assertEqual(len(REPORT.findings(rows)["failure"]), 6)

    def test_summary_issue_explains_what_to_do_and_keeps_its_header(self):
        github = FakeGitHub()
        rows = self.rows("pass", status="draft")
        REPORT.update_summary_issue(github, github.data, "draft", "JuMP", rows, "run-1")
        body = github.data[0]["body"]
        self.assertIn("What to do", body)
        self.assertIn(REPORT.README_URL, body)
        # an issue created by an older version (no explanation) gets the header on update
        github.data[0]["body"] = "# Automated cron report\n\n" + body[body.index("<!-- cron-scope"):]
        REPORT.update_summary_issue(github, github.data, "draft", "JuMP", rows, "run-2")
        self.assertIn("What to do", github.data[0]["body"])
        self.assertEqual(github.data[0]["body"].count("<!-- cron-scope: JuMP -->"), 1)

    def test_bug_issue_body_says_how_to_reproduce(self):
        row = self.rows("fail")[0]
        body = REPORT.bug_body(row, ["linux: t"], "run", "date", 0)
        self.assertIn('test_args=["JuMP", "cron", "problem=beam"]', body)
        self.assertIn(REPORT.README_URL, body)
        comparison = dict(row, modelings="JuMP vs OptimalControl")
        self.assertIn('"comparison", "cron"', REPORT.bug_body(comparison, ["x"], "run", "d", 0))

    def test_issue_quoting_a_key_without_the_bug_label_is_never_touched(self):
        spec = {
            "number": 1,
            "title": "specification quoting <!-- cron-key: beam|JuMP -->",
            "body": "example: <!-- cron-key: beam|JuMP -->",
            "labels": [],
            "state": "open",
        }
        github = FakeGitHub([spec])
        rows = self.rows("fail")
        state = {}
        REPORT.update_bug_issues(github, github.data, rows, rows, "JuMP", "run", "date", state, False)
        self.assertEqual(github.data[0]["body"], "example: <!-- cron-key: beam|JuMP -->")
        self.assertEqual(len(github.data), 2)  # a real bug issue was created instead
        self.assertEqual(github.comments, [])

    def test_replace_section_only_replaces_one_scope(self):
        original = (
            "# Report\n\n"
            "<!-- cron-scope: JuMP -->\n"
            "## old\n\n"
            "<!-- cron-scope: comparison -->\n"
            "## comparison\n"
        )
        replacement = REPORT.issue_section("JuMP", "failure", [], "run")
        result = REPORT.replace_section(original, "JuMP", replacement)
        self.assertIn("No findings.", result)
        self.assertIn("## comparison", result)
        self.assertNotIn("## old", result)

    def test_a_section_names_its_scope_and_links_its_runs(self):
        rows = self.rows("fail")
        section = REPORT.issue_section(
            "JuMP", "failure", rows, "https://x/actions/runs/1",
            generated="2026-11-07T04:12:00+00:00", previous_run="https://x/actions/runs/0",
            note="weekly, CPU",
        )
        self.assertIn("<!-- cron-scope: JuMP -->", section)
        self.assertIn("<!-- cron-status: 1 -->", section)
        self.assertIn("## JuMP — Nominal Convergence\n", section)
        self.assertIn("**Focus:**", section)
        self.assertIn("**Cadence & runners:** weekly, CPU", section)
        self.assertIn("**⚠️ 1 failure**", section)
        self.assertIn("Run [1](https://x/actions/runs/1)", section)
        self.assertIn("2026-11-07 04:12 UTC", section)
        self.assertIn("blob/results/cron/JuMP.md", section)
        self.assertIn("[Previous run](https://x/actions/runs/0)", section)
        self.assertIn("Pkg.test", section)  # the Investigate hints
        clean = REPORT.issue_section("JuMP", "failure", [], "run")
        self.assertIn("**✅ Clean**", clean)
        self.assertNotIn("How to investigate", clean)

    def test_a_missing_report_row_does_not_get_a_bogus_reproduce_command(self):
        section = REPORT.issue_section(
            "JuMP", "failure", [REPORT.missing_report_row("JuMP")], "run"
        )
        self.assertNotIn("problem=workflow", section)
        self.assertIn("failed or produced no report", section)

    def test_initial_issue_body_prepopulates_all_known_scopes(self):
        active = "<!-- cron-scope: JuMP -->\nACTIVE_JUMP_SECTION"
        body = REPORT.initial_issue_body("convergence", "JuMP", active)
        self.assertIn("ACTIVE_JUMP_SECTION", body)
        self.assertIn("<!-- cron-scope: JuMP-robustness -->", body)
        self.assertIn("<!-- cron-scope: OptimalControl -->", body)
        self.assertIn("## Overall Scope Status", body)
        self.assertIn("| [JuMP — Solver Robustness]", body)
        self.assertIn("*Awaiting run*", body)

    def test_the_platform_column_drops_the_scope_prefix(self):
        row = self.rows("fail")[0]
        row["platform"] = "cron-JuMP-1.13-ubuntu-latest"
        section = REPORT.issue_section("JuMP", "failure", [row], "run")
        self.assertIn("| 1.13-ubuntu-latest |", section)
        self.assertNotIn("cron-JuMP", section)

    def test_remember_run_tracks_the_previous_run_per_scope(self):
        state = {}
        self.assertEqual("", REPORT.remember_run(state, "JuMP", "run1"))
        self.assertEqual("run1", REPORT.remember_run(state, "JuMP", "run2"))
        # the issues call of the same run links the same previous run
        self.assertEqual("run1", REPORT.remember_run(state, "JuMP", "run2"))
        self.assertEqual("", REPORT.remember_run(state, "OptimalControl", "run2"))

    def test_summary_issue_gets_a_toc_and_a_counted_title(self):
        github = FakeGitHub()
        rows = self.rows("fail")
        REPORT.update_summary_issue(github, github.data, "failure", "JuMP", rows, "r1")
        REPORT.update_summary_issue(github, github.data, "failure", "OptimalControl", [], "r2")
        issue = github.data[0]
        self.assertIn("## Overall Scope Status", issue["body"])
        self.assertIn("[JuMP — Nominal Convergence]", issue["body"])
        self.assertIn("⚠️ **1 failure**", issue["body"])
        self.assertIn("[OptimalControl — Nominal Convergence]", issue["body"])
        self.assertIn("✅ Clean", issue["body"])
        self.assertEqual("[Cron] cron-failure — 1 failure", issue["title"])
        # all scopes clean again: the count leaves the title
        REPORT.update_summary_issue(github, github.data, "failure", "JuMP", [], "r3")
        self.assertEqual("[Cron] cron-failure", github.data[0]["title"])

    def test_a_change_of_findings_comments_and_enters_the_history(self):
        github = FakeGitHub()
        rows = self.rows("fail")
        REPORT.update_summary_issue(
            github, github.data, "failure", "JuMP", rows, "run-1", generated="2026-11-01T04:00:00+00:00"
        )
        self.assertEqual(github.comments, [])  # creation notifies, no comment
        # ...but the first finding seeds the history
        self.assertIn("new: `beam`", github.data[0]["body"])
        REPORT.update_summary_issue(
            github, github.data, "failure", "JuMP", [], "run-2", generated="2026-11-08T04:00:00+00:00"
        )
        self.assertEqual(len(github.comments), 1)
        self.assertIn("all clear", github.comments[0][1])
        body = github.data[0]["body"]
        self.assertIn("<details><summary>📜 <b>Recent history</b></summary>", body)
        self.assertIn("resolved: `beam` (all clear)", body)
        self.assertIn("[run](run-2)", body)
        # a stable finding set adds neither a comment nor an entry
        REPORT.update_summary_issue(
            github, github.data, "failure", "JuMP", [], "run-3", generated="2026-11-15T04:00:00+00:00"
        )
        self.assertEqual(len(github.comments), 1)
        body = github.data[0]["body"]
        self.assertEqual(1, body.count("resolved: `beam`"))

    def test_the_history_is_bounded(self):
        github = FakeGitHub()
        REPORT.update_summary_issue(github, github.data, "failure", "JuMP", self.rows("fail"), "r0")
        for index in range(12):  # alternating fail/clean: one entry per run
            rows = self.rows("fail") if index % 2 else []
            REPORT.update_summary_issue(github, github.data, "failure", "JuMP", rows, f"r{index+1}")
        body = github.data[0]["body"]
        entries = REPORT.section_history(body, "JuMP")
        self.assertEqual(REPORT.MAX_HISTORY, len(entries))
        self.assertIn("r12", entries[0])  # newest first



RESULT_HEADER = (
    "test\tproblem\tmodeling\tconfiguration\tgrid_size\tscheme\tresult\treason\tobjective"
    "\titerations\ttime\tgap\n"
)


def result_row(problem, result="converged", objective="1.0", platform="cron-a", solution=""):
    return {
        "test": "test_convergence c", "problem": problem, "modeling": "JuMP",
        "configuration": "solver=ipopt", "result": result, "reason": "LOCALLY_SOLVED",
        "objective": objective, "iterations": "10", "time": "0.1", "gap": "",
        "solution": solution, "platform": platform,
    }


class ConvergenceTests(unittest.TestCase):
    def test_parse_results_reads_the_tsv_with_its_platform(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            (root / "cron-a").mkdir()
            (root / "cron-a" / "JuMP-cron-results.tsv").write_text(
                RESULT_HEADER
                + "test_convergence c\tbeam\tJuMP\tsolver=ipopt\t500\ttrapeze\tconverged"
                + "\tLOCALLY_SOLVED\t8.889275372051\t18\t0.03\t\n"
                + "too\tfew\tcells\n"
            )
            # a file of before #375, without the grid and the scheme, is read too
            (root / "cron-b").mkdir()
            (root / "cron-b" / "JuMP-cron-results.tsv").write_text(
                "test\tproblem\tmodeling\tconfiguration\tresult\treason\tobjective\titerations"
                "\ttime\tgap\n"
                + "test_convergence c\tbeam\tJuMP\tsolver=ipopt\tconverged\tLOCALLY_SOLVED\t8.9\t18\t0.03\t\n"
            )
            rows = REPORT.parse_results(root)
            # the five-column parser of the outcomes reads only `*.md`
            self.assertEqual([], REPORT.parse_reports(root))
        self.assertEqual(2, len(rows))
        self.assertEqual("cron-a", rows[0]["platform"])
        self.assertEqual("converged", rows[0]["result"])
        self.assertEqual("8.889275372051", rows[0]["objective"])
        self.assertEqual("cron-b", rows[1]["platform"])

    def test_the_report_shows_six_significant_digits_of_the_objective(self):
        self.assertEqual("8.88928", REPORT.short("8.889275372051"))
        self.assertEqual("", REPORT.short(""))

    def test_previous_results_are_those_of_the_previous_run_for_both_calls(self):
        state = {}
        self.assertEqual({}, REPORT.remember_results(state, "JuMP", [result_row("beam")], "run1"))
        second = [result_row("beam", result="not_converged")]
        previous = REPORT.remember_results(state, "JuMP", second, "run2")
        self.assertEqual("converged", previous[REPORT.result_key(second[0])]["result"])
        # the issues call of the same run compares with the same previous run
        again = REPORT.remember_results(state, "JuMP", second, "run2")
        self.assertEqual(previous, again)
        # another scope keeps its own history
        self.assertEqual({}, REPORT.remember_results(state, "OptimalControl", second, "run2"))

    def test_change_and_degradations(self):
        before = {
            REPORT.result_key(result_row(p)): {"result": r, "objective": o}
            for p, r, o in (("a", "converged", "1.0"), ("b", "not_converged", "1.0"),
                            ("c", "converged", "1.0"), ("d", "converged", "1.0"))
        }
        now = [
            result_row("a", result="not_converged"),
            result_row("b"),
            result_row("c", objective="1.01"),
            result_row("d", objective="1.00000001"),
            result_row("e"),
        ]
        self.assertEqual(
            ["lost", "gained", "objective +1.00e-02", "", "new"],
            [REPORT.change(row, before) for row in now],
        )
        worse = REPORT.degradations(now, before)
        self.assertEqual(["a", "c"], [row["problem"] for row in worse])
        self.assertEqual("solver=ipopt", worse[0]["test"])
        # the issue shows before → after, not just `lost` or a bare delta
        self.assertEqual("converged → not_converged", worse[0]["outcome"])
        self.assertEqual("objective 1 → 1.01 (+1.00e-02)", worse[1]["outcome"])

    def test_a_new_solution_is_flagged_and_a_known_one_is_not(self):
        before = {REPORT.result_key(result_row(p)): {"result": "converged", "objective": "1.0"}
                  for p in ("a", "b")}
        now = [
            result_row("a", solution="new"),
            result_row("b", solution="local"),
            result_row("c", solution="new"),  # not in the previous run
            result_row("d", result="not_converged", solution=""),
        ]
        worse = REPORT.degradations(now, before)
        self.assertEqual(["a", "c"], [row["problem"] for row in worse])
        self.assertEqual(["new solution", "new solution"], [row["outcome"] for row in worse])
        # the column is shown in the report
        report = REPORT.report_markdown("JuMP", [], "run", "now", "sha", "1.13", "0.6.0", now, before)
        self.assertIn("| solution | change |", report)
        self.assertIn("| local |  |", report)

    def test_a_passing_convergence_test_does_not_make_a_draft_ready(self):
        rows = [
            {"test": "test_convergence solver=ipopt", "problem": "p", "modelings": "JuMP",
             "status": "draft", "outcome": "pass", "platform": "a"},
        ]
        self.assertEqual([], REPORT.findings(rows)["draft"])
        rows[0]["test"] = "test_robustness solver=ipopt mu_init=0.001"
        self.assertEqual([], REPORT.findings(rows)["draft"])
        rows[0]["test"] = "test_init"
        self.assertEqual(1, len(REPORT.findings(rows)["draft"]))

    def test_report_lists_the_results_with_their_change(self):
        text = REPORT.report_markdown(
            "JuMP", [], "run", "now", "sha", "1.13", "0.6.0",
            [result_row("beam", result="not_converged")],
            {REPORT.result_key(result_row("beam")): {"result": "converged", "objective": "1.0"}},
        )
        self.assertIn("## Convergence (reported, not tested)", text)
        self.assertIn("| beam | JuMP | solver=ipopt | not_converged |", text)
        self.assertTrue(text.rstrip().endswith("| lost |"))
        # no results: no section
        self.assertNotIn("Convergence", REPORT.report_markdown("JuMP", [], "r", "n", "s", "j", "p"))


# One report of a platform, as the tests write it (#314): results, trajectories, environment.
def write_run(root, platform, report, problems, objective="1.5", fingerprint="f1"):
    folder = root / platform
    (folder / f"{report}-env").mkdir(parents=True, exist_ok=True)
    (folder / f"{report}-env" / "Project.toml").write_text("[deps]\n")
    (folder / f"{report}-environment.toml").write_text('julia = "1.13.0"\n')
    lines = [RESULT_HEADER.rstrip("\n") + "\tsolution\tfingerprint"]
    solves = []
    for problem in problems:
        lines.append(
            f"test_convergence c\t{problem}\t{report}\tsolver=ipopt\t500\ttrapeze\tconverged"
            f"\tLOCALLY_SOLVED\t{objective}\t18\t0.03\t\tbest\t{fingerprint}"
        )
        solves.append(
            "[[solve]]\n"
            f'problem = "{problem}"\nmodeling = "{report}"\nconfiguration = "solver=ipopt"\n'
            f'fingerprint = "{fingerprint}"\n'
            "[solve.trajectory]\ntime = [0.0, 1.0]\nvariable = []\n"
            "[solve.trajectory.state]\nx = [0.0, 1.0]\n"
            "[solve.trajectory.control]\nu = [1.0, 1.0]\n"
            "[solve.trajectory.costate]\nx = [2.0, 2.0]\n"
        )
    (folder / f"{report}-results.tsv").write_text("\n".join(lines) + "\n")
    (folder / f"{report}-trajectories.toml").write_text("\n".join(solves))


def publish(reports, results, scope, run, **kw):
    rows = REPORT.parse_results(reports)
    REPORT.publish_results(
        reports, results, scope, rows, f"https://x/actions/runs/{run}", "sha" + run, "date" + run
    )


class ResultsTests(unittest.TestCase):
    def test_a_run_writes_the_results_the_trajectories_and_the_environment(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            write_run(root / "a", "cron-JuMP-1.13-ubuntu-latest", "JuMP", ["beam"])
            publish(root / "a", root / "results", "JuMP", "1")
            data = json.loads((root / "results/problems/beam/result.json").read_text())
            entry = data["entries"][0]
            self.assertEqual("beam", data["problem"])
            self.assertEqual(1.5, entry["objective"])
            self.assertEqual(18, entry["iterations"])
            self.assertIsNone(entry["gap"])
            self.assertEqual("f1", entry["fingerprint"])
            self.assertEqual("https://x/actions/runs/1", entry["run"])
            self.assertEqual("sha1", entry["commit"])
            self.assertNotIn("report", entry)
            trajectory = json.loads((root / "results" / entry["trajectory"]).read_text())
            self.assertEqual({"x": [0.0, 1.0]}, trajectory["state"])
            env = root / "results" / entry["environment"]
            self.assertEqual("env/1/cron-JuMP-1.13-ubuntu-latest/JuMP", entry["environment"])
            self.assertTrue((env / "environment.toml").is_file())
            self.assertTrue((env / "Project.toml").is_file())
            manifest = json.loads((root / "results/manifest.json").read_text())
            self.assertEqual(["beam"], manifest["scopes"]["JuMP"]["problems"])
            self.assertEqual("date1", manifest["updated"])

    def test_a_run_replaces_the_entries_of_its_scope_only(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            results = root / "results"
            write_run(root / "a", "cpu", "JuMP", ["beam", "chain"])
            publish(root / "a", results, "JuMP", "1")
            write_run(root / "b", "cpu", "OptimalControl", ["beam"])
            publish(root / "b", results, "OptimalControl", "2")
            # a run of JuMP restricted to `beam` (`problem=beam`)
            write_run(root / "c", "cpu", "JuMP", ["beam"], objective="2.5")
            publish(root / "c", results, "JuMP", "3")

            beam = json.loads((results / "problems/beam/result.json").read_text())["entries"]
            self.assertEqual(
                [("JuMP", 2.5, "3"), ("OptimalControl", 1.5, "2")],
                [(e["scope"], e["objective"], e["run"].rsplit("/", 1)[1]) for e in beam],
            )
            chain = json.loads((results / "problems/chain/result.json").read_text())["entries"]
            self.assertEqual("https://x/actions/runs/1", chain[0]["run"])
            # the environment of run 1 is kept while `chain` refers to it, run 2's too
            self.assertEqual({"1", "2", "3"}, {p.name for p in (results / "env").iterdir()})
            write_run(root / "d", "cpu", "JuMP", ["beam", "chain"])
            publish(root / "d", results, "JuMP", "4")
            self.assertEqual({"2", "4"}, {p.name for p in (results / "env").iterdir()})

    def test_a_solve_without_trajectory_or_environment_is_published(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            write_run(root / "a", "cpu", "JuMP", ["beam"])
            (root / "a/cpu/JuMP-trajectories.toml").unlink()
            (root / "a/cpu/JuMP-environment.toml").unlink()
            publish(root / "a", root / "results", "JuMP-robustness", "1")
            entry = json.loads((root / "results/problems/beam/result.json").read_text())[
                "entries"
            ][0]
            self.assertIsNone(entry["trajectory"])
            self.assertIsNone(entry["environment"])
            self.assertEqual("JuMP-robustness", entry["scope"])

    def test_slug(self):
        self.assertEqual("modeler-adnlp_solver-ipopt", REPORT.slug("modeler=adnlp solver=ipopt"))
        self.assertEqual("mu_init-0.001", REPORT.slug("mu_init=0.001"))


if __name__ == "__main__":
    unittest.main()
