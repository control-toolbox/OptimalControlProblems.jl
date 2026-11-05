import importlib.util
import pathlib
import tempfile
import unittest


ROOT = pathlib.Path(__file__).parents[2]
SPEC = importlib.util.spec_from_file_location("cron_report", ROOT / "tools/cron/report.py")
REPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPORT)


class ReportTest(unittest.TestCase):
    def rows(self, *outcomes):
        return [
            {
                "test": "test_convergence solver=ipopt",
                "problem": "beam",
                "modelings": "JuMP",
                "status": "valid",
                "outcome": outcome,
                "source": "JuMP-cron.md",
            }
            for outcome in outcomes
        ]

    def test_failure_and_draft_findings(self):
        rows = self.rows("fail", "pass")
        rows[1]["status"] = "draft"
        result = REPORT.findings(rows)
        self.assertEqual(result["failure"], [rows[0]])
        self.assertEqual(result["draft"], [rows[1]])

    def test_report_parser_skips_header(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "scope.md"
            path.write_text(
                "# Report\n\n"
                "| test | problem | modelings | status | outcome |\n"
                "| --- | --- | --- | --- | --- |\n"
                "| test_convergence | beam | JuMP | valid | fail |\n"
            )
            rows = REPORT.parse_reports(pathlib.Path(directory))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["outcome"], "fail")

    def test_replace_section_only_replaces_one_scope(self):
        original = (
            "# Report\n\n"
            "<!-- cron-scope: JuMP -->\n"
            "## old\n\n"
            "<!-- cron-scope: comparison -->\n"
            "## comparison\n"
        )
        replacement = REPORT.issue_section("JuMP", "cron-failure", [], "run")
        result = REPORT.replace_section(original, "JuMP", replacement)
        self.assertIn("No findings.", result)
        self.assertIn("## comparison", result)
        self.assertNotIn("## old", result)

    def test_bug_issue_is_idempotent_and_tracks_configuration_changes(self):
        class FakeGitHub:
            def __init__(self):
                self.data = []
                self.comments = []

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
                return issue

            def comment(self, number, body):
                self.comments.append((number, body))

        github = FakeGitHub()
        state = {}
        rows = self.rows("fail")
        REPORT.update_bug_issues(github, rows, "JuMP", "run-1", "date", state)
        REPORT.update_bug_issues(github, rows, "JuMP", "run-2", "date", state)
        self.assertEqual(len(github.data), 1)
        self.assertEqual(github.comments, [])

        changed = self.rows("fail")
        changed[0]["test"] = "test_convergence solver=madnlp"
        REPORT.update_bug_issues(github, changed, "JuMP", "run-3", "date", state)
        self.assertEqual(len(github.comments), 1)

    def test_general_breakage_does_not_open_per_problem_issues(self):
        class FakeGitHub:
            def __init__(self):
                self.created = 0

            def issues(self):
                return []

            def create_issue(self, *args):
                self.created += 1

        rows = []
        for index in range(6):
            row = self.rows("fail")[0]
            row["problem"] = f"problem-{index}"
            rows.append(row)
        github = FakeGitHub()
        REPORT.update_bug_issues(github, rows, "JuMP", "run", "date", {})
        self.assertEqual(github.created, 0)


if __name__ == "__main__":
    unittest.main()
