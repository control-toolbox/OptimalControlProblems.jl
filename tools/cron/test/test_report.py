import importlib.util
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
        replacement = REPORT.issue_section("JuMP", "cron-failure", [], "run")
        result = REPORT.replace_section(original, "JuMP", replacement)
        self.assertIn("No findings.", result)
        self.assertIn("## comparison", result)
        self.assertNotIn("## old", result)


if __name__ == "__main__":
    unittest.main()
