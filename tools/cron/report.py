#!/usr/bin/env python3
"""Publish one cron scope report and maintain its GitHub issue state."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import pathlib
import re
import urllib.request


SUMMARY_LABELS = {
    "failure": "cron-failure",
    "draft": "cron-drafts",
    "capability": "cron-capabilities",
}
SUCCESS = {"pass"}
FAILURE = {"fail", "error"}
SCOPE_MARKER = "<!-- cron-scope: {scope} -->"
KEY_RE = re.compile(r"<!-- cron-key: ([^ ]+) -->")


def parse_reports(directory: pathlib.Path) -> list[dict[str, str]]:
    rows = []
    for path in sorted(directory.rglob("*.md")):
        for line in path.read_text().splitlines():
            if not line.startswith("|") or line.startswith("| ---"):
                continue
            cells = [cell.strip() for cell in line.strip("|").split("|")]
            if len(cells) != 5 or cells[0] == "test":
                continue
            rows.append(
                {
                    "test": cells[0],
                    "problem": cells[1],
                    "modelings": cells[2],
                    "status": cells[3],
                    "outcome": cells[4],
                    "source": str(path),
                }
            )
    return rows


def findings(rows: list[dict[str, str]]) -> dict[str, list[dict[str, str]]]:
    valid_failures = [
        row for row in rows if row["status"] == "valid" and row["outcome"] in FAILURE
    ]
    drafts = [
        row for row in rows if row["status"] == "draft" and row["outcome"] in SUCCESS
    ]
    capabilities = [
        row
        for row in rows
        if row["status"] == "known_failure" and row["outcome"] in SUCCESS
    ]
    return {"failure": valid_failures, "draft": drafts, "capability": capabilities}


def report_markdown(
    scope: str, rows: list[dict[str, str]], run_url: str, generated: str
) -> str:
    lines = [
        f"# Cron report: {scope}",
        "",
        f"Generated: {generated}",
        f"Run: {run_url}",
        "",
        "| test | problem | modeling | status | outcome |",
        "| --- | --- | --- | --- | --- |",
    ]
    lines.extend(
        "| {test} | {problem} | {modelings} | {status} | {outcome} |".format(**row)
        for row in rows
    )
    return "\n".join(lines) + "\n"


def issue_section(scope: str, title: str, rows: list[dict[str, str]], run_url: str) -> str:
    lines = [SCOPE_MARKER.format(scope=scope), f"## {title}", "", f"Run: {run_url}", ""]
    if rows:
        lines.extend(
            ["| problem | modeling | test | outcome |", "| --- | --- | --- | --- |"]
        )
        lines.extend(
            f"| {row['problem']} | {row['modelings']} | {row['test']} | {row['outcome']} |"
            for row in rows
        )
    else:
        lines.append("No findings.")
    return "\n".join(lines)


def replace_section(body: str, scope: str, section: str) -> str:
    marker = SCOPE_MARKER.format(scope=scope)
    start = body.find(marker)
    if start < 0:
        return body.rstrip() + "\n\n" + section + "\n"
    next_heading = re.search(r"\n<!-- cron-scope: [^ ]+ -->", body[start + len(marker) :])
    end = start + len(marker) + (next_heading.start() if next_heading else len(body))
    return body[:start].rstrip() + "\n\n" + section + "\n" + body[end:].lstrip()


class GitHub:
    def __init__(self, repository: str, token: str):
        self.base = f"https://api.github.com/repos/{repository}"
        self.token = token

    def request(self, method: str, path: str, payload: object | None = None):
        data = None if payload is None else json.dumps(payload).encode()
        request = urllib.request.Request(
            self.base + path,
            data=data,
            method=method,
            headers={
                "Accept": "application/vnd.github+json",
                "Authorization": f"Bearer {self.token}",
                "X-GitHub-Api-Version": "2022-11-28",
            },
        )
        with urllib.request.urlopen(request) as response:
            return json.loads(response.read())

    def issues(self):
        result = []
        page = 1
        while True:
            batch = self.request("GET", f"/issues?state=all&per_page=100&page={page}")
            result.extend(issue for issue in batch if "pull_request" not in issue)
            if len(batch) < 100:
                return result
            page += 1

    def create_issue(self, title: str, body: str, labels: list[str]):
        return self.request("POST", "/issues", {"title": title, "body": body, "labels": labels})

    def update_issue(self, number: int, **payload):
        return self.request("PATCH", f"/issues/{number}", payload)

    def comment(self, number: int, body: str):
        return self.request("POST", f"/issues/{number}/comments", {"body": body})


def labels(issue: dict) -> set[str]:
    return {label["name"] for label in issue.get("labels", [])}


def update_summary_issue(
    github: GitHub, kind: str, scope: str, rows: list[dict[str, str]], run_url: str
) -> None:
    label = SUMMARY_LABELS[kind]
    section = issue_section(scope, label, rows, run_url)
    candidates = [issue for issue in github.issues() if label in labels(issue)]
    issue = next((item for item in candidates if item["state"] == "open"), None)
    if issue is None and not rows:
        return
    if issue is None:
        issue = github.create_issue(
            f"[Cron] {label}", f"# Automated cron report\n\n{section}\n", [label]
        )
    else:
        body = replace_section(issue.get("body") or "# Automated cron report", scope, section)
        github.update_issue(issue["number"], body=body)


def issue_key(row: dict[str, str]) -> str:
    return f"{row['problem']}|{row['modelings']}"


def bug_body(
    row: dict[str, str],
    configurations: list[str],
    run_url: str,
    date: str,
) -> str:
    key = issue_key(row)
    return (
        f"<!-- cron-key: {key} -->\n"
        f"# Cron regression: `{row['problem']}` ({row['modelings']})\n\n"
        f"Last run: {run_url}\n"
        f"Last observed: {date}\n\n"
        "Failing configurations:\n\n"
        + "".join(f"- `{configuration}`\n" for configuration in configurations)
    )


def update_bug_issues(
    github: GitHub,
    rows: list[dict[str, str]],
    scope: str,
    run_url: str,
    date: str,
    state: dict,
) -> None:
    all_issues = github.issues()
    grouped: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        grouped.setdefault(issue_key(row), []).append(row)
    if len(grouped) > 5:
        return
    for key, problem_rows in grouped.items():
        row = problem_rows[0]
        entry = state.setdefault(key, {"successes": 0, "reopenings": 0})
        entry["successes"] = 0
        entry["scope"] = scope
        configurations = sorted(row["test"] for row in problem_rows)
        configuration_set_changed = entry.get("configurations") != configurations
        entry["configurations"] = configurations
        matching = [issue for issue in all_issues if key in KEY_RE.findall(issue.get("body") or "")]
        issue = matching[0] if matching else None
        if issue and issue.get("state_reason") == "not_planned":
            continue
        body = bug_body(row, configurations, run_url, date)
        if issue is None:
            github.create_issue(
                f"[Cron] Regression: {row['problem']} ({row['modelings']})",
                body,
                ["bug", "cron-bug"],
            )
        else:
            if issue["state"] == "closed" and entry["reopenings"] < 3:
                github.update_issue(issue["number"], state="open")
                github.comment(issue["number"], f"Regression detected again: {run_url}")
                entry["reopenings"] += 1
            elif configuration_set_changed:
                github.comment(issue["number"], f"Failing configurations changed: {run_url}")
            if entry["reopenings"] >= 3:
                github.update_issue(issue["number"], labels=["bug", "cron-bug", "flaky"])
            github.update_issue(issue["number"], body=body)

    for issue in all_issues:
        match = KEY_RE.search(issue.get("body") or "")
        if not match or match.group(1) not in state:
            continue
        key = match.group(1)
        if key in grouped or issue["state"] != "open":
            continue
        entry = state[key]
        if entry.get("scope") != scope:
            continue
        entry["successes"] = entry.get("successes", 0) + 1
        if entry["successes"] >= 2:
            github.update_issue(issue["number"], state="closed", state_reason="completed")
            github.comment(issue["number"], f"Two consecutive successful runs: {run_url}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reports-dir", type=pathlib.Path, required=True)
    parser.add_argument("--scope", required=True)
    parser.add_argument("--run-url", required=True)
    parser.add_argument("--state-file", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--tests-failed", action="store_true")
    parser.add_argument("--no-github", action="store_true")
    args = parser.parse_args()

    now = dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat()
    rows = parse_reports(args.reports_dir)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(report_markdown(args.scope, rows, args.run_url, now))

    state = {}
    if args.state_file.exists():
        state = json.loads(args.state_file.read_text())
    github_token = os.environ.get("GITHUB_TOKEN")
    repository = os.environ.get("GITHUB_REPOSITORY")
    if not args.no_github and github_token and repository:
        github = GitHub(repository, github_token)
        grouped = findings(rows)
        failure_rows = grouped["failure"]
        if args.tests_failed and not failure_rows:
            failure_rows = [
                {
                    "test": "workflow",
                    "problem": "workflow",
                    "modelings": args.scope,
                    "status": "valid",
                    "outcome": "error",
                }
            ]
        update_summary_issue(github, "failure", args.scope, failure_rows, args.run_url)
        update_summary_issue(github, "draft", args.scope, grouped["draft"], args.run_url)
        update_summary_issue(github, "capability", args.scope, grouped["capability"], args.run_url)
        update_bug_issues(
            github,
            grouped["failure"],
            args.scope,
            args.run_url,
            now,
            state,
        )
    args.state_file.parent.mkdir(parents=True, exist_ok=True)
    args.state_file.write_text(json.dumps(state, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
