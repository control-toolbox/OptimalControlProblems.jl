#!/usr/bin/env python3
"""Publish one cron scope report and maintain its GitHub issue state."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import pathlib
import re
import time
import urllib.error
import urllib.request


SUMMARY_LABELS = {
    "failure": "cron-failure",
    "draft": "cron-drafts",
    "capability": "cron-capabilities",
    "convergence": "cron-convergence",
}
FAILURE = {"fail", "error"}
SCOPE_MARKER = "<!-- cron-scope: {scope} -->"
KEY_RE = re.compile(r"<!-- cron-key: (.+?) -->")
REOPENINGS_RE = re.compile(r"<!-- cron-reopenings: (\d+) -->")
README_URL = (
    "https://github.com/control-toolbox/OptimalControlProblems.jl/blob/main/"
    ".github/workflows/README.md#what-to-do-with-the-cron-issues"
)
HEADERS = {
    "failure": (
        "# Cron failures\n\n"
        "Written by the weekly cron workflows, one section per scope, replaced by each run of "
        "that scope. A `:valid` problem failed its slow tests (or the run itself failed).\n\n"
        "**Who:** a maintainer triages; nothing is fixed or changed automatically (an agent only "
        "if a maintainer asks for it).\n\n"
        "**What to do:** open the run linked in the section, find the failing problem and "
        "configuration in the table, and decide: fix the problem or the package; report an "
        "upstream break; or, if the configuration is not supported, set the problem to "
        "`:draft` in `ext/MetaData/<problem>.jl`. A `cron-bug` issue per (problem, modeling) is "
        "opened for up to 5 failing pairs per run; more than 5 is reported here only, as a "
        f"probable upstream breakage. Details: [README]({README_URL}). "
        "This issue updates itself; close it when the sections say `No findings.`"
    ),
    "draft": (
        "# Drafts ready to become valid\n\n"
        "Written by the weekly cron workflows, one section per scope, replaced by each run of "
        "that scope. Each problem below is a `:draft` that **passed all its tests on every "
        "platform of the run**.\n\n"
        "**Who:** a maintainer decides; nothing changes automatically (an agent only if a "
        "maintainer asks for it).\n\n"
        "**What to do:** check the problem, then change its status to `:valid` in "
        "`ext/MetaData/<problem>.jl` (`:status => (JuMP=:valid, ...)`, the modeling named in "
        "the table) in a pull request; from then on a failure fails the CI and the cron. "
        f"Details: [README]({README_URL}). This issue updates itself; the problem leaves the "
        "list once it is `:valid`."
    ),
    "capability": (
        "# Declared known failures that no longer happen\n\n"
        "Written by the weekly cron workflows, one section per scope. Each entry below is a "
        "known failure declared in the metadata that now passes.\n\n"
        "**What to do:** remove the entry from `:known_failures` in `ext/MetaData/<problem>.jl`. "
        f"Details: [README]({README_URL})."
    ),
}
HEADERS["convergence"] = (
    "# Convergence that got worse\n\n"
    "Written by the weekly cron workflows, one section per scope, replaced by each run of "
    "that scope. Convergence is reported, not tested (#360): each entry below is a solve "
    "that converged in the previous run of the scope and no longer does, or whose objective "
    "moved by more than the threshold of `tools/cron/report.py`.\n\n"
    "**Who:** a maintainer looks; nothing fails and nothing changes automatically.\n\n"
    "**What to do:** open the run, compare with the previous report on the `results` branch, "
    "and decide whether the change is expected (new initial guess, new solver version) or a "
    f"regression to investigate. Details: [README]({README_URL}). This issue updates itself; "
    "close it when the sections say `No findings.`"
)
# The tests that report convergence instead of testing it (`cron/<scope>/test_convergence.jl`).
CONVERGENCE_TEST = "test_convergence"
# A relative change of the objective above this is reported as a change of the convergence.
OBJECTIVE_CHANGE = 1e-4
BUG_LABEL = "cron-bug"
MAX_BUG_ISSUES_PER_RUN = 5
MAX_REOPENINGS = 3


def issue_key(row: dict[str, str]) -> str:
    return f"{row['problem']}|{row['modelings']}"


def row_configuration(row: dict[str, str]) -> str:
    platform = row.get("platform") or "unknown platform"
    return f"{platform}: {row['test']}"


def parse_reports(directory: pathlib.Path) -> list[dict[str, str]]:
    rows = []
    for path in sorted(directory.rglob("*.md")):
        relative = path.relative_to(directory)
        platform = relative.parts[0] if len(relative.parts) > 1 else "local"
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
                    "platform": platform,
                    "source": str(relative),
                }
            )
    return rows


# Columns of `<report>-results.tsv` the report reads, by name: the file may have more (grid and
# scheme since #375), in any order.
RESULT_COLUMNS = (
    "test", "problem", "modeling", "configuration", "result", "reason", "objective",
    "iterations", "time", "gap",
)


def parse_results(directory: pathlib.Path) -> list[dict[str, str]]:
    """The results of the solves (`<report>-results.tsv`, #360), with their platform."""
    results = []
    for path in sorted(directory.rglob("*-results.tsv")):
        relative = path.relative_to(directory)
        platform = relative.parts[0] if len(relative.parts) > 1 else "local"
        lines = path.read_text().splitlines()
        if not lines:
            continue
        header = lines[0].split("\t")
        if not set(RESULT_COLUMNS) <= set(header):
            continue
        for line in lines[1:]:
            cells = line.split("\t")
            if len(cells) != len(header):
                continue
            row = {column: cell for column, cell in zip(header, cells) if column in RESULT_COLUMNS}
            row["platform"] = platform
            results.append(row)
    return results


def short(value: str) -> str:
    """A number of the results with six significant digits (the file has full precision)."""
    try:
        return f"{float(value):.6g}"
    except ValueError:
        return value


def result_key(row: dict[str, str]) -> str:
    return f"{row['platform']}|{row['problem']}|{row['modeling']}|{row['configuration']}"


def remember_results(
    state: dict, scope: str, results: list[dict[str, str]], run_url: str
) -> dict[str, dict[str, str]]:
    """Store the results of this run in `state` and return those of the previous run.

    The report is written twice per run (report, then issues): the previous results are
    kept with the run they belong to, so that both calls compare with the same run."""
    entry = state.setdefault("convergence", {}).setdefault(scope, {})
    if entry.get("run") != run_url:
        entry["previous"] = entry.get("results", {})
        entry["run"] = run_url
    entry["results"] = {
        result_key(row): {"result": row["result"], "objective": row["objective"]}
        for row in results
    }
    return entry.get("previous", {})


def change(row: dict[str, str], previous: dict[str, dict[str, str]]) -> str:
    """How a result changed since the previous run: `new`, `lost`, `gained`, the relative
    change of the objective above `OBJECTIVE_CHANGE`, or nothing."""
    before = previous.get(result_key(row))
    if before is None:
        return "new"
    if before["result"] == "converged" and row["result"] != "converged":
        return "lost"
    if before["result"] != "converged" and row["result"] == "converged":
        return "gained"
    try:
        a, b = float(before["objective"]), float(row["objective"])
    except ValueError:
        return ""
    scale = abs(a) if a != 0 else 1.0
    delta = (b - a) / scale
    return f"objective {delta:+.2e}" if abs(delta) > OBJECTIVE_CHANGE else ""


def degradations(
    results: list[dict[str, str]], previous: dict[str, dict[str, str]]
) -> list[dict[str, str]]:
    """Results that got worse, as rows of a summary issue: lost convergence, or a converged
    objective that moved (a move is not better or worse without a reference)."""
    rows = []
    for row in results:
        what = change(row, previous)
        if what == "lost" or (what.startswith("objective") and row["result"] == "converged"):
            rows.append(
                {
                    "platform": row["platform"],
                    "problem": row["problem"],
                    "modelings": row["modeling"],
                    "test": row["configuration"],
                    "outcome": what,
                }
            )
    return rows


def group_rows(rows: list[dict[str, str]]) -> dict[str, list[dict[str, str]]]:
    grouped: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        grouped.setdefault(issue_key(row), []).append(row)
    return grouped


def all_pass(rows: list[dict[str, str]]) -> bool:
    return bool(rows) and all(row["outcome"] == "pass" for row in rows)


def findings(rows: list[dict[str, str]]) -> dict[str, list[dict[str, str]]]:
    valid_failures = [
        row for row in rows if row["status"] == "valid" and row["outcome"] in FAILURE
    ]
    # A convergence test reports and always passes (#360): it says nothing about whether a
    # draft is finished or equivalent, so it never makes a draft ready.
    draft_rows = [
        row
        for row in rows
        if row["status"] == "draft" and not row["test"].startswith(CONVERGENCE_TEST)
    ]
    ready_drafts = [
        row
        for group in group_rows(draft_rows).values()
        if all_pass(group)
        for row in group
    ]
    capabilities = [
        row
        for row in rows
        if row["status"] == "known_failure" and row["outcome"] == "pass"
    ]
    return {"failure": valid_failures, "draft": ready_drafts, "capability": capabilities}


def successful_keys(rows: list[dict[str, str]]) -> set[str]:
    valid_rows = [row for row in rows if row["status"] == "valid"]
    return {
        key
        for key, group in group_rows(valid_rows).items()
        if all_pass(group)
    }


def prepare_state(
    rows: list[dict[str, str]],
    scope: str,
    tests_failed: bool,
    state: dict,
) -> dict:
    failure_groups = group_rows(findings(rows)["failure"])
    for key, group in failure_groups.items():
        entry = state.setdefault(key, {"successes": 0, "reopenings": 0})
        entry["scope"] = scope
        entry["successes"] = 0

    observed = bool(rows) and not tests_failed
    if observed:
        for key in successful_keys(rows) - set(failure_groups):
            entry = state.setdefault(key, {"successes": 0, "reopenings": 0})
            if entry.get("scope") == scope or "scope" not in entry:
                entry["scope"] = scope
                entry["successes"] = entry.get("successes", 0) + 1
    return state


def report_markdown(
    scope: str,
    rows: list[dict[str, str]],
    run_url: str,
    generated: str,
    sha: str,
    julia_version: str,
    package_version: str,
    results: list[dict[str, str]] | None = None,
    previous: dict[str, dict[str, str]] | None = None,
) -> str:
    lines = [
        f"# Cron report: {scope}",
        "",
        f"Generated: {generated}",
        f"Run: {run_url}",
        f"Commit: `{sha}`",
        f"Julia: `{julia_version}`",
        f"Package version: `{package_version}`",
        "",
        "| platform | test | problem | modelings | status | outcome |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    lines.extend(
        "| {platform} | {test} | {problem} | {modelings} | {status} | {outcome} |".format(
            **row
        )
        for row in rows
    )
    if results:
        previous = previous or {}
        lines.extend(
            [
                "",
                "## Convergence (reported, not tested)",
                "",
                "| platform | problem | modeling | configuration | result | reason | objective "
                "| iterations | time | gap | change |",
                "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
            ]
        )
        lines.extend(
            "| {platform} | {problem} | {modeling} | {configuration} | {result} | {reason} "
            "| {objective} | {iterations} | {time} | {gap} | ".format(
                **{**row, "objective": short(row["objective"])}
            )
            + change(row, previous)
            + " |"
            for row in results
        )
    return "\n".join(lines) + "\n"


def issue_section(scope: str, title: str, rows: list[dict[str, str]], run_url: str) -> str:
    lines = [SCOPE_MARKER.format(scope=scope), f"## {title}", "", f"Run: {run_url}", ""]
    if rows:
        lines.extend(
            [
                "| platform | problem | modeling | test | outcome |",
                "| --- | --- | --- | --- | --- |",
            ]
        )
        lines.extend(
            f"| {row.get('platform', '')} | {row['problem']} | {row['modelings']} | "
            f"{row['test']} | {row['outcome']} |"
            for row in rows
        )
    else:
        lines.append("No findings.")
    return "\n".join(lines)


def with_header(body: str, header: str) -> str:
    """`body` with its text before the first scope section replaced by `header`."""
    start = body.find("<!-- cron-scope:")
    rest = body[start:] if start >= 0 else ""
    return header + "\n\n" + rest


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
        self._issues = None

    def request(self, method: str, path: str, payload: object | None = None):
        data = None if payload is None else json.dumps(payload).encode()
        for attempt in range(4):
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
            try:
                with urllib.request.urlopen(request) as response:
                    text = response.read()
                    return json.loads(text) if text else {}
            except urllib.error.HTTPError as error:
                if attempt == 3 or error.code not in (403, 429, 500, 502, 503, 504):
                    raise
                retry_after = error.headers.get("Retry-After")
                delay = float(retry_after) if retry_after else 2.0**attempt
                time.sleep(delay)
        raise RuntimeError("unreachable")

    def issues(self):
        if self._issues is None:
            result = []
            page = 1
            while True:
                batch = self.request("GET", f"/issues?state=all&per_page=100&page={page}")
                result.extend(issue for issue in batch if "pull_request" not in issue)
                if len(batch) < 100:
                    break
                page += 1
            self._issues = result
        return self._issues

    def create_issue(self, title: str, body: str, labels: list[str]):
        issue = self.request(
            "POST", "/issues", {"title": title, "body": body, "labels": labels}
        )
        if self._issues is not None:
            self._issues.append(issue)
        return issue

    def update_issue(self, number: int, **payload):
        issue = self.request("PATCH", f"/issues/{number}", payload)
        if self._issues is not None:
            for index, current in enumerate(self._issues):
                if current["number"] == number:
                    self._issues[index] = issue
                    break
        return issue

    def add_labels(self, number: int, new_labels: list[str]):
        return self.request("POST", f"/issues/{number}/labels", {"labels": new_labels})

    def comment(self, number: int, body: str):
        return self.request("POST", f"/issues/{number}/comments", {"body": body})


def labels(issue: dict) -> set[str]:
    return {label["name"] for label in issue.get("labels", [])}


def update_summary_issue(
    github: GitHub,
    all_issues: list[dict],
    kind: str,
    scope: str,
    rows: list[dict[str, str]],
    run_url: str,
) -> None:
    label = SUMMARY_LABELS[kind]
    section = issue_section(scope, label, rows, run_url)
    candidates = [issue for issue in all_issues if label in labels(issue)]
    issue = next((item for item in candidates if item["state"] == "open"), None)
    if issue is None and not rows:
        return
    if issue is None:
        github.create_issue(
            f"[Cron] {label}", f"{HEADERS[kind]}\n\n{section}\n", [label]
        )
    else:
        body = replace_section(issue.get("body") or "", scope, section)
        github.update_issue(issue["number"], body=with_header(body, HEADERS[kind]))


def issue_reopenings(issue: dict, entry: dict) -> int:
    marker = REOPENINGS_RE.search(issue.get("body") or "")
    recorded = int(marker.group(1)) if marker else 0
    return max(recorded, int(entry.get("reopenings", 0)))


def scope_of(row: dict[str, str]) -> str:
    """The test environment to run for `row` (the first modeling of the key)."""
    return "comparison" if " vs " in row["modelings"] else row["modelings"]


def bug_body(
    row: dict[str, str],
    configurations: list[str],
    run_url: str,
    date: str,
    reopenings: int,
) -> str:
    key = issue_key(row)
    return (
        f"<!-- cron-key: {key} -->\n"
        f"<!-- cron-reopenings: {reopenings} -->\n"
        f"# Cron regression: `{row['problem']}` ({row['modelings']})\n\n"
        f"Last run: {run_url}\n"
        f"Last observed: {date}\n\n"
        "Failing configurations:\n\n"
        + "".join(f"- `{configuration}`\n" for configuration in configurations)
        + "\n**What to do:** open the run, reproduce locally with "
        f"`Pkg.test(; test_args=[\"{scope_of(row)}\", \"cron\", \"problem={row['problem']}\"])`, "
        "then fix the problem or the package, or set the problem to `:draft` in "
        "`ext/MetaData/<problem>.jl` if the configuration is not supported. This issue is "
        "updated by each cron run and closed automatically after two consecutive successful "
        f"runs. Details: [README]({README_URL}).\n"
    )


def matching_issue(all_issues: list[dict], key: str):
    for issue in all_issues:
        if BUG_LABEL not in labels(issue):
            continue  # an issue that merely quotes a key (e.g. a specification) is not ours
        match = KEY_RE.search(issue.get("body") or "")
        if match and match.group(1) == key:
            return issue
    return None


def update_bug_issues(
    github: GitHub,
    all_issues: list[dict],
    rows: list[dict[str, str]],
    failures: list[dict[str, str]],
    scope: str,
    run_url: str,
    date: str,
    state: dict,
    tests_failed: bool,
) -> None:
    grouped = group_rows(failures)
    passed = successful_keys(rows) if rows and not tests_failed else set()
    if len(grouped) <= MAX_BUG_ISSUES_PER_RUN:
        for key, problem_rows in grouped.items():
            row = problem_rows[0]
            issue = matching_issue(all_issues, key)
            if issue and issue.get("state_reason") == "not_planned":
                continue
            entry = state.setdefault(key, {"successes": 0, "reopenings": 0})
            entry["scope"] = scope
            entry["successes"] = 0
            configurations = sorted(row_configuration(row) for row in problem_rows)
            configurations_changed = entry.get("configurations") != configurations
            entry["configurations"] = configurations
            reopenings = issue_reopenings(issue, entry) if issue else entry.get("reopenings", 0)

            if issue is None:
                body = bug_body(row, configurations, run_url, date, reopenings)
                github.create_issue(
                    f"[Cron] Regression: {row['problem']} ({row['modelings']})",
                    body,
                    ["bug", "cron-bug"],
                )
                continue

            if issue["state"] == "closed":
                if reopenings < MAX_REOPENINGS:
                    reopenings += 1
                    entry["reopenings"] = reopenings
                    github.update_issue(issue["number"], state="open")
                    github.comment(
                        issue["number"], f"Regression detected again: {run_url}"
                    )
                if reopenings >= MAX_REOPENINGS:
                    github.add_labels(issue["number"], ["flaky"])
                body = bug_body(row, configurations, run_url, date, reopenings)
                github.update_issue(issue["number"], body=body)
            else:
                entry["reopenings"] = reopenings
                if configurations_changed:
                    github.comment(
                        issue["number"], f"Failing configurations changed: {run_url}"
                    )
                if reopenings >= MAX_REOPENINGS:
                    github.add_labels(issue["number"], ["flaky"])
                body = bug_body(row, configurations, run_url, date, reopenings)
                github.update_issue(issue["number"], body=body)

    for issue in all_issues:
        if BUG_LABEL not in labels(issue):
            continue
        match = KEY_RE.search(issue.get("body") or "")
        if not match or issue["state"] != "open" or issue.get("state_reason") == "not_planned":
            continue
        key = match.group(1)
        entry = state.get(key)
        if not entry or entry.get("scope") != scope or key in grouped:
            continue
        if key in passed and entry.get("successes", 0) >= 2:
            github.update_issue(
                issue["number"], state="closed", state_reason="completed"
            )
            github.comment(
                issue["number"], f"Two consecutive successful runs: {run_url}"
            )


def missing_report_row(scope: str) -> dict[str, str]:
    return {
        "test": "report",
        "problem": "workflow",
        "modelings": scope,
        "status": "valid",
        "outcome": "error",
        "platform": "workflow",
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reports-dir", type=pathlib.Path, required=True)
    parser.add_argument("--scope", required=True)
    parser.add_argument("--run-url", required=True)
    parser.add_argument("--state-file", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--sha", default="")
    parser.add_argument("--julia-version", default="")
    parser.add_argument("--package-version", default="")
    parser.add_argument("--tests-failed", action="store_true")
    parser.add_argument("--issues", action="store_true")
    parser.add_argument("--no-github", action="store_true")
    args = parser.parse_args()

    now = dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat()
    rows = parse_reports(args.reports_dir)
    results = parse_results(args.reports_dir)

    state = {}
    if args.state_file.exists():
        state = json.loads(args.state_file.read_text())
    previous = remember_results(state, args.scope, results, args.run_url)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        report_markdown(
            args.scope,
            rows,
            args.run_url,
            now,
            args.sha,
            args.julia_version,
            args.package_version,
            results,
            previous,
        )
    )

    grouped = findings(rows)
    failure_rows = grouped["failure"]
    if not rows:
        failure_rows = [missing_report_row(args.scope)]
    elif args.tests_failed and not failure_rows:
        failure_rows = [missing_report_row(args.scope)]

    if not args.issues:
        prepare_state(rows, args.scope, args.tests_failed, state)
        args.state_file.parent.mkdir(parents=True, exist_ok=True)
        args.state_file.write_text(json.dumps(state, indent=2, sort_keys=True) + "\n")
        return

    github_token = os.environ.get("GITHUB_TOKEN")
    repository = os.environ.get("GITHUB_REPOSITORY")
    if not args.no_github and github_token and repository:
        github = GitHub(repository, github_token)
        try:
            all_issues = github.issues()
            update_summary_issue(
                github, all_issues, "failure", args.scope, failure_rows, args.run_url
            )
            update_summary_issue(
                github, all_issues, "draft", args.scope, grouped["draft"], args.run_url
            )
            update_summary_issue(
                github,
                all_issues,
                "capability",
                args.scope,
                grouped["capability"],
                args.run_url,
            )
            update_summary_issue(
                github,
                all_issues,
                "convergence",
                args.scope,
                degradations(results, previous),
                args.run_url,
            )
            update_bug_issues(
                github,
                all_issues,
                rows,
                grouped["failure"],
                args.scope,
                args.run_url,
                now,
                state,
                args.tests_failed,
            )
        finally:
            args.state_file.parent.mkdir(parents=True, exist_ok=True)
            args.state_file.write_text(json.dumps(state, indent=2, sort_keys=True) + "\n")
    else:
        args.state_file.parent.mkdir(parents=True, exist_ok=True)
        args.state_file.write_text(json.dumps(state, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
