#!/usr/bin/env python3
"""Publish one cron scope report and maintain its GitHub issue state.

With `--results-dir`, also publish the results of the solves for the documentation (#314):
`problems/<problem>/result.json`, the trajectories, the environments of the run and
`manifest.json`, on the `results` branch."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import pathlib
import re
import shutil
import time
import tomllib
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
STATUS_MARKER = "<!-- cron-status: {count} -->"
STATUS_RE = re.compile(r"<!-- cron-scope: (.+?) -->\s*<!-- cron-status: (\d+) -->")
KEY_RE = re.compile(r"<!-- cron-key: (.+?) -->")
REOPENINGS_RE = re.compile(r"<!-- cron-reopenings: (\d+) -->")
HISTORY_MARKER = "<!-- cron-history -->"
HISTORY_END = "<!-- /cron-history -->"
MAX_HISTORY = 10
REPOSITORY_URL = "https://github.com/control-toolbox/OptimalControlProblems.jl"
README_URL = (
    f"{REPOSITORY_URL}/blob/main/"
    ".github/workflows/README.md#what-to-do-with-the-cron-issues"
)
# The singular wording of one finding of each summary issue, for the status line,
# the summary table and the issue title.
STATUS_WORD = {
    "failure": "failure",
    "draft": "ready draft",
    "capability": "finding",
    "convergence": "finding",
}
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
    "**What to do:** open the run and the previous one (both linked in the section), and "
    "decide whether the change is expected (new initial guess, new solver version) or a "
    f"regression to investigate. Details: [README]({README_URL}). This issue updates itself; "
    "close it when the sections say `No findings.`"
)
# The tests that report solves instead of testing them (`cron/<env>/test_convergence.jl`,
# `robustness/<env>/test_robustness.jl`, #360, #378): they always pass and never make a draft
# ready.
REPORTING_TESTS = ("test_convergence", "test_robustness")
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


# Columns of `<report>-results.tsv` the report needs, by name: the file may have more (grid and
# scheme since #375, the fingerprint of the problem since #314), in any order, all kept.
# `solution` (#378: `best`, `local` or `new`) is empty when absent.
RESULT_COLUMNS = (
    "test", "problem", "modeling", "configuration", "result", "reason", "objective",
    "iterations", "time", "gap",
)
OPTIONAL_RESULT_COLUMNS = ("solution",)


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
            row = {column: "" for column in OPTIONAL_RESULT_COLUMNS}
            row.update(zip(header, cells))
            row["platform"] = platform
            row["report"] = path.name.removesuffix("-results.tsv")
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


def remember_run(state: dict, scope: str, run_url: str) -> str:
    """Store this run of `scope` in `state` and return the previous run ("" if none).

    Like `remember_results`: the previous run only shifts when the run changes, so the
    report and issues calls of the same run link the same previous run."""
    entry = state.setdefault("runs", {}).setdefault(scope, {})
    if entry.get("run") != run_url:
        entry["previous"] = entry.get("run", "")
        entry["run"] = run_url
    return entry.get("previous", "")


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


def change_text(what: str, row: dict[str, str], previous: dict[str, dict[str, str]]) -> str:
    """The display text of a `change`: `converged → <result>`, `objective a → b (±x)`,
    `new solution`, joined when several apply."""
    before = previous.get(result_key(row), {})
    parts = []
    for part in what.split(", "):
        if part == "lost":
            parts.append(f"converged → {row['result']}")
        elif part.startswith("objective"):
            delta = part.removeprefix("objective ")
            parts.append(
                f"objective {short(before.get('objective', '?'))} → "
                f"{short(row['objective'])} ({delta})"
            )
        else:
            parts.append(part)
    return ", ".join(parts)


def degradations(
    results: list[dict[str, str]], previous: dict[str, dict[str, str]]
) -> list[dict[str, str]]:
    """Results to look at, as rows of a summary issue: lost convergence, a converged objective
    that moved (a move is not better or worse without a reference), or a solve that reached a
    solution not known yet (#378: a new local solution, or a better one than the reference)."""
    rows = []
    for row in results:
        what = change(row, previous)
        if row.get("solution") == "new" and row["result"] == "converged":
            what = ", ".join(w for w in (what, "new solution") if w and w != "new")
        if (
            what == "lost"
            or (what.startswith("objective") and row["result"] == "converged")
            or "new solution" in what
        ):
            rows.append(
                {
                    "platform": row["platform"],
                    "problem": row["problem"],
                    "modelings": row["modeling"],
                    "test": row["configuration"],
                    "outcome": change_text(what, row, previous),
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
        if row["status"] == "draft" and not row["test"].startswith(REPORTING_TESTS)
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
    note: str = "",
) -> str:
    lines = [
        f"# Cron report: {scope}",
        "",
    ]
    if note:
        lines += [f"<sub>{note}</sub>", ""]
    lines += [
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
                "| iterations | time | gap | solution | change |",
                "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
            ]
        )
        lines.extend(
            "| {platform} | {problem} | {modeling} | {configuration} | {result} | {reason} "
            "| {objective} | {iterations} | {time} | {gap} | {solution} | ".format(
                **{**row, "objective": short(row["objective"])}
            )
            + change(row, previous)
            + " |"
            for row in results
        )
    return "\n".join(lines) + "\n"


def short_platform(platform: str, scope: str) -> str:
    """The platform without the `cron-<scope>-` artifact prefix (issue tables only)."""
    return platform.removeprefix(f"cron-{scope}-")


def problem_cell(kind: str, problem: str) -> str:
    """The problem name, linked where a link helps: its results for a convergence finding,
    its metadata file (the file to edit) for the other kinds."""
    if problem == "workflow":  # the missing-report row of a failed run
        return problem
    if kind == "convergence":
        return f"[{problem}]({REPOSITORY_URL}/blob/results/problems/{problem}/result.json)"
    return f"[{problem}]({REPOSITORY_URL}/blob/main/ext/MetaData/{problem}.jl)"


def investigate(scope: str, kind: str, rows: list[dict[str, str]]) -> str:
    """The `<details>` hint block of a section with findings (empty when there is nothing
    to reproduce: a ready draft or a gone known failure is handled in the metadata file,
    already linked in the table)."""
    if kind not in ("failure", "convergence"):
        return ""
    env = scope.removesuffix("-robustness")
    mode = "robustness" if scope.endswith("-robustness") else "cron"
    repro = f'`Pkg.test(; test_args=["{env}", "{mode}", "problem={rows[0]["problem"]}"])`'
    legend = {
        "failure": "a `cron-bug` issue may exist for this (problem, modeling)",
        "convergence": (
            "outcomes: `converged → <result>` (converged in the previous run, not now), "
            "`objective a → b (±x)` (relative move above `OBJECTIVE_CHANGE`), "
            "`new solution` (#378)"
        ),
    }[kind]
    return (
        "<details><summary>Investigate</summary>\n\n"
        f"Reproduce: {repro} · {legend}\n\n"
        "</details>"
    )


def history_markdown(entries: list[str]) -> str:
    """The bounded `<details>` history block of a section."""
    return "\n".join(
        [
            "<details><summary>History</summary>",
            HISTORY_MARKER,
            *entries,
            HISTORY_END,
            "</details>",
        ]
    )


def issue_section(
    scope: str,
    kind: str,
    rows: list[dict[str, str]],
    run_url: str,
    generated: str = "",
    previous_run: str = "",
    note: str = "",
    history: list[str] | None = None,
) -> str:
    """One scope section of a summary issue: `## <scope>` (a stable anchor), a `<sub>`
    reminder of what the scope is, a status line with the run, the findings table."""
    word = STATUS_WORD[kind]
    status = f"**⚠️ {len(rows)} {word}{'s' * (len(rows) != 1)}**" if rows else "**✅ clean**"
    bits = [status, f"Run [{run_id(run_url)}]({run_url})"]
    if generated:
        bits.append(generated[:16].replace("T", " "))
    bits.append(f"[scope report]({REPOSITORY_URL}/blob/results/cron/{scope}.md)")
    if previous_run:
        bits.append(f"[previous run]({previous_run})")
    lines = [
        SCOPE_MARKER.format(scope=scope),
        STATUS_MARKER.format(count=len(rows)),
        f"## {scope}",
        "",
    ]
    if note:
        lines += [f"<sub>{note}</sub>", ""]
    lines += [" · ".join(bits), ""]
    if rows:
        outcome_header = "change" if kind == "convergence" else "outcome"
        lines += [
            f"| platform | problem | modeling | test | {outcome_header} |",
            "| --- | --- | --- | --- | --- |",
        ]
        lines += [
            f"| {short_platform(row.get('platform', ''), scope)} | "
            f"{problem_cell(kind, row['problem'])} | {row['modelings']} | "
            f"{row['test']} | {row['outcome']} |"
            for row in rows
        ]
        hint = investigate(scope, kind, rows)
        if hint:
            lines += ["", hint]
    else:
        lines.append("No findings.")
    if history:
        lines += ["", history_markdown(history)]
    return "\n".join(lines)


def section_statuses(body: str) -> list[tuple[str, int]]:
    """`(scope, number of findings)` of every section of `body`, in order."""
    return [(scope, int(count)) for scope, count in STATUS_RE.findall(body)]


def summary_markdown(body: str, kind: str) -> str:
    """The table of contents of the scopes, regenerated inside the header region by each
    run: findings first, clean scopes grouped on one line."""
    statuses = section_statuses(body)
    word = STATUS_WORD[kind]
    flagged = [f"[{scope}](#{scope.lower()}) | ⚠️ {n} {word}{'s' * (n != 1)}" for scope, n in statuses if n]
    clean = " · ".join(f"[{scope}](#{scope.lower()})" for scope, n in statuses if not n)
    lines = ["| Scope | Status |", "| --- | --- |"]
    lines += [f"| {line} |" for line in flagged]
    if clean:
        lines.append(f"| {clean} | ✅ clean |")
    return "\n".join(lines)


def section_bounds(body: str, scope: str) -> tuple[int, int]:
    """`body[start:end]` is the section of `scope`; `(-1, -1)` when there is none."""
    marker = SCOPE_MARKER.format(scope=scope)
    start = body.find(marker)
    if start < 0:
        return -1, -1
    next_marker = re.search(r"\n<!-- cron-scope: [^ ]+ -->", body[start + len(marker) :])
    end = start + len(marker) + (next_marker.start() if next_marker else len(body))
    return start, end


def unwrap_link(cell: str) -> str:
    return re.sub(r"\[(.+?)\]\([^)]*\)", r"\1", cell).strip()


def section_rows(body: str, scope: str) -> list[dict[str, str]]:
    """The finding rows of the section of `scope` in `body` (for the diff between runs)."""
    start, end = section_bounds(body, scope)
    if start < 0:
        return []
    rows = []
    for line in body[start:end].splitlines():
        if not line.startswith("|") or line.startswith("| ---"):
            continue
        cells = [unwrap_link(cell) for cell in line.strip("|").split("|")]
        if len(cells) != 5 or cells[0] == "platform":
            continue
        rows.append(
            {
                "platform": cells[0],
                "problem": cells[1],
                "modelings": cells[2],
                "test": cells[3],
                "outcome": cells[4],
            }
        )
    return rows


def section_history(body: str, scope: str) -> list[str]:
    """The carried-over history entries of the section of `scope`."""
    start, end = section_bounds(body, scope)
    if start < 0:
        return []
    section = body[start:end]
    begin = section.find(HISTORY_MARKER)
    if begin < 0:
        return []
    stop = section.find(HISTORY_END, begin)
    block = section[begin + len(HISTORY_MARKER) : stop if stop >= 0 else len(section)]
    return [line for line in block.splitlines() if line.startswith("- ")]


def finding_key(row: dict[str, str]) -> tuple:
    """What identifies a finding between two runs (platform shown, i.e. without prefix)."""
    return (row.get("platform", ""), row["problem"], row["modelings"], row["test"], row["outcome"])


def findings_diff(
    scope: str, old: list[dict[str, str]], new: list[dict[str, str]]
) -> tuple[list[dict[str, str]], list[dict[str, str]]]:
    """`(gained, resolved)` findings of `scope`: `new` rows use full platform names, `old`
    rows the shown (stripped) ones — the keys are built on the shown form."""
    shown = [
        {**row, "platform": short_platform(row.get("platform", ""), scope)} for row in new
    ]
    old_keys = {finding_key(row) for row in old}
    new_keys = {finding_key(row) for row in shown}
    gained = [row for row in shown if finding_key(row) not in old_keys]
    resolved = [row for row in old if finding_key(row) not in new_keys]
    return gained, resolved


def finding_brief(row: dict[str, str]) -> str:
    return f"`{row['problem']}` / {row['modelings']} / `{row['test']}` — {row['outcome']}"


def change_entry(generated: str, run_url: str, gained, resolved) -> str:
    """One history entry (a markdown list line) of a change of the finding set."""
    parts = []
    if gained:
        parts.append("new: " + ", ".join(f"`{row['problem']}` ({row['outcome']})" for row in gained))
    if resolved:
        parts.append(
            "resolved: "
            + ", ".join(f"`{row['problem']}`" for row in resolved)
            + (" (all clear)" if not gained else "")
        )
    return f"- **{generated[:10] or '?'}** · [run]({run_url}) — " + "; ".join(parts)


def change_comment(scope: str, run_url: str, gained, resolved) -> str:
    """The issue comment of a change of the finding set (notification + timeline)."""
    if not gained:
        return f"**{scope}** — all clear ([run]({run_url})); safe to close when every scope is."
    lines = [f"**{scope}** — findings changed in [run]({run_url}):"]
    lines += [f"- new: {finding_brief(row)}" for row in gained]
    lines += [f"- resolved: {finding_brief(row)}" for row in resolved]
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


def issue_title(label: str, total: int, kind: str) -> str:
    word = STATUS_WORD[kind]
    suffix = f" — {total} {word}{'s' * (total != 1)}" if total else ""
    return f"[Cron] {label}{suffix}"


def update_summary_issue(
    github: GitHub,
    all_issues: list[dict],
    kind: str,
    scope: str,
    rows: list[dict[str, str]],
    run_url: str,
    generated: str = "",
    previous_run: str = "",
    note: str = "",
) -> None:
    label = SUMMARY_LABELS[kind]
    candidates = [issue for issue in all_issues if label in labels(issue)]
    issue = next((item for item in candidates if item["state"] == "open"), None)
    if issue is None and not rows:
        return
    if issue is None:
        history = [change_entry(generated, run_url, rows, [])]
        section = issue_section(
            scope, kind, rows, run_url, generated, previous_run, note, history
        )
        body = f"{HEADERS[kind]}\n\n{section}\n"
        github.create_issue(
            issue_title(label, len(rows), kind),
            with_header(body, f"{HEADERS[kind]}\n\n{summary_markdown(body, kind)}"),
            [label],
        )
        return

    old_body = issue.get("body") or ""
    gained, resolved = findings_diff(scope, section_rows(old_body, scope), rows)
    history = section_history(old_body, scope)
    if gained or resolved:
        entry = change_entry(generated, run_url, gained, resolved)
        history = [entry] + history[: MAX_HISTORY - 1]
        github.comment(issue["number"], change_comment(scope, run_url, gained, resolved))
    section = issue_section(
        scope, kind, rows, run_url, generated, previous_run, note, history
    )
    body = replace_section(old_body, scope, section)
    total = sum(count for _, count in section_statuses(body))
    header = f"{HEADERS[kind]}\n\n{summary_markdown(body, kind)}"
    github.update_issue(
        issue["number"], title=issue_title(label, total, kind), body=with_header(body, header)
    )


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


# -----------------------------------
# Results for the documentation (#314)
# -----------------------------------
# The `results` branch keeps, next to `cron/`:
#
#   manifest.json                          the last run of each scope
#   problems/<problem>/result.json         every solve of the problem, all scopes
#   problems/<problem>/<scope>/<platform>/<modeling>/<configuration>.json   its trajectory
#   env/<run id>/<platform>/<report>/      the environment of the solves of a run
#
# A run replaces, for each problem it solved, the entries of its scope and keeps the others:
# each scope publishes on its own, and a run restricted to some problems (`problem=`) leaves
# the other problems as they were. An environment is kept while an entry refers to it.

# Columns of `<report>-results.tsv` that are numbers in `result.json`.
INTEGER_COLUMNS = ("iterations", "grid_size")
FLOAT_COLUMNS = ("objective", "time", "gap", "violation")


def slug(text: str) -> str:
    """`modeler=adnlp solver=ipopt` → `modeler-adnlp_solver-ipopt`: a file name."""
    return re.sub(r"[^A-Za-z0-9._-]+", "-", text.replace(" ", "_").replace("=", "-")).strip("-")


def number(value: str, kind: type):
    try:
        return kind(value)
    except ValueError:
        return None


def solve_entry(row: dict[str, str]) -> dict:
    """An entry of `result.json`: the columns of the results, numbers as numbers."""
    entry: dict = {k: v for k, v in row.items() if k not in ("test", "report")}
    for column in INTEGER_COLUMNS:
        if column in entry:
            entry[column] = number(entry[column], int)
    for column in FLOAT_COLUMNS:
        if column in entry:
            entry[column] = number(entry[column], float)
    return entry


def parse_trajectories(directory: pathlib.Path) -> dict[str, dict]:
    """The trajectories of the solves (`<report>-trajectories.toml`, #314), by `result_key`."""
    trajectories = {}
    for path in sorted(directory.rglob("*-trajectories.toml")):
        relative = path.relative_to(directory)
        platform = relative.parts[0] if len(relative.parts) > 1 else "local"
        for solve in tomllib.loads(path.read_text()).get("solve", []):
            row = {**solve, "platform": platform}
            trajectories[result_key(row)] = solve["trajectory"]
    return trajectories


def run_id(run_url: str) -> str:
    return run_url.rstrip("/").rsplit("/", 1)[-1] or "local"


def environment_path(row: dict[str, str], run: str) -> str:
    return f"env/{run}/{row['platform']}/{row['report']}"


def copy_environment(
    directory: pathlib.Path, results_dir: pathlib.Path, row: dict[str, str], run: str
) -> bool:
    """Copy the environment of the report of `row` (`<report>-environment.toml` and
    `<report>-env/`) to `env/<run id>/<platform>/<report>/`; false when there is none."""
    base = directory / row["platform"] if row["platform"] != "local" else directory
    toml = base / f"{row['report']}-environment.toml"
    files = base / f"{row['report']}-env"
    if not toml.is_file():
        return False
    target = results_dir / environment_path(row, run)
    if not target.exists():
        if files.is_dir():
            shutil.copytree(files, target)
        target.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(toml, target / "environment.toml")
    return True


def read_json(path: pathlib.Path, default):
    return json.loads(path.read_text()) if path.is_file() else default


def write_json(path: pathlib.Path, data, indent: int | None = 2) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    separators = None if indent else (",", ":")
    path.write_text(json.dumps(data, indent=indent, separators=separators, sort_keys=True) + "\n")


def publish_results(
    directory: pathlib.Path,
    results_dir: pathlib.Path,
    scope: str,
    results: list[dict[str, str]],
    run_url: str,
    sha: str,
    generated: str,
) -> None:
    """Write the results of this run of `scope` to the `results` tree `results_dir`."""
    trajectories = parse_trajectories(directory)
    run = run_id(run_url)
    by_problem: dict[str, list[dict[str, str]]] = {}
    for row in results:
        by_problem.setdefault(row["problem"], []).append(row)

    for problem, rows in sorted(by_problem.items()):
        folder = results_dir / "problems" / problem
        shutil.rmtree(folder / scope, ignore_errors=True)
        data = read_json(folder / "result.json", {"problem": problem, "entries": []})
        entries = [e for e in data["entries"] if e.get("scope") != scope]
        for row in rows:
            entry = solve_entry(row)
            entry.update(scope=scope, run=run_url, commit=sha, date=generated)
            entry["environment"] = (
                environment_path(row, run)
                if copy_environment(directory, results_dir, row, run)
                else None
            )
            trajectory = trajectories.get(result_key(row))
            entry["trajectory"] = None
            if trajectory is not None:
                relative = (
                    f"problems/{problem}/{scope}/{slug(row['platform'])}/"
                    f"{slug(row['modeling'])}/{slug(row['configuration'])}.json"
                )
                write_json(results_dir / relative, trajectory, indent=None)
                entry["trajectory"] = relative
            entries.append(entry)
        entries.sort(key=lambda e: (e["scope"], e["modeling"], e["configuration"], e["platform"]))
        write_json(folder / "result.json", {"problem": problem, "entries": entries})

    manifest = read_json(results_dir / "manifest.json", {"scopes": {}})
    manifest["scopes"][scope] = {
        "run": run_url,
        "commit": sha,
        "date": generated,
        "platforms": sorted({row["platform"] for row in results}),
        "problems": sorted(by_problem),
    }
    manifest["updated"] = generated
    write_json(results_dir / "manifest.json", manifest)
    prune_environments(results_dir)


def prune_environments(results_dir: pathlib.Path) -> None:
    """Remove the environments of the runs no entry of `problems/` refers to any more."""
    used = {
        entry["environment"].split("/")[1]
        for path in (results_dir / "problems").glob("*/result.json")
        for entry in read_json(path, {"entries": []})["entries"]
        if entry.get("environment")
    }
    env = results_dir / "env"
    if env.is_dir():
        for folder in env.iterdir():
            if folder.name not in used:
                shutil.rmtree(folder)


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
    parser.add_argument("--results-dir", type=pathlib.Path)
    parser.add_argument("--sha", default="")
    parser.add_argument("--julia-version", default="")
    parser.add_argument("--package-version", default="")
    parser.add_argument("--tests-failed", action="store_true")
    parser.add_argument("--issues", action="store_true")
    parser.add_argument("--no-github", action="store_true")
    parser.add_argument("--scope-note", default="")
    args = parser.parse_args()

    now = dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat()
    rows = parse_reports(args.reports_dir)
    results = parse_results(args.reports_dir)

    state = {}
    if args.state_file.exists():
        state = json.loads(args.state_file.read_text())
    previous = remember_results(state, args.scope, results, args.run_url)
    previous_run = remember_run(state, args.scope, args.run_url)

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
            note=args.scope_note,
        )
    )
    if args.results_dir is not None and results:
        publish_results(
            args.reports_dir, args.results_dir, args.scope, results, args.run_url, args.sha, now
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
            for kind, kind_rows in (
                ("failure", failure_rows),
                ("draft", grouped["draft"]),
                ("capability", grouped["capability"]),
                ("convergence", degradations(results, previous)),
            ):
                update_summary_issue(
                    github,
                    all_issues,
                    kind,
                    args.scope,
                    kind_rows,
                    args.run_url,
                    generated=now,
                    previous_run=previous_run,
                    note=args.scope_note,
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
