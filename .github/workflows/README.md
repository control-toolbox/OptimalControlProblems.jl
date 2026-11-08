# Cron workflows

The cron workflows run the slow tests once per scope, independently of the pull request
scope labels. They are staggered at 04:00 UTC:

| Workflow | Scope | Trigger | GPU |
| --- | --- | --- | --- |
| `CronJuMP.yml` | `JuMP` | Tuesday | No |
| `CronOptimalControl.yml` | `OptimalControl` | Wednesday | Yes |
| `CronOptimalControlScalarized.yml` | `OptimalControlScalarized` | Thursday | Yes |
| `CronModelingToolkit.yml` | `ModelingToolkit` | manual only until the scope exists | No |
| `CronRobustness.yml` | `JuMP-robustness`, `OptimalControl-robustness` | 1st of the month | No |

Each thin workflow calls `cron-reusable.yml` (inputs `mode`: `cron`, or `robustness` for
`test/robustness/`, #378; `scope`: the environment by default; `note`: one line describing
the scope, shown under its name in the reports and the issues). The reusable workflow runs the CPU tests
(Ubuntu and Windows on Julia 1.13; macOS comes back with #344),
and, where requested, the GPU tests. It then downloads the report artifacts without
merging them, updates the summary and regression issues, and publishes `cron/<scope>.md`
and `cron/state.json` on the `results` branch. The first run creates `results`; each run
then rewrites its tip as one orphan commit containing every published result, while the
`publish` job is serialized to keep a single writer.

## Reports and issues

The test environments write reports under `test/reports/`. The reusable workflow uploads
those files with the CTActions artifact interface and downloads one directory per
artifact, so the artifact name (Julia version, runner, CPU/GPU) identifies the platform
of every report row. The final job uses the GitHub token to:

- update one repository-wide issue for each of `cron-failure`, `cron-drafts` and
  `cron-convergence` — one section per scope (with its focus, cadence/`note`, its run, the
  previous run and a bounded history), an `Overall Scope Status` dashboard table on top, a
  finding count in the title, and a comment whenever the finding set of a scope changes
  (`cron-convergence` stays open permanently as a dashboard);
- add the results of the convergence tests (#360) to the scope report, with their change
  since the previous run of the scope (kept in the state on the `results` branch);
- maintain one idempotent `cron-bug` issue per failing problem and modeling;
- reopen completed regressions at most three times, label them `flaky`, and close a
  regression after two consecutive runs where every observed valid row passes;
- report a missing report artifact as a failure rather than as a success;
- commit the scope report and state to the `results` branch, with the results of the
  solves for the documentation (#314, below).

The `cron-capabilities` transition is intentionally not active yet: it depends on the
`:known_failures` metadata added by #306. Until then no capability entries can be
produced and the README workflow keeps the placeholder label only.

The workflow requires the `upload_artifact_path` and `upload_artifact_name` inputs in
the CTActions `ci.yml` and `ci-gpu.yml` reusable workflows. CTActions must be merged
before these cron workflows are enabled or dispatched against `main`: calling a reusable
workflow with an undeclared input fails at workflow validation.

## Results for the documentation

The `publish` job also writes, from the reports of the run, what the problem pages of the
documentation draw (#314):

| Path on `results` | Content |
| --- | --- |
| `manifest.json` | The last run of each scope: run, commit, date, platforms, problems. |
| `problems/<problem>/result.json` | Every solve of the problem, all scopes: the columns of `<report>-results.tsv` (fingerprint of the problem included), with the run, its commit and the paths below. |
| `problems/<problem>/<scope>/<platform>/<modeling>/<configuration>.json` | The trajectory of a solve: time grid, state, control and costate by component, variable. |
| `env/<run id>/<platform>/<report>/` | The environment of the solves: `environment.toml`, `Project.toml`, `Manifest.toml`, `versioninfo.txt`, `status*.txt`. |

A run replaces, for each problem it solved, the entries of its scope and keeps the others:
a run restricted to some problems (`problem=`) leaves the other problems as they were. An
environment is removed once no entry refers to it. Everything is written by
`tools/cron/report.py` (`--results-dir`), tested in `tools/cron/test/`.

A convergence cron dispatched by hand takes an optional `problems` input (`beam` or
`beam,chain`): it solves those problems only, and publishes their results and environments.
Its report and its state are written aside, unpublished, and the issues are left as they
are: a restricted run cannot tell about the problems it did not solve, so `cron/<scope>.md`,
`cron/state.json` and the issues stay those of the last complete run. Use it to check a
change of the pipeline, or to refresh one problem after changing it.

## What to do with the cron issues

Four repository-wide issues, each with one section per scope (replaced by each run of that
scope), and one `cron-bug` issue per failing (problem, modeling). All of them explain what
to do in their own text; this is the longer version. **Nothing is changed automatically:** the
cron only reports. A maintainer decides what to do (and may ask an agent to do it); in
particular a draft becomes `:valid`, or a failing problem becomes `:draft`, only by a pull
request that edits `ext/MetaData/<problem>.jl`.

- **`cron-failure`** — a `:valid` problem failed in a scope (or the run failed or produced no
  report). Open the run, read the failing configuration (platform and test), reproduce it
  locally with `Pkg.test(; test_args=["<scope>", "cron", "problem=<problem>"])`, then fix the
  problem or the package, report an upstream break, or — if the configuration is not
  supported — set the problem to `:draft` in `ext/MetaData/<problem>.jl`.
- **`cron-drafts`** — a `:draft` passed all its tests on every platform. Check it, then set
  `:valid` for that modeling in `ext/MetaData/<problem>.jl` in a pull request.
- **`cron-convergence`** — convergence is reported, not tested (#360): a solve converged in
  the previous run of the scope and no longer does, its objective moved by more than
  `OBJECTIVE_CHANGE` (relative, `tools/cron/report.py`), or it reached a solution that is not
  known yet (`new solution`, #378: a new local solution, or one better than the reference;
  compute the references again if it is to be kept). Nothing fails. Open the run and the
  previous one, both linked in the section (the `results` branch is rewritten each run;
  the previous values live in `cron/state.json`), and decide whether the change is
  expected (new initial guess, new solver version) or a regression to investigate. An
  expected change of objective means computing the reference objectives again
  ([`tools/references/README.md`](../../tools/references/README.md)).
- **`cron-capabilities`** — a declared known failure no longer happens (inactive until the
  `:known_failures` metadata of #306 exists).
- **`cron-bug`** — opened per (problem, modeling) for at most 5 pairs per run (more is
  reported in `cron-failure` as a probable upstream breakage). Updated by each run; a comment
  is added only when the set of failing configurations changes; closed automatically after
  two consecutive successful runs; reopened at most three times, then labelled `flaky`.
  Closing one as *not planned* is respected: the cron leaves it alone.

## Adding a scope

1. Add a `test/suite/<scope>/Project.toml` environment and, when appropriate, a
   `test/cron/<scope>/` directory.
2. Add the scope to `test/runtests.jl` if it is opt-in.
3. Add a thin `Cron<Scope>.yml` workflow with a free UTC slot, and call
   `cron-reusable.yml` — set `note:` to one line saying what the scope is (cadence,
   platforms, CPU/GPU); it is shown under the scope name in the reports and the issues.
4. Set `gpu: true` only when the scope has GPU-capable tests.
5. Run the workflow manually twice: once with a deliberately failing fixture and once
   after fixing it. Verify issue updates and the `results` branch report.
