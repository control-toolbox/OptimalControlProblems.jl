# Cron workflows

The cron workflows run the slow tests once per scope, independently of the pull request
scope labels. They are staggered at 04:00 UTC:

| Workflow | Scope | Trigger | GPU |
| --- | --- | --- | --- |
| `CronJuMP.yml` | `JuMP` | Tuesday | No |
| `CronOptimalControl.yml` | `OptimalControl` | Wednesday | Yes |
| `CronOptimalControlScalarized.yml` | `OptimalControlScalarized` | Thursday | Yes |
| `CronComparison.yml` | `comparison` | Friday | No |
| `CronModelingToolkit.yml` | `ModelingToolkit` | manual only until the scope exists | No |

Each thin workflow calls `cron-reusable.yml`. The reusable workflow runs the CPU tests
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
  `cron-convergence`;
- add the results of the convergence tests (#360) to the scope report, with their change
  since the previous run of the scope (kept in the state on the `results` branch);
- maintain one idempotent `cron-bug` issue per failing problem and modeling;
- reopen completed regressions at most three times, label them `flaky`, and close a
  regression after two consecutive runs where every observed valid row passes;
- report a missing report artifact as a failure rather than as a success;
- commit the scope report and state to the `results` branch.

The `cron-capabilities` transition is intentionally not active yet: it depends on the
`:known_failures` metadata added by #306. Until then no capability entries can be
produced and the README workflow keeps the placeholder label only.

The workflow requires the `upload_artifact_path` and `upload_artifact_name` inputs in
the CTActions `ci.yml` and `ci-gpu.yml` reusable workflows. CTActions must be merged
before these cron workflows are enabled or dispatched against `main`: calling a reusable
workflow with an undeclared input fails at workflow validation.

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
  the previous run of the scope and no longer does, or its objective moved by more than
  `OBJECTIVE_CHANGE` (relative, `tools/cron/report.py`). Nothing fails. Compare the scope
  report with the previous one on the `results` branch and decide whether the change is
  expected (new initial guess, new solver version) or a regression to investigate.
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
   `cron-reusable.yml`.
4. Set `gpu: true` only when the scope has GPU-capable tests.
5. Run the workflow manually twice: once with a deliberately failing fixture and once
   after fixing it. Verify issue updates and the `results` branch report.
