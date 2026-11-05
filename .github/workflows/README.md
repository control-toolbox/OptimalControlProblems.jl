# Cron workflows

The cron workflows run the slow tests once per scope, independently of the pull request
scope labels. They are staggered at 04:00 UTC:

| Workflow | Scope | Day | GPU |
| --- | --- | --- | --- |
| `CronJuMP.yml` | `JuMP` | Tuesday | No |
| `CronOptimalControl.yml` | `OptimalControl` | Wednesday | Yes |
| `CronOptimalControlScalarized.yml` | `OptimalControlScalarized` | Thursday | Yes |
| `CronComparison.yml` | `comparison` | Friday | No |
| `CronModelingToolkit.yml` | `ModelingToolkit` | Saturday | Not yet enabled |

Each thin workflow calls `cron-reusable.yml`. The reusable workflow runs the CPU tests,
and, where requested, the GPU tests. It then downloads the report artifacts, updates the
summary and regression issues, and publishes `cron/<scope>.md` and the state file on the
`results` branch.

## Reports and issues

The test environments write reports under `test/reports/`. The reusable workflow keeps
those files out of the package branch and uploads them as artifacts. The final job uses
the GitHub token to:

- update one repository-wide issue for each of `cron-failure`, `cron-drafts`, and
  `cron-capabilities`;
- maintain one idempotent `cron-bug` issue per failing problem and modeling;
- reopen completed regressions, stop reopening flaky regressions after three reopenings,
  and close a regression after two consecutive successful runs;
- commit the scope report and state to the `results` branch.

The workflow requires the `upload_artifact_path` and `upload_artifact_name` inputs in
the CTActions `ci.yml` and `ci-gpu.yml` reusable workflows. Until that CTActions change
is merged, the cron jobs can be dispatched but their report collection is incomplete.

## Adding a scope

1. Add a `test/suite/<scope>/Project.toml` environment and, when appropriate, a
   `test/cron/<scope>/` directory.
2. Add the scope to `test/runtests.jl` if it is opt-in.
3. Add a thin `Cron<Scope>.yml` workflow with a free UTC slot, and call
   `cron-reusable.yml`.
4. Set `gpu: true` only when the scope has GPU-capable tests.
5. Run the workflow manually twice: once with a deliberately failing fixture and once
   after fixing it. Verify issue updates and the `results` branch report.
