# Reference objectives

Each problem has a **reference objective** per scheme, at its default grid:
`ext/MetaData/references/<problem>.toml`, one table per scheme. These files are generated
by `update.jl` from the results of the convergence tests. Never edit them by hand.

The package reads them into the metadata (`problem_metadata(p)[:reference_objective]`), and
`OptimalControlProblems.reference_objective(p; grid_size, scheme)` returns the value for one
discretization, or `nothing`. The convergence report (`test/README.md`) shows the gap of each
solve to it.

## What a reference is

- **Value:** the best objective among the solves that converged, over every modeling,
  configuration and platform of the convergence tests, at the default grid. Every problem of
  the collection is a minimization.
- **`spread`:** the relative gap between the best and the worst of these objectives.
  **`agreement`** is `false` above `1e-6`: the configurations reached different solutions,
  for instance different local minima. Do not hide it: look at the `results` of the file,
  which list each solve.
- **`grid_size`, `parameters`:** the default grid and parameters of the problem when the
  reference was computed. If either changes, the consistency tests fail. Compute the
  reference again.
- **`run`:** where the results come from (a cron run, or `local`). The package versions are
  in the logs of that run.

## Computing the references

From the cron runs (one per environment: JuMP, OptimalControl, OptimalControlScalarized),
on `main` or on a branch with `workflow_dispatch`:

```bash
gh run download <id> --dir results/<id>      # for each run
julia --project=. tools/references/update.jl --run <url of a run> results/*
```

Or locally, after the convergence tests:

```julia
using Pkg
Pkg.test(; test_args=["cron"])                         # every environment, or
Pkg.test(; test_args=["OptimalControl", "cron", "problem=beam"])
```

```bash
julia --project=. tools/references/update.jl test/reports
```

The script writes the problems it has converged results for, and keeps the other schemes of
an existing file. It prints one line per problem and scheme, and flags disagreements. Commit
the files and say in the pull request where the results come from.

## When to compute them again

- A problem's default grid or parameters change: the consistency tests say which ones.
- The cron report shows an objective change that you accept, for example a better local
  solution reached with a new initial guess.
- A new problem, or a new scheme in the convergence tests (#284).
