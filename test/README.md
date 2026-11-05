# Tests of OptimalControlProblems.jl

How the tests are organized, run and extended. The tests themselves are discovered from
the files: this page lists none of them.

## Layout

```text
test/
├── runtests.jl     # dispatcher: runs core here, then each environment in a subprocess
├── Project.toml    # light environment of the dispatcher and of core (Test, Aqua, CTBase)
├── common/         # shared code, included by the tests (configurations, statuses, solvers)
├── suite/<group>/  # fast tests, run on every pull request
└── cron/<group>/   # slow tests of the same group, run by the cron
```

- `suite/core/` has no extension loaded and runs in the dispatcher's process, always.
- Every other directory of `suite/` that holds a `Project.toml` is a **test environment**
  (one per modeling, plus `comparison` where every modeling is loaded). It runs in its
  own Julia process, in a temporary copy of its `Project.toml` where the package is
  developed, so that each extension is shown to load alone.
- `cron/<group>/` holds the slow tests of a group; they run in the environment of
  `suite/<group>/`.

## Running the tests

```julia
using Pkg
Pkg.test()                                              # core and every environment
Pkg.test(; test_args=["JuMP"])                          # core and one environment
Pkg.test(; test_args=["core"])                          # core only
Pkg.test(; test_args=["cron"])                          # slow tests (cron/), every environment
Pkg.test(; test_args=["OptimalControl", "convergence"]) # suite and convergence report
Pkg.test(; test_args=["JuMP", "cron", "*convergence*"]) # environments, mode and file globs
Pkg.test(; test_args=["problem=beam"])                  # one problem, in every environment
Pkg.test(; test_args=["JuMP", "problem=beam,robot"])    # some problems, in one environment
```

`test_args` is a list of:

- environment names: run those only; none given runs every environment except the opt-in
  ones (`OPT_IN` in `runtests.jl`);
- `cron`: run `cron/<group>/test_*.jl` instead of `suite/<group>/test_*.jl` (what the
  cron workflows run);
- `convergence`: run `suite/<group>/test_*.jl` and then `cron/<group>/test_*.jl`, that is the
  tests of every run plus the convergence report (off by default, see below); it cannot be
  combined with `cron`;
- file patterns (`*` and `?`, with or without `test_` and `.jl`);
- `problem=a,b`: test only these problems, for instance the one you add. A name that is
  not in the catalogue is an error (a selection matching nothing would test nothing and
  pass). `core` still runs entirely.

In cron mode, an environment without a `cron/<name>/` directory is skipped.

## Three kinds of tests

Only the first two can fail a run (#360).

1. **Does not break** — every run. `suite/<modeling>/test_smoke.jl` builds every problem
   of the modeling in each of its configurations, on a small grid (`SMOKE_GRID`), and runs
   a few iterations (`SMOKE_MAX_ITER`): no exception, a finite objective. Convergence is
   not checked. The contracts, methods and API tests (`test_contract`, `test_methods`,
   `test_problem`, `core`) belong here too.
2. **Model equivalence** — every run; the most important part, since comparing models is
   fair only if they are the same problem. `suite/comparison/` compares them at the
   definition, at the initial guess (see below), not at convergence.
3. **Convergence** — off by default: in the cron, and on demand with the `convergence`
   argument or the label `test convergence`. `cron/<group>/test_convergence.jl` solves each
   problem on its default grid and **reports** the result without testing it: whether the
   solver says it converged (and why not), the objective, the iterations and the time, and
   the relative gap to the reference objective only when the problem has one. An
   exception still fails. This is what helps tune an initial guess: look at the results and
   adapt.

## Statuses and report

Every problem has a status per modeling (`:valid` or `:draft`, in its metadata). Tests
loop with `Main.TestStatus.for_each_problem(f, test, backends...)`:

- a `:valid` problem is checked with `Test.@test`: a failure fails the run;
- a `:draft` problem runs in a `DraftTestSet`: its outcome is recorded, a failure never
  fails the run;
- a test between several modelings is `:valid` only when the problem is `:valid` in all.

Each run writes `test/reports/<environment>[-cron].md` (git-ignored): one line per test
and problem, with its status and outcome (`pass`, `fail`, `error`, or `no test`). The
lines worth reading (drafts and checks that report instead of failing) are also printed.
The convergence report adds a second table, `Results of the solves` (also written as
`<environment>[-cron]-results.tsv`), one line per solve.

`:draft` means that the model is not finished or not equivalent to the other modelings,
not that it does not converge: convergence is reported, not tested.
A draft whose line says `pass` everywhere is ready to become `:valid`. The cron workflows
collect these files and publish their scope-specific report on the `results` branch.

## What is compared, and what is not

- **Per modeling**, a test checks that the problem builds and runs in each configuration
  (`test_smoke`) and that the model has the expected structure (contract, methods). A new
  problem enters as `:draft` and becomes `:valid` when its model is finished and equivalent
  to the other modelings; whether it converges is reported by the convergence tests.
- **Between modelings**, the `comparison` environment checks that they describe the same
  problem: same structure, grid, trajectories and objective at the initial guess (zero
  iterations). The solutions found by two solvers are **not** compared: after a solve,
  two modelings are two optimization paths, and a problem with several local solutions
  ends on one or the other depending on the platform. The equivalence of the models
  themselves is the NLP equivalence of #284.

## Configurations

`common/configs.jl` is the single matrix of configurations: solver options, the grid and
iterations of the smoke solves, the grid of the comparison, and the configurations to solve
with per modeling. A test loops over
`Main.TestConfigs.CONFIGS.<modeling>`; it never writes solver options itself.

## Continuous integration

The CI runs on a pull request only with a runner label, and the scope labels select what
it runs:

| Labels | Effect |
| --- | --- |
| `run ci github-runner` | run the CI on GitHub runners (Ubuntu and Julia 1.13 only for now; Julia 1.10, macOS and Windows come back at the end of #310, see #344) |
| `test <environment> backend`, `test comparison` | run core and these environments only |
| `test convergence` | add the convergence report (argument `convergence`): the convergence tests of the selected environments run after the others, and report without failing |
| `test all problems` | remove the per-problem narrowing below; composes with the scope labels |
| no scope label | run core and every environment except the opt-in ones |

Scope labels add up. Adding or removing a label reruns the CI when a runner label is set.
On a pull request, a change touching only one or more per-problem files
(`ext/MetaData/<p>.jl`, `ext/<modeling>Models/<p>.jl`, or `ext/Descriptions/<p>.md`)
automatically adds `problem=<p>,...` to the test arguments. The changed problem names are
combined with any scope labels, so `test JuMP backend` and a change to `beam` run
`JuMP problem=beam`. Any shared, removed, or renamed file (a file of those directories whose name is not
a problem of `ext/MetaData/`, such as a helper, is a shared file), an unreadable or truncated
file list, a release pull request, and the `test all problems` label run the complete
catalogue: narrowing only happens on positive evidence. The decision is made by
`tools/ci/scope.py`, tested by `python3 -m unittest discover -s tools/ci/test -p 'test_*.py'`.

`problem=` also narrows `core`: checks that loop over problems run for the selected
problems only, while the repository-wide checks (files, metadata schema) are unaffected.
An edited draft problem runs as draft: it cannot fail the run. Coverage is unaffected —
`Coverage.yml` runs on pushes to `main` and tags, never on a narrowed pull request. The
cron ignores labels and problem scoping.

## Adding a test

1. Pick the group: `core` for what needs no extension, the modeling environment for one
   modeling, `comparison` for several; `suite/` if it is fast, `cron/` if it solves on
   the default grid.
2. Create `test_<name>.jl` following rule `TST1` of ct-conventions: a module, qualified
   imports, one entry function `test_<name>()`, redefined at top level:

   ```julia
   module TestName

   using Test: Test
   using OptimalControlProblems: OptimalControlProblems

   const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
   const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

   function test_name()
       Test.@testset "what is tested" verbose = VERBOSE showtiming = SHOWTIMING begin
           Main.TestStatus.for_each_problem("name", OptimalControlProblems.JuMPBackend()) do problem
               # build, solve, check with Test.@test
           end
       end
   end

   end # module TestName

   test_name() = TestName.test_name()
   ```

3. Never write a problem name: loop over the catalogue (a consistency test of `core`
   fails on a problem name written in `src/`, `test/` or an extension file). A fact about
   one problem goes into its metadata.
4. Every check you add must be shown to fail on a deliberately broken case.

## Adding an environment

Add `suite/<name>/Project.toml` (and `cron/<name>/` if needed): it is discovered. Add
its scope label `test <name> backend` to the repository, and its name to `OPT_IN` in
`runtests.jl` if it must run only when named.
