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
├── cron/<group>/   # slow tests of the same group, run by the weekly cron
└── robustness/<group>/ # robustness report of the group, run by the monthly cron (#378)
```

- `suite/core/` has no extension loaded and runs in the dispatcher's process, always.
- Every other directory of `suite/` that holds a `Project.toml` is a **test environment**
  (one per modeling, plus `comparison` where every modeling is loaded). It runs in its
  own Julia process, in a temporary copy of its `Project.toml` where the package is
  developed, so that each extension is shown to load alone.
- `cron/<group>/` holds the slow tests of a group; they run in the environment of
  `suite/<group>/`, as `robustness/<group>/` does.
- `robustness/<group>/test_robustness.jl` solves every problem on its default grid under
  other options of each solver (`TestConfigs.ROBUSTNESS`): does the initial guess still
  lead to the best known solution? Reported, never tested. Run it with the word
  `robustness` (`Pkg.test(; test_args=["OptimalControl", "robustness"])`); it runs only
  then. MadNLP 0.10 reads the barrier parameter from its `barrier` option only, which the
  MadNLP strategy of OptimalControl does not know: `SolveOptimalControl` passes it with
  `bypass`.

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
   definition, at the initial guess (see below), not at convergence, and checks that
   the JuMP NLP is CTDirect's (`test_equivalence`, #371): the trapezoidal rule for every
   problem (is it the same problem?), every scheme and ordering on a few problems chosen by
   their features, one per property the schemes treat differently (are the schemes
   CTDirect's?, #401). The product problems × schemes is not run: it answers neither
   question better. Count the nonzeros of an NLP with `Equivalence.nonzeros`, never with
   `meta.nnzh` (#372).
3. **Convergence** — off by default: in the cron, and on demand with the `convergence`
   argument or the label `test convergence`. `cron/<group>/test_convergence.jl` solves each
   problem on its default grid and **reports** the result without testing it: whether the
   solver says it converged (and why not), the objective, the iterations and the time, and
   the relative gap to the reference objective only when the problem has one, with which
   known solution it reached (`best`, `local` or `new`, #378). An
   exception still fails. This is what helps tune an initial guess: look at the results and
   adapt.

The reference objectives are computed from these results, never typed: see
[`tools/references/README.md`](../tools/references/README.md). `core/test_consistency.jl`
fails when a reference no longer matches the default grid or parameters of its problem, or
when a problem `:valid` in one modeling only has no reference.

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
`<environment>[-cron]-results.tsv`, objective at full precision), one line per solve, with
its grid size and scheme.

The results file also says how each solve was configured and how feasible its solution is
(#409), in columns after those of the table: `modeler` (empty for JuMP), `solver`, `options`
(every option given to the solver, defaults included, as an inline TOML table: read it with
`TOML.parse("o = " * cell)["o"]`) and `violation` (the largest violation of the
constraints, as each modeling measures it: JuMP counts the variable bounds, which Ipopt
relaxes by about the tolerance). A run with solves also writes
`<environment>[-cron]-environment.toml`: Julia, the machine, the commit and the run, and the
versions of the direct dependencies of the environment and of the packages that discretize
and solve (`TestStatus.KEY_DEPENDENCIES`). The tools read these files by column and by key;
they are never edited by hand. The reference solutions are computed from them
(`tools/references/`).

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

The CI runs on a pull request only with a runner label. It then tests **what the pull
request changes** (#405), and the scope labels only **add** to it:

| Labels | Effect |
| --- | --- |
| `run ci kkt-runner`, `run ci occidata-runner` | run the GPU tests (environment `GPU`, see below) on this self-hosted runner |
| `run ci github-runner` | run the CI on GitHub runners (Ubuntu and Julia 1.13 only for now; Julia 1.10, macOS and Windows come back at the end of #310, see #344) |
| `test <environment> backend`, `test comparison` | add these environments, with the problems of the automatic scope (every problem if it has none) |
| `test all problems` | every problem, in the environments of the automatic scope and of the labels |
| `test everything` | every environment (except the opt-in ones) and every problem |
| `test convergence` | add the convergence report (argument `convergence`): the convergence tests of the selected environments run after the others, and report without failing |

Adding or removing a label reruns the CI when a runner label is set. `core` always runs.

**When to add `test convergence`.** When the pull request can change what a solve returns
(a model, an initial guess, a solver option, a scheme), or changes code that only the
convergence tests run: `test/cron/`, the `outcome` and `settings` of the solve modules,
`TestStatus.result!`. The smoke solves of the default run call `solve` but none of these.
The label solves every selected problem on its default grid, as the crons do, one
environment after another: on a pull request that touches a shared file (every problem), it
adds about the time of the three convergence crons, some 35 minutes (#412). No label runs
`robustness/`: run it locally (`test_args=["<environment>", "robustness", "problem=…"]`) and
paste the output in the pull request.

**The automatic scope.** Each changed file gives environments and problems, and the run is
their union (one product, environments × problems: a little more than needed when several
modelings change different problems). The rules are read from the repository:

| Changed file | Environments | Problems |
| --- | --- | --- |
| `ext/<M>Models/<p>.jl`, a problem of a modeling | `M`, `comparison` | `p` |
| another file of `ext/<M>Models/` (e.g. `schemes/`) | `M`, `comparison` | all |
| `ext/<E>.jl`, an extension | those whose `Project.toml` holds its triggers | all |
| a file of `ext/MetaData/` or `ext/Descriptions/` named after a problem | all | `p` |
| `test/suite/<env>/**` | `env` (none for `core` and the opt-in ones) | all |
| `test/common/<f>.jl` | those whose tests include it, directly or not; all if `test/runtests.jl` names it | all |
| `test/cron/**`, `test/robustness/**`, `tools/**`, `docs/**`, Markdown outside `ext/` | none: `core` only | — |
| anything else (`src/`, `Project.toml`, `test/runtests.jl`, workflows...) | all | all |

A removed or renamed file, an unreadable or truncated file list, no file, a release pull
request, or an event other than a pull request run everything: narrowing only happens on
positive evidence. The opt-in environments (`OPT_IN` in `runtests.jl`) never enter the
automatic scope; their label adds them. The decision, file by file, is in the job summary
of the `scope` job. It is made by `tools/ci/scope.py`, tested by
`python3 -m unittest discover -s tools/ci/test -p 'test_*.py'`.

`problem=` also narrows `core`: checks that loop over problems run for the selected
problems only, while the repository-wide checks (files, metadata schema) are unaffected.
An edited draft problem runs as draft: it cannot fail the run. Coverage is unaffected —
`Coverage.yml` runs on pushes to `main` and tags, never on a narrowed pull request. The
cron ignores labels and problem scoping.

## GPU tests

The environment `GPU` (`test/suite/GPU/`) solves every problem of the OptimalControl and
OptimalControlScalarized modelings with Exa and MadNLP on the GPU (`GPU_CONFIGS` in
`common/configs.jl`, small grid). It is opt-in, like `ModelingToolkit`: `CUDA`, `CUDSS` and
`MadNLPGPU` are heavy and useless on a CPU runner. Run it with
`Pkg.test(; test_args=["GPU"])`, or with the labels above on a pull request; on a push to
`main` a single GPU run goes to `kkt`, or `occidata` if `kkt` is offline.

The capabilities are read from `Main.TestCapabilities` (`common/capabilities.jl`), never
asked to CUDA in a test file (`TST6`): without a functional device the solves are reported
as skipped (`Broken` in the summary); on a runner named `kkt` or `occidata`, a missing
device fails `test_contract`.

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
