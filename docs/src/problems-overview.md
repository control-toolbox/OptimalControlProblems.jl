# [Overview](@id problems-overview)

```@eval
using Markdown
n = length(Main.OptimalControlProblems.problems(; status=:all))
Markdown.parse("The collection holds **$(n) problems**. Each one has a page, generated from its files:")
```

- **Problem**: its description and mathematical formulation, its parameters, and its reference
  objective;
- **one tab per modeling** where it exists: its status, how to get and solve it, the model as
  written, and its results;
- **Compare**, when it exists in two modelings or more: which modelings are checked to give the
  same NLP, their results side by side, and which solution each one reached.

The tab bar stays at the top of the page, and the tab you choose is kept from one problem to the
next. A link can open a page on a tab: add `?view=JuMP` to its address.

The [browser](@ref problems-browser) finds problems by their features, and the
[Status](@ref problems-status) page shows every problem in every modeling, with the latest
convergence reports.

## What each problem comes with

| | Where it comes from |
| :--- | :--- |
| Modelings | the files of the problem: a problem exists in a modeling when its file does |
| Status | the metadata, `valid` or `draft` per modeling |
| Description | `ext/Descriptions/<problem>.md` |
| Parameters, default grid, initial guess | the metadata, `ext/MetaData/<problem>.jl` |
| Features | computed from the OptimalControl model |
| Reference objective | computed from the convergence and robustness crons, per scheme, on the default grid |
| Analytic solution | the metadata, when the continuous problem has a known solution |

## The reference objective

For each scheme, the reference objective is the **best objective among the solves that
converged**, for every modeling and configuration, on the default grid of the problem. The
crons solve every problem every week in every configuration, and a script gathers the results
into `ext/MetaData/references/<problem>.toml`; see
[how the references are computed](https://github.com/control-toolbox/OptimalControlProblems.jl/blob/main/tools/references/README.md).

The same file keeps every result and the **distinct solutions** the solves reached: two
objectives closer than a relative `OptimalControlProblems.SAME_SOLUTION` are the same solution.
A problem with several solutions has local solutions, and its Compare tab says which
configuration reached which.

The reference is the objective of the **discretized** problem. When the continuous problem has
a known solution, its objective is on the page too, and the gap between the two is the
discretization error.

## Results are reported, not tested

The tests of the package fail when a problem does not build or does not run, or when two
modelings do not give the same NLP. Convergence is not tested: every solve is shown with its
gap to the reference, as it is, without tolerance.
