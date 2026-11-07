# [Developers](@id developers)

How the collection is built, tested and documented. The procedures live next to the code they
describe, in READMEs, so that they change with it; this page says where to find them.

## The principle

**What can be computed is generated or checked, never maintained by hand.** The availability of
a problem in a modeling is the presence of its file; the features are computed from the model;
the reference objectives from the results of the crons; the documentation pages from the files
of each problem. A test checks that no list of problems is written by hand.

## Where to read

| I want to | Read |
| :--- | :--- |
| add a problem, or a modeling | [`ext/README.md`](https://github.com/control-toolbox/OptimalControlProblems.jl/blob/main/ext/README.md) |
| run the tests, or add one | [`test/README.md`](https://github.com/control-toolbox/OptimalControlProblems.jl/blob/main/test/README.md) |
| know what the crons do, and handle the issues they open | [`.github/workflows/README.md`](https://github.com/control-toolbox/OptimalControlProblems.jl/blob/main/.github/workflows/README.md) |
| compute the reference objectives again | [`tools/references/README.md`](https://github.com/control-toolbox/OptimalControlProblems.jl/blob/main/tools/references/README.md) |
| know the NLP of each JuMP scheme | [JuMP schemes](@ref jump-schemes) |
| follow the conventions of the ecosystem | [ct-conventions](https://github.com/control-toolbox/ct-conventions) and `AGENTS.md` |

## Three kinds of tests

| Kind | When | Fails on |
| :--- | :--- | :--- |
| Does not break | every pull request, small grid, a few iterations | an exception or a non-finite objective |
| Same NLP | every pull request, no solve | two modelings that give different NLPs |
| Convergence | the weekly crons, or on demand (label `test convergence`) | nothing: the solver status and the gap to the reference are reported |

The convergence results feed the reference objectives and the problem pages, and the
[Status](@ref problems-status) page shows the latest reports.

## The documentation

The problem pages, the Status page and the API Reference are generated at each build;
`docs/modelings.jl` holds what the pages need to know about each modeling. The build solves
nothing for the problem pages: their results come from the crons. To build it locally:

```bash
julia --project=docs docs/make.jl
npx serve docs/build/1 --listen 5173
```
