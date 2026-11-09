# [Migrating from 0.5](@id migration)

The breaking changes since 0.5, with how to update your code. The full list is in
[`BREAKING.md`](https://github.com/control-toolbox/OptimalControlProblems.jl/blob/main/BREAKING.md).

## An OptimalControl problem is the continuous problem

A problem of the OptimalControl modelings returns a [`Problem`](@ref OptimalControlProblems.Problems.Problem):
the continuous model and its initial guess. It no longer discretizes, and no longer takes a
description or `grid_size`: the discretization, the modeler and the solver are chosen in
`solve`.

```julia
docp = beam(OptimalControlBackend(), :adnlp; grid_size=500)        # [!code --]
nlp = nlp_model(docp)                                              # [!code --]
pb = beam(OptimalControlBackend())                                 # [!code ++]
sol = solve(model(pb), :collocation, :adnlp, :ipopt;               # [!code ++]
            init=initial_guess(pb), grid_size=500,                 # [!code ++]
            scheme=:trapeze)                                       # [!code ++]
```

## `metadata(:problem)` is `problem_metadata(:problem)`

`metadata` is now the function OptimalControl reexports; on a built problem it gives the
metadata of that problem.

```julia
metadata(:beam)[:grid_size]                                        # [!code --]
problem_metadata(:beam)[:grid_size]                                # [!code ++]
metadata(beam(OptimalControlBackend()))[:grid_size]                # [!code ++]
```

## Errors

Passing `parameters` to a problem that has none throws
`CTBase.Exceptions.PreconditionError`, instead of `CTBase.UnauthorizedCall`.

## Accessors

`time_grid`, `state`, `control`, `costate`, `variable`, `objective`, `iterations` and the
`*_components` / `*_dimension` accessors are the functions of CTModels that OptimalControl
reexports: `using OptimalControl, OptimalControlProblems` brings no name conflict, and they
work on JuMP models as on OptimalControl solutions.
