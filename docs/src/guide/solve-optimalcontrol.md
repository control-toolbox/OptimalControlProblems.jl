# [Solve with OptimalControl](@id solve-optimalcontrol)

An OptimalControl problem is continuous: `model(pb)` is an optimal control problem of
OptimalControl.jl, and `initial_guess(pb)` its initial guess. Everything else is chosen in
`solve`: the discretization, the NLP modeler, the solver, CPU or GPU. See the
[OptimalControl.jl documentation](https://control-toolbox.org/OptimalControl.jl/dev/) for every
option.

```@example oc
using OptimalControl, OptimalControlProblems
using NLPModelsIpopt
using MadNLP

pb = beam(OptimalControlBackend())
N = grid_size_data(:beam)
nothing # hide
```

## Descriptive mode

The method is described by symbols: discretization, modeler, solver.

```@example oc
sol = solve(
    model(pb), :collocation, :adnlp, :ipopt;
    init=initial_guess(pb),
    grid_size=N,
    scheme=:trapeze,
    display=false,
)
objective(sol)
@assert isapprox(objective(sol), OptimalControlProblems.reference_objective(:beam); rtol=1e-6) # hide
```

## Explicit mode

The same method, built from its components, each with its options:

```@example oc
sol = solve(
    model(pb);
    init=initial_guess(pb),
    discretizer=OptimalControl.Collocation(; grid_size=N, scheme=:trapeze),
    modeler=OptimalControl.Exa(),
    solver=OptimalControl.MadNLP(; print_level=MadNLP.ERROR),
    display=false,
)
objective(sol)
@assert isapprox(objective(sol), OptimalControlProblems.reference_objective(:beam); rtol=1e-6) # hide
```

## The scheme and the grid

The reference objective of a problem is computed for a scheme on a grid: compare a solve to the
reference of the same discretization.

```@example oc
sol = solve(
    model(pb), :collocation, :adnlp, :ipopt;
    init=initial_guess(pb), grid_size=100, scheme=:midpoint, display=false,
)
objective(sol)
```

```@example oc
OptimalControlProblems.reference_objective(:beam; grid_size=100, scheme=:midpoint)
```

`nothing`: the collection stores the references of the default grid only. The
[problem page](@ref description-beam) lists them, with the schemes they were computed for.

## The scalarized modeling

Some problems are also written with scalar components only, which the ExaModels modeler
needs on GPU. They are the same problems: the same NLP is checked on every pull request.

```@example oc
problems(OptimalControlScalarizedBackend())
```

```@example oc
pb_s = beam(OptimalControlScalarizedBackend())
sol = solve(
    model(pb_s), :collocation, :exa, :madnlp;
    init=initial_guess(pb_s), grid_size=N, scheme=:trapeze, display=false,
)
objective(sol)
@assert isapprox(objective(sol), OptimalControlProblems.reference_objective(:beam); rtol=1e-6) # hide
```

## On a GPU

With `MadNLPGPU`, `CUDA` and `CUDSS` loaded and a functional CUDA device, the ExaModels modeler
and MadNLP run on the GPU:

```julia
using MadNLPGPU, CUDA, CUDSS

sol = solve(
    model(pb_s);
    init=initial_guess(pb_s),
    discretizer=OptimalControl.Collocation(; grid_size=N, scheme=:trapeze),
    modeler=OptimalControl.Exa{OptimalControl.GPU}(),
    solver=OptimalControl.MadNLP{OptimalControl.GPU}(),
)
```

The GPU configurations are solved for every problem by the GPU crons: their results are on
each problem page, platform "GPU runner".
