# [Your first problem](@id first-problem)

We take the [clamped beam](@ref description-beam): a double integrator whose position must stay
below a bound, with a quadratic cost. We build it with OptimalControl, solve it, and compare the
objective to the reference of the collection.

```@example first
using OptimalControl, OptimalControlProblems
using NLPModelsIpopt
```

## Get the problem

A problem is a function, called with the modeling as argument. With OptimalControl it returns a
[`Problem`](@ref OptimalControlProblems.Problems.Problem): the continuous problem and its initial guess.

```@example first
pb = beam(OptimalControlBackend())
```

```@example first
model(pb)
```

## Solve it

The discretization is chosen in `solve`: here the trapezoidal rule on the default grid of the
problem, an ADNLPModels model and Ipopt. The reference objectives of the collection are
computed with the trapezoidal rule, which is not the default scheme of `solve`: give it.

```@example first
N = grid_size_data(:beam)
sol = solve(
    model(pb), :collocation, :adnlp, :ipopt;
    init=initial_guess(pb),
    grid_size=N,
    scheme=:trapeze,
    display=false,
)
objective(sol)
```

## Check the result

The collection stores a reference objective for each problem, discretized on its default grid
with the trapezoidal rule: the best objective reached by every modeling and configuration
tested.

```@example first
reference = OptimalControlProblems.reference_objective(:beam)
```

```@example first
abs(objective(sol) - reference) / abs(reference)
@assert abs(objective(sol) - reference) / abs(reference) < 1e-6 # hide
```

The [problem page](@ref description-beam) shows the same comparison for every modeling and
configuration, and how the JuMP model of the same problem reaches the same objective.

## Next

- [Guided tour](@ref guided-tour): the collection and what each problem comes with.
- [Solve with OptimalControl](@ref solve-optimalcontrol): schemes, modelers, solvers, GPU.
