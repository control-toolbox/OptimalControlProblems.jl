# [Solve with JuMP](@id solve-jump)

A JuMP problem is a `JuMP.Model`, already discretized: the scheme, the grid and the order of
the variables are chosen when the problem is built. The model reproduces the NLP that
OptimalControl builds for the same scheme; see [JuMP schemes](@ref jump-schemes).

```@example jump
using JuMP, Ipopt, OptimalControlProblems
```

## Build and solve

```@example jump
nlp = beam(JuMPBackend(); grid_size=grid_size_data(:beam), scheme=:trapeze)
set_optimizer(nlp, Ipopt.Optimizer)
set_silent(nlp)
optimize!(nlp)
termination_status(nlp), objective_value(nlp)
@assert isapprox(objective_value(nlp), OptimalControlProblems.reference_objective(:beam); rtol=1e-6) # hide
```

## The keywords

| Keyword | Values | Default |
| :--- | :--- | :--- |
| `grid_size` | number of steps | the default grid of the problem |
| `scheme` | `:euler`, `:euler_implicit`, `:trapeze`, `:midpoint`, as in `OptimalControl.Collocation` | `:trapeze` |
| `ordering` | `:interleaved` (as CTDirect) or `:blocks` (grouped by kind) | `:interleaved` |
| `parameters` | the parameters to change | the default parameters |
| `init` | another initial guess (see [Initial guess](@ref initial-guess)) | the shared one |

Other arguments are passed to `JuMP.Model`.

## Read the solution

The JuMP models store the names of the components and the time grid, so the solution reads as
an OptimalControl one:

```@example jump
x = state(nlp)      # a function of the time
u = control(nlp)
t = time_grid(nlp)
x(t[end]), u(t[1])
```

With OptimalControl loaded, `solution` builds a full OptimalControl solution from the solved
model and the `Problem` of the same problem, to display or [plot](@ref plot-solution) it:

```@example jump
using OptimalControl
pb = beam(OptimalControlBackend())
sol = solution(nlp, pb)
objective(sol), iterations(sol)
```
