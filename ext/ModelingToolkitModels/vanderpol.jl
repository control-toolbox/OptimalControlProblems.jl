"""The **Van der Pol oscillator control problem** is a benchmark in nonlinear optimal control.

Builds the `vanderpol` problem with the ModelingToolkit backend.

```julia-repl
julia> using ModelingToolkit, OptimalControlProblems
julia> pb = vanderpol(ModelingToolkitBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:vanderpol)` and `OptimalControlProblems.status(:vanderpol)`.
"""
function OptimalControlProblems.vanderpol(
    ::ModelingToolkitBackend;
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
)
    OptimalControlProblems.Catalogue._warn_if_draft(:vanderpol, :ModelingToolkit)
    params = OptimalControlProblems.parameters_data(:vanderpol, parameters)
    t0, tf = float(params[:t0]), float(params[:tf])
    ω, ε = params[:ω], params[:ε]
    x₁_t0, x₂_t0 = float(params[:x₁_t0]), float(params[:x₂_t0])

    t = ModelingToolkit.t_nounits
    D = ModelingToolkit.Differential(t)

    ModelingToolkit.@variables x₁(..) = x₁_t0 [tunable = false]
    ModelingToolkit.@variables x₂(..) = x₂_t0 [tunable = false]
    ModelingToolkit.@variables u(..) [input = true]

    eqs = [
        D(x₁(t)) ~ x₂(t),
        D(x₂(t)) ~ ε * ω * (1 - x₁(t)^2) * x₂(t) - ω^2 * x₁(t) + u(t),
    ]
    costs = [
        ModelingToolkit.Symbolics.Integral(t in (t0, tf))(
            0.5 * (x₁(t)^2 + x₂(t)^2 + u(t)^2)
        ),
    ]
    constraints = []

    ModelingToolkit.@named sys = ModelingToolkit.System(
        eqs, t; costs=costs, constraints=constraints
    )

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:vanderpol, params, init)

    return OptimalControlProblems.Problems._problem(
        :vanderpol, :ModelingToolkit, sys, init, params
    )
end
