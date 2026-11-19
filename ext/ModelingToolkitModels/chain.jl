"""The **Hanging Chain problem** is a classical benchmark in optimal control.

Builds the `chain` problem with the ModelingToolkit backend.

```julia-repl
julia> using ModelingToolkit, OptimalControlProblems
julia> pb = chain(ModelingToolkitBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:chain)` and `OptimalControlProblems.status(:chain)`.
"""
function OptimalControlProblems.chain(
    ::ModelingToolkitBackend;
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
)
    OptimalControlProblems.Catalogue._warn_if_draft(:chain, :ModelingToolkit)
    params = OptimalControlProblems.parameters_data(:chain, parameters)
    t0, tf = float(params[:t0]), float(params[:tf])
    L = float(params[:L])
    a, b = float(params[:a]), float(params[:b])
    x₂_t0, x₃_t0 = float(params[:x₂_t0]), float(params[:x₃_t0])

    t = ModelingToolkit.t_nounits
    D = ModelingToolkit.Differential(t)

    ModelingToolkit.@variables x₁(..) = a [tunable = false]
    ModelingToolkit.@variables x₂(..) = x₂_t0 [tunable = false]
    ModelingToolkit.@variables x₃(..) = x₃_t0 [tunable = false]
    ModelingToolkit.@variables u(..) [input = true]

    eqs = [
        D(x₁(t)) ~ u(t),
        D(x₂(t)) ~ x₁(t) * sqrt(1 + u(t)^2),
        D(x₃(t)) ~ sqrt(1 + u(t)^2),
    ]
    # a Mayer cost: the final value of x₂
    costs = [x₂(tf)]
    constraints = [x₁(tf) ~ b, x₃(tf) ~ L]

    ModelingToolkit.@named sys = ModelingToolkit.System(
        eqs, t; costs=costs, constraints=constraints
    )

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:chain, params, init)

    return OptimalControlProblems.Problems._problem(
        :chain, :ModelingToolkit, sys, init, params
    )
end
