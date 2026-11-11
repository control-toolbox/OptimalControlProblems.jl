"""The **Robbins benchmark problem** is a classical optimal control problem introduced by Robbins (1980), involving a **third-order state constraint**.

Builds the `robbins` problem with the ModelingToolkit backend.

```julia-repl
julia> using ModelingToolkit, OptimalControlProblems
julia> pb = robbins(ModelingToolkitBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:robbins)` and `OptimalControlProblems.status(:robbins)`.
"""
function OptimalControlProblems.robbins(
    ::ModelingToolkitBackend;
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
)
    OptimalControlProblems.Catalogue._warn_if_draft(:robbins, :ModelingToolkit)
    params = OptimalControlProblems.parameters_data(:robbins, parameters)
    t0, tf = float(params[:t0]), float(params[:tf])
    α, β, γ = params[:α], params[:β], params[:γ]
    x₁_l = float(params[:x₁_l])
    x₁_t0, x₂_t0, x₃_t0 = float(params[:x₁_t0]), float(params[:x₂_t0]), float(params[:x₃_t0])
    x₁_tf, x₂_tf, x₃_tf = float(params[:x₁_tf]), float(params[:x₂_tf]), float(params[:x₃_tf])

    t = ModelingToolkit.t_nounits
    D = ModelingToolkit.Differential(t)

    ModelingToolkit.@variables x₁(..) = x₁_t0 [tunable = false]
    ModelingToolkit.@variables x₂(..) = x₂_t0 [tunable = false]
    ModelingToolkit.@variables x₃(..) = x₃_t0 [tunable = false]
    ModelingToolkit.@variables u(..) [input = true]

    eqs = [D(x₁(t)) ~ x₂(t), D(x₂(t)) ~ x₃(t), D(x₃(t)) ~ u(t)]
    costs = [
        ModelingToolkit.Symbolics.Integral(t in (t0, tf))(
            α * x₁(t) + β * x₁(t)^2 + γ * u(t)^2
        ),
    ]
    constraints = [
        x₁(tf) ~ x₁_tf,
        x₂(tf) ~ x₂_tf,
        x₃(tf) ~ x₃_tf,
        ModelingToolkit.Symbolics.:≳(x₁(t), x₁_l),
    ]

    ModelingToolkit.@named sys = ModelingToolkit.System(
        eqs, t; costs=costs, constraints=constraints
    )

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:robbins, params, init)

    return OptimalControlProblems.Problems._problem(
        :robbins, :ModelingToolkit, sys, init, params
    )
end
