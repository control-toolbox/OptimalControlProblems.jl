"""The **clamped beam problem** is a classical benchmark in constrained optimal control.

Builds the `beam` problem with the ModelingToolkit backend.

```julia-repl
julia> using ModelingToolkit, OptimalControlProblems
julia> pb = beam(ModelingToolkitBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:beam)` and `OptimalControlProblems.status(:beam)`.
"""
function OptimalControlProblems.beam(
    ::ModelingToolkitBackend;
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
)
    OptimalControlProblems.Catalogue._warn_if_draft(:beam, :ModelingToolkit)
    params = OptimalControlProblems.parameters_data(:beam, parameters)
    t0, tf = float(params[:t0]), float(params[:tf])
    x₁_l, x₁_u = float(params[:x₁_l]), float(params[:x₁_u])
    x₁_t0, x₂_t0 = float(params[:x₁_t0]), float(params[:x₂_t0])
    x₁_tf, x₂_tf = float(params[:x₁_tf]), float(params[:x₂_tf])

    t = ModelingToolkit.t_nounits
    D = ModelingToolkit.Differential(t)

    ModelingToolkit.@variables x₁(..) = x₁_t0 [tunable = false]
    ModelingToolkit.@variables x₂(..) = x₂_t0 [tunable = false]
    ModelingToolkit.@variables u(..) [input = true]

    eqs = [D(x₁(t)) ~ x₂(t), D(x₂(t)) ~ u(t)]
    costs = [ModelingToolkit.Symbolics.Integral(t in (t0, tf))(u(t)^2)]
    constraints = [
        x₁(tf) ~ x₁_tf,
        x₂(tf) ~ x₂_tf,
        ModelingToolkit.Symbolics.:≳(x₁(t), x₁_l),
        ModelingToolkit.Symbolics.:≲(x₁(t), x₁_u),
    ]

    ModelingToolkit.@named sys = ModelingToolkit.System(
        eqs, t; costs=costs, constraints=constraints
    )

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:beam, params, init)

    return OptimalControlProblems.Problems._problem(
        :beam, :ModelingToolkit, sys, init, params
    )
end
