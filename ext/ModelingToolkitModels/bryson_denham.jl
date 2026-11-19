"""The Bryson-Denham problem is a constrained minimum-energy transfer problem.

Builds the `bryson_denham` problem with the ModelingToolkit backend.

```julia-repl
julia> using ModelingToolkit, OptimalControlProblems
julia> pb = bryson_denham(ModelingToolkitBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:bryson_denham)` and `OptimalControlProblems.status(:bryson_denham)`.
"""
function OptimalControlProblems.bryson_denham(
    ::ModelingToolkitBackend; parameters::Union{Nothing,NamedTuple}=nothing, init=nothing
)
    OptimalControlProblems.Catalogue._warn_if_draft(:bryson_denham, :ModelingToolkit)
    params = OptimalControlProblems.parameters_data(:bryson_denham, parameters)
    t0, tf = params[:t0], params[:tf]
    x1_t0, x2_t0 = params[:x1_t0], params[:x2_t0]
    x1_tf, x2_tf = params[:x1_tf], params[:x2_tf]
    x1_max = params[:x1_max]

    t = ModelingToolkit.t_nounits
    D = ModelingToolkit.Differential(t)

    ModelingToolkit.@variables x₁(..) = x1_t0 [tunable = false]
    ModelingToolkit.@variables x₂(..) = x2_t0 [tunable = false]
    ModelingToolkit.@variables u(..) [input = true]

    eqs = [D(x₁(t)) ~ x₂(t), D(x₂(t)) ~ u(t)]
    costs = [ModelingToolkit.Symbolics.Integral(t in (t0, tf))(0.5 * u(t)^2)]
    constraints = [
        x₁(tf) ~ x1_tf, x₂(tf) ~ x2_tf, ModelingToolkit.Symbolics.:≲(x₁(t), x1_max)
    ]

    ModelingToolkit.@named sys = ModelingToolkit.System(
        eqs, t; costs=costs, constraints=constraints
    )

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:bryson_denham, params, init)

    return OptimalControlProblems.Problems._problem(
        :bryson_denham, :ModelingToolkit, sys, init, params
    )
end
