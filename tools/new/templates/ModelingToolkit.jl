"""TODO: the first sentence of the description.

Builds the `{{problem}}` problem with the ModelingToolkit backend.

```julia-repl
julia> using ModelingToolkit, OptimalControlProblems
julia> pb = {{problem}}(ModelingToolkitBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:{{problem}})` and `OptimalControlProblems.status(:{{problem}})`.
"""
function OptimalControlProblems.{{problem}}(
    ::ModelingToolkitBackend; parameters::Union{Nothing,NamedTuple}=nothing, init=nothing
)
    OptimalControlProblems.Catalogue._warn_if_draft(:{{problem}}, :ModelingToolkit)
    params = OptimalControlProblems.parameters_data(:{{problem}}, parameters)
    t0, tf = float(params[:t0]), float(params[:tf])
    x_t0, x_u = float(params[:x_t0]), params[:x_u]

    # model. TODO: the model of the problem, in place of this example
    t = ModelingToolkit.t_nounits
    D = ModelingToolkit.Differential(t)

    ModelingToolkit.@variables x(..) = x_t0 [tunable = false]
    ModelingToolkit.@variables u(..) [input = true]

    eqs = [D(x(t)) ~ u(t)]
    costs = [ModelingToolkit.Symbolics.Integral(t in (t0, tf))(u(t)^2)]
    constraints = [ModelingToolkit.Symbolics.:≲(x(t), x_u)]

    ModelingToolkit.@named sys = ModelingToolkit.System(
        eqs, t; costs=costs, constraints=constraints
    )

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:{{problem}}, params, init)

    return OptimalControlProblems.Problems._problem(
        :{{problem}}, :ModelingToolkit, sys, init, params
    )
end
