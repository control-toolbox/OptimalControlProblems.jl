"""The **Jackson problem** is a classical benchmark in optimal control.

Builds the `jackson` problem with the ModelingToolkit backend.

```julia-repl
julia> using ModelingToolkit, OptimalControlProblems
julia> pb = jackson(ModelingToolkitBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:jackson)` and `OptimalControlProblems.status(:jackson)`.
"""
function OptimalControlProblems.jackson(
    ::ModelingToolkitBackend; parameters::Union{Nothing,NamedTuple}=nothing, init=nothing
)
    OptimalControlProblems.Catalogue._warn_if_draft(:jackson, :ModelingToolkit)
    params = OptimalControlProblems.parameters_data(:jackson, parameters)
    t0, tf = float(params[:t0]), float(params[:tf])
    k1, k2, k3 = params[:k1], params[:k2], params[:k3]
    a_l, a_u = float(params[:a_l]), float(params[:a_u])
    b_l, b_u = float(params[:b_l]), float(params[:b_u])
    x₃_l, x₃_u = float(params[:x₃_l]), float(params[:x₃_u])
    u_l, u_u = float(params[:u_l]), float(params[:u_u])
    a_t0, b_t0, x₃_t0 = float(params[:a_t0]), float(params[:b_t0]), float(params[:x₃_t0])

    t = ModelingToolkit.t_nounits
    D = ModelingToolkit.Differential(t)

    ModelingToolkit.@variables a(..) = a_t0 [tunable = false]
    ModelingToolkit.@variables b(..) = b_t0 [tunable = false]
    ModelingToolkit.@variables x₃(..) = x₃_t0 [tunable = false]
    ModelingToolkit.@variables u(..) [input = true, bounds = (u_l, u_u)]

    eqs = [
        D(a(t)) ~ -u(t) * (k1 * a(t) - k2 * b(t)),
        D(b(t)) ~ u(t) * (k1 * a(t) - k2 * b(t)) - (1 - u(t)) * k3 * b(t),
        D(x₃(t)) ~ (1 - u(t)) * k3 * b(t),
    ]
    # a Mayer cost: the final value of x₃
    costs = [-x₃(tf)]
    constraints = [
        ModelingToolkit.Symbolics.:≳(a(t), a_l),
        ModelingToolkit.Symbolics.:≲(a(t), a_u),
        ModelingToolkit.Symbolics.:≳(b(t), b_l),
        ModelingToolkit.Symbolics.:≲(b(t), b_u),
        ModelingToolkit.Symbolics.:≳(x₃(t), x₃_l),
        ModelingToolkit.Symbolics.:≲(x₃(t), x₃_u),
    ]

    ModelingToolkit.@named sys = ModelingToolkit.System(
        eqs, t; costs=costs, constraints=constraints
    )

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:jackson, params, init)

    return OptimalControlProblems.Problems._problem(
        :jackson, :ModelingToolkit, sys, init, params
    )
end
