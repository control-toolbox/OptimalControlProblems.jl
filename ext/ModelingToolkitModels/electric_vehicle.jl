"""The **Electric Vehicle (EV) trajectory problem** is a benchmark in optimal control for energy-efficient driving.

Builds the `electric_vehicle` problem with the ModelingToolkit backend.

```julia-repl
julia> using ModelingToolkit, OptimalControlProblems
julia> pb = electric_vehicle(ModelingToolkitBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:electric_vehicle)` and `OptimalControlProblems.status(:electric_vehicle)`.
"""
function OptimalControlProblems.electric_vehicle(
    ::ModelingToolkitBackend;
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
)
    OptimalControlProblems.Catalogue._warn_if_draft(:electric_vehicle, :ModelingToolkit)
    params = OptimalControlProblems.parameters_data(:electric_vehicle, parameters)
    t0, tf = float(params[:t0]), float(params[:tf])
    b1, b2 = params[:b1], params[:b2]
    h0, h1, h2 = params[:h0], params[:h1], params[:h2]
    α0, α1, α2, α3 = params[:α0], params[:α1], params[:α2], params[:α3]
    x_t0, v_t0 = float(params[:x_t0]), float(params[:v_t0])
    x_tf, v_tf = float(params[:x_tf]), float(params[:v_tf])

    t = ModelingToolkit.t_nounits
    D = ModelingToolkit.Differential(t)

    ModelingToolkit.@variables x(..) = x_t0 [tunable = false]
    ModelingToolkit.@variables v(..) = v_t0 [tunable = false]
    ModelingToolkit.@variables u(..) [input = true]

    # the slope of the road at the position x
    road(x) = α0 + α1 * x + α2 * x^2 + α3 * x^3

    eqs = [D(x(t)) ~ v(t), D(v(t)) ~ h1 * u(t) - h2 * v(t)^2 - h0 - road(x(t))]
    costs = [
        ModelingToolkit.Symbolics.Integral(t in (t0, tf))(b1 * u(t) * v(t) + b2 * u(t)^2),
    ]
    constraints = [x(tf) ~ x_tf, v(tf) ~ v_tf]

    ModelingToolkit.@named sys = ModelingToolkit.System(
        eqs, t; costs=costs, constraints=constraints
    )

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:electric_vehicle, params, init)

    return OptimalControlProblems.Problems._problem(
        :electric_vehicle, :ModelingToolkit, sys, init, params
    )
end
