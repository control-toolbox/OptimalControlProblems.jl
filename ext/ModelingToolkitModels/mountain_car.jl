"""The **Mountain Car problem** is a classical benchmark in optimal control: reach the top of a hill in minimal time, with a car that is too weak to climb it directly.

Builds the `mountain_car` problem with the ModelingToolkit backend.

```julia-repl
julia> using ModelingToolkit, OptimalControlProblems
julia> pb = mountain_car(ModelingToolkitBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:mountain_car)` and `OptimalControlProblems.status(:mountain_car)`.
"""
function OptimalControlProblems.mountain_car(
    ::ModelingToolkitBackend;
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
)
    OptimalControlProblems.Catalogue._warn_if_draft(:mountain_car, :ModelingToolkit)
    params = OptimalControlProblems.parameters_data(:mountain_car, parameters)
    t0 = float(params[:t0])
    tf_min = float(params[:tf_min])
    pos_t0, vel_t0 = float(params[:pos_t0]), float(params[:vel_t0])
    pos_tf, vel_tf_min = float(params[:pos_tf]), float(params[:vel_tf_min])
    pos_min, pos_max = float(params[:pos_min]), float(params[:pos_max])
    vel_min, vel_max = float(params[:vel_min]), float(params[:vel_max])
    u_min, u_max = float(params[:u_min]), float(params[:u_max])
    a, b, c = params[:a], params[:b], params[:c]

    # The final time is free. Corleone integrates on a fixed horizon: the time is reduced to
    # s = (t - t0) / (tf - t0) in [0, 1], and the final time `tf`, the variable of the problem,
    # is a parameter to optimize. The derivatives are multiplied by the duration tf - t0.
    s = ModelingToolkit.t_nounits
    D = ModelingToolkit.Differential(s)

    ModelingToolkit.@parameters tf [tunable = true, bounds = (tf_min, Inf)]
    ModelingToolkit.@variables pos(..) = pos_t0 [tunable = false]
    ModelingToolkit.@variables vel(..) = vel_t0 [tunable = false]
    ModelingToolkit.@variables u(..) [input = true, bounds = (u_min, u_max)]

    t = s
    eqs = [
        D(pos(t)) ~ (tf - t0) * vel(t),
        D(vel(t)) ~ (tf - t0) * (a * u(t) - b * cos(c * pos(t))),
    ]
    # a Mayer cost: the final time
    costs = [tf]
    # the final conditions are at s = 1
    constraints = [
        pos(1.0) ~ pos_tf,
        ModelingToolkit.Symbolics.:≳(vel(1.0), vel_tf_min),
        ModelingToolkit.Symbolics.:≳(pos(t), pos_min),
        ModelingToolkit.Symbolics.:≲(pos(t), pos_max),
        ModelingToolkit.Symbolics.:≳(vel(t), vel_min),
        ModelingToolkit.Symbolics.:≲(vel(t), vel_max),
    ]

    ModelingToolkit.@named sys = ModelingToolkit.System(
        eqs, t; costs=costs, constraints=constraints
    )

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:mountain_car, params, init)

    return OptimalControlProblems.Problems._problem(
        :mountain_car, :ModelingToolkit, sys, init, params
    )
end
