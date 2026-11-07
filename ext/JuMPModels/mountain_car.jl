"""The **Mountain Car problem** is a classic benchmark in control and reinforcement learning.

Builds the `mountain_car` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> mountain_car(JuMPBackend(); grid_size=100)
```

The keyword `scheme` chooses the discretization: `:trapeze` (default), `:euler`,
`:euler_implicit` or `:midpoint`, as in `OptimalControl.Collocation`.
The keyword `ordering` chooses the order of the variables and constraints:
`:interleaved` (default, as in CTDirect) or `:blocks` (grouped by kind).

Default parameters and the modeling status are available through
`problem_metadata(:mountain_car)` and `OptimalControlProblems.status(:mountain_car)`.
"""
function OptimalControlProblems.mountain_car(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:mountain_car),
    scheme::Symbol=:trapeze,
    ordering::Symbol=:interleaved,
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:mountain_car, :JuMP)

    # parameters
    params = parameters_data(:mountain_car, parameters)
    t0 = params[:t0]
    tf_start = params[:tf_start]
    tf_min = params[:tf_min]
    pos_t0 = params[:pos_t0]
    vel_t0 = params[:vel_t0]
    pos_tf = params[:pos_tf]
    vel_tf_min = params[:vel_tf_min]
    pos_min = params[:pos_min]
    pos_max = params[:pos_max]
    vel_min = params[:vel_min]
    vel_max = params[:vel_max]
    u_min = params[:u_min]
    u_max = params[:u_max]
    a = params[:a]
    b = params[:b]
    c = params[:c]

    data = ProblemData(;
        state=[
            Component("pos"; lower=pos_min, upper=pos_max),
            Component("vel"; lower=vel_min, upper=vel_max),
        ],
        control=[Component("u"; lower=u_min, upper=u_max)],
        variable=[Component("tf"; lower=tf_min)],
        costate=["∂pos", "∂vel"],
        times=v -> (t0, v[1]),
        dynamics=(x, u, v, t) -> [x[2], a * u[1] - b * cos(c * x[1])],
        cost=(I, x0, xf, v) -> v[1],
        boundary=[
            (pos_t0, (x0, xf, v) -> x0[1], pos_t0),
            (vel_t0, (x0, xf, v) -> x0[2], vel_t0),
            (pos_tf, (x0, xf, v) -> xf[1], pos_tf),
            (vel_tf_min, (x0, xf, v) -> xf[2], Inf),
        ],
    )
    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems._initial_guess(:mountain_car, params, init)
    return build(data, grid_size, scheme, ordering, args...; init=init, kwargs...)
end
