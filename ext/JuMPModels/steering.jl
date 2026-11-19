"""The **particle steering problem** is a classical benchmark in time-optimal control.

Builds the `steering` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> steering(JuMPBackend(); grid_size=100)
```

The keyword `scheme` chooses the discretization: `:trapeze` (default), `:euler`,
`:euler_implicit` or `:midpoint`, as in `OptimalControl.Collocation`.
The keyword `ordering` chooses the order of the variables and constraints:
`:interleaved` (default, as in CTDirect) or `:blocks` (grouped by kind).

Default parameters and the modeling status are available through
`problem_metadata(:steering)` and `OptimalControlProblems.status(:steering)`.
"""
function OptimalControlProblems.steering(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:steering),
    scheme::Symbol=:trapeze,
    ordering::Symbol=:interleaved,
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
    kwargs...,
)
    OptimalControlProblems.Catalogue._warn_if_draft(:steering, :JuMP)

    # parameters
    params = parameters_data(:steering, parameters)
    t0 = params[:t0]
    a = params[:a]
    u_min = params[:u_min]
    u_max = params[:u_max]
    tf_l = params[:tf_l]
    x₁_t0 = params[:x₁_t0]
    x₂_t0 = params[:x₂_t0]
    x₃_t0 = params[:x₃_t0]
    x₄_t0 = params[:x₄_t0]
    x₂_tf = params[:x₂_tf]
    x₃_tf = params[:x₃_tf]
    x₄_tf = params[:x₄_tf]

    data = ProblemData(;
        state=[Component("x$(c)") for (k, c) in enumerate("₁₂₃₄")],
        control=[Component("u"; lower=u_min, upper=u_max)],
        variable=[Component("tf"; lower=tf_l)],
        costate=["∂x₁", "∂x₂", "∂x₃", "∂x₄"],
        times=v -> (t0, v[1]),
        dynamics=(x, u, v, t) -> [x[3], x[4], a * cos(u[1]), a * sin(u[1])],
        cost=(I, x0, xf, v) -> v[1],
        boundary=[
            (x₁_t0, (x0, xf, v) -> x0[1], x₁_t0),
            (x₂_t0, (x0, xf, v) -> x0[2], x₂_t0),
            (x₃_t0, (x0, xf, v) -> x0[3], x₃_t0),
            (x₄_t0, (x0, xf, v) -> x0[4], x₄_t0),
            (x₂_tf, (x0, xf, v) -> xf[2], x₂_tf),
            (x₃_tf, (x0, xf, v) -> xf[3], x₃_tf),
            (x₄_tf, (x0, xf, v) -> xf[4], x₄_tf),
        ],
    )
    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:steering, params, init)
    return build(data, grid_size, scheme, ordering, args...; init=init, kwargs...)
end
