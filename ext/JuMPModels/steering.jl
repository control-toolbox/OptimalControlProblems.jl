"""The **particle steering problem** is a classical benchmark in time-optimal control.

Builds the `steering` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> steering(JuMPBackend(); grid_size=100)
```

Default parameters and the modeling status are available through
`problem_metadata(:steering)` and `OptimalControlProblems.status(:steering)`.
"""
function OptimalControlProblems.steering(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:steering),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:steering, :JuMP)

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

    #
    tf_start = 0.6
    N = grid_size
    function gen_x0(k, i)
        if i == 1 || i == 4
            return 0.0
        elseif i == 2
            return 5.0 * k * (tf_start - t0) / N
        elseif i == 3
            return 45.0 * k * (tf_start - t0) / N
        end
    end

    data = ProblemData(;
        state=[
            Component("x$(c)"; start=[gen_x0(i, k) for i in 0:grid_size]) for
            (k, c) in enumerate("₁₂₃₄")
        ],
        control=[Component("u"; lower=u_min, upper=u_max, start=0)],
        variable=[Component("tf"; lower=tf_l, start=tf_start)],
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
    return build_trapeze(data, grid_size, args...; kwargs...)
end
