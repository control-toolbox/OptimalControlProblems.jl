"""The **hang glider problem** is a classical benchmark in optimal control.

Builds the `glider` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> glider(JuMPBackend(); grid_size=100)
```

The keyword `scheme` chooses the discretization: `:trapeze` (default), `:euler`,
`:euler_implicit` or `:midpoint`, as in `OptimalControl.Collocation`.
The keyword `ordering` chooses the order of the variables and constraints:
`:interleaved` (default, as in CTDirect) or `:blocks` (grouped by kind).

Default parameters and the modeling status are available through
`problem_metadata(:glider)` and `OptimalControlProblems.status(:glider)`.
"""
function OptimalControlProblems.glider(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:glider),
    scheme::Symbol=:trapeze,
    ordering::Symbol=:interleaved,
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
    kwargs...,
)
    OptimalControlProblems.Catalogue._warn_if_draft(:glider, :JuMP)

    # parameters
    params = parameters_data(:glider, parameters)
    t0 = params[:t0]
    x_t0 = params[:x_t0]
    y_t0 = params[:y_t0]
    y_tf = params[:y_tf]
    vx_t0 = params[:vx_t0]
    vx_tf = params[:vx_tf]
    vy_t0 = params[:vy_t0]
    vy_tf = params[:vy_tf]
    u_c = params[:u_c]
    r_t0 = params[:r_t0]
    m = params[:m]
    g = params[:g]
    c0 = params[:c0]
    c1 = params[:c1]
    S = params[:S]
    ρ = params[:ρ]
    cL_min = params[:cL_min]
    cL_max = params[:cL_max]
    tf_l = params[:tf_l]
    x_l = params[:x_l]
    vx_l = params[:vx_l]

    function dynamics(x, u)
        X, _, vx, vy = x
        cL = u[1]
        r = (X / r_t0 - 2.5)^2
        w = vy - u_c * (1 - r) * exp(-r)
        v = √(vx^2 + w^2)
        D = 0.5 * (c0 + c1 * cL^2) * ρ * S * v^2
        L = 0.5 * cL * ρ * S * v^2
        return [vx, vy, -(L * w + D * vx) / (m * v), (L * vx - D * w) / (m * v) - g]
    end

    N = grid_size
    data = ProblemData(;
        state=[
            Component("x"; lower=x_l),
            Component("y"),
            Component("vx"; lower=vx_l),
            Component("vy"),
        ],
        control=[Component("cL"; lower=cL_min, upper=cL_max)],
        variable=[Component("tf"; lower=tf_l)],
        costate=["∂x", "∂y", "∂vx", "∂vy"],
        times=v -> (t0, v[1]),
        dynamics=(x, u, v, t) -> dynamics(x, u),
        cost=(I, x0, xf, v) -> -xf[1],
        boundary=[
            (x_t0, (x0, xf, v) -> x0[1], x_t0),
            (y_t0, (x0, xf, v) -> x0[2], y_t0),
            (vx_t0, (x0, xf, v) -> x0[3], vx_t0),
            (vy_t0, (x0, xf, v) -> x0[4], vy_t0),
            (y_tf, (x0, xf, v) -> xf[2], y_tf),
            (vx_tf, (x0, xf, v) -> xf[3], vx_tf),
            (vy_tf, (x0, xf, v) -> xf[4], vy_tf),
        ],
    )
    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:glider, params, init)
    return build(data, grid_size, scheme, ordering, args...; init=init, kwargs...)
end
