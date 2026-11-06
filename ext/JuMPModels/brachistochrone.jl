"""The **Brachistochrone problem** is a classical benchmark in the history of calculus of variations and optimal control.

Builds the `brachistochrone` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> brachistochrone(JuMPBackend(); grid_size=100)
```

The keyword `scheme` chooses the discretization: `:trapeze` (default), `:euler`,
`:euler_implicit` or `:midpoint`, as in `OptimalControl.Collocation`.
The keyword `ordering` chooses the order of the variables and constraints:
`:interleaved` (default, as in CTDirect) or `:blocks` (grouped by kind).

Default parameters and the modeling status are available through
`problem_metadata(:brachistochrone)` and `OptimalControlProblems.status(:brachistochrone)`.
"""
function OptimalControlProblems.brachistochrone(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:brachistochrone),
    scheme::Symbol=:trapeze,
    ordering::Symbol=:interleaved,
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:brachistochrone, :JuMP)

    # parameters
    params = parameters_data(:brachistochrone, parameters)
    g = params[:g]
    t0 = params[:t0]
    x0 = params[:x0]
    y0 = params[:y0]
    v0 = params[:v0]
    xf = params[:xf]
    yf = params[:yf]
    u_min = params[:u_min]
    u_max = params[:u_max]

    # initial guess: linear interpolation, speed estimated, u ~ 90 degrees
    interpolate(a, b) = [a + (i / grid_size) * (b - a) for i in 0:grid_size]

    data = ProblemData(;
        state=[
            Component("px"; start=interpolate(x0, xf)),
            Component("py"; start=interpolate(y0, yf)),
            Component("v"; start=[v0 + (i / grid_size) * 10.0 for i in 0:grid_size]),
        ],
        control=[Component("u"; lower=-1.57, upper=1.57, start=1.57)],
        variable=[Component("tf"; lower=0.1, upper=20.0, start=2.0)],
        costate=["∂px", "∂py", "∂v"],
        times=v -> (t0, v[1]),
        # Dymos formulation
        dynamics=(x, u, v, t) -> [x[3] * sin(u[1]), -x[3] * cos(u[1]), g * cos(u[1])],
        cost=(I, s0, sf, v) -> v[1],
        boundary=[
            (x0, (s0, sf, v) -> s0[1], x0),
            (y0, (s0, sf, v) -> s0[2], y0),
            (v0, (s0, sf, v) -> s0[3], v0),
            (xf, (s0, sf, v) -> sf[1], xf),
            (yf, (s0, sf, v) -> sf[2], yf),
        ],
    )
    return build(data, grid_size, scheme, ordering, args...; kwargs...)
end
