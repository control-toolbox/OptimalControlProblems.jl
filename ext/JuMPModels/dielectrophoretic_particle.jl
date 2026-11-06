"""The **dielectrophoretic particle problem** is a classical time-optimal control benchmark for microfluidic particle manipulation.

Builds the `dielectrophoretic_particle` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> dielectrophoretic_particle(JuMPBackend(); grid_size=100)
```

The keyword `scheme` chooses the discretization: `:trapeze` (default), `:euler`,
`:euler_implicit` or `:midpoint`, as in `OptimalControl.Collocation`.
The keyword `ordering` chooses the order of the variables and constraints:
`:interleaved` (default, as in CTDirect) or `:blocks` (grouped by kind).

Default parameters and the modeling status are available through
`problem_metadata(:dielectrophoretic_particle)` and `OptimalControlProblems.status(:dielectrophoretic_particle)`.
"""
function OptimalControlProblems.dielectrophoretic_particle(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:dielectrophoretic_particle),
    scheme::Symbol=:trapeze,
    ordering::Symbol=:interleaved,
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:dielectrophoretic_particle, :JuMP)

    # parameters
    params = parameters_data(:dielectrophoretic_particle, parameters)
    t0 = params[:t0]
    x_t0 = params[:x_t0]
    y_t0 = params[:y_t0]
    x_tf = params[:x_tf]
    α = params[:α]
    c = params[:c]
    u_l = params[:u_l]
    u_u = params[:u_u]
    tf_l = params[:tf_l]

    data = ProblemData(;
        state=[Component("x"; start=1), Component("y"; start=0.5)],
        control=[Component("u"; lower=u_l, upper=u_u, start=0.5)],
        variable=[Component("tf"; lower=tf_l, start=8)],
        costate=["∂x", "∂y"],
        times=v -> (t0, v[1]),
        dynamics=(x, u, v, t) -> [x[2] * u[1] + α * u[1]^2, -c * x[2] + u[1]],
        cost=(I, x0, xf, v) -> v[1],
        boundary=[
            (x_t0, (x0, xf, v) -> x0[1], x_t0),
            (y_t0, (x0, xf, v) -> x0[2], y_t0),
            (x_tf, (x0, xf, v) -> xf[1], x_tf),
        ],
    )
    return build(data, grid_size, scheme, ordering, args...; kwargs...)
end
