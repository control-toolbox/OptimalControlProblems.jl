"""The **Bryson–Denham problem** is a classic optimal control benchmark involving a state inequality constraint.

Builds the `bryson_denham` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> bryson_denham(JuMPBackend(); grid_size=100)
```

Default parameters and the modeling status are available through
`problem_metadata(:bryson_denham)` and `OptimalControlProblems.status(:bryson_denham)`.
"""
function OptimalControlProblems.bryson_denham(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:bryson_denham),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:bryson_denham, :JuMP)
    params = parameters_data(:bryson_denham, parameters)
    t0, tf = params[:t0], params[:tf]
    x1_t0, x2_t0 = params[:x1_t0], params[:x2_t0]
    x1_tf, x2_tf = params[:x1_tf], params[:x2_tf]
    x1_max = params[:x1_max]

    data = ProblemData(;
        state=[Component("x1"; upper=x1_max, start=0.0), Component("x2"; start=0.0)],
        control=[Component("u"; start=0.0)],
        costate=["p1", "p2"],
        times=v -> (t0, tf),
        dynamics=(x, u, v, t) -> [x[2], u[1]],
        lagrange=(x, u, v, t) -> 0.5 * u[1]^2,
        boundary=[
            (x1_t0, (x0, xf, v) -> x0[1], x1_t0),
            (x2_t0, (x0, xf, v) -> x0[2], x2_t0),
            (x1_tf, (x0, xf, v) -> xf[1], x1_tf),
            (x2_tf, (x0, xf, v) -> xf[2], x2_tf),
        ],
    )
    return build_trapeze(data, grid_size, args...; kwargs...)
end
