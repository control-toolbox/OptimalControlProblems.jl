"""The **Electric Vehicle (EV) trajectory problem** is a benchmark in optimal control for energy-efficient driving.

Builds the `electric_vehicle` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> electric_vehicle(JuMPBackend(); grid_size=100)
```

Default parameters and the modeling status are available through
`problem_metadata(:electric_vehicle)` and `OptimalControlProblems.status(:electric_vehicle)`.
"""
function OptimalControlProblems.electric_vehicle(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:electric_vehicle),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:electric_vehicle, :JuMP)

    # parameters
    params = parameters_data(:electric_vehicle, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    b1 = params[:b1]
    b2 = params[:b2]
    h0 = params[:h0]
    h1 = params[:h1]
    h2 = params[:h2]
    α0 = params[:α0]
    α1 = params[:α1]
    α2 = params[:α2]
    α3 = params[:α3]
    x_t0 = params[:x_t0]
    v_t0 = params[:v_t0]
    x_tf = params[:x_tf]
    v_tf = params[:v_tf]

    function dynamics(x, u)
        X, v = x
        road = α0 + α1 * X + α2 * X^2 + α3 * X^3
        return [v, h1 * u[1] - h2 * v^2 - h0 - road]
    end

    data = ProblemData(;
        state=[Component("x"; start=0.1), Component("v"; start=0.1)],
        control=[Component("u"; start=0.1)],
        costate=["∂x", "∂v"],
        times=v -> (t0, tf),
        dynamics=(x, u, v, t) -> dynamics(x, u),
        lagrange=(x, u, v, t) -> b1 * u[1] * x[2] + b2 * u[1]^2,
        boundary=[
            (x_t0, (x0, xf, v) -> x0[1], x_t0),
            (v_t0, (x0, xf, v) -> x0[2], v_t0),
            (x_tf, (x0, xf, v) -> xf[1], x_tf),
            (v_tf, (x0, xf, v) -> xf[2], v_tf),
        ],
    )
    return build_trapeze(data, grid_size, args...; kwargs...)
end
