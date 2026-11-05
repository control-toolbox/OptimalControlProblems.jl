"""The **Hanging Chain problem** is a classical benchmark in optimal control.

Builds the `chain` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> chain(JuMPBackend(); grid_size=100)
```

Default parameters and the modeling status are available through
`problem_metadata(:chain)` and `OptimalControlProblems.status(:chain)`.
"""
function OptimalControlProblems.chain(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:chain),
    scheme::Symbol=:trapeze,
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:chain, :JuMP)

    # parameters
    params = parameters_data(:chain, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    L = params[:L]
    a = params[:a]
    b = params[:b]
    x₁_t0 = a
    x₂_t0 = params[:x₂_t0]
    x₃_t0 = params[:x₃_t0]
    x₁_tf = b
    x₃_tf = L

    #
    tmin = b > a ? 1 / 4 : 3 / 4

    # initial guess, on the time grid
    N = grid_size
    τ = [(t0 + k * (tf - t0) / N - t0) / (tf - t0) for k in 0:N]
    u_start = [4 * abs(b - a) * (s - tmin) for s in τ]
    x₁_start = [4 * abs(b - a) * s * (0.5 * s - tmin) + a for s in τ]

    data = ProblemData(;
        state=[
            Component("x₁"; start=x₁_start),
            Component("x₂"; start=x₁_start .* u_start),
            Component("x₃"; start=u_start),
        ],
        control=[Component("u"; start=u_start)],
        costate=["∂x₁", "∂x₂", "∂x₃"],
        times=v -> (t0, tf),
        dynamics=(x, u, v, t) -> [u[1], x[1] * √(1 + u[1]^2), √(1 + u[1]^2)],
        cost=(I, x0, xf, v) -> xf[2],
        boundary=[
            (x₁_t0, (x0, xf, v) -> x0[1], x₁_t0),
            (x₂_t0, (x0, xf, v) -> x0[2], x₂_t0),
            (x₃_t0, (x0, xf, v) -> x0[3], x₃_t0),
            (x₁_tf, (x0, xf, v) -> xf[1], x₁_tf),
            (x₃_tf, (x0, xf, v) -> xf[3], x₃_tf),
        ],
    )
    return build(data, grid_size, scheme, args...; kwargs...)
end
