"""The **clamped beam problem** is a classical benchmark in constrained optimal control.

Builds the `beam` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> beam(JuMPBackend(); grid_size=100)
```

Default parameters and the modeling status are available through
`problem_metadata(:beam)` and `OptimalControlProblems.status(:beam)`.
"""
function OptimalControlProblems.beam(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:beam),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:beam, :JuMP)

    # parameters
    params = parameters_data(:beam, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    x₁_l = params[:x₁_l]
    x₁_u = params[:x₁_u]
    x₁_t0 = params[:x₁_t0]
    x₂_t0 = params[:x₂_t0]
    x₁_tf = params[:x₁_tf]
    x₂_tf = params[:x₂_tf]

    data = ProblemData(;
        state=[
            Component("x₁"; lower=x₁_l, upper=x₁_u, start=0.05), Component("x₂"; start=0.1)
        ],
        control=[Component("u"; start=0.1)],
        costate=["∂x₁", "∂x₂"],
        times=v -> (t0, tf),
        dynamics=(x, u, v, t) -> [x[2], u[1]],
        lagrange=(x, u, v, t) -> u[1]^2,
        boundary=[
            (x₁_t0, (x0, xf, v) -> x0[1], x₁_t0),
            (x₂_t0, (x0, xf, v) -> x0[2], x₂_t0),
            (x₁_tf, (x0, xf, v) -> xf[1], x₁_tf),
            (x₂_tf, (x0, xf, v) -> xf[2], x₂_tf),
        ],
    )
    return build_trapeze(data, grid_size, args...; kwargs...)
end
