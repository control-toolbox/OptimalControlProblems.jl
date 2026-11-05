"""The **Robbins benchmark problem** is a classical optimal control problem introduced by Robbins (1980), involving a **third-order state constraint**:.

Builds the `robbins` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> robbins(JuMPBackend(); grid_size=100)
```

Default parameters and the modeling status are available through
`problem_metadata(:robbins)` and `OptimalControlProblems.status(:robbins)`.
"""
function OptimalControlProblems.robbins(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:robbins),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:robbins, :JuMP)

    # parameters
    params = parameters_data(:robbins, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    α = params[:α]
    β = params[:β]
    γ = params[:γ]
    x₁_l = params[:x₁_l]
    x₁_t0 = params[:x₁_t0]
    x₂_t0 = params[:x₂_t0]
    x₃_t0 = params[:x₃_t0]
    x₁_tf = params[:x₁_tf]
    x₂_tf = params[:x₂_tf]
    x₃_tf = params[:x₃_tf]

    data = ProblemData(;
        state=[
            Component("x₁"; lower=x₁_l, start=0.1),
            Component("x₂"; start=0.1),
            Component("x₃"; start=0.1),
        ],
        control=[Component("u"; start=0.1)],
        costate=["∂x₁", "∂x₂", "∂x₃"],
        times=v -> (t0, tf),
        dynamics=(x, u, v, t) -> [x[2], x[3], u[1]],
        lagrange=(x, u, v, t) -> (α * x[1] + β * x[1]^2 + γ * u[1]^2),
        boundary=[
            (x₁_t0, (x0, xf, v) -> x0[1], x₁_t0),
            (x₂_t0, (x0, xf, v) -> x0[2], x₂_t0),
            (x₃_t0, (x0, xf, v) -> x0[3], x₃_t0),
            (x₁_tf, (x0, xf, v) -> xf[1], x₁_tf),
            (x₂_tf, (x0, xf, v) -> xf[2], x₂_tf),
            (x₃_tf, (x0, xf, v) -> xf[3], x₃_tf),
        ],
    )
    return build_trapeze(data, grid_size, args...; kwargs...)
end
