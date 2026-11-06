"""The **Robbins benchmark problem** is a classical optimal control problem introduced by Robbins (1980), involving a **third-order state constraint**:.

Builds the `robbins` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> robbins(JuMPBackend(); grid_size=100)
```

The keyword `scheme` chooses the discretization: `:trapeze` (default), `:euler`,
`:euler_implicit` or `:midpoint`, as in `OptimalControl.Collocation`.
The keyword `ordering` chooses the order of the variables and constraints:
`:interleaved` (default, as in CTDirect) or `:blocks` (grouped by kind).

Default parameters and the modeling status are available through
`problem_metadata(:robbins)` and `OptimalControlProblems.status(:robbins)`.
"""
function OptimalControlProblems.robbins(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:robbins),
    scheme::Symbol=:trapeze,
    ordering::Symbol=:interleaved,
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
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
            Component("x₁"; lower=x₁_l),
            Component("x₂"),
            Component("x₃"),
        ],
        control=[Component("u")],
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
    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems._initial_guess(:robbins, params, init)
    return build(data, grid_size, scheme, ordering, args...; init=init, kwargs...)
end
