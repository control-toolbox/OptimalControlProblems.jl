"""The **clamped beam problem** is a classical benchmark in constrained optimal control.

Builds the `beam` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> beam(JuMPBackend(); grid_size=100)
```

The keyword `scheme` chooses the discretization: `:trapeze` (default), `:euler`,
`:euler_implicit` or `:midpoint`, as in `OptimalControl.Collocation`.
The keyword `ordering` chooses the order of the variables and constraints:
`:interleaved` (default, as in CTDirect) or `:blocks` (grouped by kind).

Default parameters and the modeling status are available through
`problem_metadata(:beam)` and `OptimalControlProblems.status(:beam)`.
"""
function OptimalControlProblems.beam(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:beam),
    scheme::Symbol=:trapeze,
    ordering::Symbol=:interleaved,
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
    kwargs...,
)
    OptimalControlProblems.Catalogue._warn_if_draft(:beam, :JuMP)

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
            Component("x₁"; lower=x₁_l, upper=x₁_u), Component("x₂")
        ],
        control=[Component("u")],
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
    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:beam, params, init)
    return build(data, grid_size, scheme, ordering, args...; init=init, kwargs...)
end
