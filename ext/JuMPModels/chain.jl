"""The **Hanging Chain problem** is a classical benchmark in optimal control.

Builds the `chain` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> chain(JuMPBackend(); grid_size=100)
```

The keyword `scheme` chooses the discretization: `:trapeze` (default), `:euler`,
`:euler_implicit` or `:midpoint`, as in `OptimalControl.Collocation`.
The keyword `ordering` chooses the order of the variables and constraints:
`:interleaved` (default, as in CTDirect) or `:blocks` (grouped by kind).

Default parameters and the modeling status are available through
`problem_metadata(:chain)` and `OptimalControlProblems.status(:chain)`.
"""
function OptimalControlProblems.chain(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:chain),
    scheme::Symbol=:trapeze,
    ordering::Symbol=:interleaved,
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
    kwargs...,
)
    OptimalControlProblems.Catalogue._warn_if_draft(:chain, :JuMP)

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

    data = ProblemData(;
        state=[Component("x₁"), Component("x₂"), Component("x₃")],
        control=[Component("u")],
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
    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:chain, params, init)
    return build(data, grid_size, scheme, ordering, args...; init=init, kwargs...)
end
