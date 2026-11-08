"""The **Van der Pol oscillator control problem** is a benchmark in nonlinear optimal control.

Builds the `vanderpol` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> vanderpol(JuMPBackend(); grid_size=100)
```

The keyword `scheme` chooses the discretization: `:trapeze` (default), `:euler`,
`:euler_implicit` or `:midpoint`, as in `OptimalControl.Collocation`.
The keyword `ordering` chooses the order of the variables and constraints:
`:interleaved` (default, as in CTDirect) or `:blocks` (grouped by kind).

Default parameters and the modeling status are available through
`problem_metadata(:vanderpol)` and `OptimalControlProblems.status(:vanderpol)`.
"""
function OptimalControlProblems.vanderpol(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:vanderpol),
    scheme::Symbol=:trapeze,
    ordering::Symbol=:interleaved,
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
    kwargs...,
)
    OptimalControlProblems.Catalogue._warn_if_draft(:vanderpol, :JuMP)

    # parameters
    params = parameters_data(:vanderpol, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    ω = params[:ω]
    ε = params[:ε]
    x₁_t0 = params[:x₁_t0]
    x₂_t0 = params[:x₂_t0]

    data = ProblemData(;
        state=[Component("x₁"), Component("x₂")],
        control=[Component("u")],
        costate=["∂x₁", "∂x₂"],
        times=v -> (t0, tf),
        dynamics=(x, u, v, t) -> [x[2], ε * ω * (1 - x[1]^2) * x[2] - ω^2 * x[1] + u[1]],
        lagrange=(x, u, v, t) -> 0.5 * (x[1]^2 + x[2]^2 + u[1]^2),
        boundary=[
            (x₁_t0, (x0, xf, v) -> x0[1], x₁_t0), (x₂_t0, (x0, xf, v) -> x0[2], x₂_t0)
        ],
    )
    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:vanderpol, params, init)
    return build(data, grid_size, scheme, ordering, args...; init=init, kwargs...)
end
