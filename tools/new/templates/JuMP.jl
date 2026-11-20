"""TODO: the first sentence of the description.

Builds the `{{problem}}` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> {{problem}}(JuMPBackend(); grid_size=100)
```

The keyword `scheme` chooses the discretization: `:trapeze` (default), `:euler`,
`:euler_implicit` or `:midpoint`, as in `OptimalControl.Collocation`.
The keyword `ordering` chooses the order of the variables and constraints:
`:interleaved` (default, as in CTDirect) or `:blocks` (grouped by kind).

Default parameters and the modeling status are available through
`problem_metadata(:{{problem}})` and `OptimalControlProblems.status(:{{problem}})`.
"""
function OptimalControlProblems.{{problem}}(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:{{problem}}, JuMPBackend()),
    scheme::Symbol=:trapeze,
    ordering::Symbol=:interleaved,
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
    kwargs...,
)
    OptimalControlProblems.Catalogue._warn_if_draft(:{{problem}}, :JuMP)

    # parameters
    params = parameters_data(:{{problem}}, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    x_t0 = params[:x_t0]
    x_u = params[:x_u]

    # the continuous problem, discretized by `build`. TODO: the problem, in place of this
    # example
    data = ProblemData(;
        state=[Component("x"; upper=x_u)],
        control=[Component("u")],
        costate=["∂x"],
        times=v -> (t0, tf),
        dynamics=(x, u, v, t) -> [u[1]],
        lagrange=(x, u, v, t) -> u[1]^2,
        boundary=[(x_t0, (x0, xf, v) -> x0[1], x_t0)],
    )
    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:{{problem}}, params, init)
    return build(data, grid_size, scheme, ordering, args...; init=init, kwargs...)
end
