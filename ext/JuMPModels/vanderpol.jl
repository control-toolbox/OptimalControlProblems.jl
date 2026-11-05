"""The **Van der Pol oscillator control problem** is a benchmark in nonlinear optimal control.

Builds the `vanderpol` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> vanderpol(JuMPBackend(); grid_size=100)
```

Default parameters and the modeling status are available through
`problem_metadata(:vanderpol)` and `OptimalControlProblems.status(:vanderpol)`.
"""
function OptimalControlProblems.vanderpol(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:vanderpol),
    scheme::Symbol=:trapeze,
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:vanderpol, :JuMP)

    # parameters
    params = parameters_data(:vanderpol, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    ω = params[:ω]
    ε = params[:ε]
    x₁_t0 = params[:x₁_t0]
    x₂_t0 = params[:x₂_t0]

    data = ProblemData(;
        state=[Component("x₁"; start=0.1), Component("x₂"; start=0.1)],
        control=[Component("u"; start=0.1)],
        costate=["∂x₁", "∂x₂"],
        times=v -> (t0, tf),
        dynamics=(x, u, v, t) -> [x[2], ε * ω * (1 - x[1]^2) * x[2] - ω^2 * x[1] + u[1]],
        lagrange=(x, u, v, t) -> 0.5 * (x[1]^2 + x[2]^2 + u[1]^2),
        boundary=[
            (x₁_t0, (x0, xf, v) -> x0[1], x₁_t0), (x₂_t0, (x0, xf, v) -> x0[2], x₂_t0)
        ],
    )
    return build(data, grid_size, scheme, args...; kwargs...)
end
