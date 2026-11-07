"""The **Van der Pol oscillator control problem** is a benchmark in nonlinear optimal control.

Builds the `vanderpol` problem with the OptimalControl backend.

```julia-repl
julia> using OptimalControlProblems
julia> vanderpol(OptimalControlBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:vanderpol)` and `OptimalControlProblems.status(:vanderpol)`.
"""
function OptimalControlProblems.vanderpol(
    ::OptimalControlBackend;
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
)
    OptimalControlProblems._warn_if_draft(:vanderpol, :OptimalControl)

    # parameters
    params = parameters_data(:vanderpol, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    ω = params[:ω]
    ε = params[:ε]
    x₁_t0 = params[:x₁_t0]
    x₂_t0 = params[:x₂_t0]

    # model
    ocp = @def begin
        t ∈ [t0, tf], time
        x ∈ R², state
        u ∈ R, control

        x(t0) == [x₁_t0, x₂_t0]

        ẋ(t) == [x[2](t), ε * ω * (1 - x[1](t)^2) * x[2](t) - ω^2 * x[1](t) + u(t)]

        0.5∫(x[1](t)^2 + x[2](t)^2 + u(t)^2) → min
    end

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems._initial_guess(:vanderpol, params, init)

    return OptimalControlProblems._problem(:vanderpol, :OptimalControl, ocp, init, params)
end
