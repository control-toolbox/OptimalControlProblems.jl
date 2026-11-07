"""The **Van der Pol oscillator control problem** is a benchmark in nonlinear optimal control.

Builds the `vanderpol` problem with the OptimalControlScalarized backend.

```julia-repl
julia> using OptimalControlProblems
julia> vanderpol(OptimalControlScalarizedBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:vanderpol)` and `OptimalControlProblems.status(:vanderpol)`.
"""
function OptimalControlProblems.vanderpol(
    ::OptimalControlScalarizedBackend;
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
)
    OptimalControlProblems._warn_if_draft(:vanderpol, :OptimalControlScalarized)

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

        ∂(x₁)(t) == x₂(t)
        ∂(x₂)(t) == ε * ω * (1 - x₁(t)^2) * x₂(t) - ω^2 * x₁(t) + u(t)

        0.5∫(x₁(t)^2 + x₂(t)^2 + u(t)^2) → min
    end

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems._initial_guess(:vanderpol, params, init)

    return OptimalControlProblems._problem(:vanderpol, :OptimalControlScalarized, ocp, init, params)
end
