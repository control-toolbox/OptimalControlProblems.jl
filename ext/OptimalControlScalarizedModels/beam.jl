"""The **clamped beam problem** is a classical benchmark in constrained optimal control.

Builds the `beam` problem with the OptimalControlScalarized backend.

```julia-repl
julia> using OptimalControlProblems
julia> beam(OptimalControlScalarizedBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:beam)` and `OptimalControlProblems.status(:beam)`.
"""
function OptimalControlProblems.beam(
    ::OptimalControlScalarizedBackend;
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
)
    OptimalControlProblems._warn_if_draft(:beam, :OptimalControlScalarized)

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

    # model
    ocp = @def begin
        t ∈ [t0, tf], time
        x ∈ R², state
        u ∈ R, control

        x(t0) == [x₁_t0, x₂_t0]
        x(tf) == [x₁_tf, x₂_tf]
        x₁_l ≤ x₁(t) ≤ x₁_u

        ∂(x₁)(t) == x₂(t)
        ∂(x₂)(t) == u(t)

        ∫(u(t)^2) → min
    end

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems._initial_guess(:beam, params, init)

    return OptimalControlProblems._problem(
        :beam, :OptimalControlScalarized, ocp, init, params
    )
end
