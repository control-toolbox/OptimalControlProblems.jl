"""The **Robbins benchmark problem** is a classical optimal control problem introduced by Robbins (1980), involving a **third-order state constraint**:.

Builds the `robbins` problem with the OptimalControl backend.

```julia-repl
julia> using OptimalControlProblems
julia> robbins(OptimalControlBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:robbins)` and `OptimalControlProblems.status(:robbins)`.
"""
function OptimalControlProblems.robbins(
    ::OptimalControlBackend; parameters::Union{Nothing,NamedTuple}=nothing, init=nothing
)
    OptimalControlProblems._warn_if_draft(:robbins, :OptimalControl)

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

    # model
    ocp = @def begin
        t ∈ [t0, tf], time
        x ∈ R³, state
        u ∈ R, control

        x[1](t) ≥ x₁_l

        x(t0) == [x₁_t0, x₂_t0, x₃_t0]
        x(tf) == [x₁_tf, x₂_tf, x₃_tf]

        ẋ(t) == [x[2](t), x[3](t), u(t)]

        ∫(α * x[1](t) + β * x[1](t)^2 + γ * u(t)^2) → min
    end

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems._initial_guess(:robbins, params, init)

    return OptimalControlProblems._problem(:robbins, :OptimalControl, ocp, init, params)
end
