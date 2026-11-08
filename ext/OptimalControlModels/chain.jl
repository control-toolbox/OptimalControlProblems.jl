"""The **Hanging Chain problem** is a classical benchmark in optimal control.

Builds the `chain` problem with the OptimalControl backend.

```julia-repl
julia> using OptimalControlProblems
julia> chain(OptimalControlBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:chain)` and `OptimalControlProblems.status(:chain)`.
"""
function OptimalControlProblems.chain(
    ::OptimalControlBackend; parameters::Union{Nothing,NamedTuple}=nothing, init=nothing
)
    OptimalControlProblems._warn_if_draft(:chain, :OptimalControl)

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

    # model
    ocp = @def begin
        t ∈ [t0, tf], time
        x ∈ R³, state
        u ∈ R, control

        # initial conditions
        x₁(t0) == x₁_t0, (x₁_t0)
        x₂(t0) == x₂_t0, (x₂_t0)
        x₃(t0) == x₃_t0, (x₃_t0)

        # final conditions
        x₁(tf) == x₁_tf, (x₁_tf)
        x₃(tf) == x₃_tf, (x₃_tf)

        # dynamics
        ẋ(t) == dynamics(x(t), u(t))

        # objective
        x₂(tf) → min
    end

    # dynamics
    function dynamics(x, u)
        return [u, x[1] * √(1 + u^2), √(1 + u^2)]
    end

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems._initial_guess(:chain, params, init)

    return OptimalControlProblems._problem(:chain, :OptimalControl, ocp, init, params)
end
