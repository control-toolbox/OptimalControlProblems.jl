"""The **Hanging Chain problem** is a classical benchmark in optimal control.

Builds the `chain` problem with the OptimalControlScalarized backend.

```julia-repl
julia> using OptimalControlProblems
julia> chain(OptimalControlScalarizedBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:chain)` and `OptimalControlProblems.status(:chain)`.
"""
function OptimalControlProblems.chain(
    ::OptimalControlScalarizedBackend; parameters::Union{Nothing,NamedTuple}=nothing
)
    OptimalControlProblems._warn_if_draft(:chain, :OptimalControlScalarized)

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
        ∂(x₁)(t) == u(t)
        ∂(x₂)(t) == x₁(t) * √(1 + u(t)^2)
        ∂(x₃)(t) == √(1 + u(t)^2)

        # objective
        x₂(tf) → min
    end

    # initial guess
    tmin = b > a ? 1 / 4 : 3 / 4
    xinit =
        t -> [
            4 * abs(b - a) * (t - t0) / (tf - t0) * (0.5 * (t - t0) / (tf - t0) - tmin) + a,
            (
                4 * abs(b - a) * (t - t0) / (tf - t0) *
                (0.5 * (t - t0) / (tf - t0) - tmin) + a
            ) * (4 * abs(b - a) * ((t - t0) / (tf - t0) - tmin)),
            4 * abs(b - a) * ((t - t0) / (tf - t0) - tmin),
        ]
    uinit = t -> 4 * abs(b - a) * (t / tf - tmin)
    init = (state=xinit, control=uinit)

    return OptimalControlProblems._problem(
        :chain, :OptimalControlScalarized, ocp, init, params
    )
end
