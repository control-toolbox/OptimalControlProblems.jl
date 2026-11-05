"""The **Double Oscillator problem** is a benchmark in constrained optimal control illustrating the control of coupled mechanical systems with damping and stiffness effects.

Builds the `double_oscillator` problem with the OptimalControlScalarized backend.

```julia-repl
julia> using OptimalControlProblems
julia> double_oscillator(OptimalControlScalarizedBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:double_oscillator)` and `OptimalControlProblems.status(:double_oscillator)`.
"""
function OptimalControlProblems.double_oscillator(
    ::OptimalControlScalarizedBackend;
    parameters::Union{Nothing,NamedTuple}=nothing,
)
    OptimalControlProblems._warn_if_draft(:double_oscillator, :OptimalControlScalarized)

    # parameters
    params = parameters_data(:double_oscillator, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    m1 = params[:m1]
    m2 = params[:m2]
    c = params[:c]
    k1 = params[:k1]
    k2 = params[:k2]
    u_l = params[:u_l]
    u_u = params[:u_u]
    x₁_t0 = params[:x₁_t0]
    x₂_t0 = params[:x₂_t0]

    # model
    ocp = @def begin
        t ∈ [t0, tf], time
        x ∈ R⁴, state
        u ∈ R, control

        u_l ≤ u(t) ≤ u_u, (u_c)
        x₁(t0) == x₁_t0, (x₁_t0)
        x₂(t0) == x₂_t0, (x₂_t0)

        F = sin((t - t0) * 2π / (tf - t0))
        ∂(x₁)(t) == x₃(t)
        ∂(x₂)(t) == x₄(t)
        ∂(x₃)(t) == -(k1 + k2) / m1 * x₁(t) + k2 / m1 * x₂(t) + 1 / m1 * F
        ∂(x₄)(t) == k2 / m2 * x₁(t) - k2 / m2 * x₂(t) - c * (1 - u(t)) / m2 * x₄(t)

        0.5 * ∫(x₁(t)^2 + x₂(t)^2 + u(t)^2) → min
    end

    # initial guess
    xinit = [0.1, 0.1, 0.1, 0.1]  # [x₁, x₂, x₃, x₄]
    uinit = [0.1]  # [u]
    init = (state=xinit, control=uinit)

    return OptimalControlProblems._problem(:double_oscillator, :OptimalControlScalarized, ocp, init, params)
end
