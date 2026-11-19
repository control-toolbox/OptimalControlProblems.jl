"""The **clamped double_oscillator problem** is a classical benchmark in constrained optimal control.

Builds the `double_oscillator` problem with the ModelingToolkit backend.

```julia-repl
julia> using ModelingToolkit, OptimalControlProblems
julia> pb = double_oscillator(ModelingToolkitBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:double_oscillator)` and `OptimalControlProblems.status(:double_oscillator)`.
"""
function OptimalControlProblems.double_oscillator(
    ::ModelingToolkitBackend;
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
)
    OptimalControlProblems.Catalogue._warn_if_draft(:double_oscillator, :ModelingToolkit)
    params = OptimalControlProblems.parameters_data(:double_oscillator, parameters)
    t0, tf = float(params[:t0]), float(params[:tf])
    m1, m2, c, k1, k2 = params[:m1], params[:m2], params[:c], params[:k1], params[:k2]
    u_l, u_u = float(params[:u_l]), float(params[:u_u])
    x₁_t0, x₂_t0 = float(params[:x₁_t0]), float(params[:x₂_t0])

    t = ModelingToolkit.t_nounits
    D = ModelingToolkit.Differential(t)

    # x₃ and x₄ have a free initial value: no default, started from the initial guess
    ModelingToolkit.@variables x₁(..) = x₁_t0 [tunable = false]
    ModelingToolkit.@variables x₂(..) = x₂_t0 [tunable = false]
    ModelingToolkit.@variables x₃(..) [tunable = true]
    ModelingToolkit.@variables x₄(..) [tunable = true]
    ModelingToolkit.@variables u(..) [input = true, bounds = (u_l, u_u)]

    # the force on the first mass, a function of the time
    F = sin((t - t0) * 2π / (tf - t0))

    eqs = [
        D(x₁(t)) ~ x₃(t),
        D(x₂(t)) ~ x₄(t),
        D(x₃(t)) ~ -(k1 + k2) / m1 * x₁(t) + k2 / m1 * x₂(t) + 1 / m1 * F,
        D(x₄(t)) ~ k2 / m2 * x₁(t) - k2 / m2 * x₂(t) - c * (1 - u(t)) / m2 * x₄(t),
    ]
    costs = [
        ModelingToolkit.Symbolics.Integral(t in (t0, tf))(
            0.5 * (x₁(t)^2 + x₂(t)^2 + u(t)^2)
        ),
    ]
    constraints = []
    ModelingToolkit.@named sys = ModelingToolkit.System(
        eqs, t; costs=costs, constraints=constraints
    )

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:double_oscillator, params, init)

    return OptimalControlProblems.Problems._problem(
        :double_oscillator, :ModelingToolkit, sys, init, params
    )
end
