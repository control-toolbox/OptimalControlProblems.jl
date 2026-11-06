"""The **Double Oscillator problem** is a benchmark in constrained optimal control illustrating the control of coupled mechanical systems with damping and stiffness effects.

Builds the `double_oscillator` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> double_oscillator(JuMPBackend(); grid_size=100)
```

The keyword `scheme` chooses the discretization: `:trapeze` (default), `:euler`,
`:euler_implicit` or `:midpoint`, as in `OptimalControl.Collocation`.
The keyword `ordering` chooses the order of the variables and constraints:
`:interleaved` (default, as in CTDirect) or `:blocks` (grouped by kind).

Default parameters and the modeling status are available through
`problem_metadata(:double_oscillator)` and `OptimalControlProblems.status(:double_oscillator)`.
"""
function OptimalControlProblems.double_oscillator(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:double_oscillator),
    scheme::Symbol=:trapeze,
    ordering::Symbol=:interleaved,
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:double_oscillator, :JuMP)

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

    function dynamics(x, u, t)
        x₁, x₂, x₃, x₄ = x
        F = sin((t - t0) * 2π / (tf - t0))
        return [
            x₃,
            x₄,
            -(k1 + k2) / m1 * x₁ + k2 / m1 * x₂ + 1 / m1 * F,
            k2 / m2 * x₁ - k2 / m2 * x₂ - c * (1 - u[1]) / m2 * x₄,
        ]
    end

    data = ProblemData(;
        state=[Component("x$(c)"; start=0.1) for c in "₁₂₃₄"],
        control=[Component("u"; lower=u_l, upper=u_u, start=0.1)],
        costate=["∂x₁", "∂x₂", "∂x₃", "∂x₄"],
        times=v -> (t0, tf),
        dynamics=(x, u, v, t) -> dynamics(x, u, t),
        lagrange=(x, u, v, t) -> 0.5 * (x[1]^2 + x[2]^2 + u[1]^2),
        boundary=[
            (x₁_t0, (x0, xf, v) -> x0[1], x₁_t0), (x₂_t0, (x0, xf, v) -> x0[2], x₂_t0)
        ],
    )
    return build(data, grid_size, scheme, ordering, args...; kwargs...)
end
