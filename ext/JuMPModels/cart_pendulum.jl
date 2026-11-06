"""The **cart-pole swing-up problem** is a classical benchmark in underactuated optimal control.

Builds the `cart_pendulum` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> cart_pendulum(JuMPBackend(); grid_size=100)
```

The keyword `scheme` chooses the discretization: `:trapeze` (default), `:euler`,
`:euler_implicit` or `:midpoint`, as in `OptimalControl.Collocation`.
The keyword `ordering` chooses the order of the variables and constraints:
`:interleaved` (default, as in CTDirect) or `:blocks` (grouped by kind).

Default parameters and the modeling status are available through
`problem_metadata(:cart_pendulum)` and `OptimalControlProblems.status(:cart_pendulum)`.
"""
function OptimalControlProblems.cart_pendulum(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:cart_pendulum),
    scheme::Symbol=:trapeze,
    ordering::Symbol=:interleaved,
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:cart_pendulum, :JuMP)

    # parameters
    params = parameters_data(:cart_pendulum, parameters)
    t0 = params[:t0]
    g = params[:g]
    L = params[:L]
    m = params[:m]
    I = m * L^2 / 12    # pendulum moment of inertia
    mcart = params[:mcart]
    Fex_l = params[:Fex_l]
    Fex_u = params[:Fex_u]
    x_l = params[:x_l]
    x_u = params[:x_u]
    v_l = params[:v_l]
    v_u = params[:v_u]
    tf_l = params[:tf_l]
    x_t0 = params[:x_t0]
    θ_t0 = params[:θ_t0]
    ω_t0 = params[:ω_t0]
    θ_tf = params[:θ_tf]
    ω_tf = params[:ω_tf]

    function dynamics(x, u, v)
        _, vel, θ, ω = x
        Fex = u[1]
        ddx = v[2]
        α_ddx = 1 / (I + 0.25 * m * L^2) * 0.5 * L * m * (-ddx * cos(θ) - g * sin(θ))
        ddCOG_1 = -L * sin(θ) * ω + L / 2 * cos(θ) * α_ddx + ddx
        FXFY_1 = m * ddCOG_1
        eq = -FXFY_1 + Fex - mcart * ddx
        J = mcart
        c = eq - J * ddx
        dv = -1 / J * c
        return [
            vel, dv, ω, 1 / (I + 0.25 * m * L^2) * 0.5 * L * m * (-dv * cos(θ) - g * sin(θ))
        ]
    end

    data = ProblemData(;
        state=[
            Component("x"; lower=x_l, upper=x_u, start=0.1),
            Component("v"; lower=v_l, upper=v_u, start=0.1),
            Component("θ"; start=0.1),
            Component("ω"; start=0.1),
        ],
        control=[Component("Fex"; lower=Fex_l, upper=Fex_u, start=0.1)],
        variable=[Component("tf"; lower=tf_l, start=1.0), Component("ddx"; start=0.1)],
        costate=["∂x", "∂v", "∂θ", "∂ω"],
        times=v -> (t0, v[1]),
        dynamics=(x, u, v, t) -> dynamics(x, u, v),
        cost=(integral, x0, xf, v) -> v[1],
        boundary=[
            (x_t0, (x0, xf, v) -> x0[1], x_t0),
            (θ_t0, (x0, xf, v) -> x0[3], θ_t0),
            (ω_t0, (x0, xf, v) -> x0[4], ω_t0),
            (θ_tf, (x0, xf, v) -> xf[3], θ_tf),
            (ω_tf, (x0, xf, v) -> xf[4], ω_tf),
        ],
    )
    return build(data, grid_size, scheme, ordering, args...; kwargs...)
end
