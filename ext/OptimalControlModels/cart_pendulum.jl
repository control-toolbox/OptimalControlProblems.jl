"""The **cart-pole swing-up problem** is a classical benchmark in underactuated optimal control.

Builds the `cart_pendulum` problem with the OptimalControl backend.

```julia-repl
julia> using OptimalControlProblems
julia> cart_pendulum(OptimalControlBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:cart_pendulum)` and `OptimalControlProblems.status(:cart_pendulum)`.
"""
function OptimalControlProblems.cart_pendulum(
    ::OptimalControlBackend;
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
)
    OptimalControlProblems.Catalogue._warn_if_draft(:cart_pendulum, :OptimalControl)

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

    ocp = @def begin
        # time, variable, state and control
        w = (tf, ddx) ∈ R², variable
        t ∈ [t0, tf], time
        y = (x, v, θ, ω) ∈ R⁴, state
        Fex ∈ R, control

        # state constraints
        x_l ≤ x(t) ≤ x_u, (x_c)
        v_l ≤ v(t) ≤ v_u, (v_c)

        # control constraints
        Fex_l ≤ Fex(t) ≤ Fex_u, (Fex_c)

        # variables constraints
        tf ≥ tf_l, (tf_c)

        # initial conditions
        x(t0) == x_t0, (x_t0)
        θ(t0) == θ_t0, (θ_t0)
        ω(t0) == ω_t0, (ω_t0)

        # final conditions
        θ(tf) == θ_tf, (θ_tf)
        ω(tf) == ω_tf, (ω_tf)

        # dynamics
        ẏ(t) == dynamics(v(t), θ(t), ω(t), Fex(t), ddx)

        # objective
        tf → min
    end

    # dynamics
    function dynamics(v, θ, ω, Fex, ddx)

        #
        α(ddx) = 1 / (I + 0.25 * m * L^2) * 0.5 * L * m * (-ddx * cos(θ) - g * sin(θ))
        ddCOG = L * ω * [-sin(θ), cos(θ)] + L / 2 * [cos(θ), sin(θ)] * α(ddx) + [ddx, 0]
        FXFY = m * ddCOG + [0, m * g]
        eq = -FXFY[1] + Fex - mcart * ddx # # eq = J ddx + c
        J = mcart # should be -(m+mcart) but was mcart?
        c = eq - J * ddx

        #
        ẋ = v
        v̇ = -1 / J * c
        θ̇ = ω
        ω̇ = α(v̇)

        return [ẋ, v̇, θ̇, ω̇]
    end

    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems.Metadata._initial_guess(:cart_pendulum, params, init)

    return OptimalControlProblems.Problems._problem(:cart_pendulum, :OptimalControl, ocp, init, params)
end
