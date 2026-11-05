"""The **Goddard rocket problem** is a classical optimal control problem that models the ascent of a vertically launched rocket.

Builds the `rocket` problem with the OptimalControl backend.

```julia-repl
julia> using OptimalControlProblems
julia> rocket(OptimalControlBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:rocket)` and `OptimalControlProblems.status(:rocket)`.
"""
function OptimalControlProblems.rocket(
    ::OptimalControlBackend;
    parameters::Union{Nothing,NamedTuple}=nothing,
)
    OptimalControlProblems._warn_if_draft(:rocket, :OptimalControl)

    # parameters
    params = parameters_data(:rocket, parameters)
    t0 = params[:t0]
    h_t0 = params[:h_t0]
    v_t0 = params[:v_t0]
    m_t0 = params[:m_t0]
    g0 = params[:g0]
    Tc = params[:Tc]
    hc = params[:hc]
    vc = params[:vc]
    mc = params[:mc]
    T_l = params[:T_l]
    tf_l = params[:tf_l]

    #
    c = 0.5 * sqrt(g0 * h_t0)
    m_tf = mc * m_t0
    Dc = 0.5 * vc * (m_t0 / g0)
    Tmax = Tc * m_t0 * g0

    # Model
    ocp = @def begin
        tf ∈ R, variable
        t ∈ [t0, tf], time
        x = (h, v, m) ∈ R³, state
        T ∈ R, control

        # state constraints
        h(t) ≥ h_t0, (h_c)
        v(t) ≥ v_t0, (v_c)
        m_tf ≤ m(t) ≤ m_t0, (m_c)

        # control constraints
        T_l ≤ T(t) ≤ Tmax, (T_c)

        # time constraints
        tf ≥ tf_l, (tf_c)

        # initial conditions
        h(t0) == h_t0, (h_t0)
        v(t0) == v_t0, (v_t0)
        m(t0) == m_t0, (m_t0)

        # final conditions
        m(tf) == m_tf, (m_tf)

        # dynamics
        ẋ(t) == dynamics(h(t), v(t), m(t), T(t))

        # objective
        -h(tf) → min
    end

    # dynamics
    function dynamics(h, v, m, T)
        D = (Dc * v^2 * exp(-hc * (h - h_t0)) / h_t0)
        g = g0 * (h_t0 / h)^2
        return [v, (T - D - m * g) / m, -T / c]
    end

    # initial guess
    tf_init = 1
    xinit = t -> [1, (t / tf_init) * (1 - t / tf_init), (m_tf - m_t0) * (t / tf_init) + m_t0]
    init = (state=xinit, control=Tmax / 2, variable=tf_init)

    return OptimalControlProblems._problem(:rocket, :OptimalControl, ocp, init, params)
end
