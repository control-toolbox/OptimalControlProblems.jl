"""The **Goddard rocket problem** is a classical optimal control problem that models the ascent of a vertically launched rocket.

Builds the `rocket` problem with the OptimalControlScalarized backend.

```julia-repl
julia> using OptimalControlProblems
julia> rocket(OptimalControlScalarizedBackend(); grid_size=100)
```

Default parameters and the modeling status are available through `metadata(:rocket)` and
`status(:rocket)`.
"""
function OptimalControlProblems.rocket(
    ::OptimalControlScalarizedBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:rocket),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    OptimalControlProblems._warn_if_draft(:rocket, :OptimalControlScalarized)

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
        D = (Dc * v(t)^2 * exp(-hc * (h(t) - h_t0)) / h_t0)
        g = g0 * (h_t0 / h(t))^2
        ∂(h)(t) == v(t)
        ∂(v)(t) == (T(t) - D - m(t) * g) / m(t)
        ∂(m)(t) == -T(t) / c

        # objective
        -h(tf) → min
    end

    # initial guess
    N = grid_size
    tf_init = 1
    xinit = [[1, i / N * (1 - i / N), (m_tf - m_t0) * (i / N) + m_t0] for i in 0:N]
    time_vec = LinRange(0, tf_init, N + 1)
    init = (time=time_vec, state=xinit, control=Tmax / 2, variable=tf_init)

    # discretise the optimal control problem
    docp = direct_transcription(
        ocp,
        description...;
        lagrange_to_mayer=false,
        init=init,
        grid_size=grid_size,
        disc_method=:trapeze,
        kwargs...,
    )

    return docp
end
