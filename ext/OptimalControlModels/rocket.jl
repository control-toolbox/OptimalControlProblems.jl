"""
Goddard Rocket Problem:
    We want to find the optimal trajectory of a Goddard rocket.
    The objective is to maximize the final altitude of the rocket.
    The problem is formulated as an OptimalControl model.
"""
function OptimalControlProblems.rocket(::OptimalControlBackend; nh::Int=100)
    # parameters
    h_0 = 1.0
    v_0 = 0.0
    m_0 = 1.0
    g_0 = 1.0
    T_c = 3.5
    h_c = 500.0
    v_c = 620.0
    m_c = 0.6
    c = 0.5 * sqrt(g_0 * h_0)
    m_f = m_c * m_0
    D_c = 0.5 * v_c * (m_0 / g_0)
    T_max = T_c * m_0 * g_0

    # Model
    ocp = @def begin
        ## parameters
        h_0 = 1.0
        v_0 = 0.0
        m_0 = 1.0
        g_0 = 1.0
        T_c = 3.5
        h_c = 500.0
        v_c = 620.0
        m_c = 0.6
        c = 0.5 * sqrt(g_0 * h_0)
        m_f = m_c * m_0
        D_c = 0.5 * v_c * (m_0 / g_0)
        T_max = T_c * m_0 * g_0

        ## define the Problem
        tf ∈ R, variable
        t ∈ [0.0, tf], time
        x ∈ R³, state
        u ∈ R¹, control

        ## constraints
        # state constraints
        x₁(t) ≥ h_0, (x1_con)
        x₂(t) ≥ v_0, (x2_con)
        m_f ≤ x₃(t) ≤ m_0, (x3_con)
        # control constraints
        0 ≤ u(t) ≤ T_max, (u_con)
        # time constraints
        tf ≥ 0.0, (tf_con)
        # initial conditions
        x₁(0.0) == h_0, (x1_ic)
        x₂(0.0) == v_0, (x2_ic)
        x₃(0.0) == m_0, (x3_ic)
        # final conditions
        x₃(tf) == m_f, (x3_fc)

        ## dynamics
        ẋ(t) == dynamics(x(t), u(t))

        ## objective
        x₁(tf) → max
    end

    # dynamics
    function dynamics(x, u)
        return [
            x[2],
            (
                u - (D_c * x[2]^2 * exp(-h_c * (x[1] - h_0)) / h_0) -
                x[3] * g_0 * (h_0 / x[1])^2
            ) / x[3],
            -u / c,
        ]
    end

    # Initial guess
    xinit = [[1.0, k * (1.0 - k), (m_f - m_0) * k + m_0] for k in 0:nh]
    time_vec = LinRange(0.0, 1.0, nh + 1)
    init = (time=time_vec, state=xinit, control=T_max / 2.0, variable=1)

    # NLPModel + DOCP
    docp, nlp = direct_transcription(ocp; init=init, grid_size=nh)

    return docp, nlp
end
