"""
Goddard Rocket Problem:
    We want to find the optimal trajectory of a Goddard rocket.
    The objective is to maximize the final altitude of the rocket.
    The problem is formulated as an OptimalControl model.
"""
function OptimalControlProblems.rocket(::OptimalControlBackend; N::Int=500)

    # parameters
    h0 = 1
    v0 = 0
    m0 = 1
    g0 = 1
    Tc = 3.5
    hc = 500
    vc = 620
    mc = 0.6
    c = 0.5 * sqrt(g0 * h0)
    mf = mc * m0
    Dc = 0.5 * vc * (m0 / g0)
    Tmax = Tc * m0 * g0

    # Model
    ocp = @def begin
        tf ∈ R, variable
        t ∈ [0, tf], time
        x = (h, v, m) ∈ R³, state
        T ∈ R, control

        # state constraints
        h(t) ≥ h0, (x1_con)
        v(t) ≥ v0, (x2_con)
        mf ≤ m(t) ≤ m0, (x3_con)

        # control constraints
        0 ≤ T(t) ≤ Tmax, (Tcon)

        # time constraints
        tf ≥ 0, (tf_con)

        # initial conditions
        h(0) == h0, (x1_ic)
        v(0) == v0, (x2_ic)
        m(0) == m0, (x3_ic)

        # final conditions
        m(tf) == mf, (x3_fc)

        # dynamics
        ẋ(t) == dynamics(h(t), v(t), m(t), T(t))

        # objective
        -h(tf) → min
    end

    # dynamics
    function dynamics(h, v, m, T)
        D = (Dc * v^2 * exp(-hc * (h - h0)) / h0)
        g = g0 * (h0 / h)^2
        return [v, (T - D - m * g) / m, -T / c]
    end

    # initial guess
    xinit = [[1, i / N * (1 - i / N), (mf - m0) * (i / N) + m0] for i in 0:N]
    time_vec = LinRange(0, 1, N+1)
    init = (time=time_vec, state=xinit, control=Tmax/2, variable=1)

    # DOCP and NLP
    docp = direct_transcription(ocp; init=init, grid_size=N, disc_method=:trapeze)
    nlp = model(docp)

    return docp, nlp
end
