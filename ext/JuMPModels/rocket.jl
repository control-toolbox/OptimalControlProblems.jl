"""
Goddard Rocket Problem:
    We want to find the optimal trajectory of a Goddard rocket.
    The objective is to maximize the final altitude of the rocket.
    The problem is formulated as a JuMP model, and can be found [here](https://github.com/MadNLP/COPSBenchmark.jl/blob/main/src/rocket.jl)
"""
function OptimalControlProblems.rocket(::JuMPBackend; nh::Int=500)

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

    # model
    model = JuMP.Model()

    # state, control, variable (final time) and initial guess
    @variables(
        model,
        begin
            h[i = 0:nh] >= h0, (start = 1)
            v[i = 0:nh] >= v0, (start = i / nh * (1 - i / nh))
            mf <= m[i = 0:nh] <= m0, (start = (mf - m0) * (i / nh) + m0)
            0 <= T[i = 0:nh] <= Tmax, (start = Tmax / 2)
            0 <= tf, (start = 1)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            h_ic, h[0] == h0
            v_ic, v[0] == v0
            m_ic, m[0] == m0
            mfc, m[nh] == mf
        end
    )

    # dynamics
    @expressions(
        model,
        begin

            #
            step, tf / nh

            #
            D[i = 0:nh], Dc * v[i]^2 * exp(-hc * (h[i] - h0)) / h0
            g[i = 0:nh], g0 * (h0 / h[i])^2

            #
            dh[i = 0:nh], v[i]
            dv[i = 0:nh], (T[i] - D[i] - m[i] * g[i]) / m[i]
            dm[i = 0:nh], -T[i] / c
        end
    )

    @constraints(
        model,
        begin
            ∂h[i = 1:nh], h[i] == h[i - 1] + 0.5 * step * (dh[i] + dh[i - 1])
            ∂v[i = 1:nh], v[i] == v[i - 1] + 0.5 * step * (dv[i] + dv[i - 1])
            ∂m[i = 1:nh], m[i] == m[i - 1] + 0.5 * step * (dm[i] + dm[i - 1])
        end
    )

    # objective
    @objective(model, Min, -h[nh])

    return model
end
