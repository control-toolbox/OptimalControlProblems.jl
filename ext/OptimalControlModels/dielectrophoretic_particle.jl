"""
Dielectrophoretic particle problem:
    This problem consists of a dielectrophoretic particle system.
    The goal is to find the trajectory that minimize the time taken for the particle to travel between two points.
    The problem is formulated as an OptimalControl model.
Ref: [CPR2006] Chang, D. E., Petit, N., & Rouchon, P. (2006). Time-optimal control of a particle in a dielectrophoretic system. IEEE Transactions on Automatic Control, 51(7), 1100-1114.
"""
function OptimalControlProblems.dielectrophoretic_particle(::OptimalControlBackend; nh::Int=500)

    # parameters
    x0 = 1
    xf = 2
    α = -0.75
    c = 1

    ocp = @def begin

        tf ∈ R, variable
        t ∈ [0, tf], time
        q = (x, y) ∈ R², state
        u ∈ R, control

        x(0) == x0, (x0_con)
        y(0) == 0, (y0_con)
        x(tf) == xf, (xf_con)

        tf ≥ 0, (tf_con)
        -1 ≤ u(t) ≤ 1, (u_con)

        q̇(t) == dynamics(y(t), u(t))

        tf → min

    end

    function dynamics(y, u)
        return [y * u + α * u^2, -c * y + u]
    end

    # initial guess
    init = (state=[1, 1], control=0, variable=1)

    # DOCP and NLP
    docp = direct_transcription(ocp; init=init, grid_size=nh)
    nlp = model(docp)

    return docp, nlp
end
