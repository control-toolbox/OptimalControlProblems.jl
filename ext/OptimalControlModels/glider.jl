"""
Hang Glider Problem:
    We want to find the optimal trajectory of a hang glider.
    The objective is to maximize the final horizontal position of the glider while in the presence of a thermal updraft.
    The problem is formulated as an OptimalControl model.
    Original formulation from MadNLP/COPSBenchmark
"""
function OptimalControlProblems.glider(::OptimalControlBackend; nh::Int=100)
    # parameters
    x_0 = 0.0
    y_0 = 1000.0
    y_f = 900.0
    vx_0 = 13.23
    vx_f = 13.23
    vy_0 = -1.288
    vy_f = -1.288
    u_c = 2.5
    r_0 = 100.0
    m = 100.0
    g = 9.81
    c0 = 0.034
    c1 = 0.069662
    S = 14.0
    rho = 1.13
    cL_min = 0.0
    cL_max = 1.4
    t0 = 0.0

    function dynamics(x, u)
        x1, y, vx, vy = x
        cL = u
        ## Helper functions
        r = (x1 / r_0 - 2.5)^2
        UpD = u_c * (1 - r) * exp(-r)
        w = vy - UpD
        v = sqrt(vx^2 + w^2)
        D = 0.5 * (c0 + c1 * (cL^2)) * rho * S * (v^2)
        L = 0.5 * cL * rho * S * (v^2)
        vx_dot = (-L * (w / v) - D * (vx / v)) / m
        vy_dot = ((L * (vx / v) - D * (w / v)) / m) - g
        return [vx, vy, vx_dot, vy_dot]
    end

    ocp = @def begin

        ## define the problem
        tf ∈ R, variable
        t ∈ [t0, tf], time
        x ∈ R⁴, state
        u ∈ R, control
        ## state variables
        y = x₂
        vx = x₃
        vy = x₄
        ## control variables
        cL = u₁

        ## constraints
        # state constraints
        x₁(t) ≥ 0.0, (x_con)
        vx(t) ≥ 0.0, (vx_con)
        # control constraints
        cL_min ≤ cL(t) ≤ cL_max, (cL_con)
        # initial conditions
        x₁(t0) == x_0, (x0_con)
        y(t0) == y_0, (y0_con)
        vx(t0) == vx_0, (vx0_con)
        vy(t0) == vy_0, (vy0_con)
        # final conditions
        tf ≥ 10.0
        y(tf) == y_f, (yf_con)
        vx(tf) == vx_f, (vxf_con)
        vy(tf) == vy_f, (vyf_con)

        ## dynamics
        ẋ(t) == dynamics(x(t), u(t))

        ## objective
        x₁(tf) → max
    end

    # Initial guess
    tf = (y_f - y_0) / vy_0
    xinit = t -> [x_0 + vx_0 * t, y_0 + t / tf * (y_f - y_0), vx_0, vy_0]
    uinit = cL_max / 2.0
    init = (state=xinit, control=uinit, variable=tf)

    # NLPModel + DOCP
    docp = direct_transcription(ocp; init=init, grid_size=nh)

    return docp
end
