"""
The Moonlander Problem:
    We want to find the optimal trajectory for a moonlander to land on the moon.
    The objective is to minimize the time taken to land on the moon.
    The problem is formulated as an OptimalControl model.
"""
function OptimalControlProblems.moonlander(
    ::OptimalControlBackend; p_f::Array{Float64}=[5.0, 5.0], nh::Int=100
)
    ## parameters
    m = 1.0
    g = 9.81
    I = 0.1
    D = 1.0
    max_thrust = 2 * g

    # dynamics
    function dynamics(x, u)
        p1, p2, dp1, dp2, theta, dtheta = x
        F1, F2 = u

        F_r = [
            cos(theta) -sin(theta) p1
            sin(theta) cos(theta) p2
            0.0 0.0 1.0
        ]
        F_tot = (F_r * [0; F1 + F2; 0])[1:2]
        ddp1 = (1 / m) * F_tot[1]
        ddp2 = (1 / m) * F_tot[2] - g
        ddtheta = (1 / I) * (D / 2) * (F2 - F1)

        return [dp1, dp2, ddp1, ddp2, dtheta, ddtheta]
    end

    ocp = @def begin

        ## define the problem
        tf ∈ R, variable
        t ∈ [0.0, tf], time
        x ∈ R⁶, state
        u ∈ R², control

        ## state variables
        p1 = x₁
        p2 = x₂
        dp1 = x₃
        dp2 = x₄
        theta = x₅
        dtheta = x₆

        ## control variables
        F1 = u₁
        F2 = u₂

        ## constraints
        # control constraints
        0.0 ≤ F1(t) ≤ max_thrust, (F1_con)
        0.0 ≤ F2(t) ≤ max_thrust, (F2_con)
        # initial conditions
        p1(0.0) == 0.0, (p1_ic)
        p2(0.0) == 0.0, (p2_ic)
        dp1(0.0) == 0.0, (dp1_ic)
        dp2(0.0) == 0.0, (dp2_ic)
        theta(0.0) == 0.0, (theta_ic)
        dtheta(0.0) == 0.0, (dtheta_ic)
        # final conditions
        tf >= 0.1
        p1(tf) == p_f[1], (p1_fc)
        p2(tf) == p_f[2], (p2_fc)
        dp1(tf) == 0.0, (dp1_fc)
        dp2(tf) == 0.0, (dp2_fc)

        ## dynamics
        ẋ(t) == dynamics(x(t), u(t))

        ## objective
        tf → min
    end

    # Initial guess
    xinit = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # [p1, p2, dp1, dp2, theta, dtheta]
    uinit = [5.0, 5.0]  # [F1, F2] 
    varinit = [0.1]  # [tf] 
    init = (state=xinit, control=uinit, variable=varinit)

    # NLPModel + DOCP
    docp = direct_transcription(ocp; init=init, grid_size=nh)
    nlp = model(docp)
    return docp, nlp
end
