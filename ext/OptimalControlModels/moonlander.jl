"""
The Moonlander Problem:
    We want to find the optimal trajectory for a moonlander to land on the moon.
    The objective is to minimize the time taken to land on the moon.
    The problem is formulated as an OptimalControl model.
"""
function OptimalControlProblems.moonlander(
    ::OptimalControlBackend; target::Array{Float64}=[5.0, 5.0], nh::Int=500
)
    # parameters
    if size(target) != (2,)
        error("The input target must be of length 2.")
    end
    m = 1
    g = 9.81
    I = 0.1
    D = 1
    max_thrust = 2g

    # dynamics
    function dynamics(x, u)
        p1, p2, dp1, dp2, θ, dθ = x
        F1, F2 = u

        F_r = [
            cos(θ) -sin(θ) p1
            sin(θ) cos(θ) p2
            0 0 1
        ]
        F_tot = (F_r * [0; F1 + F2; 0])[1:2]
        ddp1 = (1 / m) * F_tot[1]
        ddp2 = (1 / m) * F_tot[2] - g
        ddθ = (1 / I) * (D / 2) * (F2 - F1)

        return [dp1, dp2, ddp1, ddp2, dθ, ddθ]
    end

    # define the problem
    ocp = @def begin

        # state, control and final time variables, and time
        tf ∈ R, variable
        t ∈ [0, tf], time
        x = (p1, p2, dp1, dp2, θ, dθ) ∈ R⁶, state
        u = (F1, F2) ∈ R², control

        # final time constraint
        tf >= 0.1

        # control constraints
        0 ≤ F1(t) ≤ max_thrust, (F1_con)
        0 ≤ F2(t) ≤ max_thrust, (F2_con)

        # initial conditions
        p1(0) == 0, (p1_ic)
        p2(0) == 0, (p2_ic)
        dp1(0) == 0, (dp1_ic)
        dp2(0) == 0, (dp2_ic)
        θ(0) == 0, (θ_ic)
        dθ(0) == 0, (dθ_ic)

        # final conditions
        p1(tf) == target[1], (p1_fc)
        p2(tf) == target[2], (p2_fc)
        dp1(tf) == 0, (dp1_fc)
        dp2(tf) == 0, (dp2_fc)

        ## dynamics
        ẋ(t) == dynamics(x(t), u(t))

        ## objective
        tf → min
    end

    # Initial guess
    xinit = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # [p1, p2, dp1, dp2, θ, dθ]
    uinit = [5.0, 5.0]  # [F1, F2] 
    varinit = [1]  # [tf] 
    init = (state=xinit, control=uinit, variable=varinit)

    # NLPModel + DOCP
    docp = direct_transcription(ocp; init=init, grid_size=nh)
    nlp = model(docp)
    return docp, nlp
end
