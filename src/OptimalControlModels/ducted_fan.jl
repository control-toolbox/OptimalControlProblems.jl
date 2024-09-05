"""
The Ducted Fan Problem:
    Implement the optimal control of a planar ducted fan.
    Instance taken from [GP2009].
    The problem is formulated as an OptimalControl model.
Ref: Graichen, K., & Petit, N. (2009). Incorporating a class of constraints into the dynamics of optimal control problems. Optimal Control Applications and Methods, 30(6), 537-561.
"""
function ducted_fan(;nh::Int=100)
    # parameters
    r = 0.2         # [m]
    J = 0.05        # [kg.m2]
    m = 2.2         # [kg]
    mg = 4.0        # [N]
    μ = 1000.0

    @def ocp begin
        ## parameters
        r = 0.2         # [m]
        J = 0.05        # [kg.m2]
        m = 2.2         # [kg]
        mg = 4.0        # [N]
        μ = 1000.0

        ## define the problem
        tf ∈ R, variable
        t ∈ [ 0.0, tf ], time
        x ∈ R⁶, state
        u ∈ R², control
        
        ## state variables
        v1 = x₂
        v2 = x₄
        α = x₅
        vα = x₆

        ## constraints
        # state constraints
        tf ≥ 0.0,                                       (tf_con)
        -deg2rad(30.0) ≤ α(t) ≤ deg2rad(30.0),          (α_con)
        # control constraints
        -5.0 ≤ u₁(t) ≤ 5.0,                             (u1_con)
        0.0 ≤ u₂(t) ≤ 17.0,                             (u2_con)
        # initial constraints
        x₁(0) == 0.0,                                   (x1_i)
        v1(0) == 0.0,                                   (v1_i)
        x₂(0) == 0.0,                                   (x2_i)
        v2(0) == 0.0,                                   (v2_i)
        α(0) == 0.0,                                    (α_i)
        vα(0) == 0.0,                                   (vα_i)
        # final constraints
        x₁(tf) == 1.0,                                  (x1_f)
        v1(tf) == 0.0,                                  (v1_f)
        x₂(tf) == 0.0,                                  (x2_f)
        v2(tf) == 0.0,                                  (v2_f)
        α(tf) == 0.0,                                   (α_f)
        vα(tf) == 0.0,                                  (vα_f)

        ## dynamics
        ẋ(t) == dynamics(x(t), u(t))

        ## objective
        (1/tf) * ∫(2*u₁(t)^2 + u₂(t)^2) + μ * tf → min
    end

    function dynamics(x,u)  
        x1, v1, x2, v2, α, vα = x
        u1, u2 = u

        dx1 = v1
        dv1 = (u1 * cos(α) - u2 * sin(α)) / m
        dx2 = v2
        dv2 = (- mg + u1 * sin(α) + u2 * cos(α)) / m
        dα = vα
        dvα = r * u1 / J

        return [dx1, dv1, dx2, dv2, dα, dvα]
    end

    # Initial guess
    init = ()

    # NLPModel
    nlp = direct_transcription(ocp ,init = init, grid_size = nh)[2]

    return nlp

end


