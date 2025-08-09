"""
The Cart-Pendulum Problem: 
    we want to find the optimal trajectory of a cart-pendulum system.
    The objective is to swing the pendulum from the downward position to the upright position in the shortest time possible.      
    The problem is formulated as a JuMP model, and can be found [here](https://arxiv.org/pdf/2303.16746).
"""
function OptimalControlProblems.cart_pendulum(::JuMPBackend; nh::Int=500)
    
    # parameters
    g = 9.81            # gravitation [m/s^2]
    L = 1               # pendulum length [m]
    m = 1               # pendulum mass [kg]
    I = m * L^2 / 12    # pendulum moment of inertia
    mcart = 0.5        # cart mass [kg]
    max_f = 5
    max_x = 1
    max_v = 2

    # model
    model = JuMP.Model()

    # variables and initial guess
    @variables(
        model,
        begin
            0.1 <= tf,                      (start = 1.0)
            ddx,                            (start = 0.1)
            -max_x <= x[0:nh] <= max_x,     (start = 0.1)
            -max_v <= v[0:nh] <= max_v,     (start = 0.1)
            θ[0:nh],                        (start = 0.1)
            ω[0:nh],                        (start = 0.1)
            -max_f <= Fex[0:nh] <= max_f,   (start = 0.1)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            x[0] == 0
            θ[0] == 0
            ω[0] == 0
            θ[nh] == π
            ω[nh] == 0
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            step, tf / nh

            α_ddx[i=0:nh], 1 / (I + 0.25 * m * L^2) * 0.5 * L * m * (-ddx * cos(θ[i]) - g * sin(θ[i]))
            
            ddCOG_1[i=0:nh], - L * sin(θ[i]) * ω[i] + L / 2 * cos(θ[i]) * α_ddx[i] + ddx
            ddCOG_2[i=0:nh],   L * cos(θ[i]) * ω[i] + L / 2 * sin(θ[i]) * α_ddx[i]

            FXFY_1[i=0:nh], m * ddCOG_1[i]
            #FXFY_2[i=0:nh], m * ddCOG_2[i] + m * g

            eq[i=0:nh], -FXFY_1[i] + Fex[i] - mcart * ddx

            J, mcart
            c[i=0:nh], eq[i] - J * ddx

            dv[i=0:nh], -1 / J * c[i]
            dω[i=0:nh], 1 / (I + 0.25 * m * L^2) * 0.5 * L * m * (-dv[i] * cos(θ[i]) - g * sin(θ[i]))

        end
    )

    @constraints(
        model,
        begin
            ∂x[k=1:nh], x[k] == x[k - 1] + 0.5 * step * ( v[k] +  v[k - 1])
            ∂v[k=1:nh], v[k] == v[k - 1] + 0.5 * step * (dv[k] + dv[k - 1])
            ∂θ[k=1:nh], θ[k] == θ[k - 1] + 0.5 * step * ( ω[k] +  ω[k - 1])
            ∂ω[k=1:nh], ω[k] == ω[k - 1] + 0.5 * step * (dω[k] + dω[k - 1])
        end
    )

    # objective
    @objective(model, Min, tf)

    return model
end
