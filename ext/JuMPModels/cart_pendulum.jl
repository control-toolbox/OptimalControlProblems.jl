"""
The Cart-Pendulum Problem: 
    we want to find the optimal trajectory of a cart-pendulum system.
    The objective is to swing the pendulum from the downward position to the upright position in the shortest time possible.      
    The problem is formulated as a JuMP model, and can be found [here](https://arxiv.org/pdf/2303.16746).
"""
function OptimalControlProblems.cart_pendulum(::JuMPBackend; nh::Int64=250)
    ## parameters
    g = 9.81      # gravitation [m/s^2]
    L = 1.0       # pendulum length [m]
    m = 1.0       # pendulum mass [kg]
    I = m * L^2 / 12  # pendulum moment of inertia
    m_cart = 0.5  # cart mass [kg]
    max_f = 5.0
    max_x = 1.0
    max_v = 2.0

    ## define the problem
    model = JuMP.Model()

    @variables(
        model,
        begin
            0.01 <= tf, (start = 0.1)
            ddx, (start = 0.1)
            -max_x <= x[0:nh] <= max_x, (start = 0.1)
            -max_v <= dx[0:nh] <= max_v, (start = 0.1)
            theta[0:nh], (start = 0.1)
            omega[0:nh], (start = 0.1)
            -max_f <= Fex[0:nh] <= max_f, (start = 0.1)
        end
    )

    @constraints(
        model,
        begin
            x[0] == 0.0
            theta[0] == 0.0
            omega[0] == 0.0
            theta[nh] == pi
            omega[nh] == 0.0
        end
    )

    @expressions(
        model,
        begin
            step, tf / nh
            COG_1[i=0:nh], L / 2 * sin(theta[i]) + x[i] + x[i]
            COG_2[i=0:nh], - L / 2 * -cos(theta[i])
            alpha[i=0:nh], 1.0 / (I + 0.25 * m * L^2) * 0.5 * L * m * (-ddx * cos(theta[i]) - g * sin(theta[i]))
            ddCOG_1[i=0:nh], - L * sin(theta[i]) * omega[i] + L / 2 * cos(theta[i]) * alpha[i] + ddx
            ddCOG_2[i=0:nh], L * cos(theta[i]) * omega[i] + L / 2 * sin(theta[i]) * alpha[i]
            FXFY_1[i=0:nh], m * ddCOG_1[i]
            FXFY_2[i=0:nh], m * ddCOG_2[i] + m * g
            eq[i=0:nh], -FXFY_1[i] + Fex[i] - m_cart * ddx
            c[i=0:nh], eq[i] - m_cart * ddx
            ddx_[i=0:nh], -1.0 / m_cart * c[i]
            alpha_[i=0:nh], 1.0 / (I + 0.25 * m * L^2) * 0.5 * L * m * (-ddx_[i] * cos(theta[i]) - g * sin(theta[i]))
        end
    )

    # Dynamics
    @constraints(
        model,
        begin
            d_x[k=1:nh], x[k] == x[k - 1] + 0.5 * step * (dx[k] + dx[k - 1])
            d_dx[k=1:nh], dx[k] == dx[k - 1] + 0.5 * step * (ddx_[k] + ddx_[k - 1])
            d_theta[k=1:nh], theta[k] == theta[k - 1] + 0.5 * step * (omega[k] + omega[k - 1])
            d_omega[k=1:nh], omega[k] == omega[k - 1] + 0.5 * step * (alpha_[k] + alpha_[k - 1])
        end
    )

    @objective(model, Min, tf)

    return model
end
