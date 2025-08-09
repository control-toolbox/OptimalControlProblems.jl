"""
Hang Glider Problem:
    We want to find the optimal trajectory of a hang glider.
    The objective is to maximize the final horizontal position of the glider while in the presence of a thermal updraft.
    The problem is formulated as a JuMP model, and can be found [here](https://www.mcs.anl.gov/~more/cops/)

"""
function OptimalControlProblems.glider(::JuMPBackend; nh::Int=500)

    # parameters
    x_0 = 0
    y_0 = 1000
    y_f = 900
    vx_0 = 13.23
    vx_f = 13.23
    vy_0 = -1.288
    vy_f = -1.288
    u_c = 2.5
    r_0 = 100
    m = 100
    g = 9.81
    c0 = 0.034
    c1 = 0.069662
    S = 14
    rho = 1.13
    cL_min = 0
    cL_max = 1.4

    # model
    model = JuMP.Model()

    # state, control, variable (final time) and initial guess
    @variables(
        model,
        begin
            0 <= tf,                        (start = 1)
            0 <= x[k=0:nh],                 (start = x_0 + vx_0 * k / nh)
            y[k=0:nh],                      (start = y_0 + (k / nh) * (y_f - y_0))
            0 <= vx[k=0:nh],                (start = vx_0)
            vy[k=0:nh],                     (start = vy_0)
            cL_min <= cL[k=0:nh] <= cL_max, (start = cL_max / 2)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            x[0] == x_0
            y[0] == y_0
            vx[0] == vx_0
            vy[0] == vy_0
            y[nh] == y_f
            vx[nh] == vx_f
            vy[nh] == vy_f
        end
    )

    # dynamics
    @expressions(
        model,
        begin

            #
            step, tf / nh

            #
            r[k=0:nh], (x[k] / r_0 - 2.5)^2
            u[k=0:nh], u_c * (1 - r[k]) * exp(-r[k])
            w[k=0:nh], vy[k] - u[k]
            v[k=0:nh], √(vx[k]^2 + w[k]^2)
            D[k=0:nh], 0.5 * (c0 + c1 * cL[k]^2) * rho * S * v[k]^2
            L[k=0:nh], 0.5 * cL[k] * rho * S * v[k]^2

            #
            dvx[k=0:nh], -(L[k] *  w[k] + D[k] * vx[k]) / (m * v[k])
            dvy[k=0:nh],  (L[k] * vx[k] - D[k] *  w[k]) / (m * v[k]) - g
        end
    )

    @constraints(
        model,
        begin
            ∂x[k=1:nh],   x[k] ==  x[k - 1] + 0.5 * step * ( vx[k] +  vx[k - 1])
            ∂y[k=1:nh],   y[k] ==  y[k - 1] + 0.5 * step * ( vy[k] +  vy[k - 1])
            ∂vx[k=1:nh], vx[k] == vx[k - 1] + 0.5 * step * (dvx[k] + dvx[k - 1])
            ∂vy[k=1:nh], vy[k] == vy[k - 1] + 0.5 * step * (dvy[k] + dvy[k - 1])
        end
    )

    # objective
    @objective(model, Min, -x[nh])

    return model
end
