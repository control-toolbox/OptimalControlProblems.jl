"""
The Insurance Problem:
    The problem is formulated as a JuMP model and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.insurance(::JuMPBackend; nh::Int=100, N::Int=30)
    # constants
    gamma = 0.2
    lambda = 0.25
    h0 = 1.5
    w = 1
    s = 10
    k = 0
    sigma = 0
    alpha = 4
    tf = 10

    # Model
    model = JuMP.Model()

    @variables(
        model,
        begin
            0 <= I[0:nh] <= 1.5, (start = 0.1)
            0 <= m[0:nh] <= 1.5, (start = 0.1)
            0 <= x3[0:nh] <= 1.5, (start = 0.1)
            0 <= h[0:nh] <= 25, (start = 0.1)
            0 <= R[0:nh], (start = 0.1)
            0 <= H[0:nh], (start = 0.1)
            0 <= U[0:nh], (start = 0.1)
            0.001 <= dUdR[0:nh], (start = 0.1)
            P >= 0, (start = 0.1)
        end
    )

    # Boundary constraints  
    @constraints(
        model,
        begin
            I[0] == 0
            m[0] == 0.001
            x3[0] == 0
            P - x3[nh] == 0
        end
    )

    @expressions(
        model,
        begin
            step, tf / nh
            epsilon[t=0:nh], k * t * step / (tf - t * step + 1)
            fx[t=0:nh], lambda * exp(-lambda * t * step) + exp(-lambda * tf) / tf
            v[t=0:nh], m[t]^(alpha / 2) / (1 + m[t]^(alpha / 2))
            vprime[t=0:nh], alpha / 2 * m[t]^(alpha / 2 - 1) / (1 + m[t]^(alpha / 2))^2
        end
    )
    @constraints(
        model,
        begin
            cond1[t=0:nh], R[t] - (w - P + I[t] - m[t] - epsilon[t]) == 0
            cond2[t=0:nh], H[t] - (h0 - gamma * step * t * (1 - v[t])) == 0
            cond3[t=0:nh], U[t] - (1 - exp(-s * R[t]) + H[t]) == 0
            cond4[t=0:nh], dUdR[t] - (s * exp(-s * R[t])) == 0
        end
    )

    # Dynamics
    @expressions(
        model,
        begin
            dI[t=0:nh], (1 - gamma * t * step * vprime[t] / dUdR[t]) * h[t]
            dm[t=0:nh], h[t]
            dx3[t=0:nh], (1 + sigma) * I[t] * fx[t]
        end
    )
    @constraints(
        model,
        begin
            con_dI[t=1:nh], I[t] == I[t - 1] + 0.5 * step * (dI[t] + dI[t - 1])
            con_dm[t=1:nh], m[t] == m[t - 1] + 0.5 * step * (dm[t] + dm[t - 1])
            con_dx3[t=1:nh], x3[t] == x3[t - 1] + 0.5 * step * (dx3[t] + dx3[t - 1])
        end
    )

    # Objective
    @objective(model, Max, sum(U[t] * fx[t] for t in 0:nh))

    return model
end
