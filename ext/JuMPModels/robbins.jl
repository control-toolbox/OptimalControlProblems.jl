"""
The Robbins Problem:
    The problem is formulated as a JuMP model and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.robbins(::JuMPBackend;nh::Int=100, N::Int=30)
    # constants
    alpha = 3
    beta = 0
    gamma = 0.5
    tf = 10
    step = tf/nh

    # Model
    model = JuMP.Model()

    @variables(model, begin
        0 <= x1[0:nh],           (start = 0.1)
        x2[0:nh],         (start = 0.1)
        x3[0:nh],         (start = 0.1)
        u[0:nh],          (start = 0.1)
    end)

    # Boundary constraints
    @constraints(model, begin
        x1[0] == 1
        x2[0] == -2
        x3[0] == 0
        x1[nh] == 0
        x2[nh] == 0
        x3[nh] == 0
    end)

    # Dynamics
    @constraints(model, begin
        con_x1[t=1:nh], x1[t] == x1[t - 1] + 0.5 * step * (x2[t] + x2[t - 1])
        con_x2[t=1:nh], x2[t] == x2[t - 1] + 0.5 * step * (x3[t] + x3[t - 1])
        con_dx3[t=1:nh], x3[t] == x3[t - 1] + 0.5 * step * (u[t] + u[t - 1])
    end)

    # Objective
    @objective(model, Min, sum(alpha * x1[t] + beta * x1[t]^2 + gamma * u[t]^2 for t in 0:nh))

    return model

end