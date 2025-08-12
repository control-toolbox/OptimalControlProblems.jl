"""
The Beam Problem:
    The problem is formulated as a JuMP model and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.beam(::JuMPBackend; nh::Int=500)

    # parameters
    tf = 1
    step = tf / nh

    # model
    model = JuMP.Model()

    # variables and initial guess
    @variables(
        model,
        begin
            0.0 <= x1[0:nh] <= 0.1, (start = 0.05)
            x2[0:nh], (start = 0.1)
            -10.0 <= u[0:nh] <= 10.0, (start = 0.1)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            x1[0] == 0
            x2[0] == 1
            x1[nh] == 0
            x2[nh] == -1
        end
    )

    # dynamics
    @constraints(
        model,
        begin
            ∂x1[t = 1:nh], x1[t] == x1[t - 1] + 0.5 * step * (x2[t] + x2[t - 1])
            ∂x2[t = 1:nh], x2[t] == x2[t - 1] + 0.5 * step * (u[t] + u[t - 1])
        end
    )

    # objective
    @objective(model, Min, 0.5 * step * sum(u[t]^2 + u[t - 1]^2 for t in 1:nh))

    return model
end
