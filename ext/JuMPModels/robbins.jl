"""
The Robbins Problem:
    The problem is formulated as a JuMP model and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.robbins(::JuMPBackend; nh::Int=500)

    # parameters
    alpha = 3
    beta = 0
    gamma = 0.5
    tf = 10
    step = tf / nh

    # model
    model = JuMP.Model()

    # state, control and initial guess
    @variables(
        model,
        begin
            0 <= x1[0:nh], (start = 0.1)
            x2[0:nh], (start = 0.1)
            x3[0:nh], (start = 0.1)
            u[0:nh], (start = 0.1)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            x1[0] == 1
            x2[0] == -2
            x3[0] == 0
            x1[nh] == 0
            x2[nh] == 0
            x3[nh] == 0
        end
    )

    # dynamics
    @constraints(
        model,
        begin

            # dynamics
            ∂x1[i=1:nh], x1[i] == x1[i - 1] + 0.5 * step * (x2[i] + x2[i - 1])
            ∂x2[i=1:nh], x2[i] == x2[i - 1] + 0.5 * step * (x3[i] + x3[i - 1])
            ∂x3[i=1:nh], x3[i] == x3[i - 1] + 0.5 * step * (u[i] + u[i - 1])

            # objective
            

        end
    )

    # objective
    @objective(
        model, Min, step * sum(alpha * x1[i] + beta * x1[i]^2 + gamma * u[i]^2 for i in 0:nh-1)
    )

    return model
end
