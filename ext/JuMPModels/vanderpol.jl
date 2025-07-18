"""
The Van der Pol Problem:
    The problem is formulated as a JuMP model and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.vanderpol(::JuMPBackend; nh::Int=100)
    # parameters
    omega = 1
    epsilon = 1
    tf = 2

    model = JuMP.Model()

    @variables(
        model,
        begin
            x1[0:nh], (start = 0.1)
            x2[0:nh], (start = 0.1)
            u[0:nh], (start = 0.1)
        end
    )

    # Boundary constraints
    @constraints(
        model,
        begin
            x1[0] == 1
            x2[0] == 0
        end
    )

    @expressions(
        model,
        begin
            step, tf / nh
            dx1[t=0:nh], x2[t]
            dx2[t=0:nh], epsilon * omega * (1 - x1[t]^2) * x2[t] - omega^2 * x1[t] + u[t]
        end
    )

    # Dynamics
    @constraints(
        model,
        begin
            con_x1[t=1:nh], x1[t] == x1[t - 1] + 0.5 * step * (dx1[t] + dx1[t - 1])
            con_x2[t=1:nh], x2[t] == x2[t - 1] + 0.5 * step * (dx2[t] + dx2[t - 1])
        end
    )

    @objective(model, Min, step * sum(0.5 * (x1[t]^2 + x2[t]^2 + u[t]^2) for t in 0:nh-1))

    return model
end
