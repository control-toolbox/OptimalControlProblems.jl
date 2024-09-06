"""
The Van der Pol Problem:
    The problem is formulated as an OptimalControl modeln and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function vanderpol(; nh::Int = 100)
    # parameters
    omega = 1
    epsilon = 1
    tf = 2

    model = JuMP.Model()

    @variables(model, begin
        x1[0:nh]
        x2[0:nh]
        u[0:nh]
        step[0:nh] == tf / nh
    end)

    # Boundary constraints
    @constraints(model, begin
        x1[0] == 1
        x2[0] == 0
    end)

    @expressions(
        model,
        begin
            dx1[t = 0:nh], x2[t]
            dx2[t = 0:nh], epsilon * omega * (1 - x1[t]^2) * x2[t] - omega^2 * x1[t] + u[t]
        end
    )

    # Dynamics
    @constraints(
        model,
        begin
            con_x1[t = 1:nh], x1[t] == x1[t - 1] + 0.5 * step[t] * (dx1[t] + dx1[t - 1])
            con_x2[t = 1:nh], x2[t] == x2[t - 1] + 0.5 * step[t] * (dx2[t] + dx2[t - 1])
        end
    )

    @objective(model, Min, sum(0.5 * (x1[t]^2 + x2[t]^2 + u[t]^2) for t = 0:nh))

    return model
end
