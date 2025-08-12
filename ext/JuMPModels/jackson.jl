"""
The Jackson Problem:
    The problem is formulated as a JuMP model and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.jackson(::JuMPBackend; nh::Int=500)

    # parameters
    k1 = 1
    k2 = 10
    k3 = 1
    tf = 4

    # model
    model = JuMP.Model()

    @variables(
        model,
        begin
            0 <= a[0:nh] <= 1.1, (start = 0.1)
            0 <= b[0:nh] <= 1.1, (start = 0.1)
            0 <= x3[0:nh] <= 1.1, (start = 0.1)
            0 <= u[0:nh] <= 1, (start = 0.1)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            a[0] == 1
            b[0] == 0
            x3[0] == 0
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            step, tf / nh
            da[i = 0:nh], -u[i] * (k1 * a[i] - k2 * b[i])
            db[i = 0:nh], u[i] * (k1 * a[i] - k2 * b[i]) - (1 - u[i]) * k3 * b[i]
            dx3[i = 0:nh], (1 - u[i]) * k3 * b[i]
        end
    )

    @constraints(
        model,
        begin
            ∂a[i = 1:nh], a[i] == a[i - 1] + 0.5 * step * (da[i] + da[i - 1])
            ∂b[i = 1:nh], b[i] == b[i - 1] + 0.5 * step * (db[i] + db[i - 1])
            ∂x3[i = 1:nh], x3[i] == x3[i - 1] + 0.5 * step * (dx3[i] + dx3[i - 1])
        end
    )

    # objective
    @objective(model, Min, -x3[nh])

    return model
end
