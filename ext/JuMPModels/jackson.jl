"""
The Jackson Problem:
    The problem is formulated as a JuMP model and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.jackson(::JuMPBackend; nh::Int=100, N::Int=30)
    # constants
    k1 = 1
    k2 = 10
    k3 = 1
    tf = 4
    step = tf / nh

    # Model
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

    # Boundary constraints
    @constraints(
        model,
        begin
            a[0] == 1
            b[0] == 0
            x3[0] == 0
        end
    )

    # Dynamics
    @expressions(
        model,
        begin
            da[t=0:nh], -u[t] * (k1 * a[t] - k2 * b[t])
            db[t=0:nh], u[t] * (k1 * a[t] - k2 * b[t]) - (1 - u[t]) * k3 * b[t]
            dx3[t=0:nh], (1 - u[t]) * k3 * b[t]
        end
    )
    @constraints(
        model,
        begin
            con_da[t=1:nh], a[t] == a[t - 1] + 0.5 * step * (da[t] + da[t - 1])
            con_db[t=1:nh], b[t] == b[t - 1] + 0.5 * step * (db[t] + db[t - 1])
            con_dx3[t=1:nh], x3[t] == x3[t - 1] + 0.5 * step * (dx3[t] + dx3[t - 1])
        end
    )

    # Objective
    @objective(model, Max, x3[nh])

    return model
end
