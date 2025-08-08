"""
The Bioreactor Problem:
    The problem is formulated as a JuMP model and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.bioreactor(::JuMPBackend; nh::Int=500, N::Int=30)

    # parameters
    beta = 1
    c = 2
    gamma = 1
    halfperiod = 5
    Ks = 0.05
    mu2m = 0.1
    mubar = 1
    r = 0.005
    T = 10N

    # model
    model = JuMP.Model()

    # variables and initial guess
    @variables(
        model,
        begin
            y[0:nh] >= 0, (start = 50)
            s[0:nh] >= 0, (start = 50)
            b[0:nh] >= 0.001, (start = 50)
            0 <= u[0:nh] <= 1, (start = 0.5)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            0.05 <= y[0] <= 0.25
            0.5 <= s[0] <= 5
            0.5 <= b[0] <= 3
        end
    )

    # dynamics
    @expressions(
        model,
        begin

            #
            step, T / nh

            # intermediate variables
            growth[k=0:nh], mu2m * s[k] / (s[k] + Ks)
            mu2[k=0:nh], growth[k]

            days[k=0:nh], (t * step) / (halfperiod * 2)
            tau[k=0:nh], (days[k] - floor(days[k])) * 2π
            light[k=0:nh], max(0, sin(tau[k]))^2
            mu[k=0:nh], light[k] * mubar

            # dynamics
            dy[k=0:nh], mu[k] * y[k] / (1 + y[k]) - (r + u[k]) * y[k]
            ds[k=0:nh], -mu2[k] * b[k] + u[k] * beta * (gamma * y[k] - s[k])
            db[k=0:nh], (mu2[k] - u[k] * beta) * b[k]

            # objective
            dc[k=0:nh], -mu2[k] * b[k] / (beta + c)

        end
    )

    @constraints(
        model,
        begin
            con_y[k=1:nh], y[k] == y[k - 1] + 0.5 * step * (dy[k] + dy[k - 1])
            con_s[k=1:nh], s[k] == s[k - 1] + 0.5 * step * (ds[k] + ds[k - 1])
            con_b[k=1:nh], b[k] == b[k - 1] + 0.5 * step * (db[k] + db[k - 1])
        end
    )

    # objective
    @objective(model, Min, 0.5 * step * sum(dc[k] + dc[k-1] for k in 1:nh))

    return model
end
