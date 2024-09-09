"""
The Bioreactor Problem:
    The problem is formulated as a JuMP modeln and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.bioreactor(::JuMPBackend;nh::Int=100, N::Int=30)
    # Parameters
    beta = 1
    c = 2
    gamma = 1
    halfperiod = 5
    Ks = 0.05
    mu2m = 0.1
    mubar = 1
    r = 0.005
    T = 10 * N
    pi = 3.141592653589793

    # Model
    model = JuMP.Model()

    @variables(model, begin
        y[0:nh] >= 0,           (start = 50)
        s[0:nh] >= 0,           (start = 50)
        b[0:nh] >= 0.001,       (start = 50)
        0 <= u[0:nh] <= 1    , (start = 0.5)
    end)

    # Boundary constraints
    @constraints(model, begin
        0.05 <= y[0] <= 0.25
        0.5 <= s[0] <= 5
        0.5 <= b[0] <= 3
    end)

    # Dynamics
    @expressions(model, begin
        step, T/nh
        # intermediate variables
        mu2[t=0:nh], mu2m * s[t] / (s[t] + Ks)
        days[t=0:nh], (t * step) / (halfperiod * 2)
        tau[t=0:nh], (days[t] - floor(days[t])) * 2 * pi
        light[t=0:nh], max(0, sin(tau[t]))^2
        mu[t=0:nh], light[t] * mubar
        # dynamics
        dy[t=0:nh], mu[t] * y[t] / (1 + y[t]) - (r + u[t]) * y[t]
        ds[t=0:nh], -mu2[t] * b[t] + u[t] * beta * (gamma * y[t] - s[t])
        db[t=0:nh], (mu2[t] - u[t] * beta) * b[t]
    end)    
    @constraints(model, begin
        con_y[t=1:nh], y[t] == y[t - 1] + 0.5 * step * (dy[t] + dy[t - 1])
        con_s[t=1:nh], s[t] == s[t - 1] + 0.5 * step * (ds[t] + ds[t - 1])
        con_b[t=1:nh], b[t] == b[t - 1] + 0.5 * step * (db[t] + db[t - 1])
    end)
    
    @objective(model, Max, sum(b[t] / (beta + c) for t in 0:nh))

    return model
end