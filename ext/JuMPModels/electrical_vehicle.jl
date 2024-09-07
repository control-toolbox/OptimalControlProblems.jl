"""
The Electrical Vehicle Problem
    Implement optimal control of an electrical vehicle.
    The problem is formulated as a JuMP model.
Ref: [PS2011] Nicolas Petit and Antonio Sciarretta. "Optimal drive of electric vehicles using an inversion-based trajectory generation approach." IFAC Proceedings Volumes 44, no. 1 (2011): 14519-14526.
"""
function OptimalControlProblems.electrical_vehicle(::JuMPBackend; nh::Int=100)
    D = 10.0
    T = 1.0
    b1 = 1e3
    b2 = 1e3
    h0 = 0.1
    h1 = 1.0
    h2 = 1e-3
    step = T / nh
    p0, p1, p2, p3 = (3.0, 0.4, -1.0, 0.1)
    road(x) = p0 + p1 * x + p2 * x^2 + p3 * x^3

    model = Model()

    @variable(model, x[0:nh])
    @variable(model, v[0:nh])
    @variable(model, u[0:nh])

    @objective(model, Min, sum(b1 * u[t] * v[t] + b2 * u[t]^2 for t in 0:nh))

    # Dynamics
    @expressions(
        model,
        begin
            dx[t=0:nh], v[t]
            dv[t=0:nh], h1 * u[t] - h2 * v[t]^2 - h0 - road(x[t])
        end
    )
    # Collocation
    @constraints(
        model,
        begin
            cond_x[t=1:nh], x[t] == x[t - 1] + 0.5 * step * (dx[t - 1] + dx[t])
            cond_v[t=1:nh], v[t] == v[t - 1] + 0.5 * step * (dv[t - 1] + dv[t])
        end
    )
    # Boundary constraints
    @constraints(
        model,
        begin
            x[0] == 0.0
            x[nh] == D
            v[0] == 0
            v[nh] == 0
        end
    )

    return model
end
