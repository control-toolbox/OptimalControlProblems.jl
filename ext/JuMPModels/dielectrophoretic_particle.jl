"""
Dielectrophoretic particle problem:
    This problem consists of a dielectrophoretic particle system.
    The goal is to find the trajectory that minimize the time taken for the particle to travel between two points.
    The problem is formulated as a JuMP model.
Ref: [CPR2006] Chang, D. E., Petit, N., & Rouchon, P. (2006). Time-optimal control of a particle in a dielectrophoretic system. IEEE Transactions on Automatic Control, 51(7), 1100-1114.
"""
function OptimalControlProblems.dielectrophoretic_particle(::JuMPBackend; nh::Int=500)

    # parameters
    x0 = 1
    xf = 2
    α = -0.75
    c = 1

    # model
    model = JuMP.Model()

    # state, control and variable (final time)
    @variable(model, x[0:nh], start = 1)
    @variable(model, y[0:nh], start = 1)
    @variable(model, -1 <= u[0:nh] <= 1, start = 0.1)
    @variable(model, 0 <= tf, start = 5)

    # Objective
    @objective(model, Min, tf)

    # Dynamics
    @expressions(
        model,
        begin
            step, tf / nh
            dx[k=0:nh], y[k] * u[k] + α * u[k]^2
            dy[k=0:nh], -c * y[k] + u[k]
        end
    )
    # Collocation
    @constraints(
        model,
        begin
            ∂x[k=1:nh], x[k] == x[k - 1] + 0.5 * step * (dx[k] + dx[k - 1])
            ∂y[k=1:nh], y[k] == y[k - 1] + 0.5 * step * (dy[k] + dy[k - 1])
        end
    )
    # Boundary constraints
    @constraints(
        model,
        begin
            x[0] == x0
            x[nh] == xf
            y[0] == 0
        end
    )

    return model
end
