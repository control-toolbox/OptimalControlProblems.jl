"""
The electric Vehicle Problem
    Implement optimal control of an electric vehicle.
    The problem is formulated as a JuMP model.
Ref: [PS2011] Nicolas Petit and Antonio Sciarretta. "Optimal drive of electric vehicles using an inversion-based trajectory generation approach." IFAC Proceedings Volumes 44, no. 1 (2011): 14519-14526.
"""
function OptimalControlProblems.electric_vehicle(::JuMPBackend; N::Int=500)

    # parameters
    D = 10
    tf = 1
    b1 = 1e0
    b2 = 1e0
    h0 = 0.1
    h1 = 1
    h2 = 1e-3
    α0, α1, α2, α3 = (3, 0.4, -1, 0.1)

    # model
    model = Model()

    # state, control and initial guess
    @variable(model, x[0:N], start = 0.1)
    @variable(model, v[0:N], start = 0.1)
    @variable(model, u[0:N], start = 0.1)

    # boundary constraints
    @constraints(
        model,
        begin
            x[0] == 0
            v[0] == 0
            x[N] == D
            v[N] == 0
        end
    )

    # dynamics
    @expressions(
        model,
        begin

            #
            step, tf / N
            road[k = 0:N], α0 + α1 * x[k] + α2 * x[k]^2 + α3 * x[k]^3

            # dynamics
            dx[k = 0:N], v[k]
            dv[k = 0:N], h1 * u[k] - h2 * v[k]^2 - h0 - road[k]

            # objective
            dc[k = 0:N], b1 * u[k] * v[k] + b2 * u[k]^2
        end
    )

    @constraints(
        model,
        begin
            ∂x[k = 1:N], x[k] == x[k - 1] + 0.5 * step * (dx[k - 1] + dx[k])
            ∂v[k = 1:N], v[k] == v[k - 1] + 0.5 * step * (dv[k - 1] + dv[k])
        end
    )

    # objective
    @objective(model, Min, 0.5 * step * sum(dc[k] + dc[k - 1] for k in 1:N))

    return model
end
