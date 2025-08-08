"""
Double Oscillator Problem:
    Implement the optimal control of a double oscillator toy model.
    The problem is formulated as a JuMP model.
Ref: [CLP2018] Coudurier, C., Lepreux, O., & Petit, N. (2018). Optimal bang-bang control of a mechanical double oscillator using averaging methods. IFAC-PapersOnLine, 51(2), 49-54.
"""
function OptimalControlProblems.double_oscillator(::JuMPBackend; nh::Int=500)
    
    # parameters
    m1 = 100    # [kg]
    m2 = 2      # [kg]
    c = 0.5     # [Ns/m]
    k1 = 100    # [N/m]
    k2 = 3      # [N/m]
    tf = 2π

    # model
    model = Model()

    # state, control and initial guess
    @variables(
        model,
        begin
            x1[0:nh], (start = 0.1)
            x2[0:nh], (start = 0.1)
            x3[0:nh], (start = 0.1)
            x4[0:nh], (start = 0.1)
            -1 <= u[0:nh] <= 1, (start = 0.1)
        end
    )

    # boundary conditions
    @constraints(
        model,
        begin
            x1[0] == 0
            x2[0] == 0
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            
            #
            step, tf / nh
            t[k=0:nh], k * tf / nh
            F[k=0:nh], sin(t[k] * 2π / tf)
            
            # dynamics
            dx1[k=0:nh], x3[k]
            dx2[k=0:nh], x4[k]
            dx3[k=0:nh], -(k1 + k2) / m1 * x1[k] + k2 / m1 * x2[k] + 1 / m1 * F[k]
            dx4[k=0:nh], k2 / m2 * x1[k] - k2 / m2 * x2[k] - c * (1 - u[k]) / m2 * x4[k]

            # objective
            dc[k=0:nh], 0.5 * (x1[k]^2 + x2[k]^2 + u[k]^2)

        end
    )

    @constraints(
        model,
        begin
            ∂x1[k=1:nh], x1[k] == x1[k - 1] + 0.5 * step * (dx1[k] + dx1[k - 1])
            ∂x2[k=1:nh], x2[k] == x2[k - 1] + 0.5 * step * (dx2[k] + dx2[k - 1])
            ∂x3[k=1:nh], x3[k] == x3[k - 1] + 0.5 * step * (dx3[k] + dx3[k - 1])
            ∂x4[k=1:nh], x4[k] == x4[k - 1] + 0.5 * step * (dx4[k] + dx4[k - 1])
        end
    )

    # objective: trapeze rule
    @objective(model, Min, 0.5 * step * sum(dc[k] + dc[k-1] for k in 1:nh))

    return model
end
