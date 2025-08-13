"""
The Robbins Problem:
    The problem is formulated as a JuMP model and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.robbins(::JuMPBackend; N::Int=500)

    # parameters
    α = 3
    β = 0
    γ = 0.5
    tf = 10

    #
    step = tf / N

    # model
    model = JuMP.Model()

    # state, control and initial guess
    @variables(
        model,
        begin
            0 <= x1[0:N], (start = 0.1)
            x2[0:N], (start = 0.1)
            x3[0:N], (start = 0.1)
            u[0:N], (start = 0.1)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            x1[0] == 1
            x2[0] == -2
            x3[0] == 0
            x1[N] == 0
            x2[N] == 0
            x3[N] == 0
        end
    )

    # dynamics
    @constraints(
        model,
        begin
            ∂x1[i = 1:N], x1[i] == x1[i - 1] + 0.5 * step * (x2[i] + x2[i - 1])
            ∂x2[i = 1:N], x2[i] == x2[i - 1] + 0.5 * step * (x3[i] + x3[i - 1])
            ∂x3[i = 1:N], x3[i] == x3[i - 1] + 0.5 * step * (u[i] + u[i - 1])
        end
    )

    # objective
    @expressions(
        model,
        begin
            dc[i = 0:N], (α * x1[i] + β * x1[i]^2 + γ * u[i]^2)
        end
    )

    @objective(model, Min, 0.5 * step * sum(dc[i] + dc[i - 1] for i in 1:N))

    return model
end
