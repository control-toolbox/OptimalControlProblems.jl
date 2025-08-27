"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Particle Steering Problem**.  
The model represents the dynamics of a particle with four state variables (`x1`, `x2`, `x3`, `x4`) and a control input `u`.  
The objective is to minimise the final time required for the particle to reach a specified altitude and terminal velocity while satisfying the system dynamics and boundary conditions.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `N::Int=500`: (Keyword) Number of discretisation steps for the time horizon.

# Returns

- `model::JuMP.Model`: A JuMP model representing the particle steering optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.steering(JuMPBackend(); N=200)
```

# References

- Problem formulation available at: https://github.com/MadNLP/COPSBenchmark.jl/blob/main/src/steering.jl
"""
function OptimalControlProblems.steering(::JuMPBackend; N::Int=steps_number_data(:steering))

    # parameters
    a = 100
    u_min = -π/2
    u_max = π/2
    xs = zeros(4)
    xf = [NaN, 5, 45, 0]

    tf_start = 1

    function gen_x0(k, i)
        if i == 1 || i == 4
            return 0.0
        elseif i == 2
            return 5.0 * k * tf_start / N
        elseif i == 3
            return 45.0 * k * tf_start / N
        end
    end

    # model
    model = JuMP.Model()

    @variable(model, u_min <= u[i = 1:(N + 1)] <= u_max, start = 0)   # control
    @variable(model, x1[i = 1:(N + 1)], start = gen_x0(i, 1))           # state x1
    @variable(model, x2[i = 1:(N + 1)], start = gen_x0(i, 2))           # state x2
    @variable(model, x3[i = 1:(N + 1)], start = gen_x0(i, 3))           # state x3
    @variable(model, x4[i = 1:(N + 1)], start = gen_x0(i, 4))           # state x4
    @variable(model, tf, start = tf_start)                             # final time

    @expression(model, Δt, tf / N) # step size

    # boundary conditions
    @constraint(model, x1[1] == xs[1])
    @constraint(model, x2[1] == xs[2])
    @constraint(model, x3[1] == xs[3])
    @constraint(model, x4[1] == xs[4])
    @constraint(model, x2[N + 1] == xf[2])
    @constraint(model, x3[N + 1] == xf[3])
    @constraint(model, x4[N + 1] == xf[4])

    # constraint on final time
    @constraint(model, tf >= 0)

    # dynamics
    @constraints(
        model,
        begin
            ∂x1[i = 1:N], x1[i + 1] == x1[i] + 0.5 * Δt * (x3[i] + x3[i + 1])
            ∂x2[i = 1:N], x2[i + 1] == x2[i] + 0.5 * Δt * (x4[i] + x4[i + 1])
            ∂x3[i = 1:N],
            x3[i + 1] == x3[i] + 0.5 * Δt * (a * cos(u[i]) + a * cos(u[i + 1]))
            ∂x4[i = 1:N],
            x4[i + 1] == x4[i] + 0.5 * Δt * (a * sin(u[i]) + a * sin(u[i + 1]))
        end
    )

    # objective
    @objective(model, Min, tf)

    return model
end
