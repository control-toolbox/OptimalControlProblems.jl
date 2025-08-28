"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Double Oscillator Optimal Control Problem**.  
The objective is to compute an optimal control trajectory for a mechanical double oscillator system, minimising a quadratic cost on positions and control.  
The system dynamics are discretised over `N` steps, with collocation constraints enforcing the dynamics.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `N::Int=500`: (Keyword) Number of discretisation steps in the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model representing the double oscillator optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.double_oscillator(JuMPBackend(); N=200)
```

# References

- [CLP2018] Coudurier, C., Lepreux, O., & Petit, N. (2018). *Optimal bang-bang control of a mechanical double oscillator using averaging methods*.  
  IFAC-PapersOnLine, 51(2), 49–54.
"""
function OptimalControlProblems.double_oscillator(
    ::JuMPBackend, args...; N::Int=steps_number_data(:double_oscillator), kwargs...
)

    # parameters
    m1 = 100    # [kg]
    m2 = 2      # [kg]
    c = 0.5     # [Ns/m]
    k1 = 100    # [N/m]
    k2 = 3      # [N/m]
    tf = final_time_data(:double_oscillator)

    # model
    model = JuMP.Model(args...; kwargs...)

    # state, control and initial guess
    @variables(
        model,
        begin
            x1[0:N], (start = 0.1)
            x2[0:N], (start = 0.1)
            x3[0:N], (start = 0.1)
            x4[0:N], (start = 0.1)
            -1 <= u[0:N] <= 1, (start = 0.1)
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
            step, tf / N
            t[k = 0:N], k * tf / N
            F[k = 0:N], sin(t[k] * 2π / tf)

            # dynamics
            dx1[k = 0:N], x3[k]
            dx2[k = 0:N], x4[k]
            dx3[k = 0:N], -(k1 + k2) / m1 * x1[k] + k2 / m1 * x2[k] + 1 / m1 * F[k]
            dx4[k = 0:N], k2 / m2 * x1[k] - k2 / m2 * x2[k] - c * (1 - u[k]) / m2 * x4[k]

            # objective
            dc[k = 0:N], 0.5 * (x1[k]^2 + x2[k]^2 + u[k]^2)
        end
    )

    @constraints(
        model,
        begin
            ∂x1[k = 1:N], x1[k] == x1[k - 1] + 0.5 * step * (dx1[k] + dx1[k - 1])
            ∂x2[k = 1:N], x2[k] == x2[k - 1] + 0.5 * step * (dx2[k] + dx2[k - 1])
            ∂x3[k = 1:N], x3[k] == x3[k - 1] + 0.5 * step * (dx3[k] + dx3[k - 1])
            ∂x4[k = 1:N], x4[k] == x4[k - 1] + 0.5 * step * (dx4[k] + dx4[k - 1])
        end
    )

    # objective: trapeze rule
    @objective(model, Min, 0.5 * step * sum(dc[k] + dc[k - 1] for k in 1:N))

    return model
end
