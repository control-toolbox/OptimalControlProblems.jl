"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Dielectrophoretic Particle Optimal Control Problem**.  
The goal is to determine the time–optimal trajectory of a particle in a dielectrophoretic system, moving it from an initial point to a target point.  
The system dynamics are discretised over `N` steps, with the final time optimised.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `N::Int=500`: (Keyword) Number of discretisation steps in the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model representing the dielectrophoretic particle optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.dielectrophoretic_particle(JuMPBackend(); N=200)
```

# References

- [CPR2006] Chang, D. E., Petit, N., & Rouchon, P. (2006). *Time-optimal control of a particle in a dielectrophoretic system*.  
  IEEE Transactions on Automatic Control, 51(7), 1100–1114.
"""
function OptimalControlProblems.dielectrophoretic_particle(::JuMPBackend; N::Int=500)

    # parameters
    x0 = 1
    xf = 2
    α = -0.75
    c = 1

    # model
    model = JuMP.Model()

    # state, control and variable (final time)
    @variable(model, x[0:N], start = 1)
    @variable(model, y[0:N], start = 1)
    @variable(model, -1 <= u[0:N] <= 1, start = 0.1)
    @variable(model, 0 <= tf, start = 5)

    # Objective
    @objective(model, Min, tf)

    # Dynamics
    @expressions(
        model,
        begin
            step, tf / N
            dx[k = 0:N], y[k] * u[k] + α * u[k]^2
            dy[k = 0:N], -c * y[k] + u[k]
        end
    )
    # Collocation
    @constraints(
        model,
        begin
            ∂x[k = 1:N], x[k] == x[k - 1] + 0.5 * step * (dx[k] + dx[k - 1])
            ∂y[k = 1:N], y[k] == y[k - 1] + 0.5 * step * (dy[k] + dy[k - 1])
        end
    )
    # Boundary constraints
    @constraints(
        model,
        begin
            x[0] == x0
            x[N] == xf
            y[0] == 0
        end
    )

    return model
end
