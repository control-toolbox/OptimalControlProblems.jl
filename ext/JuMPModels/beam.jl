"""
$(TYPEDSIGNATURES)

Constructs a JuMP model representing the **Beam optimal control problem**.  
The objective is to minimise the control effort while satisfying boundary conditions and the linear beam dynamics.  
The problem is formulated as in the BOCOP repository.

# Arguments

- `::JuMPBackend`: Placeholder type to specify the JuMP backend or solver interface.
- `N::Int=500`: (Keyword) Number of discretisation steps for the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model containing the decision variables, dynamics constraints, boundary conditions, and the quadratic objective function.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.beam(JuMPBackend(); N=100)
```

# References

- Problem formulation available at: https://github.com/control-toolbox/bocop/tree/main/bocop
"""
function OptimalControlProblems.beam(::JuMPBackend; N::Int=500)

    # parameters
    tf = 1
    step = tf / N # t0 = 0

    # model
    model = JuMP.Model()

    # variables and initial guess
    @variables(
        model,
        begin
            0.0 <= x1[0:N] <= 0.1, (start = 0.05)
            x2[0:N], (start = 0.1)
            -10 <= u[0:N] <= 5, (start = 0.1)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            x1[0] == 0
            x2[0] == 1
            x1[N] == 0
            x2[N] == -1
        end
    )

    # dynamics
    @constraints(
        model,
        begin
            ∂x1[i = 1:N], x1[i] == x1[i - 1] + 0.5 * step * (x2[i] + x2[i - 1])
            ∂x2[i = 1:N], x2[i] == x2[i - 1] + 0.5 * step * (u[i] + u[i - 1])
        end
    )

    # objective
    @objective(model, Min, 0.5 * step * sum(u[i]^2 + u[i - 1]^2 for i in 1:N))

    return model
end
