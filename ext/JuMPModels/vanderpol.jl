"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Van der Pol Problem**, a classic nonlinear oscillator system.  
The model represents the dynamics of the Van der Pol oscillator with control input `u` and seeks to minimise the quadratic cost over the states and control.  

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `N::Int=500`: (Keyword) Number of discretisation steps for the time horizon.

# Returns

- `model::JuMP.Model`: A JuMP model representing the Van der Pol optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.vanderpol(JuMPBackend(); N=100)
```

# References

- Problem formulation available at: https://github.com/control-toolbox/bocop/tree/main/bocop
"""
function OptimalControlProblems.vanderpol(::JuMPBackend; N::Int=steps_number_data(:vanderpol))

    # parameters
    tf = final_time_data(:vanderpol)
    ω = 1
    ε = 1

    # model
    model = JuMP.Model()

    # state, control and initial guess
    @variables(
        model,
        begin
            x1[0:N], (start = 0.1)
            x2[0:N], (start = 0.1)
            u[0:N], (start = 0.1)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            x1[0] == 1
            x2[0] == 0
        end
    )

    # dynamics
    @expressions(
        model,
        begin

            #
            step, tf / N

            # dynamics
            dx1[i = 0:N], x2[i]
            dx2[i = 0:N], ε * ω * (1 - x1[i]^2) * x2[i] - ω^2 * x1[i] + u[i]

            # objective
            dc[i = 0:N], 0.5 * (x1[i]^2 + x2[i]^2 + u[i]^2)
        end
    )

    @constraints(
        model,
        begin
            ∂x1[i = 1:N], x1[i] == x1[i - 1] + 0.5 * step * (dx1[i] + dx1[i - 1])
            ∂x2[i = 1:N], x2[i] == x2[i - 1] + 0.5 * step * (dx2[i] + dx2[i - 1])
        end
    )

    # objective
    @objective(model, Min, 0.5 * step * sum(dc[i] + dc[i - 1] for i in 1:N))

    return model
end
