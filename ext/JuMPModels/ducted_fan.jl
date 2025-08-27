"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Planar Ducted Fan Optimal Control Problem**.  
The objective is to determine the optimal control inputs for a planar ducted fan to move from a given initial state to a desired final state, minimising a combination of control effort and final time.  
The system is discretised over `N` steps, with collocation constraints enforcing the dynamics.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `N::Int=250`: (Keyword) Number of discretisation steps in the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model representing the planar ducted fan optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.ducted_fan(JuMPBackend(); N=100)
```

# References

- Graichen, K., & Petit, N. (2009). *Incorporating a class of constraints into the dynamics of optimal control problems*.  
  Optimal Control Applications and Methods, 30(6), 537–561. [GP2009]
"""
function OptimalControlProblems.ducted_fan(::JuMPBackend; N::Int=steps_number_data(:ducted_fan))

    # parameters
    r = 0.2         # [m]
    J = 0.05        # [kg.m2]
    m = 2.2         # [kg]
    mg = 4          # [N]
    μ = 1000

    # model
    model = Model()

    # state, control, variable (final time) and initial guess
    @variable(model, x₁[0:N], start = 0.1)
    @variable(model, v₁[0:N], start = 0.1)
    @variable(model, x₂[0:N], start = -0.1)
    @variable(model, v₂[0:N], start = 0.1)
    @variable(model, -deg2rad(30) <= α[0:N] <= deg2rad(30), start = 0.1) # radian
    @variable(model, vα[0:N], start = 0.1)
    @variable(model, -5 <= u₁[0:N] <= 5, start = 0.1) # [N]
    @variable(model, 0 <= u₂[0:N] <= 17, start = 1) # [N]
    @variable(model, 0.1 <= tf, start = 1)

    # Boundary constraints
    @constraints(
        model,
        begin

            # initial
            x₁[0] == 0
            v₁[0] == 0
            x₂[0] == 0
            v₂[0] == 0
            α[0] == 0
            vα[0] == 0

            # final
            x₁[N] == 1
            v₁[N] == 0
            x₂[N] == 0
            v₂[N] == 0
            α[N] == 0
            vα[N] == 0
        end
    )

    # dynamics
    @expressions(
        model,
        begin

            #
            step, tf / N

            # dynamics
            dx₁[k = 0:N], v₁[k]
            dv₁[k = 0:N], (u₁[k] * cos(α[k]) - u₂[k] * sin(α[k])) / m
            dx₂[k = 0:N], v₂[k]
            dv₂[k = 0:N], (-mg + u₁[k] * sin(α[k]) + u₂[k] * cos(α[k])) / m
            dα[k = 0:N], vα[k]
            dvα[k = 0:N], r * u₁[k] / J

            # objective
            dc[k = 0:N], 2 * u₁[k]^2 + u₂[k]^2
        end
    )

    @constraints(
        model,
        begin
            ∂x₁[k = 1:N], x₁[k] == x₁[k - 1] + 0.5 * step * (dx₁[k] + dx₁[k - 1])
            ∂v₁[k = 1:N], v₁[k] == v₁[k - 1] + 0.5 * step * (dv₁[k] + dv₁[k - 1])
            ∂x₂[k = 1:N], x₂[k] == x₂[k - 1] + 0.5 * step * (dx₂[k] + dx₂[k - 1])
            ∂v₂[k = 1:N], v₂[k] == v₂[k - 1] + 0.5 * step * (dv₂[k] + dv₂[k - 1])
            ∂α[k = 1:N], α[k] == α[k - 1] + 0.5 * step * (dα[k] + dα[k - 1])
            ∂vα[k = 1:N], vα[k] == vα[k - 1] + 0.5 * step * (dvα[k] + dvα[k - 1])
        end
    )

    # objective
    @objective(
        model, Min, (1 / tf) * 0.5 * step * sum(dc[k] + dc[k - 1] for k in 1:N) + (μ * tf)
    )

    return model
end
