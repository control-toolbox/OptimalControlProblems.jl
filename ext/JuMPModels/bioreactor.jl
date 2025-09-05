"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Bioreactor Optimal Control Problem**.  
The problem formulation follows the version provided in the [control-toolbox/bocop repository](https://github.com/control-toolbox/bocop/tree/main/bocop).

The model includes state variables for biomass (`y`), substrate (`s`), and bacteria (`b`),  
with control variable `u`, subject to nonlinear dynamics and constraints.  
The objective is to minimise a cost function derived from the system dynamics.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `N::Int=500`: (Keyword) Number of discretisation steps in the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model representing the bioreactor optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.bioreactor(JuMPBackend(); N=100)
```

# References

- [control-toolbox/bocop](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.bioreactor(
    ::JuMPBackend, args...; N::Int=steps_number_data(:bioreactor), kwargs...
)

    # parameters
    β = 1
    c = 2
    gamma = 1
    halfperiod = 5
    Ks = 0.05
    μ2m = 0.1
    μbar = 1
    r = 0.005
    T = final_time_data(:bioreactor)

    # model
    model = JuMP.Model(args...; kwargs...)

    # variables and initial guess
    @variables(
        model,
        begin
            y[0:N] >= 0, (start = 50)
            s[0:N] >= 0, (start = 50)
            b[0:N] >= 0.001, (start = 50)
            0 <= u[0:N] <= 1, (start = 0.5)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            0.05 <= y[0] <= 0.25
            0.5 <= s[0] <= 5
            0.5 <= b[0] <= 3
        end
    )

    # dynamics
    @expressions(
        model,
        begin

            #
            step, T / N

            # intermediate variables
            growth[k = 0:N], μ2m * s[k] / (s[k] + Ks)
            μ2[k = 0:N], growth[k]

            days[k = 0:N], (k * step) / (halfperiod * 2)
            tau[k = 0:N], (days[k] - floor(days[k])) * 2π
            light[k = 0:N], max(0, sin(tau[k]))^2
            μ[k = 0:N], light[k] * μbar

            # dynamics
            dy[k = 0:N], μ[k] * y[k] / (1 + y[k]) - (r + u[k]) * y[k]
            ds[k = 0:N], -μ2[k] * b[k] + u[k] * β * (gamma * y[k] - s[k])
            db[k = 0:N], (μ2[k] - u[k] * β) * b[k]

            # objective
            dc[k = 0:N], -μ2[k] * b[k] / (β + c)
        end
    )

    @constraints(
        model,
        begin
            ∂y[k = 1:N], y[k] == y[k - 1] + 0.5 * step * (dy[k] + dy[k - 1])
            ∂s[k = 1:N], s[k] == s[k - 1] + 0.5 * step * (ds[k] + ds[k - 1])
            ∂b[k = 1:N], b[k] == b[k - 1] + 0.5 * step * (db[k] + db[k - 1])
        end
    )

    # objective
    @objective(model, Min, 0.5 * step * sum(dc[k] + dc[k - 1] for k in 1:N))

    return model
end
