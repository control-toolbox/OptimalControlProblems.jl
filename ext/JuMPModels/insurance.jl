"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Insurance Optimal Control Problem**.  
The model represents a simplified insurance management scenario where the objective is to optimise the utility function `U` over time, subject to capital accumulation dynamics and other constraints.  
The system is discretised using `N` steps, and collocation constraints enforce the dynamics of the states `I`, `m`, and `x3`.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `N::Int=500`: (Keyword) Number of discretisation steps in the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model representing the insurance optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.insurance(JuMPBackend(); N=100)
```

# References

- Problem formulation available at: https://github.com/control-toolbox/bocop/tree/main/bocop
"""
function OptimalControlProblems.insurance(::JuMPBackend; N::Int=steps_number_data(:insurance))

    # parameters
    γ = 0.2
    λ = 0.25
    h0 = 1.5
    w = 1
    s = 10
    k = 0
    σ = 0
    α = 4
    tf = 10

    # model
    model = JuMP.Model()

    # state, control and initial guess
    @variables(
        model,
        begin
            0 <= I[0:N] <= 1.5, (start = 0.1)
            0 <= m[0:N] <= 1.5, (start = 0.1)
            0 <= x3[0:N] <= 1.5, (start = 0.1)
            0 <= h[0:N] <= 25, (start = 0.1)
            0 <= R[0:N], (start = 0.1)
            0 <= H[0:N], (start = 0.1)
            0 <= U[0:N], (start = 0.1)
            0.001 <= dUdR[0:N], (start = 0.1)
            P >= 0, (start = 0.1)
        end
    )

    # boundary constraints  
    @constraints(
        model,
        begin
            I[0] == 0
            m[0] == 0.001
            x3[0] == 0
            P - x3[N] == 0
        end
    )

    @expressions(
        model,
        begin
            step, tf / N
            t[i = 0:N], i*step
            ε[i = 0:N], k * t[i] / (tf - t[i] + 1)
            fx[i = 0:N], λ * exp(-λ * t[i]) + exp(-λ * tf) / tf
            v[i = 0:N], m[i]^(α / 2) / (1 + m[i]^(α / 2))
            vprime[i = 0:N], α / 2 * m[i]^(α / 2 - 1) / (1 + m[i]^(α / 2))^2
        end
    )

    @constraints(
        model,
        begin
            cond1[i = 0:N], R[i] - (w - P + I[i] - m[i] - ε[i]) == 0
            cond2[i = 0:N], H[i] - (h0 - γ * step * i * (1 - v[i])) == 0
            cond3[i = 0:N], U[i] - (1 - exp(-s * R[i]) + H[i]) == 0
            cond4[i = 0:N], dUdR[i] - (s * exp(-s * R[i])) == 0
        end
    )

    # dynamics
    @expressions(
        model,
        begin

            # dynamics
            dI[i = 0:N], (1 - γ * t[i] * vprime[i] / dUdR[i]) * h[i]
            dm[i = 0:N], h[i]
            dx3[i = 0:N], (1 + σ) * I[i] * fx[i]

            # objective
            dc[i = 0:N], -U[i] * fx[i]
        end
    )

    @constraints(
        model,
        begin
            ∂I[i = 1:N], I[i] == I[i - 1] + 0.5 * step * (dI[i] + dI[i - 1])
            ∂m[i = 1:N], m[i] == m[i - 1] + 0.5 * step * (dm[i] + dm[i - 1])
            ∂x3[i = 1:N], x3[i] == x3[i - 1] + 0.5 * step * (dx3[i] + dx3[i - 1])
        end
    )

    # objective
    @objective(model, Min, 0.5 * step * sum(dc[i] + dc[i - 1] for i in 1:N))

    return model
end
