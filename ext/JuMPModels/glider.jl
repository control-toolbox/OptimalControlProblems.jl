"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Hang Glider Optimal Control Problem**.  
The objective is to compute the optimal trajectory of a hang glider that maximises the final horizontal position while accounting for aerodynamic forces and a thermal updraft.  
The system dynamics are discretised over `N` steps, and collocation constraints enforce the kinematic and dynamic equations of the glider.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `N::Int=500`: (Keyword) Number of discretisation steps in the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model representing the hang glider optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.glider(JuMPBackend(); N=100)
```

# References

- Hang Glider Problem formulation as in: https://www.mcs.anl.gov/~more/cops/
"""
function OptimalControlProblems.glider(::JuMPBackend; N::Int=500)

    # parameters
    x_0 = 0
    y_0 = 1000
    y_f = 900
    vx_0 = 13.23
    vx_f = 13.23
    vy_0 = -1.288
    vy_f = -1.288
    u_c = 2.5
    r_0 = 100
    m = 100
    g = 9.81
    c0 = 0.034
    c1 = 0.069662
    S = 14
    ρ = 1.13
    cL_min = 0
    cL_max = 1.4

    # model
    model = JuMP.Model()

    # state, control, variable (final time) and initial guess
    @variables(
        model,
        begin
            0 <= tf, (start = 1)
            0 <= x[k = 0:N], (start = x_0 + vx_0 * k / N)
            y[k = 0:N], (start = y_0 + (k / N) * (y_f - y_0))
            0 <= vx[k = 0:N], (start = vx_0)
            vy[k = 0:N], (start = vy_0)
            cL_min <= cL[k = 0:N] <= cL_max, (start = cL_max / 2)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            x[0] == x_0
            y[0] == y_0
            vx[0] == vx_0
            vy[0] == vy_0
            y[N] == y_f
            vx[N] == vx_f
            vy[N] == vy_f
        end
    )

    # dynamics
    @expressions(
        model,
        begin

            #
            step, tf / N

            #
            r[k = 0:N], (x[k] / r_0 - 2.5)^2
            u[k = 0:N], u_c * (1 - r[k]) * exp(-r[k])
            w[k = 0:N], vy[k] - u[k]
            v[k = 0:N], √(vx[k]^2 + w[k]^2)
            D[k = 0:N], 0.5 * (c0 + c1 * cL[k]^2) * ρ * S * v[k]^2
            L[k = 0:N], 0.5 * cL[k] * ρ * S * v[k]^2

            #
            dvx[k = 0:N], -(L[k] * w[k] + D[k] * vx[k]) / (m * v[k])
            dvy[k = 0:N], (L[k] * vx[k] - D[k] * w[k]) / (m * v[k]) - g
        end
    )

    @constraints(
        model,
        begin
            ∂x[k = 1:N], x[k] == x[k - 1] + 0.5 * step * (vx[k] + vx[k - 1])
            ∂y[k = 1:N], y[k] == y[k - 1] + 0.5 * step * (vy[k] + vy[k - 1])
            ∂vx[k = 1:N], vx[k] == vx[k - 1] + 0.5 * step * (dvx[k] + dvx[k - 1])
            ∂vy[k = 1:N], vy[k] == vy[k - 1] + 0.5 * step * (dvy[k] + dvy[k - 1])
        end
    )

    # objective
    @objective(model, Min, -x[N])

    return model
end
