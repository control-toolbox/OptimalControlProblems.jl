"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Quadrotor Optimal Control Problem**.  
The model represents the 3D dynamics of a quadrotor with translational and rotational states, subject to thrust and tilt constraints.  
The objective is to minimise the final time (`tf`) to reach a specified target position while respecting path and actuation constraints.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `N::Int=50`: (Keyword) Number of discretisation steps for the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model representing the Quadrotor optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.quadrotor(JuMPBackend(); N=20)
```

# References

- Problem formulation available at: https://arxiv.org/pdf/2303.16746
"""
function OptimalControlProblems.quadrotor(
    ::JuMPBackend; N::Int=steps_number_data(:quadrotor)
)

    # parameters
    g = 9.81
    atmin = 0
    atmax = 9.18 * 5
    tiltmax = 1.1 / 2
    dtiltmax = 6 / 2
    p0 = [0, 0, 2.5]
    v0 = [0, 0, 0]
    u0 = [9.81, 0, 0, 0]
    pf = [0.01, 5, 2.5]
    vf = [0, 0, 0]

    # model
    model = JuMP.Model()

    # 
    @variables(
        model,
        begin

            # variable
            0.1 <= tf, (start = 1)

            # state
            p₁[0:N], (start = 0.1)
            p₂[0:N], (start = 0.1)
            p₃[0:N], (start = 0.1)
            v₁[0:N], (start = 0.1)
            v₂[0:N], (start = 0.1)
            v₃[0:N], (start = 0.1)
            -π / 2 <= ϕ[0:N] <= π / 2, (start = 0.1)
            -π / 2 <= θ[0:N] <= π / 2, (start = 0.1)

            # control
            atmin <= at[0:N] <= atmax, (start = 10)
            -dtiltmax <= dϕ[0:N] <= dtiltmax, (start = 0.1)
            -dtiltmax <= dθ[0:N] <= dtiltmax, (start = 0.1)
            ψ[0:N], (start = 0.1)
        end
    )

    # path constraints
    @constraints(
        model,
        begin
            cond_tiltmax[i = 0:N], cos(θ[i]) * cos(ϕ[i]) >= cos(tiltmax)
        end
    )

    # initial and final conditions
    @constraints(
        model,
        begin
            p₁_i, p₁[0] == p0[1]
            p₂_i, p₂[0] == p0[2]
            p₃_i, p₃[0] == p0[3]
            v₁_i, v₁[0] == v0[1]
            v₂_i, v₂[0] == v0[2]
            v₃_i, v₃[0] == v0[3]
            ϕ_i, ϕ[0] == u0[2]
            θ_i, θ[0] == u0[3]
            p₁_f, p₁[N] == pf[1]
            p₂_f, p₂[N] == pf[2]
            p₃_f, p₃[N] == pf[3]
            v₁_f, v₁[N] == vf[1]
            v₂_f, v₂[N] == vf[2]
            v₃_f, v₃[N] == vf[3]
        end
    )

    # dynamics
    @expressions(
        model,
        begin

            #
            step, tf / N

            # dynamics
            cr[i = 0:N], cos(ϕ[i])
            sr[i = 0:N], sin(ϕ[i])
            cp[i = 0:N], cos(θ[i])
            sp[i = 0:N], sin(θ[i])
            cy[i = 0:N], cos(ψ[i])
            sy[i = 0:N], sin(ψ[i])
            R[i = 0:N],
            [
                (cy[i] * cp[i]) (cy[i] * sp[i] * sr[i] - sy[i] * cr[i]) (cy[i] * sp[i] * cr[i] + sy[i] * sr[i])
                (sy[i] * cp[i]) (sy[i] * sp[i] * sr[i] + cy[i] * cr[i]) (sy[i] * sp[i] * cr[i] - cy[i] * sr[i])
                (-sp[i]) (cp[i] * sr[i]) (cp[i] * cr[i])
            ]
            at_[i = 0:N], R[i] * [0; 0; at[i]]
            g_, [0; 0; -g]
            a[i = 0:N], at_[i] + g_

            # objective
            dc[i = 0:N],
            1e-8 * (at[i]^2 + ϕ[i]^2 + θ[i]^2 + ψ[i]^2) + 1e2 * (ψ[i] - u0[3])^2
        end
    )

    @constraints(
        model,
        begin
            ∂p₁[i = 1:N], p₁[i] == p₁[i - 1] + 0.5 * step * (v₁[i] + v₁[i - 1])
            ∂p₂[i = 1:N], p₂[i] == p₂[i - 1] + 0.5 * step * (v₂[i] + v₂[i - 1])
            ∂p₃[i = 1:N], p₃[i] == p₃[i - 1] + 0.5 * step * (v₃[i] + v₃[i - 1])
            ∂v₁[i = 1:N], v₁[i] == v₁[i - 1] + 0.5 * step * (a[i][1] + a[i - 1][1])
            ∂v₂[i = 1:N], v₂[i] == v₂[i - 1] + 0.5 * step * (a[i][2] + a[i - 1][2])
            ∂v₃[i = 1:N], v₃[i] == v₃[i - 1] + 0.5 * step * (a[i][3] + a[i - 1][3])
            ∂ϕ[i = 1:N], ϕ[i] == ϕ[i - 1] + 0.5 * step * (dϕ[i] + dϕ[i - 1])
            ∂θ[i = 1:N], θ[i] == θ[i - 1] + 0.5 * step * (dθ[i] + dθ[i - 1])
        end
    )

    # objective
    @objective(model, Min, tf + 0.5 * step * sum(dc[i] + dc[i - 1] for i in 1:N))

    return model
end
