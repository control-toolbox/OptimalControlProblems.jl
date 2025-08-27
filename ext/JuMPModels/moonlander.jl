"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Moonlander Optimal Control Problem**.  
The model represents the dynamics of a moonlander with two translational and one rotational degrees of freedom.  
The objective is to minimise the final landing time (`tf`) while ensuring the lander reaches the target position with zero velocity.  
The dynamics include translational acceleration, rotation, and thrust allocation, discretised with `N` steps using trapezoidal collocation.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `N::Int=500`: (Keyword) Number of discretisation steps in the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model representing the Moonlander optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.moonlander(JuMPBackend(); N=100)
```

# References

- Problem formulation available at: https://arxiv.org/pdf/2303.16746
"""
function OptimalControlProblems.moonlander(::JuMPBackend; N::Int=steps_number_data(:moonlander))

    # parameters
    target=[5.0, 5.0]
    m = 1
    g = 9.81
    I = 0.1
    D = 1
    max_thrust = 2g

    # define the problem
    model = JuMP.Model()

    # state, control and final time variables
    @variables(
        model,
        begin
            # final time
            0.1 <= tf <= 1.0, (start = 0.5)

            # state variables
            p1[k = 0:N], (start = 0.1)
            p2[k = 0:N], (start = 0.1)
            dp1[k = 0:N], (start = 0.1)
            dp2[k = 0:N], (start = 0.1)
            θ[k = 0:N], (start = 0.1)
            dθ[k = 0:N], (start = 0.1)

            # control variables
            0 <= F1[k = 0:N] <= max_thrust, (start = 5.0)
            0 <= F2[k = 0:N] <= max_thrust, (start = 5.0)
        end
    )

    # initial and final conditions
    @constraints(
        model,
        begin
            p1[0] == 0
            p2[0] == 0
            dp1[0] == 0
            dp2[0] == 0
            θ[0] == 0
            dθ[0] == 0
            p1[N] == target[1]
            p2[N] == target[2]
            dp1[N] == 0
            dp2[N] == 0
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            F_r[k = 0:N], [
                cos(θ[k]) -sin(θ[k]) p1[k]
                sin(θ[k]) cos(θ[k]) p2[k]
                0 0 1
            ]
        end
    )
    @expressions(
        model,
        begin
            F_tot[k = 0:N], (F_r[k] * [0; F1[k] + F2[k]; 0])[1:2]
        end
    )
    @expressions(
        model,
        begin

            #
            step, tf / N

            #
            ddp1[k = 0:N], (1 / m) * F_tot[k][1]
            ddp2[k = 0:N], (1 / m) * F_tot[k][2] - g
            ddθ[k = 0:N], (1 / I) * (D / 2) * (F2[k] - F1[k])
        end
    )

    @constraints(
        model,
        begin
            ∂p1[k = 1:N], p1[k] == p1[k - 1] + 0.5 * step * (dp1[k] + dp1[k - 1])
            ∂p2[k = 1:N], p2[k] == p2[k - 1] + 0.5 * step * (dp2[k] + dp2[k - 1])
            ∂dp1[k = 1:N], dp1[k] == dp1[k - 1] + 0.5 * step * (ddp1[k] + ddp1[k - 1])
            ∂dp2[k = 1:N], dp2[k] == dp2[k - 1] + 0.5 * step * (ddp2[k] + ddp2[k - 1])
            ∂θ[k = 1:N], θ[k] == θ[k - 1] + 0.5 * step * (dθ[k] + dθ[k - 1])
            ∂dθ[k = 1:N], dθ[k] == dθ[k - 1] + 0.5 * step * (ddθ[k] + ddθ[k - 1])
        end
    )

    @objective(model, Min, tf)

    return model
end
