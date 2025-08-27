"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Truck Trailer Problem**.  
The model represents the dynamics of a truck with two trailers, including the truck's velocity, steering angles, and articulation angles between the truck and trailers.  
The objective is to minimise the total time taken to park the truck and trailers aligned vertically at a specified target location while maintaining the physical constraints and vehicle kinematics.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `N::Int=200`: (Keyword) Number of discretisation steps for the time horizon.

# Returns

- `model::JuMP.Model`: A JuMP model representing the truck trailer optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.truck_trailer(JuMPBackend(); N=100)
```

# References

- Problem formulation available at: https://arxiv.org/pdf/2303.16746
"""
function OptimalControlProblems.truck_trailer(::JuMPBackend; N::Int=steps_number_data(:truck_trailer))

    # parameters
    data=[0.4 0.1 0.2; 1.1 0.2 0.2; 0.8 0.1 0.2]
    L0 = data[1, 1]
    M0 = data[1, 2]
    W0 = data[1, 3]
    L1 = data[2, 1]
    M1 = data[2, 2]
    W1 = data[2, 3]
    L2 = data[3, 1]
    M2 = data[3, 2]
    W2 = data[3, 3]
    speedf = 1
    x2_t0 = 0
    y2_t0 = 0
    θ2_t0 = 0
    θ1_t0 = 0
    θ0_t0 = 0
    x2_tf = 0
    y2_tf = -2
    θ2_tf = π / 2
    θ1_tf = π / 2
    θ0_tf = π / 2

    # model
    model = JuMP.Model()

    # state, control, variable (final time) and initial guess
    @variables(
        model,
        begin

            # Final time
            1 <= tf <= 1000, (start = 10)

            # State variables
            x2[0:N], (start = 0.1)
            y2[0:N], (start = 0.1)
            -π / 2 <= θ0[0:N] <= π / 2, (start = 0.1)
            -π / 2 <= θ1[0:N] <= π / 2, (start = 0.1)
            θ2[0:N], (start = 0.1)
            -0.2 * speedf <= v0[0:N] <= 0.2 * speedf, (start = 0.1)
            -π / 6 <= δ0[0:N] <= π / 6, (start = 0.1)

            # Control variables
            -1 <= dv0[0:N] <= 1, (start = 0.1)
            -π / 10 <= dδ0[0:N] <= π / 10, (start = 0.1)
        end
    )

    # # positions
    # @expressions(
    #     model,
    #     begin
    #         x1[i=0:N], x2[i] + L2 * cos(θ2[i]) + M1 * cos(θ1[i])
    #         y1[i=0:N], y2[i] + L2 * sin(θ2[i]) + M1 * sin(θ1[i])
    #         x0[i=0:N], x1[i] + L1 * cos(θ1[i]) + M0 * cos(θ0[i])
    #         y0[i=0:N], y1[i] + L1 * sin(θ1[i]) + M0 * sin(θ0[i])
    #     end
    # )

    # intermediate variables
    @expressions(
        model,
        begin
            β01[i = 0:N], θ0[i] - θ1[i]
            β12[i = 0:N], θ1[i] - θ2[i]
            step, tf / N
        end
    )

    @constraints(
        model,
        begin
            β01_con[i = 0:N], -π / 2 <= β01[i] <= π / 2
            β12_con[i = 0:N], -π / 2 <= β12[i] <= π / 2
        end
    )

    # boundary conditions
    @constraints(
        model,
        begin

            # initial constraints
            x2[0] == x2_t0
            y2[0] == y2_t0
            θ0[0] == θ0_t0
            θ1[0] == θ1_t0
            θ2[0] == θ2_t0

            # final constraint
            x2[N] == x2_tf
            y2[N] == y2_tf
            θ2[N] == θ2_tf
            β01[N] == θ0_tf - θ1_tf
            β12[N] == θ1_tf - θ2_tf
        end
    )

    @expression(model, dθ0[i = 0:N], v0[i] / L0 * tan(δ0[i]))
    @expression(
        model, dθ1[i = 0:N], v0[i] / L1 * sin(β01[i]) - M0 / L1 * cos(β01[i]) * dθ0[i]
    )
    @expression(model, v1[i = 0:N], v0[i] * cos(β01[i]) + M0 * sin(β01[i]) * dθ0[i])
    @expression(
        model, dθ2[i = 0:N], v1[i] / L2 * sin(β12[i]) - M1 / L2 * cos(β12[i]) * dθ1[i]
    )
    @expression(model, v2[i = 0:N], v1[i] * cos(β12[i]) + M1 * sin(β12[i]) * dθ1[i])
    @expression(model, dx2[i = 0:N], v2[i] * cos(θ2[i]))
    @expression(model, dy2[i = 0:N], v2[i] * sin(θ2[i]))

    # Dynamics
    @constraints(
        model,
        begin
            ∂x2[i = 1:N], x2[i] == x2[i - 1] + 0.5 * step * (dx2[i] + dx2[i - 1])
            ∂y2[i = 1:N], y2[i] == y2[i - 1] + 0.5 * step * (dy2[i] + dy2[i - 1])
            ∂θ0[i = 1:N], θ0[i] == θ0[i - 1] + 0.5 * step * (dθ0[i] + dθ0[i - 1])
            ∂θ1[i = 1:N], θ1[i] == θ1[i - 1] + 0.5 * step * (dθ1[i] + dθ1[i - 1])
            ∂θ2[i = 1:N], θ2[i] == θ2[i - 1] + 0.5 * step * (dθ2[i] + dθ2[i - 1])
            ∂v0[i = 1:N], v0[i] == v0[i - 1] + 0.5 * step * (dv0[i] + dv0[i - 1])
            ∂δ0[i = 1:N], δ0[i] == δ0[i - 1] + 0.5 * step * (dδ0[i] + dδ0[i - 1])
        end
    )

    # objective
    @expression(model, dc[i = 0:N], β01[i]^2 + β12[i]^2)
    @objective(model, Min, tf + 0.5 * step * sum(dc[i] + dc[i - 1] for i in 1:N))

    return model
end
