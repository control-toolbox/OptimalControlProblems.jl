"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Multi-Phase Cannonball Problem**.  
The model represents the flight of a cannonball with a maximum muzzle energy constraint.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `grid_size::Int=100`: (Keyword) Number of discretisation steps for the time horizon.

# Returns

- `model::JuMP.Model`: A JuMP model representing the cannonball optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.cannonball(JuMPBackend(); N=100)
```
"""
function OptimalControlProblems.cannonball(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:cannonball),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:cannonball, parameters)
    t0 = params[:t0]
    rho_metal = params[:rho_metal]
    Cd = params[:Cd]
    KE_max = params[:KE_max]
    g = params[:g]
    rho0 = params[:rho0]
    hr = params[:hr]
    r_ball_l = params[:r_ball_l]
    r_ball_u = params[:r_ball_u]
    v0_l = params[:v0_l]
    v0_u = params[:v0_u]
    gamma0_l = params[:gamma0_l]
    gamma0_u = params[:gamma0_u]
    tf_l = params[:tf_l]
    tf_u = params[:tf_u]

    # model
    model = JuMP.Model(args...; kwargs...)

    # metadata
    model[:time_grid] = () -> range(t0, value(model[:tf]), grid_size + 1)
    model[:state_components] = ["v_mag", "gamma", "h", "r"]
    model[:costate_components] = ["∂v_mag", "∂gamma", "∂h", "∂r"]
    model[:control_components] = ["u"]
    model[:variable_components] = ["v0", "gamma0", "rball", "tf"]

    # N = grid_size
    @expression(model, N, grid_size)

    # variables
    @variables(
        model,
        begin
            v_mag[0:N] ≥ 0.0, (start = 100.0)
            gamma[0:N], (start = 0.785)
            h[0:N] ≥ 0.0, (start = 1.0)
            r[0:N], (start = 1.0)
            u[0:N], (start = 0.0)
            v0_l ≤ v0 ≤ v0_u, (start = 100.0)
            gamma0_l ≤ gamma0 ≤ gamma0_u, (start = 0.785)
            r_ball_l ≤ rball ≤ r_ball_u, (start = 0.05)
            tf_l ≤ tf ≤ tf_u, (start = 10.0)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            v_mag[0] == v0
            gamma[0] == gamma0
            h[0] == 0
            r[0] == 0
            h[N] == 0
        end
    )

    # design constraints
    @constraint(model, 0.5 * ((4/3) * π * rho_metal * rball^3) * v0^2 ≤ KE_max)

    # dynamics
    @expressions(
        model,
        begin
            Δt, (tf - t0) / N
            m_eff, (4/3) * π * rho_metal * rball^3
            S_eff, π * rball^2
            rho_val[i = 0:N], rho0 * exp(-h[i] / hr)
            D_eff[i = 0:N], 0.5 * rho_val[i] * v_mag[i]^2 * S_eff * Cd
            dv_mag[i = 0:N], -D_eff[i] / m_eff - g * sin(gamma[i])
            dgamma[i = 0:N], -g * cos(gamma[i]) / v_mag[i]
            dh[i = 0:N], v_mag[i] * sin(gamma[i])
            dr[i = 0:N], v_mag[i] * cos(gamma[i])
        end
    )

    @constraints(
        model,
        begin
            ∂v_mag[i = 1:N], v_mag[i] == v_mag[i - 1] + 0.5 * Δt * (dv_mag[i] + dv_mag[i - 1])
            ∂gamma[i = 1:N], gamma[i] == gamma[i - 1] + 0.5 * Δt * (dgamma[i] + dgamma[i - 1])
            ∂h[i = 1:N], h[i] == h[i - 1] + 0.5 * Δt * (dh[i] + dh[i - 1])
            ∂r[i = 1:N], r[i] == r[i - 1] + 0.5 * Δt * (dr[i] + dr[i - 1])
        end
    )

    # objective
    @objective(model, Max, r[N])

    return model
end
