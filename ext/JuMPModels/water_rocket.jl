"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Water Rocket Problem (Propelled Phase)**.  
The model represents the dynamics of a water rocket during the ejection phase.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `grid_size::Int=100`: (Keyword) Number of discretisation steps for the time horizon.

# Returns

- `model::JuMP.Model`: A JuMP model representing the Water Rocket optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.water_rocket(JuMPBackend(); N=100)
```
"""
function OptimalControlProblems.water_rocket(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:water_rocket),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:water_rocket, parameters)
    g = params[:g]
    rho_w = params[:rho_w]
    p_a = params[:p_a]
    k = params[:k]
    V_b = params[:V_b]
    A_out = params[:A_out]
    S = params[:S]
    Cd = params[:Cd]
    rho_a = params[:rho_a]
    m_empty = params[:m_empty]
    t0 = params[:t0]
    r_t0 = params[:r_t0]
    h_t0 = params[:h_t0]
    v_t0 = params[:v_t0]
    p_t0 = params[:p_t0]
    tf_start = params[:tf_start]
    Vw0_start = params[:Vw0_start]
    gamma0_start = params[:gamma0_start]

    # model
    model = JuMP.Model(args...; kwargs...)

    # metadata
    model[:time_grid] = () -> range(t0, value(model[:tf]), grid_size + 1)
    model[:state_components] = ["r", "h", "v_mag", "gamma", "p", "Vw"]
    model[:costate_components] = ["∂r", "∂h", "∂v_mag", "∂gamma", "∂p", "∂Vw"]
    model[:control_components] = ["u"]
    model[:variable_components] = ["tf", "Vw0", "gamma0"]

    # N = grid_size
    @expression(model, N, grid_size)

    # variables
    @variables(
        model,
        begin
            r[0:N], (start = 0.0)
            h[0:N] ≥ 0.0, (start = 0.0)
            v_mag[0:N], (start = 1.0)
            gamma[0:N], (start = 0.785)
            p[0:N], (start = 7.0e5)
            Vw[0:N], (start = 1.0e-3)
            u[0:N], (start = 0.0)
            0.001 ≤ tf ≤ 1.0, (start = tf_start)
            0.1e-3 ≤ Vw0 ≤ 1.9e-3, (start = Vw0_start)
            0.1 ≤ gamma0 ≤ 1.5, (start = gamma0_start)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            r[0] == r_t0
            h[0] == h_t0
            v_mag[0] == v_t0
            gamma[0] == gamma0
            p[0] == p_t0
            Vw[0] == Vw0
            Vw[N] == 0.0
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            Δt, (tf - t0) / N
            dr[i = 0:N], v_mag[i] * cos(gamma[i])
            dh[i = 0:N], v_mag[i] * sin(gamma[i])
            dVw[i = 0:N], -sqrt(2 * (p[i] - p_a) / rho_w) * A_out
            dp[i = 0:N], k * p[i] * (-sqrt(2 * (p[i] - p_a) / rho_w) * A_out) / (V_b - Vw[i])
            dv_mag[i = 0:N], (2 * A_out * (p[i] - p_a) - 0.5 * rho_a * v_mag[i]^2 * Cd * S - (m_empty + rho_w * Vw[i]) * g * sin(gamma[i])) / (m_empty + rho_w * Vw[i])
            dgamma[i = 0:N], - (g * cos(gamma[i])) / v_mag[i]
        end
    )

    @constraints(
        model,
        begin
            ∂r[i = 1:N], r[i] == r[i - 1] + 0.5 * Δt * (dr[i] + dr[i - 1])
            ∂h[i = 1:N], h[i] == h[i - 1] + 0.5 * Δt * (dh[i] + dh[i - 1])
            ∂v_mag[i = 1:N], v_mag[i] == v_mag[i - 1] + 0.5 * Δt * (dv_mag[i] + dv_mag[i - 1])
            ∂gamma[i = 1:N], gamma[i] == gamma[i - 1] + 0.5 * Δt * (dgamma[i] + dgamma[i - 1])
            ∂p[i = 1:N], p[i] == p[i - 1] + 0.5 * Δt * (dp[i] + dp[i - 1])
            ∂Vw[i = 1:N], Vw[i] == Vw[i - 1] + 0.5 * Δt * (dVw[i] + dVw[i - 1])
        end
    )

    # objective
    @objective(model, Max, h[N])

    return model
end
