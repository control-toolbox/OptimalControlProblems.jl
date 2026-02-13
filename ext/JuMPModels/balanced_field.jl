"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Aircraft Balanced Field Length Calculation (Takeoff phase)**.  
The model represents the dynamics of the aircraft during takeoff and seeks to minimise the final range.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `grid_size::Int=200`: (Keyword) Number of discretisation steps for the time horizon.

# Returns

- `model::JuMP.Model`: A JuMP model representing the problem.
"""
function OptimalControlProblems.balanced_field(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:balanced_field),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:balanced_field, parameters)
    t0 = params[:t0]
    m = params[:m]
    g = params[:g]
    ρ = params[:ρ]
    S = params[:S]
    CD0 = params[:CD0]
    AR = params[:AR]
    e = params[:e]
    CL0 = params[:CL0]
    CL_max = params[:CL_max]
    h_w = params[:h_w]
    span = params[:span]
    α_max = params[:α_max]
    T = params[:T]
    
    r_t0 = params[:r_t0]
    v_t0 = params[:v_t0]
    h_t0 = params[:h_t0]
    γ_t0 = params[:γ_t0]
    h_tf = params[:h_tf]
    γ_tf = params[:γ_tf]

    α_min = params[:α_min]
    α_max_ctrl = params[:α_max_ctrl]

    # Constants
    b = span / 2.0
    K_nom = 1.0 / (pi * AR * e)

    # model
    model = JuMP.Model(args...; kwargs...)

    # metadata
    model[:time_grid] = () -> range(t0, value(model[:tf]), grid_size + 1)
    model[:state_components] = ["r", "v", "h", "γ"]
    model[:costate_components] = ["∂r", "∂v", "∂h", "∂γ"]
    model[:control_components] = ["α"]
    model[:variable_components] = ["tf"]

    @expression(model, N, grid_size)

    # variables
    @variables(
        model,
        begin
            r[0:N], (start = r_t0)
            v[0:N] ≥ 1.0, (start = v_t0 + 10.0)
            h[0:N] ≥ 0.0, (start = h_tf / 2.0)
            γ[0:N], (start = γ_tf)
            0 ≤ α[0:N] ≤ α_max_ctrl, (start = 0.1)
            tf ≥ 0.1, (start = 10.0)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            r[0] == r_t0
            v[0] == v_t0
            h[0] == h_t0
            γ[0] == γ_t0

            h[N] == h_tf
            γ[N] == γ_tf
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            Δt, (tf - t0) / N
            
            q[i = 0:N], 0.5 * ρ * v[i]^2
            CL[i = 0:N], CL0 + (α[i] / α_max) * (CL_max - CL0)
            L[i = 0:N], q[i] * S * CL[i]
            
            h_eff[i = 0:N], h[i] + h_w
            term_h[i = 0:N], 33.0 * abs(h_eff[i] / b)^1.5
            K[i = 0:N], K_nom * term_h[i] / (1.0 + term_h[i])
            D[i = 0:N], q[i] * S * (CD0 + K[i] * CL[i]^2)
            
            dr[i = 0:N], v[i] * cos(γ[i])
            dv[i = 0:N], (T * cos(α[i]) - D[i]) / m - g * sin(γ[i])
            dh[i = 0:N], v[i] * sin(γ[i])
            dγ[i = 0:N], (T * sin(α[i]) + L[i]) / (m * v[i]) - (g * cos(γ[i])) / v[i]
        end
    )

    @constraints(
        model,
        begin
            ∂r[i = 1:N], r[i] == r[i - 1] + 0.5 * Δt * (dr[i] + dr[i - 1])
            ∂v[i = 1:N], v[i] == v[i - 1] + 0.5 * Δt * (dv[i] + dv[i - 1])
            ∂h[i = 1:N], h[i] == h[i - 1] + 0.5 * Δt * (dh[i] + dh[i - 1])
            ∂γ[i = 1:N], γ[i] == γ[i - 1] + 0.5 * Δt * (dγ[i] + dγ[i - 1])
        end
    )

    # objective
    @objective(model, Min, r[N])

    return model
end
