"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **SSTO Earth Launch Problem**.  
The goal is to minimise the time required to reach a circular orbit at an altitude of 185 km.  

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `grid_size::Int=100`: (Keyword) Number of discretisation steps for the time horizon.

# Returns

- `model::JuMP.Model`: A JuMP model representing the SSTO Earth optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.ssto_earth(JuMPBackend(); N=100)
```
"""
function OptimalControlProblems.ssto_earth(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:ssto_earth),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:ssto_earth, parameters)
    t0 = params[:t0]
    g = params[:g]
    rho_ref = params[:rho_ref]
    h_scale = params[:h_scale]
    Cd = params[:Cd]
    S = params[:S]
    Thrust = params[:Thrust]
    Isp = params[:Isp]
    m0 = params[:m0]
    y_tf = params[:y_tf]
    vx_tf = params[:vx_tf]
    vy_tf = params[:vy_tf]
    tf_l = params[:tf_l]
    tf_u = params[:tf_u]
    theta_l = params[:theta_l]
    theta_u = params[:theta_u]

    # model
    model = JuMP.Model(args...; kwargs...)

    # metadata
    model[:time_grid] = () -> range(t0, value(model[:tf]), grid_size + 1)
    model[:state_components] = ["px", "py", "vx", "vy", "m"]
    model[:costate_components] = ["∂px", "∂py", "∂vx", "∂vy", "∂m"]
    model[:control_components] = ["theta"]
    model[:variable_components] = ["tf"]

    N = grid_size
    @expression(model, N_expr, N)

    # scaling factors
    s_p = 1e5
    s_v = 1e3
    s_m = 1e5

    @variables(model, begin
        tf_l <= tf <= tf_u, (start = 150.0)
        theta_l <= theta[0:N] <= theta_u, (start = 0.5)

        px[0:N], (start = 0.0)
        py[i=0:N], (start = i/N * y_tf / s_p)
        vx[i=0:N], (start = i/N * vx_tf / s_v)
        vy[0:N], (start = 0.0)
        m[0:N], (start = m0 / s_m)
    end)

    # boundary constraints (scaled)
    @constraints(model, begin
        px[0] == 0.0
        py[0] == 0.0
        vx[0] == 0.0
        vy[0] == 0.0
        m[0] == m0 / s_m
        
        py[N] == y_tf / s_p
        vx[N] == vx_tf / s_v
        vy[N] == vy_tf / s_v
    end)

    @expressions(model, begin
        Δt, (tf - t0) / N
        
        # Unscale for dynamics
        p_val[i=0:N], py[i] * s_p
        vx_val[i=0:N], vx[i] * s_v
        vy_val[i=0:N], vy[i] * s_v
        m_val[i=0:N], m[i] * s_m
        
        # dynamics at each node
        v_at[i=0:N], sqrt(vx_val[i]^2 + vy_val[i]^2 + 1e-6)
        rho_at[i=0:N], rho_ref * exp(-p_val[i] / h_scale)
        D_factor_at[i=0:N], 0.5 * rho_at[i] * v_at[i] * Cd * S
        
        dpx[i=0:N], vx_val[i] / s_p
        dpy[i=0:N], vy_val[i] / s_p
        dvx[i=0:N], ((Thrust * cos(theta[i]) - D_factor_at[i] * vx_val[i]) / m_val[i]) / s_v
        dvy[i=0:N], ((Thrust * sin(theta[i]) - D_factor_at[i] * vy_val[i]) / m_val[i] - g) / s_v
        dm[i=0:N], (-Thrust / (g * Isp)) / s_m
    end)

    @constraints(model, begin
        ∂px[i=1:N], px[i] == px[i-1] + 0.5 * Δt * (dpx[i] + dpx[i-1])
        ∂py[i=1:N], py[i] == py[i-1] + 0.5 * Δt * (dpy[i] + dpy[i-1])
        ∂vx[i=1:N], vx[i] == vx[i-1] + 0.5 * Δt * (dvx[i] + dvx[i-1])
        ∂vy[i=1:N], vy[i] == vy[i-1] + 0.5 * Δt * (dvy[i] + dvy[i-1])
        ∂m[i=1:N], m[i] == m[i-1] + 0.5 * Δt * (dm[i] + dm[i-1])
    end)

    @objective(model, Min, tf / 100.0)

    return model
end
