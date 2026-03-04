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

    ## Scalings
    scaling_p = 1e5
    scaling_v = 1e3
    scaling_m = 1e5

    # model
    model = JuMP.Model(args...; kwargs...)

    # metadata
    model[:time_grid] = () -> range(t0, value(model[:tf]), grid_size+1)
    model[:state_components] = ["spx", "spy", "svx", "svy", "sm"]
    model[:costate_components] = ["∂px", "∂py", "∂vx", "∂vy", "∂m"]
    model[:control_components] = ["θ"]
    model[:variable_components] = ["tf"]

    N = grid_size
    @expression(model, N_expr, N)

    @variables(model, begin
        tf_l <= tf <= tf_u, (start = 150.0)
        theta_l <= θ[0:N] <= theta_u, (start = 0.5)

        spx[0:N]
        spy[0:N]
        svx[0:N]
        svy[0:N]
        sm[0:N]
    end)

    # Initial guess: linear interpolation between boundary conditions
    function linear_interpolate(x_s, x_t, n)
        return [x_s + (i - 1) / (n - 1) * (x_t - x_s) for i in 1:n]
    end

    px_tf_guess = 5.0e5 # 500 km
    m_tf_guess = m0 - (Thrust / (g * Isp)) * 150.0 # rough mass loss estimate

    set_start_value.(spx, linear_interpolate(0.0, px_tf_guess / scaling_p, N + 1))
    set_start_value.(spy, linear_interpolate(0.0, y_tf / scaling_p, N + 1))
    set_start_value.(svx, linear_interpolate(0.0, vx_tf / scaling_v, N + 1))
    set_start_value.(svy, linear_interpolate(0.0, vy_tf / scaling_v, N + 1))
    set_start_value.(sm,  linear_interpolate(m0 / scaling_m, m_tf_guess / scaling_m, N + 1))

    # unscaled expressions
    @expression(model, px[i=0:N], spx[i] * scaling_p)
    @expression(model, py[i=0:N], spy[i] * scaling_p)
    @expression(model, vx[i=0:N], svx[i] * scaling_v)
    @expression(model, vy[i=0:N], svy[i] * scaling_v)
    @expression(model, m[i=0:N],  sm[i] * scaling_m)

    # constraints
    @constraints(model, begin
        px[0] == 0.0
        py[0] == 0.0
        vx[0] == 0.0
        vy[0] == 0.0
        m[0] == m0
        
        py[N] == y_tf
        vx[N] == vx_tf
        vy[N] == vy_tf
    end)

    @expressions(model, begin
        Δt, (tf - t0) / N
        
        # dynamics at each node
        v_at[i=0:N], sqrt(vx[i]^2 + vy[i]^2 + 1e-9)
        rho_at[i=0:N], rho_ref * exp(-py[i] / h_scale)
        
        dpx[i=0:N], vx[i]
        dpy[i=0:N], vy[i]
        dvx[i=0:N], (Thrust * cos(θ[i]) - 0.5 * rho_at[i] * v_at[i] * vx[i] * Cd * S) / m[i]
        dvy[i=0:N], (Thrust * sin(θ[i]) - 0.5 * rho_at[i] * v_at[i] * vy[i] * Cd * S) / m[i] - g
        dm[i=0:N], -Thrust / (g * Isp)
    end)

    @constraints(model, begin
        ∂px[i=1:N], px[i] == px[i-1] + 0.5 * Δt * (dpx[i] + dpx[i-1])
        ∂py[i=1:N], py[i] == py[i-1] + 0.5 * Δt * (dpy[i] + dpy[i-1])
        ∂vx[i=1:N], vx[i] == vx[i-1] + 0.5 * Δt * (dvx[i] + dvx[i-1])
        ∂vy[i=1:N], vy[i] == vy[i-1] + 0.5 * Δt * (dvy[i] + dvy[i-1])
        ∂m[i=1:N], m[i] == m[i-1] + 0.5 * Δt * (dm[i] + dm[i-1])
    end)

    @objective(model, Min, tf)

    return model
end
