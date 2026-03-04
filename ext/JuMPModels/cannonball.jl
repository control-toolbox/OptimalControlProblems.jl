"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Multi-Phase Cannonball Problem**.  
The goal is to maximise the total range of a cannonball by optimizing its radius, initial velocity, and launch angle, subject to a maximum muzzle energy constraint.  

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
    ρ_metal = params[:ρ_metal]
    Cd = params[:Cd]
    KE_max = params[:KE_max]
    g = params[:g]
    ρ0 = params[:ρ0]
    hr = params[:hr]
    r_ball_l = params[:r_ball_l]
    r_ball_u = params[:r_ball_u]
    v0_l = params[:v0_l]
    v0_u = params[:v0_u]
    γ0_l = params[:γ0_l]
    γ0_u = params[:γ0_u]
    tf_l = params[:tf_l]
    tf_u = params[:tf_u]

    # model
    model = JuMP.Model(args...; kwargs...)

    # metadata
    model[:time_grid] = () -> range(t0, value(model[:tf]), grid_size+1)
    model[:state_components] = ["v", "γ", "h", "r"]
    model[:costate_components] = ["∂v", "∂γ", "∂h", "∂r"]
    model[:control_components] = String[]
    model[:variable_components] = ["v0", "γ0", "r_ball", "tf"]

    N = grid_size
    @expression(model, N_expr, N)

    @variables(model, begin
        tf_l <= tf <= tf_u, (start = 10.0)
        v0_l <= v0 <= v0_u, (start = 100.0)
        γ0_l <= γ0 <= γ0_u, (start = π/4)
        r_ball_l <= r_ball <= r_ball_u, (start = 0.05)

        v[0:N], (start = 100.0)
        γ[0:N], (start = π/4)
        h[0:N], (start = 1.0)
        r[0:N], (start = 1.0)
    end)

    # constraints
    @constraints(model, begin
        v[0] == v0
        γ[0] == γ0
        h[0] == 0
        r[0] == 0
        h[N] == 0
    end)

    # mass and KE constraint
    @expression(model, m, (4/3) * π * ρ_metal * r_ball^3)
    @constraint(model, 0.5 * m * v0^2 <= KE_max)

    @expressions(model, begin
        Δt, (tf - t0) / N
        S, π * r_ball^2
        
        # dynamics at each node
        ρ_at[i=0:N], ρ0 * exp(-h[i] / hr)
        D_at[i=0:N], 0.5 * ρ_at[i] * v[i]^2 * S * Cd
        
        dv[i=0:N], -D_at[i]/m - g * sin(γ[i])
        dγ[i=0:N], -g * cos(γ[i]) / v[i]
        dh[i=0:N], v[i] * sin(γ[i])
        dr[i=0:N], v[i] * cos(γ[i])
    end)

    @constraints(model, begin
        ∂v[i=1:N], v[i] == v[i-1] + 0.5 * Δt * (dv[i] + dv[i-1])
        ∂γ[i=1:N], γ[i] == γ[i-1] + 0.5 * Δt * (dγ[i] + dγ[i-1])
        ∂h[i=1:N], h[i] == h[i-1] + 0.5 * Δt * (dh[i] + dh[i-1])
        ∂r[i=1:N], r[i] == r[i-1] + 0.5 * Δt * (dr[i] + dr[i-1])
    end)

    @objective(model, Max, r[N])

    return model
end
