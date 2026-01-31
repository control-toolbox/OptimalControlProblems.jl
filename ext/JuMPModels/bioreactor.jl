"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Bioreactor Optimal Control Problem**.  
The problem formulation follows the version provided in the [control-toolbox/bocop repository](https://github.com/control-toolbox/bocop/tree/main/bocop).

The model includes state variables for biomass (`y`), substrate (`s`), and bacteria (`b`),  
with control variable `u`, subject to nonlinear dynamics and constraints.  
The objective is to minimise a cost function derived from the system dynamics.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `grid_size::Int=500`: (Keyword) Number of discretisation steps in the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model representing the bioreactor optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.bioreactor(JuMPBackend(); N=100)
```

# References

- [control-toolbox/bocop](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.bioreactor(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:bioreactor),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:bioreactor, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    β = params[:β]
    c = params[:c]
    γ = params[:γ]
    halfperiod = params[:halfperiod]
    Ks = params[:Ks]
    μ2m = params[:μ2m]
    μbar = params[:μbar]
    r = params[:r]
    y_l = params[:y_l]
    s_l = params[:s_l]
    b_l = params[:b_l]
    u_l = params[:u_l]
    u_u = params[:u_u]
    y_t0_l = params[:y_t0_l]
    y_t0_u = params[:y_t0_u]
    s_t0_l = params[:s_t0_l]
    s_t0_u = params[:s_t0_u]
    b_t0_l = params[:b_t0_l]
    b_t0_u = params[:b_t0_u]

    # model
    model = JuMP.Model(args...; kwargs...)

    # metadata: required
    model[:time_grid] = () -> range(t0, tf, grid_size+1) # tf is a fixed
    model[:state_components] = ["y", "s", "b"]
    model[:costate_components] = ["∂y", "∂s", "∂b"]
    model[:control_components] = ["u"]
    model[:variable_components] = String[]

    # N = grid_size
    @expression(model, N, grid_size)

    # variables and initial guess
    @variables(
        model,
        begin
            y[0:N] ≥ 0, (start = 0.15)     
            s[0:N] ≥ 0, (start = 2.75)     
            b[0:N] ≥ 0.001, (start = 1.75) 
            0 ≤ u[0:N] ≤ 1, (start = 0.5)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            y[0] == 0.05  
            s[0] == 0.5   
            b[0] == 0.5   
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            #
            Δt, (tf-t0) / N

            # intermediate variables
            growth[k = 0:N], μ2m * s[k] / (s[k] + Ks)
            μ2[k = 0:N], growth[k]

            days[k = 0:N], (k * Δt) / (halfperiod * 2)
            tau[k = 0:N], (days[k] - floor(days[k])) * 2π
            light[k = 0:N], max(0, sin(tau[k]))^2
            μ[k = 0:N], light[k] * μbar

            # dynamics
            dy[k = 0:N], μ[k] * y[k] / (1 + y[k]) - (r + u[k]) * y[k]
            ds[k = 0:N], -μ2[k] * b[k] + u[k] * β * (γ * y[k] - s[k])
            db[k = 0:N], (μ2[k] - u[k] * β) * b[k]

            # objective
            dc[k = 0:N], μ2[k] * b[k] / (β + c)
        end
    )

    @constraints(
        model,
        begin
            ∂y[k = 1:N], y[k] == y[k - 1] + 0.5 * Δt * (dy[k] + dy[k - 1])
            ∂s[k = 1:N], s[k] == s[k - 1] + 0.5 * Δt * (ds[k] + ds[k - 1])
            ∂b[k = 1:N], b[k] == b[k - 1] + 0.5 * Δt * (db[k] + db[k - 1])
        end
    )

    # objective
    @objective(model, Min, -0.5 * Δt * sum(dc[k] + dc[k - 1] for k in 1:N))

    return model
end
