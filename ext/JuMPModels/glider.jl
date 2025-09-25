"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Hang Glider Optimal Control Problem**.  
The objective is to compute the optimal trajectory of a hang glider that maximises the final horizontal position while accounting for aerodynamic forces and a thermal updraft.  
The system dynamics are discretised over `N` steps, and collocation constraints enforce the kinematic and dynamic equations of the glider.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `grid_size::Int=500`: (Keyword) Number of discretisation steps in the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model representing the hang glider optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.glider(JuMPBackend(); N=100)
```

# References

- Hang Glider Problem formulation as in: https://www.mcs.anl.gov/~more/cops/
"""
function OptimalControlProblems.glider(
    ::JuMPBackend, args...; grid_size::Int=grid_size_data(:glider), 
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...
)

    # parameters
    params = parameters_data(:glider, parameters)
    t0 = params[:t0]
    x_t0 = params[:x_t0]
    y_t0 = params[:y_t0]
    y_tf = params[:y_tf]
    vx_t0 = params[:vx_t0]
    vx_tf = params[:vx_tf]
    vy_t0 = params[:vy_t0]
    vy_tf = params[:vy_tf]
    u_c = params[:u_c]
    r_t0 = params[:r_t0]
    m = params[:m]
    g = params[:g]
    c0 = params[:c0]
    c1 = params[:c1]
    S = params[:S]
    ρ = params[:ρ]
    cL_min = params[:cL_min]
    cL_max = params[:cL_max]
    tf_l = params[:tf_l]
    x_l = params[:x_l]
    vx_l = params[:vx_l]

    # model
    model = JuMP.Model(args...; kwargs...)

    # metadata: required
    model[:time_grid] = () -> range(t0, value(model[:tf]), grid_size+1) # tf is a free
    model[:state_components] = ["x", "y", "vx", "vy"]
    model[:costate_components] = ["∂x", "∂y", "∂vx", "∂vy"]
    model[:control_components] = ["cL"]
    model[:variable_components] = ["tf"]

    # N = grid_size
    @expression(model, N, grid_size)

    # state, control, variable (final time) and initial guess
    @variables(
        model,
        begin
            tf ≥ tf_l,                         (start = 1)
            x[k = 0:N] ≥ x_l,                  (start = x_t0 + vx_t0 * k / N)
            y[k = 0:N],                         (start = y_t0 + (k / N) * (y_tf - y_t0))
            vx[k = 0:N] ≥ vx_l,                (start = vx_t0)
            vy[k = 0:N],                        (start = vy_t0)
            cL_min ≤ cL[k = 0:N] ≤ cL_max,    (start = cL_max / 2)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            x[0]  == x_t0
            y[0]  == y_t0
            vx[0] == vx_t0
            vy[0] == vy_t0
            y[N]  == y_tf
            vx[N] == vx_tf
            vy[N] == vy_tf
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            #
            Δt, (tf - t0) / N

            #
            r[k = 0:N], (x[k] / r_t0 - 2.5)^2
            u[k = 0:N], u_c * (1 - r[k]) * exp(-r[k])
            w[k = 0:N], vy[k] - u[k]
            v[k = 0:N], √(vx[k]^2 + w[k]^2)
            D[k = 0:N], 0.5 * (c0 + c1 * cL[k]^2) * ρ * S * v[k]^2
            L[k = 0:N], 0.5 * cL[k] * ρ * S * v[k]^2

            #
            dvx[k = 0:N], -(L[k] * w[k] + D[k] * vx[k]) / (m * v[k])
            dvy[k = 0:N], (L[k] * vx[k] - D[k] * w[k]) / (m * v[k]) - g
        end
    )

    @constraints(
        model,
        begin
            ∂x[k = 1:N], x[k] == x[k - 1] + 0.5 * Δt * (vx[k] + vx[k - 1])
            ∂y[k = 1:N], y[k] == y[k - 1] + 0.5 * Δt * (vy[k] + vy[k - 1])
            ∂vx[k = 1:N], vx[k] == vx[k - 1] + 0.5 * Δt * (dvx[k] + dvx[k - 1])
            ∂vy[k = 1:N], vy[k] == vy[k - 1] + 0.5 * Δt * (dvy[k] + dvy[k - 1])
        end
    )

    # objective
    @objective(model, Max, x[N])

    return model
end
