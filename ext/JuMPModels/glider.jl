"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Hang Glider Optimal Control Problem**.  
The objective is to compute the optimal trajectory of a hang glider that maximises the final horizontal position while accounting for aerodynamic forces and a thermal updraft.  
The system dynamics are discretised over `N` steps, and collocation constraints enforce the kinematic and dynamic equations of the glider.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `N::Int=500`: (Keyword) Number of discretisation steps in the time grid.

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
    ::JuMPBackend, args...; grid_size::Int=steps_number_data(:glider), 
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...
)

    # parameters
    params = parameters_data(:glider, parameters)
    t0 = params[:t0]
    x_i = params[:x_i]
    y_i = params[:y_i]
    y_f = params[:y_f]
    vx_i = params[:vx_i]
    vx_f = params[:vx_f]
    vy_i = params[:vy_i]
    vy_f = params[:vy_f]
    u_c = params[:u_c]
    r_i = params[:r_i]
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

    # ------------------------------------------------
    # expressions to get grid time infos
    @expressions(
        model,
        begin
            t0, t0  # (required if the initial time is fixed)
            N, grid_size    # (required)
        end
    )
    # ------------------------------------------------

    # state, control, variable (final time) and initial guess
    @variables(
        model,
        begin
            tf >= tf_l,                         (start = 1)
            x[k = 0:N] >= x_l,                  (start = x_i + vx_i * k / N)
            y[k = 0:N],                         (start = y_i + (k / N) * (y_f - y_i))
            vx[k = 0:N] >= vx_l,                (start = vx_i)
            vy[k = 0:N],                        (start = vy_i)
            cL_min <= cL[k = 0:N] <= cL_max,    (start = cL_max / 2)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            x[0]  == x_i
            y[0]  == y_i
            vx[0] == vx_i
            vy[0] == vy_i
            y[N]  == y_f
            vx[N] == vx_f
            vy[N] == vy_f
        end
    )

    # dynamics
    @expressions(
        model,
        begin

            #
            step, (tf - t0) / N

            #
            r[k = 0:N], (x[k] / r_i - 2.5)^2
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
            ∂x[k = 1:N], x[k] == x[k - 1] + 0.5 * step * (vx[k] + vx[k - 1])
            ∂y[k = 1:N], y[k] == y[k - 1] + 0.5 * step * (vy[k] + vy[k - 1])
            ∂vx[k = 1:N], vx[k] == vx[k - 1] + 0.5 * step * (dvx[k] + dvx[k - 1])
            ∂vy[k = 1:N], vy[k] == vy[k - 1] + 0.5 * step * (dvy[k] + dvy[k - 1])
        end
    )

    # objective
    @objective(model, Min, -x[N])

    return model
end
