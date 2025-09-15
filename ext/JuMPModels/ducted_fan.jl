"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Planar Ducted Fan Optimal Control Problem**.  
The objective is to determine the optimal control inputs for a planar ducted fan to move from a given initial state to a desired final state, minimising a combination of control effort and final time.  
The system is discretised over `N` steps, with collocation constraints enforcing the dynamics.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `N::Int=250`: (Keyword) Number of discretisation steps in the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model representing the planar ducted fan optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.ducted_fan(JuMPBackend(); N=100)
```

# References

- Graichen, K., & Petit, N. (2009). *Incorporating a class of constraints into the dynamics of optimal control problems*.  
  Optimal Control Applications and Methods, 30(6), 537–561. [GP2009]
"""
function OptimalControlProblems.ducted_fan(
    ::JuMPBackend, args...; grid_size::Int=steps_number_data(:ducted_fan), 
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...
)

    # parameters
    params = parameters_data(:ducted_fan, parameters)
    t0 = params[:t0]
    r = params[:r]
    J = params[:J]
    m = params[:m]
    mg = params[:mg]
    μ = params[:μ]
    α_l = params[:α_l]
    α_u = params[:α_u]
    u₁_l = params[:u₁_l]
    u₁_u = params[:u₁_u]
    u₂_l = params[:u₂_l]
    u₂_u = params[:u₂_u]
    tf_l = params[:tf_l]
    x_i = params[:x_i]
    x_f = params[:x_f]
    
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
            x₁[0:N],                    (start = 0.1)
            v₁[0:N],                    (start = 0.1)
            x₂[0:N],                    (start = -0.1)
            v₂[0:N],                    (start = 0.1)
            α_l <= α[0:N] <= α_u,       (start = 0.1)     # radian
            vα[0:N],                    (start = 0.1)
            u₁_l <= u₁[0:N] <= u₁_u,    (start = 0.1)     # [N]
            u₂_l <= u₂[0:N] <= u₂_u,    (start = 1)       # [N]
            tf >= tf_l,                 (start = 1.5)
        end
    )

    # Boundary constraints
    @constraints(
        model,
        begin
            # initial
            x₁[0] == x_i[1]
            v₁[0] == x_i[2]
            x₂[0] == x_i[3]
            v₂[0] == x_i[4]
            α[0]  == x_i[5]
            vα[0] == x_i[6]

            # final
            x₁[N] == x_f[1]
            v₁[N] == x_f[2]
            x₂[N] == x_f[3]
            v₂[N] == x_f[4]
            α[N]  == x_f[5]
            vα[N] == x_f[6]
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            #
            step, (tf - t0) / N

            # dynamics
            dx₁[k = 0:N], v₁[k]
            dv₁[k = 0:N], (u₁[k] * cos(α[k]) - u₂[k] * sin(α[k])) / m
            dx₂[k = 0:N], v₂[k]
            dv₂[k = 0:N], (-mg + u₁[k] * sin(α[k]) + u₂[k] * cos(α[k])) / m
            dα[k = 0:N], vα[k]
            dvα[k = 0:N], r * u₁[k] / J

            # objective
            dc[k = 0:N], 2 * u₁[k]^2 + u₂[k]^2
        end
    )

    @constraints(
        model,
        begin
            ∂x₁[k = 1:N], x₁[k] == x₁[k - 1] + 0.5 * step * (dx₁[k] + dx₁[k - 1])
            ∂v₁[k = 1:N], v₁[k] == v₁[k - 1] + 0.5 * step * (dv₁[k] + dv₁[k - 1])
            ∂x₂[k = 1:N], x₂[k] == x₂[k - 1] + 0.5 * step * (dx₂[k] + dx₂[k - 1])
            ∂v₂[k = 1:N], v₂[k] == v₂[k - 1] + 0.5 * step * (dv₂[k] + dv₂[k - 1])
            ∂α[k = 1:N],   α[k] ==  α[k - 1] + 0.5 * step * (dα[k]  + dα[k - 1])
            ∂vα[k = 1:N], vα[k] == vα[k - 1] + 0.5 * step * (dvα[k] + dvα[k - 1])
        end
    )

    # objective
    @objective(
        model, Min, (1 / tf) * 0.5 * step * sum(dc[k] + dc[k - 1] for k in 1:N) + (μ * tf)
    )

    return model
end
