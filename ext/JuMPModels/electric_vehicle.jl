"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Electric Vehicle Optimal Control Problem**.  
The objective is to compute the optimal control trajectory for an electric vehicle to travel a fixed distance while minimising a combination of energy consumption and control effort.  
The system dynamics are discretised over `N` steps, and collocation constraints are used to enforce the vehicle's kinematic and dynamic equations.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `N::Int=500`: (Keyword) Number of discretisation steps in the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model representing the electric vehicle optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.electric_vehicle(JuMPBackend(); N=100)
```

# References

- Petit, N., & Sciarretta, A. (2011). *Optimal drive of electric vehicles using an inversion-based trajectory generation approach.* IFAC Proceedings Volumes, 44(1), 14519–14526. [PS2011]
"""
function OptimalControlProblems.electric_vehicle(
    ::JuMPBackend, args...; grid_size::Int=grid_size_data(:electric_vehicle), 
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...
)

    # parameters
    params = parameters_data(:electric_vehicle, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    b1 = params[:b1]
    b2 = params[:b2]
    h0 = params[:h0]
    h1 = params[:h1]
    h2 = params[:h2]
    α0 = params[:α0]
    α1 = params[:α1]
    α2 = params[:α2]
    α3 = params[:α3]
    x_t0 = params[:x_t0]
    v_t0 = params[:v_t0]
    x_tf = params[:x_tf]
    v_tf = params[:v_tf]

    # model
    model = JuMP.Model(args...; kwargs...)

    # ------------------------------------------------
    # expressions to get grid time infos
    @expressions(
        model,
        begin
            t0, t0  # (required if the initial time is fixed)
            tf, tf  # (required if the final time is fixed)
            N, grid_size    # (required)
        end
    )
    # ------------------------------------------------

    # state, control and initial guess
    @variables(
        model,
        begin
            x[0:N],                    (start = 0.1)
            v[0:N],                    (start = 0.1)
            u[0:N],                    (start = 0.1)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            x[0] == x_t0
            v[0] == v_t0
            x[N] == x_tf
            v[N] == v_tf
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            #
            Δt, (tf - t0) / N
            road[k = 0:N], α0 + α1 * x[k] + α2 * x[k]^2 + α3 * x[k]^3

            # dynamics
            dx[k = 0:N], v[k]
            dv[k = 0:N], h1 * u[k] - h2 * v[k]^2 - h0 - road[k]

            # objective
            dc[k = 0:N], b1 * u[k] * v[k] + b2 * u[k]^2
        end
    )

    @constraints(
        model,
        begin
            ∂x[k = 1:N], x[k] == x[k - 1] + 0.5 * Δt * (dx[k - 1] + dx[k])
            ∂v[k = 1:N], v[k] == v[k - 1] + 0.5 * Δt * (dv[k - 1] + dv[k])
        end
    )

    # objective
    @objective(model, Min, 0.5 * Δt * sum(dc[k] + dc[k - 1] for k in 1:N))

    return model
end
