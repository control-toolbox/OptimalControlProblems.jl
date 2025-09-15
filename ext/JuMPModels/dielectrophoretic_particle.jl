"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Dielectrophoretic Particle Optimal Control Problem**.  
The goal is to determine the time–optimal trajectory of a particle in a dielectrophoretic system, moving it from an initial point to a target point.  
The system dynamics are discretised over `N` steps, with the final time optimised.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `grid_size::Int=500`: (Keyword) Number of discretisation steps in the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model representing the dielectrophoretic particle optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.dielectrophoretic_particle(JuMPBackend(); N=200)
```

# References

- [CPR2006] Chang, D. E., Petit, N., & Rouchon, P. (2006). *Time-optimal control of a particle in a dielectrophoretic system*.  
  IEEE Transactions on Automatic Control, 51(7), 1100–1114.
"""
function OptimalControlProblems.dielectrophoretic_particle(
    ::JuMPBackend, args...; grid_size::Int=grid_size_data(:dielectrophoretic_particle), 
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...
)

    # parameters
    params = parameters_data(:dielectrophoretic_particle, parameters)
    t0 = params[:t0]
    x_t0 = params[:x_t0]
    y_t0 = params[:y_t0]
    x_tf = params[:x_tf]
    α = params[:α]
    c = params[:c]
    u_l = params[:u_l]
    u_u = params[:u_u]
    tf_l = params[:tf_l]
    
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

    # state, control and variable (final time)
    @variables(
        model,
        begin
            x[0:N],                 (start = 1)
            y[0:N],                 (start = 1)
            u_l ≤ u[0:N] ≤ u_u,     (start = 0.1)
            tf_l ≤ tf,              (start = 5)
        end
    )

    # Boundary constraints
    @constraints(
        model,
        begin
            x[0] == x_t0
            y[0] == y_t0
            x[N] == x_tf
        end
    )

    # Dynamics
    @expressions(
        model,
        begin
            Δt, (tf - t0) / N
            dx[k = 0:N], y[k] * u[k] + α * u[k]^2
            dy[k = 0:N], -c * y[k] + u[k]
        end
    )
    @constraints(
        model,
        begin
            ∂x[k = 1:N], x[k] == x[k - 1] + 0.5 * Δt * (dx[k] + dx[k - 1])
            ∂y[k = 1:N], y[k] == y[k - 1] + 0.5 * Δt * (dy[k] + dy[k - 1])
        end
    )

    # Objective
    @objective(model, Min, tf)

    return model
end
