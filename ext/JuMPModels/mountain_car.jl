"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Mountain Car Problem**.  
The model represents the dynamics of an underpowered car climbing a hill.  
The objective is to minimize the final time `tf` to reach the target position.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `grid_size::Int=100`: (Keyword) Number of discretisation steps for the time horizon.

# Returns

- `model::JuMP.Model`: A JuMP model representing the Mountain Car optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.mountain_car(JuMPBackend(); N=100)
```

# References

- Problem formulation: https://openmdao.github.io/dymos/examples/mountain_car/mountain_car.html
"""
function OptimalControlProblems.mountain_car(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:mountain_car),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:mountain_car, parameters)
    t0 = params[:t0]
    tf_start = params[:tf_start]
    tf_min = params[:tf_min]
    pos_t0 = params[:pos_t0]
    vel_t0 = params[:vel_t0]
    pos_tf = params[:pos_tf]
    vel_tf_min = params[:vel_tf_min]
    pos_min = params[:pos_min]
    pos_max = params[:pos_max]
    vel_min = params[:vel_min]
    vel_max = params[:vel_max]
    u_min = params[:u_min]
    u_max = params[:u_max]
    a = params[:a]
    b = params[:b]
    c = params[:c]

    # model
    model = JuMP.Model(args...; kwargs...)

    # metadata
    model[:time_grid] = () -> range(t0, value(model[:tf]), grid_size + 1)
    model[:state_components] = ["pos", "vel"]
    model[:costate_components] = ["∂pos", "∂vel"]
    model[:control_components] = ["u"]
    model[:variable_components] = ["tf"]

    # N = grid_size
    @expression(model, N, grid_size)

    # variables
    @variables(
        model,
        begin
            pos_min ≤ pos[i = 0:N] ≤ pos_max, (start = pos_t0 + (pos_tf - pos_t0) * i / N)
            vel_min ≤ vel[i = 0:N] ≤ vel_max, (start = 0.0)
            u_min ≤ u[i = 0:N] ≤ u_max, (start = 0.0)
            tf ≥ tf_min, (start = tf_start)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            pos[0] == pos_t0
            vel[0] == vel_t0
            pos[N] == pos_tf
            vel[N] ≥ vel_tf_min
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            Δt, (tf - t0) / N
            dpos[i = 0:N], vel[i]
            dvel[i = 0:N], a * u[i] - b * cos(c * pos[i])
        end
    )

    @constraints(
        model,
        begin
            ∂pos[i = 1:N], pos[i] == pos[i - 1] + 0.5 * Δt * (dpos[i] + dpos[i - 1])
            ∂vel[i = 1:N], vel[i] == vel[i - 1] + 0.5 * Δt * (dvel[i] + dvel[i - 1])
        end
    )

    # objective
    @objective(model, Min, tf)

    return model
end
