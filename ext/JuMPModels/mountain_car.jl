"""The **Mountain Car problem** is a classic benchmark in control and reinforcement learning.

Builds the `mountain_car` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> mountain_car(JuMPBackend(); grid_size=100)
```

Default parameters and the modeling status are available through
`problem_metadata(:mountain_car)` and `OptimalControlProblems.status(:mountain_car)`.
"""
function OptimalControlProblems.mountain_car(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:mountain_car),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:mountain_car, :JuMP)

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
            pos_min ≤ pos[0:N] ≤ pos_max, (start = pos_t0)
            vel_min ≤ vel[0:N] ≤ vel_max, (start = vel_t0)
            u_min ≤ u[0:N] ≤ u_max, (start = 0.0)
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
