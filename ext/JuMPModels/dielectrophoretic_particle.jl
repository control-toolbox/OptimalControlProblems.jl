"""The **dielectrophoretic particle problem** is a classical time-optimal control benchmark for microfluidic particle manipulation.

Builds the `dielectrophoretic_particle` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> dielectrophoretic_particle(JuMPBackend(); grid_size=100)
```

Default parameters and the modeling status are available through
`problem_metadata(:dielectrophoretic_particle)` and `OptimalControlProblems.status(:dielectrophoretic_particle)`.
"""
function OptimalControlProblems.dielectrophoretic_particle(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:dielectrophoretic_particle),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:dielectrophoretic_particle, :JuMP)

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

    # metadata: required
    model[:time_grid] = () -> range(t0, value(model[:tf]), grid_size + 1) # tf is a free
    model[:state_components] = ["x", "y"]
    model[:costate_components] = ["∂x", "∂y"]
    model[:control_components] = ["u"]
    model[:variable_components] = ["tf"]

    # N = grid_size
    @expression(model, N, grid_size)

    # state, control and variable (final time)
    @variables(
        model,
        begin
            x[0:N], (start = 1)
            y[0:N], (start = 0.5)
            u_l ≤ u[0:N] ≤ u_u, (start = 0.5)
            tf ≥ tf_l, (start = 8)
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
