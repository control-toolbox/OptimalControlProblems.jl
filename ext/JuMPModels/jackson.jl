"""The **Jackson problem** is a classical benchmark in optimal control.

Builds the `jackson` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> jackson(JuMPBackend(); grid_size=100)
```

Default parameters and the modeling status are available through `metadata(:jackson)` and
`status(:jackson)`.
"""
function OptimalControlProblems.jackson(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:jackson),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:jackson, :JuMP)

    # parameters
    params = parameters_data(:jackson, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    k1 = params[:k1]
    k2 = params[:k2]
    k3 = params[:k3]
    a_l = params[:a_l]
    a_u = params[:a_u]
    b_l = params[:b_l]
    b_u = params[:b_u]
    x₃_l = params[:x₃_l]
    x₃_u = params[:x₃_u]
    u_l = params[:u_l]
    u_u = params[:u_u]
    a_t0 = params[:a_t0]
    b_t0 = params[:b_t0]
    x₃_t0 = params[:x₃_t0]

    # model
    model = JuMP.Model(args...; kwargs...)

    # metadata: required
    model[:time_grid] = () -> range(t0, tf, grid_size + 1) # tf is a fixed
    model[:state_components] = ["a", "b", "x₃"]
    model[:costate_components] = ["∂a", "∂b", "∂x₃"]
    model[:control_components] = ["u"]
    model[:variable_components] = String[]

    # N = grid_size
    @expression(model, N, grid_size)

    @variables(
        model,
        begin
            a_l ≤ a[0:N] ≤ a_u, (start = 0.1)
            b_l ≤ b[0:N] ≤ b_u, (start = 0.1)
            x₃_l ≤ x₃[0:N] ≤ x₃_u, (start = 0.1)
            u_l ≤ u[0:N] ≤ u_u, (start = 0.1)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            a[0] == a_t0
            b[0] == b_t0
            x₃[0] == x₃_t0
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            Δt, (tf - t0) / N
            da[i = 0:N], -u[i] * (k1 * a[i] - k2 * b[i])
            db[i = 0:N], u[i] * (k1 * a[i] - k2 * b[i]) - (1 - u[i]) * k3 * b[i]
            dx₃[i = 0:N], (1 - u[i]) * k3 * b[i]
        end
    )

    @constraints(
        model,
        begin
            ∂a[i = 1:N], a[i] == a[i - 1] + 0.5 * Δt * (da[i] + da[i - 1])
            ∂b[i = 1:N], b[i] == b[i - 1] + 0.5 * Δt * (db[i] + db[i - 1])
            ∂x₃[i = 1:N], x₃[i] == x₃[i - 1] + 0.5 * Δt * (dx₃[i] + dx₃[i - 1])
        end
    )

    # objective
    @objective(model, Min, -x₃[N])

    return model
end
