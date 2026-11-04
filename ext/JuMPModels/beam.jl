"""The **clamped beam problem** is a classical benchmark in constrained optimal control.

Builds the `beam` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> beam(JuMPBackend(); grid_size=100)
```

Default parameters and the modeling status are available through `metadata(:beam)` and
`status(:beam)`.
"""
function OptimalControlProblems.beam(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:beam),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    OptimalControlProblems._warn_if_draft(:beam, :JuMP)

    # parameters
    params = parameters_data(:beam, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    x₁_l = params[:x₁_l]
    x₁_u = params[:x₁_u]
    x₁_t0 = params[:x₁_t0]
    x₂_t0 = params[:x₂_t0]
    x₁_tf = params[:x₁_tf]
    x₂_tf = params[:x₂_tf]

    # model
    model = JuMP.Model(args...; kwargs...)

    # metadata: required
    model[:time_grid] = () -> range(t0, tf, grid_size + 1) # tf is a fixed
    model[:state_components] = ["x₁", "x₂"]
    model[:costate_components] = ["∂x₁", "∂x₂"]
    model[:control_components] = ["u"]
    model[:variable_components] = String[]

    # N = grid_size
    @expression(model, N, grid_size)

    # variables and initial guess
    @variables(
        model,
        begin
            x₁_l ≤ x₁[0:N] ≤ x₁_u, (start = 0.05)
            x₂[0:N], (start = 0.1)
            u[0:N], (start = 0.1)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            x₁[0] == x₁_t0
            x₂[0] == x₂_t0
            x₁[N] == x₁_tf
            x₂[N] == x₂_tf
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            #
            Δt, (tf - t0) / N

            # dynamics
            dx₁[i = 0:N], x₂[i]
            dx₂[i = 0:N], u[i]

            # objective
            dc[i = 0:N], u[i]^2
        end
    )
    @constraints(
        model,
        begin
            ∂x₁[i = 1:N], x₁[i] == x₁[i - 1] + 0.5 * Δt * (dx₁[i] + dx₁[i - 1])
            ∂x₂[i = 1:N], x₂[i] == x₂[i - 1] + 0.5 * Δt * (dx₂[i] + dx₂[i - 1])
        end
    )

    # objective
    @objective(model, Min, 0.5 * Δt * sum(dc[i] + dc[i - 1] for i in 1:N))

    return model
end
