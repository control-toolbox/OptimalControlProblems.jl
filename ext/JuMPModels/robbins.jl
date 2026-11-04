"""The **Robbins benchmark problem** is a classical optimal control problem introduced by Robbins (1980), involving a **third-order state constraint**:.

Builds the `robbins` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> robbins(JuMPBackend(); grid_size=100)
```

Default parameters and the modeling status are available through `metadata(:robbins)` and
`status(:robbins)`.
"""
function OptimalControlProblems.robbins(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:robbins),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:robbins, :JuMP)

    # parameters
    params = parameters_data(:robbins, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    α = params[:α]
    β = params[:β]
    γ = params[:γ]
    x₁_l = params[:x₁_l]
    x₁_t0 = params[:x₁_t0]
    x₂_t0 = params[:x₂_t0]
    x₃_t0 = params[:x₃_t0]
    x₁_tf = params[:x₁_tf]
    x₂_tf = params[:x₂_tf]
    x₃_tf = params[:x₃_tf]

    # model
    model = JuMP.Model(args...; kwargs...)

    # metadata: required
    model[:time_grid] = () -> range(t0, tf, grid_size + 1) # tf is a fixed
    model[:state_components] = ["x₁", "x₂", "x₃"]
    model[:costate_components] = ["∂x₁", "∂x₂", "∂x₃"]
    model[:control_components] = ["u"]
    model[:variable_components] = String[]

    # N = grid_size
    @expression(model, N, grid_size)

    # state, control and initial guess
    @variables(
        model,
        begin
            x₁[0:N] ≥ x₁_l, (start = 0.1)
            x₂[0:N], (start = 0.1)
            x₃[0:N], (start = 0.1)
            u[0:N], (start = 0.1)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            x₁[0] == x₁_t0
            x₂[0] == x₂_t0
            x₃[0] == x₃_t0
            x₁[N] == x₁_tf
            x₂[N] == x₂_tf
            x₃[N] == x₃_tf
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            Δt, (tf - t0) / N
        end
    )
    @constraints(
        model,
        begin
            ∂x₁[i = 1:N], x₁[i] == x₁[i - 1] + 0.5 * Δt * (x₂[i] + x₂[i - 1])
            ∂x₂[i = 1:N], x₂[i] == x₂[i - 1] + 0.5 * Δt * (x₃[i] + x₃[i - 1])
            ∂x₃[i = 1:N], x₃[i] == x₃[i - 1] + 0.5 * Δt * (u[i] + u[i - 1])
        end
    )

    # objective
    @expressions(
        model,
        begin
            dc[i = 0:N], (α * x₁[i] + β * x₁[i]^2 + γ * u[i]^2)
        end
    )

    @objective(model, Min, 0.5 * Δt * sum(dc[i] + dc[i - 1] for i in 1:N))

    return model
end
