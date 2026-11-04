"""The **ducted fan problem** is a classical nonlinear benchmark in optimal control with multiple input and state constraints.

Builds the `ducted_fan` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> ducted_fan(JuMPBackend(); grid_size=100)
```

Default parameters and the modeling status are available through `metadata(:ducted_fan)` and
`status(:ducted_fan)`.
"""
function OptimalControlProblems.ducted_fan(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:ducted_fan),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    OptimalControlProblems._warn_if_draft(:ducted_fan, :JuMP)

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
    x₁_t0 = params[:x₁_t0]
    v₁_t0 = params[:v₁_t0]
    x₂_t0 = params[:x₂_t0]
    v₂_t0 = params[:v₂_t0]
    α_t0 = params[:α_t0]
    vα_t0 = params[:vα_t0]
    x₁_tf = params[:x₁_tf]
    v₁_tf = params[:v₁_tf]
    x₂_tf = params[:x₂_tf]
    v₂_tf = params[:v₂_tf]
    α_tf = params[:α_tf]
    vα_tf = params[:vα_tf]

    # model
    model = JuMP.Model(args...; kwargs...)

    # metadata: required
    model[:time_grid] = () -> range(t0, value(model[:tf]), grid_size + 1) # tf is a free
    model[:state_components] = ["x₁", "v₁", "x₂", "v₂", "α", "vα"]
    model[:costate_components] = ["∂x₁", "∂v₁", "∂x₂", "∂v₂", "∂α", "∂vα"]
    model[:control_components] = ["u₁", "u₂"]
    model[:variable_components] = ["tf"]

    # N = grid_size
    @expression(model, N, grid_size)

    # state, control, variable (final time) and initial guess
    @variables(
        model,
        begin
            x₁[0:N], (start = 0.1)
            v₁[0:N], (start = 0.1)
            x₂[0:N], (start = -0.1)
            v₂[0:N], (start = 0.1)
            α_l ≤ α[0:N] ≤ α_u, (start = 0.1)     # radian
            vα[0:N], (start = 0.1)
            u₁_l ≤ u₁[0:N] ≤ u₁_u, (start = 0.1)     # [N]
            u₂_l ≤ u₂[0:N] ≤ u₂_u, (start = 1)       # [N]
            tf ≥ tf_l, (start = 1.5)
        end
    )

    # Boundary constraints
    @constraints(
        model,
        begin
            # initial
            x₁[0] == x₁_t0
            v₁[0] == v₁_t0
            x₂[0] == x₂_t0
            v₂[0] == v₂_t0
            α[0] == α_t0
            vα[0] == vα_t0

            # final
            x₁[N] == x₁_tf
            v₁[N] == v₁_tf
            x₂[N] == x₂_tf
            v₂[N] == v₂_tf
            α[N] == α_tf
            vα[N] == vα_tf
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            #
            Δt, (tf - t0) / N

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
            ∂x₁[k = 1:N], x₁[k] == x₁[k - 1] + 0.5 * Δt * (dx₁[k] + dx₁[k - 1])
            ∂v₁[k = 1:N], v₁[k] == v₁[k - 1] + 0.5 * Δt * (dv₁[k] + dv₁[k - 1])
            ∂x₂[k = 1:N], x₂[k] == x₂[k - 1] + 0.5 * Δt * (dx₂[k] + dx₂[k - 1])
            ∂v₂[k = 1:N], v₂[k] == v₂[k - 1] + 0.5 * Δt * (dv₂[k] + dv₂[k - 1])
            ∂α[k = 1:N], α[k] == α[k - 1] + 0.5 * Δt * (dα[k] + dα[k - 1])
            ∂vα[k = 1:N], vα[k] == vα[k - 1] + 0.5 * Δt * (dvα[k] + dvα[k - 1])
        end
    )

    # objective
    @objective(
        model, Min, (1 / tf) * 0.5 * Δt * sum(dc[k] + dc[k - 1] for k in 1:N) + (μ * tf)
    )

    return model
end
