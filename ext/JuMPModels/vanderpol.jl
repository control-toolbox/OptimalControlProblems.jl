"""The **Van der Pol oscillator control problem** is a benchmark in nonlinear optimal control.

Builds the `vanderpol` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> vanderpol(JuMPBackend(); grid_size=100)
```

Default parameters and the modeling status are available through
`problem_metadata(:vanderpol)` and `OptimalControlProblems.status(:vanderpol)`.
"""
function OptimalControlProblems.vanderpol(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:vanderpol),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:vanderpol, :JuMP)

    # parameters
    params = parameters_data(:vanderpol, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    ω = params[:ω]
    ε = params[:ε]
    x₁_t0 = params[:x₁_t0]
    x₂_t0 = params[:x₂_t0]

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

    # state, control and initial guess
    @variables(
        model,
        begin
            x₁[0:N], (start = 0.1)
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
            dx₂[i = 0:N], ε * ω * (1 - x₁[i]^2) * x₂[i] - ω^2 * x₁[i] + u[i]

            # objective
            dc[i = 0:N], 0.5 * (x₁[i]^2 + x₂[i]^2 + u[i]^2)
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
