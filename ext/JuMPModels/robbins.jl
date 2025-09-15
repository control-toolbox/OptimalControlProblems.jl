"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Robbins Optimal Control Problem**.  
The model represents a three-dimensional state system controlled by a single input `u`.  
The objective is to minimise a cost functional that combines linear and quadratic terms of the state and control over a fixed time horizon.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `N::Int=500`: (Keyword) Number of discretisation steps for the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model representing the Robbins optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.robbins(JuMPBackend(); N=100)
```

# References

- Problem formulation available at: https://github.com/control-toolbox/bocop/tree/main/bocop
"""
function OptimalControlProblems.robbins(
    ::JuMPBackend, args...; grid_size::Int=grid_size_data(:robbins), 
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...
)

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
