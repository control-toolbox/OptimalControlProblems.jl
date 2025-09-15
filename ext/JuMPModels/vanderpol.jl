"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Van der Pol Problem**, a classic nonlinear oscillator system.  
The model represents the dynamics of the Van der Pol oscillator with control input `u` and seeks to minimise the quadratic cost over the states and control.  

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `grid_size::Int=500`: (Keyword) Number of discretisation steps for the time horizon.

# Returns

- `model::JuMP.Model`: A JuMP model representing the Van der Pol optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.vanderpol(JuMPBackend(); N=100)
```

# References

- Problem formulation available at: https://github.com/control-toolbox/bocop/tree/main/bocop
"""
function OptimalControlProblems.vanderpol(
    ::JuMPBackend, args...; grid_size::Int=grid_size_data(:vanderpol), 
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...
)

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