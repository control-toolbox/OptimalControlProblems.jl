"""
$(TYPEDSIGNATURES)

Constructs a JuMP model representing the **Beam optimal control problem**.  
The objective is to minimise the control effort while satisfying boundary conditions and the linear beam dynamics.  
The problem is formulated as in the BOCOP repository.

# Arguments

- `::JuMPBackend`: Placeholder type to specify the JuMP backend or solver interface.
- `grid_size::Int=500`: (Keyword) Number of discretisation steps for the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model containing the decision variables, dynamics constraints, boundary conditions, and the quadratic objective function.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.beam(JuMPBackend(); N=100)
```

# References

- Problem formulation available at: https://github.com/control-toolbox/bocop/tree/main/bocop
"""
function OptimalControlProblems.beam(
    ::JuMPBackend, args...; 
    grid_size::Int=grid_size_data(:beam), 
    parameters::Union{Nothing, NamedTuple}=nothing, 
    kwargs...
)

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

    # ------------------------------------------------
    # expressions to get grid time infos
    @expressions(
        model,
        begin
            t0, t0          # (required if the initial time is fixed)
            tf, tf          # (required if the final time is fixed)
            N, grid_size    # (required)
        end
    )
    # ------------------------------------------------

    # variables and initial guess
    @variables(
        model,
        begin
            x₁_l ≤ x₁[0:N] ≤ x₁_u,    (start = 0.05)
            x₂[0:N],                  (start = 0.1)
            u[0:N],                   (start = 0.1)
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
