"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Robertson's Problem**, a classic stiff chemical kinetics problem.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `grid_size::Int=500`: (Keyword) Number of discretisation steps for the time horizon.

# Returns

- `model::JuMP.Model`: A JuMP model representing the Robertson's problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.robertson(JuMPBackend(); N=100)
```
"""
function OptimalControlProblems.robertson(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:robertson),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:robertson, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    k₁ = params[:k₁]
    k₂ = params[:k₂]
    k₃ = params[:k₃]
    x_t0 = params[:x_t0]
    y_t0 = params[:y_t0]
    z_t0 = params[:z_t0]

    # model
    model = JuMP.Model(args...; kwargs...)

    # metadata: required
    model[:time_grid] = () -> range(t0, tf, grid_size+1)
    model[:state_components] = ["x", "y", "z"]
    model[:costate_components] = ["∂x", "∂y", "∂z"]
    model[:control_components] = String[]
    model[:variable_components] = String[]

    # N = grid_size
    @expression(model, N, grid_size)

    # state and initial guess
    @variables(
        model,
        begin
            x[0:N], (start = 1)
            y[0:N], (start = 0)
            z[0:N], (start = 0)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            x[0] == x_t0
            y[0] == y_t0
            z[0] == z_t0
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            #
            Δt, (tf - t0) / N

            # dynamics
            dx[i = 0:N], -k₁ * x[i] + k₂ * y[i] * z[i]
            dy[i = 0:N],  k₁ * x[i] - k₂ * y[i] * z[i] - k₃ * y[i]^2
            dz[i = 0:N],  k₃ * y[i]^2

        end
    )

    @constraints(
        model,
        begin
            ∂x[i = 1:N], x[i] == x[i - 1] + 0.5 * Δt * (dx[i] + dx[i - 1])
            ∂y[i = 1:N], y[i] == y[i - 1] + 0.5 * Δt * (dy[i] + dy[i - 1])
            ∂z[i = 1:N], z[i] == z[i - 1] + 0.5 * Δt * (dz[i] + dz[i - 1])
        end
    )

    # objective
    @objective(model, Min, -z[N])

    return model
end
