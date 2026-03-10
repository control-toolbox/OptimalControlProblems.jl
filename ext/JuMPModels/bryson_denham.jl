"""
$(TYPEDSIGNATURES)

Constructs a JuMP model representing the **Bryson-Denham optimal control problem**.  
The objective is to minimize the control effort while satisfying state constraints and double integrator dynamics.

# Arguments
- `::JuMPBackend`: Placeholder type to specify the JuMP backend or solver interface.
- `grid_size::Int=500`: (Keyword) Number of discretization steps for the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model containing the decision variables, dynamics constraints, boundary conditions, and the quadratic objective function.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.bryson_denham(JuMPBackend(); N=100)
```

# References

"""
function OptimalControlProblems.bryson_denham(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:bryson_denham),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    params = parameters_data(:bryson_denham, parameters)
    t0, tf = params[:t0], params[:tf]
    x1_t0, x2_t0 = params[:x1_t0], params[:x2_t0]
    x1_tf, x2_tf = params[:x1_tf], params[:x2_tf]
    x1_max = params[:x1_max]

    model = JuMP.Model(args...; kwargs...)

    # metadata: required
    model[:time_grid] = () -> range(t0, tf, grid_size+1)
    model[:state_components] = ["x1", "x2"]
    model[:control_components] = ["u"]
    model[:costate_components] = ["p1", "p2"]
    model[:variable_components] = []          

    N = grid_size
    Δt = (tf- t0) /N

    @variable(model, x1[0:N] <= x1_max, start = 0.0)
    @variable(model, x2[0:N], start = 0.0)
    @variable(model, u[0:N], start = 0.0)

    @constraints(model, begin
        x1[0] == x1_t0
        x2[0] == x2_t0
        x1[N] == x1_tf
        x2[N] == x2_tf
        p1[i = 1:N], x1[i] == x1[i-1] + 0.5* Δt * (x2[i] + x2[i-1])
        p2[i = 1:N], x2[i] == x2[i-1] + 0.5 * Δt * (u[i] + u[i-1])
    end)

    @objective(model, Min, 0.5 * Δt * sum(0.5 * (u[i]^2 + u[i-1]^2) for i in 1:N))

    return model
end