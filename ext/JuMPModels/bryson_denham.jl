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

    # Parameters (Standard: t0=0, tf=1, x1_t0=0, x2_t0=1, x1_tf=0, x2_tf=-1)
    params = parameters_data(:bryson_denham, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    x1_t0 = params[:x1_t0]
    x2_t0 = params[:x2_t0]
    x1_tf = params[:x1_tf]
    x2_tf = params[:x2_tf]
    x1_max = params[:x1_max] # Typically 1/9

    # Model initialization
    model = JuMP.Model(args...; kwargs...)

    # Metadata
    model[:time_grid] = () -> range(t0, tf, grid_size+1)
    model[:state_components] = ["x1", "x2"]
    model[:control_components] = ["u"]

    @expression(model, N, grid_size)

    # Variables and initial guess
    @variables(
        model,
        begin
            x1[0:N] <= x1_max, (start = 0.0)
            x2[0:N],           (start = 0.0)
            u[0:N],            (start = 0.0)
        end
    )

    # Boundary constraints
    @constraints(
        model,
        begin
            x1[0] == x1_t0
            x2[0] == x2_t0
            x1[N] == x1_tf
            x2[N] == x2_tf
        end
    )

    # Dynamics and Integration (Trapezoidal Method)
    @expressions(
        model,
        begin
            Δt, (tf - t0) / N
            dx1[i = 0:N], x2[i]
            dx2[i = 0:N], u[i]
            # Cost integrand: 0.5 * u^2
            dc[i = 0:N], 0.5 * u[i]^2
        end
    )

    @constraints(
        model,
        begin
            # System dynamics via Trapezoidal rule
            [i = 1:N], x1[i] == x1[i - 1] + 0.5 * Δt * (dx1[i] + dx1[i - 1])
            [i = 1:N], x2[i] == x2[i - 1] + 0.5 * Δt * (dx2[i] + dx2[i - 1])
        end
    )

    # Objective: Minimize control effort over [t0, tf]
    @objective(model, Min, 0.5 * Δt * sum(dc[i] + dc[i - 1] for i in 1:N))

    return model
end