"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Double Oscillator Optimal Control Problem**.  
The objective is to compute an optimal control trajectory for a mechanical double oscillator system, minimising a quadratic cost on positions and control.  
The system dynamics are discretised over `N` steps, with collocation constraints enforcing the dynamics.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `grid_size::Int=500`: (Keyword) Number of discretisation steps in the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model representing the double oscillator optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.double_oscillator(JuMPBackend(); N=200)
```

# References

- [CLP2018] Coudurier, C., Lepreux, O., & Petit, N. (2018). *Optimal bang-bang control of a mechanical double oscillator using averaging methods*.  
  IFAC-PapersOnLine, 51(2), 49–54.
"""
function OptimalControlProblems.double_oscillator(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:double_oscillator),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:double_oscillator, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    m1 = params[:m1]
    m2 = params[:m2]
    c = params[:c]
    k1 = params[:k1]
    k2 = params[:k2]
    u_l = params[:u_l]
    u_u = params[:u_u]
    x₁_t0 = params[:x₁_t0]
    x₂_t0 = params[:x₂_t0]

    # model
    model = JuMP.Model(args...; kwargs...)

    # metadata: required
    model[:time_grid] = () -> range(t0, tf, grid_size+1) # tf is a fixed
    model[:state_components] = ["x₁", "x₂", "x₃", "x₄"]
    model[:costate_components] = ["∂x₁", "∂x₂", "∂x₃", "∂x₄"]
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
            x₃[0:N], (start = 0.1)
            x₄[0:N], (start = 0.1)
            u_l ≤ u[0:N] ≤ u_u, (start = 0.1)
        end
    )

    # boundary conditions
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
            t[k = 0:N], t0 + k * (tf-t0) / N
            F[k = 0:N], sin((t[k] - t0) * 2π / (tf - t0))

            # dynamics
            dx₁[k = 0:N], x₃[k]
            dx₂[k = 0:N], x₄[k]
            dx₃[k = 0:N], -(k1 + k2) / m1 * x₁[k] + k2 / m1 * x₂[k] + 1 / m1 * F[k]
            dx₄[k = 0:N], k2 / m2 * x₁[k] - k2 / m2 * x₂[k] - c * (1 - u[k]) / m2 * x₄[k]

            # objective
            dc[k = 0:N], 0.5 * (x₁[k]^2 + x₂[k]^2 + u[k]^2)
        end
    )

    @constraints(
        model,
        begin
            ∂x₁[k = 1:N], x₁[k] == x₁[k - 1] + 0.5 * Δt * (dx₁[k] + dx₁[k - 1])
            ∂x₂[k = 1:N], x₂[k] == x₂[k - 1] + 0.5 * Δt * (dx₂[k] + dx₂[k - 1])
            ∂x₃[k = 1:N], x₃[k] == x₃[k - 1] + 0.5 * Δt * (dx₃[k] + dx₃[k - 1])
            ∂x₄[k = 1:N], x₄[k] == x₄[k - 1] + 0.5 * Δt * (dx₄[k] + dx₄[k - 1])
        end
    )

    # objective: trapeze rule
    @objective(model, Min, 0.5 * Δt * sum(dc[k] + dc[k - 1] for k in 1:N))

    return model
end
