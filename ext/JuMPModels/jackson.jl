"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Jackson Optimal Control Problem**.  
The model represents a dynamic system with three state variables `a`, `b`, and `x₃` and a control variable `u`.  
The objective is to maximise the final value of `x₃` by optimising the control `u` over the time horizon `[0, tf]`.  
The dynamics are discretised using `N` steps with trapezoidal collocation.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `N::Int=500`: (Keyword) Number of discretisation steps in the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model representing the Jackson optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.jackson(JuMPBackend(); N=100)
```

# References

- Problem formulation available at: https://github.com/control-toolbox/bocop/tree/main/bocop
"""
function OptimalControlProblems.jackson(
    ::JuMPBackend, args...; grid_size::Int=grid_size_data(:jackson), 
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...
)

    # parameters
    params = parameters_data(:jackson, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    k1 = params[:k1]
    k2 = params[:k2]
    k3 = params[:k3]
    a_l = params[:a_l]
    a_u = params[:a_u]
    b_l = params[:b_l]
    b_u = params[:b_u]
    x₃_l = params[:x₃_l]
    x₃_u = params[:x₃_u]
    u_l = params[:u_l]
    u_u = params[:u_u]
    a_t0 = params[:a_t0]
    b_t0 = params[:b_t0]
    x₃_t0 = params[:x₃_t0]

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

    @variables(
        model,
        begin
            a_l ≤ a[0:N] ≤ a_u,       (start = 0.1)
            b_l ≤ b[0:N] ≤ b_u,       (start = 0.1)
            x₃_l ≤ x₃[0:N] ≤ x₃_u,    (start = 0.1)
            u_l ≤ u[0:N] ≤ u_u,       (start = 0.1)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            a[0] == a_t0
            b[0] == b_t0
            x₃[0] == x₃_t0
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            Δt, (tf - t0) / N
            da[i = 0:N], -u[i] * (k1 * a[i] - k2 * b[i])
            db[i = 0:N], u[i] * (k1 * a[i] - k2 * b[i]) - (1 - u[i]) * k3 * b[i]
            dx₃[i = 0:N], (1 - u[i]) * k3 * b[i]
        end
    )

    @constraints(
        model,
        begin
            ∂a[i = 1:N], a[i] == a[i - 1] + 0.5 * Δt * (da[i] + da[i - 1])
            ∂b[i = 1:N], b[i] == b[i - 1] + 0.5 * Δt * (db[i] + db[i - 1])
            ∂x₃[i = 1:N], x₃[i] == x₃[i - 1] + 0.5 * Δt * (dx₃[i] + dx₃[i - 1])
        end
    )

    # objective
    @objective(model, Max, x₃[N])

    return model
end
