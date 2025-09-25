"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Particle Steering Problem**.  
The model represents the dynamics of a particle with four state variables (`x₁`, `x₂`, `x₃`, `x₄`) and a control input `u`.  
The objective is to minimise the final time required for the particle to reach a specified altitude and terminal velocity while satisfying the system dynamics and boundary conditions.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `grid_size::Int=500`: (Keyword) Number of discretisation steps for the time horizon.

# Returns

- `model::JuMP.Model`: A JuMP model representing the particle steering optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.steering(JuMPBackend(); N=200)
```

# References

- Problem formulation available at: https://github.com/MadNLP/COPSBenchmark.jl/blob/main/src/steering.jl
"""
function OptimalControlProblems.steering(
    ::JuMPBackend, args...; grid_size::Int=grid_size_data(:steering), 
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...
)

    # parameters
    params = parameters_data(:steering, parameters)
    t0 = params[:t0]
    a = params[:a]
    u_min = params[:u_min]
    u_max = params[:u_max]
    tf_l = params[:tf_l]
    x₁_t0 = params[:x₁_t0]
    x₂_t0 = params[:x₂_t0]
    x₃_t0 = params[:x₃_t0]
    x₄_t0 = params[:x₄_t0]
    x₂_tf = params[:x₂_tf]
    x₃_tf = params[:x₃_tf]
    x₄_tf = params[:x₄_tf]

    #
    tf_start = 1
    function gen_x0(k, i)
        if i == 1 || i == 4
            return 0.0
        elseif i == 2
            return 5.0 * k * (tf_start-t0) / N
        elseif i == 3
            return 45.0 * k * (tf_start-t0) / N
        end
    end

    # model
    model = JuMP.Model(args...; kwargs...)

    # metadata: required
    model[:time_grid] = () -> range(t0, value(model[:tf]), grid_size+1) # tf is a free
    model[:state_components] = ["x₁", "x₂", "x₃", "x₄"]
    model[:costate_components] = ["∂x₁", "∂x₂", "∂x₃", "∂x₄"]
    model[:control_components] = ["u"]
    model[:variable_components] = ["tf"]

    # N = grid_size
    @expression(model, N, grid_size)

    # state, control and variable (final time)
    @variables(
        model,
        begin
            u_min ≤ u[i = 0:N] ≤ u_max,     (start = 0)
            x₁[i = 0:N],                    (start = gen_x0(i, 1))
            x₂[i = 0:N],                    (start = gen_x0(i, 2))
            x₃[i = 0:N],                    (start = gen_x0(i, 3))
            x₄[i = 0:N],                    (start = gen_x0(i, 4))
            tf ≥ tf_l,                      (start = tf_start)
        end
    )

    # boundary conditions
    @constraints(
        model,
        begin
            x₁[0] == x₁_t0
            x₂[0] == x₂_t0
            x₃[0] == x₃_t0
            x₄[0] == x₄_t0
            x₂[N] == x₂_tf
            x₃[N] == x₃_tf
            x₄[N] == x₄_tf
        end
    )

    # dynamics
    @expression(model, Δt, (tf - t0) / N) # Δt size
    @constraints(
        model,
        begin
            ∂x₁[i = 1:N], x₁[i] == x₁[i - 1] + 0.5 * Δt * (x₃[i - 1] + x₃[i])
            ∂x₂[i = 1:N], x₂[i] == x₂[i - 1] + 0.5 * Δt * (x₄[i - 1] + x₄[i])
            ∂x₃[i = 1:N], x₃[i] == x₃[i - 1] + 0.5 * Δt * (a * cos(u[i - 1]) + a * cos(u[i]))
            ∂x₄[i = 1:N], x₄[i] == x₄[i - 1] + 0.5 * Δt * (a * sin(u[i - 1]) + a * sin(u[i]))
        end
    )

    # objective
    @objective(model, Min, tf)

    return model
end
