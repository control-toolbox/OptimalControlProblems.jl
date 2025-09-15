"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Hanging Chain Problem**.  
The goal is to determine the equilibrium shape of a chain of fixed length `L` hanging between two fixed points `a` and `b`, by minimising its potential energy.  
The formulation follows a standard optimal control approach with discretised dynamics and constraints.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `grid_size::Int=500`: (Keyword) Number of discretisation steps in the time grid.

# Returns

- `model::JuMP.Model`: A JuMP model representing the hanging chain optimal control problem.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> using JuMP

julia> model = OptimalControlProblems.chain(JuMPBackend(); N=300)
```

# References

- [COPS Benchmark Problems – Hanging Chain](https://www.mcs.anl.gov/~more/cops/)
"""
function OptimalControlProblems.chain(
    ::JuMPBackend, args...; grid_size::Int=grid_size_data(:chain), 
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...
)

    # parameters
    params = parameters_data(:chain, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    L = params[:L]
    a = params[:a]
    b = params[:b]
    x₁_t0 = a
    x₂_t0 = params[:x₂_t0]
    x₃_t0 = params[:x₃_t0]
    x₁_tf = b
    x₃_tf = L

    #
    tmin = b > a ? 1 / 4 : 3 / 4

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

    # time
    @expressions(
        model,
        begin
            t[k = 0:N], t0 + k * (tf-t0) / N
        end
    )

    # variables and initial guess
    @variables(
        model,
        begin
            u[k = 0:N],     (start = 4 * abs(b - a) * ((t[k] - t0) / (tf - t0) - tmin))
            x₁[k = 0:N],    (start = 4 * abs(b - a) * (t[k] - t0) / (tf - t0) * (0.5 * (t[k] - t0) / (tf - t0) - tmin) + a)
            x₂[k = 0:N],
            (
                start =
                    (4 * abs(b - a) * (t[k] - t0) / (tf - t0) * (0.5 * (t[k] - t0) / (tf - t0) - tmin) + a) *
                    (4 * abs(b - a) * ((t[k] - t0) / (tf - t0) - tmin))
            )
            x₃[k = 0:N],    (start = 4 * abs(b - a) * ((t[k] - t0) / (tf - t0) - tmin))
        end
    )

    @constraints(
        model,
        begin
            x₁[0] == x₁_t0
            x₂[0] == x₂_t0
            x₃[0] == x₃_t0
            x₁[N] == x₁_tf
            x₃[N] == x₃_tf
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            Δt, (tf - t0) / N
            dx₁[k = 0:N], u[k]
            dx₂[k = 0:N], x₁[k] * √(1 + u[k]^2)
            dx₃[k = 0:N], √(1 + u[k]^2)
        end
    )

    @constraints(
        model,
        begin
            ∂x₁[k = 1:N], x₁[k] == x₁[k - 1] + 0.5 * Δt * (dx₁[k] + dx₁[k - 1])
            ∂x₂[k = 1:N], x₂[k] == x₂[k - 1] + 0.5 * Δt * (dx₂[k] + dx₂[k - 1])
            ∂x₃[k = 1:N], x₃[k] == x₃[k - 1] + 0.5 * Δt * (dx₃[k] + dx₃[k - 1])
        end
    )

    @objective(model, Min, x₂[N])

    return model
end
