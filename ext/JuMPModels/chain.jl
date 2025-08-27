"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Hanging Chain Problem**.  
The goal is to determine the equilibrium shape of a chain of fixed length `L` hanging between two fixed points `a` and `b`, by minimising its potential energy.  
The formulation follows a standard optimal control approach with discretised dynamics and constraints.

# Arguments

- `::JuMPBackend`: Specifies the backend for building the JuMP model.
- `N::Int=500`: (Keyword) Number of discretisation steps in the time grid.

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
function OptimalControlProblems.chain(::JuMPBackend; N::Int=steps_number_data(:chain))

    # parameters
    L = 4
    a = 1
    b = 3
    tf = final_time_data(:chain)

    #
    tmin = b > a ? 1 / 4 : 3 / 4

    # model
    model = JuMP.Model()

    # time
    @expressions(
        model,
        begin
            t[k = 0:N], k * tf / N
        end
    )

    # variables and initial guess
    @variables(
        model,
        begin
            u[k = 0:N], (start = 4 * abs(b - a) * (t[k] / tf - tmin))
            x1[k = 0:N],
            (start = 4 * abs(b - a) * t[k] / tf * (0.5 * t[k] / tf - tmin) + a)
            x2[k = 0:N],
            (
                start =
                    (4 * abs(b - a) * t[k] / tf * (0.5 * t[k] / tf - tmin) + a) *
                    (4 * abs(b - a) * (t[k] / tf - tmin))
            )
            x3[k = 0:N], (start = 4 * abs(b - a) * (t[k] / tf - tmin))
        end
    )

    @constraints(
        model,
        begin
            x1[0] == a
            x2[0] == 0
            x3[0] == 0
            x1[N] == b
            x3[N] == L
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            step, tf / N
            dx1[k = 0:N], u[k]
            dx2[k = 0:N], x1[k] * √(1 + u[k]^2)
            dx3[k = 0:N], √(1 + u[k]^2)
        end
    )

    @constraints(
        model,
        begin
            ∂x1[k = 1:N], x1[k] == x1[k - 1] + 0.5 * step * (dx1[k] + dx1[k - 1])
            ∂x2[k = 1:N], x2[k] == x2[k - 1] + 0.5 * step * (dx2[k] + dx2[k - 1])
            ∂x3[k = 1:N], x3[k] == x3[k - 1] + 0.5 * step * (dx3[k] + dx3[k - 1])
        end
    )

    @objective(model, Min, x2[N])

    return model
end
