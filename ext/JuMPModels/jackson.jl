"""
$(TYPEDSIGNATURES)

Constructs and returns a JuMP model for the **Jackson Optimal Control Problem**.  
The model represents a dynamic system with three state variables `a`, `b`, and `x3` and a control variable `u`.  
The objective is to maximise the final value of `x3` by optimising the control `u` over the time horizon `[0, tf]`.  
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
function OptimalControlProblems.jackson(::JuMPBackend; N::Int=steps_number_data(:jackson))

    # parameters
    k1 = 1
    k2 = 10
    k3 = 1
    tf = 4

    # model
    model = JuMP.Model()

    @variables(
        model,
        begin
            0 <= a[0:N] <= 1.1, (start = 0.1)
            0 <= b[0:N] <= 1.1, (start = 0.1)
            0 <= x3[0:N] <= 1.1, (start = 0.1)
            0 <= u[0:N] <= 1, (start = 0.1)
        end
    )

    # boundary constraints
    @constraints(
        model,
        begin
            a[0] == 1
            b[0] == 0
            x3[0] == 0
        end
    )

    # dynamics
    @expressions(
        model,
        begin
            step, tf / N
            da[i = 0:N], -u[i] * (k1 * a[i] - k2 * b[i])
            db[i = 0:N], u[i] * (k1 * a[i] - k2 * b[i]) - (1 - u[i]) * k3 * b[i]
            dx3[i = 0:N], (1 - u[i]) * k3 * b[i]
        end
    )

    @constraints(
        model,
        begin
            ∂a[i = 1:N], a[i] == a[i - 1] + 0.5 * step * (da[i] + da[i - 1])
            ∂b[i = 1:N], b[i] == b[i - 1] + 0.5 * step * (db[i] + db[i - 1])
            ∂x3[i = 1:N], x3[i] == x3[i - 1] + 0.5 * step * (dx3[i] + dx3[i - 1])
        end
    )

    # objective
    @objective(model, Min, -x3[N])

    return model
end
